import numpy as np
from scipy.stats import chi2
from scipy.optimize import linear_sum_assignment

class KalmanFilter:
    """
    A Kalman filter for tracking bounding boxes in image space.
    Constant Acceleration (CA) model with high process noise for reactive tracking.
    State vector: [x, y, a, h, vx, vy, va, vh, ax, ay]
    """
    def __init__(self):
        self._ndim = 4
        self._dt = 1.0
        
        # Motion matrix (F) - Constant Acceleration
        self._motion_mat = np.eye(2 * self._ndim + 2)
        for i in range(self._ndim):
            self._motion_mat[i, self._ndim + i] = self._dt
        self._motion_mat[0, 8] = 0.5 * self._dt**2
        self._motion_mat[1, 9] = 0.5 * self._dt**2
        self._motion_mat[4, 8] = self._dt
        self._motion_mat[5, 9] = self._dt

        # Project matrix (H)
        self._update_mat = np.zeros((self._ndim, 2 * self._ndim + 2))
        self._update_mat[:self._ndim, :self._ndim] = np.eye(self._ndim)

        # High process noise = less confident predictions = follows detection more
        self._std_weight_position = 1. / 5    # Was 1/20, now 4x higher uncertainty
        self._std_weight_velocity = 1. / 10   # Was 1/60, now 6x higher uncertainty
        self._std_weight_acceleration = 1. / 20  # Was 1/100, now 5x higher

    def initiate(self, measurement):
        mean_pos = measurement
        mean_vel = np.zeros(4)
        mean_acc = np.zeros(2)
        mean = np.r_[mean_pos, mean_vel, mean_acc]

        std = [
            2 * self._std_weight_position * measurement[3],
            2 * self._std_weight_position * measurement[3],
            1e-1,
            2 * self._std_weight_position * measurement[3],
            10 * self._std_weight_velocity * measurement[3],
            10 * self._std_weight_velocity * measurement[3],
            1e-2,
            10 * self._std_weight_velocity * measurement[3],
            10 * self._std_weight_acceleration * measurement[3],
            10 * self._std_weight_acceleration * measurement[3]
        ]
        covariance = np.diag(np.square(std))
        return mean, covariance

    def predict(self, mean, covariance):
        std_pos = [self._std_weight_position * mean[3]] * 2 + [1e-1] + [self._std_weight_position * mean[3]]
        std_vel = [self._std_weight_velocity * mean[3]] * 2 + [1e-2] + [self._std_weight_velocity * mean[3]]
        std_acc = [self._std_weight_acceleration * mean[3]] * 2
        
        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel, std_acc]))
        mean = np.dot(self._motion_mat, mean)
        covariance = np.linalg.multi_dot((self._motion_mat, covariance, self._motion_mat.T)) + motion_cov
        return mean, covariance

    def project(self, mean, covariance):
        std = [
            self._std_weight_position * mean[3],
            self._std_weight_position * mean[3],
            1e-1,
            self._std_weight_position * mean[3]
        ]
        innovation_cov = np.diag(np.square(std))
        mean = np.dot(self._update_mat, mean)
        covariance = np.linalg.multi_dot((self._update_mat, covariance, self._update_mat.T)) + innovation_cov
        return mean, covariance

    def gating_distance(self, mean, covariance, measurements, only_position=False):
        """Compute Mahalanobis distance between state and measurements."""
        mean, covariance = self.project(mean, covariance)
        if only_position:
            mean, covariance = mean[:2], covariance[:2, :2]
            measurements = measurements[:, :2]

        cholesky_factor = np.linalg.cholesky(covariance)
        d = measurements - mean
        z = np.linalg.solve(cholesky_factor, d.T)
        return np.sum(z**2, axis=0)

    def update(self, mean, covariance, measurement):
        projected_mean, projected_cov = self.project(mean, covariance)
        kalman_gain = np.linalg.solve(projected_cov, np.dot(covariance, self._update_mat.T).T).T
        innovation = measurement - projected_mean
        new_mean = mean + np.dot(innovation, kalman_gain.T)
        new_covariance = covariance - np.linalg.multi_dot((kalman_gain, projected_cov, kalman_gain.T))
        return new_mean, new_covariance

def cosine_distance_batch(embeddings_a, embeddings_b):
    """Compute cosine distance between two sets of embeddings.
    Returns cost matrix where 0 = identical, 2 = opposite."""
    if len(embeddings_a) == 0 or len(embeddings_b) == 0:
        return np.zeros((len(embeddings_a), len(embeddings_b)))
    a = np.array(embeddings_a)
    b = np.array(embeddings_b)
    # Normalize
    a_norm = a / (np.linalg.norm(a, axis=1, keepdims=True) + 1e-8)
    b_norm = b / (np.linalg.norm(b, axis=1, keepdims=True) + 1e-8)
    # Cosine distance = 1 - cosine_similarity
    return 1.0 - np.dot(a_norm, b_norm.T)


class STrack:
    def __init__(self, tlwh, score, embedding=None):
        self._tlwh = np.asarray(tlwh, dtype=float)
        self.mean, self.covariance = None, None
        self.is_activated = False
        self.score = score
        self.track_id = 0
        self.state = 1  # TrackState.New
        self.start_frame = 0
        self.frame_id = 0
        self.tracklet_len = 0
        self.input_index = -1
        self.smooth_embedding = None
        self._update_embedding(embedding)

    def activate(self, kalman_filter, frame_id):
        self.kalman_filter = kalman_filter
        self.track_id = self.next_id()
        self.mean, self.covariance = self.kalman_filter.initiate(self.tlwh_to_xyah(self._tlwh))
        self.state = 2  # TrackState.Tracked
        self.frame_id = frame_id
        self.start_frame = frame_id
        self.tracklet_len = 0
        self.is_activated = True

    def _update_embedding(self, embedding, alpha=0.9):
        if embedding is not None:
            embedding = np.asarray(embedding, dtype=np.float32)
            norm = np.linalg.norm(embedding)
            if norm > 0:
                embedding = embedding / norm
            if self.smooth_embedding is None:
                self.smooth_embedding = embedding
            else:
                self.smooth_embedding = alpha * self.smooth_embedding + (1 - alpha) * embedding
                self.smooth_embedding /= (np.linalg.norm(self.smooth_embedding) + 1e-8)

    def re_activate(self, new_track, frame_id, new_id=False):
        self.mean, self.covariance = self.kalman_filter.update(
            self.mean, self.covariance, self.tlwh_to_xyah(new_track.tlwh)
        )
        self.state = 2  # TrackState.Tracked
        self.is_activated = True
        self.frame_id = frame_id
        if new_id:
            self.track_id = self.next_id()
        self.score = new_track.score
        self.input_index = new_track.input_index
        self._update_embedding(new_track.smooth_embedding)

    def update(self, new_track, frame_id):
        self.frame_id = frame_id
        self.tracklet_len += 1
        new_tlwh = new_track.tlwh
        self.mean, self.covariance = self.kalman_filter.update(
            self.mean, self.covariance, self.tlwh_to_xyah(new_tlwh)
        )
        self.state = 2  # TrackState.Tracked
        self.is_activated = True
        self.score = new_track.score
        self.input_index = new_track.input_index
        self._update_embedding(new_track.smooth_embedding)

    @property
    def tlwh(self):
        if self.mean is None:
            return self._tlwh.copy()
        ret = self.mean[:4].copy()
        ret[2] *= ret[3]
        ret[:2] -= ret[2:] / 2
        return ret

    @property
    def tlbr(self):
        ret = self.tlwh.copy()
        ret[2:] += ret[:2]
        return ret

    @staticmethod
    def tlwh_to_xyah(tlwh):
        ret = np.asarray(tlwh).copy()
        ret[:2] += ret[2:] / 2
        ret[2] /= ret[3]
        return ret

    @staticmethod
    def tlbr_to_tlwh(tlbr):
        ret = np.asarray(tlbr).copy()
        ret[2:] -= ret[:2]
        return ret

    @staticmethod
    def next_id():
        if not hasattr(STrack, "_count"): STrack._count = 0
        STrack._count += 1
        return STrack._count

    @staticmethod
    def multi_predict(stracks, kalman_filter):
        if len(stracks) > 0:
            for st in stracks:
                st.mean, st.covariance = kalman_filter.predict(st.mean, st.covariance)

def iou_batch(bboxes1, bboxes2):
    """Computes IOU between two bboxes in the form [x1,y1,x2,y2]"""
    if len(bboxes1) == 0 or len(bboxes2) == 0:
        return np.zeros((len(bboxes1), len(bboxes2)))
        
    bboxes2 = np.expand_dims(bboxes2, 0)
    bboxes1 = np.expand_dims(bboxes1, 1)
    
    xx1 = np.maximum(bboxes1[..., 0], bboxes2[..., 0])
    yy1 = np.maximum(bboxes1[..., 1], bboxes2[..., 1])
    xx2 = np.minimum(bboxes1[..., 2], bboxes2[..., 2])
    yy2 = np.minimum(bboxes1[..., 3], bboxes2[..., 3])
    w = np.maximum(0., xx2 - xx1)
    h = np.maximum(0., yy2 - yy1)
    wh = w * h
    o = wh / ((bboxes1[..., 2] - bboxes1[..., 0]) * (bboxes1[..., 3] - bboxes1[..., 1]) +
        (bboxes2[..., 2] - bboxes2[..., 0]) * (bboxes2[..., 3] - bboxes2[..., 1]) - wh)
    return o

def center_distance_batch(bboxes1, bboxes2, img_diag=None):
    """Compute normalized Euclidean distance between bbox centers.
    bboxes in [x1, y1, x2, y2] format.
    Returns cost matrix where low values = close centers."""
    if len(bboxes1) == 0 or len(bboxes2) == 0:
        return np.zeros((len(bboxes1), len(bboxes2)))

    bboxes1 = np.asarray(bboxes1)
    bboxes2 = np.asarray(bboxes2)

    cx1 = (bboxes1[:, 0] + bboxes1[:, 2]) / 2
    cy1 = (bboxes1[:, 1] + bboxes1[:, 3]) / 2
    cx2 = (bboxes2[:, 0] + bboxes2[:, 2]) / 2
    cy2 = (bboxes2[:, 1] + bboxes2[:, 3]) / 2

    # Pairwise Euclidean distance
    dx = cx1[:, None] - cx2[None, :]
    dy = cy1[:, None] - cy2[None, :]
    dist = np.sqrt(dx**2 + dy**2)

    # Normalize by image diagonal if provided, otherwise by max bbox diagonal
    if img_diag is None:
        all_bboxes = np.vstack([bboxes1, bboxes2])
        w_max = all_bboxes[:, 2].max() - all_bboxes[:, 0].min()
        h_max = all_bboxes[:, 3].max() - all_bboxes[:, 1].min()
        img_diag = np.sqrt(w_max**2 + h_max**2)
        img_diag = max(img_diag, 1.0)  # avoid division by zero

    return dist / img_diag


def combined_cost_batch(bboxes1, bboxes2, alpha=0.6):
    """Combined cost using center distance and size similarity.
    cost = alpha * center_dist + (1-alpha) * (1 - size_similarity)
    """
    if len(bboxes1) == 0 or len(bboxes2) == 0:
        return np.zeros((len(bboxes1), len(bboxes2)))

    bboxes1 = np.asarray(bboxes1)
    bboxes2 = np.asarray(bboxes2)

    # Center distance (normalized)
    center_dist = center_distance_batch(bboxes1, bboxes2)

    # Size similarity: ratio of areas (smaller/larger), values in [0, 1]
    area1 = (bboxes1[:, 2] - bboxes1[:, 0]) * (bboxes1[:, 3] - bboxes1[:, 1])
    area2 = (bboxes2[:, 2] - bboxes2[:, 0]) * (bboxes2[:, 3] - bboxes2[:, 1])
    area1 = np.maximum(area1, 1.0)
    area2 = np.maximum(area2, 1.0)

    ratio = area1[:, None] / area2[None, :]
    size_sim = np.minimum(ratio, 1.0 / np.maximum(ratio, 1e-6))  # in [0, 1]

    return alpha * center_dist + (1 - alpha) * (1 - size_sim)


def linear_assignment(cost_matrix, thresh):
    if cost_matrix.size == 0:
        return np.empty((0, 2), dtype=int), tuple(range(cost_matrix.shape[0])), tuple(range(cost_matrix.shape[1]))
    x, y = linear_sum_assignment(cost_matrix)
    matches = np.asarray([[x[i], y[i]] for i in range(len(x))])
    
    unmatched_a = []
    for i in range(cost_matrix.shape[0]):
        if i not in matches[:, 0]:
            unmatched_a.append(i)
    unmatched_b = []
    for i in range(cost_matrix.shape[1]):
        if i not in matches[:, 1]:
            unmatched_b.append(i)
            
    matches_filt = []
    for m in matches:
        if cost_matrix[m[0], m[1]] > thresh:
            unmatched_a.append(m[0])
            unmatched_b.append(m[1])
        else:
            matches_filt.append(m)
    return np.asarray(matches_filt), tuple(unmatched_a), tuple(unmatched_b)

def joint_stracks(tlista, tlistb):
    exists = {}
    res = []
    for t in tlista:
        exists[t.track_id] = 1
        res.append(t)
    for t in tlistb:
        tid = t.track_id
        if not exists.get(tid, 0):
            exists[tid] = 1
            res.append(t)
    return res

def sub_stracks(tlista, tlistb):
    stracks = {}
    for t in tlista:
        stracks[t.track_id] = t
    for t in tlistb:
        tid = t.track_id
        if stracks.get(tid, 0):
            del stracks[tid]
    return list(stracks.values())

def remove_duplicate_stracks(stracksa, stracksb):
    pdist = iou_batch([t.tlbr for t in stracksa], [t.tlbr for t in stracksb])
    pairs = np.where(pdist < 0.15)
    dupa, dupb = set(), set()
    for a, b in zip(pairs[0], pairs[1]):
        timep = stracksa[a].frame_id - stracksa[a].start_frame
        timeq = stracksb[b].frame_id - stracksb[b].start_frame
        if timep > timeq:
            dupb.add(b)
        else:
            dupa.add(a)
    res_a = [t for i, t in enumerate(stracksa) if i not in dupa]
    res_b = [t for i, t in enumerate(stracksb) if i not in dupb]
    return res_a, res_b

def _embedding_cost(track_embs, det_embs):
    """Compute embedding cost matrix, handling None entries gracefully."""
    n, m = len(track_embs), len(det_embs)
    cost = np.ones((n, m), dtype=np.float32)  # default = max cost
    valid_t = [i for i, e in enumerate(track_embs) if e is not None]
    valid_d = [j for j, e in enumerate(det_embs) if e is not None]
    if valid_t and valid_d:
        a = np.array([track_embs[i] for i in valid_t])
        b = np.array([det_embs[j] for j in valid_d])
        sub = cosine_distance_batch(a, b)
        for ii, ti in enumerate(valid_t):
            for jj, dj in enumerate(valid_d):
                cost[ti, dj] = sub[ii, jj]
    return cost


class ByteTracker:
    def __init__(self, track_thresh=0.5, track_buffer=30, match_thresh=0.5, frame_rate=30,
                 gallery_thresh=0.35, gallery_max_size=100):
        self.tracked_stracks = []
        self.lost_stracks = []
        self.removed_stracks = []

        self.frame_id = 0
        self.det_thresh = track_thresh + 0.1
        self.track_buffer = track_buffer
        self.match_thresh = match_thresh  # Lower for turns: allows matching with lower IOU
        self.kalman_filter = KalmanFilter()

        # Long-term embedding gallery: track_id → embedding (numpy array)
        # Survives beyond track_buffer so a person who reappears gets the same ID
        self._embedding_gallery = {}  # {track_id: np.ndarray}
        self._gallery_thresh = gallery_thresh
        self._gallery_max_size = gallery_max_size

    def _gallery_lookup(self, embedding):
        """Check if an embedding matches any entry in the long-term gallery.

        Returns the matching track_id, or None if no match found.
        """
        if embedding is None or not self._embedding_gallery:
            return None
        # Collect gallery IDs that are NOT currently active (avoid ID collisions)
        active_ids = {t.track_id for t in self.tracked_stracks}
        active_ids |= {t.track_id for t in self.lost_stracks}
        candidates = {tid: emb for tid, emb in self._embedding_gallery.items()
                      if tid not in active_ids}
        if not candidates:
            return None
        tids = list(candidates.keys())
        gallery_embs = np.stack(list(candidates.values()))
        query = embedding / (np.linalg.norm(embedding) + 1e-8)
        scores = gallery_embs @ query  # cosine similarity (both are normalized)
        best_idx = np.argmax(scores)
        if scores[best_idx] > (1.0 - self._gallery_thresh):  # similarity > 1-thresh
            return tids[best_idx]
        return None

    def update(self, output_results, embeddings=None, frame=None):
        self.frame_id += 1
        activated_starcks = []
        refind_stracks = []
        lost_stracks = []
        removed_stracks = []

        for t in self.tracked_stracks + self.lost_stracks:
            t.input_index = -1

        if len(output_results):
            scores = output_results[:, 4]
            bboxes = output_results[:, :4]
        else:
            scores = np.array([])
            bboxes = np.array([])

        remain_inds = scores > self.det_thresh
        inds_low = scores > 0.1
        inds_high = scores < self.det_thresh
        inds_second = np.logical_and(inds_low, inds_high)
        
        dets_second = bboxes[inds_second]
        dets = bboxes[remain_inds]
        scores_keep = scores[remain_inds]
        scores_second = scores[inds_second]

        if len(dets) > 0:
            first_orig_indices = np.where(remain_inds)[0]
            detections = [
                STrack(STrack.tlbr_to_tlwh(tlbr), s,
                       embedding=embeddings[int(idx)] if embeddings is not None and embeddings[int(idx)] is not None else None)
                for (tlbr, s), idx in zip(zip(dets, scores_keep), first_orig_indices)
            ]
            for i, det in enumerate(detections):
                det.input_index = int(first_orig_indices[i])
        else:
            detections = []

        if len(dets_second) > 0:
            second_orig_indices = np.where(inds_second)[0]
            detections_second = [
                STrack(STrack.tlbr_to_tlwh(tlbr), s,
                       embedding=embeddings[int(idx)] if embeddings is not None and embeddings[int(idx)] is not None else None)
                for (tlbr, s), idx in zip(zip(dets_second, scores_second), second_orig_indices)
            ]
            for i, det in enumerate(detections_second):
                det.input_index = int(second_orig_indices[i])
        else:
            detections_second = []

        unconfirmed = []
        tracked_stracks = []
        for track in self.tracked_stracks:
            if not track.is_activated:
                unconfirmed.append(track)
            else:
                tracked_stracks.append(track)

        strack_pool = joint_stracks(tracked_stracks, self.lost_stracks)
        STrack.multi_predict(strack_pool, self.kalman_filter)

        # First association: Pure IOU (simple and robust for turns)
        dists = iou_batch([t.tlbr for t in strack_pool], [d.tlbr for d in detections])
        dists = 1.0 - dists
        matches, u_track, u_detection = linear_assignment(dists, thresh=self.match_thresh)

        for itracked, idet in matches:
            track = strack_pool[itracked]
            det = detections[idet]
            if track.state == 2:
                track.update(det, self.frame_id)
                activated_starcks.append(track)
                if track.smooth_embedding is not None:
                    self._embedding_gallery[track.track_id] = track.smooth_embedding.copy()
            else:
                track.re_activate(det, self.frame_id, new_id=False)
                refind_stracks.append(track)
                if track.smooth_embedding is not None:
                    self._embedding_gallery[track.track_id] = track.smooth_embedding.copy()

        # Center-distance fallback: catch tracks lost due to shape change (e.g. 90° turn)
        # Enhanced with embedding similarity when available
        u_track_tracked = [i for i in u_track if strack_pool[i].state == 2]
        if len(u_track_tracked) > 0 and len(u_detection) > 0:
            fallback_tracks = [strack_pool[i] for i in u_track_tracked]
            fallback_dets = [detections[i] for i in u_detection]
            cdist = combined_cost_batch(
                [t.tlbr for t in fallback_tracks],
                [d.tlbr for d in fallback_dets],
                alpha=0.6,
            )

            # Fuse with embedding distance if available
            track_embs = [t.smooth_embedding for t in fallback_tracks]
            det_embs = [d.smooth_embedding for d in fallback_dets]
            if any(e is not None for e in track_embs) and any(e is not None for e in det_embs):
                emb_cost = _embedding_cost(track_embs, det_embs)
                # Blend: 60% spatial, 40% appearance
                cdist = 0.6 * cdist + 0.4 * emb_cost

            matches_fb, u_track_fb, u_det_fb = linear_assignment(cdist, thresh=0.4)

            for itracked, idet in matches_fb:
                track = fallback_tracks[itracked]
                det = fallback_dets[idet]
                if track.state == 2:
                    track.update(det, self.frame_id)
                    activated_starcks.append(track)
                else:
                    track.re_activate(det, self.frame_id, new_id=False)
                    refind_stracks.append(track)
                    if track.smooth_embedding is not None:
                        self._embedding_gallery[track.track_id] = track.smooth_embedding.copy()

            # Update unmatched lists to exclude fallback-matched items
            u_track_tracked = [u_track_tracked[i] for i in u_track_fb]
            u_detection = tuple(u_detection[i] for i in u_det_fb)

        # Second association: unmatched tracked vs low-confidence detections
        r_tracked_stracks = [strack_pool[i] for i in u_track_tracked]
        r_lost_stracks = [strack_pool[i] for i in u_track if strack_pool[i].state == 3]

        dists = iou_batch([t.tlbr for t in r_tracked_stracks], [d.tlbr for d in detections_second])
        dists = 1.0 - dists
        matches, u_track_second, u_detection_second = linear_assignment(dists, thresh=0.4)

        for itracked, idet in matches:
            track = r_tracked_stracks[itracked]
            det = detections_second[idet]
            if track.state == 2:
                track.update(det, self.frame_id)
                activated_starcks.append(track)
            else:
                track.re_activate(det, self.frame_id, new_id=False)
                refind_stracks.append(track)
                if track.smooth_embedding is not None:
                    self._embedding_gallery[track.track_id] = track.smooth_embedding.copy()

        for it in u_track_second:
            track = r_tracked_stracks[it]
            if track.state != 3:
                track.state = 3
                lost_stracks.append(track)

        # Third association: lost tracks vs leftover low-conf detections
        detections_second_left = [detections_second[i] for i in u_detection_second]
        if len(r_lost_stracks) and len(detections_second_left):
            dists_lost = iou_batch([t.tlbr for t in r_lost_stracks], [d.tlbr for d in detections_second_left])
            dists_lost = 1.0 - dists_lost
            matches_lost, _, _ = linear_assignment(dists_lost, thresh=0.3)
            for itracked, idet in matches_lost:
                track = r_lost_stracks[itracked]
                det = detections_second_left[idet]
                track.re_activate(det, self.frame_id, new_id=False)
                refind_stracks.append(track)
                if track.smooth_embedding is not None:
                    self._embedding_gallery[track.track_id] = track.smooth_embedding.copy()

        # Embedding-based re-identification: lost tracks vs remaining high-conf detections
        # This helps recover identity after occlusion or brief disappearance
        remaining_dets = [detections[i] for i in u_detection]
        remaining_lost = [t for t in self.lost_stracks
                         if t.smooth_embedding is not None and t not in r_lost_stracks]  # exclude already-tried lost
        if remaining_lost and remaining_dets:
            lost_embs = [t.smooth_embedding for t in remaining_lost]
            rdet_embs = [d.smooth_embedding for d in remaining_dets]
            if any(e is not None for e in rdet_embs):
                emb_dists = _embedding_cost(lost_embs, rdet_embs)
                matches_reid, u_lost_reid, u_det_reid = linear_assignment(emb_dists, thresh=0.3)
                for itracked, idet in matches_reid:
                    track = remaining_lost[itracked]
                    det = remaining_dets[idet]
                    track.re_activate(det, self.frame_id, new_id=False)
                    refind_stracks.append(track)
                    if track.smooth_embedding is not None:
                        self._embedding_gallery[track.track_id] = track.smooth_embedding.copy()
                # Update remaining detections list
                remaining_dets = [remaining_dets[i] for i in u_det_reid]
        detections = remaining_dets
        dists = iou_batch([t.tlbr for t in unconfirmed], [d.tlbr for d in detections])
        dists = 1.0 - dists
        matches, u_unconfirmed, u_detection = linear_assignment(dists, thresh=0.7)

        for itracked, idet in matches:
            unconfirmed[itracked].update(detections[idet], self.frame_id)
            activated_starcks.append(unconfirmed[itracked])

        for it in u_unconfirmed:
            track = unconfirmed[it]
            track.state = 4
            removed_stracks.append(track)

        for inew in u_detection:
            track = detections[inew]
            if track.score < self.det_thresh:
                continue
            # Gallery lookup: check if this detection matches a previously-seen person
            gallery_match_id = self._gallery_lookup(track.smooth_embedding)
            track.activate(self.kalman_filter, self.frame_id)
            if gallery_match_id is not None:
                track.track_id = gallery_match_id  # restore original ID
            # Update gallery with latest embedding for this ID
            if track.smooth_embedding is not None:
                self._embedding_gallery[track.track_id] = track.smooth_embedding.copy()
            activated_starcks.append(track)

        for track in self.lost_stracks:
            if self.frame_id - track.frame_id > self.track_buffer:
                # Save embedding to gallery before removing — enables long-term re-id
                if track.smooth_embedding is not None:
                    self._embedding_gallery[track.track_id] = track.smooth_embedding.copy()
                    # Evict oldest entries if gallery exceeds max size
                    if len(self._embedding_gallery) > self._gallery_max_size:
                        oldest_id = next(iter(self._embedding_gallery))
                        del self._embedding_gallery[oldest_id]
                track.state = 4
                removed_stracks.append(track)

        self.tracked_stracks = [t for t in self.tracked_stracks if t.state == 2]
        self.tracked_stracks = joint_stracks(self.tracked_stracks, activated_starcks)
        self.tracked_stracks = joint_stracks(self.tracked_stracks, refind_stracks)
        self.lost_stracks = sub_stracks(self.lost_stracks, self.tracked_stracks)
        self.lost_stracks.extend(lost_stracks)
        self.lost_stracks = sub_stracks(self.lost_stracks, self.removed_stracks)
        self.removed_stracks.extend(removed_stracks)
        self.tracked_stracks, self.lost_stracks = remove_duplicate_stracks(self.tracked_stracks, self.lost_stracks)
        
        return [t for t in self.tracked_stracks if t.is_activated]
