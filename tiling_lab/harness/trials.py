"""Run one single-target follow trial per GT object, aggregate the metric suite."""
from __future__ import annotations

from dataclasses import dataclass

from hailo_tiling.classes import PERSON

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.dynamic.scheduler import TileScheduler
from .target_lock import TargetLock
from .replay import run
from .metrics import score_trial, TrialScore


@dataclass
class AggregateScore:
    n_trials: int
    mean_coverage: float
    mean_iou: float
    mean_drift_rate: float
    mean_loss_events: float
    mean_time_to_recover: float
    mean_recovery_success: float
    avg_tiles_per_frame: float
    per_trial: list[TrialScore]
    mean_frac_dets_embedded: float = 0.0
    # --- length-aware aggregates (added; legacy fields above unchanged) ---
    # frame-weighted coverage: sum(covered_frames) / sum(gt_frames) across trials.
    # Unlike mean_coverage (an unweighted per-track mean), this gives a 334-frame
    # track 334x the weight of a 16-frame fragment.
    coverage_fw: float = 0.0
    # event-weighted recovery: total recovered losses / total loss events. None
    # when there were zero loss events anywhere (no recovery to score).
    recovery_success_ew: float | None = None
    # total GT-present frames scored across all trials.
    n_gt_frames_total: int = 0


def run_all_trials(*, frames_factory, src_w, src_h, gt_tracks,
                   backend_factory, budget, fps, discovery_fps,
                   max_zoom=2.0, target_model_h=40.0,
                   discovery_grid=None, grid_overlap=0.0, iou_thr=0.5,
                   reacq_motion="frozen", reacq_radius_growth=0.0,
                   reid_assist_factory=None,
                   on_result=None) -> AggregateScore:
    """on_result: optional callback(track_id, RunResult) fired after each trial
    (e.g. to emit a replayable frames.json before the result is discarded).

    reid_assist_factory: optional zero-arg callable returning a FRESH ReidAssist
    (embedder + gallery + policy) per trial. The gallery MUST reset between trials,
    so a new assist is built inside the loop — never reuse one across trials."""
    discovery_period = max(1, int(round(fps / discovery_fps)))
    per_trial = []
    tiles_acc = 0.0
    trial_targets = [t for t in gt_tracks if t.cls == PERSON]
    for target in trial_targets:
        target_traj = target.frames
        distractors = [t.frames for t in gt_tracks if t is not target]
        _disc = {"discovery_grid": discovery_grid} if discovery_grid else {}
        scheduler = TileScheduler(src_w, src_h, discovery_period=discovery_period,
                                  max_zoom=max_zoom, target_model_h=target_model_h,
                                  grid_overlap=grid_overlap, **_disc)
        lock = TargetLock(frame_rate=int(fps), reacq_motion=reacq_motion,
                          reacq_radius_growth=reacq_radius_growth)  # frame_rate forwarded via **tracker_kwargs to create_tracker
        backend = backend_factory()
        meter = BudgetMeter(budget_inf_per_s=budget, fps=fps)
        # Fresh ReidAssist per trial: the gallery resets between trials.
        reid_assist = reid_assist_factory() if reid_assist_factory is not None else None
        try:
            res = run(frames_factory(), src_w, src_h, scheduler, lock, backend,
                      meter, target_traj, person_cls=PERSON, reid_assist=reid_assist)
        finally:
            backend.close()
        if on_result is not None:
            on_result(target.track_id, res)
        # only score frames we actually played
        gt_for_score = {f: b for f, b in target_traj.items() if f < res.n_frames}
        score = score_trial(gt_for_score, res.pred_traj,
                            distractors=distractors, iou_thr=iou_thr)
        if reid_assist is not None:
            st = reid_assist.stats
            score.reid_embeds = int(st.get("embeds", 0))
            score.reid_chip_embeds = int(st.get("chip_embeds", 0))
            score.person_dets_seen = int(st.get("person_dets_seen", 0))
            score.frac_dets_embedded = (score.reid_embeds / score.person_dets_seen
                                        if score.person_dets_seen else 0.0)
        per_trial.append(score)
        tiles_acc += res.avg_tiles_per_frame

    n = len(per_trial)
    def mean(attr):
        return sum(getattr(s, attr) for s in per_trial) / n if n else 0.0
    # length-aware aggregates: weight each trial by its GT-present frame count
    # (coverage) / its loss-event count (recovery), so long tracks dominate and
    # zero-loss tracks contribute nothing to recovery.
    gt_frames_total = sum(s.n_frames for s in per_trial)
    covered_frames_total = sum(s.coverage * s.n_frames for s in per_trial)
    loss_events_total = sum(s.loss_events for s in per_trial)
    recovered_total = sum(s.recovery_success_rate * s.loss_events for s in per_trial)
    return AggregateScore(
        n_trials=n,
        mean_coverage=mean("coverage"),
        mean_iou=mean("mean_iou"),
        mean_drift_rate=mean("drift_rate"),
        mean_loss_events=mean("loss_events"),
        mean_time_to_recover=mean("mean_time_to_recover"),
        mean_recovery_success=mean("recovery_success_rate"),
        avg_tiles_per_frame=(tiles_acc / n) if n else 0.0,
        mean_frac_dets_embedded=mean("frac_dets_embedded"),
        coverage_fw=(covered_frames_total / gt_frames_total) if gt_frames_total else 0.0,
        recovery_success_ew=(recovered_total / loss_events_total)
                            if loss_events_total else None,
        n_gt_frames_total=gt_frames_total,
        per_trial=per_trial,
    )
