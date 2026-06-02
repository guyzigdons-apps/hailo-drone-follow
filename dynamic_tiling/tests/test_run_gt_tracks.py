import json
from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.run_gt_tracks import tracks_to_doc, doc_to_tracks


def test_tracks_doc_roundtrip():
    tracks = [GtTrack(cls=1, track_id=7, frames={0: (0.1, 0.2, 0.3, 0.4), 5: (0.11, 0.2, 0.3, 0.4)})]
    doc = tracks_to_doc(tracks, clip="x")
    s = json.dumps(doc)               # must be JSON-serializable
    back = doc_to_tracks(json.loads(s))
    assert back[0].cls == 1 and back[0].track_id == 7
    assert back[0].frames[0] == (0.1, 0.2, 0.3, 0.4)   # int keys restored
