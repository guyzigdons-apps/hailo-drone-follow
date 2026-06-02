import json
from dynamic_tiling.gt_review import ReviewCase
from dynamic_tiling.run_gt_verify import cases_to_doc, doc_to_cases


def test_cases_doc_roundtrip():
    cases = [ReviewCase(kind="merge", frame=868, track_ids=(168, 193),
                        boxes=[], reason="x", score=0.55)]
    doc = cases_to_doc(cases)
    back = doc_to_cases(json.loads(json.dumps(doc)))
    assert back[0].kind == "merge" and back[0].track_ids == (168, 193)
    assert back[0].frame == 868
