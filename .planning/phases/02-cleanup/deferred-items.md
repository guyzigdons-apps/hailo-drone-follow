# Phase 2 — Deferred items

Pre-existing issues discovered during plan execution that are **out of scope** for
the current plan but worth tracking. Each entry includes the discovering plan, the
symptom, and the reason it's not being fixed now.

---

## DEFER-02-00-A — Pre-existing controller test failures

- **Discovered during:** Plan 02-00 (Wave 0 xfail scaffolds)
- **Symptom:** `python -m pytest robot_follow/tests/test_controller.py -v` reports
  `2 failed, 151 passed, 3 xfailed` on the clean tree:
  - `TestDistanceForward::test_center_y_is_ignored` — asserts
    `cmd_bot.forward_m_s == 0.0` but gets `-0.75` (controller is computing forward
    push from `cy=0.75`).
  - `TestDistanceForward::test_clamped_to_max_forward` — adjacent test in the same
    class, same root cause.
- **Why deferred:** Failure exists on `HEAD = 5f15982` *before* this plan's first
  edit (verified by running pytest on a stash-cleaned tree). Not caused by any
  Wave 0 work, not listed in CLEAN-01..18, and out of scope for Phase 2 (which is
  18 surgical cleanup edits with no controller-logic changes).
- **Suspected source:** Either the controller's frame-edge safety logic regressed
  in a recent merge, or the test's safety-margin expectations are stale. Worth a
  10-minute investigation before Phase 3 begins (Phase 3 touches the controller
  via the adapter boundary; this should be green before that).
- **Recommended action:** Open a follow-up plan (or fold into Phase 2 wave 1 as
  an extra CLEAN-19) once the immediate Phase-2 scope completes.
