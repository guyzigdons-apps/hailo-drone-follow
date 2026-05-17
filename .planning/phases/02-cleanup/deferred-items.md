# Phase 2 — Deferred items

Pre-existing issues discovered during plan execution that are **out of scope** for
the current plan but worth tracking. Each entry includes the discovering plan, the
symptom, and the reason it's not being fixed now.

---

## DEFER-02-00-A — Pre-existing controller test failures — RESOLVED 2026-05-17

- **Discovered during:** Plan 02-00 (Wave 0 xfail scaffolds)
- **Symptom (when filed):** `2 failed, 151 passed, 3 xfailed`:
  - `TestDistanceForward::test_center_y_is_ignored` — asserted
    `cmd_bot.forward_m_s == 0.0`, got `-0.75`.
  - `TestDistanceForward::test_clamped_to_max_forward` — asserted strict
    equality against `cfg.max_forward`, got `1.497`.
- **Status:** RESOLVED. Both were **test bugs, not controller bugs**. The
  controller's behavior was correct in both cases.
- **Root cause 1 (`test_center_y_is_ignored`):** the test's comment claimed
  `top_margin=bottom_margin=0.05 → safe cy ∈ [0.20, 0.80]`, but actual config
  defaults are `top_margin_safety=0.10, bottom_margin_safety=0.25`, giving
  safe zone `[0.225, 0.625]` for `bh=0.25`. Test's `cy=0.75` is firmly outside
  the actual safe zone, so the controller correctly retreats (-0.75 m/s).
  **Fix:** updated test to use `cy=0.30` and `cy=0.55` (both inside the actual
  safe zone). Comment refreshed to match current defaults.
- **Root cause 2 (`test_clamped_to_max_forward`):** with `_det(bh=0.001)` at
  default `cy=0.5`, `bbox_bottom = 0.5005` enters the bottom-margin fade zone
  `[1-2m, 1-m] = [0.5, 0.75]` (`bottom_margin=0.25`). The fade applies
  `factor = 1 - 0.0005/0.25 = 0.998`, shaving the clamped `1.5 m/s` down to
  `1.497`. Test is supposed to be about CLAMPING, not edge safety. **Fix:**
  set `top_margin_safety=0.0, bottom_margin_safety=0.0` in the test's
  `ControllerConfig` to isolate the clamp behavior.
- **Resolution:** test edits only; no controller source changed.
- **Verification:** `python -m pytest robot_follow/tests -x --ignore=robot_follow/tests/test_sim_worlds.py`
  → **176 passed, 0 failed, 0 xfailed**.
- **Phase 3 unblocked:** controller test suite is now fully green; ready for
  the adapter-boundary work in Phase 3 (ABS-01..11).
