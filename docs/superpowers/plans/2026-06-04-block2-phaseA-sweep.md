# Block 2 — Phase A Single-Target Tiling Sweep Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Coordinate-descent sweep of the tiling knobs on 0025 (3 FOVs) with the best recovery config (Block 1 + Block R) fixed, then a budget frontier at the best config — producing `PHASE_A.md`.

**Architecture:** A sweep driver (`run_sweep.py`) calls `run_all_trials` in-process per config, always through the SQLite caches; one JSON per run in `dynamic_tiling/runs/phase_a/` (untracked); a renderer aggregates to `dynamic_tiling/runs/PHASE_A.md` (committed). No new algorithm code.

**Tech Stack:** Python, existing `dynamic_tiling` harness. Spec: `docs/superpowers/specs/2026-06-04-weekend-recovery-reid-sweep-mot-design.md`.

**Prerequisites:** Block 1 + Block R complete; their best config (`reacq_motion`,
`reacq_radius_growth`, `reid_policy`) read from `BASELINE.md` / `REID_ABLATION.md`.

---

### Task 1: Sweep driver

**Files:**
- Create: `dynamic_tiling/run_sweep.py`
- Test: `dynamic_tiling/tests/test_run_sweep.py`

- [ ] **Step 1: Failing test** — the descent logic is pure; test it with a stubbed
runner:

```python
def test_coordinate_descent_walks_axes_and_keeps_best():
    from dynamic_tiling.run_sweep import coordinate_descent
    calls = []

    def fake_score(cfg):                      # higher = better
        calls.append(dict(cfg))
        return -abs(cfg["a"] - 3) - abs(cfg["b"] - 20)

    axes = {"a": [1, 2, 3], "b": [10, 20]}
    best, history = coordinate_descent({"a": 1, "b": 10}, axes, fake_score, passes=2)
    assert best == {"a": 3, "b": 20}
    assert all("score" in h for h in history)
    # pass 2 re-sweeps each axis around the updated best
    assert len(calls) >= len(axes["a"]) + len(axes["b"])
```

- [ ] **Step 2: Run, expect FAIL.**

- [ ] **Step 3: Implement** `coordinate_descent` in `run_sweep.py`:

```python
def coordinate_descent(base: dict, axes: dict, score_fn, passes: int = 2):
    """Sweep one axis at a time around `base`, keeping the best value before moving
    on. Returns (best_config, history). score_fn(cfg) -> float, higher is better.
    Configs are memoized so repeat evaluations are free."""
    best = dict(base)
    memo: dict = {}
    history = []

    def ev(cfg):
        key = tuple(sorted(cfg.items()))
        if key not in memo:
            memo[key] = score_fn(cfg)
            history.append({**cfg, "score": memo[key]})
        return memo[key]

    for _ in range(passes):
        for axis, values in axes.items():
            scored = []
            for v in values:
                cfg = {**best, axis: v}
                scored.append((ev(cfg), v))
            best[axis] = max(scored)[1]
    return best, history
```

CLI `main()`: axes per spec —

```python
AXES = {
    "discovery_grid": ["4x3", "6x4", "8x6", "12x9"],
    "discovery_fps": [1, 2, 4],
    "discovery_overlap": [0.0, 0.15, 0.25],
    "max_zoom": [1.5, 2.0, 3.0],
    "target_model_h": [30.0, 40.0, 60.0],
}
BASE = {"discovery_grid": "8x6", "discovery_fps": 2, "discovery_overlap": 0.25,
        "max_zoom": 2.0, "target_model_h": 40.0}
```

`score_fn(cfg)`: run `run_all_trials` for each of the 3 FOVs (budget 3000, recovery
config from CLI flags `--reacq-motion/--reacq-radius-growth/--reid-policy`, caches
`dynamic_tiling/runs/cache/0025_fov{F}__yolov8n4c_vga.sqlite3`), write each result JSON
to `dynamic_tiling/runs/phase_a/<cfg-slug>_fov<F>.json` (reuse `results_doc`), and
return `mean(coverage across fovs) - 0.005 * mean(tiles_per_frame)` (tiles tie-break).
`--passes 2` default; `--budget-frontier` flag runs budgets [300, 600, 1000, 1500, 3000]
at the final best config (3 fovs each) into `phase_a/frontier_b<B>_fov<F>.json`.

- [ ] **Step 4: Tests green; commit** — `feat(sweep): coordinate-descent Phase A sweep driver`

---

### Task 2: Execute the sweep (chip; long-running)

- [ ] **Step 1:** Run pass 1+2 (≈ (4+3+3+3+3) configs × dedupe × 3 fovs ≈ 35–50 trials-runs;
warm caches make repeats cheap):

```bash
source hailo-apps/venv_hailo_apps/bin/activate
python -m dynamic_tiling.run_sweep --passes 2 \
  --reacq-motion <best> --reacq-radius-growth <best> --reid-policy <best> \
  2>&1 | tee dynamic_tiling/runs/phase_a/sweep.log
```

Monitor: each config logs its per-fov score; abort+investigate any config that errors
(don't skip silently — record it in the log and PHASE_A.md).

- [ ] **Step 2:** Budget frontier at the winner: `python -m dynamic_tiling.run_sweep --budget-frontier --reacq-... <best>`.

- [ ] **Step 3:** Sanity: the best config must score ≥ the BASE config (it's in the
search space); frontier coverage must be monotone-ish in budget — flag anomalies
rather than hiding them.

---

### Task 3: PHASE_A.md

**Files:**
- Create: `dynamic_tiling/runs/PHASE_A.md` (committed)

- [ ] **Step 1:** Render from the per-run JSONs: (a) one table per axis per pass —
axis value vs mean coverage / mean IoU / tiles/frame / recovery success;
(b) the winning config + its full 3-fov metric rows; (c) the budget-frontier table +
the coverage-vs-tiles/frame pairs (the paper curve); (d) exact reproduction commands;
(e) deltas vs the Block-1 BASELINE numbers.

- [ ] **Step 2: Commit** — `docs(sweep): Phase A coordinate-descent results + budget frontier`
