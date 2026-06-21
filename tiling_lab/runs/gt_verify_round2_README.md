# Round-2 GT review runbook (2026-06-07)

All commands from the repo root. The flagged-case GUI writes
`review_decisions.json` into the outdir; after your verdicts the
finalize/corrections/lock steps follow `docs/gt-generation-guide.md`.
If a window doesn't appear over SSH: prefix with
`DISPLAY=:1 XAUTHORITY=/run/user/10615/gdm/Xauthority`.

Suggested order (quick wins → grind):

## 1. 0012 — ReID close clip · 0 flagged cases (eyeball + done)
4 tracks: 2 person full-span (the ReID pair), 2 vehicle. Just scrub the overlay:
```bash
.venv_gt/bin/python -m tiling_lab.viewer.overlay_viewer \
    --video /home/giladn/Videos/Drone/Training/DJI_20260430104732_0012_D_rotated.MP4 \
    --frames tiling_lab/runs/gt_verify_0012_default/overlay_by_id.frames.json:GT
```
Known: track 2 has 12 interior gap frames → `run_gt_interp` at corrections.

## 2. 0013 — ReID close clip · 2 cases
4 person (1+5 = full-clip pair; 4+6 early fragments — your call), 2 vehicle:
```bash
.venv_gt/bin/python -m tiling_lab.gt.gt_review_gui --outdir tiling_lab/runs/gt_verify_0013_default
```

## 3. 0019 — multi-car MOT · 16 cases (all keep_short)
47 tracks: 44 vehicle + 3 person (all 3 persons are in the queue):
```bash
.venv_gt/bin/python -m tiling_lab.gt.gt_review_gui --outdir tiling_lab/runs/gt_verify_0019_default
```

## 4. 0027 fov60/70 — pre-existing from the weekend · 5 + 3 cases
Commands in `tiling_lab/runs/gt_verify_0027_README_morning.md`.

## 5. 0029 fov50/60/70 — long-range highway clip · 17 / 21 / 28 cases
ALL-vehicle is CORRECT (frame-inspected: high-altitude landscape, highway at
right edge, no pedestrians). Mostly drop/merge verdicts on tiny car fragments:
```bash
.venv_gt/bin/python -m tiling_lab.gt.gt_review_gui --outdir tiling_lab/runs/gt_verify_0029_fov50
.venv_gt/bin/python -m tiling_lab.gt.gt_review_gui --outdir tiling_lab/runs/gt_verify_0029_fov60
.venv_gt/bin/python -m tiling_lab.gt.gt_review_gui --outdir tiling_lab/runs/gt_verify_0029_fov70
```

## 6. 0026 fov60/70 — harder yaw clip
fov70 READY · 9 cases (21 tracks: 14 person / 7 vehicle — VGA vehicle→"person"
mislabel caveat: sanity-check the "persons" against the video):
```bash
.venv_gt/bin/python -m tiling_lab.gt.gt_review_gui --outdir tiling_lab/runs/gt_verify_0026_fov70
```
fov60 READY · 2 cases (9 tracks: 6 person / 3 vehicle; raw dets are
vehicle-heavy here — 1769 veh vs 1228 person — so adjudicate classes carefully):
```bash
.venv_gt/bin/python -m tiling_lab.gt.gt_review_gui --outdir tiling_lab/runs/gt_verify_0026_fov60
```

## After verdicts (per clip-fov, per the guide)
finalize → corrections chain (interp/merge/drop as needed) → canonical remap →
lock. GT artifacts stay untracked until locked.
