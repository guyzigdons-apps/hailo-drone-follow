# Ground-Truth Generation Guide (dynamic-tiling experiment)

How to build a locked, human-verified ground-truth (GT) track set for a clip, at
each FOV, for the tile-scheduler experiment. This is the flow that produced the
locked **clip 0025** GT (`tiling_lab/runs/gt_verify_0025_fov{50,60,70}/`).

## Principles (read first)

- **Track person + vehicle only.** Never face or license-plate. Class ids are
  `person=1`, `vehicle=2` (see `hailo_tiling/classes.py`). The 4-class HEF
  emits an extra leading slot — the runners apply `class_offset` so decoded ids
  land on person=1/vehicle=2.
- **Canonical id scheme — same physical object, same id across every FOV.** For
  0025: `1 car-right, 2 car-left, 3 walker, 4 bg-person`; the widest FOV adds
  `5 car-bottom`. Align ids with `remap_track_ids` so cross-FOV comparison and
  reprojection line up.
- **Every edit is a reproducible correction, never a hand-edit.** Each fix is a
  pure function in `tiling_lab/gt/gt_edit.py` (unit-tested) driven by a
  `run_gt_*` CLI that records itself in `corrections.json`. The verified GT is
  regenerable from `gt_tracks.json` by replaying that chain.
- **Build the tightest FOV first (fov50).** It has the highest pixel resolution
  on each object and the fewest spurious peripheral tracks. Reproject its clean
  people into the wider FOVs (`crossfov_fill_track`) rather than re-cleaning
  them independently.
- **Lock when approved.** Write `GT_STATUS.json` (sha256 + chain), `chmod 444`
  the verified artifact, commit. A re-run can't silently overwrite a locked GT.

## Prerequisites

- `source setup_env.sh` (Hailo venv) for the dense detection pass.
- The boxmot tracker lives in the isolated `./.venv_gt` (protects the prod
  pipeline from boxmot's torch). `run_gt_verify`'s track step uses it.
- The clip prepared at each FOV via `python -m tiling_lab.video.prepare_video`
  (center-crop + lanczos scale to 4K; fov70 = full source, fov60/50 tighter
  center crops). Output: `<clip>_prepared__fov{50,60,70}.mp4`.

## Step 1 — Dense detections (per FOV)

A heavy 12×9 tiled dense-detection pass is the GT detection base (far denser
than any runtime tiling).

```bash
source setup_env.sh
tiling_benchmark/run_pxt_bench.py --only GT-12x9-25-multi --skip-analyze \
    --skip-existing --video <clip>_prepared__fov50.mp4 --out-dir <dense_dir_fov50>
# repeat for fov60, fov70
```

Produces `<dense_dir>/pxt_GT-12x9-25-multi.frames.json` (per-frame normalized
detections; schema uses `class_id` + `label`). This is cached — `--skip-existing`
avoids recompute.

## Step 2 — Build raw GT tracks + review queue (per FOV)

```bash
python -m tiling_lab.gt.run_gt_verify \
    --dense <dense_dir>/pxt_GT-12x9-25-multi.frames.json \
    --video <clip>_prepared__fov50.mp4 \
    --outdir tiling_lab/runs/gt_verify_<clip>_fov50
```

This dedups per-frame tiling fragments (IoMin NMS), tracks with BoT-SORT (CMC
on, ReID off), interpolates short gaps, filters by min length, auto-merges
near-certain duplicate tracks, and flags the gray zone. Writes:
`gt_tracks.json`, `overlay_by_id.frames.json`, `review_queue.json`, and zoomed
annotated PNGs under `review/`.

## Step 3 — Human review of the build

Look at the tracks and decide what's real. Two tools:

- **Flagged cases** (zoomed PNGs): `python -m tiling_lab.gt.gt_review_gui
  --outdir <dir>` — click Merge/Keep or Keep/Drop; see the general
  `human-image-review` skill.
- **Whole-clip visual check:** `python -m tiling_lab.viewer.overlay_viewer --video <mp4>
  --frames <dir>/overlay_by_id.frames.json:<label>` (shows every native track
  with its id; play it to spot splits, spurious tracks, drift, truncation).

Classify each track: which are the canonical objects, which are ID-splits to
merge, which are spurious to drop, which are peripheral objects only the wider
FOV sees (decide per experiment whether to keep — they break cross-FOV object
symmetry).

## Step 4 — Apply corrections (the reproducible toolkit)

Run the relevant `run_gt_*` CLIs in order; each reads/writes
`gt_tracks.verified.json` and appends to `corrections.json`. **Order matters**
(later steps build on earlier ones). The 0025 chains:

```bash
D=tiling_lab/runs/gt_verify_0025_fov60
python -m tiling_lab.gt.run_gt_drop          --outdir $D --track-ids 6
python -m tiling_lab.gt.run_gt_remap         --outdir $D --mapping 1:1,3:2,2:3,5:4
python -m tiling_lab.gt.run_gt_interp        --outdir $D --track-ids 1,2,3 --max-gap 25
python -m tiling_lab.gt.run_gt_despike       --outdir $D --track-ids 3 --min-ratio 0.8
python -m tiling_lab.gt.run_gt_crossfov_fill --outdir $D --track-id 4 \
    --src-dir tiling_lab/runs/gt_verify_0025_fov50 --src-track-id 4 --src-fov 50 --dst-fov 60
python -m tiling_lab.gt.run_gt_restore_width --outdir $D --track-ids 2 --anchor left
python -m tiling_lab.gt.run_gt_smooth        --outdir $D --track-ids 2 --dims x,w --window 21
```

### Corrections reference

| Correction (`run_gt_*`) | Use when | Key params |
|---|---|---|
| `drop` | a track is spurious / a false positive | `--track-ids` |
| `remap` | align ids to the canonical scheme | `--mapping old:new,…` |
| `interp` | detection-dropout gaps inside a track's span | `--track-ids --max-gap` |
| `despike` | height truncated (only top of object boxed; legs cut) | `--track-ids --min-ratio --window` |
| `restore_width` | width occluded on one side (object behind another) | `--track-ids --anchor --percentile --min-ratio` |
| `hold_tail` | object present at clip end but detector dropped it | `--track-ids [--until-frame]` |
| `drift_extend` | parked object present before it was detected (extend via a dense reference track's camera drift) | `--track-id --ref-track-id --frame-range` |
| `crossfov_fill` | a track is unstable/short but the same object is clean in another FOV | `--track-id --src-dir --src-track-id --src-fov --dst-fov` |
| `smooth` | per-frame detector jitter on a (near-)static object | `--track-ids --dims --window` |
| `pin` | object truly static *in image* (no camera drift) — **legacy** | `--track-ids --ref-frame [--frame-range]` |

### Decision heuristics learned on 0025

- **Static cars under a drifting camera: do NOT pin.** Pinning to one frame
  freezes the box; as the frame drifts (~13px over 0025) the box slides off the
  car. Use the *native* dense detections (they follow drift) + `interp` the
  small gaps. `pin` is only correct when the object is static in image space.
- **Truncation vs occlusion are different axes.** `despike` fixes vertical
  truncation (anchor top edge, restore to local *median* height). `restore_width`
  fixes horizontal occlusion (anchor the *stable* edge — check which of
  xmin/xmax has lower variance — restore to a high *percentile* width, because
  frequent occlusion drags the median down).
- **Cross-FOV reprojection is exact and cheap.** Both FOVs are center crops of
  the same source scaled to the same 4K, so a normalized bbox maps affinely
  about centre: `new = 0.5 + (old-0.5)·s`, `w·=sx`, `h·=sy`, with
  `s = crop(src)/crop(dst)` from `hailo_tiling.geometry.fov_to_crop_dims`. Validated
  against native detections to ~3e-4. Use it to fill people from fov50 into the
  wider FOVs; per-frame, so it works for moving objects too.
- **Extending a parked object backward:** don't freeze (it would drift off).
  `drift_extend` propagates a dense reference car's per-frame motion.
- **Smooth last, horizontally.** A static object's only real horizontal motion
  is the slow drift; `smooth --dims x,w` removes detector jitter without lag.

## Step 5 — Re-review and iterate

Re-render and watch: `python -m tiling_lab.viewer.overlay_viewer --video <mp4> --frames
<dir>/overlay_verified.frames.json:<label>`. The viewer paces in real time
(it subtracts per-frame render cost from the frame period). Iterate Step 4 until
clean. Note: corrections that can run more than once per chain
(`pin`, `crossfov_fill`, `drift_extend`, `restore_width`, `smooth`) accumulate
their records as a list in `corrections.json`.

## Step 6 — Lock

When a FOV is human-approved:

```bash
# write GT_STATUS.json with sha256 + correction chain (see the 0025 dirs for the schema),
# then make the authoritative artifacts read-only and commit:
chmod 444 <dir>/{gt_tracks.verified.json,corrections.json,GT_STATUS.json}
git add -f <dir>/{gt_tracks.verified.json,gt_tracks.json,corrections.json,GT_STATUS.json}
git commit -m "chore(gt): lock approved <clip> <fov> ground-truth"
```

`GT_STATUS.json` records `status/locked`, `sha256` of the verified file,
`tracks` (vehicle/person ids), `id_scheme`, and `corrections_chain`. The
`overlay_*.frames.json` files are regenerable and kept out of commits.

## Outputs per FOV dir

| file | what | committed? |
|---|---|---|
| `gt_tracks.json` | raw BoT-SORT tracks (chain input) | yes |
| `gt_tracks.verified.json` | **authoritative GT** (chmod 444) | yes |
| `corrections.json` | replay chain (provenance) | yes |
| `GT_STATUS.json` | approval + sha256 + chain | yes |
| `overlay_*.frames.json` | viewer overlays | no (regenerable) |
| `review_queue.json`, `review/*.png` | human-review artifacts | no |
