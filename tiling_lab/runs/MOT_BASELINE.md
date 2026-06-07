# MOT scorecard — multi-target dynamic vs dense static, 0026-fov50 (person class)

First identity-aware (MOT) comparison. GT: `gt_verify_0026_fov50/gt_tracks.verified.json`
(person tracks, n_gt = 1419 box-frames over 879 frames). Scorer: in-repo
`mot_metrics.score_mot` (greedy IoU 0.5; IDF1 greedy-on-counts). Recovery config
frozen/0.001; budget 3000 inf/s.

| system | tiles/frame | MOTA | IDF1 | IDsw | FP | FN | Frag | MT/ML (of 11) |
|---|---|---|---|---|---|---|---|---|
| dynamic (8x6 disc @2fps + per-target ROIs) | **5.80** | −0.897 | 0.249 | 8 | 2420 | 264 | 26 | 8 / 1 |
| static dense replay (all cached tiles) | 8.87 | −0.670 | 0.341 | 9 | 2158 | 203 | 30 | 10 / 1 |

## Reading

1. **Dense static stays slightly ahead at whole-frame multi-target** (fewer misses,
   2 more mostly-tracked ids, +0.09 IDF1) — consistent with the 2026-06-01 finding
   that uniform grids win "detect everything". The new information: **dynamic gets
   within striking distance at ~65% of the tile budget** (5.80 vs 8.87 t/f) with
   identity quality (IDsw 8 vs 9) on par.
2. **Both MOTA values are negative and dominated by a detector failure, not
   tracking**: the VGA 4-class model mislabels many 0026 vehicles as "person"
   (~2.1-2.4k FP box-frames for both systems). Per-identity metrics (IDF1, MT,
   IDsw) are the meaningful columns on this clip.
3. The static baseline is **optimistic**: it replays every cached tile (mean 8.87
   t/f — more than a plain 8x6 sweep) — documented in `run_mot_eval --from-static-cache`.

## Bugs found en route (all fixed + regression-tested)

`run_dynamic` was still on the pre-Phase-0 class convention in TWO places:
CLI defaults (`591f557`: tracker built for the unlabeled class, GT-seeding never
fired) and backend construction (`eae88c1`: missing `class_offset=1` — the
multi-target trackers were following vehicles relabeled as persons). The
`--dump-mot` / `multi_traj` recording itself was verified correct.

## Reproduction

```bash
python -m dynamic_tiling.run_dynamic \
  --video .../DJI_20260528155239_0026_D_prepared__fov50.mp4 \
  --gt dynamic_tiling/runs/mot/gt_0026_legacy.frames.json --multi-target \
  --discovery-grid 8x6 --discovery-fps 2 --budget 3000 \
  --dump-mot dyn_mot.json --dump-mot-classes 1
python - # static replay: run_mot_eval.replay_static_cache(db, classes={1}, ppv=1)
python -m dynamic_tiling.run_mot_eval --gt-tracks .../gt_tracks.verified.json --pred dyn_mot.json --classes 1
```

Follow-ups: re-score with the Phase-A winner discovery config; a person-only NMS-score
floor or model upgrade to cut the shared FP flood; `--ppv` flag for the static replay CLI.
