# Training Clips Manifest — 2026-05-28 shoot

**Source location:** `/home/giladn/Videos/Drone/Training/`
**Drone:** DJI Mavic 4 Pro (Hasselblad 4/3 main + 1/1.3" medium-tele + 1/1.5" tele)
**Captured by:** Gilad, 2026-05-28
**SRT subtitles:** enabled in DJI Fly settings (PTS-aligned 1 Hz GPS + camera params per clip)

## Clip inventory

| Clip ID | Camera        | Focal eq. | Native dims  | Native rotation | Duration | Spec variant role           |
|---------|---------------|-----------|--------------|-----------------|----------|------------------------------|
| 0025    | Main (1×)     | 28 mm     | 3384 × 6016  | portrait, rotate=90 | 43 s    | source for `FOV-70/60/50` emulation |
| 0026    | Main (1×)     | 28 mm     | 3384 × 6016  | portrait, rotate=90 | 29 s    | same                                  |
| 0027    | Main (1×)     | 28 mm     | 3384 × 6016  | portrait, rotate=90 | 63 s    | same                                  |
| 0028    | Med-tele (2.5×) | 70 mm   | 3840 × 2160  | landscape, none     | 66 s    | bonus task: mid-zoom range demo       |
| 0029    | Main (1×)     | 28 mm     | 3384 × 6016  | portrait, rotate=90 | 21 s    | source for `FOV-70/60/50` emulation   |
| 0030    | Med-tele (2.5×) | 70 mm   | 3840 × 2160  | landscape, none     | 47 s    | bonus task: mid-zoom range demo       |
| 0031    | Tele (6×)     | 168 mm    | 3840 × 2160  | landscape, none     | 67 s    | **`DJI-TELE-12`** main matrix         |
| 0032    | Main (1×)     | 28 mm     | 3384 × 6016  | portrait, rotate=90 | 37 s    | source for `FOV-70/60/50` emulation   |
| 0033    | Main (1×)     | 28 mm     | 3384 × 6016  | portrait, rotate=90 | 45 s    | source for `FOV-70/60/50` emulation   |
| 0034    | Main (1×)     | 28 mm     | 3384 × 6016  | portrait, rotate=90 | 68 s    | source for `FOV-70/60/50` emulation   |
| 0035    | Tele (6×)     | 168 mm    | 3840 × 2160  | landscape, none     | 92 s    | **`DJI-TELE-12`** main matrix         |

**Totals:** 7 main-camera 6K clips (5 min 6 s), 2 med-tele 4K clips (1 min 53 s), 2 tele 4K clips (2 min 39 s). 11 clips, 9 min 38 s aggregate runtime.

## Rotation handling

The 7 main-camera clips are stored 3384 × 6016 (portrait) with `rotate=90` / Display-Matrix-rotation `-90`. GStreamer's `decodebin` does **not** auto-apply rotation metadata, so feeding the raw file to the tiling pipeline yields stretched-portrait frames and detection collapses (same issue documented for the older `0010_D` clip — see `tiling_benchmark/prepare_video.py` docstring).

**Fix:** run `tiling_benchmark/prepare_video.py CLIP.MP4` on each — re-encodes with `ffmpeg -c:v libx265 -crf 22 -preset fast -metadata:s:v rotate=0` to bake the rotation into pixels and emit `CLIP_prepared.MP4` at 6016 × 3384 landscape. The 4 already-landscape 4K clips (0028, 0030, 0031, 0035) need no prep.

The first parallel run during this session was OOM-killed (7 simultaneous libx265 encodes saturated RAM); **sequential** re-encode works on this host.

## Telemetry

Each clip has a matching `.SRT` sidecar with one entry per video frame containing:

- `FrameCnt`, `DiffTime`
- ISO wall-clock timestamp
- `iso`, `shutter`, `fnum`, `ev`, `color_md`, `focal_len`, `ct` (colour temp), `tint`
- `latitude`, `longitude` (WGS-84 degrees)
- `rel_alt` (height above takeoff, m), `abs_alt` (MSL, m)

GPS for the 28 mm clips clusters around 31.884°, 35.026° (north / west of Jerusalem). The `focal_len` field is the authoritative camera identifier (28 mm = main, 70 mm = medium-tele, 168 mm = tele).

Plan 7 (`hailo-tiling-import-srt`) parses these into the `telemetry` table of `flight_record.sqlite3`. Until then, the SRT files stay alongside their MP4s untouched.

## GT generation plan

GT is generated **per prepared clip** by the existing `tiling_benchmark/run_pxt_yolov8m.py` driver (yolov8m HEF, 12×9 grid + 25% overlap + extra 1×1 and 3×2 grids — `GT-12x9-25-multi` config). Output goes to a per-clip subdirectory under `tiling_benchmark/pxt_runs_yolov8m/` so the 11 clips' outputs don't collide.

Runtime: ~50 min per clip on Hailo-10H (~115 tile inferences/frame × 30 fps × clip duration). Total ~9 h for all 11 clips — runs as a background activity concurrent with library-refactor work.

GT files are committed (small JSONs, < 5 MB each). The MP4 sources are **not** committed (they're large; live under `/home/giladn/Videos/Drone/`).

## Reproducibility note for the paper

Per the spec (Section 10 — Paper-with-Code Release Workflow), **one** of these 28 mm clips will be released under CC-BY-SA on Zenodo as the reproduction-recipe anchor. Selection criterion: shortest clip that contains the full follow-scenario variety (close → far slant range, walking → stopping → resuming). Candidate: **0027** (63 s, longest of the 28 mm set that's still under 1 minute trimmed). Final pick is made in Plan 9 (paper artifacts) — doesn't block any earlier plan.
