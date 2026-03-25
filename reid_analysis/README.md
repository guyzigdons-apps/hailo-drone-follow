# ReID Analysis — Person Re-Identification Evaluation Framework

Evaluate and tune the YOLO + ReID person matching pipeline on the Hailo NPU.

## Architecture

```
Video Input
    |
    v
reid_analysis_app.py  (GStreamer tiling pipeline + Hailo NPU)
    |
    |-- Person detection (YOLO via hailotilecropper)
    |-- Crop extraction (from normalized bboxes)
    |-- ReID embedding (RepVGG / OSNet on Hailo)
    |-- Gallery matching (cosine similarity)
    |
    v
match_log.jsonl  (one entry per detected crop)
    |
    v
reid_eval.py  (offline metrics from log + ground truth)
    |
    v
Precision, Fragmentation, ID Switches
    |
    v
reid_sweep.py  (automated parameter grid search)
```

## Files

| File | Purpose |
|------|---------|
| `reid_analysis_app.py` | Main pipeline: detect, embed, match, log |
| `reid_eval.py` | Compute metrics and sweep thresholds offline |
| `gallery_strategies.py` | Pluggable gallery update strategies |
| `reid_sweep.py` | Parameter sweep runner (model x threshold x strategy) |
| `ground_truth.json` | Manual mapping: predicted ID -> true person label |
| `match_log.jsonl` | Auto-generated log of every match decision |

## Quick Start

### 1. Run the pipeline

```bash
source setup_env.sh
python reid_analysis/reid_analysis_app.py \
    --input 12354541-hd_1280_720_25fps.mp4 \
    --tiles-x 2 --tiles-y 3 \
    --reid-model repvgg --threshold 0.7 \
    --gallery-strategy first_only \
    --video-sink fakesink --disable-sync
```

**Outputs:**
- `orig_person_images/` — First-seen reference crop per predicted person
- `person_images/{person_id}/` — All matched crops, organized by ID
- `match_log.jsonl` — Every match decision (frame, similarity, predicted ID)

### 2. Create ground truth

Review the reference crops in `orig_person_images/` and edit `ground_truth.json`:

```json
{
  "id_mapping": {
    "person_0": "A",
    "person_1": "B",
    "person_2": "A",
    "person_3": "false_positive"
  }
}
```

- Use letters (A, B, C, ...) for true person identities
- Use `"false_positive"` for bad detections (partial crops, multi-person boxes, background)

### 3. Evaluate

```bash
# Single threshold evaluation
python reid_analysis/reid_eval.py \
    --match-log reid_analysis/match_log.jsonl \
    --ground-truth reid_analysis/ground_truth.json

# Sweep thresholds offline (no re-run needed)
python reid_analysis/reid_eval.py \
    --match-log reid_analysis/match_log.jsonl \
    --ground-truth reid_analysis/ground_truth.json \
    --sweep
```

### 4. Parameter sweep (optional)

```bash
python reid_analysis/reid_sweep.py \
    --input 12354541-hd_1280_720_25fps.mp4 \
    --ground-truth reid_analysis/ground_truth.json \
    --tiles-x 2 --tiles-y 3
```

Default sweep: 2 models x 5 thresholds x 3 strategies = 30 runs.

## Gallery Strategies

| Strategy | `--gallery-strategy` | Description |
|----------|---------------------|-------------|
| First Only | `first_only` | Keep first embedding forever. Simple baseline. |
| Running Average | `running_average` | Gallery embedding = running average of all matches. Adapts to appearance changes. |
| Update Every N | `update_every_n` | Replace embedding every N matches. Use with `--gallery-update-interval`. |
| Multi-Embedding | `multi_embedding` | Store up to K embeddings per person, match = max similarity. Use with `--gallery-max-size`. |

## Metrics

| Metric | Meaning | Ideal |
|--------|---------|-------|
| **Precision** | Correct assignments / total assignments | 1.0 |
| **Fragmentation** | Avg predicted IDs per true person | 1.0 |
| **ID Switches** | Frame-to-frame identity flips for same true person | 0 |
| **New IDs** | Total predicted person IDs created | = true person count |

## ReID Models

| Model | Embedding Dim | Speed (Hailo-8) | HEF |
|-------|--------------|-----------------|-----|
| RepVGG A0 | 512 | ~5200 FPS | `repvgg_a0_person_reid_512.hef` |
| OSNet x1_0 | 512 | ~180 FPS | `osnet_x1_0.hef` |

Both use 256x128 input and L2-normalized embeddings (cosine similarity = dot product).

## Key CLI Arguments

### reid_analysis_app.py

| Argument | Default | Description |
|----------|---------|-------------|
| `--input` | — | Input video file or stream |
| `--tiles-x` / `--tiles-y` | 2 / 3 | Tiling grid for detection |
| `--reid-model` | `repvgg` | ReID model: `repvgg` or `osnet` |
| `--threshold` | `0.7` | Cosine similarity threshold |
| `--gallery-strategy` | `first_only` | Gallery update strategy |
| `--gallery-update-interval` | `10` | Frames between updates (for `update_every_n`) |
| `--gallery-max-size` | `10` | Max embeddings per person (for `multi_embedding`) |
| `--output-dir` | `.` (script dir) | Base output directory |
| `--video-sink` | `autovideosink` | GStreamer video sink (`fakesink` for headless) |
| `--disable-sync` | off | Run as fast as possible (no real-time sync) |

### reid_eval.py

| Argument | Default | Description |
|----------|---------|-------------|
| `--match-log` | — | Path to match_log.jsonl |
| `--ground-truth` | — | Path to ground_truth.json |
| `--sweep` | off | Sweep thresholds 0.3-0.95 offline |
| `--run-label` | — | Label for CSV output |
| `--output-csv` | — | Append results to CSV |
