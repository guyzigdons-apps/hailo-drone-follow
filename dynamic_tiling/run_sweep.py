"""Phase A single-target tiling sweep driver (Block 2, Task 1).

Coordinate-descent over the tiling knobs on 0025 (3 FOVs) with the best recovery
config (Block 1 + Block R) fixed, then an optional budget frontier at the best
config. Each config is scored by running ``run_all_trials`` in-process per FOV
through the SQLite caches, writing one result JSON per (config, fov) into
``dynamic_tiling/runs/phase_a/`` (reusing ``results_doc``). The renderer
(Task 3) aggregates those JSONs into ``PHASE_A.md``.

NO CHIP IS TOUCHED BY THE TESTS: ``coordinate_descent`` is pure, the clip
registry / AXES / BASE are static, and the main-loop tests monkeypatch
``run_all_trials`` plus the loaders. A real sweep requires the chip (or fully
warm caches) and the real videos — that execution is Task 2 (a follow-up).

Usage:
    source setup_env.sh
    python -m dynamic_tiling.run_sweep --passes 2 \
        --reacq-motion velocity --reacq-radius-growth 0.002 --reid-policy none
    python -m dynamic_tiling.run_sweep --budget-frontier \
        --reacq-motion velocity --reacq-radius-growth 0.002 --reid-policy none
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

# Reuse the CLI's shared construction so the driver never diverges from run_trials.
from .run_trials import (build_backend_factory, build_reid_assist_factory,
                         results_doc, _load_tracks, _frames_factory)
from .trials import run_all_trials

# --------------------------------------------------------------------------- #
# Static config                                                               #
# --------------------------------------------------------------------------- #

_VIDEO_DIR = Path("/home/giladn/Videos/Drone/Training")
_RUNS = Path("dynamic_tiling/runs")
_CACHE_DIR = _RUNS / "cache"

_DET_HEF = ("/usr/local/hailo/resources/models/hailo10h/"
            "hailo_yolov8n_4_classes_vga.hef")
_REID_HEF = ("/usr/local/hailo/resources/models/hailo10h/"
             "repvgg_a0_person_reid_512.hef")

# Recovery config defaults match run_reid_ablation's module constants (Block 1 /
# Block R best; fill in at execution time once BASELINE.md / REID_ABLATION.md land).
DEFAULT_REACQ_MOTION = "velocity"
DEFAULT_REACQ_RADIUS_GROWTH = 0.002
DEFAULT_REID_POLICY = "none"

DEFAULT_BUDGET = 3000.0
DEFAULT_PASSES = 2

DEFAULT_OUTDIR = _RUNS / "phase_a"

# The single video, three FOVs, that Phase A sweeps over.
FOVS = (50, 60, 70)

# Budget frontier sweep points (inf/s @ 30 fps).
FRONTIER_BUDGETS = (300, 600, 1000, 1500, 3000)

# Coordinate-descent search space + starting point (per the spec).
AXES = {
    "discovery_grid": ["4x3", "6x4", "8x6", "12x9"],
    "discovery_fps": [1, 2, 4],
    "discovery_overlap": [0.0, 0.15, 0.25],
    "max_zoom": [1.5, 2.0, 3.0],
    "target_model_h": [30.0, 40.0, 60.0],
}
BASE = {"discovery_grid": "8x6", "discovery_fps": 2, "discovery_overlap": 0.25,
        "max_zoom": 2.0, "target_model_h": 40.0}


@dataclass(frozen=True)
class Clip:
    key: str          # "0025:fov50"
    video: Path
    gt_tracks: Path
    cache: Path


def _clip(fov: int) -> Clip:
    fov_tag = f"fov{fov}"
    return Clip(
        key=f"0025:{fov_tag}",
        video=_VIDEO_DIR / f"DJI_20260528155151_0025_D_prepared__{fov_tag}.mp4",
        gt_tracks=_RUNS / f"gt_verify_0025_{fov_tag}" / "gt_tracks.verified.json",
        cache=_CACHE_DIR / f"0025_{fov_tag}__yolov8n4c_vga.sqlite3")


# The three 0025 FOVs Phase A sweeps over.
CLIPS: dict[str, Clip] = {f"0025:fov{f}": _clip(f) for f in FOVS}


# --------------------------------------------------------------------------- #
# Coordinate descent (pure)                                                   #
# --------------------------------------------------------------------------- #

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


# --------------------------------------------------------------------------- #
# Config -> filesystem-safe slug                                              #
# --------------------------------------------------------------------------- #

def cfg_slug(cfg: dict) -> str:
    """Deterministic, filesystem-safe slug for a config (no path separators,
    spaces, or colons). E.g. grid8x6_fps2_ov0.25_zoom2.0_h40."""
    def g(num):
        # 40.0 -> "40", 2.5 -> "2.5" (drop trailing .0 for the size knob)
        f = float(num)
        return f"{f:g}"

    return (f"grid{cfg['discovery_grid']}"
            f"_fps{g(cfg['discovery_fps'])}"
            f"_ov{g(cfg['discovery_overlap'])}"
            f"_zoom{float(cfg['max_zoom']):.1f}"
            f"_h{g(cfg['target_model_h'])}")


def _parse_grid(grid: str) -> tuple[int, int]:
    gx, gy = grid.lower().split("x")
    return int(gx), int(gy)


# --------------------------------------------------------------------------- #
# Helpers                                                                     #
# --------------------------------------------------------------------------- #

def _probe_dims(video: Path) -> tuple[int, int]:
    import cv2
    cap = cv2.VideoCapture(str(video))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    return w, h


def _build_reid_factory(*, reid_policy, reid_cache, reid_threshold=0.75):
    """(reid_assist_factory, embedder) for `reid_policy` (None,None for 'none').
    Thin wrapper over build_reid_assist_factory so the score loop can be tested
    without touching the chip (monkeypatch this)."""
    return build_reid_assist_factory(
        reid_policy=reid_policy, reid_hef=_REID_HEF, reid_cache=reid_cache,
        reid_threshold=reid_threshold, reid_gallery_ema=None)


def _run_one(clip: Clip, cfg: dict, *, budget, reacq_motion, reacq_radius_growth,
             reid_policy, outdir: Path, out_name: str) -> Path:
    """Run a single clip at `cfg` in-process and write its result JSON. Returns the
    path written."""
    from hailo_tiling.classes import PERSON

    tracks = _load_tracks(clip.gt_tracks)
    src_w, src_h = _probe_dims(clip.video)
    grid = _parse_grid(cfg["discovery_grid"])

    backend_factory = build_backend_factory(
        hef=_DET_HEF, nms_thresh=0.25, cache=clip.cache, video_name=clip.video.name)
    reid_assist_factory, reid_embedder = (None, None)
    if reid_policy != "none":
        reid_assist_factory, reid_embedder = _build_reid_factory(
            reid_policy=reid_policy, reid_cache=clip.cache)
    try:
        agg = run_all_trials(
            frames_factory=_frames_factory(clip.video, 0),
            src_w=src_w, src_h=src_h, gt_tracks=tracks,
            backend_factory=backend_factory,
            budget=budget, fps=30.0,
            discovery_fps=cfg["discovery_fps"],
            max_zoom=cfg["max_zoom"], target_model_h=cfg["target_model_h"],
            discovery_grid=grid, grid_overlap=cfg["discovery_overlap"],
            reacq_motion=reacq_motion, reacq_radius_growth=reacq_radius_growth,
            reid_assist_factory=reid_assist_factory)
    finally:
        if reid_embedder is not None:
            reid_embedder.close()

    params = {
        "video": str(clip.video), "gt_tracks": str(clip.gt_tracks),
        "hef": _DET_HEF, "budget": budget, "fps": 30.0,
        "discovery_fps": cfg["discovery_fps"],
        "discovery_grid": cfg["discovery_grid"],
        "discovery_overlap": cfg["discovery_overlap"], "max_zoom": cfg["max_zoom"],
        "target_model_h": cfg["target_model_h"], "reacq_motion": reacq_motion,
        "reacq_radius_growth": reacq_radius_growth, "max_frames": 0,
        "nms_thresh": 0.25, "reid_policy": reid_policy, "reid_hef": _REID_HEF,
        "reid_cache": str(clip.cache), "cache": str(clip.cache),
        "reid_threshold": 0.75, "reid_gallery_ema": None,
    }
    person_ids = [t.track_id for t in tracks if t.cls == PERSON]
    doc = results_doc(agg, person_track_ids=person_ids, params=params)
    out = outdir / out_name
    out.write_text(json.dumps(doc, indent=2))
    return doc, out


def make_score_fn(*, outdir: Path, budget, reacq_motion, reacq_radius_growth,
                  reid_policy):
    """Build the coordinate-descent score function. Each call runs the 3 FOVs of
    0025 in-process, writes per-fov JSONs to outdir/<slug>_fov<F>.json, prints the
    config's score, and returns mean(coverage) - 0.005*mean(tiles_per_frame)
    (tiles as a tie-break)."""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    def score_fn(cfg: dict) -> float:
        slug = cfg_slug(cfg)
        covs, tiles = [], []
        for fov in FOVS:
            clip = CLIPS[f"0025:fov{fov}"]
            doc, _ = _run_one(
                clip, cfg, budget=budget, reacq_motion=reacq_motion,
                reacq_radius_growth=reacq_radius_growth, reid_policy=reid_policy,
                outdir=outdir, out_name=f"{slug}_fov{fov}.json")
            agg = doc["aggregate"]
            covs.append(agg["mean_coverage"])
            tiles.append(agg["avg_tiles_per_frame"])
        mean_cov = sum(covs) / len(covs)
        mean_tiles = sum(tiles) / len(tiles)
        score = mean_cov - 0.005 * mean_tiles
        print(f"[sweep] {slug}: mean_coverage={mean_cov:.4f} "
              f"mean_tiles/frame={mean_tiles:.3f} score={score:.4f}")
        return score

    return score_fn


def run_sweep(*, outdir: Path, budget, reacq_motion, reacq_radius_growth,
              reid_policy, passes=DEFAULT_PASSES):
    """Coordinate-descent over AXES around BASE. Returns (best_config, history)."""
    score_fn = make_score_fn(
        outdir=outdir, budget=budget, reacq_motion=reacq_motion,
        reacq_radius_growth=reacq_radius_growth, reid_policy=reid_policy)
    best, history = coordinate_descent(BASE, AXES, score_fn, passes=passes)
    print(f"[sweep] best config = {best} (slug {cfg_slug(best)})")
    return best, history


def run_budget_frontier(*, cfg: dict, outdir: Path, reacq_motion,
                        reacq_radius_growth, reid_policy) -> list[Path]:
    """Run FRONTIER_BUDGETS x 3 fovs at `cfg`, writing
    frontier_b<B>_fov<F>.json. Returns the paths written (15 files)."""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    for b in FRONTIER_BUDGETS:
        for fov in FOVS:
            clip = CLIPS[f"0025:fov{fov}"]
            print(f"[sweep:frontier] budget={b} fov={fov}")
            _, out = _run_one(
                clip, cfg, budget=float(b), reacq_motion=reacq_motion,
                reacq_radius_growth=reacq_radius_growth, reid_policy=reid_policy,
                outdir=outdir, out_name=f"frontier_b{b}_fov{fov}.json")
            written.append(out)
    return written


# --------------------------------------------------------------------------- #
# CLI                                                                         #
# --------------------------------------------------------------------------- #

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--budget", type=float, default=DEFAULT_BUDGET)
    ap.add_argument("--passes", type=int, default=DEFAULT_PASSES,
                    help="coordinate-descent passes over the axes")
    ap.add_argument("--reacq-motion", choices=("frozen", "velocity"),
                    default=DEFAULT_REACQ_MOTION,
                    help="Block-1 best reacq motion model (fill in at execution time)")
    ap.add_argument("--reacq-radius-growth", type=float,
                    default=DEFAULT_REACQ_RADIUS_GROWTH,
                    help="Block-1 best reacq radius growth (fill in at execution time)")
    ap.add_argument("--reid-policy",
                    choices=("none", "generous", "prod", "ambiguity", "motion",
                             "histogram"),
                    default=DEFAULT_REID_POLICY,
                    help="Block-R best ReID recovery policy ('none' = no ReID)")
    ap.add_argument("--outdir", type=Path, default=DEFAULT_OUTDIR)
    ap.add_argument("--budget-frontier", action="store_true",
                    help="after the descent, run budgets "
                         f"{list(FRONTIER_BUDGETS)} at the best config (3 fovs each)")
    args = ap.parse_args()

    best, _ = run_sweep(
        outdir=args.outdir, budget=args.budget, reacq_motion=args.reacq_motion,
        reacq_radius_growth=args.reacq_radius_growth, reid_policy=args.reid_policy,
        passes=args.passes)

    if args.budget_frontier:
        run_budget_frontier(
            cfg=best, outdir=args.outdir, reacq_motion=args.reacq_motion,
            reacq_radius_growth=args.reacq_radius_growth,
            reid_policy=args.reid_policy)


if __name__ == "__main__":
    main()
