"""ReID inference-budget ablation driver (Block R, Task 6).

Runs each clip x arm in-process via ``run_all_trials`` (the same construction the
``run_trials`` CLI uses, factored into ``build_backend_factory`` /
``build_reid_assist_factory``), writes one result JSON per (clip, arm) into
``--outdir``, then renders ``REID_ABLATION.md``.

Arms: none (P0, no ReID), generous (P1), prod (P2), ambiguity (P3),
motion (P4, motion-gated+decay), histogram (P5, motion gate + HSV color screen).
The motion/histogram arms inherit the Block-1 best reacq config
(``--reacq-motion`` / ``--reacq-radius-growth``) so their gate geometry matches.

NO CHIP IS TOUCHED BY THE TESTS: the registry, arm->policy mapping, and
render_report are all pure / fixture-driven; the driver loop is exercised with a
monkeypatched ``run_all_trials``. A real ablation run requires the chip (or fully
warm caches) and the real videos — that execution is a separate follow-up.

Usage:
    source setup_env.sh
    python -m tiling_lab.cli.run_reid_ablation                  # full ablation
    python -m tiling_lab.cli.run_reid_ablation --gallery-sub-ablation
    python -m tiling_lab.cli.run_reid_ablation --render-only     # MD from JSONs
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

# Reuse the CLI's shared construction so the driver never diverges from run_trials.
from .run_trials import (build_backend_factory, build_reid_assist_factory,
                         results_doc, _load_tracks, _frames_factory)
from tiling_lab.harness.trials import run_all_trials
from tiling_lab.reid.reid_policy import (POLICIES, GenerousPolicy, ProdPolicy, AmbiguityPolicy,
                          MotionGatedPolicy, HistogramPolicy, ReidAssist)
from tiling_lab.reid.reid_embedder import make_hef_embedder
from tiling_lab.reid.reid_gallery import ReidGallery

# --------------------------------------------------------------------------- #
# Static config                                                               #
# --------------------------------------------------------------------------- #

_VIDEO_DIR = Path("/home/giladn/Videos/Drone/Training")
_RUNS = Path("tiling_lab/runs")
_CACHE_DIR = _RUNS / "cache"

_DET_HEF = ("/usr/local/hailo/resources/models/hailo10h/"
            "hailo_yolov8n_4_classes_vga.hef")
_REID_HEF = ("/usr/local/hailo/resources/models/hailo10h/"
             "repvgg_a0_person_reid_512.hef")

# Block-1 best (filled in at EXECUTION time once BASELINE.md validation lands).
# Defaults below are the plan's placeholders.
DEFAULT_REACQ_MOTION = "velocity"
DEFAULT_REACQ_RADIUS_GROWTH = 0.002
DEFAULT_BUDGET = 3000.0

DEFAULT_CLIPS = "0025:fov50,0025:fov60,0025:fov70,0026:fov50"
DEFAULT_ARMS = "none,generous,prod,ambiguity,motion,histogram"
DEFAULT_OUTDIR = _RUNS / "reid_ablation"

# Sub-ablation operates on these clips (one per video) per the plan.
SUB_ABLATION_CLIPS = ["0025:fov50", "0026:fov50"]


@dataclass(frozen=True)
class Clip:
    key: str          # "0025:fov50"
    video: Path
    gt_tracks: Path
    cache: Path


def _clip(key: str, video_name: str, fov: str, clip_tag: str) -> Clip:
    return Clip(
        key=key,
        video=_VIDEO_DIR / video_name,
        gt_tracks=_RUNS / f"gt_verify_{clip_tag}_{fov}" / "gt_tracks.verified.json",
        cache=_CACHE_DIR / f"{clip_tag}_{fov}__yolov8n4c_vga.sqlite3")


# Exact paths copied from the plan + on-disk state (0025 fovs 50/60/70 + 0026 fov50).
CLIPS: dict[str, Clip] = {
    "0025:fov50": _clip("0025:fov50",
                        "DJI_20260528155151_0025_D_prepared__fov50.mp4",
                        "fov50", "0025"),
    "0025:fov60": _clip("0025:fov60",
                        "DJI_20260528155151_0025_D_prepared__fov60.mp4",
                        "fov60", "0025"),
    "0025:fov70": _clip("0025:fov70",
                        "DJI_20260528155151_0025_D_prepared__fov70.mp4",
                        "fov70", "0025"),
    "0026:fov50": _clip("0026:fov50",
                        "DJI_20260528155239_0026_D_prepared__fov50.mp4",
                        "fov50", "0026"),
}

# Arm name -> policy CLASS (None for the P0 no-ReID arm).
ARMS: dict[str, type | None] = {
    "none": None,
    "generous": GenerousPolicy,
    "prod": ProdPolicy,
    "ambiguity": AmbiguityPolicy,
    "motion": MotionGatedPolicy,
    "histogram": HistogramPolicy,
}


def make_policy(arm: str, *, radius_growth: float = DEFAULT_REACQ_RADIUS_GROWTH):
    """Instantiate the policy for `arm` with the right knobs. The motion/histogram
    arms thread the Block-1 best `radius_growth` into the motion gate; others use
    their own defaults (which ignore radius_growth)."""
    cls = ARMS[arm]
    if cls is None:
        raise KeyError(f"arm {arm!r} has no policy (P0 / no-ReID)")
    if issubclass(cls, MotionGatedPolicy):
        return cls(radius_growth=radius_growth)
    return cls()


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


def _out_name(clip_key: str, arm: str) -> str:
    return f"{clip_key.replace(':', '_')}__{arm}.json"


def _reid_assist_factory_for_arm(arm: str, *, radius_growth, reid_threshold,
                                 reid_cache, ema_alpha=None):
    """Build (factory, embedder) for an arm, threading the Block-1 radius_growth
    into motion-based policies. Mirrors build_reid_assist_factory but lets us pass
    a per-arm policy instance (so motion/histogram get the right gate)."""
    if ARMS[arm] is None:
        return None, None
    embedder = make_hef_embedder(_REID_HEF, cache_path=reid_cache)

    def factory():
        gallery = ReidGallery(reid_threshold=reid_threshold, ema_alpha=ema_alpha)
        return ReidAssist(embedder, gallery, make_policy(arm, radius_growth=radius_growth))

    return factory, embedder


def _run_one(clip: Clip, arm: str, *, budget, reacq_motion, reacq_radius_growth,
             outdir: Path, ema_alpha=None, name_suffix: str = "") -> Path:
    """Run a single clip x arm in-process and write its result JSON. Returns the
    path written."""
    from hailo_tiling.classes import PERSON

    tracks = _load_tracks(clip.gt_tracks)
    src_w, src_h = _probe_dims(clip.video)
    backend_factory = build_backend_factory(
        hef=_DET_HEF, nms_thresh=0.25, cache=clip.cache, video_name=clip.video.name)
    reid_assist_factory, reid_embedder = _reid_assist_factory_for_arm(
        arm, radius_growth=reacq_radius_growth, reid_threshold=0.75,
        reid_cache=clip.cache, ema_alpha=ema_alpha)
    try:
        agg = run_all_trials(
            frames_factory=_frames_factory(clip.video, 0),
            src_w=src_w, src_h=src_h, gt_tracks=tracks,
            backend_factory=backend_factory,
            budget=budget, fps=30.0, discovery_fps=2.0,
            max_zoom=2.0, target_model_h=40.0,
            discovery_grid=(8, 6), grid_overlap=0.25,
            reacq_motion=reacq_motion, reacq_radius_growth=reacq_radius_growth,
            reid_assist_factory=reid_assist_factory)
    finally:
        if reid_embedder is not None:
            reid_embedder.close()

    params = {
        "video": str(clip.video), "gt_tracks": str(clip.gt_tracks),
        "hef": _DET_HEF, "budget": budget, "fps": 30.0, "discovery_fps": 2.0,
        "discovery_grid": "8x6", "discovery_overlap": 0.25, "max_zoom": 2.0,
        "target_model_h": 40.0, "reacq_motion": reacq_motion,
        "reacq_radius_growth": reacq_radius_growth, "max_frames": 0,
        "nms_thresh": 0.25, "reid_policy": arm, "reid_hef": _REID_HEF,
        "reid_cache": str(clip.cache), "cache": str(clip.cache),
        "reid_threshold": 0.75, "reid_gallery_ema": ema_alpha,
    }
    person_ids = [t.track_id for t in tracks if t.cls == PERSON]
    doc = results_doc(agg, person_track_ids=person_ids, params=params)
    out = outdir / f"{clip.key.replace(':', '_')}__{arm}{name_suffix}.json"
    out.write_text(json.dumps(doc, indent=2))
    return out


def run_ablation(*, clips, arms, budget, reacq_motion, reacq_radius_growth,
                 outdir: Path) -> list[Path]:
    """Main driver: run every clip x arm, write one JSON each. Returns paths."""
    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    for clip_key in clips:
        clip = CLIPS[clip_key]
        for arm in arms:
            print(f"[reid-ablation] {clip_key} x {arm}")
            written.append(_run_one(
                clip, arm, budget=budget, reacq_motion=reacq_motion,
                reacq_radius_growth=reacq_radius_growth, outdir=outdir))
    return written


# Sub-ablation gallery variants: label -> ema_alpha. FIFO keeps only the rolling
# vector list (no EMA anchor); ema/both add the StrongSORT-style EMA anchor at
# alpha=0.1 (matched alongside the FIFO vectors). "ema" and "both" share the same
# alpha; "both" is the explicit "FIFO + EMA" combo, "ema" is the EMA-leaning run.
_SUB_VARIANTS = {"fifo": None, "ema": 0.1, "both": 0.1}


def _winning_arm(outdir: Path, arms) -> str:
    """Highest mean_coverage across the sub-ablation clips, from the main-run JSONs."""
    best_arm, best_cov = None, -1.0
    for arm in arms:
        if ARMS[arm] is None:
            continue  # P0 never wins a ReID sub-ablation
        covs = []
        for clip_key in SUB_ABLATION_CLIPS:
            p = outdir / _out_name(clip_key, arm)
            if p.exists():
                covs.append(json.loads(p.read_text())["aggregate"]["mean_coverage"])
        if covs:
            mean = sum(covs) / len(covs)
            if mean > best_cov:
                best_arm, best_cov = arm, mean
    if best_arm is None:
        raise RuntimeError("no main-run JSONs found to pick a sub-ablation winner")
    return best_arm


def run_gallery_sub_ablation(*, arms, budget, reacq_motion, reacq_radius_growth,
                             outdir: Path) -> list[Path]:
    """FIFO vs ema=0.1 vs both runs of the winning arm on the sub-ablation clips."""
    outdir.mkdir(parents=True, exist_ok=True)
    winner = _winning_arm(outdir, arms)
    print(f"[reid-ablation] gallery sub-ablation winner = {winner}")
    written = []
    for clip_key in SUB_ABLATION_CLIPS:
        clip = CLIPS[clip_key]
        for variant, ema in _SUB_VARIANTS.items():
            print(f"[reid-ablation] sub {clip_key} x {winner} x {variant}")
            written.append(_run_one(
                clip, winner, budget=budget, reacq_motion=reacq_motion,
                reacq_radius_growth=reacq_radius_growth, outdir=outdir,
                ema_alpha=ema, name_suffix=f"__{variant}"))
    return written


# --------------------------------------------------------------------------- #
# Report                                                                      #
# --------------------------------------------------------------------------- #

_COLS = [
    ("coverage", "mean_coverage", "{:.3f}"),
    ("mean IoU", "mean_iou", "{:.3f}"),
    ("losses", "mean_loss_events", "{:.2f}"),
    ("t-to-recover", "mean_time_to_recover", "{:.2f}"),
    ("recovery", "mean_recovery_success", "{:.3f}"),
    ("frac dets embedded", "mean_frac_dets_embedded", "{:.3f}"),
]


def _arm_of(doc) -> str:
    return doc["params"]["reid_policy"]


def _repro_cmd(doc) -> str:
    p = doc["params"]
    return (
        "python -m tiling_lab.cli.run_trials "
        f"--video {p['video']} --gt-tracks {p['gt_tracks']} "
        f"--budget {p['budget']:g} --discovery-grid {p.get('discovery_grid', '8x6')} "
        f"--discovery-fps {p.get('discovery_fps', 2):g} "
        f"--discovery-overlap {p.get('discovery_overlap', 0.25):g} "
        f"--reacq-motion {p['reacq_motion']} "
        f"--reacq-radius-growth {p['reacq_radius_growth']:g} "
        f"--reid-policy {p['reid_policy']} "
        f"--cache {p.get('cache')} --reid-cache {p.get('reid_cache')}")


# Stable arm display order for tables.
_ARM_ORDER = ["none", "generous", "prod", "ambiguity", "motion", "histogram"]


def render_report(outdir: Path) -> str:
    """Build REID_ABLATION.md content from the result JSONs in `outdir`.

    Per-clip tables (rows = arms; cols = the metric suite), a combined quality-vs-
    fraction-embedded Pareto section, the exact repro command per run, and a
    data-driven chosen-default paragraph."""
    outdir = Path(outdir)
    # group docs by clip; main runs (no __variant suffix) only, sub-ablation runs
    # have a 3rd "__" segment.
    main: dict[str, dict[str, dict]] = {}  # clip_stem -> arm -> doc
    sub: list[tuple[str, str, str, dict]] = []  # (clip_stem, arm, variant, doc)
    for p in sorted(outdir.glob("*.json")):
        parts = p.stem.split("__")
        doc = json.loads(p.read_text())
        if len(parts) == 2:
            clip_stem, arm = parts
            main.setdefault(clip_stem, {})[arm] = doc
        elif len(parts) == 3:
            clip_stem, arm, variant = parts
            sub.append((clip_stem, arm, variant, doc))

    lines: list[str] = []
    lines.append("# ReID Inference-Budget Ablation (Block R)")
    lines.append("")
    lines.append("Person-crop-only ReID recovery on top of the Block-1 best "
                 "re-acquisition config. Arms: **none** (P0, no ReID), **generous** "
                 "(P1 upper bound), **prod** (P2), **ambiguity** (P3), **motion** "
                 "(P4 motion-gated + cadence decay), **histogram** (P5 motion gate "
                 "+ HSV color screen).")
    lines.append("")
    if main:
        ex = next(iter(next(iter(main.values())).values()))
        p = ex["params"]
        det_hef = Path(p.get("hef", "?")).name
        reid_hef = Path(p.get("reid_hef", "?")).name
        lines.append(f"Budget {p.get('budget', 0):g} inf/s @ {p.get('fps', 30):g} fps; "
                     f"reacq {p.get('reacq_motion', '?')} / radius-growth "
                     f"{p.get('reacq_radius_growth', 0):g}; det HEF `{det_hef}`, "
                     f"ReID HEF `{reid_hef}`.")
        lines.append("")

    # ---- per-clip tables ----
    pareto: list[tuple[str, str, float, float]] = []  # (clip, arm, frac, coverage)
    for clip_stem in sorted(main):
        lines.append(f"## {clip_stem}")
        lines.append("")
        header = "| arm | " + " | ".join(c[0] for c in _COLS) + " |"
        sep = "|" + "---|" * (len(_COLS) + 1)
        lines.append(header)
        lines.append(sep)
        arms_present = [a for a in _ARM_ORDER if a in main[clip_stem]]
        arms_present += [a for a in main[clip_stem] if a not in _ARM_ORDER]
        for arm in arms_present:
            agg = main[clip_stem][arm]["aggregate"]
            cells = [fmt.format(agg.get(key, 0.0)) for _, key, fmt in _COLS]
            lines.append(f"| {arm} | " + " | ".join(cells) + " |")
            pareto.append((clip_stem, arm,
                           float(agg.get("mean_frac_dets_embedded", 0.0)),
                           float(agg.get("mean_coverage", 0.0))))
        lines.append("")

    # ---- quality vs fraction embedded (Pareto read) ----
    lines.append("## Quality vs fraction of person-dets embedded")
    lines.append("")
    lines.append("Ordered by embedding fraction (lower = cheaper). The Pareto read: "
                 "the arm that holds coverage closest to **generous** at the lowest "
                 "fraction wins.")
    lines.append("")
    lines.append("| clip | arm | frac dets embedded | coverage |")
    lines.append("|---|---|---|---|")
    for clip_stem, arm, frac, cov in sorted(pareto, key=lambda t: (t[0], t[2])):
        lines.append(f"| {clip_stem} | {arm} | {frac:.3f} | {cov:.3f} |")
    lines.append("")

    # ---- gallery sub-ablation ----
    if sub:
        lines.append("## Gallery sub-ablation (winning arm)")
        lines.append("")
        lines.append("| clip | arm | variant | coverage | recovery | frac embedded |")
        lines.append("|---|---|---|---|---|---|")
        for clip_stem, arm, variant, doc in sorted(sub):
            a = doc["aggregate"]
            lines.append(
                f"| {clip_stem} | {arm} | {variant} | "
                f"{a.get('mean_coverage', 0.0):.3f} | "
                f"{a.get('mean_recovery_success', 0.0):.3f} | "
                f"{a.get('mean_frac_dets_embedded', 0.0):.3f} |")
        lines.append("")

    # ---- chosen default ----
    lines.append("## Chosen default policy")
    lines.append("")
    chosen = _choose_default(main)
    if chosen is None:
        lines.append("_(No result JSONs found — run the ablation first, then "
                     "`--render-only`.)_")
    else:
        arm, cov, frac, gen_cov = chosen
        lines.append(
            f"**{arm}** is the proposed default. Across all clips it achieves mean "
            f"coverage {cov:.3f} (generous upper bound {gen_cov:.3f}) while embedding "
            f"only {frac:.3f} of visible person-dets. <!-- TODO at execution time: "
            f"confirm this paragraph reflects the final numbers and add the rationale "
            f"vs the runner-up arm. -->")
    lines.append("")

    # ---- reproduction ----
    lines.append("## Reproduction")
    lines.append("")
    lines.append("Driver (all clips x arms):")
    lines.append("")
    lines.append("```bash")
    lines.append("source setup_env.sh")
    lines.append("python -m tiling_lab.cli.run_reid_ablation "
                 f"--clips {DEFAULT_CLIPS} --arms {DEFAULT_ARMS} "
                 f"--budget {int(DEFAULT_BUDGET)}")
    lines.append("python -m tiling_lab.cli.run_reid_ablation --gallery-sub-ablation")
    lines.append("python -m tiling_lab.cli.run_reid_ablation --render-only")
    lines.append("```")
    lines.append("")
    if main:
        lines.append("Per-row single runs:")
        lines.append("")
        lines.append("```bash")
        for clip_stem in sorted(main):
            for arm in [a for a in _ARM_ORDER if a in main[clip_stem]]:
                lines.append(_repro_cmd(main[clip_stem][arm]))
        lines.append("```")
        lines.append("")
    return "\n".join(lines)


def _choose_default(main):
    """Pick the data-driven default arm: among non-P0 arms, the one whose mean
    coverage across clips is within a small margin of generous at the lowest mean
    embedding fraction. Falls back to the highest-coverage arm. Returns
    (arm, mean_cov, mean_frac, generous_mean_cov) or None when no data."""
    if not main:
        return None
    # aggregate per-arm across clips
    by_arm: dict[str, list[tuple[float, float]]] = {}
    for clip in main.values():
        for arm, doc in clip.items():
            a = doc["aggregate"]
            by_arm.setdefault(arm, []).append(
                (float(a.get("mean_coverage", 0.0)),
                 float(a.get("mean_frac_dets_embedded", 0.0))))
    means = {arm: (sum(c for c, _ in v) / len(v), sum(f for _, f in v) / len(v))
             for arm, v in by_arm.items()}
    gen_cov = means.get("generous", (0.0, 0.0))[0]
    # candidates: non-P0, coverage within 0.02 of generous
    cands = [(arm, cov, frac) for arm, (cov, frac) in means.items()
             if arm != "none" and cov >= gen_cov - 0.02]
    if cands:
        arm, cov, frac = min(cands, key=lambda t: t[2])  # cheapest qualifying
    else:
        # fall back to highest coverage non-P0 arm
        arm, (cov, frac) = max(
            ((a, m) for a, m in means.items() if a != "none"),
            key=lambda t: t[1][0], default=("generous", means.get("generous", (0.0, 0.0))))
    return arm, cov, frac, gen_cov


# --------------------------------------------------------------------------- #
# CLI                                                                         #
# --------------------------------------------------------------------------- #

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clips", default=DEFAULT_CLIPS,
                    help="comma-separated clip keys (e.g. 0025:fov50,0026:fov50)")
    ap.add_argument("--arms", default=DEFAULT_ARMS,
                    help="comma-separated arm names (none,generous,prod,ambiguity,"
                         "motion,histogram)")
    ap.add_argument("--budget", type=float, default=DEFAULT_BUDGET)
    ap.add_argument("--reacq-motion", choices=("frozen", "velocity"),
                    default=DEFAULT_REACQ_MOTION,
                    help="Block-1 best reacq motion model (fill in at execution time)")
    ap.add_argument("--reacq-radius-growth", type=float,
                    default=DEFAULT_REACQ_RADIUS_GROWTH,
                    help="Block-1 best reacq radius growth (fill in at execution time)")
    ap.add_argument("--outdir", type=Path, default=DEFAULT_OUTDIR)
    ap.add_argument("--gallery-sub-ablation", action="store_true",
                    help="also run FIFO vs ema=0.1 vs both for the winning arm on "
                         "0025:fov50 + 0026:fov50 (winner = highest mean coverage)")
    ap.add_argument("--render-only", action="store_true",
                    help="regenerate REID_ABLATION.md from existing JSONs; run nothing")
    args = ap.parse_args()

    clips = [c.strip() for c in args.clips.split(",") if c.strip()]
    arms = [a.strip() for a in args.arms.split(",") if a.strip()]
    for c in clips:
        if c not in CLIPS:
            ap.error(f"unknown clip {c!r}; known: {', '.join(CLIPS)}")
    for a in arms:
        if a not in ARMS:
            ap.error(f"unknown arm {a!r}; known: {', '.join(ARMS)}")

    if not args.render_only:
        run_ablation(clips=clips, arms=arms, budget=args.budget,
                     reacq_motion=args.reacq_motion,
                     reacq_radius_growth=args.reacq_radius_growth,
                     outdir=args.outdir)
        if args.gallery_sub_ablation:
            run_gallery_sub_ablation(
                arms=arms, budget=args.budget, reacq_motion=args.reacq_motion,
                reacq_radius_growth=args.reacq_radius_growth, outdir=args.outdir)

    md = render_report(args.outdir)
    md_path = _RUNS / "REID_ABLATION.md"
    md_path.write_text(md)
    print(f"report -> {md_path}")


if __name__ == "__main__":
    main()
