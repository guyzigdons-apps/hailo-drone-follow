#!/usr/bin/env python3
"""Package the tiling visualizer site into a self-contained dist/ directory.

Reads site_config.json, transcodes source videos to browser-safe 1080p
H.264 (faststart), copies curated frames/trials JSONs, copies web/ and
writes data/manifest.json.

Usage:
    python3 package_site.py [--config site_config.json] [--out dist]
                            [--repo-root ../..] [--skip-transcode]
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


class ConfigError(Exception):
    pass


def parse_probe_csv(line: str) -> dict:
    """Parse `codec,width,height,fps_frac,nb_frames` from ffprobe csv output."""
    parts = line.strip().split(",")
    num, den = parts[3].split("/")
    frames = None if parts[4] in ("N/A", "") else int(parts[4])
    return {"width": int(parts[1]), "height": int(parts[2]),
            "fps": float(num) / float(den), "frames": frames}


def probe_video(path: Path) -> dict:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=codec_name,width,height,r_frame_rate,nb_frames",
         "-of", "csv=p=0", str(path)],
        check=True, capture_output=True, text=True).stdout
    info = parse_probe_csv(out)
    if info["frames"] is None:
        dur = subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration",
             "-of", "csv=p=0", str(path)],
            check=True, capture_output=True, text=True).stdout.strip()
        info["frames"] = int(float(dur) * info["fps"])
    # output dimensions after the 1080p transcode (scale=1920:-2)
    scale = min(1.0, 1920 / info["width"])
    info["out_width"] = int(info["width"] * scale)
    info["out_height"] = int(info["height"] * scale) // 2 * 2
    return info


def transcode_cmd(src: Path, dst: Path) -> list[str]:
    return ["ffmpeg", "-y", "-i", str(src),
            "-vf", "scale=1920:-2", "-c:v", "libx264", "-preset", "slow",
            "-crf", "22", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            "-an", str(dst)]


def needs_transcode(src: Path, dst: Path) -> bool:
    return not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime


def validate_config(cfg: dict, repo_root: Path) -> None:
    missing = []
    for video in cfg.get("videos", []):
        for var in video.get("variants", []):
            src = Path(var["source"])
            if not src.is_absolute():
                src = repo_root / src
            if not src.exists():
                missing.append(str(src))
            for entry in list(var.get("runs", [])) + list(var.get("trials", [])):
                p = repo_root / entry["path"]
                if not p.exists():
                    missing.append(str(p))
    if missing:
        raise ConfigError("missing input files:\n  " + "\n  ".join(missing))


def build_manifest(cfg: dict, probe_lookup) -> dict:
    videos = []
    for video in cfg.get("videos", []):
        variants = []
        for var in video.get("variants", []):
            probe = probe_lookup(Path(var["source"]))
            variants.append({
                "fov": var["fov"],
                "video": f"data/videos/{video['id']}_{var['fov']}.mp4",
                "width": probe.get("out_width", probe["width"]),
                "height": probe.get("out_height", probe["height"]),
                "fps": probe["fps"], "frames": probe["frames"],
                "runs": [{"id": r["id"], "label": r["label"], "type": r["type"],
                           "frames_json": f"data/runs/{r['id']}.frames.json"}
                          for r in var.get("runs", [])],
                "trials": [{"id": t["id"], "label": t["label"],
                             "trials_json": f"data/runs/{t['id']}.json"}
                            for t in var.get("trials", [])],
            })
        videos.append({"id": video["id"], "title": video["title"], "variants": variants})
    return {"generated": datetime.now(timezone.utc).isoformat(),
            "title": cfg.get("title", "Tiling Benchmark"), "videos": videos}


def main(argv=None) -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(here / "site_config.json"))
    ap.add_argument("--out", default=str(here / "dist"))
    ap.add_argument("--repo-root", default=str(here.parent))
    ap.add_argument("--skip-transcode", action="store_true",
                    help="copy/refresh everything except videos (fast iteration)")
    args = ap.parse_args(argv)

    cfg = json.loads(Path(args.config).read_text())
    repo_root = Path(args.repo_root).resolve()
    out = Path(args.out).resolve()
    validate_config(cfg, repo_root)

    # 1. site source -> dist root
    (out / "data" / "videos").mkdir(parents=True, exist_ok=True)
    (out / "data" / "runs").mkdir(parents=True, exist_ok=True)
    shutil.copytree(here / "web", out, dirs_exist_ok=True)
    for t in out.rglob("*.test.js"):       # tests don't ship
        t.unlink()

    # 2. run/trial JSONs
    for video in cfg["videos"]:
        for var in video["variants"]:
            for r in var.get("runs", []):
                shutil.copy2(repo_root / r["path"], out / "data" / "runs" / f"{r['id']}.frames.json")
            for t in var.get("trials", []):
                shutil.copy2(repo_root / t["path"], out / "data" / "runs" / f"{t['id']}.json")

    # 3. videos
    probes: dict[str, dict] = {}
    for video in cfg["videos"]:
        for var in video["variants"]:
            src = Path(var["source"])
            probes[str(src)] = probe_video(src)
            dst = out / "data" / "videos" / f"{video['id']}_{var['fov']}.mp4"
            if args.skip_transcode:
                print(f"[skip] {dst.name}")
            elif needs_transcode(src, dst):
                print(f"[transcode] {src.name} -> {dst.name}")
                subprocess.run(transcode_cmd(src, dst), check=True, capture_output=True)
            else:
                print(f"[fresh] {dst.name}")

    # 4. manifest
    manifest = build_manifest(cfg, probe_lookup=lambda p: probes[str(p)])
    (out / "data" / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"manifest written: {out / 'data' / 'manifest.json'}")
    print(f"dist ready: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
