"""Prepare a source video for the tiling benchmark.

DJI (and most phone-captured) MP4s carry rotation metadata — either a legacy
`tags.rotate=N` field, or a modern `side_data_list[].rotation=N` Display
Matrix. GStreamer's `decodebin` does NOT auto-apply that rotation, so a
portrait clip gets fed to the model as horizontally-stretched landscape and
detection collapses.

This helper detects rotation metadata and, if present, re-encodes the video
with `ffmpeg`, baking the rotation into the pixels and clearing the metadata
(`-metadata:s:v rotate=0 -an`). If no rotation is present, the input is
considered already landscape-safe and the script exits cleanly without
touching it (unless `--copy` is passed).

Usage:
  python -m tiling_lab.video.prepare_video /path/to/source.MP4
  python -m tiling_lab.video.prepare_video source.MP4 --output prepared.MP4
  python -m tiling_lab.video.prepare_video source.MP4 --force --verify
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# Single source of truth for the 6K-crop geometry — promoted into the installed
# library. fov_to_crop_dims + the source-dimension / FOV constants live there.
from hailo_tiling.geometry import (
    fov_to_crop_dims,
    SRC_6K_W,
    SRC_6K_H,
    SRC_NATIVE_FOV_DEG,
    ALLOWED_FOVS,
)


def run(cmd: list[str], capture: bool = False) -> subprocess.CompletedProcess:
    """Run a shell command, printing it first."""
    print(f"+ {' '.join(str(c) for c in cmd)}", flush=True)
    return subprocess.run(cmd, check=False,
                          capture_output=capture, text=capture)


def ffprobe_json(path: Path) -> dict:
    """Return ffprobe -of json output for `path` as a dict."""
    cmd = ["ffprobe", "-v", "error", "-of", "json",
           "-show_streams", "-show_format", str(path)]
    proc = run(cmd, capture=True)
    if proc.returncode != 0:
        print(f"ERROR: ffprobe failed: {proc.stderr.strip()}", file=sys.stderr)
        sys.exit(1)
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError as e:
        print(f"ERROR: ffprobe JSON parse failed: {e}", file=sys.stderr)
        sys.exit(1)


def video_stream(probe: dict) -> dict:
    """Return the first video stream from an ffprobe dict."""
    for s in probe.get("streams", []):
        if s.get("codec_type") == "video":
            return s
    print("ERROR: no video stream found", file=sys.stderr)
    sys.exit(1)


def detect_rotation(stream: dict) -> int:
    """Return rotation in degrees from tags.rotate or Display Matrix side-data.

    Returns 0 when no rotation metadata is present.
    """
    tag_rot = stream.get("tags", {}).get("rotate")
    if tag_rot is not None:
        try:
            r = int(tag_rot) % 360
            if r != 0:
                return r
        except (TypeError, ValueError):
            pass
    for sd in stream.get("side_data_list", []) or []:
        if sd.get("side_data_type") == "Display Matrix":
            r = sd.get("rotation")
            if r is not None:
                try:
                    return int(round(float(r))) % 360
                except (TypeError, ValueError):
                    pass
    return 0


def default_output(input_path: Path) -> Path:
    return input_path.with_name(f"{input_path.stem}_prepared{input_path.suffix}")


def reencode(input_path: Path, output_path: Path) -> None:
    """Re-encode `input_path` to `output_path`, dropping rotation metadata."""
    cmd = ["ffmpeg", "-y", "-i", str(input_path),
           "-c:v", "libx265", "-crf", "22", "-preset", "fast",
           "-metadata:s:v", "rotate=0",
           "-an",
           str(output_path)]
    proc = run(cmd)
    if proc.returncode != 0:
        print(f"ERROR: ffmpeg exited with code {proc.returncode}", file=sys.stderr)
        sys.exit(1)


def verify_output(output_path: Path) -> None:
    """Verify the output is landscape via ffprobe AND a one-frame gst decode."""
    probe = ffprobe_json(output_path)
    stream = video_stream(probe)
    w = int(stream["width"])
    h = int(stream["height"])
    rot = detect_rotation(stream)
    if rot != 0:
        print(f"ERROR: output still carries rotation={rot}", file=sys.stderr)
        sys.exit(1)
    if not (h < w):
        print(f"ERROR: output dims {w}x{h} are not landscape (h<w)",
              file=sys.stderr)
        sys.exit(1)
    print(f"  ffprobe OK: {w}x{h}, rotation=0")

    # GStreamer decode test — one frame to JPEG, confirm dims match.
    with tempfile.TemporaryDirectory() as tmp:
        out_jpg = Path(tmp) / "frame_%05d.jpg"
        cmd = ["gst-launch-1.0", "-q",
               "filesrc", f"location={output_path}", "!",
               "decodebin", "!", "videoconvert", "!",
               "jpegenc", "!",
               "multifilesink", f"location={out_jpg}",
               "max-files=1", "next-file=0"]
        proc = run(cmd)
        if proc.returncode != 0:
            print(f"ERROR: gst-launch-1.0 exited with code {proc.returncode}",
                  file=sys.stderr)
            sys.exit(1)
        jpgs = sorted(Path(tmp).glob("frame_*.jpg"))
        if not jpgs:
            print("ERROR: gst-launch produced no frames", file=sys.stderr)
            sys.exit(1)
        # Use ffprobe on the JPEG to confirm dims (avoids a Pillow dep).
        jpg_probe = ffprobe_json(jpgs[0])
        jpg_stream = video_stream(jpg_probe)
        jw = int(jpg_stream["width"])
        jh = int(jpg_stream["height"])
        if (jw, jh) != (w, h):
            print(f"ERROR: gst-decoded frame dims {jw}x{jh} do not match "
                  f"ffprobe dims {w}x{h}", file=sys.stderr)
            sys.exit(1)
        print(f"  gst decode OK: {jw}x{jh}")


def _vf_filter_for_fov(fov_deg: int) -> str:
    """Build the -vf filter chain string for a given FOV.

    - fov_deg=70 → no crop, only lanczos scale to 4K.
    - fov_deg<70 → center crop to fov-derived dims, then lanczos scale to 4K.
    """
    if fov_deg == SRC_NATIVE_FOV_DEG:
        return "scale=3840:2160:flags=lanczos"
    crop_w, crop_h = fov_to_crop_dims(fov_deg)
    return (
        f"crop={crop_w}:{crop_h}:(in_w-{crop_w})/2:(in_h-{crop_h})/2,"
        f"scale=3840:2160:flags=lanczos"
    )


def build_fov_ffmpeg_cmd(
    input_path: Path,
    output_path: Path,
    fov_deg: int,
    nice: int | None = None,
    ionice: int | None = None,
) -> list[str]:
    """Construct the ffmpeg argv list for one FOV variant.

    Encodes the canonical recipe from spec §8.3:
        libx265 CRF 18, preset slow, no audio, lanczos scaling, center crop
        for fov<70, no upscaling.

    Optional `nice` / `ionice` prefixes are prepended in the order
    `nice -n N ionice -c N ffmpeg …` to match the overnight prep agent.
    """
    if fov_deg not in ALLOWED_FOVS:
        raise ValueError(
            f"fov_deg must be one of {ALLOWED_FOVS}; got {fov_deg}"
        )
    prefix: list[str] = []
    if nice is not None:
        prefix += ["nice", "-n", str(nice)]
    if ionice is not None:
        prefix += ["ionice", "-c", str(ionice)]
    return prefix + [
        "ffmpeg", "-y",
        "-i", str(input_path),
        "-vf", _vf_filter_for_fov(fov_deg),
        "-c:v", "libx265", "-crf", "18", "-preset", "slow",
        "-an",
        str(output_path),
    ]


def _parse_fov_list(s: str) -> list[int]:
    """Parse `--emit-fov 70,60,50` -> [70, 60, 50]. Validates the FOV set."""
    allowed = set(ALLOWED_FOVS)
    try:
        out = [int(x) for x in s.split(",") if x.strip()]
    except ValueError as e:
        raise argparse.ArgumentTypeError(
            f"--emit-fov must be comma-separated integers; got {s!r}"
        ) from e
    if not out:
        raise argparse.ArgumentTypeError(
            f"--emit-fov requires at least one value; got {s!r}"
        )
    bad = [v for v in out if v not in allowed]
    if bad:
        raise argparse.ArgumentTypeError(
            f"--emit-fov values must be in {sorted(allowed)}; got {bad}"
        )
    return out


_SHA_CHUNK = 1024 * 1024  # 1 MiB


def sha256_of_file(path: Path) -> str:
    """Return the SHA-256 hex digest of `path`'s bytes (read in 1 MiB chunks)."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(_SHA_CHUNK), b""):
            h.update(chunk)
    return h.hexdigest()


def _read_manifest_or_empty(manifest: Path) -> list[dict]:
    """Load the manifest JSON or return []. Raises if the file isn't a list."""
    if not manifest.is_file():
        return []
    raw = manifest.read_text()
    if not raw.strip():
        return []
    data = json.loads(raw)
    if not isinstance(data, list):
        raise ValueError(
            f"manifest at {manifest} is not a JSON array; refusing to overwrite"
        )
    return data


def append_manifest_record(manifest: Path, record: dict) -> None:
    """Append `record` to the manifest list, writing atomically.

    Atomicity: write the full updated list to a sibling temp file, fsync,
    then os.replace() it onto the manifest path. Any crash mid-run leaves
    the old manifest intact.
    """
    manifest.parent.mkdir(parents=True, exist_ok=True)
    entries = _read_manifest_or_empty(manifest)
    entries.append(record)
    tmp = manifest.with_name(f".{manifest.name}.tmp.{os.getpid()}")
    serialized = json.dumps(entries, indent=2)
    with open(tmp, "w") as f:
        f.write(serialized)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, manifest)


def _resolve_manifest_path(output: Path, override: Path | None) -> Path:
    """Default manifest is fov_variants_manifest.json next to the output."""
    if override is not None:
        return override
    return output.parent / "fov_variants_manifest.json"


def _emit_fov_variant(
    src_4k_input: Path,
    fov_deg: int,
    manifest: Path,
    nice: int | None,
    ionice: int | None,
    force: bool,
) -> Path:
    """Produce one FOV variant + append to the manifest. Return output path."""
    stem = src_4k_input.stem
    output = src_4k_input.parent / f"{stem}__fov{fov_deg}.mp4"

    if output.is_file() and not force:
        existing = _read_manifest_or_empty(manifest)
        if any(e.get("output") == output.name for e in existing):
            print(f"  fov{fov_deg}: cached {output.name}; manifest already records it; skipping")
            return output
        print(f"  fov{fov_deg}: cached {output.name}; recording into manifest")
    else:
        argv = build_fov_ffmpeg_cmd(
            input_path=src_4k_input,
            output_path=output,
            fov_deg=fov_deg,
            nice=nice,
            ionice=ionice,
        )
        proc = run(argv)
        if proc.returncode != 0:
            print(
                f"ERROR: ffmpeg exited with code {proc.returncode} "
                f"for fov{fov_deg}", file=sys.stderr,
            )
            sys.exit(1)

    sha = sha256_of_file(output)
    record = {
        "input": src_4k_input.name,
        "variant": f"fov{fov_deg}",
        "output": output.name,
        "output_bytes": output.stat().st_size,
        "sha256": sha,
        "ffmpeg_cmd": " ".join(
            build_fov_ffmpeg_cmd(src_4k_input, output, fov_deg, nice, ionice)
        ),
    }
    append_manifest_record(manifest, record)
    return output


_OVERNIGHT_REQUIRED_KEYS = {
    "input", "variant", "output", "output_bytes", "sha256", "ffmpeg_cmd",
}
_OVERNIGHT_ALLOWED_VARIANTS = {"fov70", "fov60", "fov50"}


def import_overnight_manifest(manifest: Path, verify_sha: bool = True) -> list[dict]:
    """Load and validate the overnight FOV manifest at `manifest`.

    Raises ValueError on:
      - file not a JSON list at top level
      - missing required keys in any record
      - variant value not in {fov70, fov60, fov50}
      - (when verify_sha=True) recomputed SHA-256 doesn't match the recorded one
      - (when verify_sha=True) output file missing or wrong byte count

    Returns the parsed list of records on success.
    """
    if not manifest.is_file():
        raise FileNotFoundError(f"manifest not found: {manifest}")
    data = json.loads(manifest.read_text())
    if not isinstance(data, list):
        raise ValueError(f"manifest at {manifest} is not a JSON array")
    for i, rec in enumerate(data):
        if not isinstance(rec, dict):
            raise ValueError(f"manifest[{i}] is not an object: {rec!r}")
        missing = _OVERNIGHT_REQUIRED_KEYS - rec.keys()
        if missing:
            raise ValueError(
                f"manifest[{i}] missing keys {sorted(missing)}; got {sorted(rec.keys())}"
            )
        if rec["variant"] not in _OVERNIGHT_ALLOWED_VARIANTS:
            raise ValueError(
                f"manifest[{i}] variant {rec['variant']!r} not in "
                f"{sorted(_OVERNIGHT_ALLOWED_VARIANTS)}"
            )
        if not isinstance(rec["sha256"], str) or len(rec["sha256"]) != 64:
            raise ValueError(
                f"manifest[{i}] sha256 must be a 64-char hex string; got {rec['sha256']!r}"
            )
    if verify_sha:
        for rec in data:
            out = manifest.parent / rec["output"]
            if not out.is_file():
                raise ValueError(f"output file not found: {out}")
            actual_bytes = out.stat().st_size
            if actual_bytes != rec["output_bytes"]:
                raise ValueError(
                    f"{out.name}: output_bytes mismatch (recorded "
                    f"{rec['output_bytes']}, actual {actual_bytes})"
                )
            actual_sha = sha256_of_file(out)
            if actual_sha != rec["sha256"]:
                raise ValueError(
                    f"{out.name}: sha256 mismatch (recorded {rec['sha256']}, "
                    f"actual {actual_sha})"
                )
    return data


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the argparse parser for prepare_video.

    Factored out of `main()` so unit tests can exercise the CLI surface
    without invoking ffmpeg.
    """
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("input", type=Path, help="Source video file.")
    ap.add_argument("--output", type=Path, default=None,
                    help="Explicit output path (default: <stem>_prepared<ext> "
                         "next to the input).")
    ap.add_argument("--force", action="store_true",
                    help="Re-encode even if cached output is newer.")
    ap.add_argument("--copy", action="store_true",
                    help="Copy unmodified input to the output path even when "
                         "no re-encode is needed.")
    ap.add_argument("--verify", action="store_true",
                    help="After the operation, verify output is landscape "
                         "via ffprobe + a one-frame gst decode.")
    ap.add_argument("--emit-fov", type=_parse_fov_list, default=None,
                    metavar="LIST",
                    help="Comma-separated list of FOV variants to emit "
                         "from the rotation-stripped output. Valid values: "
                         "70, 60, 50. Example: --emit-fov 70,60,50")
    ap.add_argument("--nice", type=int, default=None,
                    help="If set, prefix ffmpeg with 'nice -n <N>'.")
    ap.add_argument("--ionice", type=int, default=None,
                    help="If set, prefix ffmpeg with 'ionice -c <N>'.")
    ap.add_argument("--manifest", type=Path, default=None,
                    help="Path to the FOV-variants manifest JSON. "
                         "Default: fov_variants_manifest.json in the "
                         "output's parent directory.")
    return ap


def main() -> None:
    args = build_arg_parser().parse_args()

    if not args.input.is_file():
        print(f"ERROR: input not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    probe = ffprobe_json(args.input)
    stream = video_stream(probe)
    w = int(stream["width"])
    h = int(stream["height"])
    codec = stream.get("codec_name", "?")
    nb_frames = stream.get("nb_frames", "?")
    rot = detect_rotation(stream)
    print(f"input: {args.input}")
    print(f"  codec={codec}, dims={w}x{h}, frames={nb_frames}, rotation={rot}")

    output = args.output or default_output(args.input)

    if rot == 0:
        if args.copy:
            print(f"video has no rotation metadata; copying to: {output}")
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(args.input, output)
            if args.verify:
                verify_output(output)
            if not args.emit_fov:
                sys.exit(0)
        else:
            print(f"video is already landscape-oriented; using directly: "
                  f"{args.input}")
            if args.verify:
                verify_output(args.input)
            if not args.emit_fov:
                sys.exit(0)
    else:
        # Rotation present — re-encode unless cache is fresh.
        if (output.is_file() and not args.force
                and output.stat().st_mtime > args.input.stat().st_mtime):
            print(f"cached output up to date: {output}")
            if args.verify:
                verify_output(output)
        else:
            print(f"rotation={rot} detected; re-encoding to: {output}")
            output.parent.mkdir(parents=True, exist_ok=True)
            reencode(args.input, output)
            if args.verify:
                verify_output(output)
            print(f"done: {output}")

    # --- FOV emulation (Plan 3) ------------------------------------------
    if args.emit_fov:
        # The 4K source for FOV emission: rotation-stripped output if rot!=0,
        # else the original input (no-rotation branch fell through to here).
        src_4k = output if rot != 0 else args.input
        manifest_path = _resolve_manifest_path(src_4k, args.manifest)
        print(f"emitting FOV variants: {args.emit_fov}")
        print(f"  source: {src_4k}")
        print(f"  manifest: {manifest_path}")
        for fov_deg in args.emit_fov:
            _emit_fov_variant(
                src_4k_input=src_4k,
                fov_deg=fov_deg,
                manifest=manifest_path,
                nice=args.nice,
                ionice=args.ionice,
                force=args.force,
            )
        print(f"done: {len(args.emit_fov)} FOV variant(s) emitted")
    sys.exit(0)


if __name__ == "__main__":
    main()
