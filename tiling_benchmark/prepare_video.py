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
  python tiling_benchmark/prepare_video.py /path/to/source.MP4
  python tiling_benchmark/prepare_video.py source.MP4 --output prepared.MP4
  python tiling_benchmark/prepare_video.py source.MP4 --force --verify
"""

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


# 6K source dimensions (DJI Mavic 4 Pro main camera; spec §8.1)
SRC_6K_W = 6016
SRC_6K_H = 3384
SRC_NATIVE_FOV_DEG = 70

# Allowed FOV variants. Anything outside this set would require upscaling
# from the 6K source, which is disallowed (spec §8.2).
ALLOWED_FOVS = (70, 60, 50)


def fov_to_crop_dims(fov_deg: int) -> tuple[int, int]:
    """Return the (crop_w, crop_h) needed to emulate `fov_deg` from a 6K source.

    Derived from `crop_ratio = tan(fov_deg/2) / tan(70°/2)` per spec §8.2.
    `crop_h` is the smallest even integer >= `SRC_6K_H * crop_ratio` (h264/h265
    encoders require even dims), and `crop_w` is derived from `crop_h` so the
    source 16:9 aspect ratio is preserved exactly (`int(crop_h * 16/9)`).

    The published spec §8.2 table values fall out of this construction:
        FOV-70: (6016, 3384)
        FOV-60: (4963, 2792)
        FOV-50: (4007, 2254)

    Verifies the result is <= source dims and >= 4K output dims (3840x2160);
    raises ValueError on either violation (we never upscale).
    """
    if fov_deg not in ALLOWED_FOVS:
        raise ValueError(
            f"fov_deg must be one of {ALLOWED_FOVS}; got {fov_deg}"
        )
    ratio = (
        math.tan(math.radians(fov_deg) / 2.0)
        / math.tan(math.radians(SRC_NATIVE_FOV_DEG) / 2.0)
    )
    # Round crop_h up to the nearest even integer (encoder-friendly), then
    # derive crop_w from crop_h to preserve the 16:9 source aspect ratio.
    crop_h = math.ceil(SRC_6K_H * ratio / 2.0) * 2
    crop_w = int(crop_h * SRC_6K_W / SRC_6K_H)
    if crop_w > SRC_6K_W or crop_h > SRC_6K_H:
        raise ValueError(
            f"fov_deg={fov_deg} requires crop {crop_w}x{crop_h} > source "
            f"{SRC_6K_W}x{SRC_6K_H}; refusing to upscale"
        )
    if crop_w < 3840 or crop_h < 2160:
        raise ValueError(
            f"fov_deg={fov_deg} requires crop {crop_w}x{crop_h} < 4K output "
            f"3840x2160; refusing to upscale"
        )
    return crop_w, crop_h


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
            sys.exit(0)
        print(f"video is already landscape-oriented; using directly: "
              f"{args.input}")
        if args.verify:
            verify_output(args.input)
        sys.exit(0)

    # Rotation present — re-encode unless cache is fresh.
    if (output.is_file() and not args.force
            and output.stat().st_mtime > args.input.stat().st_mtime):
        print(f"cached output up to date: {output}")
        if args.verify:
            verify_output(output)
        sys.exit(0)

    print(f"rotation={rot} detected; re-encoding to: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    reencode(args.input, output)
    if args.verify:
        verify_output(output)
    print(f"done: {output}")
    sys.exit(0)


if __name__ == "__main__":
    main()
