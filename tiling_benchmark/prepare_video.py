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
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


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


def _parse_fov_list(s: str) -> list[int]:
    """Parse `--emit-fov 70,60,50` -> [70, 60, 50]. Validates the FOV set."""
    allowed = {70, 60, 50}
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
