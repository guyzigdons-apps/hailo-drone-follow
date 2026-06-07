"""Low-level HailoRT runtime for hailo_tiling's HefBackend.

Owns the chip-touching pieces — ``HefHandle`` (VDevice + configured InferModel
for one HEF) and ``decode_nms_output`` (NMS_BY_CLASS -> Detection list) — that
``hailo_tiling.backends.hef.HefBackend`` lazy-imports so the package stays
chip-free at import time. (Moved here from ``tiling_benchmark/probe_phantom_hef.py``.)

Also runnable standalone as a phantom-detection probe: loads a HEF directly via
`hailo_platform` (no GStreamer, no post-process .so), feeds a set of controlled
inputs, and dumps the raw NMS-by-class output. Helps isolate whether a phantom
(0,0,1,1) class-0 detection originates inside the HEF itself or somewhere in the
GStreamer / yolo_hailortpp_postprocess plumbing.

Usage:
    python -m hailo_tiling.backends.hef_runtime \
        --hef /usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef \
        [--real-frame /path/to/video.mp4 --real-frame-n 100]
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

try:
    from hailo_platform import HEF, FormatType, HailoSchedulingAlgorithm, VDevice
except Exception as exc:  # pragma: no cover
    print(f"ERROR: hailo_platform import failed: {exc}", file=sys.stderr)
    print("Did you `source setup_env.sh` first?", file=sys.stderr)
    sys.exit(2)


# 4-class custom HEF: person/vehicle/face/license_plate. HEF class 0 == person.
LABELS_4CLASS = ["person", "vehicle", "face", "license_plate"]
# COCO labels — only first 5 matter per spec.
LABELS_COCO_PREFIX = ["person", "bicycle", "car", "motorcycle", "airplane"]


def label_for(hef_basename: str, cls_idx: int) -> str:
    name = hef_basename.lower()
    if "4_classes" in name:
        if 0 <= cls_idx < len(LABELS_4CLASS):
            return LABELS_4CLASS[cls_idx]
        return f"cls{cls_idx}"
    if 0 <= cls_idx < len(LABELS_COCO_PREFIX):
        return LABELS_COCO_PREFIX[cls_idx]
    return f"cls{cls_idx}"


@dataclass
class HefHandle:
    """Owns a VDevice + configured InferModel for one HEF."""

    hef_path: str
    vdevice: VDevice
    hef: HEF
    infer_model: object
    config_ctx: object
    configured_model: object
    input_name: str
    output_name: str
    input_h: int
    input_w: int
    input_c: int
    output_shape: tuple

    @classmethod
    def open(
        cls,
        hef_path: str,
        nms_score_threshold: float = 0.05,
        vdevice_params=None,
    ) -> "HefHandle":
        if vdevice_params is None:
            vdevice_params = VDevice.create_params()
            vdevice_params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        vd = VDevice(vdevice_params)
        hef = HEF(hef_path)
        im = vd.create_infer_model(hef_path)
        im.set_batch_size(1)
        im.input().set_format_type(FormatType.UINT8)
        for out_info in hef.get_output_vstream_infos():
            im.output(out_info.name).set_format_type(FormatType.FLOAT32)
            # Lower NMS threshold so we can SEE phantom-level (0.5-0.62) and
            # below detections; default may filter them out entirely.
            try:
                im.output(out_info.name).set_nms_score_threshold(nms_score_threshold)
            except Exception:
                pass

        in_info = hef.get_input_vstream_infos()[0]
        out_info = hef.get_output_vstream_infos()[0]
        shape = in_info.shape
        if len(shape) == 4:
            h, w, c = shape[1], shape[2], shape[3]
        else:
            h, w, c = shape[0], shape[1], shape[2]

        ctx = im.configure()
        cm = ctx.__enter__()
        return cls(
            hef_path=hef_path, vdevice=vd, hef=hef, infer_model=im,
            config_ctx=ctx, configured_model=cm,
            input_name=in_info.name, output_name=out_info.name,
            input_h=h, input_w=w, input_c=c,
            output_shape=tuple(im.output(out_info.name).shape),
        )

    def close(self) -> None:
        try:
            self.config_ctx.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self.vdevice.release()
        except Exception:
            pass

    def infer(self, image_uint8: np.ndarray) -> list[np.ndarray]:
        """Run a single inference. Returns list[ndarray] keyed by class.

        For HAILO_NMS_BY_CLASS each element is a (n_dets, 5) float32 ndarray
        of (y_min, x_min, y_max, x_max, score), or empty if no detections.
        """
        if image_uint8.shape != (self.input_h, self.input_w, self.input_c):
            raise ValueError(
                f"input shape {image_uint8.shape} != HEF expected "
                f"({self.input_h},{self.input_w},{self.input_c})"
            )
        if image_uint8.dtype != np.uint8:
            image_uint8 = image_uint8.astype(np.uint8)

        out_buf = np.empty(self.output_shape, dtype=np.float32)
        binding = self.configured_model.create_bindings(
            output_buffers={self.output_name: out_buf}
        )
        binding.input().set_buffer(image_uint8)
        self.configured_model.wait_for_async_ready(timeout_ms=10000)
        job = self.configured_model.run_async([binding], lambda *a, **k: None)
        job.wait(timeout_ms=10000)
        raw = binding.output(self.output_name).get_buffer()
        if isinstance(raw, np.ndarray):
            return [raw]
        return list(raw)


@dataclass
class Detection:
    cls: int
    x: float
    y: float
    w: float
    h: float
    score: float

    @property
    def area_frac(self) -> float:
        return max(0.0, self.w) * max(0.0, self.h)


def decode_nms_output(nms_out: list[np.ndarray]) -> list[Detection]:
    """Decode the list-of-ndarray NMS_BY_CLASS output into Detection objects."""
    dets: list[Detection] = []
    for cls_idx, arr in enumerate(nms_out):
        if arr is None:
            continue
        arr = np.asarray(arr)
        if arr.size == 0:
            continue
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.shape[-1] < 5:
            continue
        for row in arr:
            y_min = float(row[0]); x_min = float(row[1])
            y_max = float(row[2]); x_max = float(row[3])
            score = float(row[4])
            if score <= 0.0 and y_min == 0.0 and x_min == 0.0 and y_max == 0.0 and x_max == 0.0:
                continue
            dets.append(Detection(
                cls=cls_idx, x=x_min, y=y_min,
                w=x_max - x_min, h=y_max - y_min, score=score,
            ))
    return dets


def gen_inputs(input_h: int, input_w: int, real_frame: Optional[np.ndarray]):
    """Yields (name, image_uint8_HWC) pairs."""
    yield "uniform_gray", np.full((input_h, input_w, 3), 128, dtype=np.uint8)
    yield "uniform_skyblue", np.full((input_h, input_w, 3), [135, 206, 235], dtype=np.uint8)
    yield "uniform_dark", np.full((input_h, input_w, 3), 32, dtype=np.uint8)
    rng = np.random.default_rng(seed=42)
    yield "gaussian_noise", rng.integers(0, 256, (input_h, input_w, 3), dtype=np.uint8)
    if real_frame is not None:
        yield "real_frame", real_frame


def load_real_frame(path: str, frame_n: int, h: int, w: int) -> np.ndarray:
    import cv2
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f"could not open {path}")
    cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, frame_n))
    ok, frame_bgr = cap.read()
    cap.release()
    if not ok or frame_bgr is None:
        raise RuntimeError(f"could not read frame {frame_n} from {path}")
    return cv2.resize(frame_bgr, (w, h), interpolation=cv2.INTER_LINEAR).astype(np.uint8)


@dataclass
class ProbeRow:
    hef_label: str
    input_label: str
    n_dets_ge_05: int
    phantom_cls: Optional[int]
    phantom_label: Optional[str]
    phantom_bbox: Optional[tuple]
    phantom_score: Optional[float]
    top5: list = field(default_factory=list)


PHANTOM_AREA_THRESHOLD = 0.90
PHANTOM_SCORE_THRESHOLD = 0.5


def analyse(hef_label: str, input_label: str, dets: list[Detection]) -> ProbeRow:
    dets_sorted = sorted(dets, key=lambda d: d.score, reverse=True)
    n_ge = sum(1 for d in dets_sorted if d.score >= PHANTOM_SCORE_THRESHOLD)
    phantom: Optional[Detection] = None
    for d in dets_sorted:
        if d.score >= PHANTOM_SCORE_THRESHOLD and d.area_frac >= PHANTOM_AREA_THRESHOLD:
            phantom = d
            break
    return ProbeRow(
        hef_label=hef_label, input_label=input_label, n_dets_ge_05=n_ge,
        phantom_cls=phantom.cls if phantom else None,
        phantom_label=label_for(hef_label, phantom.cls) if phantom else None,
        phantom_bbox=((round(phantom.x, 4), round(phantom.y, 4),
                       round(phantom.w, 4), round(phantom.h, 4)) if phantom else None),
        phantom_score=round(phantom.score, 4) if phantom else None,
        top5=dets_sorted[:5],
    )


def print_top5(hef_label: str, input_label: str, dets: list[Detection]) -> None:
    print(f"    Top-5 detections for {input_label}:")
    if not dets:
        print("      <none>")
        return
    for d in dets[:5]:
        lbl = label_for(hef_label, d.cls)
        print(f"      cls={d.cls:>2} ({lbl:<14}) "
              f"bbox=(x={d.x:.4f}, y={d.y:.4f}, w={d.w:.4f}, h={d.h:.4f}) "
              f"area_frac={d.area_frac:.3f} score={d.score:.4f}")


def print_summary(rows: list[ProbeRow]) -> None:
    print()
    print("=" * 110)
    print("SUMMARY")
    print("=" * 110)
    print("| HEF                            | Input             | n_dets>=0.5 | "
          "phantom_class       | phantom_bbox                              | phantom_score |")
    print("|--------------------------------|-------------------|-------------|"
          "---------------------|-------------------------------------------|---------------|")
    for r in rows:
        if r.phantom_cls is None:
            pcls = "-"; pbbox = "-"; pscore = "-"
        else:
            pcls = f"{r.phantom_cls} ({r.phantom_label})"
            pbbox = str(r.phantom_bbox)
            pscore = f"{r.phantom_score:.4f}"
        print(f"| {r.hef_label:<30} | {r.input_label:<17} | "
              f"{r.n_dets_ge_05:^11} | {pcls:<19} | {pbbox:<41} | {pscore:^13} |")


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    ap.add_argument("--hef", required=True, help="Path to .hef file")
    ap.add_argument("--input-w", type=int, default=None,
                    help="Override HEF input width (rarely needed)")
    ap.add_argument("--input-h", type=int, default=None,
                    help="Override HEF input height (rarely needed)")
    ap.add_argument("--real-frame", default=None,
                    help="Optional positive control: path to a video file with real objects")
    ap.add_argument("--real-frame-n", type=int, default=100,
                    help="Frame index for --real-frame (default 100)")
    ap.add_argument("--nms-score-threshold", type=float, default=0.05,
                    help="On-chip NMS score floor (default 0.05; lower=see "
                         "more weak detections including phantoms)")
    args = ap.parse_args(argv)

    if not os.path.exists(args.hef):
        print(f"ERROR: HEF not found: {args.hef}", file=sys.stderr)
        return 2

    hef_label = os.path.basename(args.hef).replace(".hef", "")
    print(f"\n{'=' * 80}\nHEF: {args.hef}\n{'=' * 80}")

    handle = HefHandle.open(args.hef, nms_score_threshold=args.nms_score_threshold)
    try:
        print("  Input vstreams:")
        for info in handle.hef.get_input_vstream_infos():
            print(f"    name={info.name} shape={info.shape} "
                  f"fmt={info.format.type} order={info.format.order}")
        print("  Output vstreams:")
        for info in handle.hef.get_output_vstream_infos():
            print(f"    name={info.name} shape={info.shape} "
                  f"fmt={info.format.type} order={info.format.order}")
        print(f"  Configured output buffer shape: {handle.output_shape}")

        h = args.input_h or handle.input_h
        w = args.input_w or handle.input_w
        if (h, w) != (handle.input_h, handle.input_w):
            print(f"  WARN: --input-h/--input-w ({h},{w}) != HEF "
                  f"({handle.input_h},{handle.input_w}); using HEF values.")
            h, w = handle.input_h, handle.input_w

        real_frame = None
        if args.real_frame:
            try:
                real_frame = load_real_frame(args.real_frame, args.real_frame_n, h, w)
                print(f"  Loaded real frame #{args.real_frame_n} from "
                      f"{args.real_frame}, resized to ({h},{w})")
            except Exception as e:
                print(f"  WARN: could not load real frame: {e}")

        rows: list[ProbeRow] = []
        for name, img in gen_inputs(h, w, real_frame):
            try:
                nms_out = handle.infer(img)
            except Exception as e:
                print(f"  [{name}] inference failed: {e}")
                rows.append(ProbeRow(hef_label, name, 0, None, None, None, None, []))
                continue
            dets = decode_nms_output(nms_out)
            print()
            print(f"  -- input: {name} --")
            print(f"    total detections (any score): {len(dets)}")
            print_top5(hef_label, name, sorted(dets, key=lambda d: d.score, reverse=True))
            rows.append(analyse(hef_label, name, dets))

        print_summary(rows)
    finally:
        handle.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
