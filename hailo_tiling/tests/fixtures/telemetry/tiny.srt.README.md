# tiny.srt — DJI SRT fixture

A ~9 KB trimmed copy of a real DJI sidecar SRT, used by
`hailo_tiling.tests.test_telemetry_srt` (Plan 7 Task 3).

## Provenance

Source: `/home/giladn/Videos/Drone/Training/DJI_20260430104732_0012_D.SRT`
(277 KB, 919 frame blocks, ~30 fps DJI Mini-class clip).

## Build procedure

Trimmed to the first 30 blocks (~1 second at 30 fps) and anonymised the
GPS coordinates by multiplying `latitude` and `longitude` by `0.001`. The
precision pattern is preserved (6 decimal places) but the location is no
longer identifying. All other fields (`abs_alt`, `rel_alt`, `focal_len`,
`iso`, `shutter`, `ev`, `ct`, `tint`, …) are copied verbatim.

Reproduce with:

```python
import re
from pathlib import Path

src = Path("DJI_20260430104732_0012_D.SRT")
text = src.read_text(encoding="utf-8-sig").replace("\r\n", "\n")
blocks = [b for b in re.split(r"\n\s*\n", text) if b.strip()]

def anon(block):
    block = re.sub(r"\[latitude:\s*([-\d.]+)\]",
                   lambda m: f"[latitude: {float(m.group(1)) * 0.001:.6f}]",
                   block)
    block = re.sub(r"\[longitude:\s*([-\d.]+)\]",
                   lambda m: f"[longitude: {float(m.group(1)) * 0.001:.6f}]",
                   block)
    return block

Path("tiny.srt").write_text(
    "\n\n".join(anon(b) for b in blocks[:30]) + "\n",
    encoding="utf-8",
)
```

## Properties

- 30 frame blocks, ~1 second of flight at ~30 fps.
- Each block contains: FrameCnt, ISO timestamp, and bracketed key-value
  pairs including `latitude`, `longitude`, `rel_alt`, `abs_alt`,
  `focal_len`, `iso`, `shutter`, `fnum`, `ev`, `color_md`, `ct`, `tint`.
- `<font size="28">…</font>` HTML wrappers are present and must be
  stripped transparently by the parser.
- GPS: `latitude ≈ 0.031884`, `longitude ≈ 0.035027` (anonymised; real
  origin is in Israel — `0.001 × 31.883741`).
