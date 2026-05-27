from __future__ import annotations

from dataclasses import dataclass, field

MODEL_W = 640
MODEL_H = 480
MODEL_ASPECT = MODEL_W / MODEL_H  # 4:3


@dataclass(frozen=True)
class CropRect:
    """A source-pixel crop fed to the cropper/HEF. 4:3, height derived from w."""
    x: int
    y: int
    w: int
    h: int
    mode: str = "s"  # cropper per-tile mode tag ("s" single-scale, "m" multi)

    @property
    def scale(self) -> float:
        return MODEL_W / self.w

    @classmethod
    def from_center_width(cls, cx: float, cy: float, crop_w: int,
                          mode: str = "s") -> "CropRect":
        w = int(round(crop_w))
        h = int(round(w / MODEL_ASPECT))
        return cls(x=int(round(cx - w / 2)), y=int(round(cy - h / 2)),
                   w=w, h=h, mode=mode)

    def clamp(self, src_w: int, src_h: int) -> "CropRect":
        w = min(self.w, src_w)
        h = min(self.h, src_h)
        x = max(0, min(src_w - w, self.x))
        y = max(0, min(src_h - h, self.y))
        return CropRect(x=x, y=y, w=w, h=h, mode=self.mode)


@dataclass(frozen=True)
class Det:
    """A detection in NORMALIZED source-frame coords. bbox = (x, y, w, h)."""
    cls: int
    score: float
    x: float
    y: float
    w: float
    h: float

    @property
    def xyxy(self) -> tuple:
        return (self.x, self.y, self.x + self.w, self.y + self.h)


@dataclass
class LockState:
    track_id: int | None = None
    bbox_norm: tuple = (0.0, 0.0, 0.0, 0.0)  # (x,y,w,h) normalized
    status: str = "LOST"                     # TRACKING | SEARCHING | LOST
    frames_since_seen: int = 0
    last_velocity: tuple = (0.0, 0.0)        # (dvx, dvy) of bbox centre, normalized/frame
