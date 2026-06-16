from __future__ import annotations

__all__ = ["DetectionPersistence"]


class DetectionPersistence:
    """Detection-driven per-cell carry-forward cache for striped dense tiling.

    Each dense grid cell keeps the most recent detection(s) whose bbox centre
    falls inside it. The cache is **detection-driven**: every detection that
    arrives refreshes its own cell (``cell_of``), and detections age out after
    ``ttl`` frames. Between refreshes a cell's detections persist, so the
    visualizer draws stable boxes ("saved until the tile's next iteration" — the
    next sweep that lands a fresh detection in the cell, or TTL expiry).

    Why detection-driven and not schedule-driven: in the live pipeline a tile's
    detection arrives at the probe SEVERAL frames after the tile was inferred
    (decode + crop + infer + aggregate + queue latency). Refilling "the cells
    swept this frame" therefore drops nearly every dense detection — the
    detection lands in a cell that is no longer the one being swept. Keying off
    where each detection actually falls (its own cell) is latency-robust.

    Detections are visualizer-schema dicts:
        {"label": str, "confidence": float, "bbox": [x, y, w, h]}  (normalized)
    plus an injected ``age`` (frames since this detection was stored).
    """

    def __init__(self, dense_grid: tuple = (8, 6), ttl: int | None = None):
        self.gx, self.gy = dense_grid
        self.ttl = ttl              # frames; None = never expire
        self._cells: dict[int, list[dict]] = {}

    def cell_of(self, det: dict) -> int:
        x, y, w, h = det["bbox"]
        cx = x + w / 2.0
        cy = y + h / 2.0
        i = min(self.gx - 1, max(0, int(cx * self.gx)))
        j = min(self.gy - 1, max(0, int(cy * self.gy)))
        return j * self.gx + i

    def update(self, dets) -> None:
        """Detection-driven refresh: replace the contents of each cell that
        received a detection this frame with that detection (age 0). Cells with
        no incoming detection keep their existing (aging) contents. Stores a COPY
        so the caller's dicts are never mutated."""
        by_cell: dict[int, list[dict]] = {}
        for d in dets:
            by_cell.setdefault(self.cell_of(d), []).append({**d, "age": 0})
        for c, ds in by_cell.items():
            self._cells[c] = ds

    def tick(self) -> None:
        """Advance one frame: age every carried-forward detection and drop any
        that have exceeded ``ttl`` (a departed object whose cell stopped being
        refreshed)."""
        for c, ds in list(self._cells.items()):
            for d in ds:
                d["age"] = d.get("age", 0) + 1
            if self.ttl is not None:
                ds = [d for d in ds if d["age"] <= self.ttl]
                if ds:
                    self._cells[c] = ds
                else:
                    del self._cells[c]

    def published(self) -> list[dict]:
        out: list[dict] = []
        for ds in self._cells.values():
            out.extend(ds)
        return out
