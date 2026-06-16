from __future__ import annotations

__all__ = ["DetectionPersistence"]


class DetectionPersistence:
    """Per-cell latest-detection cache for striped dense tiling.

    Each dense grid cell keeps the most recent detections whose bbox center
    falls inside it. A cell refreshes only when its stripe is re-run (every K
    frames); between refreshes its detections persist, so the visualizer draws
    stable boxes. `published()` returns the union across all cells.

    Detections are visualizer-schema dicts:
        {"label": str, "confidence": float, "bbox": [x, y, w, h]}  (normalized)
    """

    def __init__(self, dense_grid: tuple = (8, 6)):
        self.gx, self.gy = dense_grid
        self._cells: dict[int, list[dict]] = {}

    def cell_of(self, det: dict) -> int:
        x, y, w, h = det["bbox"]
        cx = x + w / 2.0
        cy = y + h / 2.0
        i = min(self.gx - 1, max(0, int(cx * self.gx)))
        j = min(self.gy - 1, max(0, int(cy * self.gy)))
        return j * self.gx + i

    def update(self, run_cells, dets) -> None:
        """Refresh `run_cells`: clear them, then refill from `dets` that land
        in them. Cells not in `run_cells` are left untouched (persist).
        Stores a COPY of each detection with `age` stamped to 0."""
        run = set(run_cells)
        for c in run:
            self._cells[c] = []
        for d in dets:
            c = self.cell_of(d)
            if c in run:
                self._cells[c].append({**d, "age": 0})

    def tick(self) -> None:
        """Advance one frame: age every carried-forward detection."""
        for ds in self._cells.values():
            for d in ds:
                d["age"] = d.get("age", 0) + 1

    def published(self) -> list[dict]:
        out: list[dict] = []
        for ds in self._cells.values():
            out.extend(ds)
        return out
