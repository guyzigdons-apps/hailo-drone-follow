from __future__ import annotations

from collections import deque


class BudgetMeter:
    """Sliding-window tile-inference accounting.

    Enforces an average spend of `budget_inf_per_s` tile-inferences per second
    over a trailing `window_s` window, while allowing short bursts (e.g. a
    recovery spike) as long as the windowed total stays under the cap.
    """

    def __init__(self, budget_inf_per_s: float, fps: float, window_s: float = 1.0):
        self.budget_inf_per_s = float(budget_inf_per_s)
        self.fps = float(fps)
        self.window_frames = max(1, int(round(window_s * fps)))
        self.window_cap = self.budget_inf_per_s * window_s
        self._spent: deque[tuple[int, int]] = deque()  # (frame_idx, n_tiles)

    def _evict(self, frame_idx: int) -> None:
        lo = frame_idx - self.window_frames
        while self._spent and self._spent[0][0] <= lo:
            self._spent.popleft()

    def _window_total(self, frame_idx: int) -> int:
        self._evict(frame_idx)
        return sum(n for _, n in self._spent)

    def charge(self, n_tiles: int, frame_idx: int) -> None:
        self._evict(frame_idx)
        if n_tiles > 0:
            self._spent.append((frame_idx, int(n_tiles)))

    def available(self, frame_idx: int) -> float:
        """Tiles that may be spent on `frame_idx` to keep the window <= cap.

        Returns the per-frame share of remaining budget: how many tiles this
        frame can run while keeping the windowed average <= budget_inf_per_s.
        """
        remaining = max(0.0, self.window_cap - self._window_total(frame_idx))
        return remaining / self.window_frames
