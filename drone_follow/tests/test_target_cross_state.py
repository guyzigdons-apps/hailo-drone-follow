"""Unit tests for TargetCrossState — the small thread-safe holder that
bridges the local-branch metadata pad probe (writer) and the cairooverlay
draw callback (reader).

The integration that actually paints the cross on a video buffer needs
GStreamer + cairooverlay + Hailo metadata and is exercised by running the
app; this test pins the contract those two callsites depend on.
"""

from __future__ import annotations

import threading

from drone_follow.pipeline_adapter.vision_branches import TargetCrossState


def test_initial_state_is_empty():
    s = TargetCrossState()
    assert s.get() == (None, None, None)


def test_set_then_get_roundtrips_all_three_fields():
    s = TargetCrossState()
    s.set(0.42, 0.66, "LOCKED")
    assert s.get() == (0.42, 0.66, "LOCKED")


def test_clear_resets_to_initial():
    s = TargetCrossState()
    s.set(0.1, 0.2, "AUTO")
    s.clear()
    assert s.get() == (None, None, None)


def test_set_with_none_drops_the_cross():
    """Pad probe path: target disappears from frame ⇒ set(None, None, None)
    so the next draw callback skips the cross."""
    s = TargetCrossState()
    s.set(0.5, 0.5, "LOCKED")
    s.set(None, None, None)
    assert s.get() == (None, None, None)


def test_concurrent_writers_and_readers_dont_tear():
    """The state crosses thread boundaries: pad-probe runs on the streaming
    thread, draw on the cairooverlay's drawing thread. A torn read (e.g.
    new cx with old cy) would draw the cross at a phantom location.
    """
    s = TargetCrossState()
    stop = threading.Event()

    def writer():
        i = 0
        while not stop.is_set():
            v = 0.1 + (i % 9) * 0.1     # 0.1..0.9
            s.set(v, v, "LOCKED" if (i & 1) else "AUTO")
            i += 1

    seen_tears = 0
    def reader():
        nonlocal seen_tears
        for _ in range(100_000):
            cx, cy, mode = s.get()
            # Writer always sets cx == cy. A torn read would be cx != cy.
            if cx is not None and cx != cy:
                seen_tears += 1

    writers = [threading.Thread(target=writer) for _ in range(3)]
    readers = [threading.Thread(target=reader) for _ in range(3)]
    for t in writers + readers:
        t.start()
    for t in readers:
        t.join()
    stop.set()
    for t in writers:
        t.join()

    assert seen_tears == 0, (
        f"observed {seen_tears} torn reads — the lock isn't covering all "
        f"three fields atomically"
    )
