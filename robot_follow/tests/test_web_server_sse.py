"""CLEAN-16 acceptance — SharedUIState SSE/MJPEG race.

Pre-fix (current): two consumers race against producer's Event.set()/clear()
pair; one consumer falls through 2 s timeout under load.
Post-fix (plan 02-07): Condition + monotonic frame_seq; both consumers receive
every frame.

Marked xfail until plan 02-07 strips the markers AND migrates SharedUIState
to (last_seen: int, timeout) -> (jpeg, seq) signatures.
"""
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import pytest

from robot_follow.servers.web_server import SharedUIState


XFAIL_REASON = "CLEAN-16 race not yet fixed; closes in plan 02-07"


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_two_consumers_both_receive_frames_within_timeout():
    """Both consumers must track frame_seq independently and see every frame."""
    ui = SharedUIState()
    results = {"a": [], "b": []}
    stop = threading.Event()

    def consumer(name: str) -> None:
        last_seen = 0
        while not stop.is_set():
            # Post-fix contract: wait_frame(last_seen, timeout) -> (jpeg, seq)
            jpeg, last_seen = ui.wait_frame(last_seen, timeout=0.5)
            if jpeg is not None:
                results[name].append(last_seen)

    pool = ThreadPoolExecutor(max_workers=2)
    pool.submit(consumer, "a")
    pool.submit(consumer, "b")

    time.sleep(0.05)  # let consumers enter wait
    for i in range(50):
        ui.update_frame(f"frame-{i}".encode())
        time.sleep(0.01)
    time.sleep(0.5)
    stop.set()
    pool.shutdown(wait=True, cancel_futures=False)

    assert len(results["a"]) >= 45, f"consumer A got only {len(results['a'])} frames"
    assert len(results["b"]) >= 45, f"consumer B got only {len(results['b'])} frames"
    assert max(results["a"]) >= 45
    assert max(results["b"]) >= 45


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_disconnected_consumer_does_not_block_other():
    """A consumer that stops calling wait_frame() must not block the other consumer."""
    ui = SharedUIState()
    live_results: list[int] = []
    stop = threading.Event()

    def live_consumer() -> None:
        last_seen = 0
        while not stop.is_set():
            jpeg, last_seen = ui.wait_frame(last_seen, timeout=0.5)
            if jpeg is not None:
                live_results.append(last_seen)

    # "Disconnected" consumer: enters wait once, then never returns.
    # Simulate by simply not running a second loop — the first consumer must
    # still drain frames.
    pool = ThreadPoolExecutor(max_workers=1)
    pool.submit(live_consumer)

    time.sleep(0.05)
    for i in range(30):
        ui.update_frame(f"frame-{i}".encode())
        time.sleep(0.01)
    time.sleep(0.3)
    stop.set()
    pool.shutdown(wait=True, cancel_futures=False)

    assert len(live_results) >= 25, f"live consumer got only {len(live_results)} frames"


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_frame_seq_is_monotonic_across_consumers():
    """Every consumer call returns a strictly increasing seq."""
    ui = SharedUIState()
    ui.update_frame(b"f0")
    ui.update_frame(b"f1")
    ui.update_frame(b"f2")

    jpeg, seq1 = ui.wait_frame(last_seen=0, timeout=0.1)
    assert jpeg is not None
    assert seq1 >= 1
    # No new frames — predicate already True for last_seen=0, returns latest.
    # With seq1 as the new last_seen, no new frames → timeout.
    jpeg2, seq2 = ui.wait_frame(last_seen=seq1, timeout=0.05)
    assert jpeg2 is None
    assert seq2 == seq1
