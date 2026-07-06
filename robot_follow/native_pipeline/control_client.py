"""ZMQ REQ client for the C++ binary's runtime tile-control socket.

Pairs with ``control_server.cpp`` in this directory. The C++ side binds a
REP socket on ``tcp://*:<control_port>`` (default 7001) and accepts JSON
``set_tiles`` / ``get_tiles`` requests.

Why a separate module: the orchestrator's tile-apply path needs to try a
hot-swap first and fall back to the full restart when the C++ binary
isn't reachable (e.g. on first run, or while it's mid-restart). Keeping
the client small and self-contained lets the fallback live in the
caller (drone_follow_h15.py) without dragging ZMQ semantics in there.
"""

from __future__ import annotations

import json
import logging
from typing import Optional

import zmq

LOGGER = logging.getLogger(__name__)

DEFAULT_ENDPOINT = "tcp://127.0.0.1:7001"
# Hot-swap typically takes a few ms (mutex grab + atomic-equivalent
# shared_ptr swap), but allow a generous budget so the call doesn't
# spuriously fail under load or right after the C++ binary started.
DEFAULT_TIMEOUT_MS = 1500


class TileControlError(Exception):
    """Raised when the C++ control server returns an error or times out."""


def set_tiles(
    tiles: list,
    *,
    endpoint: str = DEFAULT_ENDPOINT,
    timeout_ms: int = DEFAULT_TIMEOUT_MS,
) -> list:
    """Hot-swap the C++ pipeline's tile geometry.

    ``tiles`` is the orchestrator's canonical list of ``(name, x, y, w, h)``
    tuples (subscriber-format). The wire format strips the name — the C++
    side doesn't carry it.

    Returns the active tile list as reported by the server (list of
    ``(x, y, w, h)`` tuples) on success. Raises ``TileControlError`` on
    timeout / refused / framework error.
    """
    payload = {
        "cmd": "set_tiles",
        "tiles": [[float(x), float(y), float(w), float(h)]
                  for (_n, x, y, w, h) in tiles],
    }
    response = _request(payload, endpoint=endpoint, timeout_ms=timeout_ms)
    if not response.get("ok"):
        raise TileControlError(response.get("error", "set_tiles failed"))
    return response.get("tiles", [])


def get_tiles(
    *,
    endpoint: str = DEFAULT_ENDPOINT,
    timeout_ms: int = DEFAULT_TIMEOUT_MS,
) -> list:
    """Read the C++ pipeline's current tile geometry."""
    response = _request({"cmd": "get_tiles"},
                        endpoint=endpoint, timeout_ms=timeout_ms)
    if not response.get("ok"):
        raise TileControlError(response.get("error", "get_tiles failed"))
    return response.get("tiles", [])


def _request(payload: dict, *, endpoint: str, timeout_ms: int) -> dict:
    """One-shot REQ/REP exchange. The socket is created and torn down per
    call — the cost is negligible relative to the swap latency, and it
    sidesteps REQ's strict alternating-state requirement (one stale recv
    from a prior failed call would deadlock a long-lived socket)."""
    ctx: Optional[zmq.Context] = None
    sock: Optional[zmq.Socket] = None
    try:
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
        sock.connect(endpoint)
        sock.send_string(json.dumps(payload))
        reply = sock.recv_string()
        return json.loads(reply)
    except zmq.Again as e:
        raise TileControlError(f"control server timeout ({timeout_ms} ms)") from e
    except zmq.ZMQError as e:
        raise TileControlError(f"control socket error: {e}") from e
    finally:
        if sock is not None:
            sock.close(linger=0)
