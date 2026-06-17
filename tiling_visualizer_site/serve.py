#!/usr/bin/env python3
"""Dev server for the packaged site with sane cache headers + Range support.

Two things stock `python3 -m http.server` gets wrong for this tool:

1. It sends no Cache-Control, so browsers heuristically cache JS/CSS and show a
   stale UI after repackaging. This wrapper sends `no-cache` for site files
   (browser revalidates every load) while letting videos cache normally
   (immutable per packaging run, and big).

2. SimpleHTTPRequestHandler ignores HTTP `Range` requests — it always replies
   `200` with the whole file. The browser then cannot fetch an arbitrary byte
   offset, so seeking/frame-stepping to an un-buffered position fails. We add
   `206 Partial Content` support so video scrubbing works locally (production
   nginx already does this).

Usage:
    python3 serve.py [--port 8123] [--dir dist]
"""
from __future__ import annotations

import argparse
import os
import re
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

_RANGE_RE = re.compile(r"bytes=(\d*)-(\d*)\s*$")


class _RangeFile:
    """Wrap a file object so reads stop after `length` bytes — lets the stock
    do_GET()/copyfile() loop send exactly the requested range, then EOF."""

    def __init__(self, fileobj, length: int):
        self._f = fileobj
        self._remaining = length

    def read(self, amt: int = -1) -> bytes:
        if self._remaining <= 0:
            return b""
        if amt < 0 or amt > self._remaining:
            amt = self._remaining
        data = self._f.read(amt)
        self._remaining -= len(data)
        return data

    def close(self):
        self._f.close()


class NoCacheRangeHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        if not self.path.startswith("/data/videos/"):
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, *args):  # quiet
        pass

    def send_head(self):
        range_header = self.headers.get("Range")
        if not range_header:
            return super().send_head()

        path = self.translate_path(self.path)
        if os.path.isdir(path):
            return super().send_head()
        try:
            f = open(path, "rb")
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND, "File not found")
            return None

        try:
            fs = os.fstat(f.fileno())
            size = fs.st_size
            m = _RANGE_RE.match(range_header.strip())
            if not m or (not m.group(1) and not m.group(2)):
                self.send_error(HTTPStatus.BAD_REQUEST, "Invalid Range")
                f.close()
                return None

            start_s, end_s = m.group(1), m.group(2)
            if start_s == "":                       # suffix: last N bytes
                start = max(0, size - int(end_s))
                end = size - 1
            else:
                start = int(start_s)
                end = int(end_s) if end_s else size - 1
            end = min(end, size - 1)

            if start >= size or start > end:
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header("Content-Range", f"bytes */{size}")
                self.send_header("Content-Length", "0")
                self.end_headers()
                f.close()
                return None

            length = end - start + 1
            f.seek(start)
            self.send_response(HTTPStatus.PARTIAL_CONTENT)
            self.send_header("Content-Type", self.guess_type(path))
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.send_header("Content-Length", str(length))
            self.send_header("Last-Modified", self.date_time_string(fs.st_mtime))
            self.end_headers()
            return _RangeFile(f, length)
        except Exception:
            f.close()
            raise


def main() -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8123)
    ap.add_argument("--dir", default=str(here / "dist"))
    args = ap.parse_args()
    handler = partial(NoCacheRangeHandler, directory=args.dir)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    print(f"serving {args.dir} on http://localhost:{args.port} (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
