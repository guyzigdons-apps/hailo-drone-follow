#!/usr/bin/env python3
"""Dev server for the packaged site with sane cache headers.

`python3 -m http.server` sends no Cache-Control, so browsers heuristically
cache JS/CSS and show a stale UI after repackaging. This wrapper sends
`no-cache` for site files (browser revalidates every load) while letting
videos cache normally (they're immutable per packaging run and big).

Usage:
    python3 serve.py [--port 8123] [--dir dist]
"""
from __future__ import annotations

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        if not self.path.startswith("/data/videos/"):
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, *args):  # quiet
        pass


def main() -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8123)
    ap.add_argument("--dir", default=str(here / "dist"))
    args = ap.parse_args()
    handler = partial(NoCacheHandler, directory=args.dir)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    print(f"serving {args.dir} on http://localhost:{args.port} (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
