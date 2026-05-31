"""Inject the committed 0026 ablation tables into the technical report's
results section (Plan-15 paper-with-code scaffold; Night-2 Task C2).

This renderer is deliberately dumb and honest: it reads the EXISTING, committed
``dynamic_tiling/runs/ablation_0026_fov{50,60,70}/ablation_table.md`` files and
splices them verbatim between the ``<!-- BEGIN:ablation -->`` /
``<!-- END:ablation -->`` markers in ``docs/paper/technical-report.md``. It
invents NO numbers — if a table file is missing, that FOV is reported as
"not yet generated" rather than fabricated. Re-running is idempotent (the
region between the markers is fully replaced each time).

Usage:
    python scripts/render_ablation_into_report.py            # render in place
    python scripts/render_ablation_into_report.py --check    # CI: fail if stale
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_REPORT = _REPO_ROOT / "docs" / "paper" / "technical-report.md"
_RUNS = _REPO_ROOT / "dynamic_tiling" / "runs"
_FOVS = ("fov50", "fov60", "fov70")

_BEGIN = "<!-- BEGIN:ablation -->"
_END = "<!-- END:ablation -->"


def _table_path(fov: str) -> Path:
    return _RUNS / f"ablation_0026_{fov}" / "ablation_table.md"


def _strip_table_header(md: str) -> str:
    """Drop the leading ``# Ablation table`` H1 from a committed table file so
    it nests under the report's results subsection (keep the bullet metadata +
    the markdown table)."""
    lines = md.splitlines()
    out = [ln for ln in lines if ln.strip() != "# Ablation table"]
    # Collapse a leading blank line left by the removed header.
    while out and not out[0].strip():
        out.pop(0)
    return "\n".join(out).rstrip() + "\n"


def render_block() -> str:
    """Build the results block from whatever committed tables exist."""
    parts: list[str] = []
    for fov in _FOVS:
        p = _table_path(fov)
        parts.append(f"#### Clip 0026 — {fov}\n")
        if p.exists():
            parts.append(_strip_table_header(p.read_text()))
            parts.append(f"\n*Source: `{p.relative_to(_REPO_ROOT)}` (committed).*\n")
        else:
            parts.append(
                f"_Table not yet generated "
                f"(`{p.relative_to(_REPO_ROOT)}` absent)._\n"
            )
    return "\n".join(parts)


def splice(report_text: str, block: str) -> str:
    if _BEGIN not in report_text or _END not in report_text:
        raise SystemExit(
            f"report missing {_BEGIN}/{_END} markers — cannot render."
        )
    head, _, rest = report_text.partition(_BEGIN)
    _, _, tail = rest.partition(_END)
    return f"{head}{_BEGIN}\n\n{block}\n{_END}{tail}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="render_ablation_into_report")
    ap.add_argument(
        "--check", action="store_true",
        help="Exit non-zero if the report's results block is out of date "
        "(does not write).",
    )
    args = ap.parse_args(argv)

    if not _REPORT.exists():
        raise SystemExit(f"report not found: {_REPORT}")
    current = _REPORT.read_text()
    block = render_block()
    updated = splice(current, block)

    if args.check:
        if updated != current:
            print("ablation results block is STALE — run "
                  "scripts/render_ablation_into_report.py", file=sys.stderr)
            return 1
        print("ablation results block is up to date.")
        return 0

    _REPORT.write_text(updated)
    print(f"rendered {len([f for f in _FOVS if _table_path(f).exists()])} "
          f"committed FOV table(s) into {_REPORT.relative_to(_REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
