"""The robotics toolbox, imported once per process without NumPy's "compiled using NumPy 1.x" block (2026-10-15).

spatialgeometry (pulled in by roboticstoolbox) ships a compiled scene-graph module built for NumPy 1.x. On NumPy 2
its import fails, NumPy writes a ~25-line explanation plus a stack trace to stderr, and the toolbox carries on
without that module (it catches the ImportError; nothing the robot uses needs the scene graph). Under the
supervisor stderr is logs/<NAME>.out.log: the block was most of every .out.log — 434 times in RTI.out.log, 931 in
HIGH_LEVEL.out.log — and buried the crash tracebacks the file is there for.

import_roboticstoolbox() imports it with sys.stderr captured, drops exactly that block (from NumPy's first line to
the "_ARRAY_API not found" line) and writes everything else captured back to stderr unchanged — a real warning or
error still shows. The block's removal is logged once, at DEBUG, on "kin". The cure is a spatialgeometry built
for NumPy 2; until then this keeps the .out.log files readable. Imported by robots/*.py and utility/pinocchio_attach.py
before their own `from roboticstoolbox import ...`.
"""
from __future__ import annotations

import contextlib
import io
import logging
import sys

_FIRST = "A module that was compiled using NumPy 1.x cannot be run in"
_LAST = "_ARRAY_API not found"


def strip_numpy1_block(text: str) -> tuple[str, bool]:
    """(text without NumPy's 1.x block, whether one was removed). Only a complete block is removed."""
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    i, removed = 0, False
    while i < len(lines):
        if _FIRST in lines[i]:
            end = next((k for k in range(i, len(lines)) if _LAST in lines[k]), None)
            if end is not None:
                if out and not out[-1].strip():      # NumPy's message opens with an empty line
                    out.pop()
                i, removed = end + 1, True
                continue
        out.append(lines[i])
        i += 1
    return "".join(out), removed


def import_roboticstoolbox():
    if "roboticstoolbox" in sys.modules:
        return sys.modules["roboticstoolbox"]
    buf = io.StringIO()
    try:
        with contextlib.redirect_stderr(buf):
            import roboticstoolbox
    finally:
        rest, removed = strip_numpy1_block(buf.getvalue())
        if rest:
            sys.stderr.write(rest)
        if removed:
            logging.getLogger("kin").debug("spatialgeometry's NumPy-1.x scene graph is unavailable (its import "
                                           "message suppressed; utility/quiet_imports.py)")
    return roboticstoolbox


import_roboticstoolbox()
