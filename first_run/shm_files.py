"""Shared memory read as FILES (/dev/shm/<name>) — RTI's live values without attaching a segment (2026-10-27; the
first-run check's stages 4 and 5; tools/tcp_stream.py's way). Read-only and never raises: None (or []) when a segment
is missing, short or unreadable. RCB_SHM_PREFIX selects another namespace, as everywhere."""
from __future__ import annotations

import json
import os
import struct

SHM = "/dev/shm/" + os.environ.get("RCB_SHM_PREFIX", "")


def floats(name, n):
    """The first n float64 values of segment `name` (data/shared_data.py: per joint in radians, Nm, ...), or None."""
    try:
        with open(SHM + name, "rb") as fh:
            raw = fh.read(8 * n)
    except OSError:
        return None
    return list(struct.unpack(f"<{n}d", raw)) if n > 0 and len(raw) == 8 * n else None


def text(name):
    """A SharedString segment (data/shared_data.py: a 4-byte little-endian length, then UTF-8), or None."""
    try:
        with open(SHM + name, "rb") as fh:
            raw = fh.read()
    except OSError:
        return None
    if len(raw) < 4:
        return None
    k = int.from_bytes(raw[:4], "little")
    return raw[4:4 + k].decode("utf-8", "replace") if k <= len(raw) - 4 else None


def errors():
    """RTI's latched errors and warnings as the dashboard's Errors tab shows them: [{"key", "description", "level",
    "timestamp", "tick"}]. The description carries the error's reason — "Joint homing failed: <why> (4002
    HOMING_FAILED)" — which the robot's status (robot.errors()) leaves out. [] when none or unreadable."""
    raw = text("error_state")
    try:
        data = json.loads(raw) if raw else []
    except ValueError:
        return []
    return [e for e in data if isinstance(e, dict) and "key" in e] if isinstance(data, list) else []
