#!/usr/bin/env python3
"""The incident dump and telemetry capture FILES (2026-10-03; written by the recorder process since 2026-10-16).

An incident dump is every per-tick channel from 5 s before a hard error latched to 2 s after it; a capture is the same
channels from 5 s before a request to 1-60 s after it. Both are written by recorder.py (utility/trace_recorder.py)
from RTI's shared-memory trace (utility/trace_ring.py) — RTI itself does no part of it. This module is the file layer
every user shares: the recorder saves, the dashboard's Recordings tab lists and loads, api_tests/75 loads.

    temp_data/incidents/<UTC time>_<tick>_<key>.npz   the newest INCIDENT_KEEP
    temp_data/captures/<UTC time>_<name>.npz          the newest CAPTURE_KEEP

    from utility.incident_dump import load
    meta, channels = load(path)      # channels["j_p"].shape == (1750, 6): 5 s before + 2 s after, oldest first

A file: every channel as an array (ticks, n), scalars (ticks,), the mode as strings; `meta` (JSON): time, time_utc,
tick (the fault's / the last one), keys (the error keys / the capture's name), ticks, channels, kind ("incident" /
"capture"), and per kind before_ticks / after_ticks / after_s, or seconds / request_tick / gaps.

`_leave_the_rt_core` stays here for robotics/homing_arms.py's worker thread (a thread started by RTI after its
real-time setup inherits SCHED_FIFO 99 on RTI's core; it moves itself off).
"""
from __future__ import annotations

import calendar
import json
import logging
import os
import re
import time
from pathlib import Path

import numpy as np

from config.cpu_cores import GENERAL_CPU_CORES

_META_KEY = "__meta__"
_log = logging.getLogger("proc")  # docs/logging.md "Logger names" (its one line: a worker thread left at RT priority)
INCIDENT_KEEP = 20
INCIDENT_MIN_INTERVAL_S = 10.0   # at most one incident file per 10 s (a second fault inside it shows in the first's after-window)
INCIDENT_MIN_HISTORY = 50        # ticks before the fault; an error at a run's very start has nothing to show (2026-10-05)
INCIDENT_BEFORE_S = 5.0
INCIDENT_AFTER_S = 2.0           # the stop and the settle (a held stop rang ~1 s on the arm)
CAPTURE_MIN_S = 1.0
CAPTURE_MAX_S = 60.0
CAPTURE_KEEP = 10
OTHER_CORES = tuple(GENERAL_CPU_CORES)   # never RTI's core (3) nor the CAN core (2)


def _leave_the_rt_core(cores=OTHER_CORES) -> tuple[int, set] | None:
    """Called by a worker THREAD on itself (pid 0 = the calling thread on Linux): normal scheduling, off RTI's core.
    Returns (policy, affinity) as set, None when the platform cannot."""
    try:
        wanted = set(cores) & os.sched_getaffinity(0) if hasattr(os, "sched_getaffinity") else set()
        if not wanted:                                    # a 1-3 core box: at least drop the priority
            wanted = os.sched_getaffinity(0)
        os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
        os.sched_setaffinity(0, wanted)
        return os.sched_getscheduler(0), os.sched_getaffinity(0)
    except (AttributeError, OSError) as exc:                   # pragma: no cover - not Linux, or refused
        _log.warning("the worker thread stays at the RT priority: %s", exc)
        return None


def safe_capture_name(name) -> str:
    """A capture's file name part: letters, digits, '-' and '_', at most 32 characters; "capture" when empty."""
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "-", str(name or "")).strip("-")[:32]
    return cleaned or "capture"


def save(directory, wall_time, tick, keys, channels, ticks, kind="incident", keep=INCIDENT_KEEP, extra=None) -> Path:
    """Write one file atomically (a .tmp first, then renamed) and keep the newest `keep` in `directory`."""
    meta = {"time": wall_time, "time_utc": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(wall_time)),
            "tick": int(tick), "keys": [str(k) for k in keys], "ticks": int(ticks), "channels": sorted(channels),
            "kind": kind}
    if extra:
        meta.update(extra)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    safe = str(keys[0]).replace(":", "-").replace("/", "-") if keys else "error"
    stamp = time.strftime('%Y%m%d_%H%M%S', time.gmtime(wall_time))
    path = directory / (f"{stamp}_{safe}.npz" if kind == "capture" else f"{stamp}_{int(tick)}_{safe}.npz")
    tmp = path.with_suffix(".tmp.npz")
    np.savez_compressed(tmp, **{_META_KEY: np.array(json.dumps(meta))}, **channels)
    os.replace(tmp, path)
    prune(directory, keep)
    return path


def prune(directory, keep) -> None:
    """Keep the newest `keep` .npz files in `directory` (by modification time)."""
    files = sorted(Path(directory).glob("*.npz"), key=lambda p: p.stat().st_mtime)
    for old in files[:-keep] if keep > 0 else []:
        try:
            old.unlink()
        except OSError:
            pass


def list_files(incidents_directory, captures_directory=None):
    """[{path, name, kind, time (epoch, from the file name), size}] of every incident and capture, newest first."""
    out = []
    for directory, kind in ((Path(incidents_directory), "incident"),
                            (Path(captures_directory) if captures_directory else Path(incidents_directory).parent / "captures",
                             "capture")):
        if not directory.is_dir():
            continue
        for p in directory.glob("*.npz"):
            if p.name.endswith(".tmp.npz"):
                continue
            try:
                t = calendar.timegm(time.strptime(p.name[:15], "%Y%m%d_%H%M%S"))
            except ValueError:
                t = p.stat().st_mtime
            out.append({"path": str(p), "name": p.name, "kind": kind, "time": float(t), "size": p.stat().st_size})
    out.sort(key=lambda f: f["time"], reverse=True)
    return out


def load(path):
    """(meta dict, {channel: array}) of one file."""
    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(str(z[_META_KEY]))
        channels = {k: z[k] for k in z.files if k != _META_KEY}
    return meta, channels
