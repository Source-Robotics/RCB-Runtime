#!/usr/bin/env python3
"""Measure the control loop's delay per joint, on request (2026-10-25; the dashboard's RT Diagnostics "Measure" button
runs this). It never runs on its own.

The delay: a joint's CAN reply arriving -> the CAN command RTI computed from it leaving (utility/loop_delay.py says
what it is and why it matters). For `--seconds` this records every frame on the CAN bus with `candump` (the kernel's
arrival times: the drives' replies and RTI's own commands as they leave) and RTI's per-tick CAN read times from its
shared-memory trace (utility/trace_ring.py, read the way the recorder reads it), then pairs them.

RTI does NOTHING for it: no socket, no filter, no extra work in its loop. Until 2026-10-25 RTI sampled the delay itself
and switched a CAN socket filter inside its tick, which waited for the kernel's network lock whenever another process
held it — the single-tick LOOP_CRITICAL stalls (utility/loop_delay.py). Here only this process waits, if at all.

    tools/loop_delay.py                  # 3 s, a table
    tools/loop_delay.py --seconds 5 --json

It also gives how old a trace row's joint angles are at the row's own timestamp (2026-11-25, utility/loop_delay.py
joint_data_age): the camera's frames are paired with the rows by that timestamp. The rows' timestamps are RTI's
time.perf_counter() (CLOCK_MONOTONIC); this tool puts them on CLOCK_REALTIME, candump's clock, with the offset
between the two clocks read here before and after the recording (the drift between the two readings is reported).

Exit code 0 with a result, 1 when it could not measure (RTI not running, no candump, the tick order changed meanwhile,
no frames) — the reason printed (with --json: {"ok": false, "error": ...}).
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from utility import sim_mode  # noqa: E402
from utility.loop_delay import report  # noqa: E402

SECONDS_DEFAULT = 3.0
SECONDS_MIN, SECONDS_MAX = 1.0, 30.0
POLL_S = 0.05            # the trace ring holds 5 s: polled often, nothing is lost
START_S = 0.3            # candump's start-up before the window opens


class MeasureError(Exception):
    pass


def _read_first_flag():
    """rti['can_read_first'] (1 = option B's order) — attached alone, not the whole shared-memory module."""
    from data.shared_data import _attach_or_create
    _shm, arr = _attach_or_create("can_read_first", np.float64, 1)
    return float(arr[0])


def _clock_offset():
    """CLOCK_REALTIME minus CLOCK_MONOTONIC in s, from the tightest of 50 bracketed readings (to a few us)."""
    best = None
    for _ in range(50):
        m1 = time.clock_gettime(time.CLOCK_MONOTONIC)
        r = time.clock_gettime(time.CLOCK_REALTIME)
        m2 = time.clock_gettime(time.CLOCK_MONOTONIC)
        if best is None or m2 - m1 < best[0]:
            best = (m2 - m1, r - 0.5 * (m1 + m2))
    return best[1]


def measure(seconds=SECONDS_DEFAULT, interface=None):
    """Record and pair; -> the report (utility.loop_delay.report) + "seconds", "order", "lag", "interface",
    "measured_utc", "lost_rows". Raises MeasureError with the reason."""
    from utility.trace_ring import TraceReader
    seconds = float(seconds)
    if not SECONDS_MIN <= seconds <= SECONDS_MAX:
        raise MeasureError(f"--seconds must be {SECONDS_MIN:g} to {SECONDS_MAX:g}, got {seconds:g}")
    interface = interface or sim_mode.can_interface()
    if shutil.which("candump") is None:
        raise MeasureError("candump is not installed (the can-utils package)")
    reader = TraceReader.attach()
    if reader is None:
        raise MeasureError("RTI is not running: no trace in shared memory")
    cols = {name: col for name, col in reader.layout["scalars"]}
    modes = list(reader.layout["modes"])
    joints = next((size for name, _start, size in reader.layout["arrays"] if name == "j_p"), 6)
    order_before = _read_first_flag()
    offset_before = _clock_offset()
    with tempfile.TemporaryFile("w+") as log:
        dump = subprocess.Popen(["candump", "-L", interface], stdout=log, stderr=subprocess.PIPE, text=True)
        try:
            time.sleep(START_S)
            if dump.poll() is not None:
                raise MeasureError(f"candump on {interface} stopped: {dump.stderr.read().strip()}")
            reader.poll()                                     # start at the present
            rows, lost = [], 0
            t_end = time.monotonic() + seconds
            while time.monotonic() < t_end:
                time.sleep(POLL_S)
                block, n_lost = reader.poll()
                rows.append(block)
                lost += n_lost
            time.sleep(START_S)                               # the last tick's command reaches the log
            block, n_lost = reader.poll(lag=0)
            rows.append(block)
            lost += n_lost
            if reader.stale():
                raise MeasureError("RTI restarted during the measurement")
        finally:
            dump.send_signal(signal.SIGINT)
            try:
                dump.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                dump.kill()
        log.seek(0)
        lines = log.readlines()
    offset_after = _clock_offset()
    if _read_first_flag() != order_before:
        raise MeasureError("the tick order (can_read_first) changed during the measurement — measure again")
    data = np.concatenate([r for r in rows if r.size] or [np.empty((0, reader.width))])
    if data.shape[0] == 0:
        raise MeasureError("no ticks from RTI's trace")
    mode_idx = data[:, cols["mode"]].astype(int)
    mode_names = [modes[i] if 0 <= i < len(modes) else "?" for i in mode_idx]
    lag = 0 if order_before > 0.5 else 1
    offset = 0.5 * (offset_before + offset_after)
    # RTI's own reply times (the trace's j_rx, 2026-11-25), when this RTI records them: checked against the bus
    rx = next(((start, size) for name, start, size in reader.layout["arrays"] if name == "j_rx"), None)
    rx_t = data[:, rx[0]:rx[0] + rx[1]] + offset if rx else None
    rep = report(lines, data[:, cols["tick"]], data[:, cols["can_read_t"]], mode_names, lag, joints,
                 row_t=data[:, cols["timestamp"]] + offset, rx_t=rx_t)
    if rep["frames"] == 0:
        raise MeasureError(f"no CAN frames on {interface}")
    rep.update({"seconds": seconds, "order": "read first (option B)" if lag == 0 else "normal", "lag": lag,
                "interface": interface, "lost_rows": lost, "clock_drift_us": (offset_after - offset_before) * 1e6,
                "measured_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())})
    return rep


def format_table(rep):
    lines = [f"loop delay, {rep['seconds']:g} s on {rep['interface']} ({rep['measured_utc']}), tick order: {rep['order']}"
             f"; {rep['ticks']} ticks judged" + (f", {rep['skipped_mode']} skipped (homing / flashing)"
                                                 if rep["skipped_mode"] else "")
             + (f", {len(rep['gaps'])} stretch(es) candump missed" if rep["gaps"] else ""),
             "joint   median    p99     max   (ms)   pairs"]
    for name, s in rep["joints"].items():
        if s.get("n"):
            lines.append(f"{name:5s} {s['med']:7.2f} {s['p99']:7.2f} {s['max']:7.2f}        {s['n']:6d}")
        else:
            lines.append(f"{name:5s}       —       —       —             0")
    age = rep.get("age")
    if age:
        r2r = age["read_to_row"]
        lines.append("")
        lines.append("the joint angles' age at their trace row's timestamp (ms; the camera pairs frames by that "
                     "timestamp)")
        lines.append("joint   from the command it answered (median p99)   from the reply (median p99)   rows")
        for name, a in age["joints"].items():
            cmd, rep_ = a["command"], a["reply"]
            if cmd.get("n") and rep_.get("n"):
                lines.append(f"{name:5s}   {cmd['med']:7.2f} {cmd['p99']:7.2f}                          "
                             f"{rep_['med']:7.2f} {rep_['p99']:7.2f}          {cmd['n']:6d}")
            else:
                lines.append(f"{name:5s}         —       —                                —       —               0")
        rx_lines = [f"{name:5s}   {a['rx_vs_reply']['med']:+7.3f} {a['rx_vs_reply']['max']:7.3f}"
                    f"                        {a['measured_vs_command']['med']:+7.3f} "
                    f"{a['measured_vs_command']['p99']:7.3f}"
                    for name, a in age["joints"].items()
                    if a.get("rx_vs_reply", {}).get("n") and a.get("measured_vs_command", {}).get("n")]
        if rx_lines:
            lines.append("")
            lines.append("RTI's own record of each reply (the trace's j_rx; the camera pairs each joint on it, minus "
                         "0.21 ms), against the bus (ms)")
            lines.append("joint   j_rx - the reply's bus time (median max)   j_rx - 0.21 - the command's (median p99)")
            lines += rx_lines
        if r2r.get("n"):
            lines.append(f"(check: a row's timestamp comes {r2r['med']:.2f} ms after its own CAN read, max "
                         f"{r2r['max']:.2f}; the two clocks drifted {rep.get('clock_drift_us', 0.0):+.1f} us apart "
                         f"during the recording)")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seconds", type=float, default=SECONDS_DEFAULT,
                    help=f"how long to record, {SECONDS_MIN:g} to {SECONDS_MAX:g} s (default {SECONDS_DEFAULT:g})")
    ap.add_argument("--json", action="store_true", help="print one JSON object (the dashboard reads it)")
    args = ap.parse_args(argv)
    try:
        rep = measure(args.seconds)
    except MeasureError as e:
        print(json.dumps({"ok": False, "error": str(e)}) if args.json else f"cannot measure: {e}")
        return 1
    print(json.dumps({"ok": True, **rep}) if args.json else format_table(rep))
    return 0


if __name__ == "__main__":
    sys.exit(main())
