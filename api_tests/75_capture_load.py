#!/usr/bin/env python3
"""75: what a telemetry capture costs the RT loop (2026-10-15). NOTHING MOVES.

A capture (`robot.capture_telemetry`, the Recordings tab's "Capture now") is taken by the recorder process
(recorder.py, since 2026-10-16) from RTI's shared-memory trace — RTI only writes its trace row every tick (~9 us),
capture or not. On 2026-10-16, with the capture still made inside RTI's process, a 60 s capture made 37 ticks late and
raised LOOP_DEGRADED every 3 s; this checks that it now costs the loop nothing (memory traffic from other processes
can still slow RTI on this Pi — TODO/future_todo.md). As 58 does for a block-program Check.

It samples RTI's own timing from /dev/shm (read as files — never a live segment) for --baseline seconds, asks for a
--seconds capture, and reports for the baseline, while the capture collects, and while it writes the file: RTI's
execution time per tick (sampled ~100 Hz: mean, max), RTI's rolling p99 of the loop period (max seen) and the overrun
counter (ticks over the 4 ms budget). Then the file's ticks and any gaps.

Run it with the robot DISABLED (a LOOP error then only latches, harmless); --enabled-ok measures it enabled.

    .venv/bin/python api_tests/75_capture_load.py                 # a 60 s capture (the longest)
    .venv/bin/python api_tests/75_capture_load.py --seconds 10
"""
import argparse
import glob
import os
import statistics
import struct
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from project_paths import TEMP_DATA_DIR                    # noqa: E402
from robotclient import RobotClient                        # noqa: E402
from utility.incident_dump import load                     # noqa: E402

BUDGET_S = 0.004
NAME = "load_test_75"


def shm(name):
    try:
        with open(f"/dev/shm/{name}", "rb") as f:
            return struct.unpack("<d", f.read(8))[0]
    except (OSError, struct.error):
        return None


def robot_state():
    try:
        with open("/dev/shm/robot_state", "rb") as f:        # a SharedString: 4-byte length, then the text
            raw = f.read()
        return raw[4:4 + struct.unpack("<I", raw[:4])[0]].decode(errors="replace")
    except (OSError, struct.error):
        return None


def sample(stop, out):
    """~100 Hz: (t, execution time of the tick, RTI's rolling p99 period, overrun count)."""
    while not stop.is_set():
        out.append((time.monotonic(), shm("loop_execution_time"), shm("p99_loop_period"), shm("overrun_count")))
        time.sleep(0.01)


def window(rows, t0, t1):
    part = [r for r in rows if t0 <= r[0] <= t1 and r[1] is not None]
    if not part:
        return None
    ex = [r[1] * 1000 for r in part]
    return {"n": len(part), "exec_mean": statistics.fmean(ex), "exec_max": max(ex),
            "p99_max": max(r[2] for r in part if r[2] is not None) * 1000,
            "overruns": part[-1][3] - part[0][3]}


def show(label, w):
    if w is None:
        print(f"  {label:20s} (no samples — is RTI running?)")
        return
    print(f"  {label:20s} exec mean {w['exec_mean']:.2f} ms, max {w['exec_max']:.2f} ms | RTI p99 period up to "
          f"{w['p99_max']:.2f} ms | overruns {w['overruns']:.0f}   ({w['n']} samples)")


def capture_files():
    return set(glob.glob(os.path.join(str(TEMP_DATA_DIR), "captures", f"*_{NAME}.npz")))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--seconds", type=float, default=60.0, help="the capture's length (1-60 s)")
    ap.add_argument("--baseline", type=float, default=5.0)
    ap.add_argument("--enabled-ok", action="store_true", help="measure with the robot ENABLED (nothing moves)")
    args = ap.parse_args()

    state = robot_state()
    print(f"robot {state}; a {args.seconds:g} s capture after a {args.baseline:g} s baseline. Nothing moves.")
    if state != "DISABLED" and not args.enabled_ok:
        print("The robot is not DISABLED. Measure it disabled first, or pass --enabled-ok. Nothing was run.")
        return 2

    robot = RobotClient(name="75_capture_load")
    robot.connect(args.host, args.port)
    before = capture_files()
    rows, stop = [], threading.Event()
    sampler = threading.Thread(target=sample, args=(stop, rows), daemon=True)
    sampler.start()
    t_base = time.monotonic()
    time.sleep(args.baseline)
    t0 = time.monotonic()
    robot.capture_telemetry(args.seconds, NAME)
    t_collected = t0 + args.seconds
    new, t_saved = set(), None
    while time.monotonic() < t_collected + 60.0:            # the file appears after the collection + the write
        new = capture_files() - before
        if new:
            t_saved = time.monotonic()
            break
        time.sleep(0.05)
    time.sleep(2.0)
    stop.set()
    sampler.join(1)
    robot.disconnect()

    print("\nRTI timing (the budget is 4 ms per tick):")
    show("baseline", window(rows, t_base, t0))
    show("collecting", window(rows, t0, min(t_collected, t_saved or t_collected)))
    if t_saved is None:
        print("  no capture file appeared within 60 s of the end — see RTI.log / the activity log (CAPTURE_*)")
        return 1
    show("writing the file", window(rows, t_collected, t_saved))
    show("after", window(rows, t_saved, t_saved + 2.0))
    path = sorted(new)[-1]
    meta, _ = load(path)
    print(f"\nfile: {os.path.basename(path)} — {meta['ticks']} ticks ({meta.get('before_ticks')} before the request, "
          f"{meta.get('after_ticks')} after), {meta.get('gaps', 0)} gap(s), {os.path.getsize(path) / 1e6:.1f} MB; "
          f"written {t_saved - t_collected:.1f} s after the collection ended")
    total = window(rows, t0, t_saved)
    if total:
        print(f"\nVERDICT-INPUT: overruns during the capture: {total['overruns']:.0f}; worst exec sample "
              f"{total['exec_max']:.2f} ms ({total['exec_max'] / (BUDGET_S * 1000):.0%} of the budget).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
