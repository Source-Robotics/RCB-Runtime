#!/usr/bin/env python3
"""58: what a block-program Check costs the RT loop — TODO/blockly_plan.md B7. NOTHING MOVES.

A Check (blockly/check.py) loads the real EXEC planner (roboticstoolbox, pinocchio) in a new process and plans every
move of a program. On this Pi, memory traffic from other cores is what slows RTI (TODO/future_todo.md, 2026-10-01:
interpreter start-ups +38 % exec time), so the plan says: measure it before trusting Check while the arm is in use.

It samples RTI's own timing from /dev/shm (read as files — never a live segment) for --baseline seconds, then runs
--checks Checks of a small program (J5 ±5 deg by joints, then a straight line up and down 10 mm from where the arm
is), and reports for the baseline and for the Checks: RTI's execution time per tick (sampled ~100 Hz: mean, max),
RTI's own rolling p99 of the loop period (max seen), and the overrun counter (ticks over the 4 ms budget).

Checks never send motion: the planner runs offline. It reads the arm's joints and the robot status (both open to
every client); it takes no control. Run it with the robot DISABLED first (a LOOP error then only latches, harmless);
--enabled-ok runs it with the robot enabled, which is the case being decided.

    .venv/bin/python api_tests/58_block_check_load.py
    .venv/bin/python api_tests/58_block_check_load.py --enabled-ok
"""
import argparse
import os
import statistics
import struct
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
TMP = tempfile.mkdtemp(prefix="api58_")
os.environ["RCB_PROGRAMS_DIR"] = os.path.join(TMP, "programs")

from blockly import runs                                              # noqa: E402

BUDGET_S = 0.004


def shm(name):
    try:
        with open(f"/dev/shm/{name}", "rb") as f:
            return struct.unpack("<d", f.read(8))[0]
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
        print(f"  {label:18s} (no samples — is RTI running?)")
        return
    print(f"  {label:18s} exec mean {w['exec_mean']:.2f} ms, max {w['exec_max']:.2f} ms | RTI p99 period up to "
          f"{w['p99_max']:.2f} ms | overruns {w['overruns']:.0f}   ({w['n']} samples)")


def program():
    def blk(btype, bid, fields=None, inputs=None, nxt=None):
        b = {"type": btype, "id": bid, "fields": fields or {}}
        if inputs:
            b["inputs"] = {k: {"block": v} for k, v in inputs.items()}
        if nxt:
            b["next"] = {"block": nxt}
        return b

    here = blk("rcb_here", "here")
    up = blk("rcb_shift", "up", {"DX": 0, "DY": 0, "DZ": 10}, {"TARGET": here})
    turn = lambda bid, deg, nxt: blk("rcb_turn_joint", bid, {"JOINT": "5", "DEG": deg, "SPEED": 20}, nxt=nxt)  # noqa: E731
    line = lambda bid, target, nxt=None: blk("rcb_move_linear", bid, {"SPEED": 10}, {"TARGET": target}, nxt)  # noqa: E731
    down = blk("rcb_shift", "down", {"DX": 0, "DY": 0, "DZ": -10}, {"TARGET": blk("rcb_here", "here2")})
    first = turn("t1", 5, turn("t2", -5, line("l1", up, line("l2", down))))
    return {"blocks": {"languageVersion": 0, "blocks": [
        {"type": "rcb_start", "id": "start", "x": 0, "y": 0, "next": {"block": first}}]}}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--checks", type=int, default=3)
    ap.add_argument("--baseline", type=float, default=5.0)
    ap.add_argument("--enabled-ok", action="store_true", help="measure with the robot ENABLED (nothing moves)")
    args = ap.parse_args()
    runs.configure(host=args.host, port=args.port, log_path=os.path.join(TMP, "runner.log"))

    with open("/dev/shm/robot_state", "rb") as f:        # a SharedString: 4-byte length, then the text
        raw = f.read()
    state = raw[4:4 + struct.unpack("<I", raw[:4])[0]].decode(errors="replace")
    print(f"robot {state}; {args.checks} Check(s) after a {args.baseline:.0f} s baseline. Nothing moves.")
    if state != "DISABLED" and not args.enabled_ok:
        print("The robot is not DISABLED. Measure it disabled first, or pass --enabled-ok. Nothing was run.")
        return 2

    rows, stop = [], threading.Event()
    sampler = threading.Thread(target=sample, args=(stop, rows), daemon=True)
    sampler.start()
    t_base = time.monotonic()
    time.sleep(args.baseline)
    spans, results = [], []
    for i in range(args.checks):
        t0 = time.monotonic()
        r = runs.check("api58", program(), {})
        t1 = time.monotonic()
        spans.append((t0, t1))
        results.append(r)
        s = r.get("summary") or {}
        print(f"  Check {i + 1}: {r['result']} in {t1 - t0:.1f} s"
              + (f" — {s.get('moves')} moves, {s.get('motion_s')} s of motion" if s else f" — {r.get('error')}"))
        time.sleep(1.0)
    time.sleep(1.0)
    stop.set()
    sampler.join(1)

    print("\nRTI timing (the budget is 4 ms per tick):")
    show("baseline", window(rows, t_base, t_base + args.baseline))
    for i, (t0, t1) in enumerate(spans):
        show(f"during Check {i + 1}", window(rows, t0, t1))
    total = window(rows, spans[0][0], spans[-1][1]) if spans else None
    if total:
        print(f"\nVERDICT-INPUT: overruns during the Checks: {total['overruns']:.0f}; worst exec sample "
              f"{total['exec_max']:.2f} ms ({total['exec_max'] / (BUDGET_S * 1000):.0%} of the budget).")
    return 0 if all(r["result"] in ("passed", "limited") for r in results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        import shutil
        shutil.rmtree(TMP, ignore_errors=True)
