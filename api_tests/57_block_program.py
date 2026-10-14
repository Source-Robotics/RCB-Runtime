#!/usr/bin/env python3
"""57: block programs on the arm — TODO/blockly_plan.md B6. ON THE ARM, J5 ONLY.

Runs block programs through the REAL path the editor uses — blockly/programs.py writes the file, blockly/runs.py starts
the runner (`python3 -m blockly.runner`), which drives command_executor — and checks:

  1. a program [J5 +step (joint values), wait 0.5 s, back (a named point)]: blocks announced in order, J5 back where
     it began (0.2 deg), exit 0;
  2. PAUSE once J5 has gone --into of the step (a quarter): the COMMANDED position freezes and the arm settles on it (it creeps a little
     while it catches up — seen on the arm 2026-10-05: 0.08 deg); RESUME finishes the program;
  3. STOP the same way: the program ends "stopped" (exit 2), J5 stops on its way and holds there in EXEC (2026-10-14);
  4. a move past J5's software limit: refused JOINT_LIMIT on ITS block, J5 does not go on toward it, the arm HOLDS in
     EXEC (as an EXEC program leaves it — user, 2026-10-05); errors cleared;
  5. RT timing across the runner start-ups: RTI's overrun counter before / after (an interpreter start-up costs the
     RT loop memory bandwidth — TODO/future_todo.md, 2026-10-01). Reported, not judged: look at RT Diagnostics too.

SAFETY
  * Only J5 moves: by --step degrees (default 5) at --speed percent (default 10), and back. Nothing is homed or
    enabled here: the robot must be ENABLED, homed, without errors, in IDLE or EXEC (the dashboard). The program takes
    control WITHOUT forcing it — the dashboard gives it back 5 s after its last click.
  * The limit test (4) first moves J5 to --margin degrees inside its nearer software limit (a legal pose), then asks
    for 3 degrees PAST it. If the check were somehow broken, J5 would stop 3 degrees past its soft limit — still far
    inside the hard limit (as api_tests/45). Make sure the tool / cables allow that wrist angle.
  * The programs are written to a temporary folder; the robot's programs/ is not touched.
  * Ctrl-C stops the robot and the program. J5 is brought back to where it started at the end.

    .venv/bin/python api_tests/57_block_program.py                  # dry run: prints the plan, moves nothing
    .venv/bin/python api_tests/57_block_program.py --go
    .venv/bin/python api_tests/57_block_program.py --go --skip-limit     # without the long wrist move of test 4
"""
import argparse
import os
import struct
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
TMP = tempfile.mkdtemp(prefix="api57_")
os.environ["RCB_PROGRAMS_DIR"] = os.path.join(TMP, "programs")

import numpy as np                                                    # noqa: E402

import config.xml_parser as xp                                        # noqa: E402
from blockly import programs, runs                                    # noqa: E402
from robotclient import RobotClient                                   # noqa: E402

J = 4                     # J5, 0-based
_ok, _bad = 0, []


def check(name, cond, detail=""):
    global _ok
    if cond:
        _ok += 1
    else:
        _bad.append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    return cond


class _SkipLimit(Exception):
    pass


def commanded_j5():
    """J5's COMMANDED position (deg), read from /dev/shm as a file — never open a live segment (CLAUDE.md)."""
    try:
        with open("/dev/shm/joint_position_commanded", "rb") as f:
            return float(np.degrees(struct.unpack("<6d", f.read(48))[J]))
    except (OSError, struct.error):
        return None


def shm(name):
    try:
        with open(f"/dev/shm/{name}", "rb") as f:
            return struct.unpack("<d", f.read(8))[0]
    except OSError:
        return None


# ── programs as Blockly would save them ─────────────────────────────────────────────────────────────────────────

def block(btype, bid, fields=None, inputs=None):
    b = {"type": btype, "id": bid, "fields": fields or {}}
    if inputs:
        b["inputs"] = {k: {"block": v} for k, v in inputs.items()}
    return b


def move_to(bid, angles, speed):
    return block("rcb_move_joints", bid, {"SPEED": speed},
                 {"TARGET": block("rcb_joints", bid + "_j", {f"J{i + 1}": round(float(a), 3) for i, a in enumerate(angles)})})


def move_to_point(bid, name, speed):
    return block("rcb_move_joints", bid, {"SPEED": speed}, {"TARGET": block("rcb_point", bid + "_p", {"NAME": name})})


def program(*statements):
    first = None
    for b in reversed(statements):
        if first is not None:
            b["next"] = {"block": first}
        first = b
    return {"blocks": {"languageVersion": 0, "blocks": [
        {"type": "rcb_start", "id": "start", "x": 20, "y": 20, "next": {"block": first}}]}}


def run(name, workspace, points=None, on_block=None, timeout=90.0):
    """Save, start, follow the events to the end. on_block(id) is called (in this thread) as each block is announced.
    Returns (final snapshot, the events)."""
    programs.save_program(name, {"workspace": workspace, "points": points or {}})
    runs.start(name)
    seen, events = 0, []
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        r = runs.events_since(seen)
        for e in r["events"]:
            events.append(e)
            if e["type"] == "block" and on_block is not None:
                on_block(e["id"])
            if e["type"] == "error":
                print(f"        robot said: {e['code']}: {e['message']}")
        seen = r["seq"]
        if not r["running"] and r["exit"] is not None:
            return r, events
        time.sleep(0.02)
    runs.stop()
    raise TimeoutError(f"program {name!r} did not end within {timeout:.0f} s — stopped")


def blocks_of(events):
    return [e["id"] for e in events if e["type"] == "block"]


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=29000)
    p.add_argument("--go", action="store_true", help="actually move J5")
    p.add_argument("--speed", type=float, default=10.0, help="move speed, percent")
    p.add_argument("--step", type=float, default=5.0, help="degrees J5 moves")
    p.add_argument("--margin", type=float, default=5.0, help="deg inside the soft limit for the approach pose")
    p.add_argument("--into", type=float, default=0.25,
                   help="pause / stop once J5 has gone this fraction of the step (0..1; was seconds until 2026-10-13)")
    p.add_argument("--skip-limit", action="store_true",
                   help="skip test 4 (it moves J5 to --margin degrees inside its NEARER soft limit — maybe a long way)")
    args = p.parse_args()

    lo, hi = xp.load_software_joint_limits(os.path.join(ROOT, "robots", "PAR6.xml"))
    lo_d, hi_d = float(np.degrees(lo[J])), float(np.degrees(hi[J]))
    runs.configure(host=args.host, port=args.port, log_path=os.path.join(TMP, "runner.log"))
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    q0 = [float(v) for v in robot.get_joints()]
    j0 = q0[J]
    sign = 1.0 if j0 + args.step <= hi_d - 1.0 else -1.0
    target = list(q0)
    target[J] = j0 + sign * args.step
    upper = (hi_d - j0) <= (j0 - lo_d)
    limit = hi_d if upper else lo_d
    lsign = 1.0 if upper else -1.0
    near, past = list(q0), list(q0)
    near[J] = limit - lsign * args.margin
    past[J] = limit + lsign * 3.0
    st = robot.get_status()
    print(f"J5 now {j0:.2f} deg, software limits [{lo_d:.2f}, {hi_d:.2f}] deg; robot {st.get('mode')} / "
          f"{st.get('state')} / homed={st.get('homed')}.")
    print(f"Plan: J5 -> {target[J]:.2f} and back (x3: plain, paused, stopped)" + (
        "; the limit test is SKIPPED" if args.skip_limit else
        f"; then -> {near[J]:.2f} (legal, a {abs(near[J] - j0):.0f} deg move) and {past[J]:.2f} (must be REFUSED); "
        f"back to {j0:.2f}") + f". Speed {args.speed:.0f} %.")
    if not args.go:
        print("DRY RUN: re-run with --go to execute. Nothing moved.")
        return 0
    if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("error_active") \
            or st.get("mode") not in ("IDLE", "EXEC"):
        print("The robot must be ENABLED, homed, error-free, in IDLE or EXEC — prepare it from the dashboard. "
              "Nothing moved.")
        return 2
    point = {"start": {"joints_deg": q0, "pose": [float(v) for v in robot.get_pose()]}}
    overruns0 = shm("overrun_count")
    try:
        print("1. a three-block program")
        r, ev = run("api57 plain", program(move_to("m1", target, args.speed), block("rcb_wait", "w1", {"SECONDS": 0.5}),
                                           move_to_point("m2", "start", args.speed)), point)
        j = robot.get_joints()[J]
        check("done, exit 0", r["exit"] == 0 and r["state"] == "done", f"exit {r['exit']} {r['error']}")
        check("blocks announced in order", blocks_of(ev) == ["m1", "w1", "m2"], str(blocks_of(ev)))
        check("J5 back where it began (named point)", abs(j - j0) < 0.2, f"{j:.3f} vs {j0:.3f}")

        print("2. pause mid-move")
        held = {}

        def when_moving(action):
            """`action` once J5 has really gone --into of the step — not after a fixed time: the EXEC default profile's
            jerk-limited start (ruckig, since 2026-10-11) had J5 only 0.1 deg along after the old 0.6 s at 10 %
            (2026-10-13, the simulated suite)."""
            gone = max(0.3, abs(target[J] - j0) * min(max(args.into, 0.05), 0.9))

            def wait():
                end = time.monotonic() + 20.0
                while time.monotonic() < end and abs(robot.get_joints()[J] - j0) < gone:
                    time.sleep(0.02)
                action()
            threading.Thread(target=wait, daemon=True).start()

        def pause_soon(bid):
            if bid == "m1":
                when_moving(runs.pause)

        def watch():
            end = time.monotonic() + 10
            while time.monotonic() < end and runs.snapshot()["state"] != "paused":
                time.sleep(0.02)
            time.sleep(0.2)
            held["a"], held["ca"] = robot.get_joints()[J], commanded_j5()
            time.sleep(1.5)
            held["b"], held["cb"] = robot.get_joints()[J], commanded_j5()
            runs.resume()
        threading.Thread(target=watch, daemon=True).start()
        r, ev = run("api57 pause", program(move_to("m1", target, args.speed), move_to_point("m2", "start", args.speed)),
                    point, on_block=pause_soon)
        if "b" not in held:
            check("the pause happened", False, "no 'paused' state seen")
        else:
            check("paused mid-move (J5 on its way, not arrived)",
                  abs(held["a"] - j0) > 0.1 and abs(held["a"] - target[J]) > 0.2,
                  f"J5 {held['a']:.3f}: from {j0:.2f} toward {target[J]:.2f} — adjust --into or --speed")
            check("paused: the COMMANDED position is frozen (< 0.001 deg over 1.5 s) — what pause guarantees",
                  held["ca"] is not None and abs(held["cb"] - held["ca"]) < 0.001, f"{held['ca']} -> {held['cb']}")
            check("... and the arm settles on it (within 0.15 deg)",
                  held["cb"] is not None and abs(held["b"] - held["cb"]) < 0.15,
                  f"measured {held['b']:.3f}, commanded {held['cb']}; the arm crept "
                  f"{held['b'] - held['a']:+.3f} deg while paused (catching up with the frozen setpoint)")
        check("resumed and finished: exit 0, J5 back", r["exit"] == 0 and abs(robot.get_joints()[J] - j0) < 0.2,
              f"exit {r['exit']} {r['error']}")

        print("3. stop mid-move")

        def stop_soon(bid):
            if bid == "m1":
                when_moving(runs.stop)
        r, ev = run("api57 stop", program(move_to("m1", target, args.speed), move_to_point("m2", "start", args.speed)),
                    point, on_block=stop_soon)
        time.sleep(0.5)
        j = robot.get_joints()[J]
        mode = robot.get_status().get("mode")
        check("ended 'stopped', exit 2", r["exit"] == 2 and r["state"] == "stopped", f"exit {r['exit']} {r['state']}")
        check("J5 stopped on its way (moved, short of the target)", abs(j - j0) > 0.1 and abs(j - target[J]) > 0.2,
              f"J5 {j:.2f}: from {j0:.2f} toward {target[J]:.2f}")
        check("the arm HOLDS in EXEC after the stop (braked along its path; 2026-10-14 — it used to drop to IDLE)",
              mode == "EXEC", mode)
        run("api57 back", program(move_to_point("m1", "start", args.speed)), point)

        if args.skip_limit:
            raise _SkipLimit()
        print("4. a move past the software limit")
        r, ev = run("api57 limit", program(move_to("near", near, args.speed), move_to("bad", past, args.speed),
                                           move_to_point("never", "start", args.speed)), point)
        j = robot.get_joints()[J]
        err = r["error"] or {}
        check("refused JOINT_LIMIT on its own block", r["exit"] == 1 and err.get("block") == "bad"
              and err.get("code") == "JOINT_LIMIT", str(err))
        beyond = (j - near[J]) * lsign          # > 0: further toward the refused target
        check("J5 did not go on toward the refused target (at most 0.05 deg past the approach pose)", beyond < 0.05,
              f"J5 {j:.2f}, approach {near[J]:.2f}, refused {past[J]:.2f}: {beyond:+.2f} deg past it")
        check("the block after it never ran", "never" not in blocks_of(ev), str(blocks_of(ev)))
        mode = robot.get_status().get("mode")
        check("the arm HOLDS in EXEC after the refusal (as an EXEC program leaves it: no drop to IDLE)",
              mode == "EXEC", f"mode {mode}; J5 {j:.2f}, {abs(j - near[J]):.2f} deg from the approach pose")
        robot.clear_errors(wait=True)
        r, _ = run("api57 back", program(move_to_point("m1", "start", args.speed)), point)
        check("after clear errors: back to the start", r["exit"] == 0 and abs(robot.get_joints()[J] - j0) < 0.2,
              f"exit {r['exit']} {r['error']}")
    except _SkipLimit:
        print("4. skipped (--skip-limit)")
    except KeyboardInterrupt:
        print("\nCtrl-C: stopping the robot and the program.")
        runs.stop()
        return 1
    finally:
        overruns1 = shm("overrun_count")
        if overruns0 is not None and overruns1 is not None:
            print(f"5. RT timing: RTI overruns during this test: {overruns1 - overruns0:.0f} (8 runner start-ups). "
                  "Also look at RT Diagnostics; compare with a test that starts no runner (e.g. 45).")
        robot.disconnect()
    print(f"\n{_ok} passed, {len(_bad)} failed" + (f": {_bad}" if _bad else ""))
    return 1 if _bad else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        import shutil
        shutil.rmtree(TMP, ignore_errors=True)
