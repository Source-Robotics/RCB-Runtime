#!/usr/bin/env python3
"""67: watch the EXEC motion profiles on the arm — the default (ruckig) next to the old one (trapezoid) (2026-10-11).

Every step runs twice: first with profile="ruckig" (the EXEC default since 2026-10-11), then the same moves with
profile="trapezoid" (the default before), so the two can be seen and heard side by side. All moves start and end at
the standard start pose (_common.START_POSE_DEG: J1 90, J2 -105, J3 165, J4 0, J5 -30, J6 180 deg); the arm goes
there first at 20 %.

  0  one small J1 move with NO profile named: shows which profile the executor really uses by default
  1  J1 alone +20 deg and back at 100 %   — the measured case: listen for J1 ringing after the trapezoid's stops
  2  all six joints together and back at 60 % — the joints stay in step on a straight line in joint space
  3  a straight TCP line, 80 mm sideways and back at 50 %
  4  a blended path at 60 %: through one gentle corner WITHOUT stopping (blend 50), then back to the start

Each pass prints its time from the first command to "settled". Enter before each step (--auto: no prompts).
The robot must be ENABLED and homed; the script enters EXEC (no motion) and takes control. The planner checks every
move (joint limits, zones, self-collision) before it starts; a refused or failed move ends the demo (then Clear Errors
on the dashboard). Ctrl+C stops the arm.

    cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/67_profile_demo.py                 # dry run: the plan
    cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/67_profile_demo.py --go
    cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/67_profile_demo.py --go --steps 1,4 --auto
"""
import argparse
import time

from _common import START_POSE_DEG, RobotClient

PROFILES = ("ruckig", "trapezoid")
START = [float(v) for v in START_POSE_DEG]


def rel(d):
    """START + d (deg, per joint)."""
    return [s + x for s, x in zip(START, d)]


# Each step: (title, what to watch, list of moves). A move: ("joints", target_deg, options) or
# ("line", (dx, dy, dz) in metres, options). Moves within a step are queued back to back, then waited for together.
STEPS = {
    1: ("J1 alone, +20 deg and back, 100 %",
        "the stops: the trapezoid brakes abruptly and J1 swings ~0.2 s (~0.7 mm at full reach); ruckig settles at once",
        [("joints", rel([20, 0, 0, 0, 0, 0]), {"speed": 100}), ("joints", START, {"speed": 100})]),
    2: ("all six joints together, and back, 60 %",
        "both stay on the straight line in joint space; ruckig starts and stops softer",
        [("joints", rel([15, 5, -8, 20, -15, 30]), {"speed": 60}), ("joints", START, {"speed": 60})]),
    3: ("a straight TCP line, 80 mm sideways (+X) and back, 50 %",
        "the tool stays on the line either way; here ruckig is even a little FASTER (planned 3.8 vs 4.5 s): along a "
        "line it is time-optimal, the trapezoid keeps its fixed ramp fraction",
        [("line", (0.08, 0.0, 0.0), {"speed": 50}), ("line", (-0.08, 0.0, 0.0), {"speed": 50})]),
    # A gentle corner on purpose: the speed carried through a corner shrinks with how sharply the direction changes
    # (the planner allows one tick's worth of acceleration for the turn) — here ~13 deg/s of a ~27 deg/s leg, for both.
    4: ("a blended path: J1 to +15, then on to +30 while J5 turns 4 deg — through that corner WITHOUT stopping "
        "(blend 50) — then back, 60 %",
        "at J1 +15 the arm slows to about half speed and goes on, no stop; then the stop at +30 and the return",
        [("joints", rel([15, 0, 0, 0, 0, 0]), {"speed": 60, "blend": 50}),
         ("joints", rel([30, 0, 0, 0, -4, 0]), {"speed": 60, "blend": 50}),
         ("joints", START, {"speed": 60})]),
}


def claim_control(robot, seconds=15.0):
    """Take control WITHOUT forcing; wait for the dashboard's 5 s release if it holds it (as api_tests/66)."""
    from robotclient.exceptions import CommandRejected
    end = time.monotonic() + seconds
    told = False
    while True:
        try:
            robot.take_control()
            return
        except CommandRejected as exc:
            if exc.code != "CONTROL_HELD" or time.monotonic() > end:
                raise
            if not told:
                print(f"Control is held by {robot.control_holder()} — waiting up to {seconds:.0f} s...")
                told = True
            time.sleep(1.0)


def send(robot, move, profile):
    kind, target, opts = move
    opts = dict(opts, **({"profile": profile} if profile else {}))
    if kind == "joints":
        robot.move_joints(*target, **opts)
    else:
        robot.move_linear_rel_world(*target, **opts)


def run_moves(robot, moves, profile):
    """Queue the moves back to back (so a blend can see the next one), wait until settled. Returns seconds, or None."""
    t0 = time.monotonic()
    for mv in moves:
        send(robot, mv, profile)
    fails = robot.wait_idle(timeout=120.0, raise_on_error=False)
    dt = time.monotonic() - t0
    if fails:
        print(f"    !!! the move failed or was refused: {fails}")
        print("    The demo stops here. Clear Errors on the dashboard before the next motion.")
        return None
    return dt


def pause(args, text):
    if args.auto:
        print(text)
        time.sleep(2.0)
    else:
        input(text + "  [Enter] ")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true", help="actually move the arm")
    ap.add_argument("--steps", default="0,1,2,3,4", help="which steps, in order (0 = the default-profile check)")
    ap.add_argument("--auto", action="store_true", help="no Enter prompts (2 s pauses)")
    args = ap.parse_args()
    steps = [int(s) for s in args.steps.split(",") if s.strip()]
    if any(s not in STEPS and s != 0 for s in steps):
        print(f"Unknown step in {steps}; known: 0, {', '.join(map(str, STEPS))}.")
        return 2

    print(f"Start pose: {START} deg")
    for s in steps:
        if s == 0:
            print("  0  one small J1 move (+5 deg and back, 30 %) with NO profile named — which one does the executor use?")
            continue
        title, watch, moves = STEPS[s]
        print(f"  {s}  {title}\n       watch: {watch}")
        for kind, target, opts in moves:
            where = [round(v, 1) for v in target] if kind == "joints" else f"TCP {tuple(1000 * v for v in target)} mm"
            print(f"       {kind:6s} -> {where}  {opts}")
    print(f"Each step runs with profile {PROFILES[0]!r}, then {PROFILES[1]!r}.")
    if not args.go:
        print("DRY RUN: re-run with --go to move the arm. Nothing moved.")
        return 0

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed = False
    try:
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") not in ("IDLE", "EXEC"):
            print(f"The robot must be ENABLED + homed, in IDLE or EXEC (got mode={st.get('mode')} "
                  f"state={st.get('state')} homed={st.get('homed')}). Nothing moved.")
            return 2
        claim_control(robot)
        claimed = True
        if st.get("motion_aborted"):
            robot.clear_errors(wait=True)
        if st.get("mode") != "EXEC":
            print("Entering EXEC (no motion).")
            robot.enter_exec_mode(wait=True)
        q = [round(float(v), 1) for v in robot.get_joints()]
        pause(args, f"\nThe arm is at {q} deg and goes to the start pose {START} at 20 %.")
        if run_moves(robot, [("joints", START, {"speed": 20})], "ruckig") is None:
            return 1
        for s in steps:
            if s == 0:
                pause(args, "\n--- 0: a move with NO profile named (J1 +5 deg and back, 30 %)")
                moves = [("joints", rel([5, 0, 0, 0, 0, 0]), {"speed": 30}), ("joints", START, {"speed": 30})]
                if run_moves(robot, moves, None) is None:
                    return 1
                used = (robot.get_status().get("planner") or {}).get("current_profile") \
                    or robot.status().get("current_profile")          # the profile the planner used last
                if used == "ruckig":
                    print("    The executor's default is 'ruckig'.")
                else:
                    print(f"    !!! The executor planned that move with {used!r}, not 'ruckig'. Something switched the "
                          f"default (robot.set_profile — e.g. an older api_test). tools/restart_gui_and_executor.sh "
                          f"puts it back. The steps below name their profiles, so they are unaffected.")
                continue
            title, watch, moves = STEPS[s]
            times = {}
            for prof in PROFILES:
                pause(args, f"\n--- {s}: {title} — {prof.upper()}\n    watch: {watch}")
                dt = run_moves(robot, moves, prof)
                if dt is None:
                    return 1
                times[prof] = dt
                print(f"    {prof}: {dt:.2f} s (from the first command to settled)")
            print(f"    ruckig took {times['ruckig'] / times['trapezoid']:.2f}x the trapezoid's time")
        print("\nDone — the arm holds at the start pose in EXEC.")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the robot.")
        try:
            robot.stop()
        except Exception:
            pass
        return 1
    finally:
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
