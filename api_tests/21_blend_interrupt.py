#!/usr/bin/env python3
"""21: pause / resume / stop in the MIDDLE of a blended chain.

WHY THIS EXISTS. Every interrupt test we had (11_pause_resume, 12_stop) interrupts a SINGLE
move. A blended chain is a different animal: the planner sends many batches for one continuous
trajectory, and `ruckig_corner` sends the WHOLE chain at once (it is not spliceable, so there
is no held tail to discard). That means RTI is holding a long trajectory in its buffer when the
interrupt lands.

THE CHECK THAT MATTERS is not "did the arm stop" — it is:

    after stop(), is RTI's trajectory buffer actually EMPTY?

If it is not, the next motion command replays stale samples and THE ARM JUMPS. That is the
failure this test is written to catch, and it is why it verifies `exec_samples_remaining`
straight from shared memory and then drives a known recovery move to prove the arm goes where
it was told and nowhere else.

The second thing it pins: a DELIBERATE stop must not poison the next wait_idle(). Cancelling
the queue makes the server reply STOPPED for every un-executed command, and on a blended chain
one stop cancels MANY at once. Those replies are the expected consequence of stopping, not
motion failures, and client.stop() drains them. Without that the next wait_idle() would raise
MotionFailed about commands the operator chose to cancel — a real bug once, now pinned across
a chain rather than a single move.

Runs against BOTH blend lanes (--mode), because neither had been interrupted mid-chain before.

SAFETY: small moves, low speed by default. --go required.
"""
import importlib.util
import time
from pathlib import Path

import numpy as np

from _common import (parser, connect, require_go, prepare_exec, finish, main_guard,
                     goto_start, show, START_POSE_DEG)
from robotclient import CommandRejected
from robotclient.exceptions import MotionFailed

from data import shared_data

# Reuse 20's preflight joint-limit guard rather than duplicating it — a chain with non-default
# --moves/--deg can march J0 straight out of travel, and finding that out from a planner
# rejection halfway through a stop test is exactly what we do not want.
# (Module name starts with a digit, so it cannot be imported by name.)
_spec = importlib.util.spec_from_file_location(
    "ruckig_corner_test", Path(__file__).with_name("20_ruckig_corner.py"))
_rc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_rc)
assert_within_joint_limits = _rc._assert_within_joint_limits


MODES = {
    # blend_mode, profile
    "ruckig_corner": ("ruckig_corner", "ruckig"),
    "zone": ("zone", "toppra"),
}


def exec_samples_remaining():
    """What RTI still has buffered. 0 = nothing left to execute."""
    return float(shared_data.rti["exec_samples_remaining"][0])


def chain_waypoints(moves, deg):
    """A zigzag long enough that an interrupt lands mid-chain rather than after it."""
    wps = []
    base = list(START_POSE_DEG)
    for i in range(moves):
        q = list(base)
        q[0] += (i + 1) * deg
        q[1] += deg if i % 2 == 0 else 0.0
        wps.append(q)
    return wps


def fire_chain(robot, wps, blend_mode, profile, speed, blend_mm):
    for q in wps:
        robot.move_joints(*q, profile=profile, blend_mode=blend_mode,
                          blend=blend_mm, speed=speed)


def run():
    p = parser("21: pause/resume/stop in the middle of a BLENDED chain")
    p.add_argument("--mode", default="ruckig_corner", choices=sorted(MODES),
                   help="which blend lane to interrupt")
    p.add_argument("--moves", type=int, default=6, help="moves in the chain")
    p.add_argument("--deg", type=float, default=8.0, help="degrees per move")
    p.add_argument("--blend-mm", type=float, default=15.0)
    p.add_argument("--interrupt-after", type=float, default=2.0,
                   help="seconds into the chain before pausing / stopping")
    p.add_argument("--hold", type=float, default=1.5, help="seconds to stay paused")
    p.add_argument("--drift-tol", type=float, default=0.25,
                   help="max joint drift allowed while paused, degrees")
    args = p.parse_args()

    if args.sync:
        raise SystemExit("REFUSING --sync: it blocks each move, so no chain ever buffers and "
                         "there is nothing blended to interrupt.")

    blend_mode, profile = MODES[args.mode]
    wps = chain_waypoints(args.moves, args.deg)
    assert_within_joint_limits(wps, f"{args.moves}-move chain")

    if not require_go(args, f"pause/resume AND stop during a {args.moves}-move "
                            f"blend_mode='{blend_mode}' chain at {args.speed}% speed"):
        return

    failures = []

    def check(name, cond, detail="", always=""):
        # `detail` explains a FAILURE, so only print it when it failed — printing it on a PASS
        # produces nonsense like "PASS ... wait_idle raised after a deliberate stop: None".
        # `always` is for a number worth seeing either way.
        note = always if cond else detail
        print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f" — {note}" if note else ""))
        if not cond:
            failures.append(name)

    robot = connect(args)
    try:
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")

        # ---------------------------------------------------------------- PAUSE / RESUME
        print(f"\n{'=' * 70}\nPAUSE / RESUME mid-chain — {blend_mode}\n{'=' * 70}")
        goto_start(robot, args)
        robot.wait_idle()

        fire_chain(robot, wps, blend_mode, profile, args.speed, args.blend_mm)
        time.sleep(max(0.0, args.interrupt_after))

        buffered_before = exec_samples_remaining()
        print(f"  pausing (RTI has {buffered_before:.0f} samples buffered)")
        robot.pause(wait=True)
        robot.wait(lambda s: bool(s.get("exec_paused")) is True, timeout=args.timeout,
                   what="paused")
        paused = np.array([float(v) for v in robot.get_joints()])
        check("chain PAUSED mid-motion (still had work buffered)", buffered_before > 0,
              always=f"{buffered_before:.0f} samples")

        time.sleep(max(0.0, args.hold))
        held = np.array([float(v) for v in robot.get_joints()])
        drift = float(np.abs(held - paused).max())
        check(f"arm HOLDS while paused (drift <= {args.drift_tol} deg)",
              drift <= args.drift_tol, always=f"drift {drift:.4f} deg")

        print("  resuming")
        robot.resume(wait=True)
        robot.wait(lambda s: bool(s.get("exec_paused")) is False, timeout=args.timeout,
                   what="resumed")
        robot.wait_idle(timeout=120.0)

        final = np.array([float(v) for v in robot.get_joints()])
        want = np.array(wps[-1])
        err = float(np.abs(final - want).max())
        check("chain COMPLETES after resume, landing on the final waypoint",
              err < 0.5, always=f"worst joint error {err:.3f} deg")
        check("RTI buffer empty after the chain finished",
              exec_samples_remaining() < 1.0, f"{exec_samples_remaining():.0f} samples left")

        # ---------------------------------------------------------------- STOP
        print(f"\n{'=' * 70}\nSTOP mid-chain — {blend_mode}  (the dangerous one)\n{'=' * 70}")
        goto_start(robot, args)
        robot.wait_idle()

        fire_chain(robot, wps, blend_mode, profile, args.speed, args.blend_mm)
        time.sleep(max(0.0, args.interrupt_after))

        buffered = exec_samples_remaining()
        print(f"  stopping (RTI has {buffered:.0f} samples buffered — these MUST be discarded)")
        check("stop lands mid-chain, with a real trajectory still buffered", buffered > 0,
              always=f"{buffered:.0f} samples")

        robot.stop(wait=True)
        status = robot.get_status()
        check("mode is IDLE after stop", status.get("mode") == "IDLE",
              f"mode={status.get('mode')!r}")

        # THE CHECK THIS TEST EXISTS FOR. A chain is sent WHOLE, so a stop that does not flush
        # RTI leaves a long stale trajectory sitting in its buffer, and the next move replays it.
        time.sleep(0.3)
        left = exec_samples_remaining()
        check("RTI's trajectory buffer was FLUSHED by stop (no stale samples)",
              left < 1.0,
              f"{left:.0f} samples still buffered — the next move would REPLAY them",
              always="0 samples left")

        stopped = np.array([float(v) for v in robot.get_joints()])
        print(f"  stopped at: {[round(v, 2) for v in stopped]}")

        # A DELIBERATE stop must NOT poison the next wait_idle(). Cancelling the queue makes
        # the server reply "error"/STOPPED for every un-executed command, and those replies are
        # the EXPECTED CONSEQUENCE of stopping, not motion failures — so client.stop() drains
        # them (robotclient/client.py, `self._conn.take_motion_errors()`). Without that, the
        # very next wait_idle() would raise MotionFailed about commands the operator chose to
        # cancel. That was a real bug once; this pins the fix, now also across a BLENDED chain
        # where a single stop cancels many commands at once.
        #
        # (A motion that fails for a REAL reason still surfaces — 16_motion_failure.py covers
        # that, and the two must not be conflated.)
        poisoned = None
        try:
            robot.wait_idle(timeout=5.0)
        except MotionFailed as exc:
            poisoned = str(exc)
        except CommandRejected as exc:
            poisoned = f"{exc.code}: {exc.message}"
        check("a deliberate stop does NOT poison the next wait_idle()",
              poisoned is None,
              f"wait_idle raised after a deliberate stop: {str(poisoned)[:90]}")

        # ------------------------------------------------ recovery: does the arm still obey?
        if status.get("state") != "ENABLED" or status.get("homed") is not True:
            print("\n  not ENABLED+homed after stop — skipping the recovery move")
        else:
            print("\n  RECOVERY: re-enter EXEC and drive a small KNOWN move.")
            print("  If a stale batch survived the stop, the arm goes somewhere else — that is")
            print("  the failure this proves is absent.")
            robot.enter_exec_mode(wait=True)
            target = list(stopped)
            target[1] += 3.0                      # a small, unambiguous J1 nudge
            robot.move_joints(*target, profile="trapezoid", speed=args.start_speed)
            robot.wait_idle(timeout=60.0)
            landed = np.array([float(v) for v in robot.get_joints()])
            rerr = float(np.abs(landed - np.array(target)).max())
            check("after stop, a fresh move goes EXACTLY where commanded (no stale replay)",
                  rerr < 0.5, always=f"worst joint error {rerr:.3f} deg")

            goto_start(robot, args)
            robot.wait_idle()

        print(f"\n{'=' * 70}")
        if failures:
            print(f"{len(failures)} FAILURE(S): " + ", ".join(failures))
            raise SystemExit(1)
        print(f"ALL PASS — {blend_mode} survives pause/resume and stop mid-chain.")
    finally:
        finish(robot)


if __name__ == "__main__":
    main_guard(run)
