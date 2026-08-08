#!/usr/bin/env python3
"""26: pick-and-place — the gripper and the arm together.

Every gripper piece was verified individually in 06_io.py. This exercises the
COMBINATION, which is what a user actually writes, and one specific property
that nothing else tests:

    **a failed grip must stop the sequence before the arm moves.**

That is the whole point of wait_holding_part(). Run it twice:

    python3 api_tests/26_pick_place.py --assume-ready --go              # with a part
    python3 api_tests/26_pick_place.py --assume-ready --go --no-part    # without

With a part: the full cycle runs and the arm lifts.
Without:     wait_holding_part() must raise GripFailed AND the arm must not move.

The second run is the important one. A pick that reports success it never had is
how a robot travels across a cell holding nothing, or drops a part mid-swing.

MOTION: small joint offsets from wherever the arm starts (default 8 deg on J1,
6 deg on J2), at low speed. It does not go looking for a fixture — you hold or
place the part in the jaws when prompted. That keeps the test about the
SEQUENCING logic rather than about having a jig.
"""

import sys
import time

from _common import (parser, connect, require_go, prepare_exec, finish,
                     main_guard)
from robotclient import GripFailed


def _joints(robot):
    return list(robot.get_joints())


def _describe(label, joints):
    print(f"  {label:<22} " + ", ".join(f"{v:7.2f}" for v in joints))


def run():
    p = parser("26: pick-and-place — gripper and arm together")
    p.add_argument("--no-part", action="store_true",
                   help="run the FAILURE case: expect GripFailed and NO arm motion")
    p.add_argument("--lift-j1", type=float, default=8.0,
                   help="degrees to move J1 for the 'lift' leg (default 8)")
    p.add_argument("--lift-j2", type=float, default=6.0,
                   help="degrees to move J2 for the 'lift' leg (default 6)")
    args = p.parse_args()

    robot = connect(args)
    if not require_go(args, "run a pick-and-place cycle (moves the arm and gripper)"):
        finish(robot)
        return
    prepare_exec(robot, args)

    print("\n=== PICK AND PLACE ===")
    print(f"  mode: {'NO PART (expecting GripFailed)' if args.no_part else 'WITH PART'}")

    start = _joints(robot)
    _describe("start joints (deg)", start)

    # 1. open, so the close has somewhere to travel
    print("\n1. Opening gripper ...")
    robot.gripper_open(timeout=args.timeout)
    robot.wait_gripper_move_complete(timeout=args.timeout)

    # 2. the operator supplies the part (or deliberately does not)
    if args.no_part:
        input("\n2. Leave the jaws EMPTY, then press Enter... ")
    else:
        input("\n2. Place the part between the jaws, then press Enter... ")

    # 3. close and VERIFY — this is the line the whole test exists for
    print("\n3. Closing and verifying the grip ...")
    t0 = time.perf_counter()
    robot.gripper_close(timeout=args.timeout)
    try:
        held = robot.wait_holding_part(timeout=args.timeout)
        grip_ok, grip_err = True, None
        print(f"   holding: {held!r}  ({time.perf_counter() - t0:.2f}s)")
    except GripFailed as exc:
        grip_ok, grip_err = False, exc
        print(f"   GripFailed: {exc}")

    # 4. only move if the grip is real
    before_lift = _joints(robot)
    if grip_ok:
        target = list(before_lift)
        target[0] += args.lift_j1
        target[1] += args.lift_j2
        print(f"\n4. Grip confirmed — lifting (J1 {args.lift_j1:+.1f}, "
              f"J2 {args.lift_j2:+.1f} deg) ...")
        robot.move_joints(target, speed=args.speed, timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)
        print("   lift complete.")
        print("\n5. Releasing ...")
        robot.gripper_open(timeout=args.timeout)
        try:
            print(f"   released: {robot.wait_released_part(timeout=args.timeout)!r}")
        except GripFailed as exc:
            print(f"   wait_released_part raised: {exc}")
        # back where we started, so repeat runs are reproducible
        robot.move_joints(before_lift, speed=args.speed, timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)
    else:
        print("\n4. Grip FAILED — the sequence stops here and the arm does NOT move.")

    after = _joints(robot)
    _describe("joints after (deg)", after)
    moved = max(abs(a - b) for a, b in zip(after, before_lift))
    print(f"  max joint movement since the grip check: {moved:.3f} deg")

    # --- verdict ------------------------------------------------------------
    print("\n=== VERDICT ===")
    if args.no_part:
        if grip_ok:
            print("  *** FAIL — reported a successful grip with EMPTY jaws. ***")
            print("  A pick would proceed carrying nothing.")
        elif moved > 0.5:
            print(f"  *** FAIL — grip failed but the arm still moved {moved:.2f} deg. ***")
            print("  The guard raised but did not actually stop the sequence.")
        else:
            print("  PASS — GripFailed raised AND the arm stayed put.")
            print("  A missed pick cannot become a move. This is the property")
            print("  that makes wait_holding_part() worth having.")
    else:
        if not grip_ok:
            print("  *** FAIL — a part was present but the grip was not detected. ***")
            print(f"  {grip_err}")
        else:
            print("  PASS — full cycle: open, grip verified, lift, release, return.")

    finish(robot)


if __name__ == "__main__":
    main_guard(run)
