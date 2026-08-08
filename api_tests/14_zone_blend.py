#!/usr/bin/env python3
"""14: zone (Bezier corner-cutting) blend — first hardware verification.

Sends a small burst of move_joints (or move_linear with --cartesian) with
blend_mode="zone", so the planner buffers them and flushes one continuous
Bezier-blended trajectory that CUTS the corners instead of stopping at each
waypoint. See TODO/blending_design.md.

SAFETY / usage notes:
  * Conservative defaults (small deltas, low speed). Start small; scale up only
    once you've seen it behave.
  * Zone blending requires profile="toppra" — this script forces it and warns
    if you override to anything else (it would silently fall back to individual,
    non-blended moves).
  * Must run in ASYNC mode (the default). --sync makes every move block until
    done, which defeats buffering (no blend). The script refuses --sync.
  * The moves are fired as a rapid burst (no wait between) on purpose — that's
    what makes them land in the planner's blend buffer together. wait_idle at
    the end waits for the whole chain.
  * Cross-check while it runs:  tail -f logs/activity_exec.log
    You should see CMD_QUEUED x N, then CMD_STARTED (with a "context" showing
    blend_strategy/profile), then CMD_COMPLETED x N for the chain.
"""
import time

from _common import (parser, connect, require_go, prepare_exec, finish, main_guard,
                     goto_start)


def run():
    p = parser("14: zone (Bezier corner-cutting) blend, small hardware test")
    p.add_argument("--blend-mm", type=float, default=10.0,
                   help="zone blend radius in millimetres (clamped to <= half the shorter "
                        "adjacent TCP segment). Larger = rounder corner.")
    p.add_argument("--deg", type=float, default=6.0,
                   help="per-waypoint joint delta in degrees (joint-space test)")
    p.add_argument("--cartesian", action="store_true",
                   help="run a move_linear (Cartesian) zone-blend instead of joint-space")
    p.add_argument("--rect", action="store_true",
                   help="trace a closed Cartesian RECTANGLE (4 corners) — clearest way to see "
                        "rounded vs sharp corners. Uses move_linear; --plane / --rect-mm size it.")
    p.add_argument("--plane", default="xz", choices=["xy", "xz"],
                   help="rectangle plane (--rect): 'xz' = vertical, 'xy' = horizontal")
    p.add_argument("--rect-mm2", type=float, default=None,
                   help="SECOND rectangle side length in mm (--rect). Default None = square. "
                        "USE THIS to exercise asymmetric blend fractions: the planner sets "
                        "frac_before = r/seg_before and frac_after = r/seg_after, which are only "
                        "equal when the two adjacent segments are the same length. A square makes "
                        "them equal at every corner, which is the one case that was always "
                        "well-posed — so a square could never show the blending_design.md §9.4 "
                        "bug, nor confirm its fix. Pass e.g. --rect-mm 100 --rect-mm2 40.")
    p.add_argument("--rect-mm", type=float, default=100.0,
                   help="rectangle side length in mm (--rect). Needs --samples dense enough for "
                        "the pose's IK conditioning, or corners can trip false-positive "
                        "IK_BRANCH_JUMP rejections even on a reachable path — see --samples.")
    p.add_argument("--dxy-mm", type=float, default=25.0,
                   help="per-waypoint Cartesian leg length in mm (--cartesian only)")
    p.add_argument("--joint-rect", action="store_true",
                   help="trace a closed RECTANGLE in JOINT space (4 corners, returns to start) "
                        "using plain move_joints — no IK, no Cartesian linear segments, just two "
                        "joint axes stepped like the Cartesian --rect's X/Y legs. --joint-axes / "
                        "--joint-rect-deg size it.")
    p.add_argument("--joint-axes", default="0,1",
                   help="comma-separated pair of joint indices (0-based) to form the rectangle "
                        "in (--joint-rect). Default '0,1' = base+shoulder.")
    p.add_argument("--joint-rect-deg", type=float, default=15.0,
                   help="rectangle side length in degrees (--joint-rect). Push this and/or "
                        "--blend-mm toward the half-segment clamp to see rounding clearly, same "
                        "as --rect-mm for the Cartesian version.")
    p.add_argument("--joint-rect-deg2", type=float, default=None,
                   help="SECOND rectangle side length in degrees (--joint-rect). Default None = "
                        "square. Same purpose as --rect-mm2: unequal sides are what produce "
                        "ASYMMETRIC blend fractions, the case blending_design.md §9.4 was about. "
                        "Pass e.g. --joint-rect-deg 20 --joint-rect-deg2 8.")
    p.add_argument("--samples", type=int, default=50,
                   help="explicit IK sample count per Cartesian leg (--rect / --cartesian). "
                        "The server's default adaptive density (~9mm/step) is too coarse for "
                        "some moderately-conditioned poses and trips false-positive "
                        "IK_BRANCH_JUMP rejections on an otherwise reachable path; a denser "
                        "explicit count fixes it. Verified 50 succeeds where the default fails.")
    p.add_argument("--baseline", action="store_true",
                   help="also run the same path with blend=0 first, so you can compare "
                        "the stop-at-each-corner motion against the blended one")
    p.add_argument("--baseline-only", action="store_true",
                   help="run ONLY the blend=0 (stop-at-every-corner) version — for a clean "
                        "side-by-side against a separate zone-only run")
    args = p.parse_args()

    if args.sync:
        raise SystemExit("ERROR: zone blending needs async mode; do NOT pass --sync "
                         "(it blocks each move to completion, so nothing buffers/blends).")
    if args.profile != "toppra":
        print(f"WARNING: profile={args.profile!r} — zone blending requires 'toppra'. "
              f"Forcing profile='toppra' for this test.")
    args.profile = "toppra"

    robot = connect(args)
    action = ("trace a Cartesian rectangle" if args.rect
              else "trace a joint-space rectangle" if args.joint_rect
              else "run a Cartesian zone-blend" if args.cartesian
              else "run a joint-space zone-blend")
    if require_go(args, action):
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")
        # Always begin from the canonical pose. Without this the joint rectangle is
        # built off get_joints(), so its geometry — and therefore its duration —
        # depends on wherever the arm happened to stop last, which makes runs
        # incomparable. It also protects the Cartesian lane: the arm is often parked
        # at J5=0 (the exact wrist singularity), where the rectangle's IK cannot solve.
        goto_start(robot, args)

        if args.rect:
            _run_rectangle(robot, args)
        elif args.joint_rect:
            _run_joint_rectangle(robot, args)
        elif args.cartesian:
            _run_cartesian(robot, args)
        else:
            _run_joint(robot, args)

    finish(robot)


def _report_duration(elapsed, blend, cartesian):
    """Print how long the traced path actually took.

    Without this the only way to check a blend against its predicted duration was to
    dig timestamps out of logs/activity_exec.log by hand. The number to compare
    against is the planner's own TOPPRA solve, which logs
    "Successfully parametrize path. Duration: X" — wall clock here runs a little
    longer than that because the "settled" completion policy waits for the arm to
    come to rest at the end.

    Cartesian moves are additionally capped by the planner at
    CARTESIAN_MOVE_MAX_SPEED_SCALE (0.20), so --speed above 20% has NO effect on a
    Cartesian path — worth knowing before you conclude a blend is slow.
    """
    label = f"blend={blend:g}mm" if blend > 0 else "blend=0 (sharp corners)"
    print(f"  -> {label}: {elapsed:.2f}s wall clock "
          f"(includes the 'settled' wait at the end)")
    if cartesian:
        print("     note: Cartesian speed is capped at 20% by the planner "
              "(CARTESIAN_MOVE_MAX_SPEED_SCALE), regardless of --speed.")


def _run_rectangle(robot, args):
    s = args.rect_mm / 1000.0                                    # mm -> m
    s2 = (args.rect_mm2 if args.rect_mm2 is not None else args.rect_mm) / 1000.0
    # Four world-frame relative legs forming a closed rectangle (returns to start).
    if args.plane == "xz":
        legs = [(s, 0, 0), (0, 0, s2), (-s, 0, 0), (0, 0, -s2)]  # vertical rectangle
    else:  # xy
        legs = [(s, 0, 0), (0, s2, 0), (-s, 0, 0), (0, -s2, 0)]  # horizontal rectangle

    if args.rect_mm2 is None:
        print(f"Cartesian rectangle: {args.rect_mm:.0f}mm SQUARE in the {args.plane.upper()} plane "
              f"(4 corners, returns to start).")
        print("  NOTE: equal sides -> SYMMETRIC blend fractions at every corner. That is the one "
              "case that was always well-posed, so this cannot exercise blending_design.md §9.4. "
              "Pass --rect-mm2 for unequal sides.")
    else:
        print(f"Cartesian rectangle: {args.rect_mm:.0f}mm x {args.rect_mm2:.0f}mm in the "
              f"{args.plane.upper()} plane (4 corners, returns to start).")
        print("  Unequal sides -> ASYMMETRIC blend fractions (frac_before != frac_after). This is "
              "the blending_design.md §9.4 case: before the fix these corners braked for curvature "
              "that doesn't exist (up to 27% slow).")
    print("  Involves IK — if a corner is unreachable/near-singular it falls back to individual "
          "moves (watch logs/high_level.log).")

    def trace(blend):
        t0 = time.perf_counter()
        for dx, dy, dz in legs:
            kw = dict(profile="toppra", speed=args.speed, samples=args.samples)
            if blend > 0:
                kw.update(blend_mode="zone", blend=blend)
            robot.move_linear_rel_world(dx, dy, dz, **kw)
        robot.wait_idle(timeout=args.timeout)
        _report_duration(time.perf_counter() - t0, blend, cartesian=True)

    if args.baseline or args.baseline_only:
        print("\n[baseline] blend=0 (SHARP corners — stops at each):")
        trace(0)
        if args.baseline_only:
            return
    print(f"\n[zone] blend_mode='zone', blend={args.blend_mm}mm (ROUNDED corners — flows through):")
    trace(args.blend_mm)


def _run_joint_rectangle(robot, args):
    axes = [int(x) for x in args.joint_axes.split(",")]
    if len(axes) != 2:
        raise SystemExit("--joint-axes needs exactly two comma-separated indices, e.g. '0,1'")
    a, b = axes
    d = args.joint_rect_deg
    d2 = args.joint_rect_deg2 if args.joint_rect_deg2 is not None else d
    start = [float(v) for v in robot.get_joints()]

    # Four legs forming a closed rectangle in the (axis a, axis b) joint-angle
    # plane, same leg pattern as the Cartesian --rect's (+X,+Y,-X,-Y): step a,
    # step b, step -a, step -b — returns exactly to start. Pure move_joints,
    # no IK/Cartesian path involved at all.
    p1 = list(start); p1[a] += d
    p2 = list(p1);    p2[b] += d2
    p3 = list(p2);    p3[a] -= d
    if args.joint_rect_deg2 is None:
        print(f"Joint-space rectangle: {d:.1f}deg SQUARE in the J{a}/J{b} plane "
              f"(4 corners, returns to start).")
        print("  NOTE: equal sides -> SYMMETRIC blend fractions; cannot exercise "
              "blending_design.md §9.4. Pass --joint-rect-deg2 for unequal sides.")
    else:
        print(f"Joint-space rectangle: {d:.1f}deg x {d2:.1f}deg in the J{a}/J{b} plane "
              f"(4 corners, returns to start).")
        print("  Unequal sides -> ASYMMETRIC blend fractions: the blending_design.md §9.4 case.")
    print("  start ->", [round(v, 2) for v in start])
    print("  p1    ->", [round(v, 2) for v in p1])
    print("  p2    ->", [round(v, 2) for v in p2])
    print("  p3    ->", [round(v, 2) for v in p3])
    print("  back  ->", [round(v, 2) for v in start])

    def trace(blend):
        t0 = time.perf_counter()
        for wp in (p1, p2, p3, start):
            kw = dict(profile="toppra", speed=args.speed)
            if blend > 0:
                kw.update(blend_mode="zone", blend=blend)
            robot.move_joints(wp, **kw)
        robot.wait_idle(timeout=args.timeout)
        _report_duration(time.perf_counter() - t0, blend, cartesian=False)

    if args.baseline or args.baseline_only:
        print("\n[baseline] blend=0 (SHARP corners — stops at each):")
        trace(0)
        if args.baseline_only:
            return
    print(f"\n[zone] blend_mode='zone', blend={args.blend_mm}mm (ROUNDED corners — flows through):")
    trace(args.blend_mm)


def _run_joint(robot, args):
    start = [float(v) for v in robot.get_joints()]
    d = args.deg
    # L-shaped path with two interior corners (at p1 and p2), then back to start.
    p1 = list(start); p1[0] += d
    p2 = list(p1);    p2[1] += d
    print("Joint-space path (degrees):")
    print("  start ->", [round(v, 2) for v in start])
    print("  p1    ->", [round(v, 2) for v in p1])
    print("  p2    ->", [round(v, 2) for v in p2])
    print("  back  ->", [round(v, 2) for v in start])

    if args.baseline or args.baseline_only:
        print("\n[baseline] same path, blend=0 (STOPS at every corner):")
        for wp in (p1, p2, start):
            robot.move_joints(wp, profile="toppra", speed=args.speed)
        robot.wait_idle(timeout=args.timeout)
        if args.baseline_only:
            return

    print(f"\n[zone] same path, blend_mode='zone', blend={args.blend_mm}mm (FLOWS THROUGH corners):")
    for wp in (p1, p2, start):
        robot.move_joints(wp, profile="toppra", speed=args.speed,
                          blend_mode="zone", blend=args.blend_mm)
    robot.wait_idle(timeout=args.timeout)


def _run_cartesian(robot, args):
    leg = args.dxy_mm / 1000.0  # mm -> m
    print(f"Cartesian path: three {args.dxy_mm}mm world-frame legs from the current pose "
          f"(+X, +Y, -X), corners zone-blended at blend={args.blend_mm}mm.")
    print("  NOTE: Cartesian involves IK — if any waypoint is unreachable/near-singular the "
          "planner falls back to individual (non-blended) moves; watch logs/high_level.log.")
    if args.baseline or args.baseline_only:
        print("\n[baseline] blend=0 (STOPS at every corner):")
        robot.move_linear_rel_world(leg, 0.0, 0.0, profile="toppra", speed=args.speed, samples=args.samples)
        robot.move_linear_rel_world(0.0, leg, 0.0, profile="toppra", speed=args.speed, samples=args.samples)
        robot.move_linear_rel_world(-leg, 0.0, 0.0, profile="toppra", speed=args.speed, samples=args.samples)
        robot.wait_idle(timeout=args.timeout)
        if args.baseline_only:
            return

    print(f"\n[zone] blend_mode='zone', blend={args.blend_mm}mm (cuts corners):")
    robot.move_linear_rel_world(leg, 0.0, 0.0, profile="toppra", speed=args.speed, samples=args.samples,
                                blend_mode="zone", blend=args.blend_mm)
    robot.move_linear_rel_world(0.0, leg, 0.0, profile="toppra", speed=args.speed, samples=args.samples,
                                blend_mode="zone", blend=args.blend_mm)
    robot.move_linear_rel_world(-leg, 0.0, 0.0, profile="toppra", speed=args.speed, samples=args.samples,
                                blend_mode="zone", blend=args.blend_mm)
    robot.wait_idle(timeout=args.timeout)


if __name__ == "__main__":
    main_guard(run)
