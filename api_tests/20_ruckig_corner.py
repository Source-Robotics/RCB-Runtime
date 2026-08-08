#!/usr/bin/env python3
"""20: ruckig_corner — jerk-limited blending, hardware verification.

WHAT THIS IS. `blend_mode="ruckig_corner"` (EXPERIMENTAL, opt-in) does not prescribe the
corner geometry the way `zone` does. It plans straight Ruckig legs, cuts them where they
cross a ball around the waypoint, and lets Ruckig invent the corner as a full-state
(q, qd, qdd) transition. Seams are C2 by construction and the WHOLE move is jerk-limited —
the only lane that bounds jerk through a blend. It pays for that in TIME.

WHAT TO EXPECT WITH YOUR EYES: the arm should round the corners and never stop mid-chain,
same as `zone`, but move NOTICEABLY MORE GENTLY into and out of them — no snap at the corner
entry/exit. And it will take LONGER than the same chain under `zone` (measured offline:
+44% on one corner, +71% on three). That slowdown is the feature working, not a fault.

SAFETY / usage:
  * Conservative defaults (small deltas, low speed). Scale up only once you have seen it.
  * Requires profile="ruckig" — the script forces it. Any other profile makes the planner
    fall back to individual, UNBLENDED moves (it says so in the log; it does not pretend).
  * JOINT SPACE ONLY. A Cartesian chain falls back to individual moves by design — the legs
    of a move_linear must track a prescribed line and Ruckig would carve an arc instead.
    Measured, with the reasoning: dummy_test_scripts/probe_ruckig_corner_cartesian.py.
  * NOT SPLICEABLE. A ruckig_corner chain is sent whole, so it does not join the rolling
    buffer. Commands must arrive together (a burst) to blend; a slow trickle will stop
    between flushes. That is why the moves below are fired back-to-back with no waits.
  * Must run ASYNC (the default). --sync blocks each move and defeats buffering entirely,
    so there would be nothing to blend — the script refuses it.
  * --compare runs the SAME geometry under blend_mode="zone" first, so you can watch both
    and compare the wall-clock times the script prints.

Cross-check while it runs:  tail -f logs/activity_exec.log
  Expect CMD_QUEUED xN, then CMD_STARTED with a context showing
  blend_strategy="ruckig_corner", then CMD_COMPLETED xN for the chain.
"""
import importlib.util
import time
from pathlib import Path

from _common import (parser, connect, require_go, prepare_exec, finish, main_guard,
                     goto_start, show, START_POSE_DEG)

# Reuse 19_profile_bench's shared-memory sampler rather than re-deriving it. It reads
# joint_speed_commanded / joint_acceleration_commanded — what actually went to the motors —
# KEYED TO loop_tick, so dt is exact and a dropped poll cannot manufacture a jerk spike.
# (Module name starts with a digit, so it cannot be imported by name.)
_spec = importlib.util.spec_from_file_location(
    "profile_bench", Path(__file__).with_name("19_profile_bench.py"))
_bench = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_bench)
ShmSampler, run_and_sample = _bench.ShmSampler, _bench._run_and_sample


def _corner_waypoints(axes, deg, deg2, teeth=6):
    """L (1 corner), rectangle (3 corners), or ZIGZAG (many alternating sharp corners).

    The zigzag is the one to watch. Every waypoint is a hard direction reversal, so it is
    ALL corner and almost no straight leg — the shape that shows a blend doing (or not
    doing) its job. It is also the WORST case for ruckig_corner's cost: short legs cannot
    amortize a jerk-limited ramp, so expect the biggest time penalty here of any shape.
    """
    a, b = axes
    side2 = deg if deg2 is None else deg2
    base = list(START_POSE_DEG)

    def at(da, db):
        q = list(base)
        q[a] += da
        q[b] += db
        return q

    # Advance steadily along axis a while alternating axis b up/down: a sawtooth.
    zig = [at((i + 1) * deg, side2 if i % 2 == 0 else 0.0) for i in range(teeth)]

    return {
        "L": [at(deg, 0.0), at(deg, side2)],
        "rect": [at(deg, 0.0), at(deg, side2), at(0.0, side2), at(0.0, 0.0)],
        "zigzag": zig + [at((teeth + 1) * deg, 0.0)],   # settle back on the centre line
    }


def square_waypoints(side_mm, plane):
    """A real square IN SPACE — corners IK'd, then traversed with JOINT-space legs.

    READ THIS BEFORE BELIEVING THE SHAPE. ruckig_corner is joint-space only: it cannot run
    move_linear, because a Cartesian leg is a PRESCRIBED straight line and Ruckig would carve
    an arc instead (blending_design.md §10.2). So this square hits its four corners EXACTLY —
    they are solved by IK — but the EDGES BOW, because a straight line in joint space is not
    a straight line in space. Measured on this arm: ~2.5mm of bow on a 60mm side, ~8.9mm on
    100mm, ~19mm on 150mm. The script measures and prints the real number for YOUR square, so
    nobody has to take my word for it.

    Want genuinely straight edges? That is `move_linear` + `blend_mode="zone"` — a different
    lane, and `api_tests/14_zone_blend.py --rect` already does it.

    Starts at the MIDPOINT of an edge and returns there, so all FOUR corners are interior and
    therefore blendable (starting at a corner would leave the closing one as a dead stop).
    """
    import numpy as np
    from spatialmath import SE3
    from motion.ik_config import DEFAULT_IK_CONFIG
    from motion.ik_path import SeededIKPathSolver
    from robotics.robot_kinematics import calculate_forward_kinematics
    from utility.robot_factory import create_robot
    from utility.pinocchio_attach import attach_pinocchio
    import config.xml_parser as xml_parser
    from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR

    system = xml_parser.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xml_parser.load_full_robot_config(
        str(system["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
        active_gripper=system["active_gripper"])
    robot = create_robot(system["active_robot"], gripper_name=cfg.get("gripper_name", ""))
    attach_pinocchio(robot)

    q_start = np.radians(START_POSE_DEG)
    pose0 = calculate_forward_kinematics(robot, q_start)
    s = side_mm / 1000.0

    # Offsets in the chosen plane, starting from the midpoint of the bottom edge and going
    # round the square once, back to the midpoint.
    mid = s / 2.0
    loop = [(mid, 0.0), (s, 0.0), (s, s), (0.0, s), (0.0, 0.0), (mid, 0.0)]

    def target(u, v):
        return (SE3.Trans(u, 0.0, v) if plane == "xz" else SE3.Trans(u, v, 0.0)) * pose0

    solver = SeededIKPathSolver(robot, 6, solver="pinocchio", config=DEFAULT_IK_CONFIG,
                                kinematics_backend="pinocchio")
    joints, seed = [], q_start
    for u, v in loop:
        sol = solver.solve_pose(target(u, v), seed)
        if sol is None:
            raise SystemExit(
                f"REFUSING: IK failed for a {side_mm}mm square corner in the {plane} plane "
                f"from START — nothing was sent. Try a smaller --side-mm or the other --plane.")
        seed = np.asarray(sol, dtype=float)
        joints.append(seed.copy())

    # Measure the real bow of a joint-space leg, so the print is a fact, not a promise.
    worst_bow_mm = 0.0
    for a, b in zip(joints[:-1], joints[1:]):
        pts = np.array([calculate_forward_kinematics(robot, a + t * (b - a)).t
                        for t in np.linspace(0.0, 1.0, 40)])
        u_hat = (pts[-1] - pts[0]) / np.linalg.norm(pts[-1] - pts[0])
        d = pts - pts[0]
        worst_bow_mm = max(worst_bow_mm,
                           float(np.linalg.norm(d - np.outer(d @ u_hat, u_hat), axis=1).max()) * 1000)

    return [list(np.degrees(q)) for q in joints], worst_bow_mm


def _assert_within_joint_limits(waypoints, shape):
    """Reject an out-of-travel shape HERE, before a single command is sent.

    A zigzag marches (teeth+1) x deg along the first axis, and J0 only has ~74 deg of
    headroom from START. Without this the planner would reject a move mid-chain, the arm
    would stop somewhere unplanned, and the run would look like a blending failure when
    it is really a reachability one. Fail loudly, at the shape, naming the joint.
    """
    import numpy as np
    import config.xml_parser as xml_parser
    from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR

    system = xml_parser.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xml_parser.load_full_robot_config(
        str(system["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
        active_gripper=system["active_gripper"])
    n = int(cfg["joint_num"])
    lo = np.degrees(np.asarray(cfg["software_joint_limit_negative"][:n], dtype=float))
    hi = np.degrees(np.asarray(cfg["software_joint_limit_positive"][:n], dtype=float))

    bad = []
    for w, q in enumerate(waypoints):
        for j in range(n):
            if not (lo[j] <= q[j] <= hi[j]):
                bad.append(f"waypoint {w + 1} J{j}={q[j]:.1f} deg outside "
                           f"[{lo[j]:.1f}, {hi[j]:.1f}]")
    if bad:
        raise SystemExit(
            f"REFUSING: the {shape} leaves the software joint limits — nothing was sent.\n  "
            + "\n  ".join(bad)
            + "\n\nShrink it: fewer --teeth, a smaller --deg (advance per tooth), a smaller "
              "--deg2 (amplitude), or --axes onto joints with more travel.")


def _run_chain(robot, waypoints, blend_mm, blend_mode, profile, speed, label, **extra):
    """Fire the whole chain as a burst so the planner buffers and blends it.

    Sampled from shared memory while it runs, so we report the COMMANDED jerk that actually
    went to the motors — the whole point of this lane. Wall clock alone cannot tell you
    whether the jerk bound was honoured.
    """
    print(f"\n-- {label}: {len(waypoints)} moves, blend_mode={blend_mode!r}, "
          f"profile={profile!r}, blend={blend_mm}mm, speed={speed}%"
          + (f", jerk={extra['jerk']}%" if "jerk" in extra else "") + " --")

    def fire():
        for q in waypoints:
            robot.move_joints(*q, profile=profile, blend_mode=blend_mode,
                              blend=blend_mm, speed=speed, **extra)

    wall, stats = run_and_sample(robot, fire)
    if stats:
        print(f"   {wall:.2f}s wall | commanded peak|jerk| {stats['peak_jerk_cmd']:8.1f} "
              f"rms {stats['rms_jerk_cmd']:7.1f} rad/s^3 | peak|a| {stats['peak_acc_cmd']:.2f} "
              f"rad/s^2 | peak|v| {stats['peak_vel_cmd']:.3f} rad/s")
        if stats["missed"]:
            print(f"   (note: {stats['missed']} tick(s) missed by the sampler)")
    else:
        print(f"   {wall:.2f}s wall | (too few samples to compute jerk)")
    return wall, stats


def run():
    p = parser("20: ruckig_corner jerk-limited blending, hardware test")
    p.add_argument("--blend-mm", type=float, default=15.0,
                   help="blend radius in millimetres — the ball around the waypoint that the "
                        "corner replaces. Same unit as blend_mode='zone'. Larger = rounder, "
                        "and the corner is entered faster.")
    p.add_argument("--deg", type=float, default=None,
                   help="leg length in degrees (zigzag: advance per tooth). "
                        "Default 12 for L/rect, 8 for --zigzag (which marches "
                        "(teeth+1) x deg along the first axis and would otherwise run out of "
                        "joint travel).")
    p.add_argument("--deg2", type=float, default=None,
                   help="second leg length, degrees (default = --deg, i.e. equal legs). Unequal "
                        "legs give ASYMMETRIC cut radii, which is the harder case — use it.")
    p.add_argument("--axes", default="0,1",
                   help="comma-separated pair of joint indices (0-based) forming the corner")
    p.add_argument("--rect", action="store_true",
                   help="closed rectangle (3 blended corners + return) instead of a single L")
    p.add_argument("--zigzag", action="store_true",
                   help="ZIGZAG / sawtooth: --teeth alternating sharp reversals. The clearest "
                        "shape to WATCH a blend work — it is all corner and almost no straight "
                        "leg. Also the worst case for ruckig_corner's time cost (short legs "
                        "cannot amortize a jerk-limited ramp), so expect the largest penalty. "
                        "--deg = advance per tooth, --deg2 = tooth amplitude.")
    p.add_argument("--teeth", type=int, default=6,
                   help="number of zigzag teeth (--zigzag). Each is a sharp reversal.")
    p.add_argument("--square", action="store_true",
                   help="A SQUARE IN SPACE: the 4 corners are IK'd so the tool hits them "
                        "exactly, then traversed with JOINT-space legs (all 4 corners get "
                        "blended). NOTE the edges BOW — a straight line in joint space is not "
                        "straight in space — and the script MEASURES and prints the bow. "
                        "ruckig_corner cannot do straight Cartesian edges; for those use "
                        "move_linear + blend_mode='zone' (api_tests/14_zone_blend.py --rect).")
    p.add_argument("--side-mm", type=float, default=80.0,
                   help="square side length in mm (--square). Bigger = more edge bow "
                        "(~2.5mm at 60mm, ~8.9mm at 100mm on this arm).")
    p.add_argument("--plane", default="xz", choices=["xz", "xy"],
                   help="plane of the square (--square): xz = vertical, xy = horizontal")
    p.add_argument("--jerk", type=float, default=None,
                   help="percent of the default jerk budget (3x the accel limit). Lower = "
                        "gentler. THIS is the knob that no other blend mode has.")
    p.add_argument("--compare", action="store_true",
                   help="run the same geometry under blend_mode='zone' (profile='toppra') "
                        "first, then ruckig_corner, and print both wall-clock times")
    args = p.parse_args()

    if args.sync:
        raise SystemExit(
            "REFUSING --sync: it blocks on every move, so the planner never buffers a chain "
            "and there is nothing to blend. Run async (the default).")

    axes = tuple(int(x) for x in args.axes.split(","))
    if len(axes) != 2:
        raise SystemExit(f"--axes needs exactly two joint indices, got {args.axes!r}")

    if sum(bool(x) for x in (args.rect, args.zigzag, args.square)) > 1:
        raise SystemExit("--rect, --zigzag and --square are different shapes; pick one.")

    bow_mm = None
    if args.square:
        shape = "square"
        waypoints, bow_mm = square_waypoints(args.side_mm, args.plane)
        span = f", {args.side_mm:.0f}mm side in the {args.plane} plane"
    else:
        shape = "zigzag" if args.zigzag else ("rect" if args.rect else "L")
        deg = args.deg if args.deg is not None else (8.0 if args.zigzag else 12.0)
        waypoints = _corner_waypoints(axes, deg, args.deg2, teeth=args.teeth)[shape]
        span = f", J{axes[0]} marches +{(args.teeth + 1) * deg:.0f} deg" if args.zigzag else ""

    _assert_within_joint_limits(waypoints, shape)

    if bow_mm is not None:
        print(f"\nSQUARE IN SPACE — {args.side_mm:.0f}mm side, {args.plane} plane, "
              f"{len(waypoints)} moves, all 4 corners blended.")
        print(f"  The 4 corners are IK'd, so the tool HITS THEM EXACTLY.")
        print(f"  The EDGES BOW by {bow_mm:.1f} mm ({bow_mm / args.side_mm * 100:.1f}% of the "
              f"side) — measured, not guessed.")
        print(f"  That is not a defect in the blend: a straight line in JOINT space is not a")
        print(f"  straight line in SPACE, and ruckig_corner is joint-space only. For genuinely")
        print(f"  straight edges use move_linear + blend_mode='zone' "
              f"(api_tests/14_zone_blend.py --rect).")

    if not require_go(args, f"run a {shape} ({len(waypoints)} moves{span}) "
                            f"with blend_mode='ruckig_corner' at {args.speed}% speed"):
        return

    robot = connect(args)
    try:
        prepare_exec(robot, args)
        goto_start(robot, args)
        robot.wait_idle()

        opts = {}
        if args.jerk is not None:
            opts["jerk"] = args.jerk

        if args.square:
            # The approach into the square must NOT be part of the blended chain, or the arm
            # rounds the corner on the way IN and never actually reaches the entry point.
            # Send it on its own, settle, and then blend only the closed loop.
            print(f"\n-- approach: moving to the square's entry point (unblended) --")
            robot.move_joints(*waypoints[0], speed=args.start_speed)
            robot.wait_idle()
            waypoints = waypoints[1:]

        zone_dt = zone_stats = None
        if args.compare:
            zone_dt, zone_stats = _run_chain(
                robot, waypoints, args.blend_mm, "zone", "toppra", args.speed,
                "zone (Bezier + TOPPRA)")
            goto_start(robot, args)
            robot.wait_idle()

        rc_dt, rc_stats = _run_chain(
            robot, waypoints, args.blend_mm, "ruckig_corner", "ruckig", args.speed,
            "ruckig_corner", **opts)

        planner = robot.get_status().get("planner") or {}
        diag = planner.get("last_diagnostic") or {}
        corners = diag.get("ruckig_corners")
        if corners:
            show("per-corner report (blended / entry+exit speed rad/s / closest approach rad)",
                 corners)
            unblended = [c for c in corners if not c.get("blended")]
            if unblended:
                print(f"\n!! {len(unblended)} corner(s) were NOT blended — the arm stopped "
                      f"there. Ruckig rejected the cut states (corner too tight for the "
                      f"speed). Try a larger --blend-mm or a lower --speed.")
        else:
            print("\n!! No ruckig_corners diagnostic came back. Either the chain fell back to "
                  "individual moves (check logs/activity_exec.log), or command_executor is "
                  "running OLD code — restart it so the new planner is loaded.")

        if zone_dt is not None and zone_stats and rc_stats:
            pct = (rc_dt / zone_dt - 1.0) * 100.0
            jz, jr = zone_stats["peak_jerk_cmd"], rc_stats["peak_jerk_cmd"]
            rz, rr = zone_stats["rms_jerk_cmd"], rc_stats["rms_jerk_cmd"]
            print("\n" + "=" * 78)
            print("COMPARISON — what the time bought you (COMMANDED, straight from the motors)")
            print("=" * 78)
            print(f"  {'':<24}{'zone (Bezier+TOPPRA)':>22}{'ruckig_corner':>18}{'':>10}")
            print("  " + "-" * 72)
            print(f"  {'duration':<24}{zone_dt:>19.2f}s{rc_dt:>17.2f}s{pct:>+9.0f}%")
            print(f"  {'peak |jerk| (rad/s^3)':<24}{jz:>20.1f}{jr:>18.1f}"
                  f"{('  ' + f'{jz/jr:.0f}x lower') if jr > 1e-9 else '':>10}")
            print(f"  {'rms  |jerk| (rad/s^3)':<24}{rz:>20.1f}{rr:>18.1f}"
                  f"{('  ' + f'{rz/rr:.0f}x lower') if rr > 1e-9 else '':>10}")
            print(f"  {'peak |accel| (rad/s^2)':<24}{zone_stats['peak_acc_cmd']:>20.2f}"
                  f"{rc_stats['peak_acc_cmd']:>18.2f}")
            print(f"  {'peak |vel| (rad/s)':<24}{zone_stats['peak_vel_cmd']:>20.3f}"
                  f"{rc_stats['peak_vel_cmd']:>18.3f}")
            print("\n  THE TRADE: ruckig_corner spends time to buy a bounded jerk. If the jerk")
            print("  column is NOT dramatically lower, the lane is not doing its job and the")
            print("  extra time bought nothing — say so rather than assuming it worked.")
            vz, vr = zone_stats["peak_vel_cmd"], rc_stats["peak_vel_cmd"]
            if vr < 0.75 * vz:
                print(f"\n  ruckig_corner peaked at only {vr/vz*100:.0f}% of the velocity zone used, "
                      f"and that is")
                print("  NOT a bug to be fixed. Carrying speed through the corners was built and")
                print("  measured: it is SLOWER on every geometry (+10% to +41%), because the leg is")
                print("  cut at the blend ball — the braking tail never runs — while a faster corner")
                print("  takes longer to negotiate and swings wider. See the DEAD END block in")
                print("  motion/ruckig_corner.py before touching this.")

        print("\nIf the arm STOPPED at a waypoint instead of rounding it, the chain did not")
        print("blend — check logs/activity_exec.log for a planner warning (wrong profile, or")
        print("a fallback to individual moves). It says so; it does not pretend.")
    finally:
        finish(robot)


if __name__ == "__main__":
    main_guard(run)
