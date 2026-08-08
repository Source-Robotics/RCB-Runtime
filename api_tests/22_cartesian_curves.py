#!/usr/bin/env python3
"""22: conservative hardware bring-up for absolute, WRF, and TRF curve commands.

Each selected curve is dry-run before its hardware path. Hardware motion is enabled
only by both --go and --curve-hardware-ack. The default runs every absolute curve
method from the canonical safe pose with a 10 mm world-frame radius/amplitude.
"""
from _common import parser, connect, require_go, prepare_exec, finish, goto_start, main_guard


def _curve_from_pose(robot, pose, shape, radius, frame):
    if frame == "absolute":
        x, y, z, rx, ry, rz = (float(value) for value in pose)
        orientation = [rx, ry, rz]
        if shape == "arc":
            via = [x + radius, y, z + radius, *orientation]
            end = [x + 2.0 * radius, y, z, *orientation]
            return robot.move_arc, (via, end), "10 mm absolute-world semicircular arc in XZ"
        if shape == "circle":
            center = [x + radius, y, z]
            return robot.move_circle, (center, [0.0, 1.0, 0.0]), "10 mm absolute-world XZ circle"
        waypoints = [
            [x + radius, y, z + radius, *orientation],
            [x + 2.0 * radius, y, z - radius, *orientation],
            [x + 3.0 * radius, y, z, *orientation],
        ]
        return robot.move_spline, (waypoints,), "30 mm absolute-world XZ interpolating spline"

    arc = getattr(robot, f"move_arc_rel_{frame}")
    circle = getattr(robot, f"move_circle_rel_{frame}")
    spline = getattr(robot, f"move_spline_rel_{frame}")
    if shape == "arc":
        return (
            arc,
            ([radius, 0.0, radius, 0.0, 0.0, 0.0],
             [2.0 * radius, 0.0, 0.0, 0.0, 0.0, 0.0]),
            f"10 mm {frame}-relative semicircular arc in local XZ",
        )
    if shape == "circle":
        return (
            circle,
            ([radius, 0.0, 0.0], [0.0, 1.0, 0.0]),
            f"10 mm {frame}-relative XZ circle",
        )
    waypoints = [
        [radius, 0.0, radius, 0.0, 0.0, 0.0],
        [2.0 * radius, 0.0, -radius, 0.0, 0.0, 0.0],
        [3.0 * radius, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    return spline, (waypoints,), f"30 mm {frame}-relative interpolating spline in local XZ"


def run():
    p = parser("22: Cartesian arc/circle/spline hardware bring-up")
    p.add_argument("--shape", choices=["arc", "circle", "spline", "all"], default="all",
                   help="curve example to run; default 'all' exercises every new API method")
    p.add_argument("--frame", choices=["absolute", "world", "tool"], default="absolute",
                   help="argument frame; world/tool use the new fixed-start relative commands")
    p.add_argument("--radius-mm", type=float, default=10.0,
                   help="curve radius/amplitude in mm; default is deliberately small")
    p.add_argument("--samples", type=int, default=60,
                   help="explicit Cartesian IK samples; retain 60 for first hardware runs")
    p.add_argument("--curve-hardware-ack", action="store_true",
                   help="required in addition to --go for a real Cartesian curve")
    args = p.parse_args()
    if args.radius_mm <= 0.0:
        raise SystemExit("--radius-mm must be positive")

    robot = connect(args)
    if require_go(args, f"run {args.frame}-frame Cartesian curve example(s): {args.shape}"):
        if not args.curve_hardware_ack:
            raise SystemExit(
                "SAFETY BLOCK: curve hardware motion needs --curve-hardware-ack. "
                "First inspect the dry-run diagnostics and use the default 10 mm arc.")
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")
        goto_start(robot, args)
        options = dict(profile=args.profile, speed=args.speed, samples=args.samples,
                       ik_solver=args.ik, timeout=args.timeout)
        shapes = ("arc", "circle", "spline") if args.shape == "all" else (args.shape,)
        for shape in shapes:
            move, move_args, label = _curve_from_pose(
                robot, robot.get_pose(), shape, args.radius_mm / 1000.0, args.frame)
            print(f"Preflight dry-run: {label}")
            move(*move_args, dry_run=True, **options)
            robot.wait_motion_complete(timeout=args.timeout)
            print(f"Hardware curve: {label}")
            move(*move_args, **options)
            robot.wait_motion_complete(timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
