#!/usr/bin/env python3
"""15: slow wrist-singularity Cartesian probe.

This is intentionally a probe, not a pass/fail regression test.

Default behavior:
  * Reads the current joint pose.
  * Builds a probe joint pose with J5 near/at 0 deg, where J4 and J6 are coaxial.
  * Prints local FK/Jacobian singularity metrics for that pose.
  * With --go, moves to that pose, then dry-runs a small Cartesian move_linear
    from there with constant TCP orientation.
  * Only sends the real Cartesian line when --linear-hardware-ack is present.

Optional --joint-sweep behavior:
  * Slowly sweeps J5 through 0 deg in joint space. This is useful to watch the
    mechanics/plots, but it is not the Cartesian IK singularity test.

Watch the NiceGUI plots while this runs:
  - Units=Joint: J4/J5/J6 position/velocity/acceleration near the singularity.
  - Units=TCP: the Cartesian line and any IK singularity warnings/failures.
"""

import math

import numpy as np

from _common import (
    CommandRejected,
    START_POSE_DEG,
    parser,
    connect,
    goto_start,
    prepare_exec,
    show,
    finish,
    main_guard,
)


def _load_robot_model():
    from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR
    from config.xml_parser import load_system_config, load_full_robot_config
    from utility.robot_factory import create_robot
    from utility.pinocchio_attach import attach_pinocchio
    from utility.kinematics_backend import set_kinematics_backend

    sys_cfg = load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = load_full_robot_config(
        str(sys_cfg["robot_xml_path"]),
        grippers_dir=GRIPPERS_DIR,
        active_gripper=sys_cfg["active_gripper"],
    )
    robot_model = create_robot(sys_cfg["active_robot"], gripper_name=cfg.get("gripper_name", ""))
    try:
        attach_pinocchio(robot_model)
        set_kinematics_backend("pinocchio")
    except Exception:
        set_kinematics_backend("rtb")
    return robot_model, int(cfg["joint_num"])


def _singularity_metrics(robot_model, q_rad):
    from robotics.robot_kinematics import calculate_jacobian

    jacobian = calculate_jacobian(robot_model, q_rad)
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    sigma_min = float(np.min(singular_values)) if singular_values.size else 0.0
    sigma_max = float(np.max(singular_values)) if singular_values.size else 0.0
    condition = float("inf") if sigma_min <= 1e-12 else sigma_max / sigma_min
    manipulability = float(np.prod(singular_values)) if singular_values.size else 0.0
    return {
        "condition": condition,
        "sigma_min": sigma_min,
        "manipulability": manipulability,
    }


def _condition_text(condition):
    return "inf" if math.isinf(condition) else f"{condition:.1f}"


def _print_single_probe(robot_model, q_deg, wrist_idx):
    metrics = _singularity_metrics(robot_model, np.deg2rad(q_deg))
    print("\nWrist-linear probe point:")
    print(
        f"  J{wrist_idx + 1}={q_deg[wrist_idx]:7.3f} deg  "
        f"condition={_condition_text(metrics['condition']):>9s}  "
        f"sigma_min={metrics['sigma_min']:.3e}  "
        f"manip={metrics['manipulability']:.3e}"
    )


def _print_sweep_table(robot_model, q_start_deg, q_zero_deg, q_end_deg, wrist_idx):
    print("\nJoint-sweep reference points:")
    for name, q_deg in (("start", q_start_deg), ("singularity", q_zero_deg), ("end", q_end_deg)):
        metrics = _singularity_metrics(robot_model, np.deg2rad(q_deg))
        print(
            f"  {name:11s} J{wrist_idx + 1}={q_deg[wrist_idx]:7.3f} deg  "
            f"condition={_condition_text(metrics['condition']):>9s}  "
            f"sigma_min={metrics['sigma_min']:.3e}  "
            f"manip={metrics['manipulability']:.3e}"
        )


def _planner_diag(robot):
    planner = robot.get_status().get("planner") or {}
    return planner.get("last_diagnostic") or planner


def _linear_delta(axis, distance_mm):
    unit = {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }[axis]
    scale = float(distance_mm) / 1000.0
    return tuple(v * scale for v in unit)


def run():
    p = parser("15: slow wrist-singularity Cartesian probe")
    p.set_defaults(profile="toppra", speed=5.0)
    p.add_argument("--wrist-joint", type=int, default=5,
                   help="1-based wrist singularity joint. Default 5 = J5.")
    p.add_argument("--wrist-deg", type=float, default=0.0,
                   help="absolute wrist angle for the Cartesian linear probe. "
                        "0 deg makes J4/J6 coaxial; use e.g. 0.5 if exact zero is too spicy.")
    p.add_argument("--linear-axis", default="x", choices=["x", "y", "z"],
                   help="world-frame direction for the Cartesian line from the singular pose.")
    p.add_argument("--linear-mm", type=float, default=20.0,
                   help="Cartesian line distance in millimetres from the singular pose.")
    p.add_argument("--linear-hardware-ack", "--cartesian-hardware-ack",
                   dest="linear_hardware_ack", action="store_true",
                   help="after the Cartesian dry-run, also send the real move_linear_rel_world.")
    p.add_argument("--allow-singularity-warning", action="store_true",
                   help="allow hardware execution even when the dry-run reports singularity_warning=true.")
    p.add_argument("--joint-sweep", action="store_true",
                   help="instead of the Cartesian line test, just sweep J5 through zero in joint space.")
    p.add_argument("--start-deg", type=float, default=8.0,
                   help="absolute wrist angle at the start side, in degrees (--joint-sweep).")
    p.add_argument("--end-deg", type=float, default=-8.0,
                   help="absolute wrist angle at the end side, in degrees (--joint-sweep).")
    p.add_argument("--approach-speed", type=float, default=8.0,
                   help="speed percent for the move from current pose to the probe pose.")
    p.add_argument("--samples", type=int, default=120,
                   help="IK samples for the Cartesian move_linear probe.")
    p.add_argument("--return-start", action="store_true",
                   help="after the probe, move back to the original joint pose.")
    args = p.parse_args()

    if args.profile != "toppra":
        print(f"WARNING: profile={args.profile!r} - forcing profile='toppra' for this probe.")
    args.profile = "toppra"

    robot = connect(args)
    try:
        current = [float(v) for v in robot.get_joints()]
        robot_model, joint_num = _load_robot_model()
        if len(current) < joint_num:
            raise RuntimeError(f"get_joints returned {len(current)} joints, expected at least {joint_num}")

        wrist_idx = int(args.wrist_joint) - 1
        if wrist_idx < 0 or wrist_idx >= joint_num:
            raise SystemExit(f"--wrist-joint must be between 1 and {joint_num}")

        # Build the probe from the CANONICAL START POSE, not from wherever the arm
        # happens to be. Deriving it from get_joints() meant every run probed a
        # different configuration (only J5 was pinned; J1-J4/J6 kept arbitrary values),
        # so results were not comparable between runs. From START_POSE_DEG, moving J5
        # alone reaches a genuine wrist singularity (see _common.py).
        base = list(START_POSE_DEG)[:joint_num]

        q_probe = list(base)
        q_probe[wrist_idx] = float(args.wrist_deg)

        q_start = list(base)
        q_zero = list(base)
        q_end = list(base)
        q_start[wrist_idx] = float(args.start_deg)
        q_zero[wrist_idx] = 0.0
        q_end[wrist_idx] = float(args.end_deg)

        show("current joints_deg", current[:joint_num])
        show("canonical START pose (probe is built from THIS, not from current)", base)
        if args.joint_sweep:
            _print_sweep_table(robot_model, q_start, q_zero, q_end, wrist_idx)
        else:
            _print_single_probe(robot_model, q_probe, wrist_idx)

        print("\nPlanned probe:")
        if args.joint_sweep:
            print(f"  approach: move_joints(... J{wrist_idx + 1}={args.start_deg:.3f} deg) "
                  f"at {args.approach_speed:.1f}%")
            print(f"  joint sweep: move_joints(... J{wrist_idx + 1}={args.end_deg:.3f} deg) "
                  f"at {args.speed:.1f}%")
        else:
            print(f"  approach: move_joints(... J{wrist_idx + 1}={args.wrist_deg:.3f} deg) "
                  f"at {args.approach_speed:.1f}%")
            print(f"  linear dry-run: move_linear_rel_world along +{args.linear_axis.upper()} "
                  f"by {args.linear_mm:.1f}mm, samples={args.samples}")
            if not args.linear_hardware_ack:
                print("  hardware Cartesian execution: disabled; add --linear-hardware-ack")

        if not args.go:
            print("\nDRY RUN ONLY: add --go to move the robot.")
            return

        prepare_exec(robot, args)
        robot.set_completion_policy("settled")

        # Always begin from the canonical pose so the probe is reproducible.
        goto_start(robot, args)

        approach_target = q_start if args.joint_sweep else q_probe
        print(f"\n[approach] START pose -> wrist probe pose "
              f"(J{wrist_idx + 1} -> {approach_target[wrist_idx]:.2f} deg, joint-space move)")
        robot.move_joints(approach_target, profile="toppra", speed=args.approach_speed, timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)

        if args.joint_sweep:
            print("\n[joint sweep] crossing wrist singularity in joint space")
            robot.move_joints(q_end, profile="toppra", speed=args.speed, timeout=args.timeout)
            robot.wait_idle(timeout=args.timeout)
        else:
            dx, dy, dz = _linear_delta(args.linear_axis, args.linear_mm)

            print("\n[linear dry-run] checking Cartesian line from wrist-singular pose")
            try:
                robot.move_linear_rel_world(
                    dx, dy, dz,
                    dry_run=True, ik_solver=args.ik, profile="toppra",
                    speed=args.speed, samples=args.samples, timeout=args.timeout)
                robot.wait_motion_complete(timeout=args.timeout)
            except CommandRejected as exc:
                show("planner diagnostics after rejected linear dry-run", _planner_diag(robot))
                print(
                    f"\nDry-run rejected as expected near a wrist singularity: "
                    f"code={exc.code} message={exc.message}"
                )
                print("Hardware Cartesian line skipped.")
                return

            diag = _planner_diag(robot)
            show("planner diagnostics after linear dry-run", diag)

            if diag.get("singularity_warning") is True and not args.allow_singularity_warning:
                print(
                    "\nCartesian hardware line skipped because dry-run reported "
                    "singularity_warning=true. Add --allow-singularity-warning only "
                    "if you intentionally want to execute this warned path."
                )
            elif args.linear_hardware_ack:
                print("\n[linear hardware] sending Cartesian line from wrist-singular pose")
                robot.move_linear_rel_world(
                    dx, dy, dz,
                    ik_solver=args.ik, profile="toppra",
                    speed=args.speed, samples=args.samples, timeout=args.timeout)
                robot.wait_idle(timeout=args.timeout)
                show("planner diagnostics after linear hardware", _planner_diag(robot))
            else:
                print("\nCartesian hardware line skipped.")

        if args.return_start:
            print("\n[return] moving back to original joint pose")
            robot.move_joints(current[:joint_num], profile="toppra", speed=args.approach_speed,
                              timeout=args.timeout)
            robot.wait_idle(timeout=args.timeout)
    finally:
        finish(robot)


if __name__ == "__main__":
    main_guard(run)
