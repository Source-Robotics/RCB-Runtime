#!/usr/bin/env python3
import time

from _common import parser, connect, require_go, prepare_exec, finish, show, main_guard


POLICIES = ("commanded", "settled", "strict")


def _wait_policy_reflected(robot, policy, timeout=1.0):
    deadline = time.monotonic() + timeout
    status = robot.get_status()
    while status.get("exec_completion_policy") != policy and time.monotonic() < deadline:
        time.sleep(0.02)
        status = robot.get_status()
    return status


def _run_policy(robot, args, policy, start):
    print(f"\n-- policy: {policy} --", flush=True)
    robot.set_completion_policy(policy)
    reflected = _wait_policy_reflected(robot, policy)
    show("policy set", reflected)
    if reflected.get("exec_completion_policy") != policy:
        print(f"policy readback still shows {reflected.get('exec_completion_policy')!r}; continuing because executor accepted {policy!r}")

    target = list(start)
    target[0] += args.deg
    print(f"Moving J1 relative: {start[0]:.3f} deg -> {target[0]:.3f} deg")
    t0 = time.monotonic()
    robot.move_joints(target, profile=args.profile, speed=args.speed, timeout=args.timeout)
    robot.wait_motion_complete(timeout=args.timeout)
    elapsed_out = time.monotonic() - t0
    after_out = robot.get_joints()
    print(f"Outbound completion wait: {elapsed_out:.3f}s; J1 now {after_out[0]:.3f} deg")

    # For commanded policy, completion can happen before physical settling. Give the
    # robot a small quiet window before the return move so this stays a safe test.
    if policy == "commanded":
        time.sleep(args.commanded_settle_pause)

    print("Returning to starting joint position")
    t1 = time.monotonic()
    robot.move_joints(start, profile=args.profile, speed=args.speed, timeout=args.timeout)
    robot.wait_idle(timeout=args.timeout)
    elapsed_return = time.monotonic() - t1
    after_return = robot.get_joints()
    print(f"Return wait: {elapsed_return:.3f}s; J1 now {after_return[0]:.3f} deg")

    return {
        "policy": policy,
        "outbound_wait_s": elapsed_out,
        "return_wait_s": elapsed_return,
        "j1_after_out_deg": float(after_out[0]),
        "j1_after_return_deg": float(after_return[0]),
    }


def run():
    p = parser("09: completion-policy checks with tiny J1 nudge/return")
    p.add_argument("--deg", type=float, default=1.0, help="J1 nudge in degrees")
    p.add_argument("--policy", choices=["all", *POLICIES], default="all")
    p.add_argument("--commanded-settle-pause", type=float, default=1.0,
                   help="extra pause after commanded completion before return move")
    args = p.parse_args()

    robot = connect(args)
    if require_go(args, "test completion policies with tiny J1 moves"):
        prepare_exec(robot, args)
        robot.set_blending(0)
        start = [float(v) for v in robot.get_joints()]
        policies = POLICIES if args.policy == "all" else (args.policy,)
        results = []
        for policy in policies:
            results.append(_run_policy(robot, args, policy, start))
            # Refresh the start point from actual position after each return.
            start = [float(v) for v in robot.get_joints()]
        show("completion policy results", results)
        robot.set_completion_policy("settled")
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
