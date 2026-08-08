#!/usr/bin/env python3
"""Shared helpers for api_tests scripts."""

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from robotclient import RobotClient, CommandRejected, RobotTimeout, RobotError


# Canonical start pose for every movement test. Confirmed safe in the workspace
# 2026-07-11. Motion tests that begin from "wherever the arm happens to be" are not
# reproducible — the same command gives different results from different poses, which
# has silently invalidated predictions more than once. Always begin from here.
#
# Properties (measured):
#   TCP           : x=0mm, y=203mm, z=299mm  (x=0, so +/-X is a clean lateral line)
#   Jacobian cond : ~33  (well conditioned; nowhere near the 1000 warn threshold)
#   joint margins : >= 34.8 deg from the nearest limit (tightest is J2)
#
# It also reaches the wrist singularity cleanly by moving J5 alone:
#   J5 =  2.0 deg -> condition  ~474   (approaching)
#   J5 =  0.5 deg -> condition ~1896   (near-singular; matches TODO/singularity.md §3.6)
#   J5 =  0.0 deg -> condition ~1e17   (EXACT singularity: J4/J6 coaxial)
START_POSE_DEG = [90.0, -105.0, 165.0, 0.0, -30.0, 180.0]


def parser(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=29000)
    p.add_argument("--sync", action="store_true", help="use RobotClient(sync=True)")
    p.add_argument("--go", action="store_true", help="allow hardware-affecting actions")
    p.add_argument("--speed", type=float, default=10.0, help="joint speed percent")
    p.add_argument("--profile", default="trapezoid", choices=["trapezoid", "quintic", "s_curve", "toppra"])
    p.add_argument("--ik", default="pinocchio", choices=["pinocchio", "rtb"])
    p.add_argument("--timeout", type=float, default=20.0)
    p.add_argument("--assume-ready", action="store_true",
                   help="skip clear/enable/home/enter_exec; require current status EXEC+ENABLED+homed")
    p.add_argument("--no-start-pose", action="store_true",
                   help="do NOT move to START_POSE_DEG first (results become dependent on wherever "
                        "the arm happens to be — only use this deliberately)")
    p.add_argument("--start-speed", type=float, default=10.0,
                   help="joint speed percent for the move to START_POSE_DEG")
    return p


def goto_start(robot, args):
    """Move to START_POSE_DEG so a motion test always begins from the same place.

    Skipped by --no-start-pose. Uses move_joints (joint-space, no IK) so it is safe to
    call from any reachable configuration — including from a singular one, which a
    Cartesian move could not do.
    """
    if getattr(args, "no_start_pose", False):
        print("Skipping start pose (--no-start-pose): results depend on the current pose.")
        return
    print(f"Moving to START pose {START_POSE_DEG} at {args.start_speed:.0f}%")
    robot.move_joints(START_POSE_DEG, profile=args.profile, speed=args.start_speed,
                      timeout=args.timeout)
    robot.wait_idle(timeout=args.timeout)


def connect(args):
    robot = RobotClient(sync=args.sync)
    robot.connect(args.host, args.port)
    return robot


def show(title, value):
    print(f"\n== {title} ==")
    if isinstance(value, (dict, list)):
        print(json.dumps(value, indent=2, sort_keys=True))
    else:
        print(value)


def show_basic(robot):
    show("status event snapshot", robot.status())
    try:
        show("get_status", robot.get_status())
    except Exception as exc:
        print(f"get_status failed: {exc}")
    try:
        show("joints_deg", robot.get_joints())
    except Exception as exc:
        print(f"get_joints failed: {exc}")
    try:
        show("pose", robot.get_pose())
    except Exception as exc:
        print(f"get_pose failed: {exc}")


def require_go(args, action):
    if args.go:
        return True
    print(f"DRY RUN: would {action}. Re-run with --go to allow it.")
    return False


def _assert_ready(robot):
    status = robot.get_status()
    problems = []
    if status.get("mode") != "EXEC":
        problems.append(f"mode={status.get('mode')!r}")
    if status.get("state") != "ENABLED":
        problems.append(f"state={status.get('state')!r}")
    if status.get("homed") is not True:
        problems.append(f"homed={status.get('homed')!r}")
    if problems:
        raise RuntimeError("--assume-ready requires EXEC + ENABLED + homed; got " + ", ".join(problems))
    return status


def prepare_exec(robot, args):
    if getattr(args, "assume_ready", False):
        status = _assert_ready(robot)
        robot.set_joint_speed(args.speed)
        robot.set_profile(args.profile)
        robot.set_ik_solver(args.ik)
        show("ready", status)
        return

    robot.clear_errors(wait=True)
    robot.enable(wait=True)
    robot.home(wait=True)
    robot.enter_exec_mode(wait=True)
    robot.set_joint_speed(args.speed)
    robot.set_profile(args.profile)
    robot.set_ik_solver(args.ik)
    show("prepared", robot.get_status())


def finish(robot):
    try:
        show("final status", robot.get_status())
    finally:
        robot.disconnect()


def main_guard(fn):
    try:
        fn()
    except CommandRejected as exc:
        print(f"COMMAND REJECTED: code={exc.code} message={exc.message}")
        raise SystemExit(2)
    except RobotTimeout as exc:
        print(f"TIMEOUT: {exc}")
        raise SystemExit(3)
    except RobotError as exc:
        print(f"ROBOT ERROR: {exc}")
        raise SystemExit(4)
