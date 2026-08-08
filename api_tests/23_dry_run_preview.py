#!/usr/bin/env python3
"""23: plan a representative EXEC program without connecting to the robot.

The result contains timestamped joint/TCP arrays that can be inspected now or written
as an NPZ artifact for a later NiceGUI preview/import workflow. This script has no
--go flag because DryRunRobotClient has no RTI, ZMQ output, TCP, or CAN route.
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np

from _common import START_POSE_DEG
from robotclient import DryRunRobotClient
from robotclient.dry_run_preview import ACTIVE_PREVIEW_PATH, MAX_PREVIEW_SAMPLES


def save_preview(path, preview):
    sample_count = int(sum(len(segment.time_s) for segment in preview.segments))
    if sample_count > MAX_PREVIEW_SAMPLES:
        raise ValueError(
            f"preview has more than {MAX_PREVIEW_SAMPLES:,} samples; "
            "split the program into smaller previews")
    arrays = {}
    metadata = {"duration_s": preview.duration_s, "sample_count": sample_count,
                "segments": [], "events": []}
    for index, segment in enumerate(preview.segments):
        prefix = f"segment_{index}"
        arrays[f"{prefix}_time_s"] = segment.time_s
        arrays[f"{prefix}_joint_position_rad"] = segment.joint_position_rad
        arrays[f"{prefix}_joint_velocity_rad_s"] = segment.joint_velocity_rad_s
        arrays[f"{prefix}_joint_acceleration_rad_s2"] = segment.joint_acceleration_rad_s2
        arrays[f"{prefix}_joint_jerk_rad_s3"] = segment.joint_jerk_rad_s3
        arrays[f"{prefix}_tcp_pose"] = segment.tcp_pose
        arrays[f"{prefix}_tcp_velocity"] = segment.tcp_velocity
        arrays[f"{prefix}_tcp_acceleration"] = segment.tcp_acceleration
        metadata["segments"].append({
            "prefix": prefix,
            "sample_count": int(len(segment.time_s)),
            "command_id": segment.command_id,
            "sequence": segment.sequence,
            "command": segment.command,
            "profile": segment.profile,
            "start_time_s": segment.start_time_s,
            "end_time_s": segment.end_time_s,
            "diagnostic": segment.diagnostic,
            "blend_zones": [{
                "corner_index": int(zone["corner_index"]),
                "kind": str(zone.get("kind", "zone")),
                "start_time_s": float(segment.time_s[int(zone["start_index"])]),
                "end_time_s": float(segment.time_s[int(zone["end_index"])]),
            } for zone in segment.blend_zones],
        })
    metadata["events"] = [event.__dict__ for event in preview.events]
    arrays["metadata_json"] = np.array(json.dumps(metadata))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Write beside the active file and replace it only after the archive is complete.
    # The GUI can therefore never observe a half-written preview.
    temporary_path = path.with_name(f'.{path.name}.tmp.npz')
    try:
        np.savez_compressed(temporary_path, **arrays)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)



def _mixed(robot):
    robot.set_profile("toppra")
    robot.move_joints(92.0, -105.0, 165.0, 0.0, -30.0, 180.0,
                      speed=10.0, blend=2.0, blend_mode="zone")
    robot.move_joints(92.0, -103.0, 165.0, 0.0, -30.0, 180.0,
                      speed=10.0, blend=0.0, blend_mode="zone")
    robot.gripper_close()
    robot.move_linear_rel_world(0.001, 0.0, 0.0, speed=5.0, samples=4)



def _j1_trapezoid(robot):
    """One isolated J1 point-to-point move for profile inspection."""
    start = np.asarray(START_POSE_DEG, dtype=float)
    target = start.copy()
    target[0] += 10.0
    robot.move_joints(
        *target,
        profile="trapezoid",
        speed=10.0,
        accel=10.0,
        ramp_fraction=1.0 / 3.0,
        blend=0.0,
    )

JOINT_SQUARE_TARGETS = (
    [94.0, -105.0, 165.0, 0.0, -30.0, 180.0],
    [94.0, -101.0, 165.0, 0.0, -30.0, 180.0],
    [90.0, -101.0, 165.0, 0.0, -30.0, 180.0],
    [90.0, -105.0, 165.0, 0.0, -30.0, 180.0],
)

COMPARE_PROFILES = ("trapezoid", "quintic", "s_curve", "toppra", "ruckig")


def _profile_options(profile):
    options = {"profile": profile, "speed": 8.0, "accel": 8.0, "blend": 0.0}
    if profile == "trapezoid":
        options["ramp_fraction"] = 1.0 / 3.0
    return options

def _queue_joint_square(robot, blends, *, profile="toppra", blend_mode="zone"):
    for target, blend in zip(JOINT_SQUARE_TARGETS, blends):
        robot.move_joints(target, speed=8.0, profile=profile,
                          blend=blend, blend_mode=blend_mode)


def _joint_square(robot):
    robot.set_profile("toppra")
    _queue_joint_square(robot, (2.0, 2.0, 2.0, 2.0))


def _joint_square_blend_compare(robot):
    """A blended joint square, then the identical square with exact stops."""
    robot.set_profile("toppra")
    _queue_joint_square(robot, (2.0, 2.0, 2.0, 2.0))
    _queue_joint_square(robot, (0.0, 0.0, 0.0, 0.0))

def _joint_square_ruckig(robot):
    """The same closed joint square with Ruckig's jerk-limited corner transitions."""
    _queue_joint_square(robot, (2.0, 2.0, 2.0, 2.0),
                        profile="ruckig", blend_mode="ruckig_corner")

def _joint_profile_comparison(robot):
    """Out-and-back J1 P2P moves under every supported time profile."""
    start = np.asarray(START_POSE_DEG, dtype=float)
    target = start.copy()
    target[0] += 10.0
    for profile in COMPARE_PROFILES:
        options = _profile_options(profile)
        robot.move_joints(*target, **options)
        robot.move_joints(*start, **options)


def _cartesian_profile_comparison(robot):
    """Out-and-back world-frame TCP lines under every supported time profile."""
    for profile in COMPARE_PROFILES:
        options = _profile_options(profile)
        robot.move_linear_rel_world(0.010, 0.0, 0.0, samples=40, **options)
        robot.move_linear_rel_world(-0.010, 0.0, 0.0, samples=40, **options)



def _joint_square_all_compare(robot):
    """The same closed square under zone, Ruckig-corner, then exact-stop motion."""
    _queue_joint_square(robot, (2.0, 2.0, 2.0, 2.0),
                        profile="toppra", blend_mode="zone")
    _queue_joint_square(robot, (2.0, 2.0, 2.0, 2.0),
                        profile="ruckig", blend_mode="ruckig_corner")
    _queue_joint_square(robot, (0.0, 0.0, 0.0, 0.0), profile="ruckig")


def _cartesian_square(robot):
    robot.set_profile("toppra")
    for offset, blend in (
        ((0.004, 0.0, 0.0), 0.5),
        ((0.0, 0.004, 0.0), 0.5),
        ((-0.004, 0.0, 0.0), 0.5),
        ((0.0, -0.004, 0.0), 0.0),
    ):
        robot.move_linear_rel_world(*offset, speed=4.0, samples=20,
                                    blend=blend, blend_mode="zone")


def _timeline_demo(robot):
    """Three P2P moves with delay, gripper, setup, and I/O timeline markers."""
    robot.set_profile("trapezoid")
    robot.set_joint_speed(12.0)
    robot.set_digital_output(2, True)
    robot.move_joints(94.0, -105.0, 165.0, 0.0, -30.0, 180.0,
                      speed=12.0, blend=0.0)
    robot.move_joints(98.0, -102.0, 165.0, 0.0, -30.0, 180.0,
                      speed=12.0, blend=0.0)
    robot.delay(0.75)
    robot.gripper_close()
    robot.move_joints(90.0, -105.0, 165.0, 0.0, -30.0, 180.0,
                      speed=12.0, blend=0.0)


EXAMPLES = {
    "mixed": _mixed,
    "j1_trapezoid": _j1_trapezoid,
    "joint_profile_comparison": _joint_profile_comparison,
    "cartesian_profile_comparison": _cartesian_profile_comparison,
    "joint_square_blend_compare": _joint_square_blend_compare,
    "joint_square": _joint_square,
    "cartesian_square": _cartesian_square,
    "timeline_demo": _timeline_demo,
    "joint_square_ruckig": _joint_square_ruckig,
    "joint_square_all_compare": _joint_square_all_compare,
}
def run():
    parser = argparse.ArgumentParser("23: offline EXEC trajectory preview")
    parser.add_argument("--example", choices=EXAMPLES, default="mixed",
                        help="offline program to plan; j1_trapezoid is one isolated J1 P2P move")
    parser.add_argument("--save", type=Path, default=ACTIVE_PREVIEW_PATH,
                        help=(".npz destination; defaults to the single managed active preview at "
                              f"{ACTIVE_PREVIEW_PATH}"))
    args = parser.parse_args()
    robot = DryRunRobotClient(start_joints=START_POSE_DEG)
    EXAMPLES[args.example](robot)
    preview = robot.plan_summary()
    if not preview.valid:
        for error in preview.errors:
            print(f"PLAN ERROR: {error.command} [{error.code}] {error.message}")
        raise SystemExit(2)
    print(f"Offline {args.example} plan: {len(preview.segments)} segments, "
          f"{len(preview.events)} events, {preview.duration_s:.3f} s")
    for segment in preview.segments:
        print(f"  {segment.sequence:02d} {segment.command:<22} [{segment.profile:<10}] "
              f"{len(segment.time_s):4d} samples  "
              f"{segment.start_time_s:.3f} -> {segment.end_time_s:.3f} s")
    save_preview(args.save, preview)
    print(f"Saved active preview to {args.save}")
if __name__ == "__main__":
    run()
