#!/usr/bin/env python3
"""Read-only RTI/shared-memory error snapshot for debugging clear/home stalls."""
import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from api_tests._common import parser, connect, show, finish, main_guard
from data import shared_data


MOTOR_ERROR_FIELDS = [
    "global_motor_error",
    "motor_temperature_error",
    "motor_encoder_error",
    "motor_vbus_error",
    "motor_driver_error",
    "motor_velocity_error",
    "motor_current_error",
    "motor_estop_error",
    "motor_watchdog_error",
    "motor_calibrated",
]

GRIPPER_ERROR_FIELDS = [
    "gripper_temperature_error",
    "gripper_timeout_error",
    "gripper_estop_error",
    "gripper_error",
    "gripper_motor_temp_error",
    "gripper_motor_enc_error",
    "gripper_motor_vbus_error",
    "gripper_motor_driver_error",
    "gripper_motor_velocity_error",
    "gripper_motor_current_error",
    "gripper_motor_estop_error",
    "gripper_motor_watchdog_error",
    "gripper_motor_calibrated",
]


def _list(arr, n=None):
    values = arr.tolist()
    return values if n is None else values[:n]


def _nonzero_indices(values):
    return [i for i, v in enumerate(values) if int(v) != 0]


def _motor_snapshot():
    joint_num = int(getattr(shared_data, "joint_num", 6))
    fields = {name: _list(shared_data.motor[name], joint_num) for name in MOTOR_ERROR_FIELDS}
    per_joint = []
    for i in range(joint_num):
        active = {
            name: int(vals[i])
            for name, vals in fields.items()
            if name != "motor_calibrated" and int(vals[i]) != 0
        }
        if active:
            per_joint.append({"joint": i + 1, "flags": active})
    return {"fields": fields, "active_by_joint": per_joint}


def _rti_snapshot():
    can_lost = _list(shared_data.rti["CAN_node_connection_error"], 16)
    old_data = _list(shared_data.rti["old_data_warning"], 16)
    error_counter = _list(shared_data.rti["Error_counter"], 16)
    timestamp = float(shared_data.rti["timestamp"][0])
    now = time.perf_counter()
    return {
        "robot_mode_string": shared_data.robot_mode.get(),
        "robot_state_string": shared_data.robot_state.get(),
        "homed_state_string": shared_data.homed_state.get(),
        "error_state_string": shared_data.error_state.get(),
        "error_active": int(shared_data.rti["error_active"][0]),
        "safety_violation": int(shared_data.rti["safety_violation"][0]),
        "software_estop": int(shared_data.rti["software_estop"][0]),
        "loop_tick": int(shared_data.rti["loop_tick"][0]),
        "timestamp": timestamp,
        "timestamp_age_s": None if timestamp <= 0.0 else now - timestamp,
        "loop_period_s": float(shared_data.rti["loop_period"][0]),
        "loop_execution_time_s": float(shared_data.rti["loop_execution_time"][0]),
        "overrun_count": float(shared_data.rti["overrun_count"][0]),
        "CAN_node_connection_error": can_lost,
        "CAN_lost_indices": _nonzero_indices(can_lost),
        "old_data_warning": old_data,
        "old_data_indices": _nonzero_indices(old_data),
        "Error_counter": error_counter,
        "homing_status": _list(shared_data.rti["homing_status"], 7),
        "joint_homed": _list(shared_data.rti["joint_homed"], 7),
    }


def _gripper_snapshot():
    fields = {}
    for name in GRIPPER_ERROR_FIELDS:
        if name in shared_data.gripper:
            fields[name] = _list(shared_data.gripper[name])
    active = {name: vals for name, vals in fields.items() if any(int(v) != 0 for v in vals)}
    return {"fields": fields, "active": active}


def run():
    args = parser("07: read-only RTI/motor/gripper error snapshot").parse_args()
    robot = connect(args)
    show("status", robot.get_status())
    show("rti_errors", _rti_snapshot())
    show("motor_errors", _motor_snapshot())
    show("gripper_errors", _gripper_snapshot())
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
