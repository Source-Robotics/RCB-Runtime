"""What the robot's hardware reports, in one read (2026-10-18) — the data of the first-run check (TODO/first_run_plan.md).

    snapshot(shared_data, joint_num, has_gripper, pcb_version, can_link=None) -> dict

Per drive: which CAN id, whether it answered the start-up scan, whether it answers now, its firmware / hardware
version, serial number and batch, its supply voltage and temperature, its fault flags; the same for a CAN gripper;
the CAN ids that answered the start-up scan although this robot does not use them; the CAN interface's own counters
(`can_link`, read by hardware/can_health.read_can_link_stats — a subprocess, so the caller reads it, never RTI); the
E-stop channels, the box's buttons and the isolated inputs / outputs, as their levels and as what they mean.

Pure reads of shared memory: nothing here commands the robot. The executor serves it as `get_hardware`
(`robot.get_hardware()`), open to every client.
"""
from __future__ import annotations

import time

from data.shared_memory_index import (BUTTON1, BUTTON2, ESTOP_1, ESTOP_2, ISOLATED_INPUT_1, ISOLATED_INPUT_2,
                                      ISOLATED_INPUT_3, ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3)
from hardware import firmware_support
from hardware.control_box import capabilities

SCAN_IDS = 16                    # the start-up scan pings ids 0-15 (rti['scan_received_ids'])
# A drive's fault flags as the drive reports them (motor[...] per joint, gripper[...] for the gripper's motor), in the
# order the dashboard shows them: report name -> (joint field, gripper field).
FLAGS = (("temperature", "motor_temperature_error", "gripper_motor_temp_error"),
         ("encoder", "motor_encoder_error", "gripper_motor_enc_error"),
         ("supply_voltage", "motor_vbus_error", "gripper_motor_vbus_error"),
         ("driver", "motor_driver_error", "gripper_motor_driver_error"),
         ("velocity", "motor_velocity_error", "gripper_motor_velocity_error"),
         ("current", "motor_current_error", "gripper_motor_current_error"),
         ("estop", "motor_estop_error", "gripper_motor_estop_error"),
         ("watchdog", "motor_watchdog_error", "gripper_motor_watchdog_error"))


def _i(arr, k=0) -> int:
    return int(arr[k])


def _drive(motor, rti, i) -> dict:
    return {
        "name": f"J{i + 1}", "can_id": i,
        "answered_scan": bool(_i(rti["scan_received_ids"], i)),
        "answering": not bool(_i(rti["old_data_warning"], i)),
        "lost": bool(_i(rti["CAN_node_connection_error"], i)),
        "firmware": _i(motor["SOFTWARE_VERSION"], i), "hardware": _i(motor["HARDWARE_VERSION"], i),
        "serial": _i(motor["SERIAL_NUMBER"], i), "batch": _i(motor["BATCH_DATE"], i),
        "voltage_v": round(_i(motor["motor_voltage"], i) / 1000.0, 2),
        "temperature_c": _i(motor["motor_temperature"], i),
        "fault": bool(_i(motor["global_motor_error"], i)),
        "flags": {name: bool(_i(motor[field], i)) for name, field, _g in FLAGS},
        "calibrated": bool(_i(motor["motor_calibrated"], i)),
    }


def _gripper(gripper, rti, n) -> dict:
    return {
        "name": "gripper", "can_id": n,
        "answered_scan": bool(_i(rti["scan_received_ids"], n)),
        "answering": not bool(_i(rti["old_data_warning"], n)),
        "lost": bool(_i(rti["CAN_node_connection_error"], n)),
        "firmware": _i(gripper["gripper_motor_sw_version"]), "hardware": _i(gripper["gripper_motor_hw_version"]),
        "serial": _i(gripper["gripper_motor_serial"]),
        "voltage_v": round(_i(gripper["gripper_motor_voltage"]) / 1000.0, 2),
        "temperature_c": _i(gripper["gripper_motor_temperature"]),
        "fault": _i(rti["node_error_bits"], n) == 1,
        "flags": {name: bool(_i(gripper[field])) for name, _j, field in FLAGS},
        "calibrated": bool(_i(gripper["gripper_motor_calibrated"])),
    }


def io_levels(gpio, rti, pcb_version) -> dict:
    """The box's inputs as read (levels) and as meant: the E-stop channels and buttons are active LOW (0 = pressed);
    which E-stop channels count follows the PCB (hardware/control_box.py — PCB v1 reads channel 1 only)."""
    channels = capabilities(pcb_version)["estop_channels"]
    raw = [int(gpio[ESTOP_1]), int(gpio[ESTOP_2])]
    pressed = [v == 0 for v in raw]
    return {
        "pcb_version": int(pcb_version), "estop_channels_used": channels,
        "estop_raw": raw, "estop_channel_pressed": pressed,
        "estop_pressed": any(pressed[:channels]),
        "software_estop": bool(float(rti["software_estop"][0]) > 0.5),
        "buttons_pressed": {"B1": int(gpio[BUTTON1]) == 0, "B2": int(gpio[BUTTON2]) == 0},
        "inputs": [int(gpio[k]) for k in (ISOLATED_INPUT_1, ISOLATED_INPUT_2, ISOLATED_INPUT_3)],
        "outputs": [int(gpio[k]) for k in (ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3)],
    }


def _with_firmware_verdict(entry: dict, driver_type) -> dict:
    """+ the drive's declared driver type and the verdict on its firmware (versioning V4, hardware/firmware_support.py):
    None (not reported yet), "ok", "too_old", "too_new", "undeclared"; + the supported range as text."""
    entry["driver_type"] = driver_type
    entry["firmware_supported"] = firmware_support.check(driver_type, entry["firmware"])
    entry["supported_range"] = firmware_support.range_text(driver_type)
    return entry


def snapshot(shared_data, joint_num, has_gripper, pcb_version, can_link=None, now=time.time,
             driver_types=None, gripper_driver_type=None) -> dict:
    """Everything above in one dict (see the module docstring). `can_link`: read_can_link_stats()'s dict or None.
    `driver_types` (per joint) / `gripper_driver_type`: the robot / gripper XML's <driver_type>s (V4)."""
    rti, motor = shared_data.rti, shared_data.motor
    n = int(joint_num)
    used = n + (1 if has_gripper else 0)
    types = list(driver_types or [])
    return {
        "time": now(), "tick": int(float(rti["loop_tick"][0])),
        "joint_num": n, "has_gripper": bool(has_gripper),
        "drives": [_with_firmware_verdict(_drive(motor, rti, i), types[i] if i < len(types) else None) for i in range(n)],
        "gripper": (_with_firmware_verdict(_gripper(shared_data.gripper, rti, n), gripper_driver_type)
                    if has_gripper else None),
        "scan_unexpected_ids": [k for k in range(used, SCAN_IDS) if _i(rti["scan_received_ids"], k)],
        "can_link": can_link,
        "io": io_levels(shared_data.gpio_states, rti, pcb_version),
    }
