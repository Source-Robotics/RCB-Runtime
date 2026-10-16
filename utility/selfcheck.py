"""
Startup diagnostics for the robot system.

run_selfcheck() is called once at tick 8 and can be called again after error
recovery.  It sends explicit telemetry requests to all motors and then runs its
own dedicated CAN receive loop to collect the responses — it does NOT rely on
the shared deques populated by receive_and_process_can_messages.

One function per check since 2026-10-03 (Phase 5); run_selfcheck() is the order they run in and the only
public name.
"""

from __future__ import annotations

from typing import Any

import logging
import time

import numpy as np

from utility.system_temperature import get_raspberry_pi_temperature

logger = logging.getLogger("rti.selfcheck")  # docs/logging.md "Logger names"

# CAN command IDs for the four telemetry response types
_CMD_TEMPERATURE = 23
_CMD_VOLTAGE     = 24
_CMD_DEVICE_INFO = 25
_CMD_ERRORS      = 26
_TELEMETRY_IDS   = {_CMD_TEMPERATURE, _CMD_VOLTAGE, _CMD_DEVICE_INFO, _CMD_ERRORS}


def _check_can_scan(scan_received_ids, joint_num, active_CAN_gripper, results):
    """1. CAN bus scan — use the pre-computed result if available. Returns the missing node ids."""
    if scan_received_ids is not None:
        scan_ids = np.asarray(scan_received_ids, dtype=int)
    else:
        scan_ids = np.zeros(16, dtype=int)
    results["can_scan"] = scan_ids.tolist()

    expected_nodes = list(range(joint_num))
    if active_CAN_gripper:
        expected_nodes.append(joint_num)
    missing = [n for n in expected_nodes if scan_ids[n] == 0]
    results["missing_nodes"] = missing
    if missing:
        logger.warning(f"Missing CAN nodes: {missing}")
    else:
        logger.info(f"CAN scan OK — all {len(expected_nodes)} node(s) present")
    return missing


def _request_telemetry(Motor, joint_num):
    """2. Request fresh telemetry from every joint."""
    for i in range(joint_num):
        Motor[i].Send_Respond_Device_Info()
    for i in range(joint_num):
        Motor[i].Send_Respond_Temperature()
    for i in range(joint_num):
        Motor[i].Send_Respond_Voltage()
    for i in range(joint_num):
        Motor[i].Send_Respond_State_of_Errors()


def _collect_replies(Motor, communication, joint_num, timeout):
    """3. Dedicated receive loop — read CAN frames directly, bypassing the shared deques entirely. Exits
    early once every joint has replied to all four telemetry requests, or after `timeout` seconds.
    Returns (temperature, voltage, firmware version, global error) dicts keyed by joint id."""
    temp_raw = {}   # joint_id -> temperature
    volt_raw = {}   # joint_id -> voltage
    fw_raw   = {}   # joint_id -> SOFTWARE_VERSION
    err_raw  = {}   # joint_id -> global_error (0 = no error)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        # Early-exit: all four telemetry types received from every joint
        if all(
            i in temp_raw and i in volt_raw and i in fw_raw and i in err_raw
            for i in range(joint_num)
        ):
            break

        message, msg_ID = communication.receive_can_messages(timeout=0.002)
        if message is None or msg_ID is None:
            continue

        node_id    = msg_ID[0]
        command_id = msg_ID[1]

        # Only handle telemetry responses for arm joints
        if node_id not in range(joint_num) or command_id not in _TELEMETRY_IDS:
            continue

        # Unpack the raw CAN frame into the motor object's attributes
        Motor[node_id].UnpackData(message, msg_ID)

        if command_id == _CMD_TEMPERATURE:
            temp_raw[node_id] = Motor[node_id].temperature
        elif command_id == _CMD_VOLTAGE:
            volt_raw[node_id] = Motor[node_id].voltage
        elif command_id == _CMD_DEVICE_INFO:
            fw_raw[node_id] = Motor[node_id].SOFTWARE_VERSION
        elif command_id == _CMD_ERRORS:
            err_raw[node_id] = Motor[node_id].error
    return temp_raw, volt_raw, fw_raw, err_raw


def _telemetry_list(raw, joint_num, results, key, label, info_label, timeout):
    """4./6./7. One value per joint (None = no response within the timeout), stored, missing ones warned
    about, the list logged. Returns the list."""
    values = [raw.get(i) for i in range(joint_num)]
    results[key] = values
    missing = [i for i, v in enumerate(values) if v is None]
    if missing:
        logger.warning(
            f"{label} not received for joint(s) {missing} "
            f"— no CAN response within {timeout}s"
        )
    logger.info(f"{info_label}: {values}")
    return values


def _motor_errors(err_raw, joint_num, results, timeout):
    """5. Motor error states (None = no response; 0 = no error; >0 = fault). Returns (list, faulted joints)."""
    motor_errors = [
        (int(err_raw[i]) if i in err_raw else None)
        for i in range(joint_num)
    ]
    results["motor_errors"] = motor_errors
    error_joints = [i for i, e in enumerate(motor_errors) if e is not None and e != 0]
    no_err_resp  = [i for i, e in enumerate(motor_errors) if e is None]
    if no_err_resp:
        logger.warning(
            f"Error state not received for joint(s) {no_err_resp} "
            f"— no CAN response within {timeout}s"
        )
    if error_joints:
        logger.warning(f"Motor errors on joint(s) {error_joints}: {motor_errors}")
    else:
        logger.info("Motor errors: none")
    return motor_errors, error_joints


def _write_firmware_versions(shared_data, firmware_versions):
    """9. Write the firmware versions to shared memory (joints that answered only)."""
    sw_ver_arr = shared_data.motor.get("SOFTWARE_VERSION")
    if sw_ver_arr is not None:
        for i, fw in enumerate(firmware_versions):
            if fw is not None and i < len(sw_ver_arr):
                sw_ver_arr[i] = int(fw)


def _overall(results, missing, motor_errors, error_joints):
    """10. Overall pass / fail — fails if any required CAN node is missing OR any motor has an error."""
    passed = (len(missing) == 0) and all(e == 0 for e in motor_errors if e is not None)
    results["pass"] = passed

    if passed:
        logger.info("✓ PASSED — all checks OK")
    else:
        fail_reasons = []
        if missing:
            fail_reasons.append(f"missing CAN nodes={missing}")
        if error_joints:
            fail_reasons.append(f"motor errors on joint(s) {error_joints}")
        logger.warning(f"✗ FAILED — {', '.join(fail_reasons)}")
    return passed


def run_selfcheck(
    *,
    Motor: list,
    communication: Any,
    joint_num: int,
    active_CAN_gripper: int,
    shared_data: Any,
    scan_received_ids: np.ndarray | None = None,
    timeout: float = 0.4,
) -> dict:
    """
    Run startup diagnostics and write results to shared memory.

    Parameters
    ----------
    Motor              : list of motor objects (len >= joint_num)
    communication      : CAN communication handle
    joint_num          : number of arm joints
    active_CAN_gripper : 1 if a CAN gripper is attached, else 0
    shared_data        : shared_data module (for writing results)
    scan_received_ids  : np.array already written to shared_data.rti, or None
                         (if None the CAN scan is treated as all-missing)
    timeout            : seconds to wait for CAN responses (default 0.4 s)

    Returns
    -------
    dict with keys: pass, missing_nodes, motor_errors, temperatures,
                    voltages, firmware_versions, pi_temp
    """
    results = {}
    logger.info("========== STARTUP SELFCHECK ==========")

    missing = _check_can_scan(scan_received_ids, joint_num, active_CAN_gripper, results)
    _request_telemetry(Motor, joint_num)
    temp_raw, volt_raw, fw_raw, err_raw = _collect_replies(Motor, communication, joint_num, timeout)

    firmware_versions = _telemetry_list(fw_raw, joint_num, results, "firmware_versions",
                                        "Firmware version", "Firmware versions", timeout)
    motor_errors, error_joints = _motor_errors(err_raw, joint_num, results, timeout)
    _telemetry_list(temp_raw, joint_num, results, "temperatures", "Temperature", "Motor temperatures", timeout)
    _telemetry_list(volt_raw, joint_num, results, "voltages", "Voltage", "Bus voltages", timeout)

    # 8. Raspberry Pi CPU temperature
    pi_temp = get_raspberry_pi_temperature()
    results["pi_temp"] = pi_temp
    logger.info(f"Pi CPU temperature: {pi_temp} °C")

    _write_firmware_versions(shared_data, firmware_versions)
    _overall(results, missing, motor_errors, error_joints)

    logger.info("===========================================")
    return results
