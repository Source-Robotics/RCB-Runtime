"""
Startup diagnostics for the robot system.

run_selfcheck() is called once at tick 8 and can be called again after error
recovery.  It sends explicit telemetry requests to all motors and then runs its
own dedicated CAN receive loop to collect the responses — it does NOT rely on
the shared deques populated by receive_and_process_can_messages.
"""

import logging
import time

import numpy as np

from utility.system_temperature import get_raspberry_pi_temperature

logger = logging.getLogger(__name__)

# CAN command IDs for the four telemetry response types
_CMD_TEMPERATURE = 23
_CMD_VOLTAGE     = 24
_CMD_DEVICE_INFO = 25
_CMD_ERRORS      = 26
_TELEMETRY_IDS   = {_CMD_TEMPERATURE, _CMD_VOLTAGE, _CMD_DEVICE_INFO, _CMD_ERRORS}


def run_selfcheck(
    *,
    Motor,
    communication,
    joint_num,
    active_CAN_gripper,
    shared_data,
    scan_received_ids=None,
    timeout=0.4,
):
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
    logger.info("[Selfcheck] ========== STARTUP SELFCHECK ==========")

    # ------------------------------------------------------------------
    # 1. CAN bus scan — use pre-computed result if available
    # ------------------------------------------------------------------
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
        logger.warning(f"[Selfcheck] Missing CAN nodes: {missing}")
    else:
        logger.info(f"[Selfcheck] CAN scan OK — all {len(expected_nodes)} node(s) present")

    # ------------------------------------------------------------------
    # 2. Request fresh telemetry from every joint
    # ------------------------------------------------------------------
    for i in range(joint_num):
        Motor[i].Send_Respond_Device_Info()
    for i in range(joint_num):
        Motor[i].Send_Respond_Temperature()
    for i in range(joint_num):
        Motor[i].Send_Respond_Voltage()
    for i in range(joint_num):
        Motor[i].Send_Respond_State_of_Errors()

    # ------------------------------------------------------------------
    # 3. Dedicated receive loop — read CAN frames directly, bypassing the
    #    shared deques entirely.  We exit early once every joint has replied
    #    to all four telemetry requests, or after `timeout` seconds.
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 4. Firmware versions  (None = no response within timeout)
    # ------------------------------------------------------------------
    firmware_versions = [fw_raw.get(i) for i in range(joint_num)]
    results["firmware_versions"] = firmware_versions
    no_fw = [i for i, v in enumerate(firmware_versions) if v is None]
    if no_fw:
        logger.warning(
            f"[Selfcheck] Firmware version not received for joint(s) {no_fw} "
            f"— no CAN response within {timeout}s"
        )
    logger.info(f"[Selfcheck] Firmware versions: {firmware_versions}")

    # ------------------------------------------------------------------
    # 5. Motor error states  (None = no response; 0 = no error; >0 = fault)
    # ------------------------------------------------------------------
    motor_errors = [
        (int(err_raw[i]) if i in err_raw else None)
        for i in range(joint_num)
    ]
    results["motor_errors"] = motor_errors
    error_joints = [i for i, e in enumerate(motor_errors) if e is not None and e != 0]
    no_err_resp  = [i for i, e in enumerate(motor_errors) if e is None]
    if no_err_resp:
        logger.warning(
            f"[Selfcheck] Error state not received for joint(s) {no_err_resp} "
            f"— no CAN response within {timeout}s"
        )
    if error_joints:
        logger.warning(f"[Selfcheck] Motor errors on joint(s) {error_joints}: {motor_errors}")
    else:
        logger.info("[Selfcheck] Motor errors: none")

    # ------------------------------------------------------------------
    # 6. Temperatures  (None = no response)
    # ------------------------------------------------------------------
    temperatures = [temp_raw.get(i) for i in range(joint_num)]
    results["temperatures"] = temperatures
    no_temp = [i for i, v in enumerate(temperatures) if v is None]
    if no_temp:
        logger.warning(
            f"[Selfcheck] Temperature not received for joint(s) {no_temp} "
            f"— no CAN response within {timeout}s"
        )
    logger.info(f"[Selfcheck] Motor temperatures: {temperatures}")

    # ------------------------------------------------------------------
    # 7. Bus voltages  (None = no response)
    # ------------------------------------------------------------------
    voltages = [volt_raw.get(i) for i in range(joint_num)]
    results["voltages"] = voltages
    no_volt = [i for i, v in enumerate(voltages) if v is None]
    if no_volt:
        logger.warning(
            f"[Selfcheck] Voltage not received for joint(s) {no_volt} "
            f"— no CAN response within {timeout}s"
        )
    logger.info(f"[Selfcheck] Bus voltages: {voltages}")

    # ------------------------------------------------------------------
    # 8. Raspberry Pi CPU temperature
    # ------------------------------------------------------------------
    pi_temp = get_raspberry_pi_temperature()
    results["pi_temp"] = pi_temp
    logger.info(f"[Selfcheck] Pi CPU temperature: {pi_temp} °C")

    # ------------------------------------------------------------------
    # 9. Write firmware versions to shared memory
    # ------------------------------------------------------------------
    sw_ver_arr = shared_data.motor.get("SOFTWARE_VERSION")
    if sw_ver_arr is not None:
        for i, fw in enumerate(firmware_versions):
            if fw is not None and i < len(sw_ver_arr):
                sw_ver_arr[i] = int(fw)

    # ------------------------------------------------------------------
    # 10. Overall pass / fail
    #     Fails if: any required CAN node is missing OR any motor has an error
    # ------------------------------------------------------------------
    passed = (len(missing) == 0) and all(e == 0 for e in motor_errors if e is not None)
    results["pass"] = passed

    if passed:
        logger.info("[Selfcheck] ✓ PASSED — all checks OK")
    else:
        fail_reasons = []
        if missing:
            fail_reasons.append(f"missing CAN nodes={missing}")
        if error_joints:
            fail_reasons.append(f"motor errors on joint(s) {error_joints}")
        logger.warning(f"[Selfcheck] ✗ FAILED — {', '.join(fail_reasons)}")

    logger.info("[Selfcheck] ===========================================")
    return results
