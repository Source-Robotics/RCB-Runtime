"""Initialise robot state data structures (deques, default dicts, process helpers)."""

import logging
import numpy as np
from collections import deque

logger = logging.getLogger(__name__)

MAX_DATA_POINTS = 500  # History of data points we receive from CAN bus


# --------------------------------------------------------------------------- #
# Default data factories
# --------------------------------------------------------------------------- #

def get_default_motor_data():
    return {
        "position": -99999999,
        "speed": -99999999,
        "current": -99999999,
        "timestamp": -1,
        "msg_ID": -1,
        "tick": -1,
        "error": -1,
    }

def get_default_motor_temperature():
    return {
        "temp": -99999999,
        "timestamp": -1,
        "msg_ID": -1,
        "tick": -1,
        "error": -1
    }

def get_default_motor_voltage():
    return {
        "volt": -99999999,
        "timestamp": -1,
        "msg_ID": -1,
        "tick": -1,
        "error": -1
    }

def get_default_motor_errors():
    return {
        "global_error": 0,
        "temperature_error": 0,
        "encoder_error": 0,
        "vbus_error": 0,
        "driver_error": 0,
        "velocity_error": 0,
        "current_error": 0,
        "estop_error": 0,
        "watchdog_error": 0,
        "calibrated": 0,
        "activated": 0,
        "timestamp": -1,
        "msg_ID": -1,
        "tick": -1,
        "error": -1
    }

def get_default_motor_info():
    return {
        "HARDWARE_VERSION": -1,
        "BATCH_DATE": -1,
        "SOFTWARE_VERSION": -1,
        "SERIAL_NUMBER": -1,
        "timestamp": -1,
        "msg_ID": -1,
        "tick": -1,
        "error": -1
    }

def get_default_motor_kt():
    return {
        "Kt":        -1.0,
        "timestamp": -1,
        "msg_ID":    -1,
        "tick":      -1,
        "error":     -1,
    }

def get_default_gripper_data():
    return {
        "g_p": -99999999,
        "g_c": -99999999,
        "timestamp": -1,
        "g_activated": 0,
        "g_action": 0,
        "g_obj": 0,
        "g_temp_err": 0,
        "g_tim_err": 0,
        "g_estop_err": 0,
        "g_cal": 0,
        "msg_ID": -1,
        "tick": -1,
        "g_err": 0,
        # Computed joint-level fields (SI units)
        "g_jp":  -99999999.0,   # joint position (m)
        "g_js":  -99999999.0,   # joint speed (m/s)
        "g_jf":  -99999999.0,   # joint force (N)
        "g_jpt": -99999999.0,   # joint position target (m)
        "g_jst": -99999999.0,   # joint speed target (m/s)
        "g_jft": -99999999.0,   # joint force target (N)
        "g_jpc": -99999999.0,   # joint position commanded (m)
        "g_jsc": -99999999.0,   # joint speed commanded (m/s)
        "g_jfc": -99999999.0,   # joint force commanded (N)
    }

def get_default_joint_data(num_joints):
    return {
        "j_p": np.full(num_joints, -99999999, dtype=float),
        "j_s": np.full(num_joints, -99999999, dtype=float),
        "j_t": np.full(num_joints, -99999999, dtype=float),
        "j_a": np.full(num_joints, -99999999, dtype=float),
        "j_j": np.full(num_joints, -99999999, dtype=float),
        # Filtered twins seed to 0, NOT the -99999999 sentinel: the very first filtered
        # finite difference reads these as its t-1 value, and a sentinel would make the
        # first joint_acceleration_filtered spike by ~1e10. Zero gives a clean first tick.
        "j_s_f": np.zeros(num_joints, dtype=float),
        "j_t_f": np.zeros(num_joints, dtype=float),
        "j_a_f": np.zeros(num_joints, dtype=float),
        "j_j_f": np.zeros(num_joints, dtype=float),
        "TCP_p": np.full(6, -99999999, dtype=float),
        "TCP_p_SE3": np.zeros(16, dtype=float),
        # Identity quaternion (w,x,y,z — scalar first), not the -99999999 sentinel the
        # others use: a quaternion is only meaningful on the unit sphere, so seeding it
        # with garbage would hand any consumer a non-rotation before the first real tick.
        "TCP_p_quat": np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        "TCP_s": np.full(6, -99999999, dtype=float),
        "TCP_s_f": np.zeros(6, dtype=float),
        "TCP_a": np.zeros(6, dtype=float),
        "TCP_a_f": np.zeros(6, dtype=float),
        "TCP_f_ext_tool": np.full(6, -99999999, dtype=float),
        "t_ext": np.full(num_joints, -99999999, dtype=float),
        "TCP_f_ext": np.full(6, -99999999, dtype=float),
        "j_pc": np.full(num_joints, -99999999, dtype=float),
        "j_sc": np.full(num_joints, -99999999, dtype=float),
        "j_ac": np.full(num_joints, -99999999, dtype=float),
        "j_tc": np.full(num_joints, -99999999, dtype=float),
        "TCP_p_c": np.full(6, -99999999, dtype=float),
        "TCP_s_c": np.full(6, -99999999, dtype=float),
        "j_pt": np.full(num_joints, -99999999, dtype=float),
        "j_st": np.full(num_joints, -99999999, dtype=float),
        "j_at": np.full(num_joints, -99999999, dtype=float),
        "j_tt": np.full(num_joints, -99999999, dtype=float),
        "TCP_p_t": np.full(6, -99999999, dtype=float),
        "TCP_s_t": np.full(6, -99999999, dtype=float),
        "timestamp": -1,
        "exec_t": -1,
        "tick": -1,
        "j_err": np.full(num_joints, -1, dtype=int),
        # RTI link diagnostics. Seeded to 0, not the -99999999 sentinel: these are only
        # meaningful while RTI mode streams, and "no session" is honestly zero. A sentinel
        # would make a PC reading the echo before the first command see garbage rather
        # than "nothing accepted yet".
        "rti_seq": 0,
        "rti_ok": 0.0,
        "rti_miss": 0,
        "rti_state": 0,
        "mode": "",
        "period_ms": 0.0,
    }

def get_default_system_data(num_joints):
    return {
        "i/o": np.full(8, -1, dtype=int),
        "rob_v": -1,
        "rob_c": -1,
        "rob_mode": -1,
        "pi_temp": -1,
    }


# --------------------------------------------------------------------------- #
# Top-level initialiser
# --------------------------------------------------------------------------- #

def initialize_robot_data(joint_num):
    """
    Initialise all dictionary deques based on the actual joint number.
    Returns a dict containing all the state structures.
    """
    motor_data    = {i: deque([get_default_motor_data()],        maxlen=MAX_DATA_POINTS) for i in range(16)}
    gripper_data  = {i: deque([get_default_gripper_data()],      maxlen=MAX_DATA_POINTS) for i in range(16)}
    temp_data     = {i: deque([get_default_motor_temperature()],  maxlen=MAX_DATA_POINTS) for i in range(16)}
    volt_data     = {i: deque([get_default_motor_voltage()],      maxlen=MAX_DATA_POINTS) for i in range(16)}
    err_data      = {i: deque([get_default_motor_errors()],       maxlen=1)              for i in range(16)}
    info_data     = {i: deque([get_default_motor_info()],         maxlen=1)              for i in range(16)}
    kt_data       = {i: deque([get_default_motor_kt()],           maxlen=1)              for i in range(16)}

    joint_history = deque(maxlen=MAX_DATA_POINTS)
    for _ in range(3):
        joint_history.append(get_default_joint_data(joint_num))

    system_data = deque([{"status": "booting"}], maxlen=1)

    return {
        "Motor_data":             motor_data,
        "Gripper_data":           gripper_data,
        "Motor_temperature_data": temp_data,
        "Motor_voltage_data":     volt_data,
        "Motor_errors_data":      err_data,
        "Motor_info_data":        info_data,
        "Motor_Kt_data":          kt_data,
        "Joint_data":             joint_history,
        "System_data":            system_data,
    }


# --------------------------------------------------------------------------- #
# Data append helpers (process_* functions)
# --------------------------------------------------------------------------- #

def process_Joint_data(Joint_data, position, speed, torque, acceleration, jerk,
                       speed_filtered, torque_filtered, acceleration_filtered, jerk_filtered,
                       TCP_pose_vect, TCP_pose_SE3, TCP_pose_quaternion,
                       TCP_speed, torque_external,
                       TCP_force_external,
                       TCP_speed_filtered, TCP_acceleration, TCP_acceleration_filtered,
                       TCP_force_external_tool,
                       position_commanded, speed_commanded,
                       acceleration_commanded, torque_commanded,
                       TCP_pose_commanded, TCP_speed_commanded,
                       position_target, speed_target, acceleration_target,
                       torque_target, TCP_pose_target, TCP_speed_target,
                       timestamp, exec_time, tick, error,
                       rti_seq=0, rti_ok=0.0, rti_miss=0,
                       rti_state=0, mode="", period_ms=0.0):
    Joint_data.append({
        "j_p":    position.copy(),
        "j_s":    speed.copy(),
        "j_t":    torque.copy(),
        "j_a":    acceleration.copy(),
        "j_j":    jerk.copy(),
        # Filtered twins (suffix _f). j_s_f / j_a_f are also the t-1 history the filtered
        # finite differences read next tick (see data/update_shared_data.py), so they must be
        # queued even if no recipe requests them.
        "j_s_f":  speed_filtered.copy(),
        "j_t_f":  torque_filtered.copy(),
        "j_a_f":  acceleration_filtered.copy(),
        "j_j_f":  jerk_filtered.copy(),
        "TCP_p":  TCP_pose_vect.copy(),
        "TCP_p_SE3": TCP_pose_SE3.copy(),
        "TCP_p_quat": TCP_pose_quaternion.copy(),
        "TCP_s":  TCP_speed.copy(),          # RAW (J·q̇)
        "TCP_s_f": TCP_speed_filtered.copy(),
        "TCP_a":  TCP_acceleration.copy(),   # RAW Cartesian accel
        "TCP_a_f": TCP_acceleration_filtered.copy(),
        "t_ext":  torque_external.copy(),
        "TCP_f_ext": TCP_force_external.copy(),          # world frame
        "TCP_f_ext_tool": TCP_force_external_tool.copy(),  # tool frame
        "j_pc":   position_commanded.copy(),
        "j_sc":   speed_commanded.copy(),
        "j_ac":   acceleration_commanded.copy(),
        "j_tc":   torque_commanded.copy(),
        "TCP_p_c": TCP_pose_commanded.copy(),
        "TCP_s_c": TCP_speed_commanded.copy(),
        "j_pt":   position_target.copy(),
        "j_st":   speed_target.copy(),
        "j_at":   acceleration_target.copy(),
        "j_tt":   torque_target.copy(),
        "TCP_p_t": TCP_pose_target.copy(),
        "TCP_s_t": TCP_speed_target.copy(),
        "timestamp": timestamp,
        "exec_t": exec_time,
        "tick":   tick,
        "j_err":  error.copy(),
        # RTI link diagnostics. Defaulted so every non-RTI caller is unaffected, and
        # queued unconditionally (like the filtered twins above) because a recipe may
        # request them on any tick. Zero unless RTI mode is streaming.
        "rti_seq":  rti_seq,
        "rti_ok":   rti_ok,
        "rti_miss": rti_miss,
        "rti_state": rti_state,
        "mode":      mode,
        "period_ms": period_ms,
    })


def process_Gripper_joint_data(Gripper_data, gripper_id,
                               joint_position, joint_speed, joint_force,
                               joint_position_target, joint_speed_target, joint_force_target,
                               joint_position_commanded, joint_speed_commanded, joint_force_commanded):
    """Update the latest Gripper_data queue entry with SI unit values."""
    if gripper_id not in Gripper_data or len(Gripper_data[gripper_id]) == 0:
        return
    g = Gripper_data[gripper_id][-1]
    g["g_jp"]  = joint_position
    g["g_js"]  = joint_speed
    g["g_jf"]  = joint_force
    g["g_jpt"] = joint_position_target
    g["g_jst"] = joint_speed_target
    g["g_jft"] = joint_force_target
    g["g_jpc"] = joint_position_commanded
    g["g_jsc"] = joint_speed_commanded
    g["g_jfc"] = joint_force_commanded


def process_System_data(System_data, test, gpio_states, scan_receive_ids):
    System_data.append({"test": test})


def process_Motor_data(Motor_data, joint_id, position, speed, current,
                       timestamp, CAN_msg_ID, loop_tick, error):
    if joint_id not in Motor_data:
        logger.warning(f"Invalid joint ID: {joint_id}")
        return
    Motor_data[joint_id].append({
        "position":  position,
        "speed":     speed,
        "current":   current,
        "timestamp": timestamp,
        "msg_ID":    CAN_msg_ID,
        "tick":      loop_tick,
        "error":     error,
    })


def process_Gripper_data(Motor_data, Gripper_data, gripper_id, position, current,
                         timestamp, activated, action_status, object_detection,
                         temp_error, timeout_error, estop_error, calibrated,
                         CAN_msg_ID, loop_tick, error,
                         joint_position=0.0, joint_speed=0.0, joint_force=0.0,
                         joint_position_target=0.0, joint_speed_target=0.0, joint_force_target=0.0,
                         joint_position_commanded=0.0, joint_speed_commanded=0.0, joint_force_commanded=0.0):
    if gripper_id not in Gripper_data:
        logger.warning(f"Invalid joint ID: {gripper_id}")
        return
    Gripper_data[gripper_id].append({
        "g_p":        position,
        "g_c":        current,
        "timestamp":  timestamp,
        "g_activated": activated,
        "g_action":   action_status,
        "g_obj":      object_detection,
        "g_temp_err": temp_error,
        "g_tim_err":  timeout_error,
        "g_estop_err": estop_error,
        "g_cal":      calibrated,
        "msg_ID":     CAN_msg_ID,
        "tick":       loop_tick,
        "g_err":      error,
        "g_jp":       joint_position,
        "g_js":       joint_speed,
        "g_jf":       joint_force,
        "g_jpt":      joint_position_target,
        "g_jst":      joint_speed_target,
        "g_jft":      joint_force_target,
        "g_jpc":      joint_position_commanded,
        "g_jsc":      joint_speed_commanded,
        "g_jfc":      joint_force_commanded,
    })
    Motor_data[gripper_id].append({
        "position":  0,
        "speed":     0,
        "current":   current,
        "timestamp": timestamp,
        "msg_ID":    CAN_msg_ID,
        "tick":      loop_tick,
        "error":     error,
    })


def process_Motor_temperature(Motor_temperature, joint_id, temperature,
                              timestamp, CAN_msg_ID, loop_tick, error):
    if joint_id not in Motor_temperature:
        logger.warning(f"Invalid joint ID: {joint_id}")
        return
    Motor_temperature[joint_id].append({
        "temp":      temperature,
        "timestamp": timestamp,
        "msg_ID":    CAN_msg_ID,
        "tick":      loop_tick,
        "error":     error,
    })


def process_Motor_Voltage(Motor_voltage, joint_id, voltage,
                          timestamp, CAN_msg_ID, loop_tick, error):
    if joint_id not in Motor_voltage:
        logger.warning(f"Invalid joint ID: {joint_id}")
        return
    Motor_voltage[joint_id].append({
        "volt":      voltage,
        "timestamp": timestamp,
        "msg_ID":    CAN_msg_ID,
        "tick":      loop_tick,
        "error":     error,
    })


def process_Motor_Kt(Motor_Kt, joint_id, Kt,
                     timestamp, CAN_msg_ID, loop_tick, error):
    if joint_id not in Motor_Kt:
        logger.warning(f"Invalid joint ID: {joint_id}")
        return
    Motor_Kt[joint_id].append({
        "Kt":        Kt,
        "timestamp": timestamp,
        "msg_ID":    CAN_msg_ID,
        "tick":      loop_tick,
        "error":     error,
    })


def process_Motor_errors(Motor_errors, joint_id, global_error, temperature_error,
                         encoder_error, vbus_error, driver_error, velocity_error,
                         current_error, estop_error, watchdog_error, calibrated,
                         activated, timestamp, CAN_msg_ID, loop_tick, error):
    if joint_id not in Motor_errors:
        logger.warning(f"Invalid joint ID: {joint_id}")
        return
    Motor_errors[joint_id].append({
        "global_error":      global_error,
        "temperature_error": temperature_error,
        "encoder_error":     encoder_error,
        "vbus_error":        vbus_error,
        "driver_error":      driver_error,
        "velocity_error":    velocity_error,
        "current_error":     current_error,
        "estop_error":       estop_error,
        "watchdog_error":    watchdog_error,
        "calibrated":        calibrated,
        "activated":         activated,
        "timestamp":         timestamp,
        "msg_ID":            CAN_msg_ID,
        "tick":              loop_tick,
        "error":             error,
    })


def process_Motor_info(Motor_info, joint_id, HARDWARE_VERSION, BATCH_DATE,
                       SOFTWARE_VERSION, SERIAL_NUMBER, timestamp,
                       CAN_msg_ID, loop_tick, error):
    if joint_id not in Motor_info:
        logger.warning(f"Invalid joint ID: {joint_id}")
        return
    Motor_info[joint_id].append({
        "HARDWARE_VERSION": HARDWARE_VERSION,
        "BATCH_DATE":       BATCH_DATE,
        "SOFTWARE_VERSION": SOFTWARE_VERSION,
        "SERIAL_NUMBER":    SERIAL_NUMBER,
        "timestamp":        timestamp,
        "msg_ID":           CAN_msg_ID,
        "tick":             loop_tick,
        "error":            error,
    })
