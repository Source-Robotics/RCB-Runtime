"""Serialise Joint_data deques to compact strings for telemetry / logging."""

import numpy as np

DEFAULT_PRECISION = 3


def serialize_joint_data(Joint_data, fields, precision_map=None):
    """
    Serialise the latest entry in Joint_data to a comma-separated string.

    :param Joint_data: deque of joint data dicts
    :param fields: list of field names to include
    :param precision_map: optional dict mapping field name → decimal places
    :returns: string like "j_p:0.000,1.000,...,j_s:..."
    """
    if not Joint_data:
        return ""

    latest = Joint_data[-1]
    parts  = []

    for field in fields:
        if field not in latest:
            continue
        value = latest.get(field)
        if value is None:
            continue
        precision = precision_map.get(field, DEFAULT_PRECISION) if precision_map else DEFAULT_PRECISION

        if isinstance(value, (list, np.ndarray)):
            value_str = np.array2string(
                np.asarray(value).flatten(),
                precision=precision,
                separator=",",
                suppress_small=True,
                max_line_width=np.inf,
                floatmode="fixed",
            ).strip("[]")
        else:
            value_str = f"{value:.{precision}f}" if isinstance(value, float) else str(value)

        parts.append(f"{field}:{value_str}")

    return ",".join(parts)


def serialize_optimized(Joint_data, fields, precision_map=None):
    """Faster serialiser — avoids np.array2string overhead for arrays."""
    if not Joint_data:
        return ""

    latest = Joint_data[-1]
    parts  = []

    for field in fields:
        if field not in latest:
            continue
        value = latest[field]
        if value is None:
            continue
        precision = precision_map.get(field, DEFAULT_PRECISION) if precision_map else DEFAULT_PRECISION

        if isinstance(value, (list, np.ndarray)):
            formatted = [f"{v:.{precision}f}" for v in np.asarray(value).ravel()]
            value_str = ",".join(formatted)
        else:
            value_str = f"{value:.{precision}f}" if isinstance(value, float) else str(value)

        parts.append(f"{field}:{value_str}")

    return ",".join(parts)


# Tag → (motor_buffers key, deque-entry key, is_int). Each motor tag reads the
# latest entry from one of four per-node deque dicts:
#   - "data"        → Motor_data[i]              (position / speed / current)
#   - "temperature" → Motor_temperature_data[i]  (temp)
#   - "voltage"     → Motor_voltage_data[i]      (volt)
#   - "errors"      → Motor_errors_data[i]       (per-error flags + calibrated)
# Same mapping applies to arm motor (m_*) and gripper motor (gm_*) tags — they
# just iterate different node IDs.
#
# is_int hard-codes the value type per tag so the inner loop can skip
# isinstance() per-value. Position / speed / current / voltage / error flags
# are always int from the CAN decode; only temperature is float.
_MOTOR_TAG_BUFFER_KEY = {
    # Sensor data
    "m_p":         ("data",        "position",          True),
    "m_s":         ("data",        "speed",             True),
    "m_c":         ("data",        "current",           True),
    "m_temp":      ("temperature", "temp",              False),
    "m_volt":      ("voltage",     "volt",              True),
    # Error flags + calibration
    "m_t_err":     ("errors",      "temperature_error", True),
    "m_enc_err":   ("errors",      "encoder_error",     True),
    "m_vbus_err":  ("errors",      "vbus_error",        True),
    "m_d_err":     ("errors",      "driver_error",      True),
    "m_vel_err":   ("errors",      "velocity_error",    True),
    "m_c_err":     ("errors",      "current_error",     True),
    "m_estop_err": ("errors",      "estop_error",       True),
    "m_w_err":     ("errors",      "watchdog_error",    True),
    "m_cal":       ("errors",      "calibrated",        True),

    # Gripper-motor — same fields, single node at joint_num
    "gm_p":         ("data",        "position",          True),
    "gm_s":         ("data",        "speed",             True),
    "gm_c":         ("data",        "current",           True),
    "gm_temp":      ("temperature", "temp",              False),
    "gm_volt":      ("voltage",     "volt",              True),
    "gm_t_err":     ("errors",      "temperature_error", True),
    "gm_enc_err":   ("errors",      "encoder_error",     True),
    "gm_vbus_err":  ("errors",      "vbus_error",        True),
    "gm_d_err":     ("errors",      "driver_error",      True),
    "gm_vel_err":   ("errors",      "velocity_error",    True),
    "gm_c_err":     ("errors",      "current_error",     True),
    "gm_estop_err": ("errors",      "estop_error",       True),
    "gm_w_err":     ("errors",      "watchdog_error",    True),
    "gm_cal":       ("errors",      "calibrated",        True),
}


def _serialize_motor_tag(motor_buffers, tag, precision, node_ids):
    """Build one "tag:v0,v1,...,vN" string for a single tag over node_ids.

    Branches once per tag on int-vs-float to keep the inner per-value loop
    free of isinstance() — value type is known statically from the tag.
    """
    buf_name, key, is_int = _MOTOR_TAG_BUFFER_KEY[tag]
    deques = motor_buffers.get(buf_name)
    if deques is None:
        return ""
    formatted = []
    if is_int:
        for i in node_ids:
            d = deques.get(i)
            if not d:
                formatted.append("0")
                continue
            v = d[-1].get(key)
            formatted.append("0" if v is None else str(v))
    else:
        for i in node_ids:
            d = deques.get(i)
            if not d:
                formatted.append("0")
                continue
            v = d[-1].get(key)
            formatted.append("0" if v is None else f"{v:.{precision}f}")
    return f"{tag}:" + ",".join(formatted)


def serialize_motor_data(motor_buffers, fields, precision_map, joint_num):
    """Serialise the latest raw motor sample per arm joint to "tag:v0,v1,...".

    motor_buffers is a dict of {"data", "temperature", "voltage"} → per-node
    deque dicts (Motor_data, Motor_temperature_data, Motor_voltage_data).
    Iterates arm joints 0..joint_num-1 only — the gripper at index joint_num
    is exposed via gm_* tags (see serialize_gripper_motor_data).
    """
    if not motor_buffers:
        return ""
    parts = []
    node_ids = range(joint_num)
    for tag in fields:
        if tag not in _MOTOR_TAG_BUFFER_KEY:
            continue
        precision = precision_map.get(tag, DEFAULT_PRECISION) if precision_map else DEFAULT_PRECISION
        s = _serialize_motor_tag(motor_buffers, tag, precision, node_ids)
        if s:
            parts.append(s)
    return ",".join(parts)


# Config tag → shared-memory array key in shared_data.motor_config.
# Source for both arm (m_*) and gripper (g_*) config tags. Arm tags read
# an array of length joint_num; gripper tags read a single-element array.
_CONFIG_TAG_KEY = {
    "m_KPP":     "cfg_KPP",
    "m_KPV":     "cfg_KPV",
    "m_KIV":     "cfg_KIV",
    "m_KPIQ":    "cfg_KPIQ",
    "m_KIIQ":    "cfg_KIIQ",
    "m_KP":      "cfg_KP",
    "m_KD":      "cfg_KD",
    "m_Ilim":    "cfg_Ilim",
    "m_vel_lim": "cfg_velocity_limit",
    "g_KPP":     "cfg_gripper_KPP",
    "g_KPV":     "cfg_gripper_KPV",
    "g_KIV":     "cfg_gripper_KIV",
    "g_KPIQ":    "cfg_gripper_KPIQ",
    "g_KIIQ":    "cfg_gripper_KIIQ",
    "g_KP":      "cfg_gripper_KP",
    "g_KD":      "cfg_gripper_KD",
    "g_Ilim":    "cfg_gripper_Ilim",
    "g_vel_lim": "cfg_gripper_velocity_limit",
}


def serialize_motor_config(motor_buffers, fields, precision_map, joint_num):
    """Serialise arm motor PID/limit config — one value per joint per tag.

    Reads from motor_buffers["config"], which is the shared_data.motor_config
    dict of numpy arrays. Output: "tag:v0,v1,...,vN-1" with N=joint_num.
    """
    cfg = motor_buffers.get("config") if motor_buffers else None
    if cfg is None:
        return ""
    parts = []
    for tag in fields:
        key = _CONFIG_TAG_KEY.get(tag)
        if key is None:
            continue
        arr = cfg.get(key)
        if arr is None:
            continue
        precision = precision_map.get(tag, DEFAULT_PRECISION) if precision_map else DEFAULT_PRECISION
        formatted = [f"{v:.{precision}f}" for v in arr[:joint_num]]
        parts.append(f"{tag}:" + ",".join(formatted))
    return ",".join(parts)


def serialize_gripper_config(motor_buffers, fields, precision_map):
    """Serialise gripper PID/limit config — single value per tag.

    Reads from motor_buffers["config"]. Gripper config arrays are length 1.
    """
    cfg = motor_buffers.get("config") if motor_buffers else None
    if cfg is None:
        return ""
    parts = []
    for tag in fields:
        key = _CONFIG_TAG_KEY.get(tag)
        if key is None:
            continue
        arr = cfg.get(key)
        if arr is None or len(arr) == 0:
            continue
        precision = precision_map.get(tag, DEFAULT_PRECISION) if precision_map else DEFAULT_PRECISION
        parts.append(f"{tag}:{arr[0]:.{precision}f}")
    return ",".join(parts)


def serialize_gripper_motor_data(motor_buffers, fields, precision_map, gripper_id):
    """Serialise raw motor-side data for the gripper CAN node.

    Same motor_buffers dict as serialize_motor_data, but iterates only the
    single gripper_id (= joint_num). Output is "tag:v0" — one value per tag.
    Position and speed will be 0 when the gripper is in firmware-gripper mode
    (Send_gripper_data_pack); they're populated only in motor mode.
    """
    if not motor_buffers:
        return ""
    parts = []
    node_ids = (gripper_id,)
    for tag in fields:
        if tag not in _MOTOR_TAG_BUFFER_KEY:
            continue
        precision = precision_map.get(tag, DEFAULT_PRECISION) if precision_map else DEFAULT_PRECISION
        s = _serialize_motor_tag(motor_buffers, tag, precision, node_ids)
        if s:
            parts.append(s)
    return ",".join(parts)
