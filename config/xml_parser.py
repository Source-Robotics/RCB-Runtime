"""Parse robot DH-parameter and motor configuration from XML files."""

import logging
import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path


def load_system_config(system_xml_path, robots_dir):
    """
    Read config/system.xml and return the active robot's XML path + joint count.

    This is intentionally lightweight — it only parses two small XML files so
    it can be called before shared_data.setup() at the very top of RTI.py.

    Parameters
    ----------
    system_xml_path : str or Path  — path to config/system.xml
    robots_dir      : str or Path  — path to the robots/ directory

    Returns
    -------
    dict with keys:
        active_robot    : str  — robot name, e.g. "PAR6"
        robot_xml_path  : Path — absolute path to robots/<active_robot>.xml
        joint_num       : int  — number of joints read from that XML
    """
    tree = ET.parse(str(system_xml_path))
    root = tree.getroot()
    active_robot = root.findtext("active_robot", default="").strip()
    if not active_robot:
        raise ValueError(
            f"[system.xml] <active_robot> is empty or missing in {system_xml_path}"
        )

    robot_xml_path = Path(robots_dir) / f"{active_robot}.xml"
    if not robot_xml_path.exists():
        available = [p.stem for p in Path(robots_dir).glob("*.xml")]
        raise FileNotFoundError(
            f"[system.xml] active_robot='{active_robot}' but "
            f"{robot_xml_path} does not exist. "
            f"Available robot XMLs: {available}"
        )

    # Minimal parse — just enough to read the joint count and fallback gripper name
    robot_root, joints_elem = parse_robot_config(str(robot_xml_path))
    joint_num = get_joint_count(joints_elem)

    # active_gripper: prefer system.xml value; fall back to robot XML
    active_gripper = root.findtext("active_gripper", default="").strip()
    if not active_gripper:
        active_gripper = get_active_gripper(robot_root)

    kt_source = root.findtext("kt_source", default="auto").strip().lower()
    if kt_source not in ("auto", "xml"):
        kt_source = "auto"

    jog_defaults = _parse_jog_defaults(root.find("jog_defaults"))
    rti_defaults = _parse_rti_defaults(root.find("rti_defaults"))
    maintenance_defaults = _parse_maintenance_defaults(root.find("maintenance_defaults"))

    return {
        "active_robot":   active_robot,
        "robot_xml_path": robot_xml_path,
        "joint_num":      joint_num,
        "active_gripper": active_gripper,
        "kt_source":      kt_source,
        "jog_defaults":   jog_defaults,
        "rti_defaults":   rti_defaults,
        "maintenance_defaults": maintenance_defaults,
    }


# --- Hardcoded fallbacks if <jog_defaults> is missing or malformed in system.xml.
#     Keep aligned with the example values in the XML schema.
_JOG_DEFAULTS_FALLBACK = {
    "speed_pct":    0.20,
    "accel_time":   0.55,
    "profile":      "scurve",       # "trapezoidal" or "scurve"
    "jerk_factor":  3.0,
    "control_mode": "pid",          # "pid" or "pd"
}


# RTI policy fallbacks. Deliberately mirror the module constants in
# motion/rti_handler.py — that file remains the single definition of each default, and
# these exist so a system.xml without an <rti_defaults> block behaves identically.
_RTI_DEFAULTS_FALLBACK = {
    "command_timeout_s":    0.040,
    "stopping_hold_s":      0.500,
    "start_pose_tol_rad":   0.100,
    "lowpass_cutoff_hz":    0.0,      # 0 = off
    "success_window_ticks": 100,
    "success_warn":         0.95,
    "success_bad":          0.90,
}

_RTI_DEFAULT_BOUNDS = {
    # (min, max) — a nonsense value in the XML should be refused loudly rather than
    # silently disarming a safety feature. A 0 s timeout would stop control on the
    # first tick; a 10 s one would let a dead PC drive the arm for ten seconds.
    "command_timeout_s":    (0.004, 1.0),
    "stopping_hold_s":      (0.0,   5.0),
    "start_pose_tol_rad":   (0.001, 1.0),
    "lowpass_cutoff_hz":    (0.0,   1000.0),
    "success_window_ticks": (10,    10000),
    "success_warn":         (0.0,   1.0),
    "success_bad":          (0.0,   1.0),
}


# Maintenance-mode policy. Robot-INDEPENDENT thresholds, so system.xml rather
# than the robot XML — the value answers "how much gravity load do we accept
# before refusing to let the motors go limp", which is a risk decision, not a
# property of the arm.
_MAINTENANCE_DEFAULTS_FALLBACK = {
    # Max |joint_gravity_torque| on any joint for the arm to count as "parked"
    # when entering FLASHING. joint_gravity_torque IS the torque needed to hold
    # the current pose up, so near-zero means the pose is self-supporting and
    # survives losing torque. MEASURE THIS ON THE REAL ARM at the standby pose
    # and tune — the shipped value is deliberately strict, and the operator can
    # always override with `set_mode:FLASHING:FORCE` after supporting the arm.
    "flash_park_max_gravity_nm": 1.0,
}

_MAINTENANCE_DEFAULT_BOUNDS = {
    # An enormous ceiling would silently disable the park check, so refuse it
    # loudly rather than accept it.
    "flash_park_max_gravity_nm": (0.0, 50.0),
}


def _parse_maintenance_defaults(elem):
    """Read the optional <maintenance_defaults> block from system.xml.

    Same contract as _parse_rti_defaults: a missing block, missing field, or
    out-of-range value falls back to the hardcoded default with a warning,
    never to a value that would disarm the check.
    """
    out = dict(_MAINTENANCE_DEFAULTS_FALLBACK)
    if elem is None:
        return out
    for key, default in _MAINTENANCE_DEFAULTS_FALLBACK.items():
        text = elem.findtext(key, default="").strip()
        if not text:
            continue
        try:
            val = float(text)
        except ValueError:
            logging.warning("[system.xml] <maintenance_defaults><%s> is not a number "
                            "(%r) — using %s", key, text, default)
            continue
        lo, hi = _MAINTENANCE_DEFAULT_BOUNDS[key]
        if not (lo <= val <= hi):
            logging.warning("[system.xml] <maintenance_defaults><%s> = %s is outside "
                            "[%s, %s] — using %s", key, val, lo, hi, default)
            continue
        out[key] = val
    return out


def _parse_rti_defaults(elem):
    """Read the optional <rti_defaults> block from system.xml.

    Missing block, missing field, or out-of-range value all fall back to
    _RTI_DEFAULTS_FALLBACK with a warning — never to a value that would disarm a
    safety feature.
    """
    out = dict(_RTI_DEFAULTS_FALLBACK)
    if elem is None:
        return out
    for key, default in _RTI_DEFAULTS_FALLBACK.items():
        text = elem.findtext(key, default="").strip()
        if not text:
            continue
        try:
            val = int(text) if isinstance(default, int) else float(text)
        except ValueError:
            logging.warning("[system.xml] <rti_defaults><%s> is not a number (%r) — "
                            "using %s", key, text, default)
            continue
        lo, hi = _RTI_DEFAULT_BOUNDS[key]
        if not (lo <= val <= hi):
            logging.warning("[system.xml] <rti_defaults><%s> = %s is outside [%s, %s] — "
                            "using %s", key, val, lo, hi, default)
            continue
        out[key] = val
    if out["success_bad"] > out["success_warn"]:
        logging.warning("[system.xml] success_bad (%s) > success_warn (%s) — the 'bad' "
                        "threshold would fire first. Swapping them.",
                        out["success_bad"], out["success_warn"])
        out["success_warn"], out["success_bad"] = out["success_bad"], out["success_warn"]
    return out


def _parse_jog_defaults(elem):
    """Read the optional <jog_defaults> section from system.xml.

    Missing or malformed individual fields fall back to _JOG_DEFAULTS_FALLBACK
    so older system.xml files (without the section) still load cleanly.
    """
    if elem is None:
        return dict(_JOG_DEFAULTS_FALLBACK)

    out = dict(_JOG_DEFAULTS_FALLBACK)

    speed = elem.findtext("speed_pct", default="").strip()
    if speed:
        try:
            out["speed_pct"] = max(0.0, min(1.0, float(speed)))
        except ValueError:
            pass

    accel = elem.findtext("accel_time", default="").strip()
    if accel:
        try:
            out["accel_time"] = max(0.05, float(accel))
        except ValueError:
            pass

    profile = elem.findtext("profile", default="").strip().lower()
    if profile in ("trapezoidal", "scurve"):
        out["profile"] = profile

    jerk = elem.findtext("jerk_factor", default="").strip()
    if jerk:
        try:
            out["jerk_factor"] = max(0.5, float(jerk))
        except ValueError:
            pass

    control_mode = elem.findtext("control_mode", default="").strip().lower()
    if control_mode in ("pid", "pd"):
        out["control_mode"] = control_mode

    return out

def parse_robot_config(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    joints = root.find('joints')
    return root, joints

def get_robot_name(root):
    return root.findtext('name', default='Unknown').strip()

def get_joint_count(joints):
    return int(joints.get('count', default='0'))

def get_active_gripper(root):
    return root.findtext('active_gripper', default='Unknown').strip()

def get_single_value(joints, joint_id, tag):
    joint = joints.find(f"./joint[@id='{joint_id}']")
    if joint is not None:
        element = joint.find(tag)
        if element is not None:
            return _convert_to_float(element.text.strip())
    return None

def get_float_value(root, tag):
    text = root.findtext(tag)
    if text:
        try:
            return float(text.strip())
        except ValueError:
            return None
    return None


def get_child_float(parent, tag, default):
    if parent is None:
        return default
    text = parent.findtext(tag)
    if text is None:
        return default
    try:
        return float(text.strip())
    except ValueError:
        return default


def get_safety_config(root):
    """Read <safety> block from robot XML with sensible defaults."""
    safety = root.find("safety")
    return {
        "max_joint_velocity": get_child_float(safety, "max_joint_velocity", 3.14),
        "joint_velocity_sustain_ticks": int(get_child_float(safety, "joint_velocity_sustain_ticks", 20)),
        "warn_motor_temperature": get_child_float(safety, "warn_motor_temperature", 80.0),
        "max_motor_temperature": get_child_float(safety, "max_motor_temperature", 90.0),
        "current_spike_factor": get_child_float(safety, "current_spike_factor", 2.0),
        "current_spike_lookback": int(get_child_float(safety, "current_spike_lookback", 30)),
        "collision_torque_threshold": get_child_float(safety, "collision_torque_threshold", 5.0),
        "vibration_speed_variance_threshold": get_child_float(safety, "vibration_speed_variance_threshold", 2.0e7),
        "vibration_lookback": int(get_child_float(safety, "vibration_lookback", 25)),
        "loop_overrun_limit": int(get_child_float(safety, "loop_overrun_limit", 10)),
        "can_error_limit": int(get_child_float(safety, "can_error_limit", 1)),
        "warning_speed_scale": get_child_float(safety, "warning_speed_scale", 0.7),
        # Per-check enable flags (1 = enabled, 0 = disabled)
        "check_joint_position":      int(get_child_float(safety, "check_joint_position",      1)),
        "check_joint_velocity":      int(get_child_float(safety, "check_joint_velocity",      1)),
        "check_motor_temperature":   int(get_child_float(safety, "check_motor_temperature",   1)),
        "check_current_spike":       int(get_child_float(safety, "check_current_spike",       1)),
        "check_torque_discontinuity":int(get_child_float(safety, "check_torque_discontinuity",1)),
        "check_vibration":           int(get_child_float(safety, "check_vibration",           1)),
        "check_loop_overrun":        int(get_child_float(safety, "check_loop_overrun",        1)),
        "check_can_health":          int(get_child_float(safety, "check_can_health",          1)),
        "check_estop":               int(get_child_float(safety, "check_estop",               1)),
    }


# Derivation constants for motion limits absent from a robot XML. Kept here rather than
# inline so the fallback formula has one home and matches what PAR6.xml documents.
_DERIVED_ACCEL_TIME_S = 0.5    # ramp time EXEC (ACCEL_TIME) and JOG already assume
_DERIVED_JERK_FACTOR = 50.0   # accel reaches full scale in ~20 ms; see robots/PAR6.xml joint 0 for why not ruckig's 3x (too slow) or a 500x factor (inert at a 4 ms tick)
_DERIVED_TRQ_RATE_PER_TAU = 11.5   # torque-rate-to-peak-torque ratio; a motion-smoothness limit, well below the electrical ceiling


def _fill_motion_limit_defaults(cfg):
    """Fill any NaN acceleration / jerk / torque-rate limit with a derived value.

    These three fields are optional so robot XMLs written before them still load. A NaN
    reaching the rate limiter would silently disable it for that joint (every comparison
    against NaN is False), which is the failure mode this exists to prevent.

    Derivations, in order of how much they should be trusted:
      torque rate — from real motor specs (kt, Ilim, gear ratio, efficiency) scaled by a
                    published industrial ratio. Defensible as-is.
      acceleration — from the ramp-time convention already baked into EXEC and JOG. NOT
                    physically derived; the true limit is available-torque / effective-
                    inertia and is pose-dependent.
      jerk        — a multiple of acceleration, so it inherits acceleration's weakness.

    Mutates cfg in place and returns it.
    """
    import numpy as _np

    vel = _np.asarray(cfg['joint_velocity_limit'], dtype=float)
    accel = _np.asarray(cfg['joint_acceleration_limit'], dtype=float)
    jerk = _np.asarray(cfg['joint_jerk_limit'], dtype=float)
    trq_rate = _np.asarray(cfg['torque_rate_limit'], dtype=float)

    # Some robot XMLs (INDIGO, SKAR) only carry the MOTOR-space Velocity_limit in ticks/s
    # and no joint_velocity_limit at all, which would leave the accel fallback with nothing
    # to work from. Convert: ticks/s -> motor rad/s (2*pi / 2^encoder) -> joint rad/s (/gear).
    # Verified against PAR6, which specifies both: 80000 ticks/s, 14-bit, 6.4:1 -> 4.79 rad/s
    # vs its declared 4.8.
    if _np.isnan(vel).any():
        vel = _np.where(
            _np.isnan(vel),
            (_np.asarray(cfg['motor_velocity_limit'], dtype=float)
             * (2.0 * _np.pi / _np.power(2.0, _np.asarray(cfg['motor_encoder'], dtype=float)))
             / _np.asarray(cfg['gear_ratio'], dtype=float)),
            vel)

    derived_accel = vel / _DERIVED_ACCEL_TIME_S
    accel = _np.where(_np.isnan(accel), derived_accel, accel)

    jerk = _np.where(_np.isnan(jerk), accel * _DERIVED_JERK_FACTOR, jerk)

    peak_torque = (_np.asarray(cfg['motor_kt'], dtype=float)
                   * _np.asarray(cfg['motor_Ilim'], dtype=float) / 1000.0
                   * _np.asarray(cfg['gear_ratio'], dtype=float)
                   * _np.asarray(cfg['gear_efficiency'], dtype=float))
    trq_rate = _np.where(_np.isnan(trq_rate), peak_torque * _DERIVED_TRQ_RATE_PER_TAU, trq_rate)

    cfg['joint_acceleration_limit'] = accel
    cfg['joint_jerk_limit'] = jerk
    cfg['torque_rate_limit'] = trq_rate
    return cfg


# Per-mode limit -> (cfg key, which hardware ceiling it may not exceed).
# EXEC has no jerk/torque-rate entry on purpose: nothing outside RTI reads either, so an
# EXEC copy would be a field with no reader.
_MODE_LIMIT_MAP = {
    'exec': [('velocity', 'exec_velocity_limit', 'joint_velocity_limit'),
             ('acceleration', 'exec_acceleration_limit', 'joint_acceleration_limit'),
             ('jerk', 'exec_jerk_limit', 'joint_jerk_limit')],
    'rti':  [('velocity', 'rti_velocity_limit', 'joint_velocity_limit'),
             ('acceleration', 'rti_acceleration_limit', 'joint_acceleration_limit'),
             ('jerk', 'rti_jerk_limit', 'joint_jerk_limit'),
             ('torque_rate', 'rti_torque_rate_limit', 'torque_rate_limit')],
}


def _fill_mode_limit_defaults(cfg):
    """Resolve per-mode <exec_limits> / <rti_limits> against the hardware ceiling.

    The <joint_*_limit> fields are what the ARM survives. A per-mode block says what one
    mode should ask for WITHIN that, so the two are not the same kind of number and must not
    be allowed to merge: a mode may lower a limit, never raise it.

    Two rules, both deliberate:

      MISSING -> the ceiling. A robot XML written before per-mode blocks existed keeps
        behaving exactly as it did, and a half-filled block only overrides what it states.

      ABOVE THE CEILING -> clamped, with a warning. Silently honouring it would let an
        edit to a mode block raise a hardware limit, which is the whole thing this split
        exists to prevent. Silently ignoring it would leave the operator believing a number
        that is not in force. Clamp and say so.

    Mutates cfg in place and returns it.
    """
    import numpy as _np
    _log = logging.getLogger(__name__)

    for mode, entries in _MODE_LIMIT_MAP.items():
        for _tag, key, ceiling_key in entries:
            ceiling = _np.asarray(cfg[ceiling_key], dtype=float)
            vals = _np.asarray(cfg.get(key, _np.full_like(ceiling, _np.nan)), dtype=float)
            vals = _np.where(_np.isnan(vals), ceiling, vals)

            over = vals > ceiling
            if over.any():
                for i in _np.where(over)[0]:
                    _log.warning(
                        "[XML] %s joint %d is %.4g, above the hardware limit %s=%.4g — "
                        "clamped. A per-mode block can lower a limit, never raise it.",
                        key, int(i), float(vals[i]), ceiling_key, float(ceiling[i]))
                vals = _np.minimum(vals, ceiling)

            cfg[key] = vals

    return cfg


def get_all_values(joints, tag):
    values = []
    for joint in joints.findall("joint"):
        element = joint.find(tag)
        if element is not None:
            values.append(_convert_to_float(element.text.strip()))
        else:
            values.append(np.nan)
    return np.array(values)

def get_all_strings(joints, tag, default=None):
    """Per-joint STRING values (get_all_values coerces to float, which would turn
    a driver name into NaN). Missing entries become `default` — for driver_type
    that is None meaning "not declared", never a guessed product."""
    values = []
    for joint in joints.findall("joint"):
        element = joint.find(tag)
        text = (element.text or "").strip() if element is not None else ""
        values.append(text.lower() if text else default)
    return values


def get_gripper_values(grippers, gripper_name, tag):
    """
    Returns a single value from a named gripper (embedded <grippers> element).
    """
    gripper = grippers.find(f"./gripper[@name='{gripper_name}']")
    if gripper is None:
        raise ValueError(f"Gripper '{gripper_name}' not found")

    element = gripper.find(tag)
    if element is None or element.text is None:
        return np.nan

    return _convert_to_float(element.text.strip())


def load_gripper_element(grippers_dir, gripper_name):
    """
    Parse a standalone gripper XML file from grippers_dir and return its root element.
    File is expected at: grippers_dir / f"{gripper_name}.xml"
    Returns None if the file does not exist (robot has no gripper).
    """
    path = Path(grippers_dir) / f"{gripper_name}.xml"
    if not path.exists():
        return None
    tree = ET.parse(str(path))
    return tree.getroot()


def get_gripper_value(gripper_elem, tag):
    """
    Return a single value from a standalone gripper element.
    Returns np.nan when the element or its text is missing.
    """
    if gripper_elem is None:
        return np.nan
    element = gripper_elem.find(tag)
    if element is None or element.text is None:
        return np.nan
    return _convert_to_float(element.text.strip())

# Motor/gripper voltage limit in mV, used when an XML predates the <voltage_limit> tag.
# 0 means "use VBUS" on the driver side, so it is a valid value and cannot be the missing marker.
DEFAULT_VOLTAGE_LIMIT_MV = 6000.0

# Hard floor for a REAL voltage limit. 0 is exempt — it is the driver's "use VBUS" sentinel, not
# a low limit. So the rule is: 0, or >= MIN_VOLTAGE_LIMIT_MV. Nothing in between ever reaches a
# driver. Enforced in three places so no path can get under it — XML load, live edits (RTI's
# set_pid_gains), and the CAN send itself. A GUI minimum alone would not be a guarantee: an XML
# value or a robotclient call could still ask for less.
MIN_VOLTAGE_LIMIT_MV = 6000.0


def clamp_voltage_limit(value):
    """The single definition of the voltage-limit rule, used by every path that can set one.

    0 passes through untouched — it tells the driver to use VBUS, so it is a mode, not a limit.
    Any other value (including negatives) is clamped UP to MIN_VOLTAGE_LIMIT_MV, because an
    under-limited motor cannot hold or move properly. Accepts a scalar or an array.
    """
    arr = np.asarray(value, dtype=float)
    out = np.where(arr == 0.0, 0.0, np.maximum(arr, MIN_VOLTAGE_LIMIT_MV))
    return float(out) if out.ndim == 0 else out


def _voltage_limit_or_default(value):
    """<voltage_limit> is optional — a missing tag reads as NaN (get_all_values/get_gripper_value).
    Fall back to the default so older XMLs without the tag keep loading, then apply the rule."""
    arr = np.asarray(value, dtype=float)
    filled = np.where(np.isnan(arr), DEFAULT_VOLTAGE_LIMIT_MV, arr)
    clamped = np.asarray(clamp_voltage_limit(filled), dtype=float)
    if np.any(clamped != filled):
        logging.warning("[XML] voltage_limit below the %.0f mV floor (and not the 0 = use-VBUS "
                        "sentinel) — clamped up.", MIN_VOLTAGE_LIMIT_MV)
    return float(clamped) if clamped.ndim == 0 else clamped


def set_single_value(joints, joint_id, tag, value):
    joint = joints.find(f"./joint[@id='{joint_id}']")
    if joint is not None:
        element = joint.find(tag)
        if element is None:
            element = ET.SubElement(joint, tag)
        element.text = str(value)

def set_all_values(joints, tag, values):
    for i, joint in enumerate(joints.findall("joint")):
        if i >= len(values):
            break
        element = joint.find(tag)
        if element is None:
            element = ET.SubElement(joint, tag)
        element.text = str(values[i])

def save_robot_config(root, output_path):
    tree = ET.ElementTree(root)
    tree.write(output_path, encoding='utf-8', xml_declaration=True)

def set_value(root, tag, value):
    element = root.find(tag)
    if element is None:
        element = ET.SubElement(root, tag)
    element.text = str(value)

def _convert_to_float(text):
    try:
        return float(text)
    except ValueError:
        return text


import logging

_val_logger = logging.getLogger(__name__)

# Required per-joint XML tags — must exist and be a finite positive number
_REQUIRED_JOINT_TAGS = ["kt", "Ilim", "gear_ratio", "encoder",
                        "joint_limit_negative", "joint_limit_positive"]

# Valid encoder resolutions in bits
_VALID_ENCODER_BITS = set(range(8, 21))   # 8 – 20 bit


def validate_robot_config(cfg: dict) -> tuple[bool, list[str]]:
    """
    Validate the dict returned by load_full_robot_config().

    Returns
    -------
    ok      : True if no ERROR-level issue was found (WARNINGs are non-fatal)
    issues  : list of human-readable strings, prefixed with [ERROR] or [WARNING]
    """
    issues: list[str] = []

    # ── joint count ──────────────────────────────────────────────────────────
    jn = cfg.get("joint_num", 0)
    if not isinstance(jn, int) or jn <= 0:
        issues.append(f"[ERROR] joint_num is {jn!r} — must be a positive integer")
        # Can't continue checking per-joint arrays without a valid count
        return False, issues

    # ── per-joint required fields ─────────────────────────────────────────────
    array_map = {
        "kt":                   cfg.get("motor_kt"),
        "Ilim":                 cfg.get("motor_Ilim"),
        "gear_ratio":           cfg.get("gear_ratio"),
        "encoder":              cfg.get("motor_encoder"),
        "joint_limit_negative": cfg.get("joint_limit_negative"),
        "joint_limit_positive": cfg.get("joint_limit_positive"),
    }
    for tag, arr in array_map.items():
        if arr is None:
            issues.append(f"[ERROR] '{tag}' array is missing from config")
            continue
        arr = np.asarray(arr, dtype=float)
        if len(arr) != jn:
            issues.append(
                f"[ERROR] '{tag}' has {len(arr)} entries but joint_num={jn}"
            )
        if np.any(np.isnan(arr)):
            bad = np.where(np.isnan(arr))[0].tolist()
            issues.append(f"[ERROR] '{tag}' missing on joint(s) {bad}")

    # ── range checks (only when arrays are present and full-length) ───────────
    def _arr(name):
        a = cfg.get(name)
        return np.asarray(a, dtype=float) if a is not None else None

    kt = _arr("motor_kt")
    if kt is not None and not np.any(np.isnan(kt)):
        bad = np.where(kt <= 0)[0].tolist()
        if bad:
            issues.append(f"[ERROR] kt <= 0 on joint(s) {bad} (must be positive)")

    ilim = _arr("motor_Ilim")
    if ilim is not None and not np.any(np.isnan(ilim)):
        bad = np.where(ilim <= 0)[0].tolist()
        if bad:
            issues.append(f"[ERROR] Ilim <= 0 on joint(s) {bad} (must be positive mA)")

    gr = _arr("gear_ratio")
    if gr is not None and not np.any(np.isnan(gr)):
        bad = np.where(gr <= 0)[0].tolist()
        if bad:
            issues.append(f"[ERROR] gear_ratio <= 0 on joint(s) {bad}")

    enc = _arr("motor_encoder")
    if enc is not None and not np.any(np.isnan(enc)):
        for i, e in enumerate(enc):
            if int(e) not in _VALID_ENCODER_BITS:
                issues.append(
                    f"[WARNING] encoder={int(e)} on joint {i} is unusual "
                    f"(expected {min(_VALID_ENCODER_BITS)}–{max(_VALID_ENCODER_BITS)} bits)"
                )

    lim_neg = _arr("joint_limit_negative")
    lim_pos = _arr("joint_limit_positive")
    if lim_neg is not None and lim_pos is not None:
        if len(lim_neg) == jn and len(lim_pos) == jn:
            for i, (ln, lp) in enumerate(zip(lim_neg, lim_pos)):
                if np.isnan(ln) or np.isnan(lp):
                    continue
                # Convention A: both positive magnitudes (neg limit applied in code as -ln)
                # Convention B: ln is already negative, lp is positive
                if ln >= 0 and lp >= 0:
                    # magnitude convention — just check they are non-zero
                    if ln == 0 and lp == 0:
                        issues.append(f"[WARNING] joint {i}: both joint limits are 0")
                else:
                    # signed convention — neg must be strictly less than pos
                    if ln >= lp:
                        issues.append(
                            f"[ERROR] joint {i}: joint_limit_negative ({ln}) >= "
                            f"joint_limit_positive ({lp})"
                        )

    # ── loop timing ───────────────────────────────────────────────────────────
    dt = cfg.get("delta_t")
    if dt is None:
        issues.append("[ERROR] RTI_loop_time missing from XML")
    elif not (1e-5 <= float(dt) <= 0.5):
        issues.append(
            f"[WARNING] delta_t={dt} s looks unusual (expected 10 µs – 500 ms)"
        )

    # ── gripper ───────────────────────────────────────────────────────────────
    gripper_name = cfg.get("gripper_name", "")
    if gripper_name and gripper_name.lower() not in ("none", "unknown", ""):
        gripper_kt = cfg.get("gripper_kt")
        if gripper_kt is None or (isinstance(gripper_kt, float) and np.isnan(gripper_kt)):
            issues.append(
                f"[WARNING] gripper '{gripper_name}' kt not found — "
                f"gripper XML may be missing or mis-named"
            )
        gripper_gear_r = cfg.get("gripper_gear_r")
        if gripper_gear_r is None or (isinstance(gripper_gear_r, float) and np.isnan(gripper_gear_r)):
            issues.append(
                f"[WARNING] gripper '{gripper_name}' Gear_r not found — "
                f"gripper XML may be missing or mis-named"
            )

    # ── result ────────────────────────────────────────────────────────────────
    ok = not any(i.startswith("[ERROR]") for i in issues)
    for msg in issues:
        if msg.startswith("[ERROR]"):
            _val_logger.error(f"[XML Validation] {msg}")
        else:
            _val_logger.warning(f"[XML Validation] {msg}")
    if ok and not issues:
        _val_logger.info("[XML Validation] OK — all checks passed")
    return ok, issues


def load_full_robot_config(xml_path, grippers_dir=None, active_gripper=None):
    """
    Parse the robot XML and return a flat dict with all motor and gripper parameters.

    grippers_dir:   path to the grippers/ folder containing individual gripper XML
                    files (e.g. grippers/SSG48.xml).  Pass GRIPPERS_DIR from
                    project_paths for normal usage.  When omitted, falls back to an
                    embedded <grippers> block inside the robot XML (legacy).
    active_gripper: gripper name override (e.g. from system.xml).  When provided
                    this takes precedence over <active_gripper> in the robot XML.

    RTI.py (and any other caller) can do:
        cfg = load_full_robot_config(xml_path, grippers_dir=GRIPPERS_DIR,
                                     active_gripper=_sys['active_gripper'])
        joint_num = cfg['joint_num']
        motor_kt  = cfg['motor_kt']
        ...
    The raw ElementTree objects are included as '_root' and '_joints' for callers
    that still need to call set_single_value / save_robot_config.
    """
    root, joints = parse_robot_config(xml_path)
    gripper_name = active_gripper if active_gripper else get_active_gripper(root)

    if grippers_dir is not None:
        gripper_elem = load_gripper_element(grippers_dir, gripper_name)
        def _gval(tag): return get_gripper_value(gripper_elem, tag)
    else:
        # Legacy fallback: gripper params embedded inside the robot XML
        grippers = root.find("grippers")
        def _gval(tag): return get_gripper_values(grippers, gripper_name, tag)
        gripper_elem = None

    def _gstr(tag, default=None):
        """Gripper STRING field — _gval coerces to float. None = not declared."""
        if gripper_elem is None:
            return default
        text = gripper_elem.findtext(tag, default="").strip()
        return text.lower() if text else default

    cfg = {
        # ---- metadata ----
        'robot_name':   get_robot_name(root),
        'joint_num':    get_joint_count(joints),

        # ---- per-joint motor arrays ----
        # Which firmware product each joint's driver board runs. Values match the
        # ids in tools/firmware_github.py FIRMWARE_PRODUCTS ('stepfoc',
        # 'spectral-bldc') ON PURPOSE — one vocabulary end to end, so no
        # translation table can drift. None = not declared, which the flasher
        # treats as "ask the operator", never as a guess: flashing the wrong
        # product is the mistake a board cannot report and cannot self-correct.
        'motor_driver_type':      get_all_strings(joints, 'driver_type'),
        'motor_kt':               get_all_values(joints, 'kt'),
        'motor_Ilim':             get_all_values(joints, 'Ilim'),
        'motor_voltage_limit':    _voltage_limit_or_default(get_all_values(joints, 'voltage_limit')),
        'motor_velocity_limit':   get_all_values(joints, 'Velocity_limit'),
        'joint_velocity_limit':   get_all_values(joints, 'joint_velocity_limit'),
        # Motion limits used by the RTI-mode command rate limiter (TODO/RTI_mode_plan.md §11b).
        # Optional in the XML — any joint missing one gets a derived value from
        # _fill_motion_limit_defaults() below, so robots predating these fields still load.
        'joint_acceleration_limit': get_all_values(joints, 'joint_acceleration_limit'),
        'joint_jerk_limit':         get_all_values(joints, 'joint_jerk_limit'),
        'torque_rate_limit':        get_all_values(joints, 'torque_rate_limit'),
        # PER-MODE limits (<exec_limits> / <rti_limits> per joint). The three fields above
        # are the HARDWARE CEILING; these say what one mode asks for within it, so that
        # opening acceleration up to tune RTI no longer makes every queued EXEC move
        # punchier as a side effect. Missing -> the ceiling; above it -> clamped with a
        # warning (_fill_mode_limit_defaults). find() takes a path, so the nested tags need
        # no change to get_all_values.
        'exec_velocity_limit':     get_all_values(joints, 'exec_limits/velocity'),
        'exec_acceleration_limit': get_all_values(joints, 'exec_limits/acceleration'),
        'exec_jerk_limit':         get_all_values(joints, 'exec_limits/jerk'),
        'rti_velocity_limit':      get_all_values(joints, 'rti_limits/velocity'),
        'rti_acceleration_limit':  get_all_values(joints, 'rti_limits/acceleration'),
        'rti_jerk_limit':          get_all_values(joints, 'rti_limits/jerk'),
        'rti_torque_rate_limit':   get_all_values(joints, 'rti_limits/torque_rate'),
        'motor_watchdog_rate':    get_all_values(joints, 'watchdog_rate'),
        'motor_KPP':              get_all_values(joints, 'KPP'),
        'motor_KPV':              get_all_values(joints, 'KPV'),
        'motor_KIV':              get_all_values(joints, 'KIV'),
        'motor_KIIQ':             get_all_values(joints, 'KIIQ'),
        'motor_KPIQ':             get_all_values(joints, 'KPIQ'),
        'motor_KP':               get_all_values(joints, 'KP'),
        'motor_KD':               get_all_values(joints, 'KD'),
        'motor_encoder':          get_all_values(joints, 'encoder'),
        'joint_limit_negative':             get_all_values(joints, 'joint_limit_negative'),
        'joint_limit_positive':             get_all_values(joints, 'joint_limit_positive'),
        'software_joint_limit_negative':    get_all_values(joints, 'software_joint_limit_negative'),
        'software_joint_limit_positive':    get_all_values(joints, 'software_joint_limit_positive'),
        'master_position':        get_all_values(joints, 'sector_home_master_position'),
        'gear_ratio':             get_all_values(joints, 'gear_ratio'),
        'gear_efficiency':        get_all_values(joints, 'gear_efficiency'),
        'motor_offset':           get_all_values(joints, 'sector_home_offset'),
        'motor_dir':              get_all_values(joints, 'dir'),
        'home_offset':                    get_all_values(joints, 'home_offset'),
        'home_offset_gripper_dependent':  get_all_values(joints, 'home_offset_gripper_dependent'),
        'homing_speed':           get_all_values(joints, 'homing_speed'),
        'homing_direction':       get_all_values(joints, 'homing_direction'),
        'homing_current_ma':      get_all_values(joints, 'homing_current_ma'),
        'homing_timeout_s':       get_all_values(joints, 'homing_timeout_s'),
        'homing_backoff_s':       get_all_values(joints, 'homing_backoff_s'),
        'has_hall_sensor':        get_all_values(joints, 'has_hall_sensor'),
        'homing_release_current_ma': get_all_values(joints, 'homing_release_current_ma'),
        'homing_release_duration_s': get_all_values(joints, 'homing_release_duration_s'),
        'homing_release_sample_pct': get_all_values(joints, 'homing_release_sample_pct'),
        'homing_two_pass_max_diff':  get_all_values(joints, 'homing_two_pass_max_diff'),

        # ---- gripper scalars ----
        'gripper_name':           gripper_name,
        'gripper_driver_type':    _gstr('driver_type'),
        'gripper_stroke':         _gval('Stroke'),
        'gripper_kt':             _gval('kt'),
        'gripper_gear_r':         _gval('Gear_r'),
        'gripper_Ilim':           _gval('Ilim'),
        'gripper_voltage_limit':  _voltage_limit_or_default(_gval('voltage_limit')),
        'gripper_velocity_limit': _gval('Velocity_limit'),
        'gripper_watchdog_rate':  _gval('watchdog_rate'),
        'gripper_KPP':            _gval('KPP'),
        'gripper_KPV':            _gval('KPV'),
        'gripper_KIV':            _gval('KIV'),
        'gripper_KIIQ':           _gval('KIIQ'),
        'gripper_KPIQ':           _gval('KPIQ'),
        'gripper_KP':             _gval('KP'),
        'gripper_KD':             _gval('KD'),

        # ---- gripper homing (only present for CAN grippers with homing XML) ----
        'gripper_CAN_gripper':         _gval('CAN_gripper'),
        'gripper_homing_speed':        _gval('homing_speed'),
        'gripper_homing_direction':    _gval('homing_direction'),
        'gripper_homing_current_ma':   _gval('homing_current_ma'),
        'gripper_has_hall_sensor':     _gval('has_hall_sensor'),
        'gripper_home_offset':         _gval('home_offset'),
        'gripper_homing_timeout_s':    _gval('homing_timeout_s'),
        'gripper_homing_two_pass':     _gval('homing_two_pass'),
        'gripper_homing_two_pass_max_diff': _gval('homing_two_pass_max_diff'),
        'gripper_homing_backoff_s':    _gval('homing_backoff_s'),
        'gripper_homing_order':        _gval('homing_order'),
        'gripper_post_home_position':  _gval('post_home_position'),
        'gripper_post_home_speed':     _gval('post_home_speed'),

        # ---- timing ----
        'delta_t':                get_float_value(root, 'RTI_loop_time'),
        'delta_t_RTI_joint':      get_float_value(root, 'RTI_Joint_mode_loop_time'),
        'safety':                 get_safety_config(root),

        # ---- keep raw objects for set/save operations ----
        '_root':         root,
        '_joints':       joints,
        '_gripper_elem': gripper_elem,   # standalone gripper element (or None)
    }

    # ---- optional park pose ----
    # Where the arm is sent before maintenance that drops motor torque. Optional:
    # a robot without one simply has no "Move to park" button, and the operator
    # parks by hand and confirms. Silently dropped if it is malformed or the
    # wrong length — a bad park pose must not become a move command, and it must
    # not stop the runtime from starting either.
    cfg['park_pose'] = None
    park_tag = root.find('park_pose')
    if park_tag is not None and park_tag.text:
        try:
            values = [float(v) for v in park_tag.text.split()]
        except ValueError:
            logging.warning("[robot.xml] <park_pose> is not a list of numbers — ignored")
        else:
            if len(values) == cfg['joint_num']:
                cfg['park_pose'] = values
            else:
                logging.warning(
                    "[robot.xml] <park_pose> has %d values but the robot has %d "
                    "joints — ignored", len(values), cfg['joint_num'])

    # ---- optional homing sequence file ----
    homing_config_tag = root.find('homing_config')
    if homing_config_tag is not None and homing_config_tag.text:
        homing_xml_path = Path(xml_path).parent.parent / homing_config_tag.text.strip()
        cfg['homing_sequence'] = parse_homing_sequence(str(homing_xml_path))

    # ---- gripper-dependent home_offset overrides ----
    # For joints marked home_offset_gripper_dependent=1, replace the robot XML value
    # with the per-joint offset from the active gripper's <arm_joint_home_offsets> block.
    if gripper_elem is not None:
        offsets_elem = gripper_elem.find('arm_joint_home_offsets')
        if offsets_elem is not None:
            gripper_joint_offsets = {}
            for jelem in offsets_elem.findall('joint'):
                jid = jelem.get('id')
                if jid is not None and jelem.text:
                    try:
                        gripper_joint_offsets[int(jid)] = float(jelem.text.strip())
                    except ValueError:
                        pass

            dep_flags = cfg['home_offset_gripper_dependent']
            home_offsets = cfg['home_offset'].copy()
            for j, flag in enumerate(dep_flags):
                if not np.isnan(flag) and int(flag) == 1:
                    if j in gripper_joint_offsets:
                        home_offsets[j] = gripper_joint_offsets[j]
                    # If flag is set but gripper has no entry, keep the robot XML fallback
            cfg['home_offset'] = home_offsets

    _fill_motion_limit_defaults(cfg)
    # Must run AFTER the ceilings are resolved — it clamps against them, and a NaN ceiling
    # would make every comparison False and silently let a mode limit through unclamped.
    _fill_mode_limit_defaults(cfg)
    return cfg


# ---------------------------------------------------------------------------
# Homing sequence parser
# ---------------------------------------------------------------------------

def parse_homing_sequence(xml_path):
    """
    Parse a homing sequence XML file and return a list of step dicts.

    Each step dict contains:
        pre_moves   : list of dicts with keys: joint (int), position_rad (float), speed (float)
        home_joints : list of int joint indices (or 'gripper' string for gripper)
        gripper_mode: None | 'motor' | 'firmware'  (only set when 'gripper' is in home_joints)
        post_moves  : list of dicts (same format as pre_moves; only on the last entry)

    Returns:
        list of step dicts, plus an optional trailing post_moves step
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()

    steps = []

    for step_elem in root.findall('step'):
        step = {'pre_moves': [], 'home_joints': [], 'gripper_mode': None, 'move_to': []}

        # Parse pre_moves
        pre_moves_elem = step_elem.find('pre_moves')
        if pre_moves_elem is not None:
            for move_elem in pre_moves_elem.findall('move'):
                step['pre_moves'].append({
                    'type': 'position',
                    'joint': int(move_elem.get('joint')),
                    'position_rad': float(move_elem.get('position_rad')),
                    'speed': float(move_elem.get('speed')),
                })
            for nudge_elem in pre_moves_elem.findall('nudge'):
                step['pre_moves'].append({
                    'type': 'nudge',
                    'joint': int(nudge_elem.get('joint')),
                    'speed': float(nudge_elem.get('speed')),
                    'duration_s': float(nudge_elem.get('duration_s')),
                })
            for idle_elem in pre_moves_elem.findall('idle'):
                step['pre_moves'].append({
                    'type': 'idle',
                    'joint': int(idle_elem.get('joint')),
                    'duration_s': float(idle_elem.get('duration_s')),
                })
            for gm_elem in pre_moves_elem.findall('gripper_move'):
                # position and speed are 0-255; current is mA (2 bytes).
                # activate, action, ESTOP, release_dir are bitfield flags (0/1).
                step['pre_moves'].append({
                    'type':        'gripper_move',
                    'joint':       'gripper',   # resolved to _arm_count at runtime
                    'position':    int(gm_elem.get('position')),
                    'speed':       int(gm_elem.get('speed')),
                    'current':     int(gm_elem.get('current')),
                    'activate':    int(gm_elem.get('activate',    1)),
                    'action':      int(gm_elem.get('action',      1)),
                    'estop':       int(gm_elem.get('estop',       0)),
                    'release_dir': int(gm_elem.get('release_dir', 0)),
                    'duration_s':  float(gm_elem.get('duration_s')),
                })

        # Parse home joints
        home_elem = step_elem.find('home')
        if home_elem is not None:
            joints_str = home_elem.get('joints', '')
            joint_list = []
            has_gripper = False
            for tok in joints_str.split(','):
                tok = tok.strip()
                if tok.lower() == 'gripper':
                    has_gripper = True
                    joint_list.append('gripper')
                else:
                    joint_list.append(int(tok))
            step['home_joints'] = joint_list

            if has_gripper:
                step['gripper_mode'] = home_elem.get('mode', 'motor')

        # Parse move_to elements (smooth joint-space moves)
        for mt_elem in step_elem.findall('move_to'):
            step['move_to'].append({
                'joint':        int(mt_elem.get('joint')),
                'position_rad': float(mt_elem.get('position_rad')),
                'duration_s':   float(mt_elem.get('duration_s')),
            })

        steps.append(step)

    # Parse optional global post_moves (after all steps)
    post_moves_elem = root.find('post_moves')
    if post_moves_elem is not None:
        post_moves = []
        for move_elem in post_moves_elem.findall('move'):
            post_moves.append({
                'joint': int(move_elem.get('joint')),
                'position_rad': float(move_elem.get('position_rad')),
                'speed': float(move_elem.get('speed')),
            })
        if post_moves:
            steps.append({
                'pre_moves': [],
                'home_joints': [],
                'gripper_mode': None,
                'post_moves': post_moves,
            })

    return steps


# Example usage:  python3 -m config.xml_parser   (from the repo root)
#
# Run as a MODULE, not as a path. `python3 config/xml_parser.py` puts config/ on sys.path
# instead of the repo root, so `project_paths` is not importable and this block dies before
# it does anything — which is how it sat broken, along with an import of PAROL6_XML, a
# constant that has never existed in project_paths.py (only PAR6_XML does).
if __name__ == "__main__":
    from project_paths import PAR6_XML, PROJECT_ROOT
    xml_path = str(PAR6_XML)
    output_path = str(PROJECT_ROOT / "PAR6_modified.xml")

    # Load and parse
    root, joints = parse_robot_config(xml_path)

    # Robot metadata
    print("Robot Name:", get_robot_name(root))
    print("Active Gripper:", get_active_gripper(root))

    # Single value
    print("KD of joint 2:", get_single_value(joints, 2, "KD"))

    joint_num = get_joint_count(joints)
    print("Joint count:", joint_num)

    print("joint num", get_single_value(joints, 2, "KD"))
    # All gear ratios
    var = get_all_values(joints, "gear_ratio")
    print(type(var))
    print("Gear Ratios:", get_all_values(joints, "gear_ratio"))
    print("joint type:", get_all_values(joints, "joint_type"))
    print("delta_t:",get_float_value(root,"RTI_loop_time"))
    print("delta_t_RTI_joint:",get_float_value(root,"RTI_Joint_mode_loop_time"))
    joint_num = get_joint_count(joints) 
    print(type(joint_num))
    # Modify a single value
    set_single_value(joints, 2, "KP", 3.1415)

    # Modify an array of values
    #new_kd_array = np.ones(len(joints.findall("joint"))) * 5.0
    #set_all_values(joints, "KD", new_kd_array)

    # Save updated XML
    #save_robot_config(root, output_path)
    #print(f"Modified XML saved to: {output_path}")
