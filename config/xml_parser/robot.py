"""The robot XML (+ its gripper XML) as one config dict — load_full_robot_config() — with the fallbacks for
fields a robot XML may omit (motion limits, per-mode limits, voltage limit) kept in one place, as
CLAUDE.md asks: consumers read one resolved value.

Split out of the single-file config/xml_parser.py on 2026-10-03 (refactor Phase 4); definitions moved verbatim.
"""

from __future__ import annotations

from collections.abc import Iterable
import logging
logger = logging.getLogger("config")  # docs/logging.md "Logger names"
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from config.xml_parser.xml_access import (
    get_active_gripper, get_all_strings, get_all_values, get_child_float, get_float_value, get_gripper_value,
    get_gripper_values, get_joint_count, get_robot_name, load_gripper_element, parse_robot_config,
)
from config.xml_parser.walls import get_freedrive_walls
from config.xml_parser.homing import parse_homing_sequence


def load_software_joint_limits(xml_path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """(negative, positive) software joint limits in rad from a robot XML, NaN where absent.

    For the kinematic model (robots/PAR6.py), which needs the limits but not the rest of the
    config — so the model's qlim is read from the XML instead of being a hand-typed copy of it.
    """
    _, joints = parse_robot_config(xml_path)
    return (get_all_values(joints, 'software_joint_limit_negative'),
            get_all_values(joints, 'software_joint_limit_positive'))


def get_safety_config(root: ET.Element) -> dict:
    """Read the <safety> block from the robot XML with sensible defaults.

    The robot-SPECIFIC safety numbers. The monitor's robot-independent policy (on/off switches,
    windows, margins) is <safety_monitor> in config/system.xml. The keys utility/safety_checks.py
    run_safety_checks() read (joint-velocity/temperature/current-spike/vibration thresholds and the
    check_* flags) were removed with it on 2026-09-30: that function was never called.
    """
    safety = root.find("safety")
    return {
        # TCP translational speed limit, m/s. 0 = disabled. ONE number, two consumers:
        # the RTI handler's Cartesian velocity cap (prevention) and the safety monitor's
        # TCP_OVERSPEED check (detection) — sharing the key is what keeps the preventer and
        # the detector from ever disagreeing about the limit.
        "max_tcp_velocity": get_child_float(safety, "max_tcp_velocity", 0.0),
        # The most a streaming PC may widen the SOFT_RTI command watchdog to, in SECONDS. Hard RTI
        # has no such knob: its 40 ms can only be tightened. SOFT_RTI is position-only, and a
        # stale position target converges and holds — which is what makes any widening safe. The
        # declared rate's own 3 intervals always stay allowed. See motion/rti_handler.py
        # DEFAULT_SOFT_TIMEOUT_CEILING_S. The handler clamps it to [40 ms, 1.0 s].
        "max_soft_rti_timeout": get_child_float(safety, "max_soft_rti_timeout", 0.25),
        # How far past a SOFT limit the MEASURED position may go before the safety monitor trips
        # OUT_OF_RANGE. Smaller than the soft-to-hard margin (5 deg on most PAR6 joints).
        "joint_limit_tolerance_deg": get_child_float(safety, "joint_limit_tolerance_deg", 2.0),
        # How far past a HARD limit (an endstop) the MEASURED position may read before BEYOND_ENDSTOP
        # (homing invalidated). Generous on purpose (user decision 2026-10-01): homing differs between
        # robots and an endstop hit hard reads a few degrees past it (J3 read 2.5 deg past the XML value
        # after a hand pushed through its freedrive wall). Only a reading no real arm reaches means the
        # reference is wrong. Missing in a robot XML -> 10.
        "endstop_tolerance_deg": get_child_float(safety, "endstop_tolerance_deg", 10.0),
        # The heaviest payload set_payload accepts (utility/payload.py). Missing -> 1.0 kg, a placeholder, not a
        # rating: the robot XML should carry the real number.
        "max_payload_kg": get_child_float(safety, "max_payload_kg", 1.0),
    }


# Derivation constants for motion limits absent from a robot XML. Kept here rather than
# inline so the fallback formula has one home and matches what PAR6.xml documents.
_DERIVED_ACCEL_TIME_S = 0.5    # ramp time EXEC (ACCEL_TIME) and JOG already assume


_DERIVED_JERK_FACTOR = 50.0   # accel reaches full scale in ~20 ms; see robots/PAR6.xml joint 0 for why not ruckig's 3x (too slow) or a 500x factor (inert at a 4 ms tick)


_DERIVED_TRQ_RATE_PER_TAU = 11.5   # torque-rate-to-peak-torque ratio; a motion-smoothness limit, well below the electrical ceiling


def _fill_motion_limit_defaults(cfg: dict) -> dict:
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


def _fill_mode_limit_defaults(cfg: dict) -> dict:
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

    for mode, entries in _MODE_LIMIT_MAP.items():
        for _tag, key, ceiling_key in entries:
            ceiling = _np.asarray(cfg[ceiling_key], dtype=float)
            vals = _np.asarray(cfg.get(key, _np.full_like(ceiling, _np.nan)), dtype=float)
            vals = _np.where(_np.isnan(vals), ceiling, vals)

            over = vals > ceiling
            if over.any():
                for i in _np.where(over)[0]:
                    logger.warning(
                        "robot XML %s joint %d is %.4g, above the hardware limit %s=%.4g — "
                        "clamped. A per-mode block can lower a limit, never raise it.",
                        key, int(i), float(vals[i]), ceiling_key, float(ceiling[i]))
                vals = _np.minimum(vals, ceiling)

            cfg[key] = vals

    return cfg


# Motor/gripper voltage limit in mV, used when an XML predates the <voltage_limit> tag.
# 0 means "use VBUS" on the driver side, so it is a valid value and cannot be the missing marker.
DEFAULT_VOLTAGE_LIMIT_MV = 6000.0


# Hard floor for a REAL voltage limit. 0 is exempt — it is the driver's "use VBUS" sentinel, not
# a low limit. So the rule is: 0, or >= MIN_VOLTAGE_LIMIT_MV. Nothing in between ever reaches a
# driver. Enforced in three places so no path can get under it — XML load, live edits (RTI's
# set_pid_gains), and the CAN send itself. A GUI minimum alone would not be a guarantee: an XML
# value or a robotclient call could still ask for less.
MIN_VOLTAGE_LIMIT_MV = 6000.0


def clamp_voltage_limit(value: float | np.ndarray) -> float | np.ndarray:
    """The single definition of the voltage-limit rule, used by every path that can set one.

    0 passes through untouched — it tells the driver to use VBUS, so it is a mode, not a limit.
    Any other value (including negatives) is clamped UP to MIN_VOLTAGE_LIMIT_MV, because an
    under-limited motor cannot hold or move properly. Accepts a scalar or an array.
    """
    arr = np.asarray(value, dtype=float)
    out = np.where(arr == 0.0, 0.0, np.maximum(arr, MIN_VOLTAGE_LIMIT_MV))
    return float(out) if out.ndim == 0 else out


def _voltage_limit_or_default(value: float | np.ndarray) -> float | np.ndarray:
    """<voltage_limit> is optional — a missing tag reads as NaN (get_all_values/get_gripper_value).
    Fall back to the default so older XMLs without the tag keep loading, then apply the rule."""
    arr = np.asarray(value, dtype=float)
    filled = np.where(np.isnan(arr), DEFAULT_VOLTAGE_LIMIT_MV, arr)
    clamped = np.asarray(clamp_voltage_limit(filled), dtype=float)
    if np.any(clamped != filled):
        logger.warning("robot XML voltage_limit below the %.0f mV floor (and not the 0 = use-VBUS "
                        "sentinel) — clamped up.", MIN_VOLTAGE_LIMIT_MV)
    return float(clamped) if clamped.ndim == 0 else clamped


HOLD_OSC_DEFAULT_MA = 100.0      # api_tests/59 on PAR6 J3: buzzing ~230 mA, quiet <= 25 mA (2026-10-06)


def _hold_osc_ma(values: Iterable[float | None]) -> list[float]:
    """<hold_oscillation_ma> per joint: missing -> HOLD_OSC_DEFAULT_MA (a robot XML from before the check still gets
    it), 0 -> that joint unchecked, outside [0, 5000] -> the default with a warning."""
    out = []
    for i, v in enumerate(values):
        v = float(v) if v is not None else float("nan")
        if not math.isfinite(v):
            out.append(HOLD_OSC_DEFAULT_MA)
        elif 0.0 <= v <= 5000.0:
            out.append(v)
        else:
            logger.warning("robot XML joint %d <hold_oscillation_ma> = %s is outside [0, 5000] — using %s",
                            i, v, HOLD_OSC_DEFAULT_MA)
            out.append(HOLD_OSC_DEFAULT_MA)
    return out


def _parse_collision_pairs(elem, tag: str, frame_b: int | None = None) -> list[dict]:
    """<pair a="2" b="4" planner_mm=".." monitor_mm=".." check="off"/> entries (a gripper's <pair with="4" .../> has b = the
    tool frame 6). Frames: 0 = base, 1-5 = links after joints 1-5, 6 = the tool. Missing margins = the policy's."""
    out = []
    if elem is None:
        return out
    for i, p in enumerate(elem.findall("pair")):
        try:
            a = int(p.get("a", p.get("with")))
            b = frame_b if frame_b is not None else int(p.get("b"))
        except (TypeError, ValueError):
            logger.warning("robot XML <%s><pair> %d has no valid frames — ignored", tag, i + 1)
            continue
        if not (0 <= a <= 6 and 0 <= b <= 6) or a == b:
            logger.warning("robot XML <%s><pair a=%s b=%s> is not a pair of frames 0-6 — ignored", tag, a, b)
            continue
        def _mm(name):
            v = p.get(name)
            try:
                return None if v is None else max(0.0, float(v))
            except ValueError:
                logger.warning("robot XML <%s><pair> %s=%r is not a number — ignored", tag, name, v)
                return None
        out.append({"a": min(a, b), "b": max(a, b), "planner_mm": _mm("planner_mm"), "monitor_mm": _mm("monitor_mm"),
                    "check": (p.get("check", "on").strip().lower() not in ("off", "0", "false"))})
    return out


def _nonneg(values: Iterable[float | None], tag: str, hi: float) -> list[float]:
    """Per-joint values that must be in [0, hi]: missing (NaN) -> 0, out of range -> 0 with a warning."""
    out = []
    for i, v in enumerate(values):
        v = float(v) if v is not None else float("nan")
        if not math.isfinite(v):
            out.append(0.0)
        elif 0.0 <= v <= hi:
            out.append(v)
        else:
            logger.warning("robot XML joint %d <%s> = %s is outside [0, %s] — using 0", i, tag, v, hi)
            out.append(0.0)
    return out


def load_full_robot_config(xml_path: str | Path, grippers_dir: str | Path | None = None,
                           active_gripper: str | None = None) -> dict:
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
        # (no 'motor_watchdog_rate' since 2026-10-13: every drive's watchdog is system.xml <drive_watchdog_ms>)
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
        'calibration_screw_rad':          get_all_values(joints, 'calibration_screw_rad'),  # NaN = no screw
        'home_offset_gripper_dependent':  get_all_values(joints, 'home_offset_gripper_dependent'),
        'homing_speed':           get_all_values(joints, 'homing_speed'),
        'homing_direction':       get_all_values(joints, 'homing_direction'),
        'homing_current_ma':      get_all_values(joints, 'homing_current_ma'),
        'homing_timeout_s':       get_all_values(joints, 'homing_timeout_s'),
        'homing_backoff_s':       get_all_values(joints, 'homing_backoff_s'),
        'has_hall_sensor':        get_all_values(joints, 'has_hall_sensor'),
        # Safety monitor TRACKING limit per joint, deg. Missing / 0 = that joint unchecked.
        'tracking_limit_deg':     get_all_values(joints, 'tracking_limit_deg'),
        # Safety monitor HOLD_OSCILLATION_WARNING threshold per joint, mA (fast motor-current ripple while
        # commanded still). Missing -> HOLD_OSC_DEFAULT_MA, 0 = that joint unchecked.
        'hold_oscillation_ma':    _hold_osc_ma(get_all_values(joints, 'hold_oscillation_ma')),
        'freedrive_walls':        get_freedrive_walls(joints),
        # Friction compensation (utility/friction_comp.py): MEASURED per joint by api_tests/54 — the breakaway
        # friction and the largest gravity-model error, Nm. Missing / 0 = not measured -> no compensation.
        'friction_static_nm':     _nonneg(get_all_values(joints, 'friction_static_nm'), 'friction_static_nm', 1.0),
        'friction_sliding_nm':    _nonneg(get_all_values(joints, 'friction_sliding_nm'), 'friction_sliding_nm', 1.0),
        'gravity_error_nm':       _nonneg(get_all_values(joints, 'gravity_error_nm'), 'gravity_error_nm', 1.0),
        'homing_release_current_ma': get_all_values(joints, 'homing_release_current_ma'),
        'homing_release_duration_s': get_all_values(joints, 'homing_release_duration_s'),
        'homing_release_sample_pct': get_all_values(joints, 'homing_release_sample_pct'),
        'homing_two_pass_max_diff':  get_all_values(joints, 'homing_two_pass_max_diff'),

        # ---- gripper scalars ----
        # Self-collision geometry (TODO/self_collision_plan.md): the arm's convex pieces file + per-pair overrides in the
        # robot XML <self_collision file="..">; the tool's in the gripper XML <collision file=".."> (pairs: <pair with="4"/>).
        'self_collision':         {"file": (root.find("self_collision").get("file") if root.find("self_collision") is not None else None),
                                   "pairs": _parse_collision_pairs(root.find("self_collision"), "self_collision")},
        'gripper_collision':      {"file": (gripper_elem.find("collision").get("file") if gripper_elem is not None and gripper_elem.find("collision") is not None else None),
                                   "pairs": _parse_collision_pairs(gripper_elem.find("collision") if gripper_elem is not None else None, "collision", frame_b=6)},
        'gripper_name':           gripper_name,
        'gripper_driver_type':    _gstr('driver_type'),
        'gripper_stroke':         _gval('Stroke'),
        'gripper_kt':             _gval('kt'),
        'gripper_gear_r':         _gval('Gear_r'),
        'gripper_Ilim':           _gval('Ilim'),
        'gripper_voltage_limit':  _voltage_limit_or_default(_gval('voltage_limit')),
        'gripper_velocity_limit': _gval('Velocity_limit'),
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
        'gripper_homing_calibration_wait_s': _gval('homing_calibration_wait_s'),
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
            logger.warning("robot XML <park_pose> is not a list of numbers — ignored")
        else:
            if len(values) == cfg['joint_num']:
                cfg['park_pose'] = values
            else:
                logger.warning(
                    "robot XML <park_pose> has %d values but the robot has %d "
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
