"""Building a HomingController from the dict load_full_robot_config() returns (HomingController.from_config)."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

import logging
logger = logging.getLogger("rti.homing")  # docs/logging.md "Logger names"
import math
from typing import TYPE_CHECKING

import numpy as np

from robotics.homing.gripper import GripperHomingState

if TYPE_CHECKING:                                   # the return annotation only
    from robotics.homing.controller import HomingController  # noqa: F401


# ------------------------------------------------------------------
def from_config(cls, xml_config: dict, joint_num: int) -> 'HomingController':
    """
    Build from the dict returned by load_full_robot_config().
    NaN values (missing XML fields) fall back to safe defaults.

    All timeout/backoff values are read in **seconds** from XML fields:
      homing_timeout_s  (per-joint or global scalar, default 6.0 s)
      homing_backoff_s  (global scalar, default 0.3 s)

    If cfg contains 'gripper_CAN_gripper' == 1 and 'gripper_homing_speed'
    is set, the gripper is appended as an extra actuator at index joint_num.
    Its order in the homing sequence is controlled by 'gripper_homing_order'.
    """
    dt = float(xml_config.get('delta_t', 0.001))

    def _fill(key, default):
        arr = xml_config.get(key, np.full(joint_num, np.nan))
        return np.where(np.isnan(arr), default, np.asarray(arr, dtype=float))

    # --- Arm-joint arrays ---
    home_offsets      = _fill('home_offset',       math.pi / 2)
    homing_speeds     = _fill('homing_speed',      5000.0)
    homing_directions = _fill('homing_direction',  1.0)
    homing_current_ma = _fill('homing_current_ma', 600.0)
    has_hall_sensor   = _fill('has_hall_sensor',   0.0).astype(int)
    two_pass          = _fill('homing_two_pass',   1.0).astype(bool)
    timeout_s         = _fill('homing_timeout_s', 6.0)
    backoff_s         = _fill('homing_backoff_s', 0.3)
    rehome_factor     = float(xml_config.get('homing_rehome_factor', 0.3))

    # Release phase parameters (NaN = skip for that joint)
    release_current   = _fill('homing_release_current_ma', np.nan)
    release_duration  = _fill('homing_release_duration_s',  np.nan)
    release_sample    = _fill('homing_release_sample_pct',  np.nan)

    # Two-pass max difference (NaN = use default 200 ticks)
    two_pass_max_diff = _fill('homing_two_pass_max_diff', np.nan)

    # Post-home positions / speeds for arm joints
    post_pos_raw = xml_config.get('post_home_position', None)
    post_pos = (_fill('post_home_position', np.nan)
                if post_pos_raw is not None
                else np.full(joint_num, np.nan))
    post_spd_raw = xml_config.get('post_home_speed', None)
    if post_spd_raw is not None:
        raw_spd  = _fill('homing_speed', 5000.0)
        post_spd = np.where(np.isnan(_fill('post_home_speed', np.nan)),
                            raw_spd, _fill('post_home_speed', np.nan))
    else:
        post_spd = None

    # Arm-joint homing order: 1-based per-joint field → order_arr for sorting
    order_raw = xml_config.get('homing_order', None)
    if order_raw is not None:
        order_arr = np.asarray(order_raw, dtype=float)
        order_arr = np.where(np.isnan(order_arr),
                             1e9 + np.arange(joint_num, dtype=float),
                             order_arr)
    else:
        order_arr = 1e9 + np.arange(joint_num, dtype=float)

    # --- Optional CAN-gripper homing ---
    def _gget(key, default):
        v = xml_config.get(f'gripper_{key}', np.nan)
        if isinstance(v, float) and math.isnan(v):
            return float(default)
        try:
            return float(v)
        except (TypeError, ValueError):
            return float(default)

    can_gripper    = int(_gget('CAN_gripper', 0))
    gripper_speed  = xml_config.get('gripper_homing_speed', np.nan)
    do_gripper     = (can_gripper == 1
                      and not (isinstance(gripper_speed, float)
                               and math.isnan(gripper_speed)))

    if do_gripper:
        total = joint_num + 1

        home_offsets      = np.append(home_offsets,      _gget('home_offset',      math.pi / 2))
        homing_speeds     = np.append(homing_speeds,     _gget('homing_speed',     5000.0))
        homing_directions = np.append(homing_directions, _gget('homing_direction', 1.0))
        homing_current_ma = np.append(homing_current_ma, _gget('homing_current_ma', 600.0))
        has_hall_sensor   = np.append(has_hall_sensor,   int(_gget('has_hall_sensor', 0)))
        two_pass          = np.append(two_pass,          bool(_gget('homing_two_pass', 1.0)))
        backoff_s         = np.append(backoff_s,         _gget('homing_backoff_s', 0.3))
        g_timeout         = _gget('homing_timeout_s',    6.0)
        timeout_s         = np.append(timeout_s,         g_timeout)
        post_pos          = np.append(post_pos,          np.nan)
        if post_spd is None:
            post_spd = np.append(homing_speeds[:joint_num],
                                 _gget('homing_speed', 5000.0))
        else:
            post_spd = np.append(post_spd, _gget('homing_speed', 5000.0))

        # Gripper doesn't use release phase
        release_current  = np.append(release_current,  np.nan)
        release_duration = np.append(release_duration, np.nan)
        release_sample   = np.append(release_sample,   np.nan)

        two_pass_max_diff = np.append(two_pass_max_diff, _gget('homing_two_pass_max_diff', np.nan))

        gripper_normal_ilim           = _gget('Ilim',           1000.0)
        gripper_normal_velocity_limit = _gget('Velocity_limit', 80000.0)

        g_order_raw = xml_config.get('gripper_homing_order', np.nan)
        g_order_val = (1e9 + joint_num
                       if (isinstance(g_order_raw, float) and math.isnan(g_order_raw))
                       else float(g_order_raw))
        order_arr = np.append(order_arr, g_order_val)
    else:
        total = joint_num

    home_seq = list(np.argsort(order_arr, kind='stable').astype(int))

    # Check for parsed homing sequence from XML file
    homing_steps = xml_config.get('homing_sequence', None)

    obj = cls(
        total,
        home_offsets        = home_offsets,
        homing_speeds       = homing_speeds,
        homing_directions   = homing_directions,
        homing_current_ma   = homing_current_ma,
        has_hall_sensor     = has_hall_sensor,
        dt                  = dt,
        timeout_s           = timeout_s,
        two_pass_homing     = two_pass,
        backoff_s           = backoff_s,
        rehome_speed_factor = rehome_factor,
        home_sequence       = home_seq,
        post_home_positions = post_pos,
        post_home_speeds    = post_spd,
        arm_count           = joint_num,
        homing_steps        = homing_steps,
        release_current_ma  = release_current,
        release_duration_s  = release_duration,
        release_sample_pct  = release_sample,
        two_pass_max_diff   = two_pass_max_diff,
    )
    if do_gripper:
        obj._gripper_normal_ilim           = gripper_normal_ilim
        obj._gripper_normal_velocity_limit = gripper_normal_velocity_limit
        obj._gripper_stroke_mm             = float(_gget('stroke', 0.0))
        obj._gripper_gear_r                = float(xml_config.get('gripper_gear_r', 0.0) or 0.0)
        obj._gripper_calibration_wait_s    = _gget('homing_calibration_wait_s', GripperHomingState._MIN_WAIT_S)
        if obj._gripper_calibration_wait_s >= g_timeout:
            logger.warning(
                f"gripper homing_calibration_wait_s ({obj._gripper_calibration_wait_s} s) is not below its "
                f"homing_timeout_s ({g_timeout} s) — its firmware calibration step can only time out"
            )
    return obj
