"""Every value RTI's trace carries, by its telemetry tag (2026-11-26, TODO/p4_recording_plan.md section 2B).

RTI's trace (utility/trace_ring.py) is one row of numbers per tick in shared memory. Since 2026-11-26 it carries every
value a telemetry recipe can name, so a packet can be copied out of the row whatever the recipe (the binary format)
and a recording keeps everything. Each channel is named by its telemetry tag (data/robotics_data.TAGS) and read from
an array that keeps its identity for the whole run: a shared-memory view, or RTI's motor error buffer.

Two kinds:

  * FAST: copied into every row (`TraceWriter`'s main copy). The values that can change from one tick to the next.
  * SLOW: copied into a block of the row that `TraceWriter` refreshes one fifth at a time, one fifth per tick, so
    they cost the tick about a fifth of their copy. The values that change seldom or arrive in turns: the drives'
    temperatures, voltages and error flags (each drive answers one of them every ~80 ms; `update_motor_and_gripper_
    data` copies them every 5 ticks), the gains loaded into the drives, the control board's readings. A slow value
    in a row can be up to 4 ticks older than the row.

The first 35 fast channels are the trace's channels from before 2026-11-26, in their order. Order is otherwise free:
readers find a channel by its name in the trace's layout.
"""
from __future__ import annotations

# (tag, shared_data group, key) — the group names a dict of data/shared_data.py; "error_bytes" is RTI's own
# motor_error_bytes buffer (no key).
FAST = (
    # ---- the trace before 2026-11-26 (same names, same order) ----
    ("j_p", "joint", "joint_position"), ("j_s", "joint", "joint_speed"),
    ("j_t", "joint", "joint_torque"), ("j_a", "joint", "joint_acceleration"),
    ("j_j", "joint", "joint_jerk"), ("j_s_f", "joint", "joint_speed_filtered"),
    ("j_t_f", "joint", "joint_torque_filtered"), ("j_a_f", "joint", "joint_acceleration_filtered"),
    ("j_j_f", "joint", "joint_jerk_filtered"), ("TCP_p", "tcp", "robot_pose"),
    ("TCP_p_SE3", "tcp", "robot_pose_matrix"), ("TCP_p_quat", "tcp", "robot_pose_quaternion"),
    ("TCP_s", "tcp", "TCP_speed"), ("TCP_s_f", "tcp", "TCP_speed_filtered"),
    ("TCP_a", "tcp", "TCP_acceleration_raw"), ("TCP_a_f", "tcp", "TCP_acceleration_filtered"),
    ("t_ext", "tcp", "torque_external"), ("TCP_f_ext", "tcp", "force_external"),
    ("TCP_f_ext_tool", "tcp", "force_external_tool"),
    ("j_pc", "joint", "joint_position_commanded"), ("j_sc", "joint", "joint_speed_commanded"),
    ("j_ac", "joint", "joint_acceleration_commanded"), ("j_tc", "joint", "joint_torque_commanded"),
    ("TCP_p_c", "tcp", "TCP_pose_commanded"), ("TCP_s_c", "tcp", "TCP_speed_commanded"),
    ("j_pt", "joint", "joint_position_target"), ("j_st", "joint", "joint_speed_target"),
    ("j_at", "joint", "joint_acceleration_target"), ("j_tt", "joint", "joint_torque_target"),
    ("j_t_inertia", "joint", "joint_inertia_torque"), ("j_t_coriolis", "joint", "joint_coriolis_torque"),
    ("TCP_p_t", "tcp", "TCP_pose_target"), ("TCP_s_t", "tcp", "TCP_speed_target"),
    ("j_err", "error_bytes", None), ("j_rx", "motor", "motor_reply_time"),
    # ---- added 2026-11-26 ----
    ("j_t_grav", "joint", "joint_gravity_torque"), ("j_t_fric", "joint", "joint_friction_comp_torque"),
    ("m_p", "motor", "motor_position"), ("m_s", "motor", "motor_speed"), ("m_c", "motor", "motor_current"),
    ("m_pc", "motor", "motor_position_sent"), ("m_sc", "motor", "motor_speed_sent"),
    ("m_cc", "motor", "motor_current_sent"),
    ("gm_p", "gripper", "gripper_motor_position"), ("gm_s", "gripper", "gripper_motor_speed"),
    ("gm_c", "gripper", "gripper_motor_current"),
    ("g_p", "gripper", "gripper_position"), ("g_c", "gripper", "gripper_current"),
    ("g_activated", "gripper", "gripper_activated"), ("g_action", "gripper", "gripper_action"),
    ("g_obj", "gripper", "gripper_object_detection"), ("g_err", "gripper", "gripper_error"),
    ("g_jp", "gripper", "gripper_joint_position"), ("g_js", "gripper", "gripper_joint_speed"),
    ("g_js_f", "gripper", "gripper_joint_speed_filtered"), ("g_jf", "gripper", "gripper_joint_force"),
    ("g_jpt", "gripper", "gripper_joint_position_target"), ("g_jst", "gripper", "gripper_joint_speed_target"),
    ("g_jft", "gripper", "gripper_joint_force_target"),
    ("g_jpc", "gripper", "gripper_joint_position_commanded"), ("g_jsc", "gripper", "gripper_joint_speed_commanded"),
    ("g_jfc", "gripper", "gripper_joint_force_commanded"),
    ("g_pt", "gripper", "gripper_position_target"), ("g_st", "gripper", "gripper_speed_target"),
    ("g_ct", "gripper", "gripper_current_target"), ("g_at", "gripper", "gripper_action_target"),
    ("g_rx", "gripper", "gripper_reply_time"),
    ("io", "gpio", None),
)

SLOW = (
    ("m_temp", "motor", "motor_temperature"), ("m_volt", "motor", "motor_voltage"),
    ("m_t_err", "motor", "motor_temperature_error"), ("m_enc_err", "motor", "motor_encoder_error"),
    ("m_vbus_err", "motor", "motor_vbus_error"), ("m_d_err", "motor", "motor_driver_error"),
    ("m_vel_err", "motor", "motor_velocity_error"), ("m_c_err", "motor", "motor_current_error"),
    ("m_estop_err", "motor", "motor_estop_error"), ("m_w_err", "motor", "motor_watchdog_error"),
    ("m_cal", "motor", "motor_calibrated"),
    ("gm_temp", "gripper", "gripper_motor_temperature"), ("gm_volt", "gripper", "gripper_motor_voltage"),
    ("gm_t_err", "gripper", "gripper_motor_temp_error"), ("gm_enc_err", "gripper", "gripper_motor_enc_error"),
    ("gm_vbus_err", "gripper", "gripper_motor_vbus_error"), ("gm_d_err", "gripper", "gripper_motor_driver_error"),
    ("gm_vel_err", "gripper", "gripper_motor_velocity_error"),
    ("gm_c_err", "gripper", "gripper_motor_current_error"),
    ("gm_estop_err", "gripper", "gripper_motor_estop_error"),
    ("gm_w_err", "gripper", "gripper_motor_watchdog_error"), ("gm_cal", "gripper", "gripper_motor_calibrated"),
    ("g_cal", "gripper", "gripper_calibrated"), ("g_temp_err", "gripper", "gripper_temperature_error"),
    ("g_tim_err", "gripper", "gripper_timeout_error"), ("g_estop_err", "gripper", "gripper_estop_error"),
    ("m_KPP", "motor_config", "cfg_KPP"), ("m_KPV", "motor_config", "cfg_KPV"),
    ("m_KIV", "motor_config", "cfg_KIV"), ("m_KPIQ", "motor_config", "cfg_KPIQ"),
    ("m_KIIQ", "motor_config", "cfg_KIIQ"), ("m_KP", "motor_config", "cfg_KP"),
    ("m_KD", "motor_config", "cfg_KD"), ("m_Ilim", "motor_config", "cfg_Ilim"),
    ("m_vel_lim", "motor_config", "cfg_velocity_limit"),
    ("g_KPP", "motor_config", "cfg_gripper_KPP"), ("g_KPV", "motor_config", "cfg_gripper_KPV"),
    ("g_KIV", "motor_config", "cfg_gripper_KIV"), ("g_KPIQ", "motor_config", "cfg_gripper_KPIQ"),
    ("g_KIIQ", "motor_config", "cfg_gripper_KIIQ"), ("g_KP", "motor_config", "cfg_gripper_KP"),
    ("g_KD", "motor_config", "cfg_gripper_KD"), ("g_Ilim", "motor_config", "cfg_gripper_Ilim"),
    ("g_vel_lim", "motor_config", "cfg_gripper_velocity_limit"),
    ("cb_vbus1", "control_board", "cb_filtered_VBUS1_mV"), ("cb_vbus2", "control_board", "cb_filtered_VBUS2_mV"),
    ("cb_current", "control_board", "cb_filtered_Current_mA"), ("cb_relay", "control_board", "cb_relayState"),
    ("cb_precharge", "control_board", "cb_PrechargeState"),
    ("cb_brake_res", "control_board", "cb_BrakeResistorState"), ("cb_err", "control_board", "cb_error_state"),
    ("cb_power", "control_board", "cb_power_usage"),
)

TAGS = tuple(t for t, _g, _k in FAST + SLOW)


def _array(shared_data, group, key, error_bytes):
    """One channel's array: a shared-memory view (or RTI's error-byte buffer)."""
    if group == "error_bytes":
        return error_bytes
    if group == "gpio":
        return shared_data.gpio_states
    return getattr(shared_data, group)[key]


def trace_sources(shared_data, error_bytes) -> tuple[list, list]:
    """(fast, slow): [(tag, array)] for `TraceWriter(fast, slow=slow)`. `shared_data` set up (its dicts filled);
    `error_bytes` = RTI's motor_error_bytes. Raises KeyError naming the channel when a shared array is missing."""
    out = []
    for table in (FAST, SLOW):
        part = []
        for tag, group, key in table:
            try:
                part.append((tag, _array(shared_data, group, key, error_bytes)))
            except (KeyError, AttributeError) as e:
                raise KeyError(f"trace channel {tag}: shared_data.{group}[{key!r}] is missing") from e
        out.append(part)
    return out[0], out[1]
