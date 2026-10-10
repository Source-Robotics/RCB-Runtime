"""Registry of all available telemetry tags.

Each entry maps a short tag name to:
  src  — which deque holds this field: "joint" | "gripper"
  prec — default decimal precision (0 = integer)
  desc — human-readable description

These tags are the only valid values in a data recipe XML <field tag="..."/>.
"""

# ================================================================
# Joint / TCP tags  (sourced from Joint_data deque)
# ================================================================
TAGS = {
    "j_p":       {"src": "joint",   "prec": 4, "desc": "joint positions (rad)"},
    "j_s":       {"src": "joint",   "prec": 3, "desc": "joint speeds (rad/s) — RAW"},
    "j_t":       {"src": "joint",   "prec": 3, "desc": "joint torques (Nm) — RAW"},
    "j_a":       {"src": "joint",   "prec": 3, "desc": "joint accelerations (rad/s²) — RAW"},
    "j_j":       {"src": "joint",   "prec": 3, "desc": "joint jerks (rad/s³) — RAW"},
    # Filtered twins (suffix _f). Filter per signal is set in config/filters.xml.
    "j_s_f":     {"src": "joint",   "prec": 3, "desc": "joint speeds, filtered (rad/s)"},
    "j_t_f":     {"src": "joint",   "prec": 3, "desc": "joint torques, filtered (Nm)"},
    "j_a_f":     {"src": "joint",   "prec": 3, "desc": "joint accelerations, filtered (rad/s²)"},
    "j_j_f":     {"src": "joint",   "prec": 3, "desc": "joint jerks, filtered (rad/s³)"},
    "j_pc":      {"src": "joint",   "prec": 4, "desc": "commanded joint positions (rad)"},
    "j_sc":      {"src": "joint",   "prec": 3, "desc": "commanded joint speeds (rad/s)"},
    "j_ac":      {"src": "joint",   "prec": 3, "desc": "commanded joint accelerations (rad/s²)"},
    "j_tc":      {"src": "joint",   "prec": 3, "desc": "commanded joint torques (Nm)"},
    "j_pt":      {"src": "joint",   "prec": 4, "desc": "target joint positions (rad)"},
    "j_st":      {"src": "joint",   "prec": 3, "desc": "target joint speeds (rad/s)"},
    "j_at":      {"src": "joint",   "prec": 3, "desc": "target joint accelerations (rad/s²)"},
    "j_tt":      {"src": "joint",   "prec": 3, "desc": "target joint torques (Nm)"},
    "j_err":     {"src": "joint",   "prec": 0, "desc": "joint error flags"},
    "TCP_p":     {"src": "joint",   "prec": 4, "desc": "TCP pose [x,y,z,rx,ry,rz] (m, rad)"},
    # The same measured pose in its other two forms. Pick by need:
    #   TCP_p       — human-readable, but its RPY has GIMBAL LOCK at pitch = +/-90 deg
    #   TCP_p_SE3   — full 4x4, no singularity, 16 values (the widest on the wire)
    #   TCP_p_quat  — orientation only, no singularity, 4 values. Use for orientation
    #                 error / SLERP. (w, x, y, z) — SCALAR FIRST, w >= 0. Many libraries want
    #                 (x, y, z, w): reorder at the consumer with q[[1, 2, 3, 0]].
    "TCP_p_SE3":  {"src": "joint",   "prec": 6, "desc": "TCP pose as flattened 4x4 SE(3) matrix, row-major (16 values)"},
    "TCP_p_quat": {"src": "joint",   "prec": 6, "desc": "TCP orientation as unit quaternion [w,x,y,z] — SCALAR FIRST, w>=0 (4 values)"},
    "TCP_s":     {"src": "joint",   "prec": 3, "desc": "TCP speed (m/s, rad/s) — RAW, world frame"},
    "TCP_s_f":   {"src": "joint",   "prec": 3, "desc": "TCP speed, filtered (m/s, rad/s) — world frame"},
    "TCP_a":     {"src": "joint",   "prec": 3, "desc": "TCP acceleration (m/s², rad/s²) — RAW, world frame"},
    "TCP_a_f":   {"src": "joint",   "prec": 3, "desc": "TCP acceleration, filtered — world frame"},
    "TCP_p_c":   {"src": "joint",   "prec": 4, "desc": "commanded TCP pose"},
    "TCP_s_c":   {"src": "joint",   "prec": 3, "desc": "commanded TCP speed"},
    "TCP_p_t":   {"src": "joint",   "prec": 4, "desc": "target TCP pose"},
    "TCP_s_t":   {"src": "joint",   "prec": 3, "desc": "target TCP speed"},
    "t_ext":     {"src": "joint",   "prec": 3, "desc": "external joint torques (Nm)"},
    "TCP_f_ext": {"src": "joint",   "prec": 3, "desc": "external TCP wrench (N, Nm) — WORLD frame"},
    "TCP_f_ext_tool": {"src": "joint", "prec": 3, "desc": "external TCP wrench (N, Nm) — TOOL frame"},
    "timestamp": {"src": "joint",   "prec": 6, "desc": "loop timestamp (s)"},
    "exec_t":    {"src": "joint",   "prec": 6, "desc": "loop execution time (s)"},
    "tick":      {"src": "joint",   "prec": 0, "desc": "RTI loop tick counter"},

    # --- RTI-mode link diagnostics (TODO/RTI_mode_plan.md §18) ---------------
    # Only meaningful while RobotMode.RTI is active; 0 otherwise. Opt in per recipe —
    # each costs ~1.1 us of serialisation and ~22 bytes/packet (measured), so leave them
    # out of `minimal` and put them where you actually want link visibility.
    #
    # rti_seq is the ECHO that makes round-trip timing possible at all: it is the last
    # command sequence number the robot accepted, so a PC that timestamps its sends can
    # match the echo back and measure true loop delay. Without it the PC has no way to
    # tell which state packet reflects which command, and no clock-sync-free alternative
    # exists. It doubles as per-command loss detection: a seq that never comes back was
    # never applied.
    "rti_seq":   {"src": "joint",   "prec": 0, "desc": "last RTI command seq the robot accepted (echo)"},
    # These two exist ONLY in shared memory otherwise, which a remote PC cannot read —
    # so without them a network session can only see its own end of the link.
    "rti_ok":    {"src": "joint",   "prec": 3, "desc": "RTI control_command_success_rate (0-1)"},
    "rti_miss":  {"src": "joint",   "prec": 0, "desc": "RTI consecutive ticks with no command"},
    # The RTI session sub-state, same code the GUI shows: 0 unpaired, 1 connected,
    # 2 control_active, 3 stopping_clean, 4 stopping_error. WITHOUT THIS the PC cannot
    # tell that the robot revoked control — state packets keep flowing unchanged and a
    # PC whose claim was dropped goes on streaming into an open loop it believes is
    # closed. The equivalent elsewhere is a control-mode field leaving its 'moving' state.
    "rti_state": {"src": "joint",   "prec": 0, "desc": "RTI sub-state (0 unpaired..4 stopping_error)"},
    # The GLOBAL robot mode as a string (IDLE / RTI / EXEC / ACTIVE_ERROR / ...). Distinct
    # from rti_state: that one answers "do I still have control", this answers "what is the
    # robot even doing". An E-stop or an operator pressing IDLE moves this, not rti_state.
    # This is the coarse, whole-robot state; rti_state is the fine, claim-scoped one.
    "mode":      {"src": "joint",   "prec": 0, "desc": "global RobotMode name"},
    # MEASURED wall-clock duration of the tick that produced this packet, in ms — not the
    # configured RTI_loop_time. A controller integrating velocity needs the real elapsed
    # time; using a hardcoded nominal is wrong on any tick that ran long, and wrong by 2.5x
    # on a 100 Hz robot. The measured period belongs in every control callback for
    # exactly this reason.
    "period_ms": {"src": "joint",   "prec": 3, "desc": "measured loop period of this tick (ms)"},

    # ================================================================
    # Motor tags  (raw per-node deques, arm joints 0..joint_num-1 only)
    # Sourced from Motor_data / Motor_temperature_data / Motor_voltage_data.
    # ================================================================
    "m_p":         {"src": "motor",       "prec": 0, "desc": "raw motor positions (encoder ticks)"},
    "m_s":         {"src": "motor",       "prec": 0, "desc": "raw motor speeds (encoder ticks/s)"},
    "m_c":         {"src": "motor",       "prec": 0, "desc": "raw motor currents (mA)"},
    "m_temp":      {"src": "motor",       "prec": 1, "desc": "motor temperatures (°C)"},
    "m_volt":      {"src": "motor",       "prec": 0, "desc": "motor voltages (raw int from CAN)"},
    "m_KPP":       {"src": "motor_config", "prec": 3, "desc": "motor position-loop P gain (per joint)"},
    "m_KPV":       {"src": "motor_config", "prec": 3, "desc": "motor velocity-loop P gain"},
    "m_KIV":       {"src": "motor_config", "prec": 3, "desc": "motor velocity-loop I gain"},
    "m_KPIQ":      {"src": "motor_config", "prec": 3, "desc": "motor current-loop P gain (Iq)"},
    "m_KIIQ":      {"src": "motor_config", "prec": 3, "desc": "motor current-loop I gain (Iq)"},
    "m_KP":        {"src": "motor_config", "prec": 3, "desc": "motor PD-mode P gain"},
    "m_KD":        {"src": "motor_config", "prec": 3, "desc": "motor PD-mode D gain"},
    "m_Ilim":      {"src": "motor_config", "prec": 0, "desc": "motor current limit (mA)"},
    "m_vel_lim":   {"src": "motor_config", "prec": 0, "desc": "motor velocity limit (raw int)"},
    "m_t_err":     {"src": "motor",       "prec": 0, "desc": "motor temperature error flag"},
    "m_enc_err":   {"src": "motor",       "prec": 0, "desc": "motor encoder error flag"},
    "m_vbus_err":  {"src": "motor",       "prec": 0, "desc": "motor VBUS (voltage rail) error flag"},
    "m_d_err":     {"src": "motor",       "prec": 0, "desc": "motor driver error flag"},
    "m_vel_err":   {"src": "motor",       "prec": 0, "desc": "motor velocity error flag"},
    "m_c_err":     {"src": "motor",       "prec": 0, "desc": "motor current error flag"},
    "m_estop_err": {"src": "motor",       "prec": 0, "desc": "motor estop error flag"},
    "m_w_err":     {"src": "motor",       "prec": 0, "desc": "motor watchdog error flag"},
    "m_cal":       {"src": "motor",       "prec": 0, "desc": "motor calibrated flag"},

    # ================================================================
    # Gripper-motor tags  (raw values for the gripper CAN node only)
    # Same deques as motor tags above, indexed at joint_num.
    # Distinct from g_* gripper tags which read the decoded Gripper_data deque.
    # ================================================================
    "gm_p":         {"src": "gripper_motor", "prec": 0, "desc": "raw gripper motor position (encoder ticks, motor-mode only)"},
    "gm_s":         {"src": "gripper_motor", "prec": 0, "desc": "raw gripper motor speed (encoder ticks/s, motor-mode only)"},
    "gm_c":         {"src": "gripper_motor", "prec": 0, "desc": "raw gripper motor current (mA)"},
    "gm_temp":      {"src": "gripper_motor", "prec": 1, "desc": "gripper motor temperature (°C)"},
    "gm_volt":      {"src": "gripper_motor", "prec": 0, "desc": "gripper motor voltage (raw int from CAN)"},
    "g_KPP":        {"src": "gripper_config", "prec": 3, "desc": "gripper position-loop P gain"},
    "g_KPV":        {"src": "gripper_config", "prec": 3, "desc": "gripper velocity-loop P gain"},
    "g_KIV":        {"src": "gripper_config", "prec": 3, "desc": "gripper velocity-loop I gain"},
    "g_KPIQ":       {"src": "gripper_config", "prec": 3, "desc": "gripper current-loop P gain (Iq)"},
    "g_KIIQ":       {"src": "gripper_config", "prec": 3, "desc": "gripper current-loop I gain (Iq)"},
    "g_KP":         {"src": "gripper_config", "prec": 3, "desc": "gripper PD-mode P gain"},
    "g_KD":         {"src": "gripper_config", "prec": 3, "desc": "gripper PD-mode D gain"},
    "g_Ilim":       {"src": "gripper_config", "prec": 0, "desc": "gripper current limit (mA)"},
    "g_vel_lim":    {"src": "gripper_config", "prec": 0, "desc": "gripper velocity limit (raw int)"},
    "gm_t_err":     {"src": "gripper_motor", "prec": 0, "desc": "gripper motor temperature error flag"},
    "gm_enc_err":   {"src": "gripper_motor", "prec": 0, "desc": "gripper motor encoder error flag"},
    "gm_vbus_err":  {"src": "gripper_motor", "prec": 0, "desc": "gripper motor VBUS (voltage rail) error flag"},
    "gm_d_err":     {"src": "gripper_motor", "prec": 0, "desc": "gripper motor driver error flag"},
    "gm_vel_err":   {"src": "gripper_motor", "prec": 0, "desc": "gripper motor velocity error flag"},
    "gm_c_err":     {"src": "gripper_motor", "prec": 0, "desc": "gripper motor current error flag"},
    "gm_estop_err": {"src": "gripper_motor", "prec": 0, "desc": "gripper motor estop error flag"},
    "gm_w_err":     {"src": "gripper_motor", "prec": 0, "desc": "gripper motor watchdog error flag"},
    "gm_cal":       {"src": "gripper_motor", "prec": 0, "desc": "gripper motor calibrated flag"},

    # ================================================================
    # Gripper tags  (sourced from Gripper_data deque)
    # ================================================================
    "g_p":          {"src": "gripper", "prec": 3, "desc": "gripper position (m)"},
    "g_c":          {"src": "gripper", "prec": 2, "desc": "gripper current (mA)"},
    "g_activated":  {"src": "gripper", "prec": 0, "desc": "gripper activated flag"},
    "g_action":     {"src": "gripper", "prec": 0, "desc": "gripper action status"},
    "g_obj":        {"src": "gripper", "prec": 0, "desc": "gripper object detection"},
    "g_err":        {"src": "gripper", "prec": 0, "desc": "gripper error flag"},
    "g_jp":         {"src": "gripper", "prec": 4, "desc": "gripper joint position (m)"},
    "g_js":         {"src": "gripper", "prec": 3, "desc": "gripper joint speed (m/s)"},
    "g_jf":         {"src": "gripper", "prec": 3, "desc": "gripper joint force (N)"},
}

