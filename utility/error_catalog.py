"""The cause / fix text for every RTI error key — a LEAF module (imports nothing of ours).

Moved out of utility/error_checks.py on 2026-10-03 (refactor Phase 2) to break the only import cycle in the tree:
error_checks imported activity_log at module level, and activity_log needed this dict from error_checks and so
imported it inside a function. Both now import it from here; error_checks re-exports it under its old name.
"""

# ---------------------------------------------------------------------------
# Cause/fix descriptions for every possible error key, also the RTI-hardware
# slice of the activity-log catalog (see utility/activity_log.py). The key is
# what goes into the set; "cause" is what gets printed by check_errors()/
# process_errors_tick(), "fix" is only surfaced in the activity log.
# ---------------------------------------------------------------------------
ERROR_DESCRIPTIONS = {
    # E-stop
    "ESTOP":                    {"cause": "E-STOP is active — robot is halted.", "fix": "release the E-STOP, then clear errors to resume."},
    "SW_ESTOP":                 {"cause": "Software E-STOP is active — robot is halted.", "fix": "clear the software E-STOP, then clear errors to resume."},
    "EXEC_LINK_LOST":           {"cause": "command_executor went silent during EXEC motion — the control link is lost.", "fix": "restart command_executor, then clear errors to resume."},
    "RTI_LINK_LOST":            {"cause": "the external PC stopped sending setpoints while it held RTI control — the robot stopped itself.", "fix": "check the network and the controlling PC, then clear errors and re-claim control."},
    # The RTI handler's strict command checks (2026-10-11) — the PC's stream asked for motion the arm cannot do; the
    # robot braked to rest within its limits, then latched the key (per joint: J{i}:...)
    "RTI_POSITION_LIMIT":       {"cause": "the PC's RTI target would move the joint past its software limit (or toward it faster than it can still stop before it) — the robot braked to rest and refused the stream.", "fix": "keep the PC's targets inside the joint limits and decelerate before them; clear errors and re-claim control."},
    "RTI_VELOCITY_LIMIT":       {"cause": "the PC's hard-RTI stream moved the joint faster than its RTI velocity limit (+10 %) — the robot braked to rest and refused the stream.", "fix": "slow the PC's trajectory down to the joint's velocity limit; clear errors and re-claim control."},
    "RTI_ACCELERATION_LIMIT":   {"cause": "the PC's hard-RTI stream changed the joint's speed faster than its acceleration limit (+10 %) — a jump, a dead stop or a wild packet (a 'velocity discontinuity') — the robot braked to rest and refused the stream.", "fix": "send a smooth trajectory that decelerates within the limits; filter noisy targets (rti_set_lowpass); clear errors and re-claim control."},
    "RTI_JERK_LIMIT":           {"cause": "the PC's hard-RTI stream changed the joint's acceleration faster than its jerk limit (+10 %) (an 'acceleration discontinuity') — the robot braked to rest and refused the stream.", "fix": "send a jerk-limited trajectory; filter noisy targets (rti_set_lowpass); clear errors and re-claim control."},

    # Safety monitor (utility/safety_monitor.py) — what the arm ACTUALLY did, checked every tick
    "OUT_OF_RANGE":             {"cause": "the measured joint position went past its software limit (+ tolerance) while the robot was moving itself (JOG, EXEC, RTI) — a collision, a push, or motion the limits should have stopped. (In IDLE freedrive this is only a warning.)", "fix": "clear errors, then jog the joint back into range (only motion back in is allowed)."},
    "SOFT_LIMIT_WARNING":       {"cause": "the joint is past its software limit — dragged there in freedrive, or it rests there. JOG, EXEC and RTI will only move it back in.", "fix": "none needed; jog or move it back inside and the warning clears itself."},
    "HARD_LIMIT":               {"cause": "a joint without an endstop (J6) and without a freedrive wall passed its hard limit — the cable wind-up boundary. Nothing physical stops it there.", "fix": "check the cables, clear errors, then move the joint back (only motion back in is allowed)."},
    "JOINT_WALL":               {"cause": "the joint was pushed through its freedrive wall (more than its breakthrough past it) and its wall is set to stop (robot XML <freedrive_wall freedrive=\"stop\">).", "fix": "clear errors, enable, then move the joint back inside (only motion back in is allowed)."},
    "JOINT_WALL_WARNING":       {"cause": "the joint was pushed through its freedrive wall in freedrive (more than its breakthrough past it).", "fix": "turn it back inside the wall; the warning clears itself (for J6 check the cables: its wall is the wind-up boundary)."},
    "BEYOND_ENDSTOP":           {"cause": "the measured joint position is further past its physical endstop than <endstop_tolerance_deg> (robot XML, 10 deg on PAR6) — impossible for a real arm, so the position reference is wrong. Homing has been invalidated.", "fix": "check the joint (slipped belt/gear, encoder), clear errors, then re-home."},
    "OVERSPEED":                {"cause": "the joint's filtered speed stayed above its velocity ceiling x margin.", "fix": "check what was commanding the joint; clear errors to resume."},
    "TRACKING":                 {"cause": "the joint fell too far behind its commanded position during EXEC motion — a bump, a collision or a stall.", "fix": "check for an obstruction or an overload, clear errors, then continue."},
    "OVERSPEED_WARNING":        {"cause": "the joint's filtered speed went above its velocity ceiling x margin (the check runs warn-only until its thresholds are measured).", "fix": "none needed; RTI.log has the value and, when it clears, the excursion's peak — if this was normal motion, the ceiling or margin needs raising before the check is switched to stop."},
    "TRACKING_WARNING":         {"cause": "the joint fell further behind its commanded position than its limit during EXEC motion (the check runs warn-only until its thresholds are measured).", "fix": "none needed; RTI.log has the value and the excursion's peak — if this was normal motion, raise <tracking_limit_deg> for that joint before switching the check to stop."},
    "SELF_COLLISION":           {"cause": "two parts of the arm came within the self-collision monitor margin of each other on the MEASURED pose (the collision worker's check of the convex pieces).", "fix": "move the arm apart (jog away: only directions that open the gap are allowed), then clear errors. If this was a planned move, the planner's margin should have refused it — check logs/self_collision.log and the pair named here."},
    "SELF_COLLISION_WARNING":   {"cause": "two parts of the arm are within the self-collision margin (warn-only mode).", "fix": "none needed; the pair and gap are in RTI.log; switch <self_collision><reaction> to stop once trusted."},
    "SELF_COLLISION_STALE":     {"cause": "the self-collision worker (self_collision_worker.py) has not updated shared memory for half a second — its check is not running, so self-collisions are not being watched.", "fix": "check logs/self_collision.log and the supervisor; the worker restarts with the service."},
    "HOLD_OSCILLATION_WARNING": {"cause": "the joint's motor current keeps swinging fast while it is commanded to stand still — the drive is fighting itself (an audible buzz / whine).", "fix": "most likely its PID gains are too high (lower KPV / KPP, tools/tune_motor_loops.py); else check its encoder magnet and the mechanics. Warn only — RTI.log has the level and the peak; api_tests/59_hold_noise.py records it."},
    "TCP_OVERSPEED_WARNING":    {"cause": "the tool moved faster than <safety><max_tcp_velocity> x margin (warn-only mode).", "fix": "none needed; RTI.log has the peak."},
    "TCP_OVERSPEED":            {"cause": "the tool moved faster than <safety><max_tcp_velocity> x margin.", "fix": "check what was commanding the arm; clear errors to resume."},
    # Safety zones of the active cell (utility/zones.py), formatted as "Z_<zone>:KEY" at runtime
    "ZONE_VIOLATION":           {"cause": "the measured TCP or a tool sphere crossed a stop-zone of the active cell (left an allowed zone or entered a forbidden one) — RTI.log names the zone and the depth.", "fix": "clear errors, then move the robot back out of the violation (only motion that does not go deeper is allowed); check the zone in config/cells/ if it is wrong."},
    "ZONE_WARNING":             {"cause": "the robot is in a warn-zone of the active cell.", "fix": "none needed — clears itself when the robot leaves the zone."},

    # RT loop health (see LoopDegradationMonitor)
    "LOOP_DEGRADED":            {"cause": "RT loop is running slower than its configured rate.", "fix": "check CPU load, thermal throttling, or a silent CAN node; clears itself once the loop recovers."},
    "LOOP_CRITICAL":            {"cause": "RT loop has been critically slow (p99 period over 1.5x nominal for a second, or one tick over 3x) — real-time guarantees are not being met.", "fix": "check CPU load/thermals and the timing stats in the GUI's RT Diagnostics tab, then clear errors to resume."},

    # Per-joint motor errors  (formatted as "J{i}:KEY" at runtime)
    "TEMPERATURE":              {"cause": "motor is overheating.", "fix": "let the motor cool down before re-enabling."},
    "ENCODER":                  {"cause": "encoder error — position signal is lost or corrupted.", "fix": "check the encoder cable/connector on this joint."},
    "VBUS":                     {"cause": "bus voltage error — supply voltage is out of range.", "fix": "check the power supply and bus wiring."},
    "DRIVER":                   {"cause": "motor driver fault — internal hardware error.", "fix": "power-cycle the joint driver; replace if the fault recurs."},
    "VELOCITY":                 {"cause": "velocity error — motor exceeded its velocity limit.", "fix": "check for an unexpected collision/obstruction, then re-enable."},
    "CURRENT":                  {"cause": "current error — overcurrent detected.", "fix": "check for a mechanical jam on this joint before re-enabling."},
    "ESTOP_MOTOR":              {"cause": "e-stop error reported by the motor driver.", "fix": "check the motor driver's own e-stop input wiring."},
    "WATCHDOG":                 {"cause": "watchdog error — motor did not receive a command in time.", "fix": "check CAN wiring/power to the joint; re-enable once the fault clears."},

    # Per-joint CAN communication errors
    "CAN_LOST":                 {"cause": "CAN connection lost — no data received for too long.", "fix": "check CAN bus wiring/termination and joint power."},
    "CAN_STALE":                {"cause": "CAN data is getting stale — motor may be slow to respond.", "fix": "check CAN bus load/wiring; usually resolves on its own."},
    "CAN_REPLY_MISSED_WARNING": {"cause": "the joint's drive replies keep missing RTI's CAN read (5 or more in the last second, each 1-9 ticks late) — its measured state is a tick old on those ticks; seen when the CAN chip's kernel threads run on RTI's core (2026-10-12).", "fix": "RTI.log's '[CAN] interrupt threads ... on cores' line at start-up must not report a problem; api_tests/69_can_gaps.py measures the freezes. Warn only — RTI.log has the peak."},

    # Homing
    "NOT_HOMED":                {"cause": "robot is not homed — home the robot before operating.", "fix": "run the home command."},
    "HOMING_FAILED":            {"cause": "homing failed — could not find the endstop.", "fix": "check the endstop/limit switch wiring for this joint, then retry homing."},

    # Gripper errors
    "GRIPPER_DISCONNECTED":     {"cause": "gripper is not connected — no CAN reply from the gripper.", "fix": "check the gripper's CAN wiring/power."},
    "GRIPPER_TIMEOUT":          {"cause": "gripper timeout error — gripper did not respond in time.", "fix": "check gripper CAN wiring/power."},
    "GRIPPER_ESTOP":            {"cause": "gripper e-stop error — gripper received an e-stop signal.", "fix": "clear the gripper's e-stop condition."},
    "GRIPPER_ERROR":            {"cause": "gripper reported a general fault.", "fix": "check logs/high_level.log for detail; power-cycle the gripper if it recurs."},

    # Gripper motor-level errors (same faults as arm motors, reported by the gripper's motor driver)
    "GRIPPER_MOTOR_TEMPERATURE": {"cause": "gripper motor is overheating.", "fix": "let the gripper motor cool down before re-enabling."},
    "GRIPPER_MOTOR_ENCODER":     {"cause": "gripper motor encoder error — position signal is lost or corrupted.", "fix": "check the gripper's encoder cable/connector."},
    "GRIPPER_MOTOR_VBUS":        {"cause": "gripper motor bus voltage error — supply voltage is out of range.", "fix": "check the power supply and bus wiring to the gripper."},
    "GRIPPER_MOTOR_DRIVER":      {"cause": "gripper motor driver fault — internal hardware error.", "fix": "power-cycle the gripper driver; replace if the fault recurs."},
    "GRIPPER_MOTOR_VELOCITY":    {"cause": "gripper motor velocity error — exceeded velocity limit.", "fix": "check for an obstruction in the gripper before re-enabling."},
    "GRIPPER_MOTOR_CURRENT":     {"cause": "gripper motor current error — overcurrent detected.", "fix": "check for a mechanical jam in the gripper before re-enabling."},
    "GRIPPER_MOTOR_ESTOP":       {"cause": "gripper motor e-stop error reported by the motor driver.", "fix": "check the gripper motor driver's own e-stop input wiring."},
    "GRIPPER_MOTOR_WATCHDOG":    {"cause": "gripper motor watchdog error — motor did not receive a command in time.", "fix": "check CAN wiring/power to the gripper; re-enable once the fault clears."},
}
