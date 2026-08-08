"""Safety and configuration checks used by the RT loop."""

import numpy as np
import time


def real_time_test():
    """Print actual sleep duration — verifies the RT kernel is active (should be ~1.000 s)."""
    t1 = time.perf_counter()
    time.sleep(1)
    t2 = time.perf_counter()
    print(f"sleep time is {t2 - t1}")


def check_elbow_configuration(q):
    """Return 'Elbow Up' or 'Elbow Down' based on joint 3 angle."""
    straight_pose_j3 = np.pi / 2
    if q[2] > straight_pose_j3:
        return "Elbow Down"
    elif q[2] < straight_pose_j3:
        return "Elbow Up"


def is_singular(robot, joint_angles):
    """Return True if the robot is at or near a kinematic singularity."""
    J      = robot.jacob0(joint_angles)
    det_J  = np.linalg.det(J)
    cond_J = np.linalg.cond(J)
    return abs(det_J) < 1e-6 or cond_J > 1e6


SEVERITY_ORDER = {
    "INFO": 0,
    "WARNING": 1,
    "ERROR": 2,
    "CRITICAL": 3,
}


def _escalate(current_level, new_level):
    if SEVERITY_ORDER[new_level] > SEVERITY_ORDER[current_level]:
        return new_level
    return current_level


def _finite_array(x):
    arr = np.asarray(x, dtype=float)
    return arr[np.isfinite(arr)]


def _has_sentinel(x):
    arr = np.asarray(x, dtype=float)
    return np.any(arr <= -999000)


def _latest_motor_history_values(motor_data_history, key, lookback):
    values = []
    if isinstance(motor_data_history, dict):
        histories = motor_data_history.values()
    else:
        histories = motor_data_history

    for motor_hist in histories:
        # Skip malformed entries (e.g. accidental scalars) rather than crashing RT loop.
        if isinstance(motor_hist, (int, float, np.integer, np.floating)):
            continue
        try:
            recent = list(motor_hist)[-lookback:]
        except TypeError:
            continue
        for sample in recent:
            if not isinstance(sample, dict):
                continue
            val = sample.get(key, None)
            if val is None:
                continue
            try:
                f = float(val)
            except (TypeError, ValueError):
                continue
            if np.isfinite(f) and f > -999000:
                values.append(f)
    return np.asarray(values, dtype=float)


def _last_two_joint_torque_samples(joint_data_history):
    if len(joint_data_history) < 2:
        return None, None
    prev_sample = joint_data_history[-2].get("j_t", None)
    curr_sample = joint_data_history[-1].get("j_t", None)
    if prev_sample is None or curr_sample is None:
        return None, None
    prev_arr = np.asarray(prev_sample, dtype=float)
    curr_arr = np.asarray(curr_sample, dtype=float)
    if _has_sentinel(prev_arr) or _has_sentinel(curr_arr):
        return None, None
    return prev_arr, curr_arr


def run_safety_checks(
    *,
    joint_position,
    joint_speed,
    motor_temperature,
    loop_execution_time,
    delta_t,
    overrun_count,
    old_data_warning,
    can_node_connection_error,
    estop_active,
    joint_limit_negative,
    joint_limit_positive,
    motor_data_history,
    joint_data_history,
    safety_cfg,
    safety_state,
    # --- not yet wired, reserved for placeholder checks below ---
    joint_torque=None,         # (joint_num,) Nm  — joint_power_limit, stall_detection
    joint_acceleration=None,   # (joint_num,) rad/s²  — acceleration_limits
    joint_jerk=None,           # (joint_num,) rad/s³  — jerk_limits
    tcp_speed=None,            # (6,) m/s + rad/s  — tcp_velocity_limits
    tcp_force_external=None,   # (3,) N  — tcp_force_limits
    tcp_torque_external=None,  # (3,) Nm — tcp_torque_limits
    commanded_position=None,   # (joint_num,) rad  — stall_detection
    robot=None,                # robot model object  — self_collision_detection
):
    """Run all safety checks and return aggregated status for this loop tick."""
    level = "INFO"
    reasons = []
    warning_reasons = []

    # --------- 4a: Joint position limits ---------
    if safety_cfg["check_joint_position"]:
        pos = np.asarray(joint_position, dtype=float)
        lim_neg = np.asarray(joint_limit_negative, dtype=float)
        lim_pos = np.asarray(joint_limit_positive, dtype=float)
        if not _has_sentinel(pos):
            above = np.where(pos > lim_pos)[0]
            below = np.where(pos < lim_neg)[0]
            if len(above) or len(below):
                level = _escalate(level, "ERROR")
                reasons.append(
                    f"joint position limit violation: high={above.tolist()} low={below.tolist()}"
                )

    # --------- 4a: Joint velocity limits ---------
    if safety_cfg["check_joint_velocity"]:
        speed = np.asarray(joint_speed, dtype=float)
        max_joint_velocity = float(safety_cfg["max_joint_velocity"])
        sustain_ticks = int(safety_cfg["joint_velocity_sustain_ticks"])
        vel_counter = safety_state.setdefault("joint_vel_overlimit_ticks", np.zeros_like(speed, dtype=int))
        if not _has_sentinel(speed):
            over = np.abs(speed) > max_joint_velocity
            vel_counter[over] += 1
            vel_counter[~over] = 0
            if np.any(over):
                level = _escalate(level, "WARNING")
                warning_reasons.append(
                    f"joint velocity over limit on joints={np.where(over)[0].tolist()}"
                )
            if np.any(vel_counter >= sustain_ticks):
                level = _escalate(level, "ERROR")
                reasons.append(
                    f"joint velocity sustained over limit for >= {sustain_ticks} ticks"
                )

    # --------- 4a: Motor temperature ---------
    if safety_cfg["check_motor_temperature"]:
        mot_temp = np.asarray(motor_temperature, dtype=float)
        warn_temp = float(safety_cfg["warn_motor_temperature"])
        err_temp = float(safety_cfg["max_motor_temperature"])
        if not _has_sentinel(mot_temp):
            warn_idx = np.where(mot_temp >= warn_temp)[0]
            err_idx = np.where(mot_temp >= err_temp)[0]
            if len(warn_idx):
                level = _escalate(level, "WARNING")
                warning_reasons.append(f"motor temp warning on joints={warn_idx.tolist()}")
            if len(err_idx):
                level = _escalate(level, "ERROR")
                reasons.append(f"motor temp error on joints={err_idx.tolist()}")

    # --------- 4a: Motor current spike from deque history ---------
    if safety_cfg["check_current_spike"]:
        current_vals = _latest_motor_history_values(
            motor_data_history,
            key="current",
            lookback=int(safety_cfg["current_spike_lookback"]),
        )
        if current_vals.size >= 6:
            peak = float(np.max(np.abs(current_vals[-3:])))
            baseline = float(np.median(np.abs(current_vals[:-3])))
            if baseline > 1.0:
                spike_ratio = peak / baseline
                if spike_ratio >= float(safety_cfg["current_spike_factor"]):
                    level = _escalate(level, "WARNING")
                    warning_reasons.append(
                        f"current spike detected (ratio={spike_ratio:.2f}x baseline)"
                    )
                if spike_ratio >= float(safety_cfg["current_spike_factor"]) * 1.5:
                    level = _escalate(level, "ERROR")
                    reasons.append(
                        f"severe current spike detected (ratio={spike_ratio:.2f}x baseline)"
                    )

    # --------- 4a: Torque discontinuity from deque history ---------
    if safety_cfg["check_torque_discontinuity"]:
        prev_torque, curr_torque = _last_two_joint_torque_samples(joint_data_history)
        if prev_torque is not None and curr_torque is not None:
            jump = np.abs(curr_torque - prev_torque)
            jump_idx = np.where(jump > float(safety_cfg["collision_torque_threshold"]))[0]
            if len(jump_idx):
                level = _escalate(level, "ERROR")
                reasons.append(
                    f"torque discontinuity on joints={jump_idx.tolist()} (possible collision)"
                )

    # --------- 4a: Vibration detection from motor speed variance ---------
    if safety_cfg["check_vibration"]:
        speed_vals = _latest_motor_history_values(
            motor_data_history,
            key="speed",
            lookback=int(safety_cfg["vibration_lookback"]),
        )
        if speed_vals.size >= 8:
            variance = float(np.var(speed_vals))
            if variance > float(safety_cfg["vibration_speed_variance_threshold"]):
                level = _escalate(level, "WARNING")
                warning_reasons.append(
                    f"speed variance high ({variance:.2f}) -> possible vibration"
                )

    # --------- 4a: Loop timing overrun ---------
    if safety_cfg["check_loop_overrun"]:
        if float(loop_execution_time) > float(delta_t):
            level = _escalate(level, "WARNING")
            warning_reasons.append(
                f"loop overrun this tick ({loop_execution_time*1e3:.3f} ms > {delta_t*1e3:.3f} ms)"
            )
        if float(overrun_count) >= float(safety_cfg["loop_overrun_limit"]):
            level = _escalate(level, "ERROR")
            reasons.append(
                f"loop overrun_count={int(overrun_count)} >= limit={int(safety_cfg['loop_overrun_limit'])}"
            )

    # --------- 4a: CAN health ---------
    if safety_cfg["check_can_health"]:
        old_data = np.asarray(old_data_warning, dtype=float)
        can_err = np.asarray(can_node_connection_error, dtype=float)
        if np.any(old_data > 0):
            level = _escalate(level, "WARNING")
            warning_reasons.append("old CAN data warning present")
        if np.any(can_err >= float(safety_cfg["can_error_limit"])):
            level = _escalate(level, "CRITICAL")
            reasons.append("CAN node connection error detected")

    # --------- 4a: E-stop hardware ---------
    if safety_cfg["check_estop"]:
        if bool(estop_active):
            level = _escalate(level, "CRITICAL")
            reasons.append("E-stop active")

    # --------- placeholder: joint power limit ---------
    # Total mechanical power across all joints: sum(|torque_i * speed_i|)
    # Bounds energy output regardless of whether it comes from speed or torque.
    # A slow high-torque joint can be just as dangerous as a fast low-torque one.
    # Needs: joint_torque (Nm), joint_speed (rad/s)
    # Config keys: check_joint_power, max_joint_power (W)
    if safety_cfg.get("check_joint_power", 0):
        pass  # TODO: not yet implemented

    # --------- placeholder: joint acceleration limits ---------
    # Sustained high acceleration stresses the gearbox and signals unexpected dynamics.
    # Similar pattern to velocity limits — use a sustain_ticks counter to avoid false
    # positives from single-tick transients.
    # Needs: joint_acceleration (rad/s²)
    # Config keys: check_joint_acceleration, max_joint_acceleration, joint_accel_sustain_ticks
    if safety_cfg.get("check_joint_acceleration", 0):
        pass  # TODO: not yet implemented

    # --------- placeholder: jerk limits ---------
    # Excessive jerk = mechanical shock to the drivetrain and gearbox.
    # Single-tick threshold is sufficient (jerk spikes are instantaneous).
    # Needs: joint_jerk (rad/s³)
    # Config keys: check_joint_jerk, max_joint_jerk
    if safety_cfg.get("check_joint_jerk", 0):
        pass  # TODO: not yet implemented

    # --------- placeholder: TCP velocity limit ---------
    # Joint velocity limits do NOT bound TCP speed — near singularities a slow
    # joint can produce very high end-effector velocity.
    # Use translational norm: np.linalg.norm(tcp_speed[:3])
    # Needs: tcp_speed (6,) [vx, vy, vz, wx, wy, wz]
    # Config keys: check_tcp_velocity, max_tcp_velocity (m/s), max_tcp_angular_velocity (rad/s)
    if safety_cfg.get("check_tcp_velocity", 0):
        pass  # TODO: not yet implemented

    # --------- placeholder: TCP force/torque limits ---------
    # Two-tier limit: low threshold = contact WARNING, high threshold = collision ERROR.
    # force_external and torque_external are already computed in update_shared_data every tick.
    # Needs: tcp_force_external (3,) N, tcp_torque_external (3,) Nm
    # Config keys: check_tcp_force, tcp_force_contact_threshold (N), tcp_force_collision_threshold (N)
    #              tcp_torque_contact_threshold (Nm), tcp_torque_collision_threshold (Nm)
    if safety_cfg.get("check_tcp_force", 0):
        pass  # TODO: not yet implemented

    # --------- placeholder: stall detection ---------
    # Motor is commanded to a position but not moving while current is at limit —
    # joint is stalled against an unexpected obstacle.
    # Different from joint slip: stall = not moving when it should be;
    #                            slip  = moving in the wrong direction.
    # Needs: joint_position (actual rad), commanded_position (target rad), joint_torque (Nm)
    # Config keys: check_stall, stall_position_error_threshold (rad),
    #              stall_torque_threshold (Nm), stall_ticks
    if safety_cfg.get("check_stall", 0):
        pass  # TODO: not yet implemented

    # --------- placeholder: self-collision detection ---------
    # Check if current joint configuration brings any two links within a minimum
    # clearance distance of each other.
    # Requires a collision geometry model (capsules or spheres per link) — not yet defined.
    # Computationally heavier than other checks; may need to run every N ticks.
    # Needs: robot model with geometry, joint_position
    # Config keys: check_self_collision, self_collision_clearance (m)
    if safety_cfg.get("check_self_collision", 0):
        pass  # TODO: not yet implemented

    # --------- placeholder: payload identification ---------
    # At rest (velocity ≈ 0), the measured joint torques should match the gravity
    # model prediction for the known payload. A persistent mismatch means the
    # attached payload has changed unexpectedly (tool swap, dropped object, etc.).
    # Needs: joint_torque (Nm), joint_speed (rad/s), robot model (gravity prediction)
    # Config keys: check_payload_change, payload_torque_mismatch_threshold (Nm),
    #              payload_check_velocity_threshold (rad/s)
    if safety_cfg.get("check_payload_change", 0):
        pass  # TODO: not yet implemented

    # Merge warning reasons behind critical/error reasons for compact logs.
    combined_reasons = reasons + warning_reasons

    return {
        "level": level,
        "safety_violation": level in ("ERROR", "CRITICAL"),
        "warning_only": level == "WARNING",
        "reasons": combined_reasons,
        "speed_scale": float(safety_cfg["warning_speed_scale"]) if level == "WARNING" else 1.0,
    }
