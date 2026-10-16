"""The measured value behind a fault, for the error list and the activity log (2026-10-15).

Until now a latched error said only its title: "Drive overheating (2101 TEMPERATURE)". Now the entry carries what was
measured when it latched — the way an industrial controller's event log quotes the value that tripped the limit:
"Joint 3: Drive overheating: 71 C (2101 TEMPERATURE)". RTI asks for it ONLY on the tick a key latches, from
utility.error_checks._update_error_metadata (`detail_of`), so it costs the loop nothing on the other ticks; the text
is built in this cold function.

Sources, in order:
  * the safety monitor's `details` dict — every firing site writes the value it tripped on (speed, gap, position,
    ripple current) under its key;
  * the loop monitor: LOOP_DEGRADED / LOOP_CRITICAL — the loop's p99 and longest period against the nominal tick;
  * CAN: J{n}:CAN_LOST / CAN_STALE / GRIPPER_DISCONNECTED — ticks since that node's last reply;
  * the drives: TEMPERATURE (C), VBUS (V), CURRENT (mA) per joint; the gripper's the same.
Anything else: None (the entry keeps the title alone).
"""
from __future__ import annotations

from utility.error_catalog import short_key_of


def fault_detail(key, monitor_details, rti, motor, gripper, motor_data, gripper_data, tick, delta_t, joint_num):
    """The text behind `key` now, or None. `monitor_details`: SafetyMonitor.details; `rti` / `motor` / `gripper`: the
    shared-memory blocks; `motor_data` / `gripper_data`: the per-node reply histories (the newest entry's "tick");
    `tick`: the loop tick; `delta_t`: the nominal tick (s)."""
    text = monitor_details.get(key) if monitor_details else None
    if text:
        return str(text)
    short = short_key_of(key)
    joint = _joint_index(key)
    if short in ("LOOP_DEGRADED", "LOOP_CRITICAL"):
        p99 = float(rti["p99_loop_period"][0]) * 1e3
        longest = float(rti["max_loop_period"][0]) * 1e3
        return (f"p99 period {p99:.2f} ms, longest {longest:.2f} ms since start, nominal {delta_t * 1e3:.1f} ms, "
                f"{int(rti['overrun_count'][0])} overruns since start")
    if short in ("CAN_LOST", "CAN_STALE") and joint is not None:
        return _silence(motor_data, joint, tick, delta_t)
    if short == "GRIPPER_DISCONNECTED":
        return _silence(gripper_data, 0, tick, delta_t)
    if joint is not None:
        if short == "TEMPERATURE":
            return f"{int(motor['motor_temperature'][joint])} C"
        if short == "VBUS":
            return f"{float(motor['motor_voltage'][joint]) / 1000.0:.1f} V"
        if short == "CURRENT":
            return f"{int(motor['motor_current'][joint])} mA"
    if short == "GRIPPER_MOTOR_TEMPERATURE":
        return f"{int(gripper['gripper_motor_temperature'][0])} C"
    if short == "GRIPPER_MOTOR_VBUS":
        return f"{float(gripper['gripper_motor_voltage'][0]) / 1000.0:.1f} V"
    if short == "GRIPPER_MOTOR_CURRENT":
        return f"{int(gripper['gripper_motor_current'][0])} mA"
    return None


def _joint_index(key) -> int | None:
    """'J3:X' -> 2 (0-based); None for any other key."""
    if not key.startswith("J") or ":" not in key:
        return None
    try:
        return int(key[1:key.index(":")]) - 1
    except ValueError:
        return None


def _silence(histories, index, tick, delta_t):
    try:
        last = int(histories[index][-1]["tick"])
    except (KeyError, IndexError, TypeError, ValueError):
        return None
    if last < 0:
        return "no reply since start"
    gap = max(0, int(tick) - last)
    return f"no reply for {gap} ticks ({gap * delta_t * 1e3:.0f} ms)"
