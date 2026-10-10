"""
Homing routines for joint calibration.

Two homing strategies, selected per joint via the has_hall_sensor XML field:

  Hall-sensor joints (e.g. PAR6 J6, has_hall_sensor=1):
      Motor runs in HALL_INDEX mode (Send_data_pack_HALL).
      Endstop detected when Motor.HALL_trigger == 0 OR Motor.hall_index == 1.

  Endstop/stall joints (all others, has_hall_sensor=0):
      Motor runs in velocity mode (Send_data_pack_1 with Speed set).
      Endstop detected when position stops changing AND current exceeds
      homing_current_ma threshold for N consecutive ticks.

After detection both strategies (two-pass / single-pass):

  Two-pass mode (default, as 3D printers' sensorless homing does it):
      1. Fast first approach  → endstop hit  → dwell (motor stops) → back off
         for backoff_s seconds → pause (motor stops again).
      2. Slow second approach → endstop hit  → settle.
      The dwell and pause phases ensure the motor is fully stationary before
      direction changes, preventing false triggers from momentum/current spikes.
      The second pass uses rehome_speed_factor × the original homing speed for a
      more repeatable trigger point.

  Single-pass mode (two_pass_homing[j] == False):
      Endstop detection → settle immediately (original behaviour).

  Release (optional, per joint, stall joints only):
    After the final endstop detection, command a controlled current (or zero)
    for a configurable duration.  This relieves gearbox preload so the joint
    relaxes to its true resting endstop position.  The encoder position is
    latched at a configurable point during the release window.
    Controlled by homing_release_current_ma, homing_release_duration_s,
    homing_release_sample_pct.  All NaN = skip (default).

  Settle: Send Idle for _SETTLE_TIME seconds, then update
    joint.master_position to the endstop encoder tick so that
    get_joint_position(endstop_tick) == home_offset[j] radians.

  Post-home move (optional, per joint):
    After the settle period a joint may be driven to a specific angle before the
    next joint starts homing.  Controlled by post_home_position[j] (radians).
    NaN = skip (default).  Speed is post_home_speeds[j].
    Useful when a downstream joint can only reach its endstop if the upstream
    joint is already in a known clear position.

Homing order
------------
  By default joints are homed in index order (0, 1, 2 …).  Override by setting
  home_sequence directly, or via the per-joint XML field <homing_order>N</homing_order>
  (1-based integer).  Joints without a value are appended last in index order.

  Parallel homing is supported via a homing sequence file (see parse_homing_sequence).
  Each step can home multiple joints simultaneously with optional pre-moves.

Integration with RTI.py
-----------------------
1.  Instantiate once after XML loading:
        homing_ctrl = HomingController.from_config(cfg, joint_num)

2.  On HOMING mode transition:
        homing_ctrl.enter(Motor, joint_num, load_motor_configs, motor_config)

3.  Each RT tick instead of send_motor_commands():
        result = homing_ctrl.step(Motor, Joint, shared_data)
        if result == 'done':   pending_mode_request = ENABLED
        if result == 'failed': pending_mode_request = ERROR

4.  On leaving HOMING mode:
        homing_ctrl.exit(Motor, joint_num, load_motor_configs, motor_config)
"""

# 2026-10-09 (TODO/homing_plan.md H3): this file was the whole module (2139 lines); it is now a package, one
# module per job — status, reference, joint (one joint's phases), gripper (firmware calibration), config
# (from_config), moves (pre-moves / move_to / post-moves / holds), controller (the sequence). Import from here
# as before; the names below are the same objects.

from robotics.homing.status import STATUS_DONE, STATUS_FAILED, STATUS_IDLE, STATUS_RUNNING
from robotics.homing.reference import _update_home_reference
from robotics.homing.joint import JointHomingState
from robotics.homing.gripper import GripperHomingState
from robotics.homing.controller import HomingController

__all__ = ["STATUS_IDLE", "STATUS_RUNNING", "STATUS_DONE", "STATUS_FAILED", "JointHomingState",
           "GripperHomingState", "HomingController", "_update_home_reference"]
