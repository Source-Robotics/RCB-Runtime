"""The home reference: a joint's encoder position at its endstop -> its joint angle."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

import math


def _update_home_reference(joint, motor_pos: int, home_offset: float):
    """Update joint reference so get_joint_position(motor_pos) == home_offset."""
    joint.master_position = int(motor_pos)
    joint.offset          = float(home_offset)
    if joint.dir == 0:
        joint.offset_ticks = joint.radians_to_ticks(joint.offset)
    else:
        joint.offset_ticks = joint.radians_to_ticks(2.0 * math.pi - joint.offset)
