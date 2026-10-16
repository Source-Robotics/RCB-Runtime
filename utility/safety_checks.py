"""Safety and configuration checks used by the RT loop."""

import numpy as np


# real_time_test() (a 1 s sleep whose duration it printed) was removed on 2026-10-15: never called, and the
# services log instead of printing. The real-time check is config/system_config.setup_realtime_process's
# "[RT setup] OK" line in RTI.log.


def is_singular(robot, joint_angles):
    """Return True if the robot is at or near a kinematic singularity."""
    J      = robot.jacob0(joint_angles)
    det_J  = np.linalg.det(J)
    cond_J = np.linalg.cond(J)
    return abs(det_J) < 1e-6 or cond_J > 1e6


# run_safety_checks() and its helpers were removed on 2026-09-30. It was never called from the RT
# loop, cost ~1.1 ms of the 4 ms tick when measured, and pooled all motors into one array (a spike
# on J1 was invisible; smooth motion read as "vibration"). Its replacement is
# utility/safety_monitor.py — see TODO/safety_stack_research.md for the measurements.
