"""Homing status codes written to shared_data.rti["homing_status"][j]."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

# ---------------------------------------------------------------------------
# Homing status codes written to shared_data.rti["homing_status"][j]
# ---------------------------------------------------------------------------
STATUS_IDLE    = 0
STATUS_RUNNING = 1
STATUS_DONE    = 2
STATUS_FAILED  = 3


def log_milestone(code, message=None, level="info"):
    """One homing milestone in the activity log (2026-10-15): HOMING_STARTED, HOMING_JOINT_HOMED, HOMING_COMPLETED.
    RTI.log keeps the detail (~100 lines a homing); the activity log gets the steps an operator follows. A failure is
    HOMING_FAILED through the error latch (with its reason), an abort / invalidation is logged where it happens."""
    from utility.activity_log import get_rti_activity_logger, log_activity
    log_activity(get_rti_activity_logger(), "RTI", level, code, message=message)
