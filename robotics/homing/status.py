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
