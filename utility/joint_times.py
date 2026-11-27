"""When each joint's angle in RTI's trace was measured (2026-11-25; TODO/camera_review_plan.md, option A: true timing).

RTI records, per row and joint, when the reply that carried the angle arrived: the trace's column j_rx (shared memory
motor_reply_time), on RTI's own clock (time.perf_counter, the clock of the rows' timestamps and of the camera's frame
times). The drive measured the angle when RTI's command reached it and answered at once (firmware 110: the CAN handler
packs the newest encoder position of its 6250 Hz loop), so the angle is REPLY_AFTER_MEASURE_S older than its reply's
arrival: the reply's frame on the bus, then the kernel's receive. Measured 2026-11-25 with tools/loop_delay.py (each
command's bus time against its reply's, per joint): 0.18 to 0.24 ms, median 0.21.

A row's angles are 2.9 to 4.5 ms old at the row's own timestamp (measured the same day: J1 the oldest, J6 the
youngest; it moves by a few tenths of a ms between runs and ticks), which is why a picture is paired with each joint on
that joint's own measurement times, not on the row's.

The code lives in robotclient/joint_times.py since 2026-11-27 (the PC's LeRobot export pairs frames the same way, and
robotclient goes to the PC on its own); this module re-exports it: one copy, one constant.
"""
from robotclient.joint_times import REPLY_AFTER_MEASURE_S, joint_at, joint_samples, measured_at

__all__ = ["REPLY_AFTER_MEASURE_S", "joint_at", "joint_samples", "measured_at"]
