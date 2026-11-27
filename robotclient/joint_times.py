"""When each joint's angle in the robot's packets was measured (2026-11-25; TODO/camera_review_plan.md, option A: true
timing). Here since 2026-11-27 (from utility/joint_times.py, which re-exports it): the PC's LeRobot export pairs camera
frames with joints the same way, and robotclient goes to the PC on its own. Needs numpy.

RTI records, per row and joint, when the reply that carried the angle arrived: the trace's column / the packets' tag
j_rx (shared memory motor_reply_time; the gripper's: g_rx), on RTI's own clock (time.perf_counter, the clock of the
rows' timestamps and of the camera's frame times). The drive measured the angle when RTI's command reached it and
answered at once (firmware 110: the CAN handler packs the newest encoder position of its 6250 Hz loop), so the angle
is REPLY_AFTER_MEASURE_S older than its reply's arrival: the reply's frame on the bus, then the kernel's receive.
Measured 2026-11-25 with tools/loop_delay.py (each command's bus time against its reply's, per joint): 0.18 to 0.24 ms,
median 0.21.

A row's angles are 2.9 to 4.5 ms old at the row's own timestamp (measured the same day: J1 the oldest, J6 the
youngest; it moves by a few tenths of a ms between runs and ticks), which is why a picture is paired with each joint on
that joint's own measurement times, not on the row's.
"""
from __future__ import annotations

import numpy as np

REPLY_AFTER_MEASURE_S = 0.00021


def measured_at(reply_time):
    """When the drive measured each angle (RTI's clock, s), from the trace's j_rx (an array or a number)."""
    return np.asarray(reply_time, dtype=float) - REPLY_AFTER_MEASURE_S


def joint_samples(reply_time, values):
    """One joint's (measurement times, values) from its trace columns, ready to interpolate: rows without a reply
    (NaN) and rows that repeat the previous reply (no new reply that tick: the same angle again) left out."""
    x = measured_at(reply_time)
    y = np.asarray(values, dtype=float)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if x.size == 0:
        return x, y
    keep = np.concatenate(([True], np.diff(x) > 0))
    return x[keep], y[keep]


def joint_at(t, reply_time, values):
    """Each joint's value at the times t (RTI's clock, s), interpolated on that joint's own measurement times.
    reply_time, values: rows x joints (the trace's j_rx and a channel such as j_p). -> len(t) x joints; NaN where t
    lies outside a joint's measured span."""
    t = np.atleast_1d(np.asarray(t, dtype=float))
    rt = np.asarray(reply_time, dtype=float)
    v = np.asarray(values, dtype=float)
    out = np.full((t.size, rt.shape[1]), np.nan)
    for j in range(rt.shape[1]):
        x, y = joint_samples(rt[:, j], v[:, j])
        if x.size < 2:
            continue
        inside = (t >= x[0]) & (t <= x[-1])
        out[inside, j] = np.interp(t[inside], x, y)
    return out
