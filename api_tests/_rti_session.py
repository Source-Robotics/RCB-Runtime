#!/usr/bin/env python3
"""The RTI session the hardware tests used to re-implement (53, 54, 56 — Phase 5, 2026-10-03).

    from _rti_session import rti_session, Pacer

    with rti_session(robot) as (rti, first):          # rti_connect + claim + the first state packet
        pace = Pacer(FS)
        while ...:
            rti.send_joint_torque(...)
            pace()                                    # once per loop: sleeps to the next tick, no drift
    # here control is released and the robot is back in IDLE — ALWAYS, on success, on ControlLost and on Ctrl-C

The teardown is the one every test had: stop, release, close, then rti_disconnect(wait=True), each
best-effort so a link that is already gone cannot mask the test's own result. Nothing else changed: the
calls, their order and the 0.3 s settle after pairing are the ones the tests made.
"""
import time
from contextlib import contextmanager

from robotclient import RTIClient

FS = 250.0          # RTI's loop rate — the rate a hard-RTI test paces its loop at


@contextmanager
def rti_session(robot, rate_hz=None, clock="pc", settle_s=0.3, first_read_s=1.0, stop=True):
    """Pair with RTI (SOFT_RTI when rate_hz is given), claim control, read the first state packet.

    Yields (rti, first): the RTIClient and that first packet (a dict with j_p, j_tc, ... per the recipe).
    On exit — however the block ends — stop / release / close and rti_disconnect, each best-effort.
    stop=False leaves the stop out (53: after a zone trip the robot has already stopped itself).
    """
    rti = RTIClient(robot, clock=clock)
    if rate_hz is None:
        rti.connect()
    else:
        rti.connect(rate_hz=rate_hz)
    try:
        time.sleep(settle_s)
        rti.claim()
        first = rti.read_once(timeout=first_read_s)
        yield rti, first
    finally:
        for fn in ((rti.stop, rti.release, rti.close) if stop else (rti.release, rti.close)):
            try:
                fn()
            except Exception:
                pass
        try:
            robot.rti_disconnect(wait=True)
        except Exception:
            pass


class Pacer:
    """Call once per loop iteration: sleeps until the next tick of `rate_hz`, measured from the first call's
    instant, so the loop keeps the rate without accumulating drift."""

    def __init__(self, rate_hz=FS):
        self.dt = 1.0 / rate_hz
        self._next = time.monotonic()

    def __call__(self):
        self._next += self.dt
        s = self._next - time.monotonic()
        if s > 0:
            time.sleep(s)
