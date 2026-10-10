#!/usr/bin/env python3
"""Shared helpers for the SOFT_RTI servo tests (32-35).

Complements _common.py, which owns START_POSE_DEG and the EXEC-mode setup. This module owns
the RTI-side pieces the servo tests all need: a loop-rate shared-memory sampler, a streaming
loop that never goes silent, and the config catalogue.

TWO RULES ENCODED HERE, both learned the hard way on 2026-08-21:

1. NEVER GO SILENT WHILE CLAIMED. A claimed session has no idle state. Sleeping between
   phases -- even 0.2 s -- exceeds the command timeout at low rates, and the watchdog
   correctly latches RTI_LINK_LOST and stops the arm. Every wait here streams.

2. PACING IS STICKY. vel_scale, pace and min_duration persist until changed, so a config
   that does not name one must actively CLEAR it or it inherits the previous config's and
   the comparison is silently contaminated. apply_config() always writes every knob.
"""

import math
import struct
import threading
import time

SHM = "/dev/shm"
LOOP_HZ = 250.0
DT = 1.0 / LOOP_HZ

# Joints these tests are allowed to move. J2 (shoulder) and J3 (elbow) are EXCLUDED: they
# carry the gravity load, and J3 additionally has a known velocity-loop oscillation
# (servo_mode_options.md 14.6) that is unrelated to servo mode but would confound any
# torque measurement taken near it.
SAFE_JOINTS = (0, 3, 4, 5)          # J1, J4, J5, J6

CONFIGS = {
    "ruckig":           dict(gen="ruckig"),
    "tracker":          dict(gen="tracker"),
    "advancing":        dict(gen="ruckig_advancing"),
    "ruckig+vel25":     dict(gen="ruckig", vel_scale=0.25),
    "ruckig+pace1.5":   dict(gen="ruckig", pace=1.5),
    "ruckig+mindur1.5": dict(gen="ruckig", min_duration=1.5),
    "advancing+mindur": dict(gen="ruckig_advancing", min_duration=1.5),
    "tracker+gain30":   dict(gen="tracker", gain=30.0),
    "tracker+self":     dict(gen="tracker", self_lookahead=0.0667),
}
DEFAULT_CONFIGS = ("ruckig", "tracker", "advancing", "advancing+mindur")

# Rates that divide EXACTLY into the loop rate. A non-integer ratio makes commands land
# alternately on two tick counts, which corrupts every velocity estimate the robot derives
# from the stream -- it would confound the comparison rather than test it.
CLEAN_RATES = (5.0, 10.0, 25.0, 50.0, 125.0, 250.0)


def deg(r):
    return r * 180.0 / math.pi


def rd(name, n=6):
    with open(f"{SHM}/{name}", "rb") as f:
        return list(struct.unpack(f"{n}d", f.read(8 * n)))


def clean_rate(hz):
    t = LOOP_HZ / hz
    return abs(t - round(t)) < 1e-9


class Sampler(threading.Thread):
    """Log shared memory at the loop rate. Records the first error rather than hiding it.

    An earlier sampler read a field that did not exist behind a bare `except: pass`, so
    every read raised and the run printed an all-NaN table -- indistinguishable from a real
    measurement that found nothing. Silent failures that return a value are worse than ones
    that raise.
    """

    FIELDS = ("joint_position", "joint_position_commanded", "joint_torque")
    # Ring-buffer cap. Unbounded growth is not a memory problem here, it is a TIMING one:
    # analysing 30 s of samples takes seconds, and every second of that is a second of not
    # streaming while the claim is held.
    MAX_ROWS = 3000

    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self.error = [], None
        self._stop, self._on = threading.Event(), False

    def run(self):
        while not self._stop.is_set():
            if self._on:
                try:
                    self.rows.append(tuple(rd(f) for f in self.FIELDS))
                    if len(self.rows) > self.MAX_ROWS:
                        del self.rows[:len(self.rows) - self.MAX_ROWS]
                except Exception as exc:
                    self.error = self.error or repr(exc)
            time.sleep(DT * 0.5)

    def start_window(self):
        self.rows = []
        self._on = True

    def end_window(self):
        self._on = False
        return list(self.rows)

    def stop(self):
        self._stop.set()


class gap_guard:
    """Measure how long the command stream was interrupted, and say so if it was too long.

    A claimed session has no idle state. Everything between one stream() and the next is a
    gap: sleeps, TCP round-trips, AND COMPUTE. That last one is the easiest to miss --
    analysing 30 s of samples is over a million Python operations and takes seconds, which
    silently exceeded the timeout at every rate and killed four separate test runs before
    anyone suspected arithmetic.

        with gap_guard(rate_hz, "post-cell analysis"):
            ...work...

    Warns rather than raises: the run is already compromised by the time we know, and a
    warning that names the offender is more use than an exception that hides the numbers.
    """

    def __init__(self, rate_hz, what="", budget=0.5):
        self.limit = (3.0 / rate_hz) * budget
        self.what = what

    def __enter__(self):
        self.t0 = time.monotonic()
        return self

    def __exit__(self, *exc):
        took = time.monotonic() - self.t0
        if took > self.limit:
            print(f"\n  !! GAP {took*1e3:.0f} ms in '{self.what}' — over the "
                  f"{self.limit*1e3:.0f} ms safe budget. The claim may have timed out; "
                  f"treat the next result as suspect.")
        return False


def apply_config(robot, cfg):
    """Write EVERY knob, defaults included. Pacing is sticky -- see rule 2 above."""
    robot.rti_set_vel_scale(cfg.get("vel_scale", 1.0))
    robot.rti_set_pace(cfg.get("pace", 0.0))
    robot.rti_set_min_duration(cfg.get("min_duration", 0.0))
    robot.rti_set_lowpass(cfg.get("lowpass", 0.0))
    robot.rti_set_servo_generator(cfg["gen"],
                                  gain=cfg.get("gain"),
                                  lookahead=cfg.get("lookahead"),
                                  self_lookahead=cfg.get("self_lookahead", 0.0))


def clear_config(robot):
    """Leave nothing sticky for the next block, the next test, or the next operator."""
    robot.rti_set_vel_scale(1.0)
    robot.rti_set_pace(0.0)
    robot.rti_set_min_duration(0.0)
    robot.rti_set_lowpass(0.0)
    # The state rate is sticky too, and forgetting it is not cosmetic: a leftover 25 Hz
    # throttle from a previous run makes the next test read a 10-tick gap between state
    # packets and report it as 90% PACKET LOSS. Found exactly that way on 2026-08-21.
    try:
        robot.rti_set_state_rate(0.0)
    except Exception:
        pass          # older robot without the command — nothing to clear


def keep_alive(rti, q, rate_hz, seconds):
    """Stream `q` for `seconds`. The ONLY correct way to wait while claimed -- see rule 1."""
    period = 1.0 / rate_hz
    end, nxt = time.monotonic() + seconds, time.monotonic()
    while time.monotonic() < end:
        rti.send_joint_position(q)
        nxt += period
        slp = nxt - time.monotonic()
        if slp > 0:
            time.sleep(slp)


def stream(rti, q0, shape, rate_hz, seconds, sampler=None):
    """Stream shape(t) -> {joint: offset_rad} at rate_hz. Returns (rows, achieved_hz).

    shape returns a DICT so a test can drive several joints at once with different phases.
    """
    period = 1.0 / rate_hz
    if sampler:
        sampler.start_window()
    sent, t0, nxt = 0, time.monotonic(), time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t >= seconds:
            break
        q = list(q0)
        for j, off in shape(t).items():
            q[j] = q0[j] + off
        rti.send_joint_position(q)
        sent += 1
        nxt += period
        slp = nxt - time.monotonic()
        if slp > 0:
            time.sleep(slp)
    achieved = sent / max(time.monotonic() - t0, 1e-9)
    rows = sampler.end_window() if sampler else []
    return rows, achieved


def analyse(rows, joints):
    """Per-joint (rms lag deg, reversals, hf torque rms Nm, peak excursion deg)."""
    out = {}
    if len(rows) < 20:
        return {j: (float("nan"),) * 4 for j in joints}
    for j in joints:
        meas = [r[0][j] for r in rows]
        cmd = [r[1][j] for r in rows]
        trq = [r[2][j] for r in rows]
        lag = math.sqrt(sum((m - c) ** 2 for m, c in zip(meas, cmd)) / len(meas))
        uniq = [cmd[0]]
        for v in cmd[1:]:
            if v != uniq[-1]:
                uniq.append(v)
        d = [b - a for a, b in zip(uniq, uniq[1:])]
        rev = sum(1 for x, y in zip(d, d[1:]) if x * y < 0)
        w = 13
        if len(trq) > w * 3:
            hf = [trq[i] - sum(trq[i - w:i + w + 1]) / (2 * w + 1)
                  for i in range(w, len(trq) - w)]
            trms = math.sqrt(sum(v * v for v in hf) / len(hf))
        else:
            trms = float("nan")
        peak = max(abs(m - meas[0]) for m in meas)
        out[j] = (deg(lag), rev, trms, deg(peak))
    return out


def preflight(robot, need_enabled=True):
    """Common gate. Returns None if OK, else a message."""
    # status() is the latest PUSHED status event, so it is EMPTY until one arrives. That race
    # is invisible on loopback and bites immediately over a network — wait rather than rely
    # on the event beating us here.
    try:
        robot.wait(lambda s: s.get("mode") is not None, 10.0, "first status event")
    except Exception as exc:
        return f"connected, but no status event arrived ({exc!r})"
    st = robot.status()
    if need_enabled and st.get("state") != "ENABLED":
        return f"robot is {st.get('state')}, need ENABLED"
    if not st.get("homed"):
        return "robot is not homed"
    if st.get("err"):
        return f"error latched: {st.get('errors')}"
    return None
