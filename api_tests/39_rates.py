#!/usr/bin/env python3
"""39: does every command rate stream smoothly, or only the ones that divide 250?

Off-hardware measurement says the command rate splits into two populations, and the split is
NOT "faster is worse" -- it is whether LOOP_HZ / rate is a whole number:

    clean   5 10 20 25 30 40 50 60 62.5 83.3 100 120 125 Hz    ripple 0.004%
    rough   55 65 70 90 Hz                                     ripple 20-29%

Commands can only ARRIVE on a whole tick. For a rate of 250/n the pattern of arrival gaps
repeats with period n, so 60 Hz (4.1667 ticks) arrives 4,4,4,4,4,5 forever. Everything the
robot derives from the stream -- the target velocity, the extrapolation horizon -- is derived
over that gap, so a rate whose pattern the estimator cannot average exactly gets a target
velocity that is wrong by a few percent, permanently, in an alternating pattern.

Three fixes landed on 2026-08-21 and this test exists to confirm them on the arm:
    * the aim point ages from a FRACTIONAL ideal clock, not the integer arrival tick
    * GAP_WINDOW = 12

# Shortest constant-velocity leg the ripple metric can be trusted on.
#
# This is the whole reason the first version of this test reported nothing. The generator
# needs a few seconds of UNINTERRUPTED constant velocity before its output settles, and a
# short leg measures the reversal transient instead -- which is identical at every rate, so
# every rate scored the same and the test concluded "not reproduced" against a real effect.
# Measured separation between clean and rough rates, off-hardware:
#     0.8 s legs   17.8%  vs  17.9%   -- none, the transient swamps it
#     2.0 s legs    0.15% vs  19-28%  -- 130x
#     4.0 s legs    0.024% vs 14-23%  -- 800x
MIN_LEG_S = 1.5

# Steady state needs enough COMMANDS inside the measured window, not just enough seconds. At
# 5 Hz a 1.2 s window holds six of them and the generator is still settling, so 5 Hz scores
# 33% off-hardware while the same rate on a long ramp scores 0.005%. That is the window being
# short, not the rate being rough, and a row with too few commands is marked rather than
# quietly counted as a rough result.
MIN_CMDS_IN_WINDOW = 20, which is a whole number of periods for n = 1,2,3,4,6,12
    * a drop is only declared past SPAN_MARGIN, which stopped the estimator locking at
      1.75 ticks when the truth was 2.5 (this is what made 100 Hz bad)

WHY LONG LEGS. The leg must be at least MIN_LEG_S of uninterrupted constant velocity, and
this is not a detail -- at 0.8 s legs this test measured 17.8% for clean rates and 17.9% for
rough ones and declared the effect absent. It is not: the generator needs seconds of steady
velocity before its output settles, and anything shorter measures the reversal transient,
which is identical at every rate. At 4 s legs the same rates read 0.024% and 14-23%.

WHY A TRIANGLE AND NOT A SINE. The first version of this test commanded a sine and measured
the high-frequency residual of the commanded velocity. It reported 48-124% roughness at EVERY
rate including the exactly-clean ones, against 0.004% off-hardware -- and that was the metric,
not the arm. On a sine the true velocity is always changing, so a moving-average detrend
cannot separate real roughness from command-arrival transients: at 5 Hz commands land every
50 ticks and at 125 Hz every 2, and a 27-tick window removes neither. The off-hardware number
came from the sd of velocity along a CONSTANT-velocity ramp, where the true velocity is a flat
line and sd means exactly one thing. This test now commands a triangle and measures sd inside
the constant-velocity legs, discarding the reversals -- the same quantity, on the arm.

WHAT IS MEASURED, and why it survives a Python sampler that drops samples. Velocity needs a
difference, and differencing across a DROPPED sample invents roughness that is not there --
the flaw that produced a bogus jerk reading earlier in this project. So the sampler records
`loop_tick` with every row and velocity is computed ONLY between rows exactly one tick apart.
Everything else is discarded, and the coverage is reported so a thin sample is visible rather
than silently believed.

    ripple%   sd of commanded velocity DURING A CONSTANT-VELOCITY LEG, % of that leg speed.
              The generator's own roughness -- the same quantity measured off-hardware.
    lag       rms |measured - commanded| (deg). What the arm actually did.
    t_rms     high-frequency joint torque (Nm). What the operator hears.

    python3 api_tests/39_rates.py --host <ip> --go
    python3 api_tests/39_rates.py --host <ip> --go --rates 50,55,60,100
"""
import argparse
import math
from fractions import Fraction
import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost

JN = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6"}

# The window the robot averages arrival gaps over. Must track GAP_WINDOW in rti_handler.py.
GAP_WINDOW = 12

# Shortest constant-velocity leg the ripple metric can be trusted on.
#
# This is the whole reason the first version of this test reported nothing. The generator
# needs a few seconds of UNINTERRUPTED constant velocity before its output settles, and a
# short leg measures the reversal transient instead -- which is identical at every rate, so
# every rate scored the same and the test concluded "not reproduced" against a real effect.
# Measured separation between clean and rough rates, off-hardware:
#     0.8 s legs   17.8%  vs  17.9%   -- none, the transient swamps it
#     2.0 s legs    0.15% vs  19-28%  -- 130x
#     4.0 s legs    0.024% vs 14-23%  -- 800x
MIN_LEG_S = 1.5

# Steady state needs enough COMMANDS inside the measured window, not just enough seconds. At
# 5 Hz a 1.2 s window holds six of them and the generator is still settling, so 5 Hz scores
# 33% off-hardware while the same rate on a long ramp scores 0.005%. That is the window being
# short, not the rate being rough, and a row with too few commands is marked rather than
# quietly counted as a rough result.
MIN_CMDS_IN_WINDOW = 20


def predicted_clean(hz, window=GAP_WINDOW):
    """Should this rate stream smoothly?

    NOT the same question as `sc.clean_rate`, which asks whether 250/rate is a whole number.
    That was the right predicate before 2026-08-21 and is the WRONG one now: the fixes landed
    that day made 20, 30, 40, 60, 100 and 120 Hz clean without making any of them integer.

    Write 250/rate as p/q in lowest terms. Arrivals are whole ticks, so the pattern of gaps
    repeats with period q -- 25/6 for 60 Hz means a six-command cycle of 4,4,4,4,4,5. An
    average over `window` gaps is exact when q divides the window, and biased when it does
    not. So the real predicate is `q divides GAP_WINDOW`, which reproduces every off-hardware
    result: clean for q in 1,2,3,4,6,12 and rough for 55/65/70/90 Hz (q = 11,13,7,9).
    """
    ratio = (Fraction(sc.LOOP_HZ).limit_denominator(10 ** 6)
             / Fraction(hz).limit_denominator(10 ** 6))
    return window % ratio.denominator == 0

# Clean and rough interleaved, ascending. Interleaved on purpose: if the arm warms up or a
# joint drifts over the sweep, a block of clean rates followed by a block of rough ones would
# blame the rate for it. Alternating makes a trend visible instead of confounding.
# Written as exact divisions where they ARE exact: typing 83.3333 makes clean_rate() call it
# rough, because 250/83.3333 is 3.0000012 and the test is 1e-9. The rate really is 250/3, and
# a test that mislabels its own input would report a false positive against itself.
DEFAULT_RATES = (5.0, 10.0, 20.0, 25.0, 30.0, 40.0, 50.0, 55.0, 60.0,
                 62.5, 65.0, 70.0, 250.0 / 3.0, 90.0, 100.0, 120.0, 125.0)


class TickSampler(threading.Thread):
    """Shared-memory sampler that records WHICH loop tick each row came from.

    sc.Sampler does not, and without it two rows can be the same tick read twice (Python
    polls faster than the loop updates) or a hundred ticks apart (the GIL went elsewhere).
    Differencing either one fabricates a velocity. Recording the tick makes both detectable.
    """

    FIELDS = ("joint_position", "joint_position_commanded", "joint_torque")
    MAX_ROWS = 6000
    # row = (loop_tick, monotonic_time, joint_position, joint_position_commanded, joint_torque)

    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self.error = [], None
        self._stop, self._on = threading.Event(), False

    def run(self):
        last = None
        while not self._stop.is_set():
            if self._on:
                try:
                    with open(f"{sc.SHM}/loop_tick", "rb") as f:
                        tick = struct.unpack("d", f.read(8))[0]
                    if tick != last:
                        last = tick
                        self.rows.append((tick, time.monotonic())
                                         + tuple(sc.rd(f) for f in self.FIELDS))
                        if len(self.rows) > self.MAX_ROWS:
                            del self.rows[:len(self.rows) - self.MAX_ROWS]
                except Exception as exc:
                    self.error = self.error or repr(exc)
            time.sleep(sc.DT * 0.35)

    def start_window(self):
        self.rows = []
        self._on = True

    def end_window(self):
        self._on = False
        return list(self.rows)

    def stop(self):
        self._stop.set()


def stream_triangle(rti, q0, joints, amp, period, rate_hz, seconds, sampler):
    """Stream a triangle on `joints` and return (rows, achieved_hz, t0, leg_speed).

    Local rather than sc.stream because the analysis needs t0 -- without it there is no way
    to tell a constant-velocity leg from a reversal, and the reversals are exactly what must
    be excluded.
    """
    def tri(t):
        """Offset from q0. STARTS AND ENDS AT ZERO -- this is not cosmetic.

        The first version started at -amp, so the opening command of every rate ordered the
        arm a full amplitude away from where it was standing and it snapped there at the
        generator's limits. Seventeen rates, two jumps each. A test that measures smoothness
        must not begin by commanding a step.

        0 -> +amp -> -amp -> 0 over one period. The middle leg is twice the length of the two
        end legs, and it is the only one measured -- the end legs are half-length and would
        not reach steady state.
        """
        ph = (t / period) % 1.0
        if ph < 0.25:
            return amp * (4.0 * ph)                 # 0 -> +amp
        if ph < 0.75:
            return amp * (2.0 - 4.0 * ph)           # +amp -> -amp, the LONG leg
        return amp * (4.0 * ph - 4.0)               # -amp -> 0

    per = 1.0 / rate_hz
    sampler.start_window()
    sent = 0
    t0 = time.monotonic()
    nxt = t0
    while True:
        t = time.monotonic() - t0
        if t >= seconds:
            break
        off = tri(t)
        q = list(q0)
        for j in joints:
            q[j] = q0[j] + off
        rti.send_joint_position(q)
        sent += 1
        nxt += per
        slp = nxt - time.monotonic()
        if slp > 0:
            time.sleep(slp)
    # Walk whatever offset is left back to exactly zero at the leg speed, so the caller's
    # hold-at-q0 does not begin with a step either. Whole periods land near zero already;
    # at 5 Hz the last command can still sit a few degrees out.
    leg_speed = 4.0 * amp / period
    off = tri(time.monotonic() - t0)
    steps = int(abs(off) / max(leg_speed * per, 1e-9)) + 1
    for i in range(steps - 1, -1, -1):
        q = list(q0)
        for j in joints:
            q[j] = q0[j] + off * i / steps
        rti.send_joint_position(q)
        time.sleep(per)
    achieved = sent / max(time.monotonic() - t0, 1e-9)
    rows = sampler.end_window()
    return rows, achieved, t0, leg_speed


def measure(rows, joints, t0, period, leg_speed):
    """Per-joint (ripple%, lag deg, t_rms Nm) plus coverage. NEVER call while claimed."""
    n = len(rows)
    if n < 60:
        return None
    # Keep only the flat middle of each leg. 0.15-0.35 and 0.65-0.85 of the period sit
    # inside a leg with margin on both sides, so no reversal transient leaks in.
    def in_leg(t):
        # Only the LONG middle leg (phase 0.25-0.75), and only its flat centre. The two end
        # legs are half as long and would not reach steady state -- measuring them is what
        # made the first version of this test report no difference between any two rates.
        ph = ((t - t0) / period) % 1.0
        return 0.35 <= ph <= 0.65
    pairs = [(a, b) for a, b in zip(rows, rows[1:])
             if round(b[0] - a[0]) == 1 and in_leg(a[1]) and in_leg(b[1])]
    cover = 100.0 * len(pairs) / max(n - 1, 1)
    if len(pairs) < 40 or leg_speed <= 1e-9:
        return None
    out = {}
    for j in joints:
        # |v| because the two legs run in opposite directions; sd of |v| inside a leg is
        # exactly the off-hardware metric, which measured sd along a one-directional ramp.
        v = [abs((b[3][j] - a[3][j]) / sc.DT) for a, b in pairs]
        mean = sum(v) / len(v)
        sd = math.sqrt(sum((x - mean) ** 2 for x in v) / len(v))
        ripple = 100.0 * sd / leg_speed
        meas = [r[2][j] for r in rows]
        cmd = [r[3][j] for r in rows]
        trq = [r[4][j] for r in rows]
        lag = sc.deg(math.sqrt(sum((m - c) ** 2 for m, c in zip(meas, cmd)) / n))
        w = 13
        if len(trq) > w * 3:
            hft = [trq[i] - sum(trq[i - w:i + w + 1]) / (2 * w + 1)
                   for i in range(w, len(trq) - w)]
            trms = math.sqrt(sum(x * x for x in hft) / len(hft))
        else:
            trms = float("nan")
        out[j] = (ripple, lag, trms)
    return out, cover


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true", help="required: this MOVES the arm")
    p.add_argument("--rates", default="", help="comma list, default the full sweep")
    p.add_argument("--joints", default="3,4,5", help="J4,J5,J6. J2/J3 are refused")
    p.add_argument("--amp", type=float, default=15.0, help="degrees, half the triangle span")
    p.add_argument("--period", type=float, default=4.0,
                   help="seconds per full triangle. Each LEG is half of this and must be "
                        "long enough to reach steady state -- see MIN_LEG_S")
    p.add_argument("--seconds", type=float, default=16.0, help="per rate; keep it a whole "
                                                               "number of periods")
    p.add_argument("--gen", default="ruckig_advancing", help="generator under test")
    p.add_argument("--dump", default="", help="write the raw per-tick rows of the FIRST rate "
                                              "to this CSV and stop. For diagnosing a result "
                                              "that disagrees with the model.")
    a = p.parse_args()

    joints = [int(x) for x in a.joints.split(",") if x.strip() != ""]
    bad = [j for j in joints if j not in sc.SAFE_JOINTS]
    if bad:
        print(f"ABORT: joints {bad} are not in SAFE_JOINTS {sc.SAFE_JOINTS}.")
        print("       J2/J3 carry the gravity load and are excluded from servo tests.")
        return 1
    rates = ([float(x) for x in a.rates.split(",") if x.strip() != ""]
             if a.rates else list(DEFAULT_RATES))
    if a.period / 2.0 < MIN_LEG_S:
        print(f"ABORT: --period {a.period:g} gives a {a.period/2:.2f}s measured leg, under "
              f"the {MIN_LEG_S:g}s minimum.")
        print("       Short legs measure the reversal transient, which is the same at every")
        print("       rate, so the sweep would report no difference where one exists.")
        return 1
    amp = math.radians(a.amp)
    leg_speed = 4.0 * amp / a.period             # constant speed inside each triangle leg

    print("=" * 96)
    print("39: COMMAND-RATE SWEEP — is a rate smooth, and does it depend on dividing 250?")
    print("=" * 96)
    print(f"  generator {a.gen}   joints {[JN[j] for j in joints]}   "
          f"triangle 0 -> +/-{a.amp:g} deg over {a.period:g}s   "
          f"{sc.deg(leg_speed):.1f} deg/s")
    print(f"  starts and ends at the pose it claims -- no step at either end")
    print(f"  {len(rates)} rates x {a.seconds:g}s -> about {len(rates)*(a.seconds+3)/60:.1f} min\n")
    if not a.go:
        print("Refusing without --go. This MOVES the arm. Clear the workspace, keep the")
        print("E-stop in reach, and confirm the arm is homed and ENABLED.")
        return 1

    smp = TickSampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
    results, failures = {}, {}
    try:
        why = sc.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 1

        for hz in rates:
            tag = "expect clean" if predicted_clean(hz) else "expect ROUGH"
            print(f"  {hz:8.4g} Hz  ({sc.LOOP_HZ/hz:6.3f} ticks/cmd, {tag}) ... ",
                  end="", flush=True)
            rti = RTIClient(robot, clock="pc")
            try:
                # One session PER RATE. The rate is fixed by the contract at connect time,
                # and re-negotiating it under a live claim would change the timeout out from
                # under a streaming PC. Releasing between rates also means the analysis below
                # runs UNCLAIMED, where taking a few hundred ms costs nothing -- inside a
                # claim it would be a gap, and the watchdog would be right to stop the arm.
                rti.connect(rate_hz=hz)
                time.sleep(0.4)
                # Configure BEFORE claiming. apply_config is five TCP round trips, and a
                # claimed session has no idle state -- those round trips would be a gap the
                # watchdog is entitled to stop the arm for.
                sc.apply_config(robot, {"gen": a.gen})
                rti.read_once(timeout=2.0)
                rti.claim()
                q0 = rti.read_once(timeout=1.0)["j_p"]
                sc.keep_alive(rti, q0, hz, 0.6)          # settle, still streaming
                rows, achieved, t0, leg = stream_triangle(
                    rti, q0, joints, amp, a.period, hz, a.seconds, smp)
                sc.keep_alive(rti, q0, hz, 0.6)          # return to rest, still streaming
                rti.release()
            except ControlLost as exc:
                failures[hz] = f"CONTROL LOST: {exc}"
                print("CONTROL LOST")
                try:
                    robot.enter_idle_mode()
                except Exception:
                    pass
                continue
            if smp.error:
                print(f"\n*** sampler failed: {smp.error}")
                return 1
            got = measure(rows, joints, t0, a.period, leg)
            if got is None:
                failures[hz] = f"not enough in-leg samples from {len(rows)} rows"
                print(f"too few in-leg samples ({len(rows)} rows)")
                continue
            m, cover = got
            results[hz] = (m, cover, achieved)
            if a.dump:
                with open(a.dump, "w") as f:
                    f.write("tick,t_rel,phase," + ",".join(
                        f"{JN[j]}_meas,{JN[j]}_cmd" for j in joints) + "\n")
                    for r in rows:
                        ph = ((r[1] - t0) / a.period) % 1.0
                        f.write(f"{r[0]:.0f},{r[1]-t0:.6f},{ph:.6f}," + ",".join(
                            f"{r[2][j]:.9f},{r[3][j]:.9f}" for j in joints) + "\n")
                print(f"\n  wrote {len(rows)} rows to {a.dump} — stopping after one rate.")
                sc.clear_config(robot); robot.enter_idle_mode()
                return 0
            print(f"sent {achieved:6.2f} Hz, {len(rows)} rows, {cover:.0f}% adjacent")
        sc.clear_config(robot); robot.enter_idle_mode()
    except ControlLost as exc:
        print(f"\n*** CONTROL LOST: {exc}")
        return 1
    finally:
        smp.stop()
        try:
            sc.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    if not results:
        print("\nNo rate produced usable data.")
        return 1

    print("\n" + "=" * 96)
    print("RESULTS — rip% is sd of commanded velocity inside a constant-velocity leg")
    print("(the generator's own roughness); lag is rms |measured - commanded|.")
    print("Prediction: 'clean' rates near 0, 'rough' (55/65/70/90 Hz) markedly higher.")
    print("'clean' here means the gap period divides GAP_WINDOW=12 -- NOT that 250/rate is")
    print("a whole number. 20/30/40/60/100/120 Hz are non-integer AND predicted clean.")
    print("=" * 96)
    print(f"{'':>9} {'':>7} {'ratio':>7}")
    hdr = "".join(f"{JN[j]+' rip%':>10}{JN[j]+' lag':>9}" for j in joints)
    print(f"{'rate':>9} {'ticks':>7} {'expect':>7} {'cover':>6} |{hdr}{'t_rms':>9}")
    for hz in sorted(results):
        m, cover, _ = results[hz]
        kind = "clean" if predicted_clean(hz) else "rough"
        cells = "".join(f"{m[j][0]:10.3f}{m[j][1]:9.3f}" for j in joints)
        trms = max(m[j][2] for j in joints)
        ncmd = hz * 0.30 * a.period          # commands inside the measured window
        warn = ""
        if cover < 8:
            warn = "  <-- thin sample"
        elif ncmd < MIN_CMDS_IN_WINDOW:
            warn = f"  <-- only {ncmd:.0f} cmds in window, not settled"
        print(f"{hz:9.4g} {sc.LOOP_HZ/hz:7.3f} {kind:>7} {cover:5.0f}% |{cells}{trms:9.4f}{warn}")

    # Rates whose window holds too few commands are excluded from the verdict entirely --
    # they measure settling, not the rate, and would drag the clean median up.
    ok = [h for h in results if h * 0.30 * a.period >= MIN_CMDS_IN_WINDOW]
    cl = [max(results[h][0][j][0] for j in joints) for h in ok if predicted_clean(h)]
    ro = [max(results[h][0][j][0] for j in joints) for h in ok if not predicted_clean(h)]
    skipped = sorted(h for h in results if h not in ok)
    if skipped:
        print(f"\n  excluded from the verdict (window too short at these rates): "
              + ", ".join(f"{h:g}" for h in skipped) + " Hz")
        print(f"  raise --period to judge them; they are not evidence either way.")
    print("\n" + "-" * 96)
    if cl:
        print(f"  clean rates: worst-joint ripple  median {sorted(cl)[len(cl)//2]:6.3f}%   "
              f"max {max(cl):6.3f}%   ({len(cl)} rates)")
    if ro:
        print(f"  rough rates: worst-joint ripple  median {sorted(ro)[len(ro)//2]:6.3f}%   "
              f"max {max(ro):6.3f}%   ({len(ro)} rates)")
    if cl and ro:
        mc, mr = sorted(cl)[len(cl)//2], sorted(ro)[len(ro)//2]
        if mr > mc * 3.0:
            print(f"\n  CONFIRMED: rough rates are {mr/max(mc,1e-9):.1f}x rougher. The "
                  f"integer-ratio rule holds on hardware.")
        else:
            print(f"\n  NOT REPRODUCED: rough rates are only {mr/max(mc,1e-9):.1f}x the clean "
                  f"median. Either the\n  arm's own dynamics dominate at this speed, or the "
                  f"off-hardware model is wrong.\n  Re-run faster (--freq 1.0 --amp 10) "
                  f"before concluding either way.")
    if failures:
        print("\n  rates that did not complete:")
        for hz, why in sorted(failures.items()):
            print(f"    {hz:8.4g} Hz  {why}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
