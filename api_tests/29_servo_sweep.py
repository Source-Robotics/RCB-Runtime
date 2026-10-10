#!/usr/bin/env python3
"""29: sweep generator x command rate x speed on the real arm.

Everything in TODO/servo_mode_options.md 13.6 came from simulation against a perfect plant.
This is the same matrix on metal.

WHY IT LOGS SHARED MEMORY. Smoothness is a property of the 250 Hz commanded signal, so it
has to be sampled at 250 Hz. Reading the UDP state stream at the PC's own rate samples the
output more slowly than it changes -- at 5 Hz you would see 50 ticks of motion as one point
and every generator would look identical. The sweep therefore logs joint_position and
joint_position_commanded straight from /dev/shm in a background thread, and reports:

    jerk   peak |3rd difference| of the COMMANDED signal, rad/s^3. This is the number the
           simulation tables predict. The arm's jerk limit is 3000; above that the limiter
           is saturating.
    lag    rms |measured - commanded|, degrees. Tracking accuracy.
    rate   the rate the PC ACTUALLY achieved. Python + time.sleep() cannot hold 250 Hz
           reliably, so a requested rate that was not met makes its row meaningless --
           which is why it is measured rather than assumed.

WHAT IT DOES TO THE ROBOT: sines one joint at +/- --amp degrees, once per combination.
Default 3 deg on the wrist. Hand on the E-stop.

    python3 api_tests/29_servo_sweep.py --host 127.0.0.1
    python3 api_tests/29_servo_sweep.py --host 127.0.0.1 --amp 8 --freq 0.5
    python3 api_tests/29_servo_sweep.py --host 127.0.0.1 --joint 1 --amp 5   # shoulder
"""
import argparse
import math
import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robotclient import RobotClient, RTIClient, ControlLost

SHM = "/dev/shm"
LOOP_HZ = 250.0
DT = 1.0 / LOOP_HZ
# A "config" is a generator PLUS its pacing, because the generator alone is not the whole
# choice. Pacing is Ruckig-only (the tracker's pacing is its bandwidth), so the tracker rows
# vary its own knobs instead.
#
# Every field is applied on every cell, defaults included -- pacing is sticky, so a cell that
# does not name min_duration must actively clear it or it inherits the previous cell's and
# the comparison is silently contaminated.
CONFIGS = {
    # --- plain generators -------------------------------------------------
    "ruckig":            dict(gen="ruckig"),
    "tracker":           dict(gen="tracker"),
    "advancing":         dict(gen="ruckig_advancing"),
    # --- Ruckig + each pacing mechanism -----------------------------------
    "ruckig+vel25":      dict(gen="ruckig", vel_scale=0.25),      # velocity-ceiling model
    "ruckig+pace1.5":    dict(gen="ruckig", pace=1.5),            # self-tuning ceiling
    "ruckig+mindur1.5":  dict(gen="ruckig", min_duration=1.5),    # duration floor
    # --- the combinations simulation says are the good ones ---------------
    "advancing+mindur":  dict(gen="ruckig_advancing", min_duration=1.5),
    "advancing+vel25":   dict(gen="ruckig_advancing", vel_scale=0.25),
    "tracker+self":      dict(gen="tracker", self_lookahead=0.0667),   # damping-style lookahead
    "tracker+gain30":    dict(gen="tracker", gain=30.0),
}
DEFAULT_CONFIGS = ("ruckig", "tracker", "advancing", "ruckig+vel25",
                   "ruckig+mindur1.5", "advancing+mindur")
# Clean ratios only: LOOP_HZ / n for integer n, so commands land the same number of ticks
# apart every time. A non-integer ratio corrupts every velocity estimate derived from the
# stream and would confound the comparison.
RATES = (5.0, 10.0, 25.0, 50.0, 125.0, 250.0)


def rd(name, n=6):
    with open(f"{SHM}/{name}", "rb") as f:
        return list(struct.unpack(f"{n}d", f.read(8 * n)))


def deg(r):
    return r * 180.0 / math.pi


class Sampler(threading.Thread):
    """Log measured + commanded joint positions at the loop rate."""

    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self._stop, self._on = [], threading.Event(), False

    def run(self):
        while not self._stop.is_set():
            if self._on:
                try:
                    self.rows.append((time.monotonic(), rd("joint_position"),
                                      rd("joint_position_commanded")))
                except Exception:
                    pass
            time.sleep(DT * 0.5)          # oversample; duplicates are harmless

    def start_window(self):
        self.rows = []
        self._on = True

    def end_window(self):
        self._on = False
        return list(self.rows)

    def stop(self):
        self._stop.set()


def analyse(rows, j):
    """(peak commanded jerk rad/s^3, rms lag deg, peak measured deg-from-start)."""
    if len(rows) < 20:
        return float("nan"), float("nan"), float("nan"), 0
    t = [r[0] for r in rows]
    meas = [r[1][j] for r in rows]
    cmd = [r[2][j] for r in rows]
    dt = (t[-1] - t[0]) / max(1, len(t) - 1)
    # Drop duplicate samples: the sampler oversamples, and repeated values would read as
    # zero jerk and flatter every generator equally.
    uniq, ut = [cmd[0]], [t[0]]
    for tv, v in zip(t[1:], cmd[1:]):
        if v != uniq[-1]:
            uniq.append(v)
            ut.append(tv)
    jerk = 0.0
    if len(uniq) > 4:
        # Spacing from the TIMESTAMPS of the unique samples, not from DT. Deduplicating an
        # oversampled signal leaves samples that are only APPROXIMATELY DT apart, and a
        # third difference divided by the wrong dt cubed is wrong by that ratio cubed --
        # which is how the first run reported jerk far above the arm's own 3000 limit.
        gaps = sorted(ut[i] - ut[i - 1] for i in range(1, len(ut)))
        step = gaps[len(gaps) // 2]
        if step > 1e-6:
            for i in range(3, len(uniq)):
                d3 = uniq[i] - 3 * uniq[i - 1] + 3 * uniq[i - 2] - uniq[i - 3]
                jerk = max(jerk, abs(d3) / (step ** 3))
    lag = math.sqrt(sum((m - c) ** 2 for m, c in zip(meas, cmd)) / len(meas))
    peak = max(abs(m - meas[0]) for m in meas)
    # Direction changes in the COMMANDED signal. Needs no differentiation, so unlike the
    # jerk column it is trustworthy -- and it is what exposed ruckig_advancing stepping at
    # every command boundary (94 reversals on a motion that has 2).
    d = [b - a for a, b in zip(uniq, uniq[1:]) if b != a]
    rev = sum(1 for x, y in zip(d, d[1:]) if x * y < 0)
    return jerk, deg(lag), deg(peak), rev


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--joint", type=int, default=5)
    ap.add_argument("--amp", type=float, default=3.0, help="degrees")
    ap.add_argument("--freq", type=float, default=0.25, help="Hz of the sine")
    ap.add_argument("--seconds", type=float, default=4.0, help="per combination")
    ap.add_argument("--rates", type=str, default="",
                    help="comma list, default 5,10,25,50,125,250")
    ap.add_argument("--configs", type=str, default="",
                    help="comma list of config names; --configs list to see them all")
    a = ap.parse_args()

    rates = [float(x) for x in a.rates.split(",")] if a.rates else list(RATES)
    if a.configs.strip() == "list":
        print("Available configs:\n")
        for k, v in CONFIGS.items():
            extras = ", ".join(f"{kk}={vv}" for kk, vv in v.items() if kk != "gen")
            print(f"  {k:20} {v['gen']:18} {extras}")
        print(f"\nDefault set: {', '.join(DEFAULT_CONFIGS)}")
        return 0
    gens = a.configs.split(",") if a.configs else list(DEFAULT_CONFIGS)
    unknown = [g for g in gens if g not in CONFIGS]
    if unknown:
        print(f"unknown config(s): {unknown}. Use --configs list")
        return 1
    amp = math.radians(a.amp)
    peak_speed = amp * 2 * math.pi * a.freq

    print("=" * 96)
    print(f"SERVO SWEEP   joint {a.joint}  +/-{a.amp:g} deg @ {a.freq:g} Hz "
          f"(peak {deg(peak_speed):.2f} deg/s)  {a.seconds:g}s per cell")
    print("=" * 96)
    for r in rates:
        tk = LOOP_HZ / r
        if abs(tk - round(tk)) > 1e-9:
            print(f"  refusing {r:g} Hz — {tk:.2f} ticks/command is not an integer ratio")
            return 1
    print(f"  rates: {', '.join(f'{r:g}' for r in rates)} Hz")
    print(f"  generators: {', '.join(gens)}\n")

    sampler = Sampler()
    sampler.start()
    robot = RobotClient()
    robot.connect(a.host)
    results = {}
    try:
        st = robot.status()
        if st.get("state") != "ENABLED" or not st.get("homed"):
            print(f"ABORT: need ENABLED+homed, got state={st.get('state')} "
                  f"homed={st.get('homed')}")
            return 1

        for rate in rates:
            period = 1.0 / rate
            rti = RTIClient(robot, clock="pc")
            rti.connect(rate_hz=rate)
            time.sleep(0.4)
            q0 = rti.read_once(timeout=2.0)["j_p"]
            rti.claim()
            # Re-read after the claim: a pose captured before it is stale, and commanding
            # a stale pose is a real step (see 27/28 and the read_once fix).
            q0 = rti.read_once(timeout=1.0)["j_p"]

            def keep_alive(seconds):
                """Stream the hold pose. NEVER go silent while holding the claim.

                The first version of this script slept 0.3 s between generators. At 10 Hz
                the command timeout is 3/10 = 0.3 s, so the sleep plus the TCP round-trip
                blew through it and the watchdog latched RTI_LINK_LOST -- correctly. A
                claimed session has no idle state: if you are not streaming, you are dead.
                """
                end, nxt = time.monotonic() + seconds, time.monotonic()
                while time.monotonic() < end:
                    rti.send_joint_position(q0)
                    nxt += period
                    slp = nxt - time.monotonic()
                    if slp > 0:
                        time.sleep(slp)

            for gen in gens:
                cfg = CONFIGS[gen]
                # Announce BEFORE running. Results printed after a 5 s cell cannot be
                # matched to a sound you heard during it; this line can.
                print(f"  >> {rate:6.0f} Hz  {gen:18s} ...", end="", flush=True)
                # EVERY knob, EVERY cell. Pacing is sticky: a cell that does not name
                # min_duration must clear it, or it silently inherits the previous cell's
                # and the whole comparison is contaminated.
                robot.rti_set_vel_scale(cfg.get("vel_scale", 1.0))
                robot.rti_set_pace(cfg.get("pace", 0.0))
                robot.rti_set_min_duration(cfg.get("min_duration", 0.0))
                robot.rti_set_servo_generator(
                    cfg["gen"],
                    gain=cfg.get("gain"),
                    lookahead=cfg.get("lookahead"),
                    self_lookahead=cfg.get("self_lookahead", 0.0))
                # The TCP round-trips are well inside one command period; what is not is
                # waiting afterwards, so the wait streams.
                keep_alive(0.4)
                sent, t0 = 0, time.monotonic()
                nxt = t0
                sampler.start_window()
                while True:
                    t = time.monotonic() - t0
                    if t >= a.seconds:
                        break
                    q = list(q0)
                    q[a.joint] = q0[a.joint] + amp * math.sin(2 * math.pi * a.freq * t)
                    rti.send_joint_position(q)
                    sent += 1
                    nxt += period
                    slp = nxt - time.monotonic()
                    if slp > 0:
                        time.sleep(slp)
                rows = sampler.end_window()
                achieved = sent / (time.monotonic() - t0)
                results[(rate, gen)] = analyse(rows, a.joint) + (achieved,)
                jk, lag, pk, rev, ach = results[(rate, gen)]
                keep_alive(0.6)          # settle back to the start pose
                print(f"\r  {rate:6.0f} Hz  {gen:18s} lag {lag:6.3f} deg  "
                      f"peak {pk:5.2f} deg  rev {rev:5.0f}  achieved {ach:6.1f} Hz    ")
            rti.release()
            # Leave nothing sticky behind for the next rate block or the next operator.
            robot.rti_set_vel_scale(1.0)
            robot.rti_set_pace(0.0)
            robot.rti_set_min_duration(0.0)
            robot.enter_idle_mode()
            time.sleep(0.4)
    except ControlLost as exc:
        print(f"\n*** CONTROL LOST: {exc}")
        print("    The robot stopped itself because the PC went quiet while claimed.")
        print("    Clear errors and re-enable before re-running.")
        return 1
    except Exception as exc:
        print(f"\n*** {type(exc).__name__}: {exc}")
        print("    Partial results below; the robot may need clear_errors + enable.")
    finally:
        sampler.stop()
        try:
            robot.enter_idle_mode()
            robot.disconnect()
        except Exception:
            pass

    print("\n" + "=" * 96)
    print("COMMANDED DIRECTION REVERSALS — the motion itself has 2. More = a stuttering setpoint")
    print("=" * 96)
    print(f"{'rate':>8} " + "".join(f"{g:>19}" for g in gens))
    for rate in rates:
        row = "".join(f"{results[(rate,g)][3]:19.0f}" if (rate, g) in results else f"{'-':>19}"
                      for g in gens)
        print(f"{rate:7.0f}Hz {row}")
    print("\n" + "=" * 96)
    print("RMS TRACKING LAG (deg) — lower is more accurate")
    print("=" * 96)
    print(f"{'rate':>8} " + "".join(f"{g:>19}" for g in gens))
    for rate in rates:
        row = "".join(f"{results[(rate,g)][1]:19.3f}" if (rate, g) in results else f"{'-':>19}"
                      for g in gens)
        print(f"{rate:7.0f}Hz {row}")
    print("\nACHIEVED PC RATE — a row well below its requested rate is not a valid comparison")
    for rate in rates:
        got = [results[(rate, g)][4] for g in gens if (rate, g) in results]
        if got:
            print(f"  requested {rate:6.0f} Hz -> achieved {sum(got)/len(got):6.1f} Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
