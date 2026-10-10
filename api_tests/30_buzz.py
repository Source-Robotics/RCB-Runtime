#!/usr/bin/env python3
"""30: measure the buzz directly, as motor-current oscillation vs command rate.

The operator reports more vibration and audible buzz as the command rate rises. That is the
opposite of what simulation predicted, and neither of the obvious explanations survived
measurement: the PC's own pacing is good (0.06 ms stdev at 250 Hz) and the link is clean
(99% success, 1.2% discard at 250 Hz).

So measure the thing itself. Vibration IS current oscillation -- the motor fighting a
setpoint that keeps changing direction. This streams the same motion at each rate and
reports, per joint:

    i_rms     rms of the current's HIGH-FREQUENCY content (current minus its own moving
              average). Slow current changes are the motion; fast ones are the buzz.
    i_pp      peak-to-peak of that high-frequency component.
    cmd_rev   how often the COMMANDED position reverses direction. A setpoint that
              stutters back and forth is the most likely thing a motor turns into noise,
              and it is invisible in an rms tracking-lag number.

Jerk is deliberately NOT reported here: sampling /dev/shm from a Python thread drops samples
often enough that a third difference is unreliable, which is why 29's jerk column reads
above the arm's own 3000 limit and should not be trusted. Current and reversals need no
differentiation.

    python3 api_tests/30_buzz.py --host 127.0.0.1
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


def rd(name, n=6):
    with open(f"{SHM}/{name}", "rb") as f:
        return list(struct.unpack(f"{n}d", f.read(8 * n)))


class Sampler(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self._stop, self._on = [], threading.Event(), False
        self.error = None

    def run(self):
        while not self._stop.is_set():
            if self._on:
                try:
                    self.rows.append((rd("joint_torque"), rd("joint_position_commanded")))
                except Exception as exc:
                    # NEVER swallow this. The first version read "motor_current", which is
                    # 3 doubles not 6, so every read raised, the bare except ate it, and the
                    # run reported all-NaN -- indistinguishable from a real measurement.
                    self.error = self.error or repr(exc)
            time.sleep(0.002)

    def start_window(self):
        self.rows = []
        self._on = True

    def end_window(self):
        self._on = False
        return list(self.rows)

    def stop(self):
        self._stop.set()


def hf_stats(series, win=13):
    """rms and peak-to-peak of the signal minus its own moving average."""
    if len(series) < win * 3:
        return float("nan"), float("nan")
    hf = []
    for i in range(win, len(series) - win):
        avg = sum(series[i - win:i + win + 1]) / (2 * win + 1)
        hf.append(series[i] - avg)
    rms = math.sqrt(sum(v * v for v in hf) / len(hf))
    return rms, max(hf) - min(hf)


def reversals(series):
    """How many times the signal changes direction (per second of samples)."""
    d = [b - a for a, b in zip(series, series[1:]) if b != a]
    if len(d) < 3:
        return 0.0
    return sum(1 for a, b in zip(d, d[1:]) if a * b < 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--joint", type=int, default=5)
    ap.add_argument("--amp", type=float, default=3.0)
    ap.add_argument("--freq", type=float, default=0.25)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--gen", default="ruckig_advancing")
    ap.add_argument("--rates", default="5,25,50,125,250")
    ap.add_argument("--configs", default="",
                    help="sweep CONFIGS instead of rates (needs --rate-fixed)")
    ap.add_argument("--rate-fixed", type=float, default=50.0,
                    help="rate to hold while sweeping configs or speeds")
    ap.add_argument("--freqs", default="",
                    help="sweep SPEED instead: comma list of sine frequencies in Hz")
    a = ap.parse_args()

    # Build the cases. Exactly ONE axis varies per run, so a difference has one cause.
    from importlib import import_module
    _sweep = import_module("api_tests.29_servo_sweep") if False else None
    CONFIGS = {
        "ruckig":           dict(gen="ruckig"),
        "tracker":          dict(gen="tracker"),
        "advancing":        dict(gen="ruckig_advancing"),
        "ruckig+vel25":     dict(gen="ruckig", vel_scale=0.25),
        "ruckig+pace1.5":   dict(gen="ruckig", pace=1.5),
        "ruckig+mindur1.5": dict(gen="ruckig", min_duration=1.5),
        "advancing+mindur": dict(gen="ruckig_advancing", min_duration=1.5),
        "tracker+gain30":   dict(gen="tracker", gain=30.0),
    }
    if a.configs:
        names = a.configs.split(",")
        bad = [n for n in names if n not in CONFIGS]
        if bad:
            print(f"unknown config(s): {bad}. Known: {', '.join(CONFIGS)}")
            return 1
        cases = [(f"{n}", a.rate_fixed, CONFIGS[n], a.freq) for n in names]
        axis = "CONFIG"
    elif a.freqs:
        cases = [(f"{float(f):g}Hz sine", a.rate_fixed, {"gen": a.gen}, float(f))
                 for f in a.freqs.split(",")]
        axis = "SPEED"
    else:
        cases = [(f"{float(r):g} Hz cmd", float(r), {"gen": a.gen}, a.freq)
                 for r in a.rates.split(",")]
        axis = "COMMAND RATE"

    amp = math.radians(a.amp)
    print("=" * 84)
    print(f"BUZZ MEASUREMENT — varying {axis}")
    print(f"  joint {a.joint}  +/-{a.amp:g} deg   {a.seconds:g}s per case")
    if axis != "COMMAND RATE":
        print(f"  command rate held at {a.rate_fixed:g} Hz")
    if axis != "SPEED":
        print(f"  sine {a.freq:g} Hz  ->  peak speed "
              f"{a.amp * 2 * math.pi * a.freq:.2f} deg/s")
    print("=" * 84)
    print("t_rms/t_pp: high-frequency joint TORQUE (Nm) = measured current x Kt.")
    print("cmd_rev: commanded direction changes. A stuttering setpoint shows up here.\n")
    print(f"{'case':>20} {'t_rms':>9} {'t_pp':>9} {'cmd_rev':>9} {'pk deg/s':>10}")
    print("-" * 62)

    smp = Sampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
    out = {}
    try:
        for label, rate, cfg, freq in cases:
            period = 1.0 / rate
            rti = RTIClient(robot, clock="pc")
            rti.connect(rate_hz=rate); time.sleep(0.4)
            rti.read_once(timeout=2.0)
            rti.claim()
            q0 = rti.read_once(timeout=1.0)["j_p"]
            # Every knob every case: pacing is sticky and would contaminate the next case.
            robot.rti_set_vel_scale(cfg.get("vel_scale", 1.0))
            robot.rti_set_pace(cfg.get("pace", 0.0))
            robot.rti_set_min_duration(cfg.get("min_duration", 0.0))
            robot.rti_set_servo_generator(cfg["gen"], gain=cfg.get("gain"),
                                          self_lookahead=cfg.get("self_lookahead", 0.0))
            end = time.monotonic() + 0.4
            while time.monotonic() < end:          # never silent while claimed
                rti.send_joint_position(q0); time.sleep(period)

            t0, nxt = time.monotonic(), time.monotonic()
            smp.start_window()
            while time.monotonic() - t0 < a.seconds:
                t = time.monotonic() - t0
                q = list(q0)
                q[a.joint] = q0[a.joint] + amp * math.sin(2 * math.pi * freq * t)
                rti.send_joint_position(q)
                nxt += period
                slp = nxt - time.monotonic()
                if slp > 0:
                    time.sleep(slp)
            rows = smp.end_window()
            if smp.error:
                print(f"\n*** sampler failed: {smp.error}")
                print("    No data collected. Fix this before reading anything above.")
                return 1
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rti.send_joint_position(q0); time.sleep(period)
            rti.release()
            robot.rti_set_vel_scale(1.0); robot.rti_set_pace(0.0)
            robot.rti_set_min_duration(0.0)
            robot.enter_idle_mode(); time.sleep(0.4)

            trq = [r[0][a.joint] for r in rows]
            cmd = [r[1][a.joint] for r in rows]
            trms, tpp = hf_stats(trq)
            rev = reversals(cmd)
            out[label] = (trms, tpp, rev)
            print(f"{label:>20} {trms:9.4f} {tpp:9.4f} {rev:8.0f} "
                  f"{a.amp * 2 * math.pi * freq:9.2f}")
    except ControlLost as exc:
        print(f"\n*** CONTROL LOST: {exc}")
    finally:
        smp.stop()
        try:
            robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    if len(out) > 1:
        worst = max(out, key=lambda k: out[k][0])
        best = min(out, key=lambda k: out[k][0])
        print(f"\n  quietest: {best:>20}  t_rms {out[best][0]:.4f} Nm")
        print(f"  loudest : {worst:>20}  t_rms {out[worst][0]:.4f} Nm  "
              f"({out[worst][0]/max(out[best][0],1e-9):.1f}x)")
        print("\n  If cmd_rev tracks t_rms, the SETPOINT is stuttering and the motor is")
        print("  following it faithfully -- our problem. If not, it is the motor's response.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
