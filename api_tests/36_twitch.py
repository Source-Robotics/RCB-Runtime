#!/usr/bin/env python3
"""36: why is it twitchy at 10-20 Hz but not 25-50? Measure the SIZE of the step, not the count.

The reversal metric counts direction changes and says 10 Hz is BETTER than 50 Hz for
ruckig_advancing (182 vs 412). The operator reports the opposite: twitchy at 10-20 Hz,
smooth at 25-50. Both cannot be right, and the metric is the thing to distrust -- it treats
a 0.001 deg wobble and a 0.1 deg jerk as one event each.

THE HYPOTHESIS. ruckig_advancing aims at `target + v_tgt*age`. When a command lands, `age`
resets to zero and `target` jumps forward one interval; any mismatch leaves a step. The
NUMBER of steps scales with the command rate, but the SIZE of each one scales with the GAP,
because that is how long the extrapolation ran before being corrected:

    10 Hz -> 25 ticks -> 100 ms of extrapolation per correction
    50 Hz ->  5 ticks ->  20 ms

So low rates should give FEWER but MUCH BIGGER steps. Few large twitches are audible and
visible; many tiny ones are not. A count-based metric inverts the ranking.

This measures magnitude directly, from the commanded signal at loop rate:

    dv_max   largest single-tick change in commanded VELOCITY (deg/s). One tick is 4 ms,
             so this is the size of the discontinuity the motors are asked to absorb.
    dv_p99   99th percentile of the same -- the typical big one, not the single worst.
    a_max    peak commanded acceleration (deg/s^2).
    rev      the old count, for comparison.

    python3 api_tests/36_twitch.py --host <ip> --go
    python3 api_tests/36_twitch.py --host <ip> --go --rates 5,10,20,25,50,125
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost


def twitch(rows, j):
    """Magnitude of the commanded discontinuities, in deg/s of one-tick velocity change."""
    cmd = [r[1][j] for r in rows]
    uniq = [cmd[0]]
    for v in cmd[1:]:
        if v != uniq[-1]:
            uniq.append(v)
    if len(uniq) < 5:
        return (float("nan"),) * 4
    vel = [(b - a) / sc.DT for a, b in zip(uniq, uniq[1:])]
    dv = sorted(abs(b - a) for a, b in zip(vel, vel[1:]))
    if not dv:
        return (float("nan"),) * 4
    d = [b - a for a, b in zip(uniq, uniq[1:])]
    rev = sum(1 for x, y in zip(d, d[1:]) if x * y < 0)
    a_max = max(abs(x) for x in (b - a for a, b in zip(vel, vel[1:]))) / sc.DT
    return (sc.deg(dv[-1]), sc.deg(dv[int(len(dv) * 0.99)]), sc.deg(a_max), rev)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true")
    p.add_argument("--joint", type=int, default=5)
    p.add_argument("--amp", type=float, default=4.0)
    p.add_argument("--freq", type=float, default=0.5)
    p.add_argument("--rates", default="5,10,20,25,50,125,250")
    p.add_argument("--configs", default="ruckig,advancing,advancing+mindur,tracker")
    p.add_argument("--seconds", type=float, default=5.0)
    a = p.parse_args()

    if not a.go:
        print("Refusing without --go: this MOVES the arm.")
        return 1
    if a.joint not in sc.SAFE_JOINTS:
        print(f"REFUSED: --joint must be within {list(sc.SAFE_JOINTS)}.")
        return 1
    rates = [float(x) for x in a.rates.split(",")]
    cfgs = a.configs.split(",")
    amp = math.radians(a.amp)
    j = a.joint

    print("=" * 100)
    print(f"TWITCH MAGNITUDE   J{j+1}   +/-{a.amp:g} deg @ {a.freq:g} Hz "
          f"({a.amp*2*math.pi*a.freq:.1f} deg/s peak)")
    print("=" * 100)
    print("dv_max/dv_p99: biggest one-tick change in COMMANDED velocity (deg/s).")
    print("That is the discontinuity the motors must absorb in 4 ms. rev is the old count.\n")
    print(f"{'rate':>7} {'ticks/cmd':>10} {'config':>18} {'dv_max':>9} {'dv_p99':>9} "
          f"{'a_max':>11} {'rev':>6}")
    print("-" * 82)

    smp = sc.Sampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
    res = {}
    try:
        why = sc.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 1
        for rate in rates:
            ticks = sc.LOOP_HZ / rate
            rti = RTIClient(robot, clock="pc")
            rti.connect(rate_hz=rate); time.sleep(0.4)
            rti.read_once(timeout=2.0)
            rti.claim()
            q0 = rti.read_once(timeout=1.0)["j_p"]
            for name in cfgs:
                sc.apply_config(robot, sc.CONFIGS[name])
                sc.keep_alive(rti, q0, rate, 0.4)
                shape = lambda t: {j: amp * math.sin(2 * math.pi * a.freq * t)}
                rows, _ = sc.stream(rti, q0, shape, rate, a.seconds, smp)
                if smp.error:
                    print(f"\n*** sampler failed: {smp.error}")
                    return 1
                dv, p99, am, rev = twitch(rows, j)
                res[(rate, name)] = (dv, p99, am, rev)
                mark = "" if abs(ticks - round(ticks)) < 1e-9 else "  (non-integer!)"
                print(f"{rate:6.0f}H {ticks:10.1f} {name:>18} {dv:9.2f} {p99:9.2f} "
                      f"{am:11.0f} {rev:6.0f}{mark}")
                sc.keep_alive(rti, q0, rate, 0.6)
            rti.release(); sc.clear_config(robot); robot.enter_idle_mode(); time.sleep(0.3)
    except ControlLost as exc:
        print(f"\n*** CONTROL LOST: {exc}")
        return 1
    finally:
        smp.stop()
        try:
            sc.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    print("\n" + "=" * 100)
    print("PEAK ONE-TICK VELOCITY STEP (deg/s) — the size of the twitch")
    print("=" * 100)
    print(f"{'rate':>7} " + "".join(f"{c:>20}" for c in cfgs))
    for rate in rates:
        print(f"{rate:6.0f}H " + "".join(
            f"{res[(rate,c)][0]:20.2f}" if (rate, c) in res else f"{'-':>20}" for c in cfgs))
    print("\nREVERSAL COUNT (the old metric, for comparison)")
    print(f"{'rate':>7} " + "".join(f"{c:>20}" for c in cfgs))
    for rate in rates:
        print(f"{rate:6.0f}H " + "".join(
            f"{res[(rate,c)][3]:20.0f}" if (rate, c) in res else f"{'-':>20}" for c in cfgs))
    print("\nIf dv_max FALLS as the rate rises while rev RISES, the count metric was")
    print("inverting the ranking and magnitude is what the operator was hearing.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
