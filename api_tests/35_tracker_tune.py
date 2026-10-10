#!/usr/bin/env python3
"""35: tune the tracker on hardware — gain x lookahead x self_lookahead.

The tracker's defaults came from simulation against a perfect plant: gain 15 rad/s chosen
from a sweep, lookahead derived as 2/gain, self_lookahead defaulted OFF because at gain 15
it cost more tracking than the overshoot it bought back.

Hardware has since shown that the plant is NOT perfect -- the arm's own servo lag is 0.90
deg at every command rate (servo_mode_options.md 14.2), which is the thing the tracker's
bandwidth has to work against. The simulated optimum has no particular reason to survive
that, so this re-runs the sweep on metal.

Three axes, one at a time so a difference has one cause:

    --sweep gain    bandwidth. Higher tracks tighter and passes more noise. Above ~20 a
                    large step overshoots (in simulation -- unverified here).
    --sweep look    the TARGET lookahead. Cancels lag, CAUSES stop-overshoot. Default 2/gain.
    --sweep self    damping-style lookahead: projects the ROBOT forward, adding damping. PREVENTS
                    stop-overshoot at the cost of tracking. Never run on hardware.

Reports tracking lag, direction reversals and high-frequency torque for a sine, plus STOP
OVERSHOOT -- the case self_lookahead exists for and the only one where it can win.

MOVES J1/J4/J5/J6 (default J6). J2/J3 excluded.

    python3 api_tests/35_tracker_tune.py --host <ip> --go --sweep gain
    python3 api_tests/35_tracker_tune.py --host <ip> --go --sweep self
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost

DEFAULT_GAIN = 15.0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true")
    p.add_argument("--sweep", choices=["gain", "look", "self"], default="gain")
    p.add_argument("--values", default="", help="comma list; sensible defaults per axis")
    p.add_argument("--joint", type=int, default=5)
    p.add_argument("--amp", type=float, default=4.0)
    p.add_argument("--freqs", default="0.25,1.0")
    p.add_argument("--rate", type=float, default=25.0)
    p.add_argument("--seconds", type=float, default=5.0)
    a = p.parse_args()

    if not a.go:
        print("Refusing without --go: this MOVES the arm.")
        return 1
    if a.joint not in sc.SAFE_JOINTS:
        print(f"REFUSED: --joint must be within {list(sc.SAFE_JOINTS)}.")
        return 1
    if not sc.clean_rate(a.rate):
        print(f"REFUSED: {a.rate:g} Hz is not an integer tick ratio.")
        return 1

    DEFAULTS = {"gain": "5,10,15,25,40,60", "look": "0,0.05,0.133,0.25,0.5",
                "self": "0,0.033,0.0667,0.133,0.25"}
    vals = [float(x) for x in (a.values or DEFAULTS[a.sweep]).split(",")]
    freqs = [float(x) for x in a.freqs.split(",")]
    amp = math.radians(a.amp)
    j = a.joint

    def cfg_for(v):
        c = {"gen": "tracker"}
        if a.sweep == "gain":
            c["gain"] = v                     # lookahead re-derives as 2/gain
        elif a.sweep == "look":
            c["gain"] = DEFAULT_GAIN; c["lookahead"] = v
        else:
            c["gain"] = DEFAULT_GAIN; c["self_lookahead"] = v
        return c

    print("=" * 96)
    print(f"TRACKER TUNING — sweeping {a.sweep.upper()}   joint J{j+1}   "
          f"+/-{a.amp:g} deg   {a.rate:g} Hz")
    if a.sweep == "gain":
        print("  lookahead re-derives as 2/gain automatically, so the pair stays consistent")
    else:
        print(f"  gain held at {DEFAULT_GAIN:g}")
    print("=" * 96)

    smp = sc.Sampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
    res = {}
    try:
        why = sc.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 1
        rti = RTIClient(robot, clock="pc")
        rti.connect(rate_hz=a.rate); time.sleep(0.4)
        rti.read_once(timeout=2.0)
        rti.claim()
        q0 = rti.read_once(timeout=1.0)["j_p"]

        for v in vals:
            sc.apply_config(robot, cfg_for(v))
            sc.keep_alive(rti, q0, a.rate, 0.4)
            for f in freqs:
                shape = lambda t, f=f: {j: amp * math.sin(2 * math.pi * f * t)}
                print(f"  >> {a.sweep}={v:<7g} {f:4.2f}Hz ...", end="", flush=True)
                rows, _ = sc.stream(rti, q0, shape, a.rate, a.seconds, smp)
                if smp.error:
                    print(f"\n*** sampler failed: {smp.error}")
                    return 1
                lag, rev, trms, pk = sc.analyse(rows, [j])[j]
                res[(v, f)] = (lag, rev, trms)
                print(f"\r  {a.sweep}={v:<7g} {f:4.2f}Hz  lag {lag:6.3f} deg  "
                      f"rev {rev:4.0f}  hf-torque {trms:7.4f} Nm  "
                      f"pk {a.amp*2*math.pi*f:5.1f} d/s   ")
                sc.keep_alive(rti, q0, a.rate, 0.6)

            # STOP TEST — travel then halt. The case self_lookahead exists for.
            half = 3.0
            ramp = lambda t: {j: amp * min(t / half, 1.0)}
            rows, _ = sc.stream(rti, q0, ramp, a.rate, half * 2, smp)
            meas = [r[0][j] for r in rows]
            n = len(meas) // 2
            tgt = q0[j] + amp
            ov = sc.deg(max(0.0, max(meas[n:]) - tgt)) if meas[n:] else float("nan")
            res[(v, "stop")] = ov
            print(f"     {a.sweep}={v:<7g} STOP overshoot {ov:6.3f} deg")
            sc.keep_alive(rti, q0, a.rate, 1.0)

        rti.release(); sc.clear_config(robot); robot.enter_idle_mode()
    except ControlLost as exc:
        print(f"\n*** CONTROL LOST: {exc}")
        return 1
    finally:
        smp.stop()
        try:
            sc.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    print("\n" + "=" * 96)
    print(f"TRACKING LAG (deg) — {a.sweep} down the side, sine speed across")
    print(f"{a.sweep:>10} " + "".join(f"{f'{f:g}Hz':>14}" for f in freqs) + f"{'STOP ov':>14}")
    for v in vals:
        row = "".join(f"{res[(v,f)][0]:14.3f}" if (v, f) in res else f"{'-':>14}"
                      for f in freqs)
        print(f"{v:>10g} {row}{res.get((v,'stop'), float('nan')):14.3f}")
    print(f"\nDIRECTION REVERSALS (the motion has 2)")
    print(f"{a.sweep:>10} " + "".join(f"{f'{f:g}Hz':>14}" for f in freqs))
    for v in vals:
        print(f"{v:>10g} " + "".join(f"{res[(v,f)][1]:14.0f}" if (v, f) in res
                                     else f"{'-':>14}" for f in freqs))
    print(f"\nHIGH-FREQUENCY TORQUE (Nm)")
    print(f"{a.sweep:>10} " + "".join(f"{f'{f:g}Hz':>14}" for f in freqs))
    for v in vals:
        print(f"{v:>10g} " + "".join(f"{res[(v,f)][2]:14.4f}" if (v, f) in res
                                     else f"{'-':>14}" for f in freqs))
    if a.sweep == "self":
        print("\n  self_lookahead is the damping-style lookahead: it adds damping, so expect stop")
        print("  overshoot to FALL and tracking lag to RISE. The question is the exchange")
        print("  rate on real hardware, which simulation put at 0.09 -> 1.44 deg of lag")
        print("  to buy 0.91 -> 0.00 deg of overshoot. Hardware may price it differently.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
