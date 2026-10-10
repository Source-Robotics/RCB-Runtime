#!/usr/bin/env python3
"""37: how smooth can a 5 Hz stream be, and what does it cost? The Pareto frontier.

At 5 Hz the robot must invent 50 ticks -- 200 ms -- of motion between commands. It cannot
know what happened in there, so it must choose: assume the target kept moving (accurate,
snappy, corrects hard when wrong) or assume it stopped (smooth, laggy). That is not an
implementation limit, it is an information limit, and "make 5 Hz smooth" has no single
answer -- it has a FRONTIER, and an operator picks a point on it.

So this measures the frontier directly. Every smoothing mechanism the handler has, swept at
one fixed low rate, plotted as roughness against accuracy:

    min_duration   stretch each plan so it never finishes early
    tracker gain   lower bandwidth = heavier filtering
    self_lookahead damping-style lookahead
    lowpass        a one-pole filter on the incoming command stream. Built, shipped
                   defaulted OFF, and NEVER RUN until now.
    pace           self-tuning speed ceiling

Both metrics need no differentiation, so unlike jerk or velocity-step counts they survive a
Python sampler that drops samples:

    t_rms   high-frequency joint torque (Nm). Roughness -- what the operator hears.
    lag     rms |measured - commanded| (deg). Accuracy -- what the operator sees.

A config is PARETO-OPTIMAL if nothing else is both smoother AND more accurate. Those are the
only ones worth choosing between; everything else is strictly beaten.

    python3 api_tests/37_frontier.py --host <ip> --go
    python3 api_tests/37_frontier.py --host <ip> --go --rate 10 --freq 0.5
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost

# Ordered roughly snappy -> smooth. Each is one point in the smoothing space.
CANDIDATES = {
    "advancing (baseline)":     dict(gen="ruckig_advancing"),
    "ruckig (shipped)":         dict(gen="ruckig"),
    # --- duration pacing: stretch the plan ---
    "adv + mindur 1.5":         dict(gen="ruckig_advancing", min_duration=1.5),
    "adv + mindur 2.0":         dict(gen="ruckig_advancing", min_duration=2.0),
    "adv + mindur 3.0":         dict(gen="ruckig_advancing", min_duration=3.0),
    "adv + mindur 5.0":         dict(gen="ruckig_advancing", min_duration=5.0),
    # --- tracker bandwidth: the tracker's pacing IS its gain ---
    "tracker gain 3":           dict(gen="tracker", gain=3.0),
    "tracker gain 5":           dict(gen="tracker", gain=5.0),
    "tracker gain 15":          dict(gen="tracker", gain=15.0),
    "tracker g5 + self 0.25":   dict(gen="tracker", gain=5.0, self_lookahead=0.25),
    "tracker g15 + self 0.25":  dict(gen="tracker", gain=15.0, self_lookahead=0.25),
    # --- command low-pass: NEVER TESTED before this script ---
    "adv + lowpass 25Hz":       dict(gen="ruckig_advancing", lowpass=25.0),
    "adv + lowpass 10Hz":       dict(gen="ruckig_advancing", lowpass=10.0),
    "adv + lowpass 5Hz":        dict(gen="ruckig_advancing", lowpass=5.0),
    "adv + lowpass 2Hz":        dict(gen="ruckig_advancing", lowpass=2.0),
    "adv + mindur + lp 5Hz":    dict(gen="ruckig_advancing", min_duration=1.5, lowpass=5.0),
    # --- speed ceiling ---
    "adv + pace 1.5":           dict(gen="ruckig_advancing", pace=1.5),
}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true")
    p.add_argument("--rate", type=float, default=5.0, help="the LOW rate under test")
    p.add_argument("--joints", default="3,4,5",
                   help="comma indices; default J4,J5,J6 -- three wrist joints moving at "
                        "once is easier to SEE than one, and the operator's eye has been "
                        "ahead of the instruments more than once here")
    p.add_argument("--amp", type=float, default=4.0, help="degrees, per joint")
    p.add_argument("--freq", type=float, default=0.25)
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--only", default="", help="comma substrings to filter candidates")
    a = p.parse_args()

    if not a.go:
        print("Refusing without --go: this MOVES the arm.")
        return 1
    joints = [int(x) for x in a.joints.split(",")]
    bad = [j for j in joints if j not in sc.SAFE_JOINTS]
    if bad:
        print(f"REFUSED: joints {bad} are not in the safe set {list(sc.SAFE_JOINTS)} "
              f"(J1/J4/J5/J6). J2/J3 carry the gravity load.")
        return 1
    if not sc.clean_rate(a.rate):
        print(f"REFUSED: {a.rate:g} Hz is {sc.LOOP_HZ/a.rate:.2f} ticks/command, not an "
              f"integer ratio.")
        return 1

    names = [k for k in CANDIDATES
             if not a.only or any(t.strip() in k for t in a.only.split(","))]
    amp = math.radians(a.amp)
    pk = a.amp * 2 * math.pi * a.freq
    JN = {0: "J1", 3: "J4", 4: "J5", 5: "J6"}
    # Phase-shifted so they do not all peak together -- a coordinated motion rather than
    # three solos, and the phase relationship is itself visible if it breaks down.
    phase = {j: 2 * math.pi * i / len(joints) for i, j in enumerate(joints)}

    print("=" * 92)
    print(f"SMOOTHNESS / ACCURACY FRONTIER at {a.rate:g} Hz "
          f"({sc.LOOP_HZ/a.rate:.0f} ticks/command)")
    print(f"  {[JN[j] for j in joints]}  +/-{a.amp:g} deg each @ {a.freq:g} Hz  "
          f"({pk:.1f} deg/s peak)  {len(names)} configs x {a.seconds:g}s")
    print(f"  phases: " + "  ".join(f"{JN[j]} {math.degrees(phase[j]):.0f}deg"
                                    for j in joints))
    print("=" * 92)
    print(f"{'config':>24} {'t_rms Nm':>10} {'worst lag':>10} {'spread':>8}   per-joint lag")
    print("-" * 92)

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
        def shape(t):
            return {j: amp * math.sin(2 * math.pi * a.freq * t + phase[j]) for j in joints}

        for name in names:
            sc.apply_config(robot, CANDIDATES[name])
            sc.keep_alive(rti, q0, a.rate, 0.6)
            rows, _ = sc.stream(rti, q0, shape, a.rate, a.seconds, smp)
            if smp.error:
                print(f"\n*** sampler failed: {smp.error}")
                return 1
            m = sc.analyse(rows, joints)
            lags = [m[j][0] for j in joints]
            # Worst joint, not the mean: a config that is excellent on two joints and poor
            # on the third is poor. Spread is the coordination cost -- joints lagging by
            # DIFFERENT amounts distort the shape of a coordinated move.
            trms = max(m[j][2] for j in joints)
            worst, spread = max(lags), max(lags) - min(lags)
            res[name] = (trms, worst, spread)
            detail = " ".join(f"{JN[j]} {m[j][0]:5.3f}" for j in joints)
            print(f"{name:>24} {trms:10.4f} {worst:10.3f} {spread:8.3f}   {detail}")
            sc.keep_alive(rti, q0, a.rate, 0.6)

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

    if not res:
        return 1
    # Pareto: nothing else is BOTH smoother and more accurate.
    front = [n for n in res
             if not any(res[o][0] < res[n][0] and res[o][1] < res[n][1] for o in res)]
    order = sorted(res, key=lambda n: res[n][0])

    print("\n" + "=" * 92)
    print("THE FRONTIER — sorted by roughness. * marks Pareto-optimal: nothing else is")
    print("both smoother AND more accurate. Everything unmarked is strictly beaten.")
    print("=" * 92)
    print(f"{'':>2} {'config':>24} {'t_rms Nm':>10} {'worst lag':>10}   roughness")
    lo = min(res[n][0] for n in res)
    hi = max(res[n][0] for n in res)
    for n in order:
        t, l, _ = res[n]
        bar = "#" * int(1 + 40 * (t - lo) / max(hi - lo, 1e-9))
        print(f"{'*' if n in front else ' ':>2} {n:>24} {t:10.4f} {l:10.3f}   {bar}")

    base = res.get("advancing (baseline)")
    if base:
        print(f"\n  baseline 'advancing': {base[0]:.4f} Nm / {base[1]:.3f} deg")
        best_s = min(res, key=lambda n: res[n][0])
        best_a = min(res, key=lambda n: res[n][1])
        print(f"  smoothest : {best_s:>24}  {res[best_s][0]:.4f} Nm "
              f"({base[0]/max(res[best_s][0],1e-9):.2f}x quieter) "
              f"lag {res[best_s][1]:.3f} deg")
        print(f"  most accurate: {best_a:>21}  {res[best_a][1]:.3f} deg "
              f"lag, {res[best_a][0]:.4f} Nm")
    print("\n  Pick a point ON the frontier. Anything off it is worse in both directions.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
