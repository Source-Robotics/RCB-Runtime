#!/usr/bin/env python3
"""32: multi-joint coordinated streaming — config x rate x speed, on J1/J4/J5/J6.

Every servo test so far moved ONE joint. That leaves the realistic case untested, and it
leaves one mechanism entirely unexercised: min_duration flips Ruckig to
Synchronization.Time, where joints share ONE trajectory duration instead of moving
independently. With a single joint that switch is invisible. Here it is the point.

Joints are driven with PHASE-SHIFTED sines so they do not all peak together -- a coordinated
motion rather than four solos. Per-joint lag, direction reversals and high-frequency torque
are reported, plus a COORDINATION error: how much the joints drift out of their intended
phase relationship, which is the thing single-joint tests cannot see at all.

MOVES J1, J4, J5, J6 ONLY. J2 and J3 are excluded deliberately -- they carry the gravity
load, and J3 has a known velocity-loop oscillation (servo_mode_options.md 14.6) that would
confound any torque measurement near it.

    python3 api_tests/32_multi_joint.py --host <ip> --go
    python3 api_tests/32_multi_joint.py --host <ip> --go --freqs 0.25,0.5,1.0
    python3 api_tests/32_multi_joint.py --host <ip> --go --configs list
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost

JNAMES = {0: "J1", 3: "J4", 4: "J5", 5: "J6"}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true", help="required: this MOVES the arm")
    p.add_argument("--joints", default="0,3,4,5", help="comma indices; default J1,J4,J5,J6")
    p.add_argument("--amp", type=float, default=4.0, help="degrees, per joint")
    p.add_argument("--freqs", default="0.25,0.75", help="sine frequencies, Hz")
    p.add_argument("--rates", default="10,25,50", help="command rates, Hz")
    p.add_argument("--configs", default="", help="'list' to enumerate")
    p.add_argument("--seconds", type=float, default=5.0)
    a = p.parse_args()

    if a.configs.strip() == "list":
        for k, v in sc.CONFIGS.items():
            extra = ", ".join(f"{kk}={vv}" for kk, vv in v.items() if kk != "gen")
            print(f"  {k:20} {v['gen']:18} {extra}")
        return 0
    if not a.go:
        print("Refusing to run without --go: this test MOVES four joints at once.")
        return 1

    joints = [int(x) for x in a.joints.split(",")]
    bad = [j for j in joints if j not in sc.SAFE_JOINTS]
    if bad:
        print(f"REFUSED: joints {bad} are not in the safe set {list(sc.SAFE_JOINTS)}.")
        print("J2/J3 carry the gravity load and J3 has a known velocity-loop oscillation.")
        return 1
    rates = [float(x) for x in a.rates.split(",")]
    for r in rates:
        if not sc.clean_rate(r):
            print(f"REFUSED: {r:g} Hz is {sc.LOOP_HZ/r:.2f} ticks/command, not an integer "
                  f"ratio. Clean: {', '.join(f'{c:g}' for c in sc.CLEAN_RATES)}")
            return 1
    freqs = [float(x) for x in a.freqs.split(",")]
    cfgs = a.configs.split(",") if a.configs else list(sc.DEFAULT_CONFIGS)
    amp = math.radians(a.amp)

    # Phase-shifted so the joints do not all peak together. Evenly spread over a full cycle.
    phase = {j: 2 * math.pi * i / len(joints) for i, j in enumerate(joints)}

    def shape_for(f):
        def shape(t):
            return {j: amp * math.sin(2 * math.pi * f * t + phase[j]) for j in joints}
        return shape

    ncell = len(rates) * len(freqs) * len(cfgs)
    print("=" * 100)
    print(f"MULTI-JOINT SWEEP   joints {[JNAMES[j] for j in joints]}   +/-{a.amp:g} deg each")
    print(f"  {len(cfgs)} configs x {len(rates)} rates x {len(freqs)} speeds "
          f"= {ncell} cells x {a.seconds:g}s = ~{ncell*(a.seconds+1.5)/60:.1f} min of motion")
    print(f"  phases: " + "  ".join(f"{JNAMES[j]} {math.degrees(phase[j]):.0f}deg"
                                    for j in joints))
    print("=" * 100)

    smp = sc.Sampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
    results = {}
    try:
        why = sc.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 1
        for rate in rates:
            rti = RTIClient(robot, clock="pc")
            rti.connect(rate_hz=rate); time.sleep(0.4)
            rti.read_once(timeout=2.0)
            rti.claim()
            q0 = rti.read_once(timeout=1.0)["j_p"]     # AFTER the claim; never a stale pose
            for freq in freqs:
                pk = a.amp * 2 * math.pi * freq
                for name in cfgs:
                    sc.apply_config(robot, sc.CONFIGS[name])
                    sc.keep_alive(rti, q0, rate, 0.4)
                    print(f"  >> {rate:5.0f}Hz {freq:5.2f}Hz {name:18s} ...",
                          end="", flush=True)
                    rows, ach = sc.stream(rti, q0, shape_for(freq), rate, a.seconds, smp)
                    if smp.error:
                        print(f"\n*** sampler failed: {smp.error}")
                        return 1
                    m = sc.analyse(rows, joints)
                    # Coordination: worst pairwise deviation from the intended phase offset,
                    # measured as how far each joint's lag differs from the others'. Joints
                    # that lag by DIFFERENT amounts distort the shape of a coordinated move.
                    lags = [m[j][0] for j in joints]
                    coord = max(lags) - min(lags)
                    results[(rate, freq, name)] = (m, coord, ach)
                    print(f"\r  {rate:5.0f}Hz {freq:5.2f}Hz {name:18s} "
                          f"lag " + " ".join(f"{JNAMES[j]} {m[j][0]:5.3f}" for j in joints) +
                          f"  spread {coord:5.3f}  pk {pk:5.1f}d/s   ")
                    sc.keep_alive(rti, q0, rate, 0.8)
            rti.release(); sc.clear_config(robot); robot.enter_idle_mode(); time.sleep(0.4)
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
    print("COORDINATION SPREAD (deg) — max minus min per-joint lag. Lower = the joints stay")
    print("in phase, so the coordinated SHAPE is preserved. This is what a single-joint")
    print("test cannot measure, and what Synchronization.Time exists to protect.")
    print("=" * 100)
    print(f"{'rate':>7} {'speed':>7} " + "".join(f"{c:>20}" for c in cfgs))
    for rate in rates:
        for freq in freqs:
            row = "".join(f"{results[(rate,freq,c)][1]:20.3f}"
                          if (rate, freq, c) in results else f"{'-':>20}" for c in cfgs)
            print(f"{rate:6.0f}H {freq:6.2f}H {row}")

    print("\nWORST PER-JOINT LAG (deg)")
    print(f"{'rate':>7} {'speed':>7} " + "".join(f"{c:>20}" for c in cfgs))
    for rate in rates:
        for freq in freqs:
            row = ""
            for c in cfgs:
                if (rate, freq, c) in results:
                    m = results[(rate, freq, c)][0]
                    row += f"{max(m[j][0] for j in joints):20.3f}"
                else:
                    row += f"{'-':>20}"
            print(f"{rate:6.0f}H {freq:6.2f}H {row}")

    print("\nTOTAL DIRECTION REVERSALS across all joints (the motion itself has "
          f"{2*len(joints)})")
    print(f"{'rate':>7} {'speed':>7} " + "".join(f"{c:>20}" for c in cfgs))
    for rate in rates:
        for freq in freqs:
            row = ""
            for c in cfgs:
                if (rate, freq, c) in results:
                    m = results[(rate, freq, c)][0]
                    row += f"{sum(m[j][1] for j in joints):20.0f}"
                else:
                    row += f"{'-':>20}"
            print(f"{rate:6.0f}H {freq:6.2f}H {row}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
