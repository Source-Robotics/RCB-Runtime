#!/usr/bin/env python3
"""38: end-to-end FIDELITY — how close does the arm get to the wave you sent?

Every other servo test measures |measured - joint_position_commanded|, i.e. how well the arm
follows the GENERATOR. That is a useful servo number and the wrong question for an operator:
a generator that emits a lazy setpoint scores well on it while tracking your intent badly.
It is also why those tables show "lag" RISING with command rate, which is nonsense as
fidelity.

This measures the only thing that answers "did the robot do what I asked":

    err = rms | measured(t)  -  the analytic sine at time t |

against the CONTINUOUS wave, not against the samples the PC managed to send. Sampling a
continuous intent at 5 Hz already discards information; comparing to the samples would hide
exactly the loss being measured.

It sweeps command rate AGAINST signal frequency, because that ratio -- samples per cycle --
is the thing that actually decides fidelity. Nyquist says a signal needs more than 2
samples/cycle to exist at all, and smooth reconstruction wants ~10. A 0.25 Hz wave at 5 Hz
gets 20 samples/cycle and is easy; a 2 Hz wave at 5 Hz gets 2.5 and is impossible. The
command rate alone tells you nothing without the signal it is carrying.

MOVES J4/J5/J6 by default. J2/J3 refused.

    python3 api_tests/38_fidelity.py --host <ip> --go
    python3 api_tests/38_fidelity.py --host <ip> --go --rates 5,10,25,50,125 --freqs 0.25,1,2
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost

JN = {0: "J1", 3: "J4", 4: "J5", 5: "J6"}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true")
    p.add_argument("--joints", default="3,4,5")
    p.add_argument("--amp", type=float, default=4.0)
    p.add_argument("--rates", default="5,10,25,50,125")
    p.add_argument("--freqs", default="0.25,0.5,1.0,2.0")
    p.add_argument("--config", default="advancing+mindur")
    p.add_argument("--seconds", type=float, default=6.0)
    a = p.parse_args()

    if not a.go:
        print("Refusing without --go: this MOVES the arm.")
        return 1
    joints = [int(x) for x in a.joints.split(",")]
    if any(j not in sc.SAFE_JOINTS for j in joints):
        print(f"REFUSED: joints must be within {list(sc.SAFE_JOINTS)}.")
        return 1
    rates = [float(x) for x in a.rates.split(",")]
    for r in rates:
        if not sc.clean_rate(r):
            print(f"REFUSED: {r:g} Hz is not an integer tick ratio.")
            return 1
    freqs = [float(x) for x in a.freqs.split(",")]
    if a.config not in sc.CONFIGS:
        print(f"unknown config {a.config!r}. Known: {', '.join(sc.CONFIGS)}")
        return 1
    amp = math.radians(a.amp)
    phase = {j: 2 * math.pi * i / len(joints) for i, j in enumerate(joints)}

    print("=" * 96)
    print(f"END-TO-END FIDELITY   {[JN[j] for j in joints]}  +/-{a.amp:g} deg  "
          f"config={a.config}")
    print("  error is measured against the CONTINUOUS wave, not against the samples sent")
    print("=" * 96)
    print(f"{'rate':>7} {'signal':>8} {'samp/cyc':>9} {'pk deg/s':>10} "
          f"{'err rms':>9} {'err max':>9} {'t_rms Nm':>10}")
    print("-" * 74)

    smp = sc.Sampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
    res = {}
    pending = []            # (rate, freq, samples_per_cycle, rows, duration)
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
            q0 = rti.read_once(timeout=1.0)["j_p"]
            sc.apply_config(robot, sc.CONFIGS[a.config])
            sc.keep_alive(rti, q0, rate, 0.5)
            for f in freqs:
                period = 1.0 / rate
                smp.start_window()
                t0, nxt = time.monotonic(), time.monotonic()
                while True:
                    t = time.monotonic() - t0
                    if t >= a.seconds:
                        break
                    q = list(q0)
                    for j2 in joints:
                        q[j2] = q0[j2] + amp * math.sin(2 * math.pi * f * t + phase[j2])
                    rti.send_joint_position(q)
                    nxt += period
                    slp = nxt - time.monotonic()
                    if slp > 0:
                        time.sleep(slp)
                dur = time.monotonic() - t0
                # STASH, do not analyse. Analysing here is a gap of hundreds of ms in the
                # command stream while the claim is held -- fatal at 125 Hz, where the
                # timeout is 24 ms. The first run of this test produced a whole 125 Hz block
                # of 4.098 deg errors (= the full amplitude, i.e. the arm never moved) for
                # exactly that reason, and it read like a fidelity result.
                with sc.gap_guard(rate, f"stash {rate:g}Hz/{f:g}Hz"):
                    pending.append((rate, f, rate / f, smp.end_window(), dur, list(q0)))
                if smp.error:
                    print(f"\n*** sampler failed: {smp.error}")
                    return 1
                sc.keep_alive(rti, q0, rate, 0.8)
            rti.release(); sc.clear_config(robot); robot.enter_idle_mode(); time.sleep(0.3)
    except ControlLost as exc:
        print(f"\n*** CONTROL LOST: {exc}")
    finally:
        smp.stop()
        try:
            sc.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    # ---- analysis, with the claim long since released ----
    for rate, f, spc, rows, dur, base in pending:
        if len(rows) < 20:
            print(f"{rate:6.0f}H {f:7.2f}H  too few samples")
            continue
        n = len(rows)
        half = n // 2
        errs, worst = [], 0.0
        for i2 in range(half, n):
            t = dur * i2 / n
            for j2 in joints:
                truth = base[j2] + amp * math.sin(2 * math.pi * f * t + phase[j2])
                e = abs(rows[i2][0][j2] - truth)
                errs.append(e)
                worst = max(worst, e)
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        m = sc.analyse(rows, joints)
        trms = max(m[j2][2] for j2 in joints)
        res[(rate, f)] = (sc.deg(rms), sc.deg(worst), trms)
        print(f"{rate:6.0f}H {f:7.2f}H {spc:9.1f} {a.amp*2*math.pi*f:10.1f} "
              f"{sc.deg(rms):9.3f} {sc.deg(worst):9.3f} {trms:10.4f}")

    print("\n" + "=" * 96)
    print("FIDELITY: rms error vs the wave you asked for (deg). Rows = command rate.")
    print("=" * 96)
    print(f"{'rate':>7} " + "".join(f"{f'{f:g}Hz sig':>14}" for f in freqs))
    for rate in rates:
        print(f"{rate:6.0f}H " + "".join(
            f"{res[(rate,f)][0]:14.3f}" if (rate, f) in res else f"{'-':>14}" for f in freqs))
    print("\nSAMPLES PER CYCLE — under ~2 the wave cannot be represented at all;")
    print("under ~10 it cannot be represented SMOOTHLY. This is the real variable.")
    print(f"{'rate':>7} " + "".join(f"{f'{f:g}Hz sig':>14}" for f in freqs))
    for rate in rates:
        row = ""
        for f in freqs:
            spc = rate / f
            mark = " !!" if spc < 2 else (" !" if spc < 10 else "")
            row += f"{f'{spc:.1f}{mark}':>14}"
        print(f"{rate:6.0f}H {row}")
    print("\n  Read the two tables together: where samples/cycle is comfortable, raising the")
    print("  command rate buys little. Where it is marginal, the rate is everything.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
