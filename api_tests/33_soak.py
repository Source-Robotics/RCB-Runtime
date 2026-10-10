#!/usr/bin/env python3
"""33: SOFT_RTI soak — does it stay good for minutes, not seconds?

The longest servo run so far is 15 s. Nothing is known about what happens over minutes:
whether position drifts cumulatively, whether the link degrades, whether motors heat, or
whether anything latches. Short tests cannot see any of that, and a teleop session is
minutes long.

Reports every --report seconds so a trend is visible while it runs, not only at the end:

    drift      cumulative position change from the START of the soak, per joint. A hold
               that slowly walks away is the failure this exists to catch.
    lag        rms |measured - commanded|. Should be flat; a rising trend means something
               is degrading.
    success    robot-side command success rate. At 25 Hz against a 250 Hz loop a HEALTHY
               link reads 10%, so watch the trend, not the value.
    discard    packets arriving faster than the loop consumes them.
    temp       motor temperature, if published.

MOVES J1/J4/J5/J6 in a slow phase-shifted sine. Amplitude defaults small.

    python3 api_tests/33_soak.py --host <ip> --go --minutes 10
    python3 api_tests/33_soak.py --host <ip> --go --minutes 30 --hold-only
"""
import argparse
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost

JN = {0: "J1", 3: "J4", 4: "J5", 5: "J6"}


def scalar(name):
    try:
        return struct.unpack("d", open(f"{sc.SHM}/{name}", "rb").read(8))[0]
    except Exception:
        return float("nan")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true")
    p.add_argument("--minutes", type=float, default=10.0)
    p.add_argument("--rate", type=float, default=25.0)
    p.add_argument("--config", default="advancing+mindur")
    p.add_argument("--joints", default="0,3,4,5")
    p.add_argument("--amp", type=float, default=3.0)
    p.add_argument("--freq", type=float, default=0.1, help="slow by design")
    p.add_argument("--report", type=float, default=30.0, help="seconds between reports")
    p.add_argument("--hold-only", action="store_true",
                   help="zero motion: soak the HOLD instead. Safest long-run test")
    a = p.parse_args()

    if not a.go:
        print("Refusing without --go." + ("" if a.hold_only else " This MOVES the arm."))
        return 1
    joints = [int(x) for x in a.joints.split(",")]
    if any(j not in sc.SAFE_JOINTS for j in joints):
        print(f"REFUSED: joints must be within {list(sc.SAFE_JOINTS)} (J1/J4/J5/J6).")
        return 1
    if not sc.clean_rate(a.rate):
        print(f"REFUSED: {a.rate:g} Hz is not an integer tick ratio.")
        return 1
    if a.config not in sc.CONFIGS:
        print(f"unknown config {a.config!r}. Known: {', '.join(sc.CONFIGS)}")
        return 1

    amp = 0.0 if a.hold_only else math.radians(a.amp)
    phase = {j: 2 * math.pi * i / len(joints) for i, j in enumerate(joints)}
    total = a.minutes * 60.0

    print("=" * 96)
    print(f"SOFT_RTI SOAK   {a.minutes:g} min   {a.rate:g} Hz   config={a.config}")
    what = ("HOLD ONLY (zero motion)" if a.hold_only
            else f"joints {[JN[j] for j in joints]} +/-{a.amp:g} deg @ {a.freq:g} Hz")
    print(f"  {what}")
    print("=" * 96)

    smp = sc.Sampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
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
        sc.apply_config(robot, sc.CONFIGS[a.config])
        sc.keep_alive(rti, q0, a.rate, 0.5)
        start_pose = sc.rd("joint_position")

        def shape(t):
            return {j: amp * math.sin(2 * math.pi * a.freq * t + phase[j]) for j in joints}

        print(f"\n{'elapsed':>9} {'worst drift':>12} {'worst lag':>11} {'success':>9} "
              f"{'discard':>9} {'max temp':>9}  status")
        print("-" * 78)
        t0 = time.monotonic()
        n_report = 0
        while time.monotonic() - t0 < total:
            chunk = min(a.report, total - (time.monotonic() - t0))
            if chunk <= 0:
                break
            rows, ach = sc.stream(rti, q0, shape, a.rate, chunk, smp)
            # Everything from here to the next stream() call is a gap in the command
            # stream. Keep it to shared-memory reads and one print -- microseconds, not the
            # milliseconds a TCP round-trip costs.
            if smp.error:
                print(f"\n*** sampler failed: {smp.error}")
                return 1
            # Analyse only the TAIL. The report is a trend indicator, not a full accounting,
            # and analysing a whole 30 s chunk is over a million Python operations -- which
            # is a multi-second gap in the command stream while the claim is held. That,
            # not the earlier robot.status() call, is what killed the first two soaks.
            with sc.gap_guard(a.rate, "soak report"):
                m = sc.analyse(rows[-600:], joints)
                now = sc.rd("joint_position")
                drift = max(abs(sc.deg(now[j] - start_pose[j])) for j in range(6))
                lag = max(m[j][0] for j in joints)
                try:
                    mt = max(sc.rd("motor_temperature"))
                except Exception:
                    mt = float("nan")
            # Read the error state from SHARED MEMORY, not robot.status(). status() is a
            # TCP round-trip, and doing one between stream chunks means going SILENT while
            # claimed -- which at 25 Hz exceeds the 0.12 s timeout and latches
            # RTI_LINK_LOST. The first version of this soak died at 30 s every time, on its
            # own reporting code, with "worst consecutive misses: 31" against a 30-tick
            # budget. Nothing here may touch TCP while the claim is held.
                bad = scalar("error_active") > 0.5
                flag = "  *** ERROR LATCHED" if bad else "  ok"
                el = time.monotonic() - t0
                line = (f"{el:8.0f}s {drift:11.4f} {lag:10.4f} "
                        f"{scalar('control_success_rate')*100:8.1f}% "
                        f"{scalar('rti_discard_pct'):8.1f}% {mt:8.1f}{flag}")
            print(line)
            n_report += 1
            if bad:
                print("\n*** ABORTING: the robot left SOFT_RTI or latched an error.")
                break
        rti.release(); sc.clear_config(robot); robot.enter_idle_mode()
        end_pose = sc.rd("joint_position")
        print("\n" + "=" * 96)
        print("CUMULATIVE DRIFT over the whole soak (deg) — a hold that walks away is a leak")
        for j in range(6):
            d = sc.deg(end_pose[j] - start_pose[j])
            mark = "  <<<" if abs(d) > 0.5 else ""
            print(f"  J{j+1}: {d:+8.4f}{mark}")
        print(f"\n  worst consecutive misses: {scalar('rti_worst_consecutive_misses'):.0f}")
        return 0
    except ControlLost as exc:
        print(f"\n*** CONTROL LOST after {time.monotonic()-t0:.0f}s: {exc}")
        return 1
    finally:
        smp.stop()
        try:
            sc.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
