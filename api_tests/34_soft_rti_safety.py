#!/usr/bin/env python3
"""34: SOFT_RTI safety layers — the watchdog and the start-pose check. ZERO MOTION by design.

25_rti_comm.py verifies these for RTI. SOFT_RTI has never had an equivalent, and its
contract differs in exactly the way that matters: the timeout is sized from the PC's rate
rather than fixed at 40 ms, so the number being tested is one the mode computed itself.

Both checks have fired accidentally during bring-up -- always because a test script slept
while holding the claim. Neither has ever been verified DELIBERATELY in SOFT_RTI.

WHAT IT DOES TO THE ROBOT: nothing, if the checks work. Test A goes deliberately silent and
expects the robot to stop itself and latch RTI_LINK_LOST. Test B sends a first setpoint far
from the arm and expects control to be REFUSED. If B's guard is broken the named joint ramps
by --offset degrees, which is why it defaults to a wrist joint. Hand on the E-stop.

    python3 api_tests/34_soft_rti_safety.py --host <ip> --go
    python3 api_tests/34_soft_rti_safety.py --host <ip> --go --rates 5,25,50
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


def scalar(n):
    try:
        return struct.unpack("d", open(f"{sc.SHM}/{n}", "rb").read(8))[0]
    except Exception:
        return float("nan")


def recover(robot):
    robot.clear_errors(); time.sleep(0.8)
    robot.enable(); time.sleep(1.2)
    return robot.status()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true")
    p.add_argument("--rates", default="5,25,50")
    p.add_argument("--joint", type=int, default=5, help="joint the start-pose test offsets")
    p.add_argument("--offset", type=float, default=8.0,
                   help="degrees off. MUST exceed the start-pose tolerance (0.1 rad = 5.73)")
    a = p.parse_args()
    if not a.go:
        print("Refusing without --go. Zero motion is EXPECTED, not guaranteed: if the")
        print("start-pose guard is broken, the wrist ramps by --offset degrees.")
        return 1
    if a.joint not in sc.SAFE_JOINTS:
        print(f"REFUSED: --joint must be within {list(sc.SAFE_JOINTS)}.")
        return 1
    rates = [float(x) for x in a.rates.split(",")]
    for r in rates:
        if not sc.clean_rate(r):
            print(f"REFUSED: {r:g} Hz is not an integer tick ratio.")
            return 1

    robot = RobotClient(); robot.connect(a.host)
    passes, fails = [], []
    try:
        print("=" * 92)
        print("SOFT_RTI SAFETY LAYERS — zero motion expected")
        print("=" * 92)

        # ---------------- A: the command watchdog, at each rate ----------------
        print("\nA. COMMAND WATCHDOG — stream, then go deliberately silent.")
        print("   SOFT_RTI sizes the timeout at 3 command intervals, so the expected stop")
        print("   time DIFFERS per rate. That is the part RTI's fixed 40 ms cannot test.\n")
        for rate in rates:
            why = sc.preflight(robot)
            if why:
                print(f"   {rate:5.0f} Hz  SKIP — {why}")
                continue
            expect = 3.0 / rate
            rti = RTIClient(robot, clock="pc")
            rti.connect(rate_hz=rate); time.sleep(0.4)
            rti.read_once(timeout=2.0)
            rti.claim()
            q0 = rti.read_once(timeout=1.0)["j_p"]
            sc.keep_alive(rti, q0, rate, 1.0)          # establish control
            before = sc.rd("joint_position")
            t_silent = time.monotonic()
            stopped_at = None
            while time.monotonic() - t_silent < expect * 4 + 1.0:
                if robot.status().get("err"):
                    stopped_at = time.monotonic() - t_silent
                    break
                time.sleep(0.02)
            after = sc.rd("joint_position")
            moved = max(abs(sc.deg(after[j] - before[j])) for j in range(6))
            errs = robot.status().get("errors") or []
            ok = stopped_at is not None and "RTI_LINK_LOST" in errs and moved < 0.5
            tag = "PASS" if ok else "FAIL"
            (passes if ok else fails).append(f"watchdog @ {rate:g} Hz")
            st_s = f"{stopped_at:.3f}s" if stopped_at else "NEVER"
            print(f"   {rate:5.0f} Hz  expect ~{expect:.3f}s  stopped {st_s:>8}  "
                  f"moved {moved:.4f} deg  errs={errs}  [{tag}]")
            recover(robot)

        # ---------------- B: the start-pose check ----------------
        print(f"\nB. START-POSE CHECK — first setpoint {a.offset:g} deg off on joint "
              f"{a.joint+1}.")
        print("   Tolerance is 0.1 rad = 5.73 deg. Expect control REFUSED and no motion.")
        print("   If the guard is broken, that joint ramps. WATCH IT.\n")
        for rate in (rates[0], rates[-1]):
            why = sc.preflight(robot)
            if why:
                print(f"   {rate:5.0f} Hz  SKIP — {why}")
                continue
            rti = RTIClient(robot, clock="pc")
            rti.connect(rate_hz=rate); time.sleep(0.4)
            rti.read_once(timeout=2.0)
            rti.claim()
            q0 = rti.read_once(timeout=1.0)["j_p"]
            before = sc.rd("joint_position")
            bad = list(q0)
            bad[a.joint] += math.radians(a.offset)
            for _ in range(int(rate)):            # ~1 s of a deliberately wrong setpoint
                rti.send_joint_position(bad)
                time.sleep(1.0 / rate)
            time.sleep(0.4)
            after = sc.rd("joint_position")
            moved = abs(sc.deg(after[a.joint] - before[a.joint]))
            errs = robot.status().get("errors") or []
            ok = moved < 1.0
            (passes if ok else fails).append(f"start-pose @ {rate:g} Hz")
            print(f"   {rate:5.0f} Hz  joint moved {moved:6.3f} deg  errs={errs}  "
                  f"[{'PASS' if ok else 'FAIL — THE GUARD DID NOT HOLD'}]")
            recover(robot)

        print("\n" + "=" * 92)
        print(f"  {len(passes)} passed, {len(fails)} failed")
        if fails:
            print("  FAILED: " + ", ".join(fails))
        return 1 if fails else 0
    finally:
        try:
            recover(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
