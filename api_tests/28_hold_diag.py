#!/usr/bin/env python3
"""28: diagnose the joint-2 excursion during a SOFT_RTI hold. SHORT, and it logs everything.

27's phase 2 streams the arm's own measured position back as the target -- a hold -- and on
this arm joint 2 moves ~5 deg and joint 3 ~1.9 deg while it does. Two theories have already
been wrong (IDLE gravity drift; a feedback ratchet in the advancing generator -- both
disproved), so this stops theorising and records what actually happens.

It logs shared memory at the FULL loop rate through connect, claim and a short hold, then
prints the moment of the excursion tick by tick: measured, commanded and target side by
side. Those three columns separate the three possible causes:

    commanded jumps, measured follows   -> WE commanded it. A step from the generator.
    commanded flat, measured moves      -> the arm moved on its own; the loop then chased it.
    target jumps                        -> the PC sent a bad setpoint.

WHAT IT DOES TO THE ROBOT: claims control and streams a hold for --seconds (default 4).
Expect the same ~5 deg joint-2 excursion 27 produces. Hand on the E-stop.

    python3 api_tests/28_hold_diag.py --host 127.0.0.1 --rate 25
"""
import argparse
import math
import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robotclient import RobotClient, RTIClient

SHM = "/dev/shm"
FIELDS = ("joint_position", "joint_position_commanded", "joint_position_target")


def rd(name, n=6):
    with open(f"{SHM}/{name}", "rb") as f:
        return list(struct.unpack(f"{n}d", f.read(8 * n)))


def deg(r):
    return r * 180.0 / math.pi


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--rate", type=float, default=25.0)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--watch", type=int, default=2, help="joint index to report (default 2 = J3? no: index 1 is J2)")
    a = ap.parse_args()

    log, stop = [], threading.Event()

    def sampler():
        while not stop.is_set():
            try:
                log.append((time.monotonic(),
                            rd("joint_position"),
                            rd("joint_position_commanded"),
                            rd("joint_position_target")))
            except Exception:
                pass
            time.sleep(0.004)

    th = threading.Thread(target=sampler, daemon=True)
    th.start()
    t_marks = {}

    robot = RobotClient()
    robot.connect(a.host)
    try:
        st = robot.status()
        print(f"[0] mode={st.get('mode')} state={st.get('state')} homed={st.get('homed')}")
        t_marks["start"] = time.monotonic()

        rti = RTIClient(robot, clock="pc")
        rti.connect(rate_hz=a.rate)
        t_marks["connected"] = time.monotonic()
        time.sleep(0.5)

        s0 = rti.read_once(timeout=2.0)
        q0 = s0["j_p"]
        t_marks["read_pose"] = time.monotonic()
        print(f"[1] pose at read: {[round(deg(v), 3) for v in q0]}")

        rti.claim()
        t_marks["claimed"] = time.monotonic()
        print(f"[2] claimed — streaming a live hold at {a.rate:g} Hz for {a.seconds:g}s")

        period, nxt = 1.0 / a.rate, time.monotonic()
        t0 = time.monotonic()
        base = list(q0)
        while time.monotonic() - t0 < a.seconds:
            s = rti.read_state()
            if s and isinstance(s.get("j_p"), list):
                base = list(s["j_p"])
            rti.send_joint_position(base)
            nxt += period
            slp = nxt - time.monotonic()
            if slp > 0:
                time.sleep(slp)
        t_marks["done"] = time.monotonic()
        rti.release()
        robot.enter_idle_mode()
    finally:
        stop.set()
        th.join(timeout=1.0)
        try:
            robot.disconnect()
        except Exception:
            pass

    if not log:
        print("no samples captured")
        return 1

    base_t = log[0][0]
    j = a.watch
    print(f"\n=== joint index {j} (J{j+1}) — the excursion, tick by tick ===")
    print("  event marks: " + "  ".join(f"{k}={v-base_t:.3f}s" for k, v in t_marks.items()))
    m0 = log[0][1][j]
    # Find where it starts moving
    onset = next((i for i, r in enumerate(log) if abs(r[1][j] - m0) > math.radians(0.2)), None)
    if onset is None:
        print(f"  J{j+1} never moved more than 0.2 deg. Nothing to diagnose here.")
    else:
        print(f"  onset at t={log[onset][0]-base_t:.3f}s\n")
        print(f"  {'t (s)':>8} {'measured':>10} {'commanded':>10} {'target':>10} "
              f"{'cmd-meas':>10}")
        lo, hi = max(0, onset - 15), min(len(log), onset + 60)
        for i in range(lo, hi, 2):
            t, meas, cmd, tgt = log[i]
            mark = "  <-- onset" if i == onset else ""
            print(f"  {t-base_t:8.3f} {deg(meas[j]):10.3f} {deg(cmd[j]):10.3f} "
                  f"{deg(tgt[j]):10.3f} {deg(cmd[j]-meas[j]):10.3f}{mark}")

    print(f"\n=== totals over the whole capture ({len(log)} samples) ===")
    for jj in range(6):
        mv = max(abs(r[1][jj] - log[0][1][jj]) for r in log)
        cv = max(abs(r[2][jj] - log[0][2][jj]) for r in log)
        print(f"  J{jj+1}: measured moved {deg(mv):7.3f} deg   commanded moved {deg(cv):7.3f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
