#!/usr/bin/env python3
"""31: reproduce the J3 buzz on demand, and find out whether it is TUNING or HARDWARE.

The buzz only ever appeared after the robot fell into ACTIVE_ERROR, which holds joints with
velocity=0 rather than gravity torque. Measured on this arm: J3 speed sd 33.7 deg/s in
velocity-hold against 0.027 in gravity-comp torque hold -- a 1250x difference on one joint.

CLAUDE.md records that exact signature for a detached encoder magnet on J5. But there is a
simpler candidate the gain table points straight at:

    J2 (shoulder, loaded)   KPV 0.010  KIV 0.0010  KPIQ 2   <- deliberately detuned
    J3 (elbow,    loaded)   KPV 0.015  KIV 0.0015  KPIQ 7   <- J1's gains
    J1 (base,   unloaded)   KPV 0.015  KIV 0.0015  KPIQ 7

J3 carries shoulder-like load on base-joint gains. A velocity loop too stiff for its load
limit-cycles, and it does so ONLY where that loop is in circuit -- which is exactly the
observed pattern, with no broken sensor required.

REPRODUCING IT SAFELY. handle_idle() falls back to velocity-hold whenever gravity
compensation is off, so `gravity_comp(False)` in IDLE puts the arm in the same hold that
ACTIVE_ERROR uses -- on demand, reversible, and with no error latched.

THE DISCRIMINATOR: if lowering J3's velocity-loop gains to J2's values stops the
oscillation, it is tuning. If it oscillates identically at any gain, the loop is chasing a
bad signal and the sensor is the suspect.

WHAT IT DOES TO THE ROBOT: no commanded motion. It switches the hold type and, with
--gains, temporarily changes J3's velocity-loop gains. Gains are RAM-only -- they are
reloaded from the robot XML on every boot and on every motor reconnect -- so nothing here
is persistent. It restores them anyway. Hand on the E-stop.

    python3 api_tests/31_velhold_buzz.py --host 127.0.0.1
    python3 api_tests/31_velhold_buzz.py --host 127.0.0.1 --gains
"""
import argparse
import math
import os
import statistics
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robotclient import RobotClient

NJ = 6


def rd(name):
    with open(f"/dev/shm/{name}", "rb") as f:
        return list(struct.unpack(f"{NJ}d", f.read(8 * NJ)))


def sample(secs):
    pos, spd, trq = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        pos.append(rd("joint_position"))
        spd.append(rd("joint_speed"))
        trq.append(rd("joint_torque"))
        time.sleep(0.002)
    return pos, spd, trq


def stats(pos, spd, trq, j):
    p = [r[j] for r in pos]
    s = [math.degrees(r[j]) for r in spd]
    t = [r[j] for r in trq]
    return (math.degrees(max(p) - min(p)), statistics.pstdev(s),
            statistics.pstdev(t), statistics.mean(t))


def row(tag, pos, spd, trq, watch):
    pp, ss, ts, tm = stats(pos, spd, trq, watch)
    loud = "  <<< OSCILLATING" if ss > 5.0 else ""
    print(f"  {tag:<34} pos p-p {pp:7.4f} deg   speed sd {ss:8.3f} deg/s   "
          f"torque sd {ts:7.4f} Nm{loud}")
    return ss


def all_joints(tag, pos, spd, trq):
    print(f"\n  {tag} — every joint")
    print(f"  {'':>5} {'pos p-p deg':>13} {'speed sd deg/s':>16} {'torque sd Nm':>14} {'load Nm':>10}")
    for j in range(NJ):
        pp, ss, ts, tm = stats(pos, spd, trq, j)
        loud = "  <<<" if ss > 5.0 else ""
        print(f"    J{j+1} {pp:12.4f} {ss:15.3f} {ts:13.4f} {tm:9.3f}{loud}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--watch", type=int, default=2, help="joint index (2 = J3)")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--gains", action="store_true",
                    help="also try J2's velocity-loop gains on the watched joint")
    ap.add_argument("--hold", type=float, default=0.0,
                    help="seconds of velocity-hold with live readout — NUDGE the joint by "
                         "hand to try to start the limit cycle")
    ap.add_argument("--perturb", action="store_true",
                    help="jog the joint briefly before holding, to inject a disturbance")
    ap.add_argument("--set-gains", type=str, default="",
                    help="hold these velocity-loop gains for the WHOLE run, e.g. "
                         "0.006,0.0005,2 (KPV,KIV,KPIQ). The confirming test: if the "
                         "oscillation cannot be STARTED at these gains, the gains fixed it. "
                         "A single quiet window does not prove that, because it is bistable.")
    ap.add_argument("--repeat", type=int, default=1,
                    help="retry the velocity-hold window N times; it is bistable, so one "
                         "quiet run proves nothing")
    a = ap.parse_args()
    j = a.watch

    robot = RobotClient()
    robot.connect(a.host)
    try:
        st = robot.status()
        print("=" * 92)
        print(f"VELOCITY-HOLD BUZZ — watching J{j+1}   mode={st.get('mode')} "
              f"state={st.get('state')}")
        print("=" * 92)
        if st.get("state") != "ENABLED":
            print("ABORT: robot must be ENABLED (IDLE uses velocity-hold when disabled,")
            print("       so the two holds would not differ).")
            return 1
        pose = [round(math.degrees(v), 2) for v in rd("joint_position")]
        print(f"  pose: {pose}")
        print("  NOTE: gravity load is pose-dependent. If this is tuning, moving the arm")
        print("        so J3 carries less weight should quieten it. Re-run at a few poses.\n")

        held = None
        if a.set_gains:
            try:
                kpv, kiv, kpiq = [float(x) for x in a.set_gains.split(",")]
            except ValueError:
                print(f"ABORT: --set-gains wants KPV,KIV,KPIQ, got {a.set_gains!r}")
                return 1
            robot.set_pid_gains(j, KPV=kpv, KIV=kiv, KPIQ=kpiq)
            held = (kpv, kiv, kpiq)
            time.sleep(1.0)
            print(f"  HOLDING gains on J{j+1} for the whole run: "
                  f"KPV={kpv} KIV={kiv} KPIQ={kpiq}")
            print(f"  Stock is KPV=0.015 KIV=0.0015 KPIQ=7.")
            print("  Now try hard to START the oscillation. Failing to start it IS the result.\n")

        print("A. gravity-comp TORQUE hold (gravity_comp ON) — the loop is NOT in circuit")
        robot.set_gravity_comp(True); time.sleep(1.0)
        pa = sample(a.seconds)
        quiet = row("gravity-comp torque hold", *pa, watch=j)

        print("\nB. VELOCITY hold (gravity_comp OFF) — the same hold ACTIVE_ERROR uses")
        print("   LISTEN NOW.")
        robot.set_gravity_comp(False); time.sleep(1.0)

        if a.hold > 0:
            print(f"   Holding {a.hold:g}s. NUDGE J{j+1} BY HAND — the limit cycle is")
            print("   bistable and needs a disturbance to start.")
            t0, worst = time.monotonic(), 0.0
            while time.monotonic() - t0 < a.hold:
                p_, s_, t_ = sample(0.5)
                ss = stats(p_, s_, t_, j)[1]
                worst = max(worst, ss)
                bar = "#" * min(60, int(ss))
                print(f"\r   speed sd {ss:8.3f} deg/s  worst {worst:8.3f}  {bar:<60}",
                      end="", flush=True)
            print()

        pb, loud = None, 0.0
        for attempt in range(max(1, a.repeat)):
            if a.perturb:
                # A brief jog then a stop is the disturbance the limit cycle needs. Jog
                # runs in its own mode, so hand the arm back to IDLE before measuring.
                robot.set_gravity_comp(True); time.sleep(0.3)
                try:
                    robot.enter_jog_mode(); time.sleep(0.3)
                    robot.jog(j, 1, 15); time.sleep(0.35)
                    robot.jog_stop(); time.sleep(0.3)
                    robot.enter_idle_mode(); time.sleep(0.3)
                except Exception as exc:
                    print(f"   (perturb skipped: {exc})")
                robot.set_gravity_comp(False); time.sleep(0.8)
            pb = sample(a.seconds)
            got = stats(*pb, j)[1]
            tag = f"velocity hold, stock gains" + (f" (try {attempt+1})" if a.repeat > 1 else "")
            got = row(tag, *pb, watch=j)
            loud = max(loud, got)
            if got >= 5.0:
                break
        all_joints("velocity hold", *pb)

        # REFUSE to draw a conclusion from a test that did not reproduce the symptom.
        # The first version compared quiet against quiet and printed "NOT tuning" from it.
        # An experiment that did not run is not evidence either way.
        REPRO = 5.0
        if held is not None:
            print("\n" + "=" * 92)
            if loud < REPRO:
                print(f"  COULD NOT START IT at KPV={held[0]} after {max(1,a.repeat)} "
                      f"attempts (worst {loud:.3f} deg/s).")
                print("  The oscillation reproduces reliably at stock gains and not at these.")
                print("  -> TUNING. Set these in robots/PAR6.xml for this joint.")
            else:
                print(f"  IT STILL OSCILLATES at KPV={held[0]} ({loud:.1f} deg/s).")
                print("  These gains are not low enough, or gains are not the whole story.")
            print("=" * 92)
            robot.set_pid_gains(j, KPV=0.015, KIV=0.0015, KPIQ=7)
            print("  stock gains restored")
            return 0 if loud < REPRO else 3

        if loud < REPRO:
            print(f"\n  *** NOT REPRODUCED — velocity hold measured {loud:.3f} deg/s, "
                  f"below the {REPRO:g} threshold.")
            print("      The oscillation is BISTABLE: it needs a disturbance to start, then")
            print("      sustains. Same pose and gains can be quiet or loud.")
            print("      No verdict is possible from this run. To trigger it:")
            print(f"        --perturb   jog J{j+1} briefly, then hold (injects the kick)")
            print(f"        --hold 30   long window; NUDGE the joint by hand and watch")
            if a.gains:
                print("      Skipping the gain comparison — there is nothing to compare against.")
            return 2

        result = None
        if a.gains:
            print(f"\nC. VELOCITY hold with J2's gains on J{j+1} (KPV 0.010, KIV 0.0010, KPIQ 2)")
            print("   THE DISCRIMINATOR: quieter here means TUNING, not a sensor.")
            robot.set_pid_gains(j, KPV=0.010, KIV=0.0010, KPIQ=2)
            time.sleep(1.0)
            pc = sample(a.seconds)
            result = row("velocity hold, J2 gains", *pc, watch=j)
            print(f"\nD. even softer (KPV 0.006, KIV 0.0005, KPIQ 2)")
            robot.set_pid_gains(j, KPV=0.006, KIV=0.0005, KPIQ=2)
            time.sleep(1.0)
            pd = sample(a.seconds)
            soft = row("velocity hold, soft gains", *pd, watch=j)
        # restore
        robot.set_gravity_comp(True)
        if a.gains:
            robot.set_pid_gains(j, KPV=0.015, KIV=0.0015, KPIQ=7)
            print("\n  gains restored (they are RAM-only and reload from the XML on boot anyway)")
        time.sleep(0.8)

        print("\n" + "=" * 92)
        print(f"  torque hold        speed sd {quiet:8.3f} deg/s")
        print(f"  velocity hold      speed sd {loud:8.3f} deg/s   "
              f"({loud/max(quiet,1e-9):6.0f}x)")
        if result is not None:
            print(f"  + J2 gains         speed sd {result:8.3f} deg/s   "
                  f"({result/max(quiet,1e-9):6.0f}x)")
            print(f"  + soft gains       speed sd {soft:8.3f} deg/s   "
                  f"({soft/max(quiet,1e-9):6.0f}x)")
            print()
            if soft < loud * 0.5:
                print("  VERDICT: softer gains quietened it -> TUNING. J3 runs J1's velocity")
                print("           gains while carrying J2-like load. Fix in robots/PAR6.xml.")
            else:
                print("  VERDICT: gains barely moved it -> NOT tuning. The loop is chasing a")
                print("           bad signal; suspect the sensor. Run tools/joint_quiet_check.py")
                print("           with the runtime down, and compare encoder wander to peers.")
        return 0
    finally:
        try:
            robot.set_gravity_comp(True)
            robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
