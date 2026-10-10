#!/usr/bin/env python3
"""27: SOFT_RTI bring-up — the low-rate streaming contract, on hardware.

FIRST hardware run of everything built 2026-08-19/20: the RTI/SOFT_RTI mode split, three
position generators, three pacing mechanisms, and the runtime command timeout. Every number
behind those was off-hardware simulation against a perfect plant. This is where that stops
being true.

Ordered safest-first, and PHASE 2 IS THE WHOLE TEST IN MINIATURE: it streams the arm's OWN
measured position at 20 Hz, so a correct system produces no motion at all while exercising
the mode, the claim, the timeout sizing, the packet path and the generator. If the arm
twitches there, stop — something is wrong and nothing has moved yet.

    # 1. Zero motion. Run this first, always.
    python3 api_tests/27_soft_rti.py --host 192.168.0.100

    # 2. Small real motion on the wrist, once phase 2 is clean.
    python3 api_tests/27_soft_rti.py --host 192.168.0.100 --move

    # 3. Compare all three generators on the same motion.
    python3 api_tests/27_soft_rti.py --host 192.168.0.100 --move --compare

    # 4. The teleop case: travel, then stop dead. Measures real overshoot.
    python3 api_tests/27_soft_rti.py --host 192.168.0.100 --move --stop-test

Needs the arm ENABLED and HOMED. Keep the E-stop within reach: --move streams setpoints to
the motors. Motion is confined to one joint (default 5, the wrist) and +/- --amp degrees.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robotclient import RobotClient, RTIClient, ControlLost

RATE = 20.0          # Hz. NOT 20-divides-250 (12.5 ticks/cmd) -- see the note in phase 3.
# A rate is "clean" when the loop rate divides by it EXACTLY -- commands then land the same
# number of ticks apart every time. A non-integer ratio makes them alternate between two
# tick counts, which corrupts every velocity estimate the robot derives from the stream
# (85% derived-feedforward blow-up at 100 Hz, 2481 jerk spike at 20 Hz).
# Computed, not listed: the first version hardcoded (50, 25, 10) and so warned about 5 Hz,
# which is 50.0 ticks/command and perfectly clean.
LOOP_HZ = 250.0


def deg(r):
    return r * 180.0 / math.pi


def stream(rti, shape, seconds, rate_hz, joint, q0, hold=False):
    """Send targets at rate_hz. shape(t)->offset in RAD. Returns (commanded, measured, all).

    `hold=True` streams the arm's LIVE measured pose every tick rather than a base captured
    before the claim. That distinction matters more than it looks: the arm sits on a
    gravity-comp TORQUE hold in IDLE (handle_idle sets trq_nm only, no position), so it
    floats and drifts -- most on joint 2, the shoulder, which carries the most gravity and
    is therefore most exposed to model error. A base pose captured before the claim is
    already stale by the time control starts, and streaming it commands the arm BACK to
    where it used to be. api_tests/25_rti_comm.py has always done it the live way; this
    script did not, and a 5 deg shoulder move on the first hardware run was the result.

    `all_j` records every joint, not just the one under test -- the same 5 deg move was
    invisible because only `joint` was being watched.
    """
    period = 1.0 / rate_hz
    cmd, meas, all_j = [], [], []
    base = list(q0)
    t0 = time.monotonic()
    nxt = t0
    while True:
        t = time.monotonic() - t0
        if t >= seconds:
            break
        st = rti.read_state()          # non-blocking, drains to the newest
        if st and isinstance(st.get("j_p"), list):
            meas.append((t, st["j_p"][joint]))
            all_j.append((t, list(st["j_p"])))
            if hold:
                base = list(st["j_p"])     # command where it IS, not where it was
        q = list(base)
        q[joint] = base[joint] + shape(t)
        rti.send_joint_position(q)
        cmd.append((t, q[joint]))
        nxt += period
        slp = nxt - time.monotonic()
        if slp > 0:
            time.sleep(slp)
    return cmd, meas, all_j


def keep_alive(rti, q0, rate_hz, seconds):
    """Stream the hold pose. NEVER go silent while holding the claim.

    A claimed session has no idle state: if you are not streaming, you are dead. Phases 4-6
    used time.sleep(0.2) after a settings change, which at 25 Hz exceeds the 0.12 s command
    timeout -- the watchdog latched RTI_LINK_LOST, the arm stopped, and the "capped" TCP
    measurement was really a stopped arm being mistaken for a working speed limit.
    """
    period = 1.0 / rate_hz
    end, nxt = time.monotonic() + seconds, time.monotonic()
    while time.monotonic() < end:
        rti.send_joint_position(q0)
        nxt += period
        slp = nxt - time.monotonic()
        if slp > 0:
            time.sleep(slp)


def report(label, cmd, meas, q0j):
    """Print peak excursion and, if we have feedback, tracking error."""
    if not cmd:
        print(f"    {label}: no commands sent")
        return
    peak_cmd = max(abs(c - q0j) for _, c in cmd)
    line = f"    {label:28} commanded peak {deg(peak_cmd):6.2f} deg"
    if meas:
        peak_meas = max(abs(m - q0j) for _, m in meas)
        # Pair each measurement with the command nearest in time.
        err = []
        ci = 0
        for tm, m in meas:
            while ci + 1 < len(cmd) and cmd[ci + 1][0] <= tm:
                ci += 1
            err.append(abs(m - cmd[ci][1]))
        line += (f"   measured peak {deg(peak_meas):6.2f} deg"
                 f"   |lag| mean {deg(sum(err)/len(err)):5.3f} max {deg(max(err)):5.3f} deg")
    print(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--rate", type=float, default=RATE, help="PC command rate in Hz")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--joint", type=int, default=5, help="joint to move (default 5 = wrist)")
    ap.add_argument("--amp", type=float, default=5.0, help="motion amplitude in DEGREES")
    ap.add_argument("--move", action="store_true",
                    help="PHASE 3+: actually move the arm. Without this the test is zero-motion.")
    ap.add_argument("--compare", action="store_true",
                    help="PHASE 4: run the same motion through all three generators")
    ap.add_argument("--stop-test", action="store_true",
                    help="PHASE 6: travel then halt; measures real stop-overshoot")
    ap.add_argument("--tcp-cap", type=float, default=0.0,
                    help="PHASE 5: enable the TCP speed cap at this m/s (0 = skip)")
    a = ap.parse_args()

    amp = math.radians(a.amp)
    print("=" * 78)
    print(f"SOFT_RTI BRING-UP   host={a.host}  rate={a.rate:g} Hz  joint={a.joint}")
    print("=" * 78)
    ticks = LOOP_HZ / a.rate
    if abs(ticks - round(ticks)) > 1e-9:
        clean = [LOOP_HZ / n for n in (2, 5, 10, 25, 50) if LOOP_HZ / n <= 125]
        print(f"NOTE: {a.rate:g} Hz is {ticks:.1f} ticks/command — NOT an integer ratio.")
        print(f"      Commands will land alternately on two tick counts, which corrupts")
        print(f"      every velocity estimate the robot derives from the stream.")
        print(f"      Clean rates: {', '.join(f'{r:g}' for r in clean)} Hz.")
        print()
    else:
        print(f"      {a.rate:g} Hz = {ticks:.0f} ticks/command exactly (clean ratio).")
        print()

    robot = RobotClient()
    robot.connect(a.host, a.port)
    try:
        st = robot.status()
        print(f"[0] mode={st.get('mode')} state={st.get('state')} homed={st.get('homed')}")
        if st.get("state") != "ENABLED":
            print("    ABORT: robot is not ENABLED — rti_claim would be refused.")
            return 1
        if not st.get("homed"):
            print("    ABORT: robot is not homed — RTI modes require it.")
            return 1
        if st.get("err"):
            print("    ABORT: an error is latched — clear it first.")
            return 1

        # -- PHASE 1: enter SOFT_RTI. Zero motion, and it must REPORT itself. ----
        rti = RTIClient(robot, clock="pc")
        rti.connect(rate_hz=a.rate)
        time.sleep(0.3)
        st = robot.status()
        print(f"[1] paired — mode={st.get('mode')}")
        if st.get("mode") != "SOFT_RTI":
            print(f"    ABORT: expected SOFT_RTI, got {st.get('mode')!r}. The rate did not "
                  f"reach the mode selector.")
            return 1
        print("    SOFT_RTI entered. Timeout was sized for this rate on entry.")

        state = rti.read_once(timeout=2.0)
        q0 = state.get("j_p")
        if not isinstance(q0, list):
            print(f"    ABORT: 'j_p' missing from the state packet (got {q0!r}). The active "
                  f"recipe must carry joint positions.")
            return 1
        print(f"    start pose: {[round(deg(v), 2) for v in q0]} deg")

        # -- PHASE 2: hold position at the PC's rate. MUST NOT MOVE. ------------
        print(f"\n[2] HOLD at {a.rate:g} Hz for {a.seconds:g}s — the arm must NOT move.")
        print("    This exercises mode, claim, timeout sizing, packet path and generator")
        print("    while commanding zero motion. It is the whole test in miniature.")
        rti.claim()
        # Re-read AFTER the claim. The pose captured before it is stale by however long the
        # claim took, and commanding a stale pose is a real step the arm really tracks --
        # one that fits inside the 0.1 rad start-pose tolerance and so passes the guard.
        q0 = rti.read_once(timeout=1.0)["j_p"]
        cmd, meas, all_j = stream(rti, lambda t: 0.0, a.seconds, a.rate, a.joint, q0,
                                  hold=True)
        if all_j:
            first = all_j[0][1]
            worst = [max(abs(row[j] - first[j]) for _, row in all_j)
                     for j in range(len(first))]
            print("    per-joint drift (deg): " +
                  "  ".join(f"J{j+1} {deg(w):6.3f}" for j, w in enumerate(worst)))
            wj = max(range(len(worst)), key=lambda j: worst[j])
            if deg(worst[wj]) > 0.5:
                print(f"    *** ABORT: joint {wj+1} moved {deg(worst[wj]):.3f} deg while "
                      f"being told to hold. Stop here. ***")
                rti.release()
                return 1
            print(f"    OK — held position. Worst joint was J{wj+1} at "
                  f"{deg(worst[wj]):.4f} deg.")
        else:
            print("    WARNING: no state feedback parsed; cannot confirm it held.")

        if not a.move:
            print("\nZero-motion phases complete. Re-run with --move for phases 3-6.")
            rti.release()
            robot.enter_idle_mode()
            robot.disconnect()
            return 0

        # -- PHASE 3: small real motion, default generator for SOFT_RTI ---------
        print(f"\n[3] MOTION — joint {a.joint}, +/-{a.amp:g} deg sine at {a.rate:g} Hz")
        shape = lambda t: amp * math.sin(2.0 * math.pi * 0.25 * t)
        cmd, meas, all_j = stream(rti, shape, a.seconds, a.rate, a.joint, q0)
        report("ruckig_advancing (default)", cmd, meas, q0[a.joint])

        # -- PHASE 4: the same motion through every generator -------------------
        if a.compare:
            print("\n[4] GENERATOR COMPARISON — same motion, same rate")
            for gen, kw in (("ruckig", {}),
                            ("tracker", {}),
                            ("ruckig_advancing", {}),
                            ("ruckig_advancing", {"min_duration": 1.5})):
                robot.rti_set_min_duration(kw.get("min_duration", 0.0))
                robot.rti_set_servo_generator(gen)
                keep_alive(rti, q0, a.rate, 0.4)
                cmd, meas, all_j = stream(rti, shape, a.seconds, a.rate, a.joint, q0)
                tag = gen + (f" + min_dur {kw['min_duration']}" if "min_duration" in kw else "")
                report(tag, cmd, meas, q0[a.joint])
            robot.rti_set_min_duration(0.0)
            robot.rti_set_servo_generator("ruckig_advancing")

        # -- PHASE 5: the TCP speed cap, which has NEVER been verified ----------
        if a.tcp_cap > 0.0:
            print(f"\n[5] TCP SPEED CAP at {a.tcp_cap:g} m/s — the arm should CRAWL.")
            print("    Built 2026-08-18, ships disabled, never verified on hardware.")
            robot.rti_set_tcp_vel_limit(a.tcp_cap)
            keep_alive(rti, q0, a.rate, 0.4)
            cmd, meas, all_j = stream(rti, shape, a.seconds, a.rate, a.joint, q0)
            report(f"capped at {a.tcp_cap:g} m/s", cmd, meas, q0[a.joint])
            print("    Compare the measured peak against phase 3: it should be much smaller.")
            robot.rti_set_tcp_vel_limit(0.0)

        # -- PHASE 6: the teleop case — travel, then stop dead ------------------
        if a.stop_test:
            print("\n[6] STOP TEST — travel, then halt. Measures REAL overshoot.")
            print("    Simulation says advancing/tracker sail ~2.7 deg past a stop at 5 Hz,")
            print("    and that min_duration 1.5x removes it. This is that claim, on metal.")
            half = a.seconds / 2.0
            ramp = lambda t: amp * min(t / half, 1.0)
            for tag, mult in (("advancing", 0.0), ("advancing + min_dur 1.5", 1.5)):
                robot.rti_set_min_duration(mult)
                robot.rti_set_servo_generator("ruckig_advancing")
                keep_alive(rti, q0, a.rate, 0.4)
                cmd, meas, all_j = stream(rti, ramp, a.seconds, a.rate, a.joint, q0)
                if meas:
                    after = [m for t, m in meas if t > half]
                    if after:
                        target = q0[a.joint] + amp
                        ov = max(0.0, max(after) - target)
                        print(f"    {tag:28} overshoot past the stop: {deg(ov):5.2f} deg")
            robot.rti_set_min_duration(0.0)

        rti.release()
        # Return to IDLE. Leaving the robot parked in SOFT_RTI means the next run's
        # rti_connect is a same-mode request, and it also leaves the arm on a POSITION hold
        # when the operator probably expects IDLE's gravity-comp hold.
        robot.enter_idle_mode()
        print("\nDone. Released control, back to IDLE.")
        return 0

    except ControlLost as exc:
        print(f"\n*** CONTROL LOST: {exc}")
        print("    The robot stopped itself. Check the timeout against your actual rate.")
        return 1
    finally:
        try:
            robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
