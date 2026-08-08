#!/usr/bin/env python3
"""25: RTI mode communication test — link quality, ZERO MOTION.

A zero-motion link-quality check, and the first thing to run
on hardware after RTI mode is deployed (TODO/RTI_mode_plan.md §18b).

WHAT IT DOES TO THE ROBOT: nothing. It claims real-time control and then streams the arm's
own measured position straight back to it, tick after tick. The commanded setpoint is
always "stay exactly where you are", so a correct system produces no motion at all. That is
what makes it safe to run first — it exercises the entire stack (TCP handshake, UDP both
directions, recipe parsing, rate limiter, start-pose check, timeout, success rate) without
ever asking the arm to go anywhere.

It still needs the arm ENABLED, because claiming control on a disabled robot is refused.
Keep the E-stop within reach anyway: this is the first code in the project that streams
setpoints to the motors.

    # Stage 1 — on the Pi, over loopback. Proves the protocol works.
    python3 api_tests/25_rti_comm.py

    # Stage 2 — from the real control PC. Now the numbers describe your network.
    python3 api_tests/25_rti_comm.py --host 192.168.0.100

    # Longer soak
    python3 api_tests/25_rti_comm.py --seconds 30

Read the verdict at the end, not just the numbers.
"""

import argparse
import math
import os
import statistics
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robotclient import (RobotClient, RTIClient, JointPositionCommand,
                         MotionFinished, ControlLost)

SHM = "/dev/shm"


# ---------------------------------------------------------------------------- #
# Robot-side truth, readable only when this runs ON the robot.
# The PC cannot see success_rate / miss counters any other way — they are not in
# the recipe payload — so a remote run reports PC-side timing only.
# ---------------------------------------------------------------------------- #

def shm_available():
    return os.path.exists(f"{SHM}/rti_substate")


def shm_read(name):
    with open(f"{SHM}/{name}", "rb") as f:
        return struct.unpack("<d", f.read(8))[0]


def robot_side_stats():
    if not shm_available():
        return None
    return {
        "substate":    int(shm_read("rti_substate")),
        "success":     shm_read("control_success_rate"),
        "misses":      int(shm_read("rti_consecutive_misses")),
        "worst_miss":  int(shm_read("rti_worst_consecutive_misses")),
    }


SUBSTATE_NAME = {0: "unpaired", 1: "connected", 2: "control_active",
                 3: "stopping_clean", 4: "stopping_error"}


def summarise(name, samples_ms):
    """Report a timing distribution. Mean hides everything that matters here."""
    if not samples_ms:
        print(f"  {name:26} no samples")
        return None
    s = sorted(samples_ms)
    p = lambda q: s[min(len(s) - 1, int(len(s) * q))]
    print(f"  {name:26} n={len(s):<6} min {s[0]:7.3f} | med {p(0.50):7.3f} | "
          f"p95 {p(0.95):7.3f} | p99 {p(0.99):7.3f} | max {s[-1]:7.3f}  ms")
    return {"n": len(s), "min": s[0], "med": p(0.50), "p95": p(0.95),
            "p99": p(0.99), "max": s[-1],
            "jitter": statistics.pstdev(s) if len(s) > 1 else 0.0}


def _clear_latched(robot, why):
    """Restore the robot to the state the test found it in: no error, ENABLED.

    Both safety tests deliberately latch an error, and a latched error DISABLES the robot.
    Clearing the error returns the mode to IDLE but leaves it DISABLED — so without the
    re-enable below, the next test aborts at phase 0 with "robot is not ENABLED", which is
    exactly what happened on the first run of this.
    """
    print(f"    restoring: clearing the latched error ({why}) and re-enabling")
    try:
        robot.clear_errors()
        time.sleep(0.3)
        st = robot.status()
        if st.get("err"):
            print(f"    STILL LATCHED (err={st.get('err')}) — clear it from the GUI")
            return False
        # Re-enable only because phase 0 already established the robot WAS enabled when we
        # started. This restores that, it does not enable something the operator had off.
        robot.enable()
        time.sleep(0.4)
        st = robot.status()
        ok = st.get("state") == "ENABLED"
        print(f"    {'restored' if ok else 'NOT re-enabled — enable it from the GUI'} "
              f"(err={st.get('err')}, state={st.get('state')}, mode={st.get('mode')})")
        return ok
    except Exception as exc:
        print(f"    restore failed: {type(exc).__name__}: {exc}")
        return False


def _test_timeout(robot, rti, a):
    """Claim control, then go silent. The robot must stop itself. ZERO MOTION.

    This is the watchdog that protects you when the controlling PC dies mid-stream —
    it has never once fired on hardware, so 'it works' is currently an assumption.
    """
    print("\n[4] SAFETY TEST: command timeout")
    print("    Streaming for 0.5 s to establish control, then going deliberately silent.")
    print("    Expect: robot stops itself after ~10 ticks (40 ms) and latches an error.")
    rti.claim()

    sent = [0]
    silent_at = [None]

    def go_silent(state):
        # Returning None sends nothing — the PC falls silent while still receiving,
        # which is exactly what a crashed or wedged controller looks like.
        if state.elapsed < 0.5:
            sent[0] += 1
            return JointPositionCommand(state["j_p"])
        if silent_at[0] is None:
            silent_at[0] = time.perf_counter()
        return None

    t_raise, err = None, None
    try:
        # Leave `timeout` at its 0.1 s default. It is the steady-state recv timeout —
        # how quickly we notice the state stream stopping — and this test passing 2.0
        # made a 40 ms event take 2 s to detect. Detection is now
        # (robot trips: ~44 ms) + (recv timeout: 100 ms) ~= 145 ms.
        rti.control(go_silent)
        print("    ✗ FAIL: control() returned without raising — the robot never stopped us.")
    except ControlLost as exc:
        t_raise, err = time.perf_counter(), exc
    except RuntimeError as exc:
        print(f"    ✗ FAIL: got a plain RuntimeError, not ControlLost: {exc}")
        _clear_latched(robot, "timeout test")
        return 1

    # Read the latch BEFORE clearing it. The previous version cleared first and then
    # reported "error latched: False" — it had wiped the thing it was testing.
    st_after = robot.status()
    latched = bool(st_after.get("err"))

    if err is None:
        _clear_latched(robot, "timeout test")
        return 1

    detect_ms = (t_raise - silent_at[0]) * 1e3 if silent_at[0] else float("nan")
    print(f"    ✓ ControlLost raised {detect_ms:.0f} ms after we stopped sending")
    print(f"      {err}")
    print(f"      ({sent[0]} commands sent before going silent)")

    rs = robot_side_stats()
    if rs:
        print(f"      robot-side worst miss streak: {rs['worst_miss']} ticks")

    print(f"    {'✓' if latched else '✗'} error latched: {latched}"
          f"{'' if latched else '   <- expected a latch after an ERROR stop'}")
    print(f"      robot mode after the stop: {st_after.get('mode')} "
          f"(a hard error forces ACTIVE_ERROR, which is why the state stream ends)")

    # 40 ms budget + up to a tick of detection lag + the PC's own recv scheduling.
    # Generous upper bound: this is a sanity check, not a timing benchmark.
    sane = 20.0 <= detect_ms <= 400.0
    print(f"    {'✓' if sane else '✗'} detection latency {detect_ms:.0f} ms is plausible "
          f"(40 ms budget + PC scheduling)")

    ok = latched and sane
    _clear_latched(robot, "timeout test")
    print("\n" + "=" * 78)
    print("PASS: the command watchdog works — a dead PC will not leave the arm driven."
          if ok else "Investigate above before trusting the timeout.")
    print("=" * 78)
    return 0 if ok else 1


def _test_start_pose(robot, rti, a):
    """Send a deliberately wrong FIRST setpoint; the robot must refuse control.

    RISK: if the check is broken the offset is applied. It is ramped by the rate limiter
    rather than jumped, and defaults to the wrist, but this is the one test here that can
    move the arm. 0.12 rad = 6.9 deg, chosen as the smallest offset that clears the
    0.1 rad tolerance.
    """
    OFFSET = 0.12
    j = a.sp_joint
    print("\n[4] SAFETY TEST: start-pose rejection")
    print(f"    Sending a first setpoint {OFFSET} rad ({OFFSET*57.3:.1f} deg) off on joint {j}.")
    print("    Expect: robot REFUSES control and does not move.")
    print(f"    If the check is broken, joint {j} ramps {OFFSET*57.3:.1f} deg. Hand on the E-stop.")
    rti.claim()

    first = [None]

    def bad_first(state):
        q = list(state["j_p"])
        if first[0] is None:
            first[0] = q[j]
        q[j] += OFFSET
        return JointPositionCommand(q)

    err = None
    try:
        rti.control(bad_first)
        print("    ✗ FAIL: control() returned without raising — the bad setpoint was ACCEPTED.")
    except ControlLost as exc:
        err = exc
    except RuntimeError as exc:
        print(f"    ✗ FAIL: plain RuntimeError, not ControlLost: {exc}")

    if err is None:
        _clear_latched(robot, "start-pose test")
        return 1

    print(f"    ✓ control refused: {err}")

    # Did it actually stay put? The whole point is that refusing means NOT moving.
    #
    # Read it over TCP, not from the UDP state stream. Refusing control latches an error,
    # which forces ACTIVE_ERROR, which takes the robot out of RTI mode and STOPS the stream —
    # so read_once() has nothing to read and this check used to fail on its own plumbing
    # while the safety feature underneath was working perfectly.
    # get_joints() is DEGREES at the API; j_p on the wire is radians.
    time.sleep(0.3)
    try:
        after_rad = math.radians(robot.get_joints()[j])
        moved = abs(after_rad - first[0])
        held = moved < 0.01
        print(f"    {'✓' if held else '✗'} joint {j} moved {math.degrees(moved):.3f} deg "
              f"({'held, as intended' if held else 'MOVED — the refusal did not prevent motion'})")
    except Exception as exc:
        held = False
        print(f"    could not re-read position over TCP: {type(exc).__name__}: {exc}")

    st = robot.status()
    print(f"    error latched: {bool(st.get('err'))}")
    _clear_latched(robot, "start-pose test")

    print("\n" + "=" * 78)
    print("PASS: a bad first setpoint is refused before it reaches a motor."
          if held else "Investigate above before commanding motion.")
    print("=" * 78)
    return 0 if held else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1",
                    help="robot IP. Default 127.0.0.1 = running on the Pi itself.")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="duration of each measurement phase")
    ap.add_argument("--test-timeout", action="store_true",
                    help="SAFETY TEST, zero motion: claim control, then deliberately stop "
                         "sending. The robot should warn at 5 ticks and stop itself at 10 "
                         "(40 ms), latching an error. Verifies the watchdog that protects "
                         "you when the controlling PC dies mid-stream.")
    ap.add_argument("--test-start-pose", action="store_true",
                    help="SAFETY TEST: send a deliberately WRONG first setpoint and expect "
                         "the robot to refuse control. If the check works, nothing moves. "
                         "If it is broken, joint --sp-joint moves ~7 deg (ramped by the rate "
                         "limiter, not a jump). Defaults to the wrist for that reason.")
    ap.add_argument("--sp-joint", type=int, default=5,
                    help="joint the start-pose test offsets (default 5 = wrist, lowest "
                         "consequence if the check fails)")
    ap.add_argument("--no-claim", action="store_true",
                    help="stop after passive streaming; never claim control. Use this if "
                         "you want to watch state packets without arming anything.")
    a = ap.parse_args()

    local = a.host in ("127.0.0.1", "localhost")
    print("=" * 78)
    print(f"RTI COMMUNICATION TEST   host={a.host}  {'(loopback)' if local else '(network)'}")
    print("Streams the arm's own measured position back to it — expect NO movement.")
    print("=" * 78)

    robot = RobotClient()
    robot.connect(a.host, a.port)
    rti = None
    claimed = False
    try:
        # -- Phase 0: is the robot in a state where this can work at all? -------
        st = robot.status()
        print(f"\n[0] mode={st.get('mode')} state={st.get('state')} homed={st.get('homed')} "
              f"err={st.get('err')}")
        if st.get("state") != "ENABLED":
            print("    ABORT: robot is not ENABLED — rti_claim would be refused.")
            return 1
        if not st.get("homed"):
            print("    ABORT: robot is not homed — rti_connect requires it.")
            return 1
        if st.get("err"):
            print("    ABORT: an error is latched — clear it first.")
            return 1

        # -- Phase 1: pair. Robot switches to RTI mode and starts unicasting. ---
        rti = RTIClient(robot, clock="robot")
        rti.connect()
        print(f"[1] paired — robot={rti.robot_ip} state packets -> this host:30002")
        time.sleep(0.3)
        rs = robot_side_stats()
        if rs:
            print(f"    robot-side substate: {SUBSTATE_NAME.get(rs['substate'], '?')}")

        # -- Phase 2: does a state packet arrive, and does it parse? ------------
        # The single most likely thing to be broken, and it costs one packet to find out.
        state = rti.read_once(timeout=2.0)
        tags = sorted(state.keys())
        print(f"[2] first state packet OK — tags: {tags}")
        jp = state.get("j_p")
        if not isinstance(jp, list):
            print(f"    ABORT: 'j_p' missing or not a vector (got {jp!r}). The active recipe "
                  f"must include joint positions for the hold-position controller to work.")
            return 1
        print(f"    joint positions: {[round(v, 4) for v in jp]}")

        # -- Phase 3: passive stream. No claim, so no timing rules are enforced. -
        print(f"\n[3] measuring the robot->PC stream for {a.seconds:.0f}s (control NOT claimed)")
        # Drain first. The robot has been unicasting since phase 1, so by now the socket
        # holds every packet sent during the handshake and phase 2 — at 250 Hz that is ~75
        # of them. Measuring without draining reads that backlog back-to-back and inflates
        # the packet count by exactly the pairing delay (observed: 73 extra vs 75 predicted).
        # Percentiles survive it; a rate computed from the count does not.
        _drained = 0
        while rti.read_state() is not None:
            _drained += 1
        if _drained:
            print(f"    (drained {_drained} queued packets from the pairing delay first)")
        gaps, last, n = [], None, 0
        t_end = time.monotonic() + a.seconds
        while time.monotonic() < t_end:
            try:
                rti.read_once(timeout=1.0)
            except RuntimeError:
                print("    WARNING: gap > 1 s with no state packet")
                continue
            now = time.perf_counter()
            if last is not None:
                gaps.append((now - last) * 1e3)
            last, n = now, n + 1
        _elapsed = time.monotonic() - (t_end - a.seconds)
        arrival = summarise("state packet interval", gaps)
        if arrival:
            # Two independent rate estimates. They should agree once the backlog is
            # drained; a gap between them means packets are arriving bunched rather than
            # one per tick, which the median alone would hide.
            print(f"  {'rate (from median gap)':26} {1000.0/arrival['med']:.1f} Hz "
                  f"(jitter σ {arrival['jitter']:.3f} ms)")
            print(f"  {'rate (from packet count)':26} {(n)/_elapsed:.1f} Hz "
                  f"({n} packets / {_elapsed:.1f}s)")

        if a.no_claim:
            print("\n--no-claim given — stopping before control is armed.")
            return 0

        # -- Safety tests. Both END the session, so they run instead of phase 4. --
        if a.test_timeout:
            return _test_timeout(robot, rti, a)
        if a.test_start_pose:
            return _test_start_pose(robot, rti, a)

        # -- Phase 4: claim and hold. This is the real round-trip test. ---------
        print(f"\n[4] claiming control and holding position for {a.seconds:.0f}s")
        print("    (the first setpoint is the CURRENT pose, so the start-pose check "
              "should pass)")
        rti.claim()
        claimed = True
        # Snapshot the byte counters here: they accumulate from pairing, and phases 1-3
        # are state-only. Dividing lifetime bytes by the claimed duration would mix a
        # one-directional phase into a two-directional rate.
        _tx0, _rx0 = rti.bytes_tx, rti.bytes_rx

        turnaround, ticks, t0 = [], 0, time.monotonic()

        def hold(s):
            """Send back exactly what the robot reported. Commands no motion."""
            nonlocal ticks
            t_recv = time.perf_counter()
            if s.elapsed > a.seconds:
                return MotionFinished()
            q = s.get("j_p")
            if not isinstance(q, list):
                return MotionFinished()      # lost the position channel — stop cleanly
            ticks += 1
            cmd = JointPositionCommand(q)
            turnaround.append((time.perf_counter() - t_recv) * 1e3)
            return cmd

        rti.control(hold)
        claimed = False                      # control() sends CMD_STOP on MotionFinished
        wall = time.monotonic() - t0
        print(f"    streamed {ticks} commands in {wall:.1f}s ({ticks/wall:.1f} Hz)")
        summarise("PC turnaround (recv->send)", turnaround)

        # -- Loop delay, from the sequence echo -----------------------------------
        lat = rti.latency_stats()
        print("\n[5] loop delay (command sent -> its echo comes back)")
        if lat is None:
            print("    unavailable — the active recipe carries no 'rti_seq' tag.")
            print("    Add <field tag=\"rti_seq\"/> to the recipe, or switch to standard/diagnostics.")
        else:
            print(f"  {'loop delay':26} n={lat['n']:<6} min {lat['min']:7.3f} | "
                  f"med {lat['med']:7.3f} | p95 {lat['p95']:7.3f} | "
                  f"p99 {lat['p99']:7.3f} | max {lat['max']:7.3f}  ms")
            # TWO ticks are structural, not one (plan §1.1). Tick T+1 reads the command and
            # applies it, but the echo only goes out in T+2's state packet, because
            # _send_state() runs before _receive_command() inside the handler. Subtracting
            # one tick — as this line originally did — overstates wire RTT by a whole tick:
            # on loopback it reported ~4 ms of "wire time" across a socket to localhost.
            tick_ms = arrival['med'] if arrival else 0.0
            structural = 2.0 * tick_ms
            wire = max(0.0, lat['med'] - structural)
            print(f"  {'  structural (2 ticks)':26} ~{structural:.3f} ms "
                  f"(1 to read the command, 1 for the echo to go out)")
            print(f"  {'  approx wire round-trip':26} ~{wire:.3f} ms"
                  + ("   <- ~0 expected on loopback" if local else ""))
            print(f"  {'commands never echoed':26} {lat['lost']}")

        print("\n[6] link bandwidth during the claimed phase (measured at THIS end,"
              " so it works remotely)")
        _tx = rti.bytes_tx - _tx0
        _rx = rti.bytes_rx - _rx0
        print(f"  {'PC -> robot (commands)':26} {_tx/wall/1024:7.1f} kB/s "
              f"({_tx/max(ticks,1):.0f} B/packet)")
        print(f"  {'robot -> PC (state)':26} {_rx/wall/1024:7.1f} kB/s "
              f"(scales with the active recipe)")
        print(f"  {'total':26} {(_tx+_rx)/wall/1024:7.1f} kB/s "
              f"({(_tx+_rx)*8/wall/1e6:.2f} Mbit/s)")

        rs = robot_side_stats()
        if rs:
            print("\n[7] robot-side link quality (read from shared memory)")
            print(f"  {'success rate':26} {rs['success']*100:.1f}%")
            print(f"  {'worst consecutive misses':26} {rs['worst_miss']} ticks")
        else:
            # This used to say "try --recipe diagnostics". There is no such flag, and there
            # cannot be one today: the active recipe is a SharedString that only the GUI
            # writes — no TCP command exists to change it, so a remote PC cannot choose its
            # own telemetry. Sending an operator to a flag that does not exist, at the exact
            # moment they cannot see robot-side link quality, is worse than saying nothing.
            print("\n[7] robot-side link quality UNAVAILABLE from a remote PC.")
            print("    success rate / consecutive misses live in the robot's shared memory,")
            print("    which only a run ON the robot can read.")
            print("    To see them over the network the state packet must carry rti_ok /")
            print("    rti_miss — switch the active recipe to 'diagnostics' in the GUI")
            print("    (RT Diagnostics tab). There is deliberately no flag for it: selecting")
            print("    a recipe over TCP is not implemented — see RTI_mode_plan.md open item.")

        # -- Verdict ------------------------------------------------------------
        print("\n" + "=" * 78)
        ok = True
        if ticks == 0:
            print("FAIL: control was claimed but no command was ever sent.")
            ok = False
        if rs and rs["worst_miss"] > 0:
            # Any miss at all on loopback means something is wrong; on a network it is
            # only interesting relative to the timeout budget.
            note = "expected 0 on loopback" if local else "compare against the timeout budget"
            print(f"NOTE: worst miss streak was {rs['worst_miss']} ticks ({note})")
        if rs and rs["success"] < 0.95:
            print(f"WARN: success rate {rs['success']*100:.1f}% — the PC is not keeping up "
                  f"with the robot's tick rate, or packets are being lost.")
            ok = False
        if arrival and arrival["max"] > 3 * arrival["med"]:
            print(f"WARN: state arrival is bursty (max {arrival['max']:.1f} ms vs median "
                  f"{arrival['med']:.1f} ms) — check for CPU contention or network jitter.")
        print("PASS: link is healthy and the arm held position." if ok
              else "Investigate the warnings above before commanding any motion.")
        print("=" * 78)
        return 0 if ok else 1

    finally:
        # Hand control back no matter how we got here. An exception mid-stream would
        # otherwise leave the robot in control_active until its own timeout fires.
        try:
            if rti is not None:
                if claimed:
                    rti.stop()
                    time.sleep(0.6)          # let the 0.5 s stop-hold complete
                rti.disconnect()
                rti.close()
        except Exception as exc:
            print(f"[cleanup] {type(exc).__name__}: {exc}")
        robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
