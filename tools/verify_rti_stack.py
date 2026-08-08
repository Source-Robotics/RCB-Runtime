"""Hardware verification of everything built since the last real hardware run.

Three claims are currently believed on simulation evidence alone. Each one below can
INVALIDATE a claim rather than merely confirm it, which is the point of running it.

  A. RUCKIG CONVERGES. The old limiter left joint_position_commanded oscillating +/-0.6 deg
     around a constant target, forever. If Ruckig is live, commanded sits ON target. This is
     the single number that proves the whole limiter rewrite reached the arm.

  B. THE COMMAND PATH IS STILL LIVE. Re-proves what the 2026-08-02 liveness check proved,
     because everything underneath it has been replaced since.

  C. GRAVITY REACHES THE WIRE. RTI used to send Current=0 always. Tested differentially
     (toggle on vs off) so it does not depend on knowing the Nm->mA factor.

MOTION: one wrist joint, +0.002 rad (0.115 deg), returned to start. Everything else is a
position hold. Keep the E-stop within reach anyway.
"""
import argparse
import struct
import sys
import time

sys.path.insert(0, "/home/robot/Desktop/RCB-Runtime")

from robotclient import RobotClient, RTIClient

SHM = "/dev/shm"
N = 6


def arr(name):
    with open(f"{SHM}/{name}", "rb") as f:
        return list(struct.unpack(f"<{N}d", f.read(8 * N)))


def snap():
    return {k: arr(k) for k in ("joint_position_target", "joint_position_commanded",
                                "joint_position", "joint_gravity_torque",
                                "motor_current_sent", "motor_position_sent")}


def stream(client, pose, seconds, hz=250.0):
    period = 1.0 / hz
    t_end = time.perf_counter() + seconds
    t_next = time.perf_counter()
    while time.perf_counter() < t_end:
        if time.perf_counter() < t_next:
            continue
        t_next += period
        client.read_state()
        client.send_joint_position(pose)


def sample_commanded(client, pose, seconds, j):
    """Hold `pose` and record the spread of joint_position_commanded[j].

    The spread is the whole test: the old limiter never settled, so a wide band means the
    rewrite did not reach the arm no matter what the mean says.
    """
    period = 1.0 / 250.0
    t_end = time.perf_counter() + seconds
    t_next = time.perf_counter()
    vals, tgts = [], []
    while time.perf_counter() < t_end:
        if time.perf_counter() < t_next:
            continue
        t_next += period
        client.read_state()
        client.send_joint_position(pose)
        vals.append(arr("joint_position_commanded")[j])
        tgts.append(arr("joint_position_target")[j])
    return vals, tgts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", type=int, default=5, help="joint to nudge (default 5 = wrist)")
    ap.add_argument("--delta", type=float, default=0.002, help="rad (default 0.002 = 0.115 deg)")
    a = ap.parse_args()
    j = a.joint
    ok = True

    robot = RobotClient()
    robot.connect("127.0.0.1")
    st = robot.status()
    print(f"[0] mode={st['mode']} state={st['state']} homed={st.get('homed')} "
          f"errors={st.get('errors')}")
    if st["state"] != "ENABLED":
        print("ABORT: robot must be ENABLED and homed.")
        return 1

    client = RTIClient(robot)
    client.connect()
    first = client.read_once(timeout=2.0)
    q0 = list(first["j_p"])
    print(f"[1] paired; start pose {[round(v, 4) for v in q0]}")
    client.claim()
    print("[2] control claimed\n")

    try:
        # ---------------------------------------------------------- A. converge
        print("[A] RUCKIG CONVERGENCE — holding a CONSTANT target for 4 s")
        stream(client, q0, 1.5)                       # settle first
        vals, tgts = sample_commanded(client, q0, 4.0, j)
        import statistics as sstat
        band = max(vals) - min(vals)
        off = sstat.median(vals) - sstat.median(tgts)
        print(f"    commanded[{j}] band   {band:.6f} rad ({band * 57.2958:.4f} deg)")
        print(f"    median offset target  {off:+.6f} rad ({off * 57.2958:+.4f} deg)")
        print(f"    OLD limiter measured  ~0.020 rad band, ~0.007 rad offset")
        if band > 0.002:
            print(f"    FAIL: still oscillating — Ruckig is NOT live on this robot")
            ok = False
        elif abs(off) > 0.002:
            print(f"    FAIL: converged but sits off target by {off:+.6f} rad")
            ok = False
        else:
            print(f"    PASS: converged and on target")

        # ---------------------------------------------------------- C. gravity (ON half)
        # The OFF half runs in a SECOND claim, further down. A TCP toggle mid-stream costs a
        # round trip, and 40 ms of that is a dead link as far as the watchdog is concerned —
        # the first version of this script latched RTI_LINK_LOST on itself doing exactly that,
        # then reported the resulting paralysis as a liveness failure. Every toggle now happens
        # while NOT streaming.
        print("\n[C] GRAVITY ON THE WIRE — zero motion")
        stream(client, q0, 1.0)
        s_on = snap()
        cur_on = s_on["motor_current_sent"]
        grav = s_on["joint_gravity_torque"]
        print(f"    gravity torque (Nm)   {[round(v, 3) for v in grav]}")
        print(f"    motor_current_sent    {[None if v != v else round(v, 1) for v in cur_on]}")
        nonzero_on = sum(1 for v in cur_on if v == v and abs(v) > 1.0)
        if nonzero_on == 0:
            print("    FAIL: no current commanded with gravity ON — RTI still sends Current=0")
            ok = False
        else:
            print(f"    PASS: {nonzero_on}/{N} joints carry gravity current "
                  f"(joint 0 reads 0 correctly — vertical axis, gravity does no work on it)")

        # ---------------------------------------------------------- B. liveness
        print(f"\n[B] COMMAND PATH LIVE — joint {j} {a.delta:+.4f} rad "
              f"({a.delta * 57.2958:+.3f} deg)")
        before = snap()
        q1 = list(q0)
        q1[j] = q0[j] + a.delta
        stream(client, q1, 3.0)
        after = snap()
        d_t = after["joint_position_target"][j] - before["joint_position_target"][j]
        d_c = after["joint_position_commanded"][j] - before["joint_position_commanded"][j]
        d_a = after["joint_position"][j] - before["joint_position"][j]
        print(f"    target     {d_t:+.6f} rad")
        print(f"    commanded  {d_c:+.6f} rad")
        print(f"    ACTUAL     {d_a:+.6f} rad   <- measured encoder, the real proof")
        stream(client, q0, 3.0)
        back = snap()
        print(f"    returned; residual {back['joint_position'][j] - before['joint_position'][j]:+.6f} rad")

        tol = abs(a.delta) * 0.35
        if abs(d_a - a.delta) > tol:
            print(f"    FAIL: the arm did not follow (expected {a.delta:+.6f})")
            ok = False
        elif abs(d_c - a.delta) > tol:
            print(f"    FAIL: commanded did not follow the target")
            ok = False
        else:
            print("    PASS: the command reached the motors and the arm moved")

        for k in range(N):
            if k != j and abs(after["joint_position_commanded"][k]
                              - before["joint_position_commanded"][k]) > 1e-3:
                print(f"    FAIL: joint {k} moved but was never commanded to")
                ok = False
        # ------------------------------------------------- C. gravity (OFF half)
        # Second claim, so the toggle happens with no stream running and the watchdog has
        # nothing to complain about. Proves the toggle DOES something, which a one-sided
        # "current is non-zero" reading cannot.
        print("\n[C2] GRAVITY TOGGLE — second claim, zero motion")
        client.stop()
        time.sleep(0.7)                       # let the clean stop finish; not streaming, so safe
        robot.rti_set_gravity_comp(False)
        client.claim()
        stream(client, q0, 1.0)
        cur_off = snap()["motor_current_sent"]
        print(f"    with comp OFF         {[None if v != v else round(v, 1) for v in cur_off]}")
        client.stop()
        time.sleep(0.7)
        robot.rti_set_gravity_comp(True)      # leave the robot as we found it
        print("    gravity comp restored to ON")

        # NaN or 0 both mean "no torque commanded"; dispatch writes 0 when the channel is unset.
        if not all(v != v or abs(v) < 1.0 for v in cur_off):
            print("    FAIL: current still flowing with gravity OFF — the toggle does nothing")
            ok = False
        else:
            print("    PASS: the toggle silences it, so the ON reading was really gravity")
    finally:
        try:
            client.stop()
            time.sleep(0.7)
        except Exception:
            pass
        client.disconnect()
        client.close()

    print("\n" + "=" * 70)
    print("PASS: Ruckig, gravity and the command path are all live on hardware."
          if ok else "FAILED — see above. Do not build on the affected claims.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
