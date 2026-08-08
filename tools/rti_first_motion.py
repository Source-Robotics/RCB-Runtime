"""First real RTI motion — a smooth streamed trajectory, not a step.

Everything RTI has done on hardware so far either held position or nudged 0.115 deg. This is
the first time an external stream actually DRIVES the arm somewhere and back.

Shape: q(t) = q0 + A * env(t) * sin(2*pi*f*t), with env a raised cosine ramping 0 -> 1 -> 0.
At t=0 both the offset and the velocity are zero, so control starts from rest with no
discontinuity — the start-pose check passes and the rate limiter never sees a step. The
analytic derivative is sent as feedforward, which is what keeps tracking tight (measured
1.41 deg -> 0.48 deg on a ramp when feedforward was wired in).

SAFETY
  - both toggles stay ON (rate limiter, start-pose check) — this is not a bring-up bypass
  - the requested motion is checked against the joint's OWN limits BEFORE anything moves,
    and refused if it does not fit, with the numbers printed
  - defaults are deliberately gentle: 5 deg on the wrist, 0.25 Hz => ~4% of velocity and
    ~1% of acceleration budget
  - returns to the exact starting pose and verifies the residual

Keep the E-stop within reach.
"""
import argparse
import math
import struct
import sys
import time

sys.path.insert(0, "/home/robot/Desktop/RCB-Runtime")

import numpy as np

from robotclient import RobotClient, RTIClient

SHM = "/dev/shm"
N = 6


def arr(name):
    with open(f"{SHM}/{name}", "rb") as f:
        return list(struct.unpack(f"<{N}d", f.read(8 * N)))


def scalar(name):
    with open(f"{SHM}/{name}", "rb") as f:
        return struct.unpack("<d", f.read(8))[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", type=int, default=5, help="joint to drive (default 5 = wrist)")
    ap.add_argument("--deg", type=float, default=5.0, help="amplitude in degrees (default 5)")
    ap.add_argument("--hz", type=float, default=0.25, help="sine frequency (default 0.25)")
    ap.add_argument("--cycles", type=float, default=3.0, help="full cycles at amplitude")
    ap.add_argument("--ramp", type=float, default=2.0, help="ramp in/out seconds")
    a = ap.parse_args()
    j = a.joint
    A = math.radians(a.deg)
    w = 2.0 * math.pi * a.hz

    # ---- pre-flight: does this motion FIT, on this joint? ------------------
    import config.xml_parser as X
    from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR
    sysc = X.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = X.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                   active_gripper=sysc["active_gripper"])
    vlim = float(np.atleast_1d(cfg["rti_velocity_limit"])[j])
    alim = float(np.atleast_1d(cfg["rti_acceleration_limit"])[j])
    lo = float(np.atleast_1d(cfg["software_joint_limit_negative"])[j])
    hi = float(np.atleast_1d(cfg["software_joint_limit_positive"])[j])
    pk_v, pk_a = A * w, A * w * w

    print(f"joint {j}   amplitude {a.deg:.2f} deg   {a.hz} Hz   {a.cycles} cycles")
    print(f"  peak velocity      {pk_v:8.4f} rad/s   ({pk_v / vlim * 100:5.1f}% of {vlim})")
    print(f"  peak acceleration  {pk_a:8.4f} rad/s2  ({pk_a / alim * 100:5.1f}% of {alim})")
    if pk_v > vlim * 0.5 or pk_a > alim * 0.5:
        print("ABORT: over 50% of a limit. Reduce --deg or --hz for a first motion.")
        return 1

    robot = RobotClient()
    robot.connect("127.0.0.1")
    st = robot.status()
    print(f"\n[0] mode={st['mode']} state={st['state']} homed={st.get('homed')} "
          f"errors={st.get('errors')}")
    if st["state"] != "ENABLED" or not st.get("homed"):
        print("ABORT: robot must be ENABLED and homed.")
        return 1

    client = RTIClient(robot)
    client.connect()
    first = client.read_once(timeout=2.0)
    q0 = list(first["j_p"])
    if not (lo < q0[j] - A and q0[j] + A < hi):
        print(f"ABORT: {a.deg} deg around {q0[j]:.3f} would reach the soft limits [{lo}, {hi}].")
        client.disconnect(); client.close()
        return 1
    print(f"[1] start pose {[round(v, 4) for v in q0]}")
    print(f"    joint {j} will sweep {q0[j] - A:.4f} .. {q0[j] + A:.4f} rad "
          f"(soft range {lo:.3f} .. {hi:.3f})")

    client.claim()
    print("[2] control claimed — both safety toggles ON\n[3] moving...\n")

    dt = 1.0 / 250.0
    hold = a.cycles / a.hz
    total = 2 * a.ramp + hold
    pose = list(q0)
    ff = [0.0] * N
    t_cmd, t_act, t_tgt = [], [], []

    try:
        t0 = time.perf_counter()
        t_next = t0
        while True:
            now = time.perf_counter()
            if now < t_next:
                continue
            t_next += dt
            t = now - t0
            if t >= total:
                break

            # raised-cosine envelope: 0 -> 1 over `ramp`, flat, then 1 -> 0
            if t < a.ramp:
                env = 0.5 * (1.0 - math.cos(math.pi * t / a.ramp))
                denv = 0.5 * math.pi / a.ramp * math.sin(math.pi * t / a.ramp)
            elif t < a.ramp + hold:
                env, denv = 1.0, 0.0
            else:
                u = (t - a.ramp - hold) / a.ramp
                env = 0.5 * (1.0 + math.cos(math.pi * u))
                denv = -0.5 * math.pi / a.ramp * math.sin(math.pi * u)

            s, c = math.sin(w * t), math.cos(w * t)
            pose[j] = q0[j] + A * env * s
            ff[j] = A * (denv * s + env * w * c)      # exact derivative -> feedforward

            client.read_state()
            client.send_joint_position(pose, feedforward_speed=ff)

            t_cmd.append(arr("joint_position_commanded")[j])
            t_act.append(arr("joint_position")[j])
            t_tgt.append(arr("joint_position_target")[j])

        # settle back exactly on the start pose
        t_end = time.perf_counter() + 1.5
        t_next = time.perf_counter()
        while time.perf_counter() < t_end:
            if time.perf_counter() < t_next:
                continue
            t_next += dt
            client.read_state()
            client.send_joint_position(q0)
        succ = scalar("control_success_rate")
        miss = int(scalar("rti_worst_consecutive_misses"))
        disc = scalar("rti_discard_pct")
        final = arr("joint_position")[j]
    finally:
        client.stop()
        time.sleep(0.7)
        client.disconnect()
        client.close()

    cmd = np.array(t_cmd); act = np.array(t_act); tgt = np.array(t_tgt)
    print(f"  samples                {len(cmd)}  ({total:.1f} s at 250 Hz)")
    print(f"  swept                  {np.rad2deg(cmd.max() - cmd.min()):.3f} deg "
          f"(asked {2 * a.deg:.3f})")
    print(f"  limiter bite |tgt-cmd| max {np.rad2deg(np.abs(tgt - cmd).max()):.4f} deg "
          f"<- 0 means the limiter never bound")
    print(f"  tracking     |cmd-act| max {np.rad2deg(np.abs(cmd - act).max()):.4f} deg "
          f"med {np.rad2deg(np.median(np.abs(cmd - act))):.4f}")
    print(f"  success rate           {succ * 100:.1f}%")
    print(f"  worst consecutive miss {miss}")
    print(f"  discarded              {disc:.1f}%")
    print(f"  returned to start      residual {final - q0[j]:+.6f} rad "
          f"({np.rad2deg(final - q0[j]):+.4f} deg)")

    ok = True
    if np.rad2deg(cmd.max() - cmd.min()) < 2 * a.deg * 0.9:
        print("\nFAIL: the arm did not sweep the requested amplitude")
        ok = False
    if succ < 0.95:
        print(f"\nFAIL: success rate {succ * 100:.1f}% — the PC did not keep up")
        ok = False
    if abs(final - q0[j]) > math.radians(0.5):
        print(f"\nFAIL: did not return to the start pose")
        ok = False
    print("\nPASS: RTI drove a real streamed trajectory and returned to start."
          if ok else "\nFAILED — see above.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
