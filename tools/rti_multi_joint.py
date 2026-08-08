"""Multi-joint streamed motion — and a test that the limiter treats joints INDEPENDENTLY.

Every RTI motion so far moved one joint. A real controller moves several at once, and that is
also the first thing that can expose a coupling bug.

THE INDEPENDENCE CLAIM UNDER TEST. `motion/rti_handler.py` sets `Synchronization.No` on
purpose: a rate limiter must plan each joint on its own. Ruckig's default is to SYNCHRONISE —
it stretches every joint's profile so they all finish together, which is right for a
coordinated point-to-point move and wrong for a safety filter, because one demanding joint
would then slow down five innocent ones.

Nothing has ever checked that on hardware. So the three joints here are given DIFFERENT
amplitudes AND DIFFERENT frequencies:

  - if independent, each joint's tracking gap scales with ITS OWN aggressiveness
  - if synchronised, the gentle joints would be dragged toward the demanding one's profile,
    and their gaps would inflate well past what their own motion justifies

Shape per joint is the same raised-cosine-enveloped sine as rti_first_motion.py: starts and
ends at rest, no discontinuity, analytic derivative sent as feedforward. Both safety toggles
stay ON. Defaults are gentle — under 9% of any velocity limit. E-stop within reach.
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

# joint -> (amplitude deg, frequency Hz). Deliberately all different.
PLAN = {3: (3.0, 0.20), 4: (5.0, 0.25), 5: (8.0, 0.30)}


def arr(name):
    with open(f"{SHM}/{name}", "rb") as f:
        return list(struct.unpack(f"<{N}d", f.read(8 * N)))


def scalar(name):
    with open(f"{SHM}/{name}", "rb") as f:
        return struct.unpack("<d", f.read(8))[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=14.0, help="total motion time")
    ap.add_argument("--ramp", type=float, default=2.5, help="ramp in/out seconds")
    ap.add_argument("--scale", type=float, default=1.0, help="scale all amplitudes")
    a = ap.parse_args()

    import config.xml_parser as X
    from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR
    sysc = X.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = X.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                   active_gripper=sysc["active_gripper"])
    V = np.atleast_1d(cfg["rti_velocity_limit"]).astype(float)
    A_ = np.atleast_1d(cfg["rti_acceleration_limit"]).astype(float)
    lo = np.atleast_1d(cfg["software_joint_limit_negative"]).astype(float)
    hi = np.atleast_1d(cfg["software_joint_limit_positive"]).astype(float)

    print("plan — deliberately different per joint, so coupling would be visible")
    print(f"{'joint':>6}{'amp deg':>9}{'Hz':>7}{'peak v':>9}{'%vlim':>8}{'peak a':>9}{'%alim':>8}")
    print("-" * 56)
    spec = {}
    for j, (deg, hz) in PLAN.items():
        amp = math.radians(deg * a.scale)
        w = 2 * math.pi * hz
        spec[j] = (amp, w)
        pv, pa = amp * w, amp * w * w
        print(f"{j:>6}{deg * a.scale:>9.2f}{hz:>7.2f}{pv:>9.4f}{pv / V[j] * 100:>7.1f}%"
              f"{pa:>9.4f}{pa / A_[j] * 100:>7.1f}%")
        if pv > V[j] * 0.5 or pa > A_[j] * 0.5:
            print(f"ABORT: joint {j} exceeds 50% of a limit.")
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
    q0 = list(client.read_once(timeout=2.0)["j_p"])
    for j, (amp, _w) in spec.items():
        if not (lo[j] < q0[j] - amp and q0[j] + amp < hi[j]):
            print(f"ABORT: joint {j} would reach its soft limits.")
            client.disconnect(); client.close()
            return 1
    print(f"[1] start pose {[round(v, 4) for v in q0]}")
    client.claim()
    print("[2] control claimed — both toggles ON\n[3] moving 3 joints at once...\n")

    dt = 1.0 / 250.0
    hold = a.seconds - 2 * a.ramp
    pose, ff = list(q0), [0.0] * N
    rec = {j: {"cmd": [], "act": [], "tgt": []} for j in spec}

    try:
        t0 = time.perf_counter()
        t_next = t0
        while True:
            now = time.perf_counter()
            if now < t_next:
                continue
            t_next += dt
            t = now - t0
            if t >= a.seconds:
                break
            if t < a.ramp:
                env = 0.5 * (1 - math.cos(math.pi * t / a.ramp))
                denv = 0.5 * math.pi / a.ramp * math.sin(math.pi * t / a.ramp)
            elif t < a.ramp + hold:
                env, denv = 1.0, 0.0
            else:
                u = (t - a.ramp - hold) / a.ramp
                env = 0.5 * (1 + math.cos(math.pi * u))
                denv = -0.5 * math.pi / a.ramp * math.sin(math.pi * u)

            for j, (amp, w) in spec.items():
                s, c = math.sin(w * t), math.cos(w * t)
                pose[j] = q0[j] + amp * env * s
                ff[j] = amp * (denv * s + env * w * c)

            client.read_state()
            client.send_joint_position(pose, feedforward_speed=ff)

            cmd_all, act_all, tgt_all = (arr("joint_position_commanded"),
                                         arr("joint_position"),
                                         arr("joint_position_target"))
            for j in spec:
                rec[j]["cmd"].append(cmd_all[j])
                rec[j]["act"].append(act_all[j])
                rec[j]["tgt"].append(tgt_all[j])

        t_end = time.perf_counter() + 1.5
        t_next = time.perf_counter()
        while time.perf_counter() < t_end:
            if time.perf_counter() < t_next:
                continue
            t_next += dt
            client.read_state()
            client.send_joint_position(q0)
        succ, miss = scalar("control_success_rate"), int(scalar("rti_worst_consecutive_misses"))
        disc = scalar("rti_discard_pct")
        final = arr("joint_position")
    finally:
        client.stop()
        time.sleep(0.7)
        client.disconnect()
        client.close()

    print(f"{'joint':>6}{'asked deg':>11}{'swept deg':>11}{'gap tgt-cmd':>13}"
          f"{'track cmd-act':>15}{'residual deg':>14}")
    print("-" * 72)
    ok = True
    gaps = {}
    for j, (amp, w) in spec.items():
        cmd = np.array(rec[j]["cmd"]); act = np.array(rec[j]["act"]); tgt = np.array(rec[j]["tgt"])
        swept = np.rad2deg(cmd.max() - cmd.min())
        gap = np.rad2deg(np.abs(tgt - cmd).max())
        trk = np.rad2deg(np.abs(cmd - act).max())
        res = np.rad2deg(final[j] - q0[j])
        gaps[j] = gap
        print(f"{j:>6}{np.rad2deg(2 * amp):>11.3f}{swept:>11.3f}{gap:>13.4f}"
              f"{trk:>15.4f}{res:>14.4f}")
        if swept < np.rad2deg(2 * amp) * 0.9:
            print(f"   FAIL: joint {j} did not sweep its amplitude"); ok = False
        if abs(res) > 0.5:
            print(f"   FAIL: joint {j} did not return to start"); ok = False

    print(f"\n  success rate {succ * 100:.1f}%   worst miss {miss}   discarded {disc:.1f}%")
    if succ < 0.95:
        print("  FAIL: success rate too low"); ok = False

    # --- the independence check ---------------------------------------------
    # Each joint's gap should be explained by ITS OWN peak acceleration. Under Ruckig's
    # default synchronisation the gentle joints would be dragged toward the demanding one's
    # timing, so their gap-per-unit-acceleration would blow out rather than stay comparable.
    print("\nINDEPENDENCE — gap normalised by each joint's OWN peak acceleration:")
    norm = {}
    for j, (amp, w) in spec.items():
        pa = amp * w * w
        norm[j] = gaps[j] / pa
        print(f"  joint {j}: gap {gaps[j]:.4f} deg / peak accel {pa:.4f} rad/s2 = {norm[j]:.4f}")
    spread = max(norm.values()) / max(min(norm.values()), 1e-9)
    print(f"  spread across joints: {spread:.2f}x")
    if spread > 4.0:
        print("  WARN: one joint's gap is far out of proportion to its own motion —")
        print("        that is what coupling would look like. Worth investigating.")
    else:
        print("  PASS: each joint's gap tracks its own demand — planning is independent.")

    print("\nPASS: multi-joint streaming works." if ok else "\nFAILED — see above.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
