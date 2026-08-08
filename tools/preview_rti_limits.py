#!/usr/bin/env python3
"""Show what the RTI rate limiter will do to a command, before the arm does it.

The limiter has never clamped anything on hardware — the communication test streams a
hold-position command, which gives it nothing to clamp. Its first real engagement will be
during motion, using acceleration limits that are NOT physically derived: they come from
the 0.5 s ramp-time convention EXEC and JOG already assume, not from torque/inertia
(see robots/PAR6.xml joint 0 and TODO/RTI_mode_plan.md §9).

It was written to answer "will the arm feel sluggish?" and instead found two defects in the
hand-rolled cascade it used to exercise: on a DISCONTINUOUS command that limiter overshot by
up to 132%, and — found later, on hardware — it NEVER CONVERGED, limit-cycling at +/-0.6 deg
on a constant target forever. Both are gone since the position path moved to Ruckig
(2026-08-02): overshoot is 0.0% and the target is reached exactly. This tool is now the
regression check for that. See TODO/RTI_mode_plan.md §11b.

Nothing here touches hardware — it runs the real `_limit_position()` against the real robot
XML and reports the resulting motion.

    python3 tools/preview_rti_limits.py                 # 90 deg step, every joint
    python3 tools/preview_rti_limits.py --deg 10        # a small move
    python3 tools/preview_rti_limits.py --joint 1 --trace   # tick-by-tick for one joint
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config.xml_parser as read_XML2
from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR
from motion.rti_handler import RTIModeHandler


def simulate(handler, joint, step_rad, max_ticks=5000):
    """Drive one joint with a step command and record what the limiter emits.

    Reports OVERSHOOT as a first-class result. An earlier version of this tool only
    measured "time to first come within 0.1% of target", which was actively misleading:
    the limiter crosses the target at full speed, skipping straight over that window, so
    the number it printed was when the RINGING happened to land inside the tolerance.
    """
    dt = handler.delta_t

    # Start each joint at the MIDPOINT of its own soft range, not at 0. PAR6 joints 1 and 2
    # have ranges that do not contain zero at all ([-2.441,-0.112] and [1.991,6.563]), so a
    # simulation starting at 0 is outside the workspace and the joint-limit backstop
    # (correctly) drags it to the nearest limit — which looks like a limiter bug and is not.
    lo, hi = handler.limit_neg[joint], handler.limit_pos[joint]
    start = 0.0 if not (np.isfinite(lo) and np.isfinite(hi)) else 0.5 * (lo + hi)
    # Step in whichever direction has room, and truncate to what the range allows.
    goal = start + step_rad
    if np.isfinite(hi) and goal > hi:
        goal = start - step_rad
    goal = float(np.clip(goal, lo, hi))
    actual = goal - start

    start_vec = np.zeros(handler.joint_num)
    for k in range(handler.joint_num):
        klo, khi = handler.limit_neg[k], handler.limit_pos[k]
        start_vec[k] = 0.0 if not (np.isfinite(klo) and np.isfinite(khi)) else 0.5 * (klo + khi)
    target = start_vec.copy()
    target[joint] = goal
    handler._rl_q[:] = start_vec
    handler._rl_v[:] = 0.0
    handler._rl_a[:] = 0.0

    pos, vel = [], []
    for _ in range(max_ticks):
        out = handler._limit_position(target)
        pos.append(out[joint])
        vel.append(handler._rl_v[joint])
    pos, vel = np.array(pos), np.array(vel)

    # Everything below is measured relative to the joint's own start, in the direction
    # it actually travelled.
    rel = (pos - start) * np.sign(actual) if actual else pos - start
    mag = abs(actual)
    # Within 0.1% of the step, not bit-exact. While the limiter overshot, `rel >= mag`
    # fired early — during the overshoot — so exactness cost nothing. Ruckig approaches
    # from BELOW and lands to float precision, so the same test would report the tick where
    # the last 1e-13 rad disappeared, or "never" when it never quite does (joint 0 settles
    # 1e-13 short of target and would have printed 'never' while being visibly arrived).
    tol = mag * 1e-3
    reach = int(np.argmax(rel >= mag - tol)) if (rel >= mag - tol).any() else None
    peak = float(rel.max())
    overshoot = (peak - mag) / mag * 100.0 if mag else 0.0
    return {"reach_ticks": reach, "reach_s": reach * dt if reach is not None else None,
            "start": start, "goal": goal, "step_used": actual,
            "peak": peak, "overshoot_pct": overshoot,
            "peak_vel": float(np.max(np.abs(vel))), "pos": pos, "vel": vel}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--deg", type=float, default=90.0, help="step size in degrees")
    ap.add_argument("--joint", type=int, help="only this joint")
    ap.add_argument("--trace", action="store_true", help="print the first ticks tick-by-tick")
    a = ap.parse_args()

    sysc = read_XML2.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = read_XML2.load_full_robot_config(str(sysc["robot_xml_path"]),
                                           grippers_dir=GRIPPERS_DIR,
                                           active_gripper=sysc["active_gripper"])
    jn = int(cfg["joint_num"])
    dt = cfg.get("delta_t") or 0.004

    h = RTIModeHandler(joint_num=jn, delta_t=dt,
                       vel_limit=cfg["joint_velocity_limit"],
                       accel_limit=cfg["joint_acceleration_limit"],
                       jerk_limit=cfg["joint_jerk_limit"],
                       torque_rate_limit=cfg["torque_rate_limit"],
                       limit_neg=cfg["software_joint_limit_negative"],
                       limit_pos=cfg["software_joint_limit_positive"])

    step = np.deg2rad(a.deg)
    joints = [a.joint] if a.joint is not None else range(jn)

    print(f"{sysc['active_robot']}   tick {dt*1e3:.1f} ms ({1/dt:.0f} Hz)   "
          f"step {a.deg:.0f} deg ({step:.3f} rad)")
    print()
    print(f"{'j':>2} {'soft range':>16} {'step used':>10} | "
          f"{'reaches':>9} {'OVERSHOOT':>10} {'peak vel':>9} {'% vlim':>7}")
    print("-" * 92)

    worst = 0.0
    for j in joints:
        r = simulate(h, j, step)
        vl = h.vel_limit[j]
        worst = max(worst, r["overshoot_pct"])
        reach = f"{r['reach_s']:.3f}s" if r["reach_s"] is not None else "  never"
        rng = f"[{h.limit_neg[j]:.2f},{h.limit_pos[j]:.2f}]"
        used = f"{np.rad2deg(r['step_used']):+.1f}d"
        print(f"{j:>2} {rng:>16} {used:>10} | "
              f"{reach:>9} {r['overshoot_pct']:>9.1f}% "
              f"{r['peak_vel']:>9.3f} {r['peak_vel']/vl*100 if vl else 0:>6.0f}%")

    print()
    print("Every joint starts at the MIDPOINT of its own soft range and steps toward the end")
    print("with room. 'step used' is truncated when the range is smaller than the request.")
    print()
    if worst > 5.0:
        print("*** OVERSHOOT IS LARGE. Ruckig solves a jerk-limited profile to the target,")
        print("    so a correct run overshoots by 0.0% — check whether _limit_position() is")
        print("    still using it, or has fallen back to the legacy cascade (it logs an")
        print("    error at startup when it does). That cascade capped velocity at")
        print("    sqrt(2*a*|d|), which overshot 132% on a 90 deg step, 2421% on 5 deg, and")
        print("    never converged at all: it limit-cycled at +/-0.6 deg forever.")
    else:
        print("Overshoot is 0% and the target is reached exactly — Ruckig plans a")
        print("jerk-limited profile rather than clamping derivatives and hoping. The")
        print("software joint limits clamp the output regardless, so even a limiter fault")
        print("can never leave the workspace.")

    if a.trace:
        j = a.joint if a.joint is not None else 0
        r = simulate(h, j, step)
        pos, vel = r["pos"], r["vel"]
        n = min(15, len(pos))
        print(f"\nfirst {n} ticks, joint {j}:")
        print(f"{'tick':>5} {'ms':>7} {'pos rad':>10} {'deg':>8} {'vel rad/s':>10}")
        for k in range(n):
            print(f"{k+1:>5} {(k+1)*dt*1e3:>7.1f} {pos[k]:>10.6f} "
                  f"{np.rad2deg(pos[k]):>8.4f} {vel[k]:>10.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
