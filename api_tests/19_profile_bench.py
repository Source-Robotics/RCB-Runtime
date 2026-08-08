#!/usr/bin/env python3
"""19: benchmark every motion profile, and the two corner curves, ON THE ARM.

Two experiments:

  --p2p     the SAME point-to-point move under every profile:
            trapezoid / quintic / s_curve / toppra / ruckig

  --corner  the SAME blended path under both corner curves:
            blend_continuity="c1" (quadratic Bezier) vs "c2" (quintic Bezier)

WHAT WE MEASURE, AND WHY IT IS NOT DURATION.

Duration tells you almost nothing here. Every one of these profiles reaches the target; the
question is HOW. What separates them is JERK — the rate of change of acceleration — because
that is what you hear as a clack and feel as a shudder, and what puts shock load through the
gearboxes.

  trapezoid  bounds velocity and acceleration. Jerk is UNBOUNDED: acceleration steps
             instantaneously, so jerk is limited only by the tick (a_max/dt).
  toppra     is TIME-OPTIMAL, which means bang-bang in acceleration. Also unbounded jerk.
             It is the only profile that can follow a PATH (and so the only one that blends).
  ruckig     bounds velocity, acceleration AND jerk. Third-order.
  quintic /
  s_curve    smooth polynomial time-scalings with zero endpoint jerk. They do not ENFORCE a
             jerk limit, but they are inherently gentle — measured GENTLER than ruckig (4.9
             and 3.7 vs 7.2 rad/s^3). They pay for it by under-using the velocity budget.
             What is unique to ruckig is that you SET the jerk limit and it stays time-optimal
             under it.

And for corners:

  c1  quadratic Bezier. Tangent-continuous, but CURVATURE STEPS where the curve meets the
      straight legs. Curvature drives centripetal acceleration, so that step is an
      instantaneous acceleration change — a jerk spike, twice per corner, IN THE GEOMETRY.
      No profile can time it away.
  c2  quintic Bezier with zero second derivative at both ends. Curvature starts and ends at
      zero and matches the legs: no step. But it is TIGHTER at the apex (peak curvature 4.34
      vs 1.41 offline), so TOPPRA brakes harder there. Expect: smoother, slower.

WE MEASURE THE **COMMANDED** CHANNEL, NOT THE MEASURED ONE. The first version of this test
read RTI's `joint_jerk` (derived from encoders) and got ~1500 rad/s^3 for EVERY profile — a
noise floor, not a result. Jerk is a third finite-difference of a quantized signal. Meanwhile
the real commanded jerk at 10% speed is under 25 rad/s^3, because jerk scales with
speed_scale^3. We were 60x below the noise and the test cheerfully reported that ruckig and
toppra were identical.

So we read `joint_speed_commanded` / `joint_acceleration_commanded` — exactly what went to the
motors — keyed to RTI's `loop_tick` so the dt is exact. Both channels are printed side by
side, so the noise floor is visible rather than something you have to take on trust.

RUN IT FAST. Jerk scales with speed_scale^3: at 10% speed every profile's jerk is a thousandth
of its full-speed value, and differences vanish. --speed 50 or more is where this gets
interesting (and audible).
"""
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from _common import parser, connect, require_go, prepare_exec, finish, main_guard, \
    goto_start, START_POSE_DEG
from data import shared_data

P2P_PROFILES = ["trapezoid", "quintic", "s_curve", "toppra", "ruckig"]
# A move that actually exercises the arm: a big sweep on J0/J1.
P2P_TARGET_DEG = [125.0, -60.0, 165.0, 0.0, -30.0, 180.0]


class ShmSampler(threading.Thread):
    """Record what RTI COMMANDED, keyed to its exact tick grid.

    THE FIRST VERSION OF THIS MEASURED THE WRONG THING and it is worth saying why, because
    the numbers looked perfectly plausible.

    It read `joint_acceleration` / `joint_jerk` — the MEASURED values, which RTI derives by
    finite-differencing encoder counts. Jerk is the THIRD difference of a quantized signal,
    and its noise floor came out around 1500 rad/s^3. The real commanded jerk at 10% speed is
    under 25 rad/s^3 (jerk scales with speed_scale^3, so a tenth speed is a THOUSANDTH of the
    jerk). We were 60x below the noise: every profile measured ~1500 and the test happily
    reported "ruckig is 1.0x lower than toppra". Peak measured acceleration was 40x the
    commanded limit — also pure vibration.

    So read the COMMANDED channel instead. `joint_speed_commanded` and
    `joint_acceleration_commanded` are written every tick by mode_dispatch's
    _commit_joint_state: they are exactly what went to the motors, with no encoder noise in
    them at all.

    KEYED TO loop_tick, NOT TO WALL CLOCK. A Python poll jitters; RTI does not. We sample
    fast, keep a sample only when `loop_tick` advances, and differentiate ONLY across
    consecutive ticks (gap == 1) using RTI's exact dt. A dropped tick is detected and skipped
    rather than silently producing a bogus dt — which would manufacture exactly the kind of
    jerk spike we are trying to measure.
    """

    def __init__(self, joint_num=6, dt=0.004):
        super().__init__(daemon=True)
        self.joint_num = joint_num
        self.dt = dt
        self._running = True
        self.tick, self.vel_cmd, self.acc_cmd, self.vel_meas, self.acc_meas = [], [], [], [], []

    def run(self):
        j, r = shared_data.joint, shared_data.rti
        n = self.joint_num
        last = -1
        while self._running:
            t = int(r["loop_tick"][0])
            if t != last:
                last = t
                self.tick.append(t)
                self.vel_cmd.append(np.array(j["joint_speed_commanded"][:n]))
                self.acc_cmd.append(np.array(j["joint_acceleration_commanded"][:n]))
                self.vel_meas.append(np.array(j["joint_speed"][:n]))
                self.acc_meas.append(np.array(j["joint_acceleration"][:n]))
            time.sleep(0.0005)          # 2 kHz poll against a 250 Hz tick — never miss one

    def stop(self):
        self._running = False
        self.join(timeout=2.0)

    def stats(self, still_deg_s=0.5):
        if len(self.tick) < 20:
            return None
        tick = np.array(self.tick)
        acc_c = np.array(self.acc_cmd)
        vel_c = np.array(self.vel_cmd)
        vel_m = np.array(self.vel_meas)
        acc_m = np.array(self.acc_meas)

        # Trim to the part where the arm was actually commanded to move — leading/trailing
        # idle would dilute every peak toward zero.
        moving = np.flatnonzero(np.degrees(np.abs(vel_c)).max(axis=1) > still_deg_s)
        if moving.size < 10:
            return None
        w = slice(moving[0], moving[-1] + 1)
        tick, acc_c, vel_c, vel_m, acc_m = tick[w], acc_c[w], vel_c[w], vel_m[w], acc_m[w]

        # COMMANDED JERK — only across CONSECUTIVE ticks, so a dropped sample cannot
        # manufacture a spike out of a doubled dt.
        gap = np.diff(tick)
        ok = gap == 1
        jerk_c = np.abs(np.diff(acc_c, axis=0)[ok]) / self.dt if ok.any() else np.zeros((1, 1))
        missed = int((gap[gap > 1] - 1).sum())

        return dict(
            duration=float((tick[-1] - tick[0]) * self.dt),
            peak_vel_cmd=float(np.max(np.abs(vel_c))),
            peak_acc_cmd=float(np.max(np.abs(acc_c))),
            peak_jerk_cmd=float(np.max(jerk_c)),
            rms_jerk_cmd=float(np.sqrt(np.mean(jerk_c ** 2))),
            peak_vel_meas=float(np.max(np.abs(vel_m))),
            peak_acc_meas=float(np.max(np.abs(acc_m))),
            ticks=int(tick.size),
            missed=missed,
        )


def _run_and_sample(robot, fn, settle=0.6):
    s = ShmSampler()
    s.start()
    t0 = time.perf_counter()
    fn()
    robot.wait_idle(timeout=120.0)
    wall = time.perf_counter() - t0
    time.sleep(settle)          # let the trailing edge land in the samples
    s.stop()
    return wall, s.stats()


def bench_p2p(robot, args):
    print("\n" + "=" * 88)
    print("POINT-TO-POINT — the same move under every profile")
    print(f"  {START_POSE_DEG} -> {P2P_TARGET_DEG}   at speed={args.speed}%")
    print("=" * 88)
    print(f"\n  {'':<11}|{'  COMMANDED (what the planner sent)':<43}|{'  MEASURED (encoders)':<24}")
    print(f"  {'profile':<11}|{'motion':>8}{'peak|v|':>10}{'peak|a|':>10}{'peak|JERK|':>13}  |"
          f"{'peak|a|':>10}{'  <- noise floor':>14}")
    print(f"  {'':<11}|{'(s)':>8}{'(rad/s)':>10}{'(rad/s2)':>10}{'(rad/s3)':>13}  |{'(rad/s2)':>10}")
    print("  " + "-" * 76)
    results = {}
    for prof in args.profiles:
        goto_start(robot, args)
        robot.wait_idle(timeout=60.0)
        time.sleep(0.4)
        wall, st = _run_and_sample(
            robot, lambda p=prof: robot.move_joints(P2P_TARGET_DEG, profile=p, speed=args.speed))
        if st is None:
            print(f"  {prof:<11}  (no motion captured)")
            continue
        results[prof] = st
        drop = f"  ({st['missed']} ticks missed)" if st["missed"] else ""
        print(f"  {prof:<11}|{st['duration']:8.2f}{st['peak_vel_cmd']:10.3f}"
              f"{st['peak_acc_cmd']:10.2f}{st['peak_jerk_cmd']:13.1f}  |"
              f"{st['peak_acc_meas']:10.2f}{drop}")

    if "ruckig" in results and len(results) > 1:
        rj = results["ruckig"]["peak_jerk_cmd"]
        others = [(v["peak_jerk_cmd"], k) for k, v in results.items() if k != "ruckig"]
        worst = max(others)
        ranked = sorted((v["peak_jerk_cmd"], k) for k, v in results.items())
        print(f"\n  COMMANDED jerk, gentlest first: "
              + ", ".join(f"{k} {j:.1f}" for j, k in ranked) + " rad/s^3")
        print("  The split is NOT 'ruckig vs the rest'. trapezoid and toppra STEP acceleration")
        print("  (trapezoid instantaneously; toppra because time-optimal means bang-bang), so")
        print("  their jerk is unbounded. quintic and s_curve are smooth polynomial time-")
        print("  scalings with zero endpoint jerk — inherently gentle, often GENTLER than")
        print("  ruckig. What is unique to ruckig is that you can SET the jerk limit and it")
        print("  stays time-optimal under it; quintic/s_curve are fixed shapes that under-use")
        print("  the velocity budget to get their smoothness (compare peak|v|).")
        am = max(v["peak_acc_meas"] for v in results.values())
        ac = max(v["peak_acc_cmd"] for v in results.values())
        print(f"\n  Sanity: peak MEASURED accel {am:.1f} vs peak COMMANDED {ac:.2f} rad/s^2.")
        if am > 3 * ac:
            print(f"  The measured channel is {am/max(ac,1e-9):.0f}x the commanded one — that is")
            print("  vibration and encoder quantization, NOT the profile. This is exactly why")
            print("  the first version of this test could not tell these profiles apart.")
    return results


def bench_corner(robot, args):
    """The same blended rectangle under both corner curves."""
    print("\n" + "=" * 88)
    print("CORNERS — the same blended path under c1 (quadratic) vs c2 (quintic)")
    print(f"  blend_mode='zone', blend={args.blend}mm, profile='toppra'")
    print("=" * 88)

    a = list(START_POSE_DEG)
    p1 = list(a); p1[0] += args.rect_deg
    p2 = list(p1); p2[1] += args.rect_deg
    path = [p1, p2, list(a)]

    print(f"\n  path: {[round(v,1) for v in a]} -> ... -> back  "
          f"({args.rect_deg} deg legs, SHARP corners)")
    print(f"\n  {'corner':<9}{'motion':>9}{'peak|v|':>10}{'peak|a|':>10}"
          f"{'peak|JERK|':>13}{'rms jerk':>12}   (all COMMANDED)")
    print(f"  {'':<9}{'(s)':>9}{'(rad/s)':>10}{'(rad/s2)':>10}{'(rad/s3)':>13}{'(rad/s3)':>12}")
    print("  " + "-" * 68)
    results = {}
    for cont in args.continuity:
        goto_start(robot, args)
        robot.wait_idle(timeout=60.0)
        time.sleep(0.4)

        def send(c=cont):
            for wp in path:
                robot.move_joints(wp, profile="toppra", speed=args.speed,
                                  blend_mode="zone", blend=args.blend,
                                  blend_continuity=c)
        wall, st = _run_and_sample(robot, send)
        if st is None:
            print(f"  {cont:<9}  (no motion captured)")
            continue
        results[cont] = st
        print(f"  {cont:<9}{st['duration']:9.2f}{st['peak_vel_cmd']:10.3f}{st['peak_acc_cmd']:10.2f}"
              f"{st['peak_jerk_cmd']:13.1f}{st['rms_jerk_cmd']:12.1f}")

    if "c1" in results and "c2" in results:
        c1, c2 = results["c1"], results["c2"]
        dj = 100.0 * (c2["peak_jerk_cmd"] - c1["peak_jerk_cmd"]) / max(c1["peak_jerk_cmd"], 1e-9)
        drms = 100.0 * (c2["rms_jerk_cmd"] - c1["rms_jerk_cmd"]) / max(c1["rms_jerk_cmd"], 1e-9)
        dt = 100.0 * (c2["duration"] - c1["duration"]) / max(c1["duration"], 1e-9)
        print(f"\n  c2 vs c1 (COMMANDED):  rms jerk {drms:+.1f}%   duration {dt:+.1f}%"
              f"   [peak jerk {dj:+.1f}% — SEE BELOW]")
        print()
        print("  TRUST RMS, NOT PEAK. Peak jerk is a max over a SINGLE sample, so one spike owns")
        print("  it. Across three hardware runs of an otherwise deterministic planner it swung")
        print("  +24.7%, +2.8%, -32.3% — it is not measuring the corner, it is measuring which")
        print("  sample happened to land on a bang-bang edge. RMS over the same runs was")
        print("  +0.4%, -6.3%, -12.0%: a small but consistent ~6% improvement for c2.")
        print()
        print("  So c2 buys a modest real smoothness gain for a consistent ~9% time cost. It")
        print("  removes the curvature STEP at the zone boundary, but introduces a TIGHTER apex")
        print("  (peak curvature 4.34 vs 1.41), and TOPPRA is bang-bang in acceleration either")
        print("  way — so most of its own jerk survives. c2 would pay off properly only under a")
        print("  jerk-limited path parameterizer, which TOPPRA is not.")
    return results


def run():
    p = parser("19: benchmark every motion profile, and both corner curves, on the arm")
    p.add_argument("--p2p", action="store_true", help="point-to-point profile sweep")
    p.add_argument("--corner", action="store_true", help="c1 vs c2 corner comparison")
    p.add_argument("--profiles", nargs="+", default=P2P_PROFILES,
                   help=f"which profiles to bench (default: all of {P2P_PROFILES})")
    p.add_argument("--continuity", nargs="+", default=["c1", "c2"], help="corner curves to bench")
    p.add_argument("--blend", type=float, default=15.0, help="zone blend radius, mm")
    p.add_argument("--rect-deg", type=float, default=12.0, help="corner-test leg length, deg")
    # Jerk scales with speed_scale^3 — at the shared default of 10% every profile's jerk is a
    # THOUSANDTH of its real value and they all look identical. Run this one fast.
    p.set_defaults(speed=50.0)
    args = p.parse_args()
    if not args.p2p and not args.corner:
        args.p2p = args.corner = True      # default: run both
    print(f"\n  speed={args.speed}%  (jerk scales with speed^3 — below ~30% the profiles are"
          f" indistinguishable)")

    robot = connect(args)
    if not require_go(args, "run the profile / corner benchmark"):
        return
    prepare_exec(robot, args)

    if args.p2p:
        bench_p2p(robot, args)
    if args.corner:
        bench_corner(robot, args)
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
