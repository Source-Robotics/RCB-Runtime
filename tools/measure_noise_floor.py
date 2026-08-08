"""Sample the joint kinematics noise floor to pick filter time constants.

Run with the stack UP, the arm ENABLED, and the robot AT REST (not homing, not moving):

    python3 tools/measure_noise_floor.py            # 2 s sample
    python3 tools/measure_noise_floor.py --secs 5

At rest the TRUE joint acceleration and jerk are exactly zero, so whatever
shared_data reports for them is pure noise. That noise floor is what sets the filter
time constants for the raw/filtered split (see currently_doing.md, and
utility/signal_filters.py for the poles-equal-differentiations rule).

Read-only: this only READS /dev/shm and never sends a CAN frame or writes shared memory.
It intentionally refuses to run while the arm is moving, because a real motion would
inflate the "noise" with genuine signal and give a nonsense recommendation.
"""

import argparse
import struct
import sys
import time

import numpy as np

JOINT_FIELDS = ("joint_speed", "joint_acceleration", "joint_jerk")
MOVE_GUARD_RAD_S = 0.02          # if |speed| exceeds this, the arm is not "at rest"


def _read(name, n):
    with open(f"/dev/shm/{name}", "rb") as fh:
        return np.array(struct.unpack("<" + "d" * n, fh.read(8 * n)))


def _read1(name):
    return _read(name, 1)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=2.0, help="sample duration (s)")
    ap.add_argument("--joints", type=int, default=6)
    ap.add_argument("--dt", type=float, default=0.004, help="loop period for the tau math")
    args = ap.parse_args()

    # RTI must be alive, or /dev/shm is a frozen snapshot.
    t0 = _read1("loop_tick")
    time.sleep(0.2)
    if _read1("loop_tick") == t0:
        sys.exit("RTI is not running (loop_tick frozen) — start the stack first.")

    n = args.joints
    samples = {k: [] for k in JOINT_FIELDS}
    print(f"Sampling {args.secs:.1f}s at rest — keep the arm still...")
    end = time.time() + args.secs
    peak_speed = 0.0
    while time.time() < end:
        for k in JOINT_FIELDS:
            v = _read(k, n)
            samples[k].append(v)
        peak_speed = max(peak_speed, float(np.abs(samples["joint_speed"][-1]).max()))
        time.sleep(args.dt)

    if peak_speed > MOVE_GUARD_RAD_S:
        sys.exit(f"Arm was MOVING (peak |speed| {peak_speed:.3f} > {MOVE_GUARD_RAD_S} rad/s) — "
                 f"measurement invalid. Re-run with the robot at rest.")

    print(f"\n{len(samples['joint_speed'])} samples, arm confirmed at rest "
          f"(peak |speed| {peak_speed:.4f} rad/s)\n")
    print(f"{'signal':22}{'noise std (per joint)':>44}")
    print("-" * 68)
    noise = {}
    for k in JOINT_FIELDS:
        arr = np.array(samples[k])
        std = arr.std(axis=0)
        noise[k] = float(std.max())
        print(f"{k:22}" + "  ".join(f"{s:6.4f}" for s in std[:n]))

    # A crude but useful tau suggestion: to knock the worst noise band down ~10x with a
    # 1-pole roll-off you want a corner well below the noise. This only ballparks the knob;
    # the structural rule (poles == differentiations) is what actually matters.
    print("\nStarting-point time constants (REFINE by eye on the live signal):")
    print("  NOTE joint_speed here is ALREADY ema'd today, so its std understates raw noise.")
    for k, diffs in (("joint_speed", 0), ("joint_acceleration", 1), ("joint_jerk", 2)):
        poles = max(1, diffs)      # need at least as many poles as differentiations
        print(f"  {k:22} noise {noise[k]:.4f}  -> suggest {poles} pole(s), tau ~20-40 ms "
              f"({'none may be fine' if noise[k] < 1e-3 else 'filtering worthwhile'})")

    print("\nToday's hardcoded alpha=0.15 == tau 24.6 ms at 250 Hz (the back-compat anchor).")


if __name__ == "__main__":
    main()
