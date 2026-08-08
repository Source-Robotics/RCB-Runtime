#!/usr/bin/env python3
"""17: rolling buffer — splice a command into an already-executing move.

This is the G-code / toolpath case: commands arrive one at a time, SLOWER than the
arm executes them. Before phase 5 each one ran to a full stop before the next was
planned, so a long path stuttered at every segment. Now the planner holds each move's
braking tail back and splices the next command in, so the arm flows through.

WHY THE PACING MATTERS: if you send all the waypoints at once they land in the blend
buffer together and get planned as ONE batch-blended chain — which already worked and
proves nothing about splicing. The whole point here is that each command arrives while
the previous one is ALREADY RUNNING. --gap controls that.

WHAT TO WATCH:
  * The arm should not stop between waypoints. Watch the corners.
  * logs/high_level.log will say "SPLICED id=... into the in-flight move" per command,
    or "splice REJECTED: <reason>" — a rejection is NOT a bug. It means the splice
    could not have reproduced the batch blend exactly, so the planner refused to fake
    it: that move finishes normally and the next one runs on its own. Honest stutter
    beats a silently wrong blend.

Compare against --no-splice, which sends the same path with the same gaps but with
blending off, so every waypoint stops. The duration difference is the win.

Usage:
    .venv/bin/python3 api_tests/17_rolling_buffer.py                 # dry run
    .venv/bin/python3 api_tests/17_rolling_buffer.py --go
    .venv/bin/python3 api_tests/17_rolling_buffer.py --go --no-splice   # baseline
"""
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from _common import (parser, connect, require_go, prepare_exec, finish, main_guard,
                     goto_start, START_POSE_DEG)


class MotionMonitor(threading.Thread):
    """Poll the arm while it moves and COUNT how many times it comes to a standstill.

    "The arm never stops" has to be measured, not eyeballed. Everything we know about the
    streaming case comes from a simulator — and a simulator is not an arm. It already
    missed one real bug (the stale-runway race) by modelling RTI's ack as instant.

    A "stop" is the arm being essentially stationary for at least STOP_MIN_S. That
    duration threshold is the important part: at a blended corner the arm slows a LOT but
    keeps moving, and that must not count. A genuine stop is followed by a settle wait, so
    it lasts far longer than any corner does.
    """

    STILL_DEG_S = 1.5     # below this the arm counts as stationary...
    STOP_MIN_S = 0.15     # ...for at least this long, or it is just a slow corner

    def __init__(self, robot, poll_hz=25.0):
        super().__init__(daemon=True)
        self._robot = robot
        self._dt = 1.0 / poll_hz
        self._samples = []
        self._running = True

    def run(self):
        while self._running:
            try:
                self._samples.append((time.perf_counter(),
                                      np.asarray(self._robot.get_joints(), dtype=float)))
            except Exception:
                pass
            time.sleep(self._dt)

    def stop(self):
        self._running = False
        self.join(timeout=2.0)

    def report(self):
        if len(self._samples) < 10:
            print("  (motion monitor got too few samples to judge)")
            return None
        t = np.array([s[0] for s in self._samples])
        q = np.array([s[1] for s in self._samples])
        speed = np.linalg.norm(np.diff(q, axis=0), axis=1) / np.maximum(np.diff(t), 1e-6)

        # Only look BETWEEN the first and last movement — the arm is legitimately at rest
        # before it starts and after it finishes; those are not mid-path stops.
        moving = np.nonzero(speed > self.STILL_DEG_S)[0]
        if moving.size < 2:
            print("  (arm barely moved — nothing to judge)")
            return None
        lo, hi = int(moving[0]), int(moving[-1])
        interior, it = speed[lo:hi], t[lo:hi]

        still = interior < self.STILL_DEG_S
        stops, i = [], 0
        while i < len(still):
            if still[i]:
                j = i
                while j < len(still) and still[j]:
                    j += 1
                dur = it[min(j, len(it) - 1)] - it[i]
                if dur >= self.STOP_MIN_S:
                    stops.append((it[i] - it[0], dur))
                i = j
            else:
                i += 1

        print(f"  motion: {t[hi] - t[lo]:.2f}s moving, "
              f"min mid-path speed {interior.min():.2f} deg/s, "
              f"median {np.median(interior):.1f} deg/s")
        if stops:
            print(f"  *** {len(stops)} STANDSTILL(S) mid-path:")
            for at, dur in stops[:12]:
                print(f"        at t={at:5.2f}s, lasting {dur:.2f}s")
            if len(stops) > 12:
                print(f"        ... and {len(stops) - 12} more")
        else:
            print("  *** ZERO standstills — the arm traced the whole path without stopping")
        return len(stops)


def main():
    p = parser("17: rolling buffer — splice into an executing move")
    p.add_argument("--blend-mm", type=float, default=None,
                   help="zone blend radius (mm). 0 disables blending entirely. "
                        "Default 25 (joint) / 20 (Cartesian). Note the builder clamps a "
                        "blend to HALF its segment, so a radius near half the leg length "
                        "leaves no run-in and the splice will be refused.")
    p.add_argument("--gap", type=float, default=None,
                   help="seconds to wait between sending each waypoint. This is what "
                        "makes the test meaningful: each command must arrive while the "
                        "previous one is still executing. Too large and the move ends "
                        "before the next arrives (you'll see the tail get committed); "
                        "too small and they all batch-blend instead of splicing. "
                        "Default 0.35s (joint) / 0.8s (Cartesian — those moves are "
                        "~3x longer because of the 20%% speed cap).")
    p.add_argument("--cartesian", action="store_true",
                   help="stream move_linear_rel_world legs instead of move_joints. The "
                        "Cartesian continuation pays an IK solve, so it gets its own "
                        "(larger) safety margin — see docs/blending.md.")
    p.add_argument("--leg-mm", type=float, default=80.0,
                   help="sweep length in mm (--cartesian). The zig-zag steps 20%% of this "
                        "in Z between sweeps; sized to stay inside the workspace from "
                        "START_POSE (bigger steps climb out of reach and IK refuses).")
    p.add_argument("--samples", type=int, default=60,
                   help="IK sample count per Cartesian leg (--cartesian). The default "
                        "sample_count_for_line() is coarse enough to trip false "
                        "IK_BRANCH_JUMP rejections on an otherwise reachable path.")
    p.add_argument("--legs", type=int, default=5,
                   help="how many waypoints to stream")
    p.add_argument("--deg", type=float, default=30.0,
                   help="leg size in degrees. Must be big enough that a body of at "
                        "least SPLICE_MIN_BODY_S survives the split, or the planner "
                        "sends each move whole and nothing splices.")
    p.add_argument("--stream", action="store_true",
                   help="G-CODE MODE: fire N short chords back-to-back with NO gaps — a "
                        "smooth arc chopped into short moves, which is what a real toolpath "
                        "looks like. This is the case the rolling buffer exists for and the "
                        "ONLY one that exercises BATCH splicing: commands pile into the "
                        "buffer faster than the arm executes them, so each splice carries a "
                        "whole batch. The arm should trace the arc WITHOUT EVER STOPPING.")
    p.add_argument("--chords", type=int, default=200,
                   help="number of short moves in the streamed arc (--stream). Must exceed "
                        "MAX_BLEND_LOOKAHEAD (100) or the whole path fits in the blend "
                        "buffer and gets planned as ONE batch chain — which tests batch "
                        "blending, not the rolling buffer, and splices nothing at all. "
                        "200 gives: 100 flush -> hold tail -> 100 accumulate -> ONE batch "
                        "splice of 100 commands into the move already running.")
    p.add_argument("--arc-deg", type=float, default=25.0,
                   help="radius of the J0/J1 arc in degrees (--stream). Sized to stay well "
                        "inside the joint limits from START_POSE (J0 65..115, J1 -105..-55).")
    p.add_argument("--no-splice", action="store_true",
                   help="baseline: same path, same gaps, blending OFF — every waypoint "
                        "runs to a full stop. Run this first to feel the difference.")
    # The shared defaults (speed=10%, timeout=20s) are wrong for this test: five 30-degree
    # legs at 10% is 32s of motion, so wait_idle gives up before the arm finishes. They
    # also make the splice pointless — at a crawl there is no corner speed worth keeping.
    p.set_defaults(speed=40.0, timeout=180.0)
    args = p.parse_args()
    if args.gap is None:
        args.gap = 0.8 if args.cartesian else 0.35
    if args.blend_mm is None:
        # A G-code chord is short, so its blend must be too: the builder clamps a blend to
        # HALF the segment, and a chord blended to the clamp has no straight part left at
        # all. 3mm on a ~7mm chord is a real corner-cut that still leaves geometry behind.
        # A G-code chord is SHORT (~2.8mm of TCP at the --stream defaults), and the builder
        # clamps a blend to HALF its segment — so a big radius just pins every corner at the
        # clamp and leaves no geometry behind. 2mm on a 2.8mm chord is a real corner-cut.
        args.blend_mm = 2.0 if args.stream else (20.0 if args.cartesian else 25.0)

    if args.sync:
        raise SystemExit("ERROR: the rolling buffer needs async mode; do NOT pass "
                         "--sync (it blocks each move to completion, so nothing is "
                         "ever in flight when the next command arrives).")
    args.profile = "toppra"

    robot = connect(args)
    lane = "CARTESIAN" if args.cartesian else "JOINT"
    mode = "baseline (blend OFF, stops at every waypoint)" if args.no_splice \
        else f"rolling buffer (zone blend {args.blend_mm:g}mm)"
    if args.stream:
        action = (f"STREAM {args.chords} short JOINT chords back-to-back "
                  f"(G-code case) — {mode}")
    else:
        action = (f"stream {args.legs} {lane} waypoints, {args.gap:.2f}s apart — {mode}")
    if not require_go(args, action):
        finish(robot)
        return

    prepare_exec(robot, args)
    robot.set_completion_policy("settled")
    goto_start(robot, args)

    kw = dict(profile="toppra", speed=args.speed)
    if not args.no_splice and args.blend_mm > 0:
        kw.update(blend_mode="zone", blend=args.blend_mm)

    # A raster / boustrophedon zig-zag — the shape a real toolpath actually has. One axis
    # sweeps back and forth between two rails while the other steps steadily along, so
    # EVERY waypoint is a genuine reversal corner with a real blend to reproduce, and no
    # waypoint is ever revisited.
    if args.stream:
        # A smooth circular arc in the J0/J1 plane, chopped into short chords — which is
        # exactly what a G-code toolpath is. Adjacent chords are nearly collinear, so every
        # corner is shallow and blendable; no waypoint is ever revisited.
        R = args.arc_deg
        legs = []
        for i in range(args.chords):
            a = 2.0 * np.pi * i / (args.chords - 1)
            wp = list(START_POSE_DEG)
            wp[0] += R * np.sin(a)
            wp[1] += R * (1.0 - np.cos(a))
            legs.append(wp)
        chord_deg = 2.0 * R * np.sin(np.pi / (args.chords - 1))
        print(f"\nSTREAMING {len(legs)} short chords BACK-TO-BACK (no gaps) — {mode}")
        print(f"  A {R:.0f}deg circular arc in the J0/J1 plane, chopped into "
              f"{chord_deg:.1f}deg chords. This is the G-code case.")
        print(f"  Commands arrive far faster than the arm executes them, so they pile into "
              f"the blend buffer and each splice carries a WHOLE BATCH.")
        print(f"  J0 sweeps {START_POSE_DEG[0] - R:.0f}..{START_POSE_DEG[0] + R:.0f}, "
              f"J1 sweeps {START_POSE_DEG[1]:.0f}..{START_POSE_DEG[1] + 2 * R:.0f}")
        send = lambda wp: robot.move_joints(wp, **kw)
    elif args.cartesian:
        s = args.leg_mm / 1000.0
        # world-frame relative legs in the XZ plane: sweep +X, step +Z, sweep -X, ...
        legs = [((s if i % 2 == 0 else -s), 0.0, s * 0.2) for i in range(args.legs)]
        print(f"\nStreaming {len(legs)} CARTESIAN legs ({args.leg_mm:.0f}mm), one every "
              f"{args.gap:.2f}s — {mode}")
        print("  Cartesian speed is capped at 20% by the planner, so these moves are LONG "
              "(~3s each) — hence the bigger --gap.")
        for i, leg in enumerate(legs):
            print(f"  [{i + 1}] dx={leg[0] * 1000:+.0f}mm dz={leg[2] * 1000:+.0f}mm")
        send = lambda leg: robot.move_linear_rel_world(leg[0], leg[1], leg[2],
                                                       samples=args.samples, **kw)
    else:
        step = args.deg * 0.3
        legs = []
        for i in range(args.legs):
            wp = list(START_POSE_DEG)
            wp[0] += args.deg if i % 2 == 0 else 0.0     # sweep out and back
            wp[1] += step * (i + 1)                       # and advance
            legs.append(wp)
        print(f"\nStreaming {len(legs)} JOINT waypoints, one every {args.gap:.2f}s — {mode}")
        for i, wp in enumerate(legs):
            print(f"  [{i + 1}] {[round(v, 1) for v in wp]}")
        send = lambda wp: robot.move_joints(wp, **kw)

    monitor = MotionMonitor(robot)
    monitor.start()
    t0 = time.perf_counter()
    for leg in legs:
        send(leg)
        if not args.stream:
            time.sleep(args.gap)      # <-- the point: the next one arrives mid-move
    robot.wait_idle(timeout=args.timeout)
    elapsed = time.perf_counter() - t0
    monitor.stop()

    gaps = 0.0 if args.stream else args.gap * len(legs)
    print(f"\n  total {elapsed:.2f}s wall clock for {len(legs)} waypoints "
          f"(includes {gaps:.2f}s of deliberate send gaps)")
    monitor.report()
    print("  grep logs/high_level.log for 'SPLICED' / 'splice REJECTED' / "
          "'committing held tail' to see what the planner actually did.")
    finish(robot)


if __name__ == "__main__":
    main_guard(main)
