#!/usr/bin/env python3
"""18: run a G-code toolpath.

Parses G-code (motion/gcode.py) and streams it as move_linear commands with
blend_mode="tolerance" — the G64 P<tol> knob. This is the toolpath front end: each line
becomes a short Cartesian move, they pile into the blend buffer, and the rolling buffer
splices them so the arm flows through the corners instead of stopping at every one.

    .venv/bin/python3 api_tests/18_gcode.py                     # dry run, built-in demo
    .venv/bin/python3 api_tests/18_gcode.py --go
    .venv/bin/python3 api_tests/18_gcode.py --go --file part.gcode
    .venv/bin/python3 api_tests/18_gcode.py --go --tol 2.0      # override the G64 tolerance
    .venv/bin/python3 api_tests/18_gcode.py --go --exact-stop   # G61: stop at every corner

READ THIS BEFORE YOU TRUST A NUMBER:

  * TOLERANCES BELOW ~1mm CURRENTLY DO NOT BLEND. The composite path is sampled at uniform
    arc length (2mm in the Cartesian lane), and a blend zone smaller than the spacing has no
    samples in it — so the corner is never rounded and TOPPRA brakes for a sharp corner. The
    planner warns loudly when this happens ("blend zone is UNDER-SAMPLED"). It is safe, just
    not blended. Fixing it needs the arc-length-s work (blending_design.md).

  * F (FEEDRATE) IS IGNORED. G-code asks for mm/min; our planner asks TOPPRA for the
    time-optimal timing under the arm's joint limits. There is no feedrate input today.
    Use --speed. This is stated rather than faked.

  * E / M104 / M106 ARE IGNORED. No extruder axis, no heaters. A sliced model will trace the
    MOTION of the print and extrude nothing.
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from _common import parser, connect, require_go, prepare_exec, finish, main_guard
from motion.gcode import parse, GcodeMove, GcodeDwell, GcodePause, GcodeError

# A star polygon — sharp corners of VERY different turn angles, which is the case a fixed
# blend=<mm> radius cannot serve and a tolerance can. Drawn in the XY plane at the arm's
# current Z.
DEMO = """
G21
G90
G64 P{tol}
G1 X{x0:.3f} Y{y0:.3f} F1200
{lines}
"""


def demo_program(cx, cy, r_mm, points, tol):
    """A {points}-pointed star: alternating sharp and shallow corners."""
    xs, ys = [], []
    for i in range(2 * points + 1):
        a = np.pi / 2 + i * np.pi / points
        rad = r_mm if i % 2 == 0 else r_mm * 0.45
        xs.append(cx + rad * np.cos(a))
        ys.append(cy + rad * np.sin(a))
    lines = "\n".join(f"G1 X{x:.3f} Y{y:.3f}" for x, y in zip(xs[1:], ys[1:]))
    return DEMO.format(tol=tol, x0=xs[0], y0=ys[0], lines=lines)


def run():
    p = parser("18: run a G-code toolpath through the tolerance-blending lane")
    p.add_argument("--file", type=str, default=None, help=".gcode file (default: built-in star)")
    p.add_argument("--tol", type=float, default=None,
                   help="override every G64 tolerance, mm. Below ~1mm will NOT blend today "
                        "(the planner will say so).")
    p.add_argument("--exact-stop", action="store_true",
                   help="G61: blending off — the arm stops at every corner. The baseline.")
    p.add_argument("--star-mm", type=float, default=60.0, help="demo star radius, mm")
    p.add_argument("--points", type=int, default=5, help="demo star points")
    args = p.parse_args()

    robot = connect(args)
    if not require_go(args, "run a G-code toolpath"):
        return
    prepare_exec(robot, args)

    pose = robot.get_pose()                      # [x, y, z, rx, ry, rz] — METERS + radians
    x0, y0, z0 = pose[0] * 1000.0, pose[1] * 1000.0, pose[2] * 1000.0
    rpy = pose[3:6]                              # G-code has no orientation — hold the current one
    print(f"\nStart pose: ({x0:.1f}, {y0:.1f}, {z0:.1f}) mm, holding rx/ry/rz fixed")

    tol = args.tol if args.tol is not None else 1.5
    if args.file:
        src = Path(args.file).read_text()
        print(f"Parsing {args.file}")
    else:
        src = demo_program(x0, y0, args.star_mm, args.points, tol)
        print(f"Built-in demo: {args.points}-pointed star, r={args.star_mm:.0f}mm, "
              f"G64 P{tol} — alternating sharp and shallow corners")

    try:
        prog = parse(src, start_xyz=(x0, y0, z0), default_blend_mm=tol)
    except GcodeError as exc:
        print(f"\nG-CODE ERROR: {exc}")
        return

    moves = prog.moves()
    if not moves:
        print("no moves in this program")
        return

    # Apply the CLI overrides on top of whatever the file said.
    for m in moves:
        if args.exact_stop:
            m.blend_mm = 0.0
        elif args.tol is not None:
            m.blend_mm = args.tol

    blend = moves[0].blend_mm
    mode = "EXACT STOP (G61) — stops at every corner" if blend <= 0 else \
           f"G64 tolerance {blend}mm"
    print(f"{len(moves)} moves, {mode}")
    if 0 < blend < 1.0:
        print(f"  *** WARNING: {blend}mm is below the ~1mm sampling floor — these corners "
              f"will NOT be rounded (see the planner's UNDER-SAMPLED warning). Safe, but "
              f"you are measuring an unblended path.")
    if prog.ignored:
        print(f"  ignored (this arm cannot do them): "
              f"{', '.join(f'{k} x{n}' for k, n in prog.ignored.items())}")
    print(f"  F (feedrate) is IGNORED — using speed={args.speed}%")

    kw = dict(profile="toppra", speed=args.speed)
    if blend > 0:
        kw.update(blend_mode="tolerance", blend=blend)

    t0 = time.perf_counter()
    for op in prog.ops:
        if isinstance(op, GcodeMove):
            k = dict(kw)
            if op.blend_mm > 0:
                k.update(blend_mode="tolerance", blend=op.blend_mm)
            else:
                k.pop("blend_mode", None)
                k.pop("blend", None)
            robot.move_linear(op.x / 1000.0, op.y / 1000.0, op.z / 1000.0, *rpy, **k)
        elif isinstance(op, GcodeDwell):
            robot.delay(op.seconds)
        elif isinstance(op, GcodePause):
            print("  M0 pause — resuming immediately (no operator prompt in this test)")

    robot.wait_idle(timeout=max(args.timeout, 120.0))
    print(f"\n  total {time.perf_counter() - t0:.2f}s wall clock for {len(moves)} moves")
    print("  grep logs/high_level.log for 'SPLICED' / 'UNDER-SAMPLED' to see what the "
          "planner actually did.")
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
