#!/usr/bin/env python3
"""62: does the gripper's reported position put the jaws where they really are? (2026-10-07, S3f.) Moves NOTHING by itself.

The collision worker can place the gripper's jaws where the gripper SAYS they are, instead of over their whole stroke
(<self_collision><jaws>measured</jaws> in config/system.xml; shipped "swept" until this check passes). In firmware mode the
gripper reports 0-255: the motor's encoder mapped linearly between the two endstops it found when it calibrated, 0 = open,
255 = closed (read from the STEPFOC / Spectral BLDC firmware), c = raw / 255. In motor mode (2026-10-07) the opening in
metres from the endstop it stall-homed against (the open end), c = 1 - opening / stroke. The model turns c into a jaw
position on the CAD stroke: each jaw closes c x stroke from the open end. This script checks that against a ruler — run it
in the mode(s) you will use (switch the gripper's mode from the dashboard between readings if you want both).

You: set the gripper to an opening (dashboard Gripper tab — fully open, half, closed, a few in between), measure the gap
between the two jaws' INNER faces (calipers or a ruler, at the fingertips), type it in mm. The script: reads the gripper's
report (as FILES in /dev/shm, never a live segment), prints the gap the model places the jaws at, and the difference.
The model's jaws are widened by <jaw_band_mm> each way, so a difference within 2 x the band is covered; the summary says
whether to switch <jaws> to measured. Enter alone = read and print only; q = summary and quit.

    .venv/bin/python api_tests/62_jaw_position.py
"""
import json
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

from gui_jaw_view import OPEN_GAP_MM   # noqa: E402  the CAD's open gap between the jaws' inner faces (one table, shared)


def text(name):
    import struct
    with open(f"/dev/shm/{name}", "rb") as fh:
        b = fh.read(260)
    k = struct.unpack("<I", b[:4])[0]
    return b[4:4 + k].decode(errors="replace")


def i32(name, n=1):
    with open(f"/dev/shm/{name}", "rb") as fh:
        return np.frombuffer(fh.read(4 * n), dtype=np.int32).copy()


def f64(name, n=1):
    with open(f"/dev/shm/{name}", "rb") as fh:
        return np.frombuffer(fh.read(8 * n), dtype=np.float64).copy()


def main():
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from self_collision_worker import jaw_closed_fraction, measured_jaws_refusal
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=sysc["active_gripper"])
    pol = sysc["self_collision"]
    gfile = cfg["gripper_collision"].get("file")
    if gfile not in OPEN_GAP_MM:
        print(f"The active gripper ({sysc['active_gripper']}) has no jaw model here (collision file {gfile!r}). Nothing to check.")
        return 2
    tool = json.load(open(os.path.join(GRIPPERS_DIR, gfile)))
    why = measured_jaws_refusal(cfg, tool)
    if why:
        print(f"Measured jaws are not possible for {sysc['active_gripper']}: {why}. Nothing to check.")
        return 2
    stroke_mm = 1e3 * float(tool["jaws"][0]["stroke_m"])
    open_gap = OPEN_GAP_MM[gfile]
    band = float(pol["jaw_band_mm"])
    node = int(cfg["joint_num"])
    hd = cfg.get("gripper_homing_direction")
    motor_ok = hd is not None and hd == hd and int(hd) == 0
    print(f"{sysc['active_gripper']}: CAD gap {open_gap:.1f} mm open, {stroke_mm:.1f} mm of travel per jaw; band {band:g} mm per jaw "
          f"(the model covers a difference up to {2 * band:g} mm). <jaws> is now '{pol['jaws']}'.")
    print("Set an opening, measure the gap between the jaws' inner faces, type it in mm (Enter = read only, q = done).\n")
    print(f"{'#':>3} {'mode':>9} {'cal':>4} {'fresh':>6} {'raw':>4} {'closed':>7} {'model gap':>10} {'measured':>9} {'diff':>7}  worker"
          f"   (raw: firmware 0-255, motor mode: mm open)")
    rows = []
    while True:
        try:
            ans = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if ans.lower() == "q":
            break
        rti = {"gripper_ctrl_mode": i32("gripper_ctrl_mode"), "old_data_warning": i32("old_data_warning", 16)}
        grip = {"gripper_calibrated": i32("gripper_calibrated"), "gripper_position": i32("gripper_position"),
                "gripper_homed": i32("gripper_homed"), "gripper_joint_position": f64("gripper_joint_position"),
                "gripper_stroke_m": f64("gripper_stroke_m"), "gripper_ticks_per_meter": f64("gripper_ticks_per_meter")}
        mode, cal, raw = int(rti["gripper_ctrl_mode"][0]), int(grip["gripper_calibrated"][0]), int(grip["gripper_position"][0])
        fresh = int(rti["old_data_warning"][node]) == 0
        try:
            worker = float(f64("sc_jaw_closed")[0])
        except OSError:
            worker = float("nan")
        c_ = jaw_closed_fraction(rti, grip, node, text("homed_state"), motor_ok)       # exactly what the worker computes
        c = 0.0 if c_ is None else c_
        model_gap = open_gap - 2.0 * c * stroke_mm
        measured = None
        if ans:
            try:
                measured = float(ans.replace(",", "."))
            except ValueError:
                print("  (not a number — read only)")
        usable = c_ is not None
        if mode == 1:
            raw = int(round(1e3 * float(grip["gripper_joint_position"][0])))           # shown in the raw column: mm open
        diff = (measured - model_gap) if measured is not None else None
        rows.append((usable, c, model_gap, measured, diff))
        print(f"{len(rows):3d} {('firmware' if mode == 2 else 'motor' if mode == 1 else 'none'):>9} {cal:>4} {('yes' if fresh else 'NO'):>6} "
              f"{raw:4d} {100 * c:6.0f}% {model_gap:8.1f}mm " + (f"{measured:7.1f}mm {diff:+6.1f}" if diff is not None else f"{'':>9} {'':>7}")
              + "  " + ("swept" if worker < 0 else f"{100 * worker:.0f}% closed" if worker == worker else "-")
              + ("" if usable else "   <- not usable: the worker would use the swept jaws here"
                 + (" (motor mode needs the gripper to home toward its open end)" if mode == 1 and not motor_ok else "")))
    pts = [r for r in rows if r[0] and r[4] is not None]
    print()
    if not pts:
        print("No measured openings with a usable report (firmware mode calibrated, or motor mode homed; fresh). Nothing decided.")
        return 1
    d = np.array([r[4] for r in pts])
    cs = np.array([r[1] for r in pts])
    print(f"{len(pts)} measured openings ({100 * cs.min():.0f}-{100 * cs.max():.0f} % closed): real minus model gap "
          f"{d.min():+.1f} .. {d.max():+.1f} mm (each jaw: half of that).")
    ok = np.all(np.abs(d) <= 2 * band) and cs.max() - cs.min() >= 0.5
    if ok:
        print(f"Within 2 x the band ({2 * band:g} mm) over at least half the stroke: the mapping holds — set <jaws>measured</jaws> in "
              f"config/system.xml and restart the worker (pkill -f self_collision_worker.py; the supervisor restarts it).")
    elif cs.max() - cs.min() < 0.5:
        print("Measure over a wider range (fully open to closed) before deciding.")
    else:
        print(f"Some openings are off by more than 2 x the band: keep <jaws>swept</jaws> (or raise <jaw_band_mm>) and look at why "
              f"(the calibration, the direction, another rail length).")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
