#!/usr/bin/env python3
"""Homing report: where each joint homed, how repeatably, and whether that moved (2026-10-09). Reads logs; moves nothing.

Every homing logs, per joint, the motor encoder position it latched at the endstop (RTI.log: "[Homing] Jn [CANID ..]
reference set: master_pos=..."). This groups the log into homings (one per "Sequence started" line) and prints per joint:

  * the spread of the latch over the homings, in degrees AT THE JOINT (motor ticks / encoder counts / gear x 360) — how
    repeatable homing is on this arm;
  * with --split TIME: the shift of the latch between the homings before and after TIME (e.g. a PID change) and the
    home_offset that keeps the joint reading the same angle at the same physical pose. A shift means the endstop is
    latched somewhere else (how hard the joint is pressed into its stop when the position is taken), not that the arm
    moved — check it at the joint's calibration screw before writing it into the robot XML.

The latch is compared by its place WITHIN one motor revolution (mod the encoder counts), so a different multi-turn count
after a power cycle is not a shift; a real shift of half a motor revolution or more cannot be told apart this way.
2026-10-09, the old gains, the last 10 homings: J1 / J4 / J5 within +-0.01 deg, J6 +-0.05, J2 +-0.06, J3 -0.05..+0.15.

    .venv/bin/python tools/homing_report.py [--log logs/RTI.log] [--since "2026-10-04"] [--split "2026-10-09 09:00"]
                                            [--sessions] [--tz Europe/Zagreb]
Times (shown and typed) are in the box's display zone — config/system.xml <display_timezone>, the dashboard's "Time"
card; --tz overrides it. The log itself keeps UTC.
"""
import argparse
import math
import os
import re
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

from utility.logger_setup import parse_line  # noqa: E402  # both line formats (the one before 2026-10-15 too)

_REF = re.compile(r"^J(\d+) \[CANID \d+\] reference set: master_pos=(-?\d+)")
_TWO_PASS = re.compile(r"^J(\d+) \[CANID \d+\] two-pass difference: (\d+) ticks")
_LATCH_FAILED = re.compile(r"^J(\d+) \[CANID \d+\] release: latched position None")


def parse_sessions(lines):
    """Homings from RTI.log lines: [{"t": start (unix s), "end": done time or None, "failed": bool,
    "joints": {joint index 0..: {"latch": ticks, "two_pass": ticks or None, "latch_failed": bool}}}]."""
    sessions = []
    for line in lines:
        rec = parse_line(line)
        if rec is None or rec["name"] != "rti.homing":
            continue
        t, line = rec["time"], rec["message"]
        if "Sequence started" in line:
            sessions.append({"t": t, "end": None, "failed": False, "joints": {}})
            continue
        if not sessions:
            continue
        s = sessions[-1]
        if "All joints homed successfully" in line:
            s["end"] = t
        elif "FAILED" in line:
            s["failed"] = True
        elif (m := _REF.search(line)) is not None:
            s["joints"].setdefault(int(m.group(1)) - 1, {"two_pass": None, "latch_failed": False})["latch"] = int(m.group(2))
        elif (m := _TWO_PASS.search(line)) is not None:
            s["joints"].setdefault(int(m.group(1)) - 1, {"two_pass": None, "latch_failed": False})["two_pass"] = int(m.group(2))
        elif (m := _LATCH_FAILED.search(line)) is not None:
            s["joints"].setdefault(int(m.group(1)) - 1, {"two_pass": None, "latch_failed": False})["latch_failed"] = True
    return sessions


def circular_centre(latches, counts):
    """(centre in [0, counts), each latch's signed distance from it in ticks) — on the circle of one motor revolution, so
    a joint whose latch sits near the wrap (PAR6 J2: ~9 of 16384) is not split in two."""
    a = 2.0 * math.pi * (np.asarray(latches, float) % counts) / counts
    c = math.atan2(float(np.mean(np.sin(a))), float(np.mean(np.cos(a)))) % (2.0 * math.pi)
    dev = (a - c + math.pi) % (2.0 * math.pi) - math.pi
    return c / (2.0 * math.pi) * counts, dev / (2.0 * math.pi) * counts


def wrap_ticks(d, counts):
    """A tick difference brought into [-counts/2, counts/2)."""
    return (d + counts / 2.0) % counts - counts / 2.0


def ticks_to_joint_deg(ticks, counts, gear):
    return np.asarray(ticks, float) / counts / gear * 360.0


def offset_for_shift(home_offset, shift_ticks, counts, gear, motor_dir):
    """The home_offset that keeps a joint reading the same angle at the same physical pose when its endstop latch moved
    by shift_ticks (motor ticks, new - old). From SourceRoboticsToolbox.Joint + homing._update_home_reference: dir 0 reads
    offset + (pos - master) k, dir 1 reads offset - (pos - master) k, k = 2 pi / (counts gear) — so the offset moves by
    +shift k for dir 0 and -shift k for dir 1."""
    k = 2.0 * math.pi / (counts * gear)
    return home_offset + (shift_ticks * k if int(motor_dir) == 0 else -shift_ticks * k)


def parse_time(text):
    """A typed time, in the display zone -> epoch seconds."""
    from utility.display_time import parse_local
    try:
        return parse_local(text)
    except ValueError as exc:
        raise SystemExit(str(exc))


def utc(t):
    """An epoch time as shown (the display zone; the name is historical)."""
    from utility.display_time import fmt
    return fmt(t, "%m-%d %H:%M")


def group_report(name, sessions, n, counts, gear):
    """Per joint: (centre ticks, deviations in deg at the joint) for the homings that latched that joint; prints a table."""
    print(f"\n{name}: {len(sessions)} homing(s)" + (f", {utc(sessions[0]['t'])} .. {utc(sessions[-1]['t'])}" if sessions else ""))
    out = {}
    for j in range(n):
        lat = [s["joints"][j]["latch"] for s in sessions if "latch" in s["joints"].get(j, {})]
        if not lat:
            continue
        c, dev = circular_centre(lat, counts[j])
        deg = ticks_to_joint_deg(dev, counts[j], gear[j])
        tp = [s["joints"][j]["two_pass"] for s in sessions if s["joints"].get(j, {}).get("two_pass") is not None]
        lf = sum(1 for s in sessions if s["joints"].get(j, {}).get("latch_failed"))
        print(f"  J{j + 1}: n={len(lat):3d}  latch spread {deg.min():+.3f} .. {deg.max():+.3f} deg at the joint (sd {deg.std():.3f})"
              + (f"  two-pass diff median {np.median(tp):.0f} ticks" if tp else "")
              + (f"  release latch fell back to the end: {lf}x" if lf else ""))
        out[j] = (c, deg)
    done = [s["end"] - s["t"] for s in sessions if s["end"] is not None]
    if done:
        print(f"  duration: median {np.median(done):.1f} s (min {min(done):.1f}, max {max(done):.1f}); "
              f"{sum(1 for s in sessions if s['end'] is None)} did not finish")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--log", default=os.path.join(_ROOT, "logs", "RTI.log"))
    ap.add_argument("--since", help="only homings from this time on (UTC)")
    ap.add_argument("--split", help="compare the homings before and after this time (UTC)")
    ap.add_argument("--sessions", action="store_true", help="also list every homing")
    ap.add_argument("--tz", help="show / read times in this zone (default: system.xml <display_timezone>)")
    a = ap.parse_args(argv)
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    from utility.display_time import set_zone, zone_name
    set_zone(a.tz or sysc.get("display_timezone"))
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=sysc["active_gripper"])
    n = int(cfg["joint_num"])
    counts = [2 ** int(b) for b in cfg["motor_encoder"][:n]]
    gear = [abs(float(g)) for g in cfg["gear_ratio"][:n]]
    with open(a.log, errors="replace") as fh:
        sessions = parse_sessions(fh)
    if a.since:
        sessions = [s for s in sessions if s["t"] >= parse_time(a.since)]
    if not sessions:
        print("no homings in the log" + (" since then" if a.since else ""))
        return 1
    print(f"(times in {zone_name()})")
    if a.sessions:
        print("homing        duration  " + "  ".join(f"{'J' + str(j + 1):>9}" for j in range(n)) + "   (latch, motor ticks)")
        for s in sessions:
            d = f"{s['end'] - s['t']:6.1f} s" if s["end"] is not None else ("FAILED  " if s["failed"] else "unfinished")
            print(f"{utc(s['t'])}  {d:>8}  " + "  ".join(f"{s['joints'].get(j, {}).get('latch', ''):>9}" for j in range(n)))
    if not a.split:
        group_report("All", sessions, n, counts, gear)
        return 0
    t_split = parse_time(a.split)
    before = group_report("Before", [s for s in sessions if s["t"] < t_split], n, counts, gear)
    after = group_report("After", [s for s in sessions if s["t"] >= t_split], n, counts, gear)
    print("\nShift of the latch, after - before (a shift moves every angle this joint reads by the same amount):")
    for j in range(n):
        if j not in before or j not in after:
            continue
        shift = wrap_ticks(after[j][0] - before[j][0], counts[j])
        deg = float(ticks_to_joint_deg(shift, counts[j], gear[j]))
        noise = max(0.05, 3.0 * float(before[j][1].std()), 3.0 * float(after[j][1].std()))
        new = offset_for_shift(float(cfg["home_offset"][j]), shift, counts[j], gear[j], cfg["motor_dir"][j])
        where = "the gripper XML" if int(cfg["home_offset_gripper_dependent"][j]) else "the robot XML"
        verdict = (f"MOVED — home_offset {float(cfg['home_offset'][j]):.6f} -> {new:.6f} ({where}) keeps the old "
                   f"readings; check at the calibration screw first") if abs(deg) > noise else "same within the scatter"
        print(f"  J{j + 1}: {shift:+7.0f} motor ticks = {deg:+.3f} deg (scatter {noise:.3f})  {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
