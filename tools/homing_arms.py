#!/usr/bin/env python3
"""The arms homing has learned — robotics/homing_arms.py, temp_data/homing_arms.json (2026-10-09, homing plan H1).

After every homing RTI compares each joint's endstop latch (within one motor revolution) with where THIS arm's joint
always homes: it recognises the arm, learns the first one (known after 3 homings that agree), and FAILS homing when a
joint of a recognised arm is more than 1 deg off — a false stall, a skipped belt, a slipping coupling — or (2026-10-14)
when the first joints match no known arm, unless --learn-next was set.

    .venv/bin/python tools/homing_arms.py                       # list the arms, their joints' spots and spread
    .venv/bin/python tools/homing_arms.py --name arm-1 grey      # give an arm a name (shown in the homing log)
    .venv/bin/python tools/homing_arms.py --forget arm-2         # forget an arm (it is learned again from scratch)
    .venv/bin/python tools/homing_arms.py --accept-last          # after a DELIBERATE mechanical change (belt tensioned,
                                                                 # motor re-mounted): the last homing that failed becomes
                                                                 # the reference for the joints that were off
    .venv/bin/python tools/homing_arms.py --learn-next           # a DIFFERENT arm is connected: the next homing learns it
                                                                 # (once an arm is known, an arm no known one matches
                                                                 # FAILS homing — 2026-10-14); --no-learn-next cancels

Edit while the arm is not homing: RTI re-reads the file when a homing starts and writes it when one ends.
"""
import math
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

import robotics.homing_arms as ha                                    # noqa: E402
import utility.display_time as dt                                    # noqa: E402


def setup():
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML, TEMP_DATA_DIR
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    dt.set_zone(sysc.get("display_timezone"))
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=sysc["active_gripper"])
    n = int(cfg["joint_num"])
    return (os.path.join(str(TEMP_DATA_DIR), "homing_arms.json"), [float(g) for g in cfg["gear_ratio"][:n]],
            [int(f) for f in cfg["home_offset_gripper_dependent"][:n]], [2 ** int(b) for b in cfg["motor_encoder"][:n]],
            str(cfg.get("robot_name") or "ARM"))


def listing(table, gears, counts):
    if not table["arms"]:
        return "no arms learned yet — each arm is learned over its first 3 homings"
    out = []
    for a in table["arms"]:
        out.append(f"{a['id']}  {a.get('uid', '(no ID yet)')}  {a.get('name') or '(no name)'}  — "
                   f"{'KNOWN' if a['known'] else 'learning'}, {a['homings']} homing(s) ({len(a.get('history', []))} in the "
                   f"history), {dt.fmt_utc_text(a.get('first_utc')) or '?'} .. {dt.fmt_utc_text(a.get('last_utc')) or '?'}")
        for key, c, n, spread in ha.joint_spots(a, gears, counts):
            out.append(f"    {key:34s} spot {c:8.1f} motor ticks, {n} homing(s), within ±{spread:.3f} deg")
    lf = table.get("last_failed")
    if lf:
        out.append(f"last FAILED homing: arm {lf['arm']} at {dt.fmt_utc_text(lf['utc'], '%Y-%m-%d %H:%M:%S %Z')} (gripper {lf['gripper']}) — --accept-last takes it "
                   f"after a deliberate change")
    return "\n".join(out)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path, gears, dep, counts, prefix = setup()
    table = ha.load_table(path)
    if ha.ensure_uids(table, prefix):                 # arms learned before IDs existed get theirs now, once
        ha.save_table(path, table)
    if not argv:
        print(listing(table, gears, counts))
        return 0
    try:
        if argv[0] == "--name" and len(argv) == 3:
            msg = ha.rename(table, argv[1], argv[2])
        elif argv[0] == "--forget" and len(argv) == 2:
            msg = ha.forget(table, argv[1])
        elif argv[0] == "--accept-last" and len(argv) == 1:
            msg = ha.accept_last(table, gears, dep, counts)
        elif argv[0] == "--learn-next" and len(argv) == 1:
            msg = ha.set_learn_next(table, True)
        elif argv[0] == "--no-learn-next" and len(argv) == 1:
            msg = ha.set_learn_next(table, False)
        else:
            print(__doc__)
            return 2
    except KeyError as exc:
        print(exc.args[0])
        return 1
    ha.save_table(path, table)
    print(msg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
