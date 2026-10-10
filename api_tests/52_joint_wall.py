#!/usr/bin/env python3
"""52: one joint's freedrive wall — its limit is a wall you FEEL in IDLE gravity comp, not a stop. You turn the
joint by hand. Nothing is commanded to move.

Every joint has one (docs/safety_zones.md §2b): at its SOFT limit by default, J6 (no endstop) at its HARD limit
(the cable wind-up boundary). The wall in force comes from command_executor (get_joint_walls — the same values
the dashboard's Walls tab shows and tunes). Needs RTI AND command_executor running the per-joint wall code
(RTI.log: "freedrive joint walls ON at start-up") and the robot homed. 250 Hz telemetry is recorded (recipe
haptic_wall, restored at the end) and saved to temp_data/haptic_wall/; the wall torque is j_tc - gravity(q), the
same gravity function and model RTI uses.

A joint WITH an endstop: pushed through = at its endstop, so the script lowers that joint's breakthrough LIVE to
1 deg short of the endstop for the test and puts the setting back when it ends (also on Ctrl-C).

  A. Turn the joint towards its nearest wall and a little into it: a J{i}:SOFT_LIMIT_WARNING (keys are 0-based),
     you feel the wall, no error.
  B. Hold it inside the wall; when the script says LET GO, let go. PASS: while you held it, the wall torque on
     that joint matched its setting (pose scale x stiffness x depth, capped at its max torque — the wall is stiffer
     where the arm is heavier, utility/haptic_walls.py JOINT WALLS) and pushed it back. Whether it
     then MOVES back is reported, not checked: friction can hold a light joint (J6 did not return at 0.11 Nm).
  C. Push THROUGH it (past its breakthrough). The joint's reaction: warn (default) -> J{i}:JOINT_WALL_WARNING,
     the robot stays in IDLE, and it clears once the joint is back inside the wall; stop -> J{i}:JOINT_WALL,
     the robot stops — the script clears it, re-enables freedrive and you turn the joint back.

    python3 api_tests/52_joint_wall.py --joint 6          # dry run: the wall in force for J6
    python3 api_tests/52_joint_wall.py --joint 6 --go
J2/J3 carry the arm's weight: their walls push up to several Nm. Keep both hands on the arm.
"""
import argparse
import os
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robotclient import RobotClient                                   # noqa: E402

FS = 250.0
RECIPE = "haptic_wall"
_ok, _bad = 0, []


def check(name, cond, detail=""):
    global _ok
    if cond:
        _ok += 1
    else:
        _bad.append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    return cond


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--joint", type=int, default=6, help="joint 1-6 (default 6)")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--wait", type=float, default=120.0, help="seconds you get for each hand step")
    args = ap.parse_args()
    N, j = args.joint, args.joint - 1
    SOFT, WALLW, WALLE = f"J{N}:SOFT_LIMIT_WARNING", f"J{N}:JOINT_WALL_WARNING", f"J{N}:JOINT_WALL"   # keys 1-based (2026-10-09)

    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from utility.robot_factory import create_robot
    w51 = __import__("51_haptic_wall")
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    if not 1 <= N <= int(cfg["joint_num"]):
        print(f"--joint must be 1..{cfg['joint_num']}")
        return 2
    model = create_robot(sysc["active_robot"], gripper_name=cfg["gripper_name"])     # as RTI builds it
    from utility.haptic_walls import make_effective_inertia
    inertia_at = make_effective_inertia(model)                                         # RTI's pose scale

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    base = __import__("46_monitor_baseline")
    claimed = False
    capture = prev_recipe = restore_spec = None
    fric_was = False
    try:
        from robotclient.exceptions import CommandRejected
        try:
            w = robot.get_joint_walls()["walls"][j]
        except CommandRejected as e:
            if e.code != "UNKNOWN_CMD":
                raise
            print("command_executor does not know get_joint_walls: it runs code from before the per-joint walls. "
                  "Restart RTI + command_executor (then re-home).")
            return 2
        if not w["enabled"]:
            print(f"J{N} has no freedrive wall (joint walls off in <haptic_walls>, or no inertia). Nothing to test.")
            return 2
        q = float(robot.get_joints()[j])
        upper = (w["hi_deg"] - q) <= (q - w["lo_deg"])
        side, limit = (1.0, w["hi_deg"]) if upper else (-1.0, w["lo_deg"])
        bt, endstop_room = w["breakthrough_deg"], w["breakthrough_max_deg"]
        test_bt = bt if endstop_room is None or bt <= endstop_room - 1.0 else max(2.0, endstop_room - 1.0)
        k, tmax = w["stiffness_nm_per_deg"], w["max_torque_nm"]
        print(f"J{N} now {q:.1f} deg. Wall at its {w['at'].upper()} limit: [{w['lo_deg']:.1f}, {w['hi_deg']:.1f}] deg; "
              f"nearest the {'upper' if upper else 'lower'} side at {limit:.1f} deg ({abs(limit - q):.0f} deg away). "
              f"{w['frequency_hz']:.2f} Hz, damping ratio {w['damping_ratio']:.2f} (= {k:.4g} Nm/deg, "
              f"{w['damping_nm_s_per_deg']:.3g} Nm.s/deg at its lightest pose), max {tmax:.3g} Nm, pushed through "
              f"{bt:.1f} deg past it -> {w['freedrive']}.")
        if test_bt != bt:
            print(f"J{N} has an endstop {endstop_room:.1f} deg past the wall: the test lowers its breakthrough to "
                  f"{test_bt:.1f} deg live and puts it back afterwards.")
        if N in (2, 3):
            print(f"!! J{N} carries the arm's weight: its wall pushes up to {tmax:.1f} Nm. Keep both hands on the arm.")
        if not args.go:
            print("DRY RUN: re-run with --go. Nothing moved, nothing changed.")
            return 0
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True:
            print(f"Robot must be ENABLED and homed (got state={st.get('state')} homed={st.get('homed')}).")
            return 2

        base.claim_control(robot)
        claimed = True
        fric_was = bool(robot.status().get("friction_comp"))
        if fric_was:
            robot.set_friction_comp(False)
            print("Friction compensation switched OFF for this test (the wall is judged as torque sent - gravity); "
                  "it is switched back on at the end.")
        if test_bt != bt:
            restore_spec = {key: ("auto" if v is None else v) for key, v in w["spec"].items()}
            robot.set_joint_wall(N, breakthrough_deg=test_bt)
        capture = base.Capture()
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        if st.get("mode") == "EXEC":
            robot.enter_idle_mode(wait=True)
        robot.set_gravity_comp(True)
        robot.release_control()
        claimed = False
        time.sleep(0.5)
        A_DEG = min(4.0, 0.5 * test_bt)
        B_LO, B_HI = 0.25 * test_bt, 0.75 * test_bt

        def past():                                   # deg past the wall (> 0 = in it)
            return side * (float(robot.get_joints()[j]) - limit)

        def watch(seconds, until, note):
            end, last, s = time.monotonic() + seconds, 0.0, robot.status()
            while time.monotonic() < end:
                p = past()
                s = robot.status()
                if until(p, s):
                    print()
                    return p, s
                if time.monotonic() - last > 0.3:
                    print(f"   J{N} {p:+7.2f} deg past the wall   {note}", end="\r")
                    last = time.monotonic()
                time.sleep(0.03)
            print()
            return None, s

        print(f"\nA. Turn J{N} by hand towards {limit:.1f} deg and ~{A_DEG:.0f} deg past it (you should feel the wall).")
        p, s = watch(args.wait, lambda p, s: p > A_DEG, f"(go ~{A_DEG:.0f} deg in)")
        if not check(f"J{N} reached ~{A_DEG:.0f} deg into its wall", p is not None, "" if p is not None else "not in time"):
            return 1
        errs = s.get("errors") or []
        check(f"past the soft limit: {SOFT}", SOFT in errs, str(errs))
        check("in the wall: no error, robot still ENABLED in IDLE", WALLE not in errs and s.get("state") == "ENABLED"
              and s.get("mode") == "IDLE", "" if WALLE not in errs else str(errs))

        print(f"\nB. Hold it {B_LO:.1f}-{B_HI:.1f} deg into the wall. When the script says LET GO, let go.")
        start = len(capture.packets)
        held, since, end = None, None, time.monotonic() + args.wait
        while time.monotonic() < end:
            p = past()
            since = (since or time.monotonic()) if B_LO <= p <= B_HI else None
            if since and time.monotonic() - since >= 1.0:
                held = p
                break
            time.sleep(0.03)
        if check(f"held {B_LO:.1f}-{B_HI:.1f} deg in", held is not None, "" if held is not None else "not held in time"):
            print("   >>> LET GO NOW <<<")
            time.sleep(2.5)
            p1 = past()
            d = w51.to_arrays(capture.packets[start:])
            path = w51.save_recording(d, f"52 J{N} B", wall=w, side=side, limit_deg=limit)
            tk = d["tick"]
            t_go = tk[-1] - 2.5 * FS if tk.size else 0.0
            sel = (tk >= t_go - FS) & (tk < t_go) & (d["mode"] == "IDLE")
            if sel.any():
                tau = -side * w51.wall_torque({"j_p": d["j_p"][sel], "j_tc": d["j_tc"][sel]}, model)[:, j]
                depth = side * (np.degrees(d["j_p"][sel, j]) - limit)
                ieff, scale = np.zeros(6), []
                for qrow in d["j_p"][sel]:
                    inertia_at(qrow, ieff)
                    scale.append(max(1.0, ieff[j] / w["inertia_kg_m2"]))
                expect = np.minimum(np.asarray(scale) * k * np.maximum(depth, 0.0), tmax)
                got, want = float(np.median(tau)), float(np.median(expect))
                check(f"while held, the wall torque on J{N} pushed it back and matched its setting (pose scale x k x "
                      f"depth, capped at {tmax:.3g} Nm)", got > 0.0 and abs(got - want) <= 0.3 * want + 0.005,
                      f"{got:.4f} Nm measured vs {want:.4f} expected at {float(np.median(depth)):.2f} deg, "
                      f"pose scale x{float(np.median(scale)):.2f}")
            else:
                check(f"telemetry recorded while J{N} was held", False, f"{tk.size} samples")
            # 2026-10-02 J6: 3.8 -> 11.6 deg was reported as "friction held it" — it had sprung back out, stopped where
            # the wall torque fell below the static friction, and a hand then pushed it back IN 2.3 s later.
            how = ("" if p1 < 0.5 * held else " — it went DEEPER: something pushed it in (your hand?)" if p1 > held + 0.5
                   else " — friction held it (the wall torque above is what matters)")
            print(f"   2.5 s after letting go: {p1:.1f} deg in the wall (held at {held:.1f}){how}"
                  + ("" if path is None else f"   (saved {path.relative_to(_ROOT)})"))

        thr = limit + side * test_bt
        print(f"\nC. Push THROUGH it, past {thr:.1f} deg ({test_bt:.1f} deg past the wall).")
        p, s = watch(args.wait, lambda p, s: WALLW in (s.get("errors") or []) or WALLE in (s.get("errors") or []),
                     f"(through at {test_bt:.1f} deg)")
        errs = s.get("errors") or []
        if w["freedrive"] == "warn":
            check(f"pushed through: {WALLW} (warn)", WALLW in errs and WALLE not in errs, str(errs))
            time.sleep(0.3)
            s = robot.status()
            check("...the robot stayed ENABLED in IDLE", s.get("state") == "ENABLED" and s.get("mode") == "IDLE",
                  f"state={s.get('state')} mode={s.get('mode')}")
            print("   Now turn it back inside the wall.")
            p, s = watch(args.wait, lambda p, s: WALLW not in (s.get("errors") or []) and p < 0, "(back inside)")
            check("back inside: the warning cleared", p is not None and WALLW not in (s.get("errors") or []))
        else:
            check(f"pushed through: {WALLE} (stop)", WALLE in errs, str(errs))
            time.sleep(0.3)
            s = robot.status()
            check("...the robot stopped (DISABLED, ACTIVE_ERROR)", s.get("state") == "DISABLED"
                  and s.get("mode") == "ACTIVE_ERROR", f"state={s.get('state')} mode={s.get('mode')}")
            base.claim_control(robot)
            claimed = True
            robot.clear_errors(wait=True)
            robot.enable(wait=True)
            robot.set_gravity_comp(True)
            robot.release_control()
            claimed = False
            print("   Cleared and re-enabled freedrive. Now turn it back inside the wall.")
            p, s = watch(args.wait, lambda p, s: p < 0, "(back inside)")
            check("back inside, nothing fired again on the way", p is not None and WALLE not in (s.get("errors") or []))
    finally:
        try:
            if restore_spec is not None or prev_recipe or fric_was:
                if not claimed:
                    base.claim_control(robot)
                    claimed = True
                if fric_was:
                    robot.set_friction_comp(True)
                if restore_spec is not None:
                    robot.set_joint_wall(N, **restore_spec)
                    print(f"\nJ{N}'s wall setting restored.")
                if prev_recipe:
                    robot.set_recipe(prev_recipe)
        except Exception as e:
            print(f"\n!! could not restore ({e}) — check J{N} in the dashboard's Walls tab (or restart RTI + "
                  f"command_executor: the robot XML was never changed)")
        if capture is not None:
            capture.stop()
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()
    print(f"\n{_ok} passed, {len(_bad)} failed" + (f": {_bad}" if _bad else ""))
    return 1 if _bad else 0


if __name__ == "__main__":
    sys.exit(main())
