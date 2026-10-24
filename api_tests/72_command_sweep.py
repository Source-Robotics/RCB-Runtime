#!/usr/bin/env python3
"""72: every runtime-setting command once, against the SIMULATED robot (2026-10-13).

Coverage (docs/testing.md) showed ten of RTI's command handlers had never run in any test and seven more only partly
— mostly the dashboard's buttons (gravity / inertia / Coriolis / friction compensation, the jog settings, the gripper
mode and calibration, PID gains, zone / filter / recipe reloads, the Hall check, the joint walls, the payload). That is
the class of path that killed RTI on 2026-10-01 (every Apply in the Walls tab). Each command here:

  * a VALID value takes effect where the robot reports it (status(), its shared memory read as files), and
  * a MALFORMED value is refused by the executor (BAD_ARGS / the command's own refusal) and changes nothing,

while RTI keeps ticking and writes no CRITICAL line (a handler that raises is caught and logged CRITICAL — the command
silently did nothing). Every setting is put back at the end: the jog settings to system.xml's <jog_defaults>, the
rest to what they were.

Refused unless RTI runs the simulated robot (/dev/shm/sim_mode reads ON): it changes PID gains, the payload, walls.
In the simulated suite: .venv/bin/python tools/sim_run.py --tests "01_mode --go,suite"   (needs IDLE, enabled, homed)
Exit 0 = all passed, 1 = a check failed, 2 = refused.
"""
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _common import parser, connect                                        # noqa: E402
from robotclient import CommandRejected                                     # noqa: E402
import config.xml_parser as xp                                              # noqa: E402
from project_paths import GRIPPERS_DIR, RTI_LOG, ROBOTS_DIR, SYSTEM_XML     # noqa: E402

fails = []


def check(name, ok, detail=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + ("" if ok or not detail else f"   [{detail}]"), flush=True)
    if not ok:
        fails.append(name)


def shm_float(name):
    """A one-value segment read as a FILE (never attached): float64 (8 bytes) or int32 (4 bytes), as a float."""
    try:
        with open(f"/dev/shm/{name}", "rb") as fh:
            raw = fh.read(8)
    except OSError:
        return None
    if len(raw) == 8:
        return struct.unpack("<d", raw)[0]
    return float(struct.unpack("<i", raw[:4])[0]) if len(raw) >= 4 else None


def jog_state():
    """RTI's mirror of the jog handler's settings: (accel time, profile 0 = trapezoidal / 1 = scurve, jerk factor,
    control mode 0 = pid / 1 = pd)."""
    return (shm_float("jog_accel_time"), shm_float("jog_profile_idx"), shm_float("jog_jerk_factor"),
            shm_float("jog_control_mode"))


def shm_string(name):
    try:
        raw = open(f"/dev/shm/{name}", "rb").read()
        n = int.from_bytes(raw[:4], "little")
        return raw[4:4 + n].decode(errors="replace")
    except OSError:
        return ""


def ticking():
    a = shm_float("loop_tick")
    time.sleep(0.15)
    b = shm_float("loop_tick")
    return a is not None and b is not None and b > a


def criticals_since(offset):
    try:
        with open(RTI_LOG, errors="replace") as fh:
            fh.seek(offset)
            return [ln.strip() for ln in fh if "CRITICAL" in ln]
    except OSError:
        return []


def refused(fn, *codes):
    """(True, code) when fn raises CommandRejected with one of `codes` (any code if none given)."""
    try:
        fn()
        return False, "accepted"
    except CommandRejected as e:
        return (not codes or e.code in codes), e.code


def settle(seconds=0.3):
    time.sleep(seconds)


def run(robot):
    st = robot.get_status()
    print(f"robot: mode={st.get('mode')} state={st.get('state')} homed={st.get('homed')}")
    if st.get("mode") != "IDLE":
        robot.enter_idle_mode(wait=True)
    log0 = os.path.getsize(RTI_LOG) if os.path.exists(RTI_LOG) else 0
    s0 = robot.status()
    raw = robot._immediate                                     # the wire, for values the typed client would not send

    print("\n1. compensation switches (status() reports each)")
    for key, setter in (("gravity_comp", robot.set_gravity_comp), ("inertia_comp", robot.set_inertia_comp),
                        ("coriolis_comp", robot.set_coriolis_comp), ("friction_comp", robot.set_friction_comp),
                        ("friction_ff", robot.set_friction_ff), ("can_read_first", robot.set_can_read_first)):
        was = bool(s0.get(key))
        setter(not was)
        settle()
        flipped = bool(robot.status().get(key)) == (not was)
        setter(was)
        settle()
        back = bool(robot.status().get(key)) == was
        check(f"{key}: switched and switched back", flipped and back, f"was {was}")
        ok, code = refused(lambda k=key: raw(k, ["yes"]), "BAD_ARGS")
        settle()
        check(f"{key}: a malformed value is refused ({code}) and changes nothing",
              ok and bool(robot.status().get(key)) == was, code)

    print("\n2. jog settings (RTI mirrors them in shared memory)")
    robot.set_jog_accel(0.4)
    robot.set_jog_profile("trapezoidal")
    robot.set_jog_jerk_factor(2.0)
    robot.set_jog_control_mode("pd")
    settle()
    js = jog_state()
    check("jog accel 0.4 s, trapezoidal, jerk factor 2.0, pd: all taken (RTI's mirror), no CRITICAL",
          js[0] is not None and abs(js[0] - 0.4) < 1e-9 and js[1:] == (0.0, 2.0, 1.0) and not criticals_since(log0),
          (js, criticals_since(log0)[:1]))
    for name, bad in (("jog_accel", "abc"), ("jog_accel", "-1"), ("jog_jerk_factor", "x"), ("jog_profile", "zigzag"),
                      ("jog_control_mode", "pi")):
        ok, code = refused(lambda n=name, b=bad: raw(n, [b]), "BAD_ARGS")
        check(f"{name} {bad!r} refused ({code})", ok, code)
    settle()
    check("... none of them changed a jog setting or reached RTI as an exception (no CRITICAL)",
          jog_state() == js and not criticals_since(log0), (jog_state(), criticals_since(log0)[:1]))

    print("\n3. gripper (simulated CAN gripper)")
    mode0 = shm_float("gripper_ctrl_mode")              # put back at the end: the mode lasts until RTI restarts
    robot.set_gripper_ctrl_mode(2)
    settle()
    m2 = shm_float("gripper_ctrl_mode")
    robot.set_gripper_ctrl_mode(1)
    settle()
    check("gripper_ctrl_mode 2 then 1 (firmware / motor), in shared memory", m2 == 2.0 and shm_float("gripper_ctrl_mode")
          == 1.0, f"{m2} -> {shm_float('gripper_ctrl_mode')}")
    ok, code = refused(lambda: raw("gripper_ctrl_mode", ["3"]), "BAD_ARGS")
    settle()
    check(f"gripper_ctrl_mode 3 refused ({code}), the mode unchanged", ok and shm_float("gripper_ctrl_mode") == 1.0, code)
    robot.gripper_calibrate()
    settle(4.0)
    check("gripper_calibrate: no error after 4 s", robot.status().get("err") == 0, robot.status().get("errors"))
    if mode0 in (1.0, 2.0):
        robot.set_gripper_ctrl_mode(int(mode0))
        settle()
    check(f"the gripper's control mode back to the one found ({mode0})",
          mode0 not in (1.0, 2.0) or shm_float("gripper_ctrl_mode") == mode0, shm_float("gripper_ctrl_mode"))

    print("\n4. PID gains (the XML's own values: a no-op that runs the whole path)")
    sysc = xp.load_system_config(str(SYSTEM_XML), str(ROBOTS_DIR))
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    gains = {k: float(cfg[f"motor_{k}"][3]) for k in ("KPP", "KPV", "KIV")}
    robot.set_pid_gains(3, **gains)                            # index 3 = J4 (set_pid_gains is 0-based)
    settle(0.5)
    check(f"set_pid_gains J4 {gains}: RTI takes it, no error", robot.status().get("err") == 0 and ticking())
    for label, kw in (("joint 42", dict(joint=42, KPP=1.0)), ("gain 'kpp'", dict(joint=3, kpp=1.0)),
                      ("KPV NaN", dict(joint=3, KPV=float("nan")))):
        ok, code = refused(lambda kw=kw: robot.set_pid_gains(kw.pop("joint"), **kw), "BAD_ARGS")
        check(f"set_pid_gains with {label} refused ({code})", ok, code)

    print("\n5. reloads and the recipe")
    robot.reload_zones()
    robot.reload_filters()
    robot.reload_recipes()
    settle(0.5)
    check("reload_zones / reload_filters / reload_recipes: no error, RTI ticking", robot.status().get("err") == 0
          and ticking())
    rec = robot.status().get("recipe")
    other = next((r for r in robot.status().get("recipes") or [] if r != rec), rec)
    robot.set_recipe(other)
    settle()
    switched = robot.status().get("recipe") == other
    robot.set_recipe(rec)
    settle()
    check(f"set_recipe {other!r} and back to {rec!r}", switched and robot.status().get("recipe") == rec)
    ok, code = refused(lambda: robot.set_recipe("no_such_recipe"), "BAD_ARGS")
    check(f"set_recipe of an unknown name refused ({code})", ok and robot.status().get("recipe") == rec, code)

    print("\n6. the Hall sensor check (IDLE)")
    robot.hall_check(True)
    settle(0.5)
    on = robot.status().get("hall_check") or {}
    robot.hall_check(False)
    settle(0.5)
    off = robot.status().get("hall_check") or {}
    check("hall_check on: the Hall joint reported (1-based); off: 0, no pin", on.get("joint", 0) > 0
          and off.get("joint") == 0 and off.get("pin") is None, f"{on} / {off}")

    print("\n7. a joint wall (J4), then back to auto")
    w = robot.get_joint_walls()["walls"][3]
    lo, hi = w.get("damping_ratio_window") or (0.5, 1.0)
    target = round((lo + hi) / 2, 3)
    robot.set_joint_wall(4, damping_ratio=target)
    changed = abs(robot.get_joint_walls()["walls"][3]["damping_ratio"] - target) < 1e-6
    ok, code = refused(lambda: robot.set_joint_wall(4, frequency_hz=99.0), "WALL_UNSTABLE")
    robot.set_joint_wall(4, damping_ratio="auto")
    back = robot.get_joint_walls()["walls"][3]
    check(f"J4 wall damping {target} live, then auto again", changed and abs(back["damping_ratio"] - w["damping_ratio"])
          < 1e-6, f"{w['damping_ratio']} -> {back['damping_ratio']}")
    check(f"an unstable wall (99 Hz) refused ({code})", ok, code)

    print("\n8. payload")
    robot.set_payload(0.2, (0.0, 0.0, 0.05))
    settle()
    p = (robot.status().get("payload") or {}).get("mass")
    robot.set_payload(0.0)
    settle()
    p0 = (robot.status().get("payload") or {}).get("mass")
    check("set_payload 0.2 kg, then 0", p is not None and abs(p - 0.2) < 1e-6 and p0 is not None and abs(p0) < 1e-9,
          f"{p} -> {p0}")
    ok, code = refused(lambda: robot.set_payload(50.0), "PAYLOAD_LIMIT")
    check(f"set_payload 50 kg refused ({code})", ok, code)

    print("\n9. the jog settings back to system.xml, and the robot as it was")
    jd = xp.load_system_config(str(SYSTEM_XML), str(ROBOTS_DIR)).get("jog_defaults") or {}
    robot.set_jog_accel(float(jd.get("accel_time", 0.55)))
    robot.set_jog_profile(str(jd.get("profile", "scurve")))
    robot.set_jog_jerk_factor(float(jd.get("jerk_factor", 3.0)))
    robot.set_jog_control_mode(str(jd.get("control_mode", "pid")))
    settle(0.5)
    want = (float(jd.get("accel_time", 0.55)), 0.0 if jd.get("profile", "scurve") == "trapezoidal" else 1.0,
            float(jd.get("jerk_factor", 3.0)), 0.0 if jd.get("control_mode", "pid") == "pid" else 1.0)
    got = jog_state()
    check(f"the jog settings back to system.xml's {want}", all(g is not None and abs(g - w) < 1e-9
                                                             for g, w in zip(got, want)), got)
    s1 = robot.status()
    check("RTI ticking, no error, no CRITICAL line in RTI.log since the start", ticking() and s1.get("err") == 0
          and not criticals_since(log0), (s1.get("errors"), criticals_since(log0)[:2]))
    check("every switch as it was", all(bool(s1.get(k)) == bool(s0.get(k)) for k in
          ("gravity_comp", "inertia_comp", "coriolis_comp", "friction_comp", "friction_ff", "can_read_first")))


def main():
    args = parser("72: every runtime-setting command once, against the simulated robot").parse_args()
    if shm_string("sim_mode") != "ON":
        print("REFUSED: RTI is not running the simulated robot (/dev/shm/sim_mode is not ON) — this test changes PID "
              "gains, the payload and the walls. Run it through tools/sim_run.py.")
        return 2
    robot = connect(args)
    try:
        robot.take_control()
    except CommandRejected as e:
        print(f"REFUSED: control is held ({e.message})")
        return 2
    try:
        run(robot)
    finally:
        try:
            robot.release_control()
        except Exception:                                       # noqa: BLE001
            pass
        robot.disconnect()
    print(f"\n72: {'ALL PASS' if not fails else f'{len(fails)} FAILED: ' + '; '.join(fails)}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
