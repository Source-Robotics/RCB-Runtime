#!/usr/bin/env python3
"""51: haptic zone wall in freedrive — you FEEL a safety zone in IDLE gravity comp. You move the arm by hand.

Needs RTI AND command_executor running the haptic-wall code (RTI.log: "[SAFETY] haptic walls:") and the robot
homed. Like api_tests/50 it writes its own temporary cell (and, for tune, its own <haptic_walls> values) and
RESTORES config/system.xml and the original zones when it ends (also on Ctrl-C). If it is killed hard, put
system.xml back yourself and delete config/cells/api_test_51.xml.

The wall: an ALLOWED PLANE the TCP crosses when J5 turns 4 deg from where it is now, default zone settings
(haptic="1", freedrive="warn"). It is a Cartesian force, so it pushes through EVERY joint a little — keep a
hand on the arm. Along J5's direction of travel the arm weighs only ~0.13 kg at the TCP, which is what limits
how stiff the wall can be at 250 Hz (see utility/haptic_walls.py STABILITY).

Default run (--go). 250 Hz telemetry is recorded throughout (recipe haptic_wall) and every step is saved to
temp_data/haptic_wall/*.npz. The WALL TORQUE is what RTI sent on top of gravity: j_tc - gravity(q), the same
gravity function and model RTI uses.
  A. You rotate J5 slowly INTO the wall: it should resist, harder the further in. No warning, no error (past the
     breakthrough the warning is correct and only reported). Careful reading the vibration: a hand that holds
     still and then moves on (seen 2026-10-02) also shows up above 5 Hz.
     Reports the VIBRATION above 5 Hz (your hand's motion is below that): amplitude, frequency, and J5's motor
     electrical frequency at that speed. A roughly fixed frequency that grows with stiffness = the wall's own
     limit cycle (soften it); a frequency that follows the electrical one = stepper torque ripple (firmware).
  B. Hold it 5-20 mm in; when the script says LET GO, let go. PASS: the wall pushed the TCP back out with stiffness x depth
     while you held it. Whether it then MOVES back is reported, not checked: anything still touching the arm holds
     it, and a joint moves only once its share of the push beats its friction (api_tests/54: J5 ~0.08 Nm).
  C. Push THROUGH it (past the breakthrough depth): a Z_api_haptic:ZONE_WARNING, the robot stays in IDLE.
     Bring it back out: the warning clears.
  D0. Let go of the arm OUTSIDE the wall for 2.5 s: how far it drifts with no wall at all (gravity-comp error).
  D. Never pushes on its own: hold it 5-20 mm in again; the script restarts freedrive (a new session) while you
     hold it, then says LET GO. PASS: no wall torque on any joint after the restart (it was inside before the
     session began). How far the arm moves is reported next to D0's drift (it is not the wall if they match).

--record: kept for old command lines (recording is always on now).

--tune skips A-D: an interactive loop — type stiffness / damping / filter, the script applies them (only inside
the simulated-stable table; anything else is refused and shown the stable damping range), you feel the wall,
optionally record it, repeat. Enter keeps a value, q finishes (at any prompt). At the end it prints the
<haptic_walls> values to keep in config/system.xml. JOINT walls are tuned in the dashboard's Walls tab instead
(api_tests/52_joint_wall.py checks one joint's wall).

    python3 api_tests/51_haptic_wall.py                  # dry run: the plan and the geometry
    python3 api_tests/51_haptic_wall.py --go [--record]
    python3 api_tests/51_haptic_wall.py --go --tune
"""
import argparse
import os
import re
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robotclient import RobotClient                                   # noqa: E402
from robotclient.rti_client import parse_state_packet                 # noqa: E402

J = 4                                  # J5, 0-based
WALL_DEG = 4.0
WARN = "Z_api_haptic:ZONE_WARNING"
STOP = "Z_api_haptic:ZONE_VIOLATION"
CELL = "api_test_51"
RECIPE = "haptic_wall"
FS = 250.0
POLE_PAIRS = 50
HAND_HZ = 5.0                          # below this it is the hand moving the arm, not vibration
_ok, _bad = 0, []


class _Quit(Exception):
    pass


def ask(prompt, default):
    """A number from the keyboard: Enter = default, q = finish (raises _Quit). Anything else asks again."""
    while True:
        ans = input(prompt).strip().lower()
        if ans == "q":
            raise _Quit
        if not ans:
            return default
        try:
            return float(ans)
        except ValueError:
            print(f"   '{ans}' is not a number: type a number, Enter to keep {default:g}, or q to finish")


def check(name, cond, detail=""):
    global _ok
    if cond:
        _ok += 1
    else:
        _bad.append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    return cond


# ------------------------------------------------------------------------------------------- analysis
def to_arrays(packets):
    rows = []
    for raw in packets:
        try:
            r = parse_state_packet(raw.decode() if isinstance(raw, bytes) else raw)
        except Exception:
            continue
        if all(k in r for k in ("TCP_p", "TCP_s", "j_p", "j_s", "j_t", "j_tc", "tick")):
            rows.append(r)
    rows.sort(key=lambda r: r["tick"])
    out, seen = [], set()
    for r in rows:
        if r["tick"] not in seen:
            seen.add(r["tick"])
            out.append(r)
    keys = ("TCP_p", "TCP_s", "j_p", "j_s", "j_t", "j_tc")
    d = {k: np.array([r[k] for r in out], dtype=float) for k in keys}
    d["tick"] = np.array([r["tick"] for r in out], dtype=float)
    d["mode"] = np.array([str(r.get("mode", "")) for r in out])
    return d


def alias(f, fs=FS):
    return abs(((f + fs / 2) % fs) - fs / 2)


def segments(mask, min_len):
    """(start, end) of every run of True at least min_len long."""
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return [(a, b) for a, b in zip(edges[::2], edges[1::2]) if b - a >= min_len]


def highpass(x, lo_hz=HAND_HZ):
    """x above lo_hz: linear trend removed, then every FFT bin below lo_hz zeroed. Returns (x_hp, freqs, |spectrum|)."""
    t = np.arange(x.size, dtype=float)
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(x.size, 1.0 / FS)
    spec[f < lo_hz] = 0.0
    return np.fft.irfft(spec, n=x.size), f, np.abs(spec)


def resonance(f, mag, lo=None, hi=40.0):
    """(peak Hz, how many times it stands above the band 2 Hz below it) for the strongest 1 Hz band in lo..hi Hz,
    or None when the spectrum just falls off from below — the hand's own motion leaking past the 5 Hz cut, which
    is what 51 reported as "likely the wall" on 2026-10-02 (strongest at 5.1-5.3 Hz every run = the cut's edge,
    a smooth fall-off and no peak in the full spectrum)."""
    lo = HAND_HZ if lo is None else lo
    edges = np.arange(np.floor(lo), hi + 1.0)
    bands = np.array([mag[(f >= a) & (f < a + 1.0)].mean() if ((f >= a) & (f < a + 1.0)).any() else 0.0 for a in edges])
    i = int(np.argmax(bands))
    below = bands[i - 2] if i >= 2 else 0.0
    if i < 2 or below <= 0.0 or bands[i] < 1.5 * below:
        return None
    return float(edges[i] + 0.5), float(bands[i] / below)


def vibration(d, p_wall, n, gear):
    """The part of the depth into the wall ABOVE 5 Hz while in contact (your hand moves the arm below that):
    (rms mm, dominant Hz, J5 electrical Hz at that speed and its alias, J5 torque ripple rms Nm above 5 Hz,
    samples in contact, deepest mm). Contact = runs at least 0.5 s long deeper than 0.5 mm. None if none."""
    if d["tick"].size < 100:
        return None
    depth = -((d["TCP_p"][:, :3] - p_wall) @ n) * 1000.0
    segs = segments(depth > 0.5, int(0.5 * FS))
    if not segs:
        return None
    hp, trq, best = [], [], None
    for a, b in segs:
        x, f, mag = highpass(depth[a:b])
        hp.append(x)
        trq.append(highpass(d["j_t"][a:b, J])[0])
        if best is None or b - a > best[0]:
            best = (b - a, f, mag)
    seg, trq = np.concatenate(hp), np.concatenate(trq)
    f_dom = float(best[1][int(np.argmax(best[2]))])
    contact = np.zeros(depth.size, dtype=bool)
    for a, b in segs:
        contact[a:b] = True
    qd = float(np.median(np.abs(d["j_s"][contact, J])))
    f_el = POLE_PAIRS * qd * abs(gear) / (2 * np.pi)
    return (float(np.sqrt(np.mean(seg ** 2))), f_dom, f_el, alias(f_el), float(np.sqrt(np.mean(trq ** 2))),
            int(contact.sum()), float(depth[contact].max()), resonance(best[1], best[2]))


def wall_torque(d, model):
    """What RTI sent ON TOP of gravity, per tick and joint (Nm): j_tc - gravity(j_p) with RTI's own gravity
    function. In IDLE gravity comp that is exactly the haptic walls' torque (zone + joint walls)."""
    from robotics.dynamics import calculate_gravity_compensation
    grav = np.array([np.asarray(calculate_gravity_compensation(model, q), dtype=float) for q in d["j_p"]])
    return d["j_tc"] - grav.reshape(d["j_tc"].shape)


def wall_force(d, model, n):
    """The zone wall is a CARTESIAN force at the TCP; RTI sends it as joint torques tau = J_p^T F. Recover F from the
    recorded wall torque (least squares over the 6 joints) and return (F along n, the out-of-wall direction, in N;
    the torque F does NOT explain, Nm) per tick. Which joint carries how much depends on the whole arm pose — on
    2026-10-02 the TCP was pushed in with J5 at 86 deg instead of near the wall angle, J5's share had the other sign,
    and a J5-torque check failed a wall that pushed exactly right (7.7 N at 38 mm)."""
    wt = wall_torque(d, model)
    f_n, rest = np.zeros(len(wt)), np.zeros(len(wt))
    for i, q in enumerate(d["j_p"]):
        Jp = np.asarray(model.pinocchio_jacob0(q))[:3]
        F = np.linalg.lstsq(Jp.T, wt[i], rcond=None)[0]
        f_n[i], rest[i] = float(F @ n), float(np.linalg.norm(Jp.T @ F - wt[i]))
    return f_n, rest


def zone_scale(model, minv_fn, q, n, policy):
    """What RTI multiplies a zone wall by at pose q for a TCP contact pushed along n: m_eff / 0.1 kg (capped like
    utility/haptic_walls.ZONE_SCALE_MAX), or 1 with <zone_mass_scaled> off / no dynamics model."""
    from config.xml_parser import HAPTIC_REFERENCE_MASS
    from utility.haptic_walls import ZONE_SCALE_MAX
    if minv_fn is None or not policy.get("zone_mass_scaled", 1):
        return 1.0
    mi = np.zeros((6, 6))
    minv_fn(q, mi)
    a = np.asarray(model.pinocchio_jacob0(q))[:3].T @ n
    return min(1.0 / (float(a @ mi @ a) * HAPTIC_REFERENCE_MASS), ZONE_SCALE_MAX)


def save_recording(d, label, **meta):
    """temp_data/haptic_wall/<time>_<label>.npz — every array, plus the metadata as JSON."""
    import json
    from project_paths import TEMP_DATA_DIR
    if d is None or d["tick"].size == 0:
        return None
    out = TEMP_DATA_DIR / "haptic_wall"
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d_%H%M%S')}_{re.sub(r'[^A-Za-z0-9]+', '_', label).strip('_')}.npz"
    np.savez(path, meta=json.dumps(meta, default=str), **d)
    return path


def report(v, label):
    if v is None:
        print(f"   [{label}] no contact recorded (deeper than 0.5 mm for at least 0.5 s)")
        return
    rms, f_dom, f_el, f_al, trq, nc, deepest, peak = v
    print(f"   [{label}] {nc / FS:.1f} s in contact, deepest {deepest:.1f} mm: vibration above {HAND_HZ:g} Hz "
          f"{rms:.2f} mm RMS, strongest at {f_dom:.1f} Hz; J5 torque ripple {trq:.3f} Nm RMS; J5 electrical "
          f"{f_el:.1f} Hz (seen at {f_al:.1f} Hz at 250 Hz)")
    if rms < 0.05:
        print("      -> smooth (under 0.05 mm)")
    elif peak is None:
        print("      -> no resonance: it falls off smoothly above 5 Hz — your hand's own motion, not the wall")
    elif abs(peak[0] - f_al) < 2.0:
        print(f"      -> a peak at {peak[0]:.1f} Hz on the motor's electrical frequency: stepper torque ripple, not the wall")
    else:
        print(f"      -> a real peak at {peak[0]:.1f} Hz ({peak[1]:.1f}x above 2 Hz lower), not the motor's frequency: "
              f"likely the wall ringing — try less stiffness, more damping or more filter")


# ------------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--record", action="store_true", help="(kept for old command lines: recording is always on)")
    ap.add_argument("--tune", action="store_true", help="interactive stiffness/damping/filter tuning loop")
    ap.add_argument("--wait", type=float, default=90.0, help="seconds you get for each hand step")
    args = ap.parse_args()

    import config.xml_parser as xp
    from project_paths import CELLS_DIR, GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from utility.pinocchio_attach import attach_pinocchio
    from utility.robot_factory import create_robot
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    model = create_robot(sysc["active_robot"], gripper_name=cfg.get("gripper_name", ""))
    attach_pinocchio(model)
    from utility.haptic_walls import make_inverse_mass
    minv_fn = make_inverse_mass(model)
    gear = float(model.links[J].G or 1.0)
    lo = float(np.degrees(cfg["software_joint_limit_negative"][J]))
    hi = float(np.degrees(cfg["software_joint_limit_positive"][J]))
    policy = xp.load_zone_setup(SYSTEM_XML, CELLS_DIR, None)["haptic"]

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    base = __import__("46_monitor_baseline")
    original_xml = SYSTEM_XML.read_text()
    cell_path = CELLS_DIR / f"{CELL}.xml"
    claimed = switched = False
    capture = prev_recipe = None
    fric_was = False
    try:
        q0 = np.asarray(robot.get_joints(), dtype=float)
        sign = 1.0 if (hi - q0[J]) >= (q0[J] - lo) else -1.0

        def tcp(dj5):
            q = np.radians(q0.copy())
            q[J] += np.radians(sign * dj5)
            return np.asarray(model.pinocchio_fkine(q))[:3, 3]

        p_now, p_wall = tcp(0.0), tcp(WALL_DEG)
        n = p_now - p_wall
        n /= np.linalg.norm(n)
        live = np.asarray(robot.get_pose()[:3], dtype=float)
        clear_now = float(np.dot(p_now - p_wall, n)) * 1000

        def depth_mm():                          # how far PAST the wall the TCP is now (> 0 = in it)
            return -float(np.dot(np.asarray(robot.get_pose()[:3], dtype=float) - p_wall, n)) * 1000

        print(f"J5 now {q0[J]:.1f} deg; the wall is at J5 {sign * WALL_DEG:+.0f} deg, the TCP {clear_now:.1f} mm before "
              f"it. Wall: {policy['stiffness']:.0f} N/m, {policy['damping']:.0f} N.s/m, {1000 * policy['velocity_tau']:.0f} ms "
              f"filter, max {policy['max_force']:.0f} N, breakthrough {1000 * policy['breakthrough']:.0f} mm"
              + (f"; x m_eff/0.1 kg = x{zone_scale(model, minv_fn, np.radians(q0), n, policy):.2f} here along the push "
                 f"(zone_mass_scaled)." if minv_fn is not None and policy.get("zone_mass_scaled", 1) else "."))
        if np.linalg.norm(p_now - live) > 0.002 or clear_now < 6.0:
            print("Geometry not usable (model and robot disagree, or J5 barely moves the TCP here). Nothing changed.")
            return 2
        if not args.go:
            print("DRY RUN: re-run with --go (or --go --tune). Nothing moved, no file changed.")
            return 0
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True:
            print(f"Robot must be ENABLED and homed (got state={st.get('state')} homed={st.get('homed')}).")
            return 2

        cell_path.write_text(
            '<?xml version="1.0" encoding="utf-8"?>\n<!-- written by api_tests/51_haptic_wall.py; deleted when it ends -->\n'
            f'<cell>\n  <zone name="api_haptic" shape="plane" kind="allowed">\n'
            f'    <point>{p_wall[0]:.6f} {p_wall[1]:.6f} {p_wall[2]:.6f}</point>\n'
            f'    <normal>{n[0]:.6f} {n[1]:.6f} {n[2]:.6f}</normal>\n  </zone>\n</cell>\n')

        def apply(k=None, d=None, tau=None):
            """Write the test cell (and any wall values) into system.xml and reload. Returns the APPLIED policy."""
            text = re.sub(r"<active_cell>.*?</active_cell>", f"<active_cell>{CELL}</active_cell>", original_xml)
            for tag, val in (("stiffness", k), ("damping", d), ("velocity_tau", tau)):
                if val is not None:
                    text = re.sub(rf"(<haptic_walls>.*?<{tag}>)[^<]*(</{tag}>)", rf"\g<1>{val:g}\g<2>", text, flags=re.S)
            SYSTEM_XML.write_text(text)
            return robot.reload_zones()["haptic_walls"]

        switched = True
        base.claim_control(robot)
        claimed = True
        fric_was = bool(robot.status().get("friction_comp"))
        if fric_was:
            robot.set_friction_comp(False)
            print("Friction compensation switched OFF for this test (the wall is judged as torque sent - gravity); "
                  "it is switched back on at the end.")
        summary = apply()
        check("reload_zones applied the test cell", bool(summary))
        capture = base.Capture()
        prev_recipe = robot.status().get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        if st.get("mode") == "EXEC":
            robot.enter_idle_mode(wait=True)
        robot.set_gravity_comp(True)
        robot.release_control()
        claimed = False
        time.sleep(0.5)

        def watch(seconds, until=None, note=""):
            end, last, deepest, s = time.monotonic() + seconds, 0.0, -1e9, robot.status()
            while time.monotonic() < end:
                d = depth_mm()
                deepest = max(deepest, d)
                s = robot.status()
                if until is not None and until(d, s):
                    break
                if time.monotonic() - last > 0.3:
                    print(f"   depth {d:+7.1f} mm   {note}", end="\r")
                    last = time.monotonic()
                time.sleep(0.03)
            print()
            return deepest, s

        def hold(lo_mm=5.0, hi_mm=20.0, seconds=1.0):
            """Wait until the TCP is held lo..hi mm in for `seconds`. Returns the depth, or None on timeout."""
            end, since, last = time.monotonic() + args.wait, None, 0.0
            while time.monotonic() < end:
                d = depth_mm()
                since = (since or time.monotonic()) if lo_mm <= d <= hi_mm else None
                if since and time.monotonic() - since >= seconds:
                    print()
                    return d
                if time.monotonic() - last > 0.3:
                    print(f"   depth {d:+7.1f} mm   (hold it between {lo_mm:.0f} and {hi_mm:.0f} mm)", end="\r")
                    last = time.monotonic()
                time.sleep(0.03)
            print()
            return None

        meta = {"p_wall": p_wall.tolist(), "n": n.tolist(), "sign": sign, "q0_deg": q0.tolist()}

        def recorded(label, fn, vib=True):
            """Run fn() while recording; report the vibration, save the recording. Returns (fn's result, data)."""
            start = len(capture.packets)
            result = fn()
            d = to_arrays(capture.packets[start:])
            if vib:
                report(vibration(d, p_wall, n, gear), label)
            path = save_recording(d, label, policy=cur_policy, **meta)
            if path is not None:
                print(f"   (saved {path.relative_to(_ROOT)})")
            return result, d

        cur_policy = dict(summary)

        if args.tune:
            cur = dict(summary)
            print("\nTUNING. Freedrive is on; the wall is live. Enter = keep a value, q = finish (at any prompt).")
            try:
                while True:
                    print(f"\n   now: stiffness {cur['stiffness']:g} N/m, damping {cur['damping']:g} N.s/m, filter "
                          f"{1000 * cur['velocity_tau']:g} ms")
                    k = ask("   stiffness N/m (100-750): ", cur["stiffness"])
                    tau = ask("   filter ms (0, 4 or 8): ", 1000.0 * cur["velocity_tau"]) / 1000.0
                    win = xp.haptic_damping_window(k, tau)
                    print("   stable damping at that stiffness/filter: " +
                          ("NONE — pick less stiffness or less filtering" if win is None else f"{win[0]:g}-{win[1]:g} N.s/m"))
                    d = ask("   damping N.s/m: ", cur["damping"])
                    base.claim_control(robot)
                    applied = apply(k, d, tau)
                    robot.release_control()
                    if (applied["stiffness"], applied["damping"], applied["velocity_tau"]) != (k, d, tau):
                        print(f"   REFUSED (outside the simulated-stable table) — still "
                              f"{applied['stiffness']:g} / {applied['damping']:g} / {1000 * applied['velocity_tau']:g} ms")
                    cur = cur_policy = applied
                    ans = input("   Feel it. Enter = record 10 s while you press into it, s = skip, q = finish: ")
                    if ans.strip().lower() == "q":
                        raise _Quit
                    if ans.strip().lower() != "s":
                        recorded(f"tune k{cur['stiffness']:g} d{cur['damping']:g} f{1000 * cur['velocity_tau']:g}",
                                 lambda: watch(10.0, note="(press into the wall, slowly in and out)"))
            except _Quit:
                pass
            print("\nKeep these in config/system.xml <haptic_walls>:")
            print(f"    <stiffness>{cur['stiffness']:g}</stiffness>\n    <damping>{cur['damping']:g}</damping>\n"
                  f"    <velocity_tau>{cur['velocity_tau']:g}</velocity_tau>")
            return 0

        print("\nA. Rotate J5 slowly {}wards INTO the wall, up to ~15 mm deep (you should feel it build).".format(
            "+" if sign > 0 else "-"))
        input("   Ready? Enter to start watching (20 s)...")
        (deepest, s), rec_a = recorded("A, pressing", lambda: watch(20.0, note="(feel it?)"))
        check("you reached the wall", deepest > 2.0, f"deepest {deepest:.1f} mm")
        bt = 1000 * summary["breakthrough"]
        if deepest > bt:      # 2026-10-02 on the arm: 33.9 mm -> the warning was RIGHT, and was reported as a failure
            print(f"   you went {deepest:.1f} mm in — past the {bt:.0f} mm breakthrough, so {WARN} is the correct "
                  f"reaction (not checked here; C checks it). Stay under ~15 mm in A to check 'no warning'.")
            check("...robot still ENABLED in IDLE (warn never stops)", s.get("state") == "ENABLED"
                  and s.get("mode") == "IDLE" and STOP not in (s.get("errors") or []), f"errors={s.get('errors')}")
        else:
            check("no warning and no error while pressing it",
                  WARN not in (s.get("errors") or []) and STOP not in (s.get("errors") or [])
                  and s.get("state") == "ENABLED" and s.get("mode") == "IDLE", "" if WARN not in (s.get("errors") or [])
                  else f"errors={s.get('errors')}")

        # The model's gravity vs what RTI sent, OUTSIDE the wall during A: how exact "wall torque" can be.
        wt_a = wall_torque(rec_a, model) if rec_a["tick"].size else np.zeros((0, 6))
        out_a = (-((rec_a["TCP_p"][:, :3] - p_wall) @ n) * 1000.0 < -2.0) if rec_a["tick"].size else np.zeros(0, bool)
        resid = float(np.max(np.abs(wt_a[out_a]))) if out_a.any() else 0.0
        quiet = max(0.01, 3.0 * resid)
        print(f"   wall torque outside the wall during A (should be ~0): max {resid:.4f} Nm -> 'no push' below {quiet:.3f} Nm")

        print("\nB. Hold it 5-20 mm into the wall. When the script says LET GO, let go.")

        def hold_and_release():
            got = hold()
            if got is None:
                return None, None
            print("   >>> LET GO NOW <<<")
            time.sleep(2.5)
            return got, depth_mm()

        (d0, d1), rec_b = recorded("B, held then released", hold_and_release, vib=False)
        if check("held in the wall", d0 is not None, "" if d0 is not None else "not held 5-20 mm in time"):
            tk = rec_b["tick"]
            t_go = tk[-1] - 2.5 * FS if tk.size else 0.0
            held = (tk >= t_go - FS) & (tk < t_go)                    # the last second you held it
            f_n, rest = wall_force(rec_b, model, n)
            depth_b = -((rec_b["TCP_p"][:, :3] - p_wall) @ n) * 1000.0
            push = float(np.median(f_n[held])) if held.any() else 0.0
            sc_b = np.array([zone_scale(model, minv_fn, q, n, policy) for q in rec_b["j_p"][held]])
            want = float(np.median(np.minimum(summary["stiffness"] * sc_b * np.maximum(depth_b[held], 0.0) / 1000.0,
                                              summary["max_force"]))) if held.any() else 0.0
            unexplained = float(np.max(rest[held])) if held.any() else float("nan")
            check("while held, the wall pushed the TCP back OUT with stiffness x m_eff/0.1 kg x depth (the Cartesian "
                  "force, recovered from the joint torques)", push > 0.2 and abs(push - want) <= 0.15 * want + 0.2 and unexplained < 0.02,
                  f"{push:+.2f} N out of the wall vs {want:.2f} N expected; torque not explained by a TCP force "
                  f"{unexplained:.3f} Nm")
            # Reported, not checked (as in 52). 2026-10-02 on the arm: 0.53 Nm on J5 at 19 mm, motor torque ==
            # command, J5 did not move for 2.5 s. First read as J5's friction — api_tests/54 then measured J5
            # breaking loose at 0.06-0.09 Nm, so something else held it (most likely the hand, still on the arm).
            back = d1 < 0.5 * d0
            print(f"   released at {d0:.1f} mm -> {d1:.1f} mm after 2.5 s: " + (
                "it moved back towards the surface" if back else
                f"it stayed (the wall pushed {push:+.2f} N at the TCP) — was anything still touching the arm? A joint "
                f"also stays while its share of the push is below its friction (api_tests/54)"))

        if WARN in (robot.status().get("errors") or []):
            print(f"\n   {WARN} is still on from before — bring the TCP back OUT of the wall first.")
            watch(args.wait, until=lambda d, s: d < -1.0 and WARN not in (s.get("errors") or []), note="(back out)")
        print(f"\nC. Push THROUGH it, past {bt:.0f} mm.")
        (deepest, s), _ = recorded("C, pushing through", lambda: watch(
            args.wait, until=lambda d, s: WARN in (s.get("errors") or []), note=f"(through at {bt:.0f} mm)"))
        check(f"pushed through: {WARN}", WARN in (s.get("errors") or []), f"deepest {deepest:.1f} mm")
        time.sleep(0.3)
        s = robot.status()
        check("...the robot stayed ENABLED in IDLE (warn is the freedrive default)",
              s.get("state") == "ENABLED" and s.get("mode") == "IDLE" and STOP not in (s.get("errors") or []),
              f"state={s.get('state')} mode={s.get('mode')}")
        print("   Now bring it back OUT of the wall.")
        _, s = watch(args.wait, until=lambda d, s: d < -1.0 and WARN not in (s.get("errors") or []), note="(back out)")
        check("back out: the warning cleared", WARN not in (s.get("errors") or []))

        print("\nD0. Hold the arm still OUTSIDE the wall (5+ mm before it). When the script says LET GO, let go.")
        drift0 = None
        if hold(lo_mm=-60.0, hi_mm=-5.0) is not None:
            p_a = np.asarray(robot.get_pose()[:3], dtype=float)
            print("   >>> LET GO NOW <<<")
            time.sleep(2.5)
            drift0 = float(np.linalg.norm(np.asarray(robot.get_pose()[:3], dtype=float) - p_a)) * 1000
            print(f"   with no wall at all the TCP drifted {drift0:.1f} mm in 2.5 s (gravity-comp error + friction)")

        print("\nD. Hold it 5-20 mm into the wall again. The script restarts freedrive while you hold it, then says LET GO.")
        d0 = hold()
        if check("held in the wall", d0 is not None, "" if d0 is not None else "not held 5-20 mm in time"):
            base.claim_control(robot)
            claimed = True
            robot.set_gravity_comp(False)          # velocity hold
            time.sleep(0.3)
            robot.set_gravity_comp(True)           # a NEW freedrive session, started with the TCP inside
            robot.release_control()
            claimed = False
            time.sleep(0.1)

            def release_d():
                p_a = np.asarray(robot.get_pose()[:3], dtype=float)
                d_a = depth_mm()
                print("   >>> LET GO NOW <<<")
                time.sleep(2.5)
                return d_a, depth_mm(), float(np.linalg.norm(np.asarray(robot.get_pose()[:3], dtype=float) - p_a)) * 1000

            (d0, d1, moved), rec_d = recorded("D, new session inside", release_d, vib=False)
            idle = rec_d["mode"] == "IDLE"
            wt = wall_torque(rec_d, model)[idle] if idle.any() else np.zeros((0, 6))
            peak = float(np.max(np.abs(wt))) if wt.size else float("nan")
            check("a session started inside the wall does NOT push on its own: no wall torque on any joint",
                  d0 > 2.0 and wt.size > 0 and peak < quiet,
                  f"peak {peak:.4f} Nm (threshold {quiet:.3f}), {d0:.1f} -> {d1:.1f} mm deep")
            print(f"   the arm moved {moved:.1f} mm in 2.5 s" + ("" if drift0 is None else
                  f" (no-wall drift in D0: {drift0:.1f} mm — similar = gravity comp, not the wall)"))
            print("   (Move it out and back in: the wall is armed again once it has been outside.)")
    finally:
        if switched:
            SYSTEM_XML.write_text(original_xml)
            try:
                if not claimed:
                    base.claim_control(robot)
                    claimed = True
                if prev_recipe:
                    robot.set_recipe(prev_recipe)
                if fric_was:
                    robot.set_friction_comp(True)
                after = robot.reload_zones()
                print(f"\nRestored config/system.xml; zones back to cell '{after['cell']}' ({len(after['zones'])} zone(s)).")
            except Exception as e:
                print(f"\n!! system.xml restored, but reload_zones failed ({e}) — run robot.reload_zones() or restart.")
        if capture is not None:
            capture.stop()
        if cell_path.exists():
            cell_path.unlink()
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
