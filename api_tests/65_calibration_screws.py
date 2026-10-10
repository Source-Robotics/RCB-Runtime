#!/usr/bin/env python3
"""65: each joint at its calibration screw — how far its reading is from the screw's angle (2026-10-09). Moves NOTHING.

Every PAR6 joint has a calibration screw that pins it at a known angle (robot XML <calibration_screw_rad>: J1 90, J2 -90,
J3 180, J4 0, J5 0, J6 180 deg). A homed joint pinned there should read exactly that angle; what it reads instead is the
error of its home_offset on THIS arm (the offset that removes it: home_offset - error).

Live (default): once a second the mode, gravity compensation on/off, and per joint its reading minus its screw angle
(only the pinned joints mean anything). Change the motor state while pinned — Disable, gravity compensation off — and
watch how far the readings move: the encoder sits on the MOTOR, so gear play and wind-up between the motor and the
pinned joint show up here (2026-10-09: J3 read 0.9 deg away from that morning's reading at the same screw).

Measured on the arm 2026-10-09 (J1-J3 pinned): gravity compensation on / off moves the readings by <= 0.04 deg — any
IDLE state will do.

--arm NAME: record. Enter -> the joints you name (e.g. 123) are averaged over 1 s (they must be still, the robot homed)
and ADDED to temp_data/calibration/NAME.json with the mode, the gravity state and the homing they belong to (the last
"Sequence started" in logs/RTI.log). Record after SEVERAL homings (screws out, home, screws in, record): the readings of
one joint should agree within the homing scatter (~0.05 deg); their mean is the arm's value. J1-J3 now and J4-J6 later
is fine. Use one NAME per physical arm.
--compare: every saved arm side by side — per joint the home_offset that would make each arm read its screw angle (the
mean over its homings), their spread (> ~0.2 deg: one shared value cannot fit every arm) and their mean (the shared
value).

    .venv/bin/python api_tests/65_calibration_screws.py [--arm NAME | --compare]

Reads shared memory as FILES (/dev/shm/<name>), never a live segment.
"""
import glob
import json
import math
import os
import struct
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

STILL_DEG = 0.02        # a recorded joint may move at most this much (peak to peak) over the 1 s average
SPREAD_OK_DEG = 0.2     # arms whose offsets agree within this can share one value


def f64(name, n=1):
    with open(f"/dev/shm/{name}", "rb") as fh:
        return np.frombuffer(fh.read(8 * n), dtype=np.float64).copy()


def text(name):
    with open(f"/dev/shm/{name}", "rb") as fh:
        b = fh.read(260)
    k = struct.unpack("<I", b[:4])[0]
    return b[4:4 + k].decode(errors="replace")


def wrap_deg(x):
    """An angle difference brought into [-180, 180)."""
    return (np.asarray(x, float) + 180.0) % 360.0 - 180.0


def screw_errors_deg(q_rad, screw_rad):
    """Reading minus screw angle per joint, degrees (NaN where a joint has no screw)."""
    return wrap_deg(np.degrees(np.asarray(q_rad, float)) - np.degrees(np.asarray(screw_rad, float)))


def corrected_offset(home_offset, error_deg):
    """The home_offset that makes the joint read its screw angle at the same pose: a joint reads offset + (pos - master) k
    for motor dir 0 and offset - (pos - master) k for dir 1 (SourceRoboticsToolbox.Joint + homing._update_home_reference),
    so in both directions the reading moves 1:1 with the offset."""
    return float(home_offset) - math.radians(float(error_deg))


def merge_record(data, joints, q_rad, screw_rad, home_offset, mode, gravity_on, utc, homed_at=None):
    """ADD the named joints' readings to an arm's record (one sample per joint per call); returns the record."""
    err = screw_errors_deg(q_rad, screw_rad)
    for j in joints:
        data.setdefault("joints", {}).setdefault(f"J{j + 1}", []).append({
            "reading_deg": round(float(np.degrees(q_rad[j])), 4), "screw_deg": round(float(np.degrees(screw_rad[j])), 4),
            "error_deg": round(float(err[j]), 4), "home_offset_used": float(home_offset[j]),
            "corrected_offset": round(corrected_offset(home_offset[j], err[j]), 6),
            "mode": mode, "gravity_comp": bool(gravity_on), "utc": utc, "homed_at": homed_at})
    return data


def joint_summary(samples):
    """One joint's samples on one arm -> {"n", "homings", "corrected_offset" (mean), "spread_deg" (max - min)}."""
    v = np.array([s["corrected_offset"] for s in samples])
    return {"n": len(samples), "homings": len({s.get("homed_at") for s in samples}),
            "corrected_offset": float(v.mean()), "spread_deg": float(np.degrees(v.max() - v.min()))}


def compare_arms(arms):
    """{arm: record} -> {joint: {"per_arm": {arm: mean corrected offset}, "mean": .., "spread_deg": ..}} — corrected
    offsets, so arms measured while different offsets were in use still compare."""
    out = {}
    for arm, rec in sorted(arms.items()):
        for jn, samples in rec.get("joints", {}).items():
            sm = joint_summary(samples)
            d = out.setdefault(jn, {"per_arm": {}, "own": {}})
            d["per_arm"][arm] = sm["corrected_offset"]
            d["own"][arm] = (sm["homings"], sm["spread_deg"])
    for jn, d in out.items():
        v = np.array(list(d["per_arm"].values()))
        d["mean"] = float(v.mean())
        d["spread_deg"] = float(np.degrees(v.max() - v.min()))
    return dict(sorted(out.items()))


def last_homing_utc(log_path):
    """The time of the last "[Homing] Sequence started" in RTI.log (UTC text), or None — tells the samples' homings apart."""
    try:
        with open(log_path, "rb") as fh:
            fh.seek(max(0, os.path.getsize(log_path) - 4_000_000))
            tail = fh.read().decode(errors="replace").splitlines()
    except OSError:
        return None
    for line in reversed(tail):
        if "[Homing] Sequence started" in line:
            try:
                return time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(float(line.split("|")[1])))
            except (IndexError, ValueError):
                return None
    return None


def calib_dir():
    from project_paths import TEMP_DATA_DIR
    return os.path.join(str(TEMP_DATA_DIR), "calibration")


def main():
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    import utility.display_time as dtz
    dtz.set_zone(sysc.get("display_timezone"))
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=sysc["active_gripper"])
    n = int(cfg["joint_num"])
    screw = np.asarray(cfg["calibration_screw_rad"], float)[:n]
    offs = np.asarray(cfg["home_offset"], float)[:n]
    where = ["the gripper XML (arm_joint_home_offsets)" if int(f) else "the robot XML" for f in cfg["home_offset_gripper_dependent"][:n]]

    if "--compare" in sys.argv:
        arms = {}
        for p in sorted(glob.glob(os.path.join(calib_dir(), "*.json"))):
            with open(p) as fh:
                arms[os.path.splitext(os.path.basename(p))[0]] = json.load(fh)
        if not arms:
            print(f"nothing saved yet in {calib_dir()} — record with --arm NAME first")
            return 1
        print(f"{len(arms)} arm(s): {', '.join(arms)}\n")
        for jn, d in compare_arms(arms).items():
            j = int(jn[1:]) - 1
            per = "  ".join(f"{a} {v:+.6f} ({d['own'][a][0]} homing(s), spread {d['own'][a][1]:.3f} deg)"
                            for a, v in d["per_arm"].items())
            verdict = ("one shared value fits" if d["spread_deg"] <= SPREAD_OK_DEG else "TOO DIFFERENT for one shared value")
            print(f"  {jn}: home_offset per arm: {per}\n        between arms: spread {d['spread_deg']:.3f} deg — {verdict}; "
                  f"shared value {d['mean']:+.6f} (now {offs[j]:+.6f}, in {where[j]})")
        return 0

    arm = sys.argv[sys.argv.index("--arm") + 1] if "--arm" in sys.argv else None
    labels = "  ".join(f"J{j + 1}@{math.degrees(screw[j]):g}" for j in range(n) if not math.isnan(screw[j]))
    print(f"Screw angles (deg): {labels}. Readings below are reading - screw angle, deg; only pinned joints mean anything.")
    if arm is None:
        print("Live, once a second. Ctrl-C to stop.\n")
        try:
            while True:
                q = f64("joint_position", n)
                err = screw_errors_deg(q, screw)
                grav = "on " if f64("gravity_comp_enabled")[0] >= 0.5 else "off"
                homed = text("homed_state")
                print(f"{dtz.fmt(time.time(), '%H:%M:%S')}  {text('robot_mode'):12s} gravity comp {grav}  "
                      + ("" if homed == "HOMED" else f"[{homed}]  ")
                      + "  ".join(f"J{j + 1} {err[j]:+7.3f}" for j in range(n)), flush=True)
                time.sleep(1.0)
        except KeyboardInterrupt:
            return 0

    path = os.path.join(calib_dir(), f"{arm}.json")
    os.makedirs(calib_dir(), exist_ok=True)
    data = json.load(open(path)) if os.path.exists(path) else {"arm": arm}
    print(f"Recording arm '{arm}' -> {os.path.relpath(path, _ROOT)}")
    last = ""
    while True:
        try:
            ans = input(f"Pinned joints now (e.g. 123{', Enter = ' + last if last else ''}; q = quit): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if ans == "q":
            break
        ans = ans or last
        joints = sorted({int(c) - 1 for c in ans if c.isdigit() and 1 <= int(c) <= n})
        joints = [j for j in joints if not math.isnan(screw[j])]
        if not joints:
            print("  name at least one joint that has a screw")
            continue
        last = ans
        if text("homed_state") != "HOMED":
            print(f"  not recorded: the robot is {text('homed_state')} — home it first")
            continue
        qs = []
        for _ in range(20):
            qs.append(f64("joint_position", n))
            time.sleep(0.05)
        qs = np.array(qs)
        p2p = np.degrees(qs.max(0) - qs.min(0))
        moving = [j for j in joints if p2p[j] > STILL_DEG]
        if moving:
            print("  not recorded: " + ", ".join(f"J{j + 1} moved {p2p[j]:.3f} deg" for j in moving) + " — is the screw in?")
            continue
        mode, grav = text("robot_mode"), f64("gravity_comp_enabled")[0] >= 0.5
        homed_at = last_homing_utc(os.path.join(_ROOT, "logs", "RTI.log"))
        merge_record(data, joints, qs.mean(0), screw, offs, mode, grav, time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()), homed_at)
        with open(path, "w") as fh:
            json.dump(data, fh, indent=1)
        for j in joints:
            r = data["joints"][f"J{j + 1}"][-1]
            sm = joint_summary(data["joints"][f"J{j + 1}"])
            print(f"  J{j + 1}: reads {r['reading_deg']:.3f}, screw {r['screw_deg']:g} -> error {r['error_deg']:+.3f} deg; "
                  f"home_offset {r['home_offset_used']:+.6f} -> {r['corrected_offset']:+.6f} ({where[j]})\n"
                  f"        so far {sm['n']} reading(s) from {sm['homings']} homing(s), spread {sm['spread_deg']:.3f} deg, "
                  f"mean {sm['corrected_offset']:+.6f}  [{mode}, gravity comp {'on' if grav else 'off'}, homed {dtz.fmt_utc_text(homed_at) or '?'}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
