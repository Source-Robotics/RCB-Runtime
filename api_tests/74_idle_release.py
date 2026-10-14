#!/usr/bin/env python3
"""74: how far the arm moves when IDLE lets it go after a braked stop — against how long it was held first (2026-10-14).

Found by 73 on the arm: enter_idle_mode() during a move brakes along the path, holds until the MOTOR reads still, then
IDLE (freedrive) — and J1 moved 1.15 deg once let go (72 ms after the stop, ~0 motor current, up to 17 deg/s). In the
held stop the drive's torque kept swinging for ~1 s (4-5 Hz) while the motor encoder moved only +-0.03 deg: the arm is
not settled when the motor reads still. User: measure before changing anything.

Each cycle: one EXEC move, J1 and J5 together (as 73), a Stop half way (brakes along the path at the ceilings, then
holds in EXEC), held for --holds seconds, then enter_idle_mode() — the release; the arm is watched 1 s in IDLE, then put
back (EXEC, 20 %). Per release, from the robot's per-tick telemetry (the `tracking` recipe): how long it was held, J1's
drive torque and its swing over the last 100 ms, where J1 sat relative to the hold point, and J1's (and J5's) drift in
the 1 s after it — back (against the travel) and on —, and J5's held torque against the gravity model (on 2026-10-14
J5 held with 0.353 Nm where the model gives 0.231, then sagged 2.78 deg in IDLE: a model error, not the stop). Nothing
is judged but the run itself: this is a measurement.

Before: the arm ENABLED, homed, error-free; J1 free to turn --deg (50) and J5 a fifth of that (refused otherwise).
Simulator first (ideal drives: no drift there):
    .venv/bin/python tools/sim_run.py --tests "01_mode --go,74_idle_release --go --holds 0.1,0.6 --repeat 1"
On the arm, someone at the E-stop (~1 min):
    cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/74_idle_release.py --go
Options: --holds 0.1,0.3,0.6,1.0,2.0 (s), --repeat 2, --speed 0.6, --deg 50, --selftest (the analysis on synthetic
traces; no robot), --reanalyse temp_data/idle_release/<run> (every cycle's raw telemetry is saved there).
Exit 0 = ran, 1 = a cycle could not be analysed / an error latched / the selftest failed, 2 = refused / not run.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
J1, J5 = 0, 4
DT = 0.004
J5_RATIO = 0.2                  # J5's travel / J1's (as 73)
WATCH_S = 1.0                   # how long the arm is watched in IDLE
SWING_S = 0.1                   # the torque swing is measured over this long before the release
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW_DIR = os.path.join(ROOT, "temp_data", "idle_release")

fails = []


def check(name, ok, detail=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + ("" if ok or not detail else f"   [{detail}]"), flush=True)
    if not ok:
        fails.append(name)


# ------------------------------------------------------------------------------------------------ the analysis
def analyse(d, sgn):
    """d: to_arrays() of one cycle (rad, rad/s, Nm); sgn: +1 / -1, J1's direction of travel. Returns what happened at
    the release (the first IDLE tick after the motion came to rest), or {"ok": False, "why": ...}."""
    pc, p, sc, s, trq, mode = d["j_pc"], d["j_p"], d["j_sc"], d["j_s"], d["j_t"], d["mode"]
    n = len(pc)
    # The move, its rest and the release among EXEC ticks only: in IDLE the commanded values mirror the measured ones.
    moving = np.nonzero((mode == "EXEC") & (np.abs(sc[:, J1]) > 1e-3))[0]
    if not len(moving):
        return {"ok": False, "why": "J1 never moved"}
    idle = np.nonzero(mode[int(moving[0]):] == "IDLE")[0]
    if not len(idle):
        return {"ok": False, "why": "no IDLE after the stop"}
    i = int(moving[0]) + int(idle[0])
    rest = int(moving[moving < i][-1]) + 1
    if rest >= i:
        return {"ok": False, "why": "IDLE before the commanded motion came to rest"}
    hold = pc[i - 1]
    end = min(n - 1, i + int(round(WATCH_S / DT)))
    sw = slice(max(rest, i - int(round(SWING_S / DT))), i)
    sgn5 = 1.0 if pc[i - 1, J5] >= pc[int(moving[0]), J5] else -1.0
    rel = (p[i:end + 1, J1] - p[i, J1]) * sgn                # + = on in J1's direction of travel
    rel5 = (p[i:end + 1, J5] - p[i, J5]) * sgn5              # ... J5's own
    return {"ok": True, "held_s": (i - rest) * DT, "watched_s": (end - i) * DT,
            "torque_nm": float(trq[i - 1, J1]), "swing_nm": float(np.ptp(trq[sw, J1])),
            "past_hold_deg": math.degrees(sgn * float(p[i - 1, J1] - hold[J1])),
            "speed_dps": math.degrees(float(s[i - 1, J1])),
            "j1_back_deg": math.degrees(max(0.0, float(np.max(-rel)))),
            "j1_on_deg": math.degrees(max(0.0, float(np.max(rel)))),
            "j1_end_deg": math.degrees(float(rel[-1])),
            "j5_end_deg": math.degrees(float(rel5[-1])),
            # J5 carries gravity: the torque its drive held with vs the gravity model (what IDLE then applies) — a
            # difference is a model error the arm falls through once let go, however long it was held.
            "j5_held_nm": float(trq[i - 1, J5]), "j5_model_nm": float(d["j_tc"][i - 1, J5]) if "j_tc" in d else 0.0}


def report(rows):
    """rows: [(hold asked, result)] -> a table, one line per release, then the mean drift per hold time."""
    print("\n  hold    held   torque  swing   past hold   J1 back    J1 on   J1 end   J5 end  J5 held/model  "
          "(s, Nm, deg; + = on in each joint's direction of travel)")
    for h, r in rows:
        if not r.get("ok"):
            print(f"  {h:4.1f}   -- {r.get('why')}")
            continue
        print(f"  {h:4.1f}  {r['held_s']:5.2f}  {r['torque_nm']:+6.3f}  {r['swing_nm']:5.3f}   {r['past_hold_deg']:+8.3f}  "
              f"{r['j1_back_deg']:7.3f}  {r['j1_on_deg']:7.3f}  {r['j1_end_deg']:+7.3f}  {r['j5_end_deg']:+7.3f}  "
              f"{r['j5_held_nm']:+.3f}/{r['j5_model_nm']:+.3f}")
    print("\n  per hold time: the mean |drift| after the release")
    for h in sorted({h for h, _ in rows}):
        v = [abs(r["j1_end_deg"]) for hh, r in rows if hh == h and r.get("ok")]
        v5 = [abs(r["j5_end_deg"]) for hh, r in rows if hh == h and r.get("ok")]
        if v:
            print(f"    {h:4.1f} s: J1 {np.mean(v):.3f} deg (max {max(v):.3f}), J5 {np.mean(v5):.3f} deg (n={len(v)})")


def selftest():
    """A synthetic cycle whose answers are known: a move, a stop, a hold of 0.3 s, IDLE, then a known drift."""
    print("74 selftest: the analysis on a synthetic cycle")
    rows = []
    q, v = 0.0, 0.0
    sgn = -1.0
    for k in range(600):
        mode = "EXEC"
        if k < 100:
            v = sgn * 1.0
        elif k < 110:
            v = sgn * (1.0 - (k - 99) * 0.1)
        else:
            v = 0.0
        q += v * DT
        meas = q
        trq = 0.36 if 180 <= k < 185 else 0.1
        if k >= 185:                                          # released at tick 185 (hold 0.3 s from rest at 110)
            mode = "IDLE"
            meas = q - sgn * math.radians(1.15) * min(1.0, (k - 185) / 25.0)    # 1.15 deg back over 0.1 s
        row_p = np.zeros(6)
        row_p[J1] = meas
        row_pc = np.zeros(6)
        row_pc[J1] = q if mode == "EXEC" else meas
        row_sc = np.zeros(6)
        row_sc[J1] = v if mode == "EXEC" else 0.0
        row_t = np.zeros(6)
        row_t[J1] = trq
        rows.append((row_pc, row_p, row_sc, row_t, mode))
    d = {"j_pc": np.array([r[0] for r in rows]), "j_p": np.array([r[1] for r in rows]),
         "j_sc": np.array([r[2] for r in rows]), "j_s": np.array([r[2] for r in rows]),
         "j_t": np.array([r[3] for r in rows]), "mode": np.array([r[4] for r in rows])}
    r = analyse(d, sgn)
    check("selftest: analysed", r.get("ok"), r.get("why"))
    if r.get("ok"):
        check(f"selftest: held 0.3 s ({r['held_s']:.3f})", abs(r["held_s"] - 0.3) < 0.01)
        check(f"selftest: the torque at the release 0.36 Nm ({r['torque_nm']:.3f}), its swing 0.26 ({r['swing_nm']:.3f})",
              abs(r["torque_nm"] - 0.36) < 1e-9 and abs(r["swing_nm"] - 0.26) < 1e-9)
        check(f"selftest: 1.15 deg back, ended -1.15 ({r['j1_back_deg']:.3f}, {r['j1_end_deg']:+.3f})",
              abs(r["j1_back_deg"] - 1.15) < 1e-6 and abs(r["j1_end_deg"] + 1.15) < 1e-6 and r["j1_on_deg"] < 1e-9)
    report([(0.3, r)])
    print(f"\n74 selftest: {'ALL PASS' if not fails else f'{len(fails)} FAILED'}")
    return 1 if fails else 0


# ------------------------------------------------------------------------------------------------ on the robot
def run_cycle(robot, cap, q0_deg, target_deg, speed, hold_s):
    from _tracking_common import to_arrays
    tick0 = cap.last_tick()
    n0 = len(cap.packets)
    robot.move_joints(target_deg, speed=speed)
    half = q0_deg[J1] + 0.5 * (target_deg[J1] - q0_deg[J1])
    sgn = 1.0 if target_deg[J1] >= q0_deg[J1] else -1.0
    end = time.monotonic() + 20.0
    while time.monotonic() < end and sgn * (robot.get_joints()[J1] - half) < 0:
        time.sleep(0.005)
    robot.stop(wait=True)                                    # braked along the path, holding in EXEC
    time.sleep(hold_s)
    robot.enter_idle_mode(wait=True)                         # the release (at rest: IDLE within a few ticks)
    time.sleep(WATCH_S + 0.2)
    raw = cap.packets[n0:]
    d = to_arrays(raw)
    if tick0 is not None:
        keep = d["tick"] > tick0
        d = {k: v[keep] for k, v in d.items()}
    st = robot.status()
    errors = [str(e) for e in (st.get("errors") or [])] if st.get("err") else []
    return d, raw, tick0, sgn, errors


def reanalyse(run_dir):
    from _tracking_common import to_arrays
    meta = json.load(open(os.path.join(run_dir, "run.json")))
    rows = []
    for c in meta["cycles"]:
        d = to_arrays(open(os.path.join(run_dir, c["file"]), "rb").read().split(b"\n"))
        if c["tick0"] is not None:
            keep = d["tick"] > c["tick0"]
            d = {k: v[keep] for k, v in d.items()}
        rows.append((c["hold"], analyse(d, c["sgn"])))
    print(f"74 reanalyse {run_dir}: {len(rows)} releases")
    report(rows)
    return 0 if all(r.get("ok") for _, r in rows) else 1


def main():
    ap = argparse.ArgumentParser(description="74: the arm's drift when IDLE lets it go after a braked stop")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true", help="actually move J1 and J5")
    ap.add_argument("--holds", default="0.1,0.3,0.6,1.0,2.0", help="seconds held before the release")
    ap.add_argument("--repeat", type=int, default=2)
    ap.add_argument("--deg", type=float, default=50.0, help="J1's travel (J5: a fifth of it)")
    ap.add_argument("--speed", type=float, default=0.6, help="the move's speed (a fraction; J1 peaks near 60 deg/s)")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--reanalyse", metavar="RUN_DIR")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.reanalyse:
        return reanalyse(args.reanalyse)
    if not args.go:
        print("Nothing moved: add --go (J1 moves --deg, J5 a fifth of it, at speed, stopped and let go; someone at the "
              "E-stop).")
        return 2
    holds = [float(x) for x in args.holds.split(",") if x.strip()]
    from robotclient import RobotClient
    from _tracking_common import Capture
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(str(SYSTEM_XML), str(ROBOTS_DIR))
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    lo = np.degrees(np.asarray(cfg["software_joint_limit_negative"], float))
    hi = np.degrees(np.asarray(cfg["software_joint_limit_positive"], float))
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    st = robot.get_status()
    if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("err"):
        print(f"REFUSED: the arm must be ENABLED, homed and error-free (state={st.get('state')} homed={st.get('homed')} "
              f"err={st.get('err')})")
        return 2
    q0_deg = [float(x) for x in robot.get_joints()]
    target_deg = list(q0_deg)
    for j, dist in ((J1, args.deg), (J5, args.deg * J5_RATIO)):
        up, down = hi[j] - q0_deg[j], q0_deg[j] - lo[j]
        if max(up, down) < dist + 5.0:
            print(f"REFUSED: J{j + 1} at {q0_deg[j]:.1f} deg needs {dist + 5:.0f} deg of room on one side "
                  f"(soft limits {lo[j]:.0f} / {hi[j]:.0f})")
            return 2
        target_deg[j] += (1.0 if up >= down else -1.0) * dist
    stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    run_dir = os.path.join(RAW_DIR, stamp)
    os.makedirs(run_dir, exist_ok=True)
    prev_recipe = (robot.status().get("recipe") or "standard")
    cap = Capture()
    robot.set_recipe("tracking")
    print(f"74: J1 {q0_deg[J1]:.1f} -> {target_deg[J1]:.1f} deg, J5 {q0_deg[J5]:.1f} -> {target_deg[J5]:.1f} deg at "
          f"speed {args.speed}; holds {holds} s x {args.repeat}; raw telemetry: {os.path.relpath(run_dir, ROOT)}")
    rows, cycles = [], []
    try:
        for rep in range(args.repeat):
            for h in holds:
                if st.get("mode") != "EXEC" or rows:
                    robot.enter_exec_mode(wait=True)
                print(f"--- hold {h:.1f} s (#{rep + 1})", flush=True)
                d, raw, tick0, sgn, errors = run_cycle(robot, cap, q0_deg, target_deg, args.speed, h)
                name = f"hold{h:.1f}_{rep + 1}.raw"
                with open(os.path.join(run_dir, name), "wb") as fh:
                    fh.write(b"\n".join(raw))
                cycles.append({"hold": h, "file": name, "tick0": tick0, "sgn": sgn})
                r = analyse(d, sgn)
                rows.append((h, r))
                check(f"hold {h:.1f} s #{rep + 1}: analysed", r.get("ok"), r.get("why"))
                check(f"hold {h:.1f} s #{rep + 1}: no error latched", not errors, errors)
                if r.get("ok"):
                    print(f"    held {r['held_s']:.2f} s; torque {r['torque_nm']:+.3f} Nm (swing {r['swing_nm']:.3f}); "
                          f"J1 then {r['j1_end_deg']:+.3f} deg (back up to {r['j1_back_deg']:.3f})", flush=True)
                if errors:
                    robot.clear_errors(wait=True)
                    robot.enable(wait=True)
                robot.enter_exec_mode(wait=True)
                robot.move_joints(q0_deg, speed=20)
                robot.wait_idle(timeout=30)
    finally:
        with open(os.path.join(run_dir, "run.json"), "w") as fh:
            json.dump({"q0_deg": q0_deg, "target_deg": target_deg, "speed": args.speed, "cycles": cycles}, fh)
        try:
            robot.set_recipe(prev_recipe)
        except Exception:                                    # noqa: BLE001
            pass
        cap.stop()
        robot.disconnect()
    report(rows)
    print(f"\n74: {'done' if not fails else f'{len(fails)} FAILED: ' + '; '.join(fails)} — "
          f"reanalyse: .venv/bin/python api_tests/74_idle_release.py --reanalyse {os.path.relpath(run_dir, ROOT)}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
