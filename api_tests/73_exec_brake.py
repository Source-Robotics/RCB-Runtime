#!/usr/bin/env python3
"""73: EXEC's CONTROLLED pause / stop at speed (2026-10-13, user's choice: braked along the path, as industrial arms
commonly do it). Until then a pause froze the setpoint (the drives stopped the arm as hard as they could) and a stop
switched to IDLE at speed — freedrive with gravity compensation, so the arm COASTED on.

One EXEC joint move, J1 and J5 together on a straight joint-space line (J5 a fifth of J1's travel; the default profile
keeps a multi-joint move exactly on that line), then the EVENT once J1 is half way, near its top speed:
  pause   brake ALONG THE PATH at the joints' hardware ceilings (the robot XML's <joint_acceleration_limit>), hold,
          resume after --hold s from the same point at EXEC's acceleration, finish on the target
  stop    brake the same way, the rest of the move dropped (STOPPED), then HOLD there, still in EXEC (2026-10-14,
          user: as industrial arms end a stop; on 2026-10-13 a stop went to IDLE and J1 moved 0.9 deg once let go)
  idle    enter_idle_mode() mid-move: brake the same way, IDLE once the arm is at rest; what it does once let go
          (freedrive) is reported with the drive's state at that moment — not judged
Measured from the robot's own per-tick telemetry (the `tracking` recipe): the first braking tick (J1's commanded
deceleration past half its ceiling — the plan's own is at most EXEC's, 0.3 of it on PAR6), the commanded deceleration's
peak (<= the ceiling: the handler bounds it exactly), the ticks and the travel to rest (vs v / A and v^2 / 2A), every
commanded point on the start -> target line, the hold (pause) and the finish on the target, (stop) no IDLE and the
measured arm settling on the stop point, (idle) IDLE only with the arm at rest.

Between cases J1 / J5 go back to the start (EXEC, 20 %). Before: the arm ENABLED, homed, error-free; J1 free to turn
--deg (50) and J5 a fifth of that from where they are (the test refuses otherwise). Run it in the simulator first:
    .venv/bin/python tools/sim_run.py --tests "01_mode --go,73_exec_brake --go"
On the arm, someone at the E-stop:
    cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/73_exec_brake.py --go
Options: --deg (50), --speed (0.6: J1 peaks near 60 deg/s), --cases pause,stop,idle, --hold (1.0 s), --selftest (the
analysis on synthetic traces; no robot), --reanalyse temp_data/exec_brake/<run>_<case> (each case's raw telemetry is
saved there; `idle` also prints what the drive was doing when IDLE let it go). Exit 0 = all passed, 1 = a check failed, 2 = refused /
not run.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
J1, J5 = 0, 4
DT = 0.004
J5_RATIO = 0.2                  # J5's travel / J1's
BRAKE_FRAC = 0.5                # J1's commanded deceleration past this x its ceiling = braking (not the plan's own)
REST_DPS = 1.0                  # deg/s: at rest (measured)
LINE_TOL_DEG = 0.003            # the telemetry's 5 decimals (0.0006 deg) on two joints, plus margin
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW_DIR = os.path.join(ROOT, "temp_data", "exec_brake")

fails = []


def check(name, ok, detail=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + ("" if ok or not detail else f"   [{detail}]"), flush=True)
    if not ok:
        fails.append(name)


# ------------------------------------------------------------------------------------------------ the analysis
def analyse(case, d, q0, target, a_lim):
    """d: to_arrays() of the telemetry over one case (rad, rad/s, rad/s^2); q0 / target the move's ends (rad); a_lim the
    ceilings (rad/s^2). Returns a dict of what was measured, or {"ok": False, "why": ...}."""
    pc, p, sc, s, ac, mode = d["j_pc"], d["j_p"], d["j_sc"], d["j_s"], d["j_ac"], d["mode"]
    n = len(pc)
    if n < 10:
        return {"ok": False, "why": f"{n} packets"}
    sgn = 1.0 if target[J1] >= q0[J1] else -1.0
    moving = np.nonzero(np.abs(pc[:, J1] - pc[0, J1]) > 1e-4)[0]    # the hold before the move, then the plan
    if not len(moving):
        return {"ok": False, "why": "J1 never moved"}
    m0 = int(moving[0])
    ref = pc[m0]                    # a sample of the plan: the line runs from it to the target (q0 is only measured)
    idle = np.nonzero(mode[m0:] == "IDLE")[0]
    first_idle = m0 + int(idle[0]) if len(idle) else None
    end = first_idle if first_idle is not None else n         # in IDLE the commanded values are the measured ones
    braking = np.nonzero(-sgn * ac[m0:end, J1] > BRAKE_FRAC * a_lim[J1])[0]
    if not len(braking):
        return {"ok": False, "why": "no braking tick before IDLE (J1's commanded deceleration never passed half its "
                                    "ceiling) — IDLE at speed?"}
    k = m0 + int(braking[0])
    v_at = abs(float(sc[k - 1, J1]))
    rest = next((r for r in range(k, end) if abs(sc[r, J1]) < 1e-9 and abs(sc[r, J5]) < 1e-9), None)
    if rest is None:
        return {"ok": False, "why": "the commanded motion never came to rest before IDLE"}
    lam = (pc[m0:end, J1] - ref[J1]) / (target[J1] - ref[J1])       # where on the line J1 says the point is
    off = np.degrees(np.abs(pc[m0:end, J5] - (ref[J5] + lam * (target[J5] - ref[J5]))))
    back = np.degrees(np.max(-sgn * np.diff(pc[m0:end, J1]))) if end - m0 > 1 else 0.0
    res = {"ok": True, "k": k, "rest": rest, "v_at_dps": math.degrees(v_at),
           "brake_ticks": rest - k + 1, "expected_ticks": int(math.ceil(v_at / a_lim[J1] / DT)) + 2,
           "travel_deg": math.degrees(abs(pc[rest, J1] - pc[k - 1, J1])),
           "expected_deg": math.degrees(v_at * v_at / (2.0 * a_lim[J1]) + v_at * DT),
           "peak_ratio": float(np.max(np.abs(ac[k:rest + 1][:, [J1, J5]]) / a_lim[[J1, J5]])),
           "off_line_deg": float(np.max(off)), "back_deg": max(0.0, float(back)), "first_idle": first_idle}
    if case == "pause":
        resumed = next((r for r in range(rest, end) if abs(sc[r, J1]) > 1e-9), None)
        res["held_s"] = ((resumed if resumed is not None else end) - rest) * DT
        res["hold_drift_deg"] = (math.degrees(float(np.max(np.abs(pc[rest:resumed, J1] - pc[rest, J1]))))
                                 if resumed is not None else None)
        res["final_err_deg"] = math.degrees(abs(float(p[-1, J1]) - target[J1]))
        res["final_cmd_err_deg"] = math.degrees(float(np.max(np.abs(pc[end - 1, [J1, J5]] - target[[J1, J5]]))))
    elif case == "stop":
        # Held in EXEC (2026-10-14): the commanded point stays put, the measured arm settles on it, no IDLE.
        tail = slice(max(rest, end - int(round(0.5 / DT))), end)
        res["hold_s"] = (end - rest) * DT
        res["hold_cmd_drift_deg"] = math.degrees(float(np.max(np.abs(pc[tail, J1] - pc[rest, J1]))))
        res["hold_meas_err_deg"] = math.degrees(float(np.max(np.abs(p[tail, J1] - pc[rest, J1]))))
        res["overshoot_deg"] = math.degrees(max(0.0, float(np.max(sgn * (p[rest:end, J1] - pc[rest, J1])))))
    elif first_idle is not None:
        res["speed_at_idle_dps"] = math.degrees(abs(float(s[first_idle, J1])))
        later = min(n - 1, first_idle + int(round(0.5 / DT)))
        res["moved_in_idle_deg"] = math.degrees(abs(float(p[later, J1] - p[first_idle, J1])))
        res["idle_seen_s"] = (later - first_idle) * DT
        # What the drive was doing when it was let go (reported, not judged): J1's distance from the hold point
        # (+ = past it, in the direction of travel), its measured and commanded torque, the overshoot while held,
        # and where it went in IDLE (+ = on in the direction of travel, - = back).
        hold = float(pc[first_idle - 1, J1])
        res["diag"] = {
            "held_ms": (first_idle - rest) * DT * 1000.0,
            "overshoot_while_held_deg": math.degrees(max(0.0, float(np.max(sgn * (p[rest:first_idle, J1] - hold))))),
            "past_hold_at_idle_deg": math.degrees(sgn * (float(p[first_idle, J1]) - hold)),
            "torque_at_idle_nm": float(d["j_t"][first_idle - 1, J1]) if "j_t" in d else None,
            "cmd_torque_at_idle_nm": float(d["j_tc"][first_idle - 1, J1]) if "j_tc" in d else None,
            "idle_forward_max_deg": math.degrees(float(np.max(sgn * (p[first_idle:later + 1, J1] - p[first_idle, J1])))),
            "idle_back_max_deg": math.degrees(float(np.max(-sgn * (p[first_idle:later + 1, J1] - p[first_idle, J1])))),
            "idle_end_deg": math.degrees(sgn * float(p[later, J1] - p[first_idle, J1])),
        }
    return res


def judge(case, res):
    if not res.get("ok"):
        check(f"{case}: analysed", False, res.get("why"))
        return
    print(f"    J1 at {res['v_at_dps']:.1f} deg/s when the braking began; at rest {res['brake_ticks']} ticks later, "
          f"{res['travel_deg']:.2f} deg on (v^2/2A {res['expected_deg']:.2f})")
    check(f"{case}: the commanded deceleration within the ceilings (peak {res['peak_ratio']:.4f} x)",
          res["peak_ratio"] <= 1.0 + 1e-3, f"{res['peak_ratio']:.4f}")
    check(f"{case}: braked as fast as the ceiling allows: at rest in {res['brake_ticks']} ticks "
          f"(<= v / A + 2: {res['expected_ticks']})", res["brake_ticks"] <= res["expected_ticks"])
    check(f"{case}: travel while braking {res['travel_deg']:.2f} deg <= 1.2 x v^2 / 2A + 0.05",
          res["travel_deg"] <= 1.2 * res["expected_deg"] + 0.05, f"{res['travel_deg']:.2f}")
    check(f"{case}: every commanded point on the start -> target line (J5 off it {res['off_line_deg']:.4f} deg)",
          res["off_line_deg"] <= LINE_TOL_DEG, f"{res['off_line_deg']:.4f}")
    check(f"{case}: the commanded J1 never turned back ({res['back_deg']:.4f} deg)", res["back_deg"] <= 0.001)
    if case == "pause":
        check(f"pause: held at rest ({res['held_s']:.2f} s, the setpoint moved {res['hold_drift_deg']} deg)",
              res["held_s"] > 0.5 and res["hold_drift_deg"] is not None and res["hold_drift_deg"] <= 0.001)
        check(f"pause: resumed and finished on the target (commanded {res['final_cmd_err_deg']:.4f}, measured "
              f"{res['final_err_deg']:.3f} deg off)", res["final_cmd_err_deg"] <= 0.001 and res["final_err_deg"] <= 0.1)
    elif case == "stop":
        print(f"    held {res['hold_s']:.2f} s; the measured J1 overshot the hold point by {res['overshoot_deg']:.3f} deg "
              f"and settled within {res['hold_meas_err_deg']:.3f} deg of it")
        check("stop: held in EXEC — no IDLE (2026-10-14: a stop holds, it no longer lets go)", res["first_idle"] is None,
              res["first_idle"])
        check(f"stop: the setpoint stays on the stop point ({res['hold_cmd_drift_deg']:.4f} deg over the last 0.5 s)",
              res["hold_s"] >= 0.5 and res["hold_cmd_drift_deg"] <= 0.001)
        check(f"stop: the arm settles on it — no coasting, no release ({res['hold_meas_err_deg']:.3f} deg <= 0.1)",
              res["hold_meas_err_deg"] <= 0.1)
    else:
        if "diag" in res:
            dg = res["diag"]
            print(f"    at IDLE: held {dg['held_ms']:.0f} ms at rest first; J1 overshot the hold by "
                  f"{dg['overshoot_while_held_deg']:.3f} deg while held and was {dg['past_hold_at_idle_deg']:+.3f} deg "
                  f"past it when let go; torque measured {dg['torque_at_idle_nm']} / commanded "
                  f"{dg['cmd_torque_at_idle_nm']} Nm")
            print(f"    in IDLE (0.5 s): J1 went up to {dg['idle_forward_max_deg']:.3f} deg on, {dg['idle_back_max_deg']:.3f}"
                  f" deg back, ended {dg['idle_end_deg']:+.3f} deg (+ = on in the direction of travel)")
        check("idle: IDLE came (after the commanded motion was at rest)", res["first_idle"] is not None)
        if res["first_idle"] is not None:
            check(f"idle: the arm at rest when IDLE began (J1 {res['speed_at_idle_dps']:.2f} deg/s <= 3)",
                  res["speed_at_idle_dps"] <= 3.0)
            # IDLE is freedrive: what the arm does after it is let go is reported above, not judged (2026-10-14: on the
            # arm J1 moved 0.9 deg there — the measurement this report exists for).
            print(f"    idle: J1 moved {res['moved_in_idle_deg']:.3f} deg in the {res['idle_seen_s']:.2f} s after "
                  f"IDLE (reported, not judged: freedrive)")


def selftest():
    """The analysis on synthetic traces whose answers are known: a clean controlled pause and stop pass; braking past
    the ceiling, the old frozen pause, the old stop (IDLE at speed), IDLE before rest, the arm drifting in IDLE and J5
    off the line are caught."""
    print("73 selftest: the analysis on synthetic traces")
    a = np.array([32.0, 10.0, 11.3, 51.3, 51.3, 20.7])
    q0 = np.zeros(6)
    target = q0.copy()
    target[J1], target[J5] = math.radians(50.0), math.radians(50.0 * J5_RATIO)
    line = target / target[J1]

    def trace(case, a_brake=32.0, freeze=False, old_stop=False, idle_early=False, drift=0.0, j5_off=0.0,
              stop_idles=False):
        """J1 speeds up at 4 rad/s^2 to 1.05 rad/s, brakes at a_brake once half way (J5 on the line), holds; pause:
        resumes after 1.2 s and finishes on the target; stop: holds (or, stop_idles, goes IDLE like the 10-13 stop);
        idle: IDLE 2 ticks after rest (there commanded = measured). The measured arm drifts at `drift` rad/s once at
        rest."""
        rows, q, v, mode, phase, hold, q_idle = [], 0.0, 0.0, "EXEC", "up", 0, 0.0
        for _ in range(5000):
            a_cmd = 0.0
            if phase == "up":
                a_cmd = 4.0 if v < 1.05 else 0.0
                if q >= 0.5 * target[J1]:
                    phase = "brake"
                    if old_stop:
                        mode, phase = "IDLE", "coast"
            if phase == "brake":
                a_cmd = -v / DT if freeze else -min(a_brake, v / DT)
                if idle_early and v < 0.5:
                    mode = "IDLE"
                if v + a_cmd * DT <= 1e-12:
                    a_cmd, phase = -v / DT, "rest"
            elif phase == "rest":
                hold += 1
                if case == "pause" and hold > 300:
                    phase = "resume"
                if (case == "idle" or stop_idles) and hold > 2:
                    mode = "IDLE"
            elif phase == "resume":
                a_cmd = 4.0 if target[J1] - q > v * v / 8.0 + v * DT else -4.0
                if a_cmd < 0 and v + a_cmd * DT <= 0:
                    a_cmd, phase = -v / DT, "done"
            elif phase == "coast":
                a_cmd = -0.5                                  # freedrive: only friction slows it
            v = max(0.0, v + a_cmd * DT)
            q = q + v * DT if phase != "done" else target[J1]
            if phase == "resume" and q >= target[J1]:          # arrived: no overshoot to come back from
                q, v, phase = target[J1], 0.0, "done"
            if phase == "rest":
                q_idle += drift * DT
            qm = line * q
            qm[J1] += q_idle
            qc = line * q
            qc[J5] += j5_off if 0.3 < q < 0.6 else 0.0
            if mode == "IDLE":
                rows.append((qm.copy(), qm.copy(), np.zeros(6), line * v, np.zeros(6), mode))
            else:
                rows.append((qc.copy(), qm.copy(), line * v, line * v, line * a_cmd, mode))
            if (phase == "done" and q >= target[J1]) and len(rows) > 80 and all(r[5] == "EXEC" for r in rows[-75:]):
                break
            if mode == "IDLE" and sum(r[5] == "IDLE" for r in rows) > 200:
                break
            if case == "stop" and phase == "rest" and hold > 300:
                break
        return {"j_pc": np.array([r[0] for r in rows]), "j_p": np.array([r[1] for r in rows]),
                "j_sc": np.array([r[2] for r in rows]), "j_s": np.array([r[3] for r in rows]),
                "j_ac": np.array([r[4] for r in rows]), "mode": np.array([r[5] for r in rows])}

    def failures(case, **kw):
        global fails
        saved, fails = fails, []
        judge(case, analyse(case, trace(case, **kw), q0, target, a))
        got, fails = fails, saved
        return got

    clean = not failures("pause") and not failures("stop") and not failures("idle")
    hard = any("deceleration within" in f for f in failures("pause", a_brake=48.0))
    frozen = any("deceleration within" in f for f in failures("pause", freeze=True))
    old_stop = bool(failures("stop", old_stop=True)) and bool(failures("idle", old_stop=True))
    early = bool(failures("idle", idle_early=True))
    let_go = any("no IDLE" in f for f in failures("stop", stop_idles=True))
    drift = any("settles on it" in f for f in failures("stop", drift=0.05))
    off = any("line" in f for f in failures("pause", j5_off=math.radians(0.05)))
    check("selftest: a clean controlled pause, stop (held) and idle pass", clean)
    check("selftest: braking at 1.5 x the ceiling is caught", hard)
    check("selftest: the old frozen pause (the speed to 0 in one tick) is caught", frozen)
    check("selftest: the old stop (IDLE at speed, the arm coasting) is caught, as a stop and as an idle", old_stop)
    check("selftest: IDLE before the commanded motion was at rest is caught", early)
    check("selftest: a stop that lets go into IDLE (the 2026-10-13 stop) is caught", let_go)
    check("selftest: the arm drifting away from the hold (0.05 rad/s) is caught", drift)
    check("selftest: J5 0.05 deg off the line is caught", off)
    print(f"\n73 selftest: {'ALL PASS' if not fails else f'{len(fails)} FAILED'}")
    return 1 if fails else 0


# ------------------------------------------------------------------------------------------------ on the robot
def run_case(robot, cap, case, q0_deg, target_deg, speed, hold_s):
    from _tracking_common import to_arrays
    tick0 = cap.last_tick()
    n0 = len(cap.packets)
    robot.move_joints(target_deg, speed=speed)
    half = q0_deg[J1] + 0.5 * (target_deg[J1] - q0_deg[J1])
    sgn = 1.0 if target_deg[J1] >= q0_deg[J1] else -1.0
    end = time.monotonic() + 20.0
    while time.monotonic() < end and sgn * (robot.get_joints()[J1] - half) < 0:
        time.sleep(0.005)
    if case == "pause":
        robot.pause(wait=True)
        time.sleep(hold_s)
        robot.resume(wait=True)
        robot.wait_idle(timeout=30)
        time.sleep(0.3)
    elif case == "stop":
        robot.stop(wait=True)                                # brakes, holds in EXEC
        time.sleep(1.0)                                      # watch it hold
    else:
        robot.enter_idle_mode(wait=True)                     # brakes, then IDLE (freedrive) at rest
        time.sleep(1.0)                                      # watch the arm let go
        # Leaving EXEC mid-program aborts the move (by design): wait_idle() hands that expected failure back.
        from robotclient import MotionFailed
        try:
            robot.wait_idle(timeout=5)
            check("idle: the interrupted move reported ABORTED", False, "no failure reported")
        except MotionFailed as e:
            check("idle: the interrupted move reported ABORTED", "ABORTED" in str(e), str(e)[:120])
    raw = cap.packets[n0:]
    d = to_arrays(raw)
    if tick0 is not None:
        keep = d["tick"] > tick0
        d = {k: v[keep] for k, v in d.items()}
    st = robot.status()
    errors = [str(e) for e in (st.get("errors") or [])] if st.get("err") else []
    print(f"    {len(d['tick'])} telemetry packets; mode now {st.get('mode')}; robot errors: {errors}")
    return d, errors, raw, tick0


def save_raw(stamp, case, raw, tick0, q0, target, a_lim):
    """The case's raw packets + what analyse() needs, under temp_data/exec_brake/ (--reanalyse reads them back)."""
    import json
    out = os.path.join(RAW_DIR, f"{stamp}_{case}")
    os.makedirs(RAW_DIR, exist_ok=True)
    with open(out + ".raw", "wb") as fh:
        fh.write(b"\n".join(raw))
    with open(out + ".json", "w") as fh:
        json.dump({"case": case, "tick0": tick0, "q0": list(map(float, q0)), "target": list(map(float, target)),
                   "a_lim": list(map(float, a_lim))}, fh)
    return out


def reanalyse(path):
    """A saved case analysed again (no robot)."""
    import json
    from _tracking_common import to_arrays
    stem = path[:-5] if path.endswith(".json") else path[:-4] if path.endswith(".raw") else path
    meta = json.load(open(stem + ".json"))
    d = to_arrays(open(stem + ".raw", "rb").read().split(b"\n"))
    if meta["tick0"] is not None:
        keep = d["tick"] > meta["tick0"]
        d = {k: v[keep] for k, v in d.items()}
    print(f"73 reanalyse {stem}: {len(d['tick'])} packets")
    judge(meta["case"], analyse(meta["case"], d, np.asarray(meta["q0"]), np.asarray(meta["target"]),
                                np.asarray(meta["a_lim"])))
    print(f"\n73: {'ALL PASS' if not fails else f'{len(fails)} FAILED: ' + '; '.join(fails)}")
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(description="73: EXEC's controlled pause / stop at speed")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true", help="actually move J1 and J5")
    ap.add_argument("--deg", type=float, default=50.0, help="J1's travel (J5: a fifth of it)")
    ap.add_argument("--speed", type=float, default=60.0, help="the move's speed (percent; J1 peaks near 60 deg/s)")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds paused before the resume")
    ap.add_argument("--cases", default="pause,stop,idle")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--reanalyse", metavar="RUN", help="analyse a saved case again (temp_data/exec_brake/<run>.raw)")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.reanalyse:
        return reanalyse(args.reanalyse)
    if not args.go:
        print("Nothing moved: add --go (J1 moves --deg, J5 a fifth of it, at speed; someone at the E-stop).")
        return 2
    from robotclient import RobotClient
    from _tracking_common import Capture
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(str(SYSTEM_XML), str(ROBOTS_DIR))
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    a_lim = np.asarray(cfg["joint_acceleration_limit"], float)
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
    move = {J1: args.deg, J5: args.deg * J5_RATIO}
    direction = {}
    for j, dist in move.items():
        up, down = hi[j] - q0_deg[j], q0_deg[j] - lo[j]
        if max(up, down) < dist + 5.0:
            print(f"REFUSED: J{j + 1} at {q0_deg[j]:.1f} deg needs {dist + 5:.0f} deg of room on one side "
                  f"(soft limits {lo[j]:.0f} / {hi[j]:.0f})")
            return 2
        direction[j] = 1.0 if up >= down else -1.0
    target_deg = list(q0_deg)
    for j, dist in move.items():
        target_deg[j] += direction[j] * dist
    q0, target = np.radians(q0_deg), np.radians(target_deg)
    prev_recipe = (robot.status().get("recipe") or "standard")
    cap = Capture()
    robot.set_recipe("tracking")
    stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    print(f"73: J1 {q0_deg[J1]:.1f} -> {target_deg[J1]:.1f} deg, J5 {q0_deg[J5]:.1f} -> {target_deg[J5]:.1f} deg at "
          f"speed {args.speed}; ceilings J1 {a_lim[J1]:.1f}, J5 {a_lim[J5]:.1f} rad/s^2")
    try:
        if st.get("mode") != "EXEC":
            robot.enter_exec_mode(wait=True)
        for case in [c.strip() for c in args.cases.split(",") if c.strip()]:
            print(f"\n--- {case}")
            d, errors, raw, tick0 = run_case(robot, cap, case, q0_deg, target_deg, args.speed, args.hold)
            saved = save_raw(stamp, case, raw, tick0, q0, target, a_lim)
            print(f"    saved: {os.path.relpath(saved, ROOT)}.raw (--reanalyse)")
            judge(case, analyse(case, d, q0, target, a_lim))
            check(f"{case}: no error latched", not errors, errors)
            if errors or case == "idle":     # leaving EXEC mid-program latches the executor's abort (by design)
                robot.clear_errors(wait=True)
                robot.enable(wait=True)
            robot.enter_exec_mode(wait=True)
            robot.move_joints(q0_deg, speed=20)
            robot.wait_idle(timeout=30)
    finally:
        try:
            robot.set_recipe(prev_recipe)
        except Exception:                                    # noqa: BLE001
            pass
        cap.stop()
        robot.disconnect()
    print(f"\n73: {'ALL PASS' if not fails else f'{len(fails)} FAILED: ' + '; '.join(fails)}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
