#!/usr/bin/env python3
"""46: what "normal" looks like to the step-2 safety monitor — measure it BEFORE choosing thresholds.

The monitor (TODO/safety_stack_research.md §8, step 2) will trip on measured position past the
soft limits, per-joint overspeed, and commanded-vs-measured TRACKING error. A threshold picked by
guessing either never fires or fires in normal use — and a monitor that fires in normal use gets
switched off. This records, tick by tick, what the arm actually does in EXEC:

  * tracking error (commanded - measured position) while moving and while holding,
  * raw and filtered measured speed, against the joint's hardware ceiling and against command,
  * speed noise at standstill,
  * how close the arm came to its soft limits, TCP speed, motor temperature, dropped packets,

and prints a recommended threshold per joint from those numbers.

WHY TELEMETRY, NOT SHARED MEMORY. Sampling shared memory from Python pairs values from different
ticks (TODO/servo_mode_options.md §15.4): RTI writes the measured position early in a tick and the
commanded position late, so a torn read is off by one tick of motion — up to ~1.8 deg at 7.7 rad/s,
which would dwarf the tracking error being measured. The UDP telemetry packet is built INSIDE the
tick, after dispatch and after process_joint_data, exactly where the monitor will run, so each
packet is one consistent tick. The script switches the telemetry recipe to `monitor_baseline` for
the run and restores the previous one after.

MOTION. Only the selected joints (default J4, J5, J6; J1 allowed with --joints; J2/J3 refused),
one at a time: start -> +amp -> -amp -> start, at 25 %, 50 %, then --max-speed % (default 100) of
the EXEC limits, with a 1 s hold after every move; then all selected joints together at max speed.
Ctrl+C at any time stops the robot (stop), restores the recipe and releases control. The robot
must already be ENABLED and homed; the script enters EXEC itself (that moves nothing) and takes
control without forcing.

    python3 api_tests/46_monitor_baseline.py --selftest   # check the analysis on synthetic data
    python3 api_tests/46_monitor_baseline.py              # dry run: prints the plan, moves nothing
    python3 api_tests/46_monitor_baseline.py --go
    python3 api_tests/46_monitor_baseline.py --go --hand 20   # + 20 s of hand-guiding in IDLE
    python3 api_tests/46_monitor_baseline.py --listen 300     # PASSIVE: moves nothing; you run your
                                                              # own programs meanwhile (all joints,
                                                              # any mode) — how J1-J3 get measured

Results: printed, and saved to temp_data/monitor_baseline/<time>.npz + .json.
"""
import argparse
import json
import os
import socket
import sys
import threading
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
from robotclient.rti_client import parse_state_packet                 # noqa: E402

RECIPE = "monitor_baseline"
TELEMETRY_PORT = 30003          # config/networking.py TELEMETRY_PORT (import would pull in zmq setup)
ALLOWED_JOINTS = (1, 4, 5, 6)   # 1-based. J2/J3 never move in these tests.
MOVING_CMD_SPEED = 1e-3         # rad/s — a commanded speed above this is "moving"
HOLD_AFTER_S = 0.5              # a sample is "holding" once the command has been still this long
GLITCH_DEG = 3.0                # an arm cannot move this far in one 4 ms tick (api_tests/external)


# ----------------------------------------------------------------------------- telemetry capture
class Capture:
    """Receives the robot's per-tick telemetry broadcast on this machine."""
    def __init__(self, port=TELEMETRY_PORT):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.2)
        self.packets = []
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while self._run:
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            self.packets.append(data)

    def stop(self):
        self._run = False
        self._t.join(timeout=1.0)
        self._sock.close()


# ----------------------------------------------------------------------------- analysis (pure)
def to_arrays(packets, joint_num=6):
    """Raw packets -> dict of numpy arrays ordered by tick, duplicates dropped. Packets missing a
    field (e.g. sent under the previous recipe) are skipped."""
    need = ("j_p", "j_pc", "j_s", "j_s_f", "j_sc", "TCP_s", "TCP_s_f", "tick")
    rows = []
    for raw in packets:
        try:
            r = parse_state_packet(raw.decode() if isinstance(raw, bytes) else raw)
        except Exception:
            continue
        if all(k in r for k in need):
            rows.append(r)
    rows.sort(key=lambda r: r["tick"])
    out, seen = [], set()
    for r in rows:
        if r["tick"] in seen:
            continue
        seen.add(r["tick"])
        out.append(r)

    def vec(k, n):
        return np.array([np.resize(np.asarray(r[k], float), n) for r in out]) if out else np.zeros((0, n))

    temps = [np.atleast_1d(np.asarray(r.get("m_temp", np.nan), float)) for r in out]
    width = max((t.size for t in temps), default=1)
    return {
        "tick": np.array([r["tick"] for r in out], dtype=np.int64),
        "mode": np.array([str(r.get("mode", "")) for r in out]),
        "j_p": vec("j_p", joint_num), "j_pc": vec("j_pc", joint_num),
        "j_s": vec("j_s", joint_num), "j_s_f": vec("j_s_f", joint_num), "j_sc": vec("j_sc", joint_num),
        "TCP_s": vec("TCP_s", 6), "TCP_s_f": vec("TCP_s_f", 6),
        "m_temp": np.array([np.pad(t, (0, width - t.size), constant_values=np.nan) for t in temps])
        if temps else np.zeros((0, 1)),
    }


def glitch_mask(pos, jump_deg=GLITCH_DEG, half=3):
    """True where a sample is impossible: more than jump_deg from the MEDIAN of its neighbours on
    any joint (same rule as api_tests/external deglitch)."""
    n = pos.shape[0]
    if n < 2 * half + 1:
        return np.zeros(n, dtype=bool)
    pad = np.pad(pos, ((half, half), (0, 0)), mode="edge")
    win = np.lib.stride_tricks.sliding_window_view(pad, 2 * half + 1, axis=0)   # (n, J, w)
    neigh = np.delete(win, half, axis=2)
    med = np.median(neigh, axis=2)
    return np.any(np.abs(pos - med) > np.radians(jump_deg), axis=1)


def longest_run(mask):
    best = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0
        best = max(best, cur)
    return best


def analyse(d, joints0, lo, hi, ceiling, dt, modes=("EXEC",)):
    """joints0: 0-based joint indices to report. lo/hi/ceiling: per-joint rad, rad/s.
    Only samples taken in `modes` count (default EXEC)."""
    n = d["tick"].size
    res = {"samples": int(n)}
    if n == 0:
        return res
    gaps = np.diff(d["tick"]) - 1
    res["dropped_packets"] = int(gaps[gaps > 0].sum())
    exec_ = np.isin(d["mode"], list(modes))
    bad = glitch_mask(d["j_p"])
    res["glitch_samples"] = int(bad.sum())
    ok = exec_ & ~bad

    cmd_speed = np.abs(d["j_sc"][:, joints0]).max(axis=1) if joints0 else np.zeros(n)
    moving = cmd_speed > MOVING_CMD_SPEED
    still_ticks = np.zeros(n, dtype=np.int64)       # ticks since the command last moved
    c = 0
    for i in range(n):
        c = 0 if moving[i] else c + 1
        still_ticks[i] = c
    holding = still_ticks >= int(round(HOLD_AFTER_S / dt))
    res["mode_samples"] = int(exec_.sum())
    res["modes"] = list(modes)
    res["moving_samples"] = int((ok & moving).sum())
    res["holding_samples"] = int((ok & holding).sum())

    err = np.degrees(d["j_pc"] - d["j_p"])
    spd = np.degrees(d["j_s"])
    spd_f = np.degrees(d["j_s_f"])
    spd_c = np.degrees(d["j_sc"])
    per = {}
    for j in joints0:
        mv, hd = ok & moving, ok & holding
        e_mv, e_hd = np.abs(err[mv, j]), np.abs(err[hd, j])
        ceil_deg = np.degrees(ceiling[j])
        raw = np.abs(spd[ok, j])
        filt = np.abs(spd_f[ok, j])
        over_cmd = (np.abs(spd[mv, j]) - np.abs(spd_c[mv, j])) if mv.any() else np.zeros(1)
        margin = np.degrees(np.minimum(d["j_p"][ok, j] - lo[j], hi[j] - d["j_p"][ok, j]))
        per[f"J{j + 1}"] = {
            "track_move_max_deg": float(e_mv.max()) if e_mv.size else None,
            "track_move_p999_deg": float(np.percentile(e_mv, 99.9)) if e_mv.size else None,
            "track_hold_max_deg": float(e_hd.max()) if e_hd.size else None,
            "track_hold_std_deg": float(np.std(err[hd, j])) if e_hd.size else None,
            "cmd_speed_max_deg_s": float(np.abs(spd_c[ok, j]).max()) if ok.any() else None,
            "raw_speed_max_deg_s": float(raw.max()) if raw.size else None,
            "raw_speed_max_ratio_of_ceiling": float(raw.max() / ceil_deg) if raw.size else None,
            "filt_speed_max_deg_s": float(filt.max()) if filt.size else None,
            "raw_over_command_max_deg_s": float(over_cmd.max()),
            "raw_speed_hold_std_deg_s": float(np.std(spd[hd, j])) if hd.any() else None,
            "raw_speed_hold_max_deg_s": float(np.abs(spd[hd, j]).max()) if hd.any() else None,
            "ticks_above_ceiling_longest": longest_run(np.abs(spd[ok, j]) > ceil_deg),
            "soft_limit_margin_min_deg": float(margin.min()) if margin.size else None,
            "ceiling_deg_s": float(ceil_deg),
        }
    res["joints"] = per
    tcp = np.linalg.norm(d["TCP_s"][ok, :3], axis=1)
    tcp_f = np.linalg.norm(d["TCP_s_f"][ok, :3], axis=1)
    res["tcp_speed_max_m_s"] = float(tcp.max()) if tcp.size else None
    res["tcp_speed_filt_max_m_s"] = float(tcp_f.max()) if tcp_f.size else None
    res["motor_temp_max_c"] = [None if np.isnan(x) else float(x) for x in np.nanmax(d["m_temp"], axis=0)] \
        if d["m_temp"].size and not np.all(np.isnan(d["m_temp"])) else None
    return res


def recommend(res):
    """Thresholds from ONE session — a starting point, not a verdict."""
    rec = {}
    for name, s in (res.get("joints") or {}).items():
        mv = s["track_move_max_deg"] or 0.0
        r = s["raw_speed_max_ratio_of_ceiling"] or 0.0
        rec[name] = {
            # Twice the worst seen, rounded up to 0.5 deg, never under 1 deg.
            "tracking_limit_deg": max(1.0, float(np.ceil(2.0 * mv / 0.5) * 0.5)),
            # Ceiling x margin: at least 1.2, and 10 % above the worst raw reading seen.
            "overspeed_margin": max(1.2, float(np.ceil(1.1 * r / 0.05) * 0.05)),
            # Must outlast the longest run seen above the ceiling (normally 0).
            "overspeed_sustain_ticks": max(5, 2 * int(s["ticks_above_ceiling_longest"])),
        }
    return rec


# ----------------------------------------------------------------------------- self-test
def selftest():
    """Synthetic session with KNOWN answers, fed through the real parser and analysis."""
    dt, n = 0.004, 3000
    ok_all = True

    def chk(name, cond, detail=""):
        nonlocal ok_all
        ok_all &= bool(cond)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail and not cond else ""))

    t = np.arange(n) * dt
    pc = np.zeros((n, 6)); sc = np.zeros((n, 6))
    move = (t > 2.0) & (t < 6.0)                          # J5 moves 2..6 s, holds otherwise
    w = 2 * np.pi * 0.25
    pc[move, 4] = 0.5 * np.sin(w * (t[move] - 2.0))
    sc[move, 4] = 0.5 * w * np.cos(w * (t[move] - 2.0))
    p = pc.copy()
    p[:, 4] -= np.radians(0.40) * (sc[:, 4] / (0.5 * w))  # tracking error exactly 0.40 deg at peak speed
    p[t > 9.0, 4] += np.radians(0.05)                     # a 0.05 deg standing error while holding
    settle = t >= 6.0                                     # a 0.3 deg settle transient after the move:
    p[settle, 4] -= np.radians(0.3) * np.exp(-(t[settle] - 6.0) / 0.1)   # NOT 'holding' yet
    s = sc.copy(); s[:, 4] += np.radians(1.0) * np.sin(np.arange(n))   # 1 deg/s of noise
    tick = 1000 + np.arange(n)
    drop = np.zeros(n, bool); drop[[100, 101, 2500]] = True            # 3 lost packets
    glitch_at = 1500
    pkts = []
    for i in range(n):
        if drop[i]:
            continue
        jp = p[i].copy()
        if i == glitch_at:
            jp[2] += np.radians(40.0)                     # one impossible packet (J3 +40 deg)
        f = lambda a, k=5: ",".join(f"{x:.{k}f}" for x in a)
        pkts.append(f"j_p:{f(jp)},j_pc:{f(pc[i])},j_s:{f(s[i], 4)},j_s_f:{f(sc[i], 4)},j_sc:{f(sc[i], 4)},"
                    f"TCP_s:{f([0.3, 0.4, 0, 0, 0, 0], 4)},TCP_s_f:{f([0.3, 0.4, 0, 0, 0, 0], 4)},"
                    f"m_temp:{f([40, 41, 42, 43, 44, 45, 46], 1)},tick:{tick[i]},mode:EXEC".encode())
    pkts.append(pkts[10])                                  # a duplicate
    d = to_arrays(pkts[::-1])                              # out of order on purpose
    lo = np.full(6, -np.pi); hi = np.full(6, np.pi); ceil = np.full(6, np.radians(90.0))
    res = analyse(d, [4], lo, hi, ceil, dt)
    j5 = res["joints"]["J5"]
    chk("duplicates dropped and order restored", res["samples"] == n - 3, res["samples"])
    chk("dropped packets counted", res["dropped_packets"] == 3, res["dropped_packets"])
    chk("the impossible packet is flagged (and only it)", res["glitch_samples"] == 1, res["glitch_samples"])
    chk("moving tracking error = 0.40 deg", abs(j5["track_move_max_deg"] - 0.40) < 0.01, j5["track_move_max_deg"])
    chk("holding tracking error = 0.05 deg", abs(j5["track_hold_max_deg"] - 0.05) < 0.002, j5["track_hold_max_deg"])
    peak = np.degrees(0.5 * w)
    chk("commanded peak speed", abs(j5["cmd_speed_max_deg_s"] - peak) < 0.05, j5["cmd_speed_max_deg_s"])
    chk("raw speed = command + <=1 deg/s noise", peak <= j5["raw_speed_max_deg_s"] <= peak + 1.01, j5["raw_speed_max_deg_s"])
    chk("hold noise ~ 1 deg/s", 0.5 < j5["raw_speed_hold_std_deg_s"] < 1.0, j5["raw_speed_hold_std_deg_s"])
    chk("soft-limit margin = 180 - 28.6 deg", abs(j5["soft_limit_margin_min_deg"] - (180 - np.degrees(0.5))) < 0.05,
        j5["soft_limit_margin_min_deg"])
    chk("TCP speed = 0.5 m/s", abs(res["tcp_speed_max_m_s"] - 0.5) < 1e-3, res["tcp_speed_max_m_s"])
    chk("temperatures per motor", res["motor_temp_max_c"] == [40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0],
        res["motor_temp_max_c"])
    rec = recommend(res)["J5"]
    chk("recommended tracking limit = 1.0 deg (2 x 0.40 -> 1.0 floor)", rec["tracking_limit_deg"] == 1.0, rec)
    chk("recommended overspeed margin >= 1.2", rec["overspeed_margin"] >= 1.2, rec)
    print("selftest:", "ALL PASSED" if ok_all else "FAILED")
    return 0 if ok_all else 1


# ----------------------------------------------------------------------------- robot session
def claim_control(robot, seconds=15.0):
    """Take control WITHOUT forcing; wait for the dashboard's 5 s release if it holds it."""
    from robotclient.exceptions import CommandRejected
    end = time.monotonic() + seconds
    told = False
    while True:
        try:
            robot.take_control()
            return
        except CommandRejected as exc:
            if exc.code != "CONTROL_HELD" or time.monotonic() > end:
                raise
            if not told:
                print(f"Control is held by {robot.control_holder()} — waiting up to {seconds:.0f} s...")
                told = True
            time.sleep(1.0)


def plan_moves(q0_deg, joints1, lo_deg, hi_deg, amps, speeds, clearance=10.0):
    """[(label, target_deg, speed_pct)] — each joint alone, then all together at the top speed."""
    moves = []
    for j1 in joints1:
        j = j1 - 1
        a = amps[j1]
        up = min(q0_deg[j] + a, hi_deg[j] - clearance)
        dn = max(q0_deg[j] - a, lo_deg[j] + clearance)
        for sp in speeds:
            for tag, v in (("+", up), ("-", dn), ("0", q0_deg[j])):
                t = np.array(q0_deg, float)
                t[j] = v
                moves.append((f"J{j1} {tag} @{sp:.0f}%", t, sp))
    both = np.array(q0_deg, float)
    for j1 in joints1:
        j = j1 - 1
        both[j] = min(q0_deg[j] + amps[j1], hi_deg[j] - clearance)
    moves.append((f"all {'+'.join(f'J{j}' for j in joints1)} together @{speeds[-1]:.0f}%", both, speeds[-1]))
    moves.append(("back to start", np.array(q0_deg, float), speeds[-1]))
    return moves


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true", help="actually move the arm")
    ap.add_argument("--selftest", action="store_true", help="check the analysis on synthetic data and exit")
    ap.add_argument("--joints", default="4,5,6", help="1-based joints to move (allowed: 1,4,5,6)")
    ap.add_argument("--max-speed", type=float, default=100.0, help="top EXEC speed percent")
    ap.add_argument("--amp", type=float, default=None, help="travel each way, deg (default J4/J5 40, J6 60, J1 30)")
    ap.add_argument("--hand", type=float, default=0.0, help="seconds of hand-guiding to record in IDLE at the end")
    ap.add_argument("--listen", type=float, default=0.0,
                    help="PASSIVE: record this many seconds without moving anything while you run your own "
                         "programs (all joints, EXEC/JOG/RTI/SOFT_RTI reported separately)")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.listen > 0:
        return listen_session(args)

    joints1 = [int(x) for x in args.joints.split(",") if x.strip()]
    if any(j not in ALLOWED_JOINTS for j in joints1):
        print(f"Only joints {ALLOWED_JOINTS} may move in these tests (J2/J3 never). Got {joints1}.")
        return 2
    default_amp = {1: 30.0, 4: 40.0, 5: 40.0, 6: 60.0}
    amps = {j: (args.amp if args.amp is not None else default_amp[j]) for j in joints1}
    speeds = sorted({min(25.0, args.max_speed), min(50.0, args.max_speed), args.max_speed})

    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    lo = np.asarray(cfg["software_joint_limit_negative"], float)
    hi = np.asarray(cfg["software_joint_limit_positive"], float)
    ceiling = np.asarray(cfg["joint_velocity_limit"], float)
    v_ex = np.asarray(cfg["exec_velocity_limit"], float)
    a_ex = np.asarray(cfg["exec_acceleration_limit"], float)
    dt = float(cfg.get("delta_t", 0.004))

    from robotclient import RobotClient
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    capture = None
    prev_recipe = None
    claimed = False
    try:
        st = robot.status()
        q0 = np.asarray(robot.get_joints(), float)
        moves = plan_moves(q0, joints1, np.degrees(lo), np.degrees(hi), amps, speeds)
        print(f"Joints now (deg): {np.round(q0, 2).tolist()}")
        print(f"Plan: {len(moves)} moves, joints {joints1}, speeds {speeds} % of EXEC limits, 1 s hold after each.")
        for j1 in joints1:
            j = j1 - 1
            # No peak-speed estimate: the first version predicted 266 deg/s for J4/J5 and the planner
            # actually commanded 153 (its trapezoid does not use the full acceleration). The peak
            # reached is MEASURED and reported at the end instead.
            print(f"  J{j1}: +/-{amps[j1]:.0f} deg around {q0[j]:.1f} (EXEC limit {np.degrees(v_ex[j]):.0f} "
                  f"deg/s, ceiling {np.degrees(ceiling[j]):.0f})")
        if args.hand > 0:
            print(f"  then {args.hand:.0f} s in IDLE for you to move the arm by hand (gravity comp).")
        if not args.go:
            print("DRY RUN: re-run with --go to execute. Nothing moved.")
            return 0
        if "motion_aborted" not in st:
            print("command_executor is running old code — restart it first. Nothing moved.")
            return 2
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") not in ("IDLE", "EXEC"):
            print(f"Robot must be ENABLED + homed, in IDLE or EXEC (got mode={st.get('mode')} "
                  f"state={st.get('state')} homed={st.get('homed')}). Nothing moved.")
            return 2

        capture = Capture()
        claim_control(robot)
        claimed = True
        if st.get("motion_aborted"):
            robot.clear_errors(wait=True)
        if st.get("mode") != "EXEC":
            print("Entering EXEC (no motion).")
            robot.enter_exec_mode(wait=True)
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        time.sleep(0.5)
        probe = to_arrays(capture.packets[-50:])
        if probe["tick"].size == 0:
            print("No telemetry with the monitor_baseline fields is arriving on UDP 30003 — is RTI's "
                  "telemetry going somewhere else (config/networking.py TELEMETRY_DESTINATION)? Nothing moved.")
            return 2
        capture.packets.clear()
        time.sleep(2.0)                                   # 2 s of quiet standstill first
        for i, (label, target, sp) in enumerate(moves, 1):
            print(f"  [{i:2d}/{len(moves)}] {label}")
            robot.move_joints(*[float(x) for x in target], speed=sp)
            fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
            if fails:
                print(f"Move failed: {fails}. Stopping the session.")
                break
            time.sleep(1.0)
        if args.hand > 0:
            robot.enter_idle_mode(wait=True)
            print(f"\nIDLE now — move the joints by hand for {args.hand:.0f} s (J2/J3 too if you like; "
                  f"nothing moves by itself).")
            time.sleep(args.hand)
            print("Hand window over.")
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the robot.")
        try:
            robot.stop()
        except Exception:
            pass
    finally:
        if capture is not None:
            capture.stop()
        try:
            if prev_recipe and prev_recipe != RECIPE:
                robot.set_recipe(prev_recipe)
        except Exception as exc:
            print(f"Could not restore recipe {prev_recipe!r}: {exc}")
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()

    if capture is None or not capture.packets:
        return 0
    d = to_arrays(capture.packets)
    j0 = [j - 1 for j in joints1]
    res = analyse(d, j0, lo, hi, ceiling, dt)
    rec = recommend(res)
    hand = None
    if args.hand > 0:
        idle = d["mode"] == "IDLE"
        if idle.any():
            hand = {f"J{j + 1}": float(np.degrees(np.abs(d["j_s"][idle, j])).max()) for j in range(6)}
    print_report(res, rec, hand)
    out_dir = os.path.join(_ROOT, "temp_data", "monitor_baseline")
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    np.savez_compressed(os.path.join(out_dir, f"{stamp}.npz"), **d)
    with open(os.path.join(out_dir, f"{stamp}.json"), "w") as fh:
        json.dump({"result": res, "recommended": rec, "hand_speed_max_deg_s": hand,
                   "joints": joints1, "speeds": speeds, "amps": amps}, fh, indent=2)
    print(f"\nSaved temp_data/monitor_baseline/{stamp}.npz and .json")
    return 0


def _limits():
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    return (np.asarray(cfg["software_joint_limit_negative"], float),
            np.asarray(cfg["software_joint_limit_positive"], float),
            np.asarray(cfg["joint_velocity_limit"], float), float(cfg.get("delta_t", 0.004)))


def listen_session(args):
    """Record telemetry WITHOUT moving anything, while the user runs their own programs.

    The telemetry recipe can only be changed by whoever holds control, so: take control briefly
    (waiting for the dashboard's release), switch the recipe, give control back, record, then take
    it again only to restore the recipe. Nothing here moves the arm or changes a mode.
    """
    from robotclient import RobotClient
    lo, hi, ceiling, dt = _limits()
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    capture = Capture()
    prev = robot.status().get("recipe")
    try:
        claim_control(robot)
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        robot.release_control()
        time.sleep(0.5)
        if to_arrays(capture.packets[-50:])["tick"].size == 0:
            print("No telemetry with the monitor_baseline fields is arriving on UDP 30003. Nothing recorded.")
            return 2
        capture.packets.clear()
        print(f"Recording for {args.listen:.0f} s. Control is FREE — run your normal programs now "
              f"(dashboard, scripts, RTI). Ctrl+C ends early.")
        end = time.monotonic() + args.listen
        try:
            while time.monotonic() < end:
                time.sleep(1.0)
                left = int(end - time.monotonic())
                if left % 30 == 0:
                    print(f"  {left} s left, {len(capture.packets)} packets")
        except KeyboardInterrupt:
            print("\nEnding early.")
    finally:
        capture.stop()
        try:
            if prev and prev != RECIPE:
                claim_control(robot, seconds=60.0)
                robot.set_recipe(prev)
                robot.release_control()
        except Exception as exc:
            print(f"Could not restore the telemetry recipe to {prev!r} ({exc}). Set it back from the "
                  f"dashboard, or re-run when control is free.")
        robot.disconnect()

    d = to_arrays(capture.packets)
    reports = {}
    for mode in ("EXEC", "JOG", "RTI", "SOFT_RTI"):
        if np.count_nonzero(d["mode"] == mode) < 250:          # under a second of it: nothing to say
            continue
        res = analyse(d, list(range(6)), lo, hi, ceiling, dt, modes=(mode,))
        rec = recommend(res)
        print_report(res, rec, None)
        reports[mode] = {"result": res, "recommended": rec}
    if not reports:
        print("Less than a second in any of EXEC/JOG/RTI/SOFT_RTI — nothing to report.")
    out_dir = os.path.join(_ROOT, "temp_data", "monitor_baseline")
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S") + "_listen"
    np.savez_compressed(os.path.join(out_dir, f"{stamp}.npz"), **d)
    with open(os.path.join(out_dir, f"{stamp}.json"), "w") as fh:
        json.dump(reports, fh, indent=2)
    print(f"\nSaved temp_data/monitor_baseline/{stamp}.npz and .json")
    return 0


def print_report(res, rec, hand):
    print("\n" + "=" * 78)
    print(f"samples {res.get('samples')} ({'/'.join(res.get('modes', []))} {res.get('mode_samples')}, moving {res.get('moving_samples')}, "
          f"holding {res.get('holding_samples')}), dropped packets {res.get('dropped_packets')}, "
          f"impossible packets {res.get('glitch_samples')}")
    f = lambda v, k=2: "-" if v is None else f"{v:.{k}f}"
    print(f"{'':4s} {'track move':>11s} {'p99.9':>7s} {'track hold':>11s} {'cmd spd':>8s} {'raw spd':>8s} "
          f"{'raw/ceil':>8s} {'raw-cmd':>8s} {'hold noise':>10s} {'soft margin':>11s}")
    print(f"{'':4s} {'max deg':>11s} {'deg':>7s} {'max deg':>11s} {'deg/s':>8s} {'deg/s':>8s} {'':>8s} "
          f"{'deg/s':>8s} {'sd deg/s':>10s} {'min deg':>11s}")
    for name, s in (res.get("joints") or {}).items():
        print(f"{name:4s} {f(s['track_move_max_deg'], 3):>11s} {f(s['track_move_p999_deg'], 3):>7s} "
              f"{f(s['track_hold_max_deg'], 3):>11s} {f(s['cmd_speed_max_deg_s'], 1):>8s} "
              f"{f(s['raw_speed_max_deg_s'], 1):>8s} {f(s['raw_speed_max_ratio_of_ceiling'], 2):>8s} "
              f"{f(s['raw_over_command_max_deg_s'], 1):>8s} {f(s['raw_speed_hold_std_deg_s'], 2):>10s} "
              f"{f(s['soft_limit_margin_min_deg'], 1):>11s}")
    print(f"TCP speed max {f(res.get('tcp_speed_max_m_s'), 3)} m/s raw, {f(res.get('tcp_speed_filt_max_m_s'), 3)} "
          f"filtered (wrist-only moves say little about item 5).  Motor temp max: {res.get('motor_temp_max_c')}")
    if hand:
        print(f"Hand-guiding max raw speed (deg/s): {hand}")
    print("\nRecommended starting thresholds (from this one session):")
    for name, r in rec.items():
        print(f"  {name}: tracking {r['tracking_limit_deg']:.1f} deg, overspeed = ceiling x {r['overspeed_margin']:.2f} "
              f"for {r['overspeed_sustain_ticks']} ticks")


if __name__ == "__main__":
    sys.exit(main())
