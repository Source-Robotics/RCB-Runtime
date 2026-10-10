#!/usr/bin/env python3
"""59: does a joint buzz while it holds? Record what the drives report, quiet holds and buzzing ones.

Asked 2026-10-05: J3 sometimes whines (audible) while it holds still after an EXEC move — most likely its PID
gains. Before RTI can warn about it, we need to know whether the buzz is VISIBLE in what RTI receives, and how far
a buzzing hold is from a quiet one. This records it.

What is recorded: the drives' own DATA_PACK replies on can0, with `candump` (read-only — it sends nothing). One
reply per joint per RTI tick (4 ms) carries the encoder position (ticks), the speed (the drive's filtered value)
and the motor current. That current is the drive's instantaneous FOC current at the moment it replies (STEPFOC
firmware: `Data_pack_1_CAN()` packs `FOC.Iq`, computed from the raw current sense in the 6250 Hz loop) — so a
buzz at a few hundred Hz is sampled, not averaged away; at 250 samples/s it shows up folded below 125 Hz.

Per hold and joint it reports:
  I sd      the current's standard deviation over the hold (mA) — includes the slow drift of the integral term
  I hp      the current minus its own 0.25 s moving mean, RMS (mA): the slow drift removed, everything above
            ~2 Hz kept — the number a warning would use
  I fast    RMS of tick-to-tick changes / sqrt(2) (mA): the same for white noise, but blind to a buzz that
            folds to a LOW frequency at 250 samples/s (a 262 Hz whine reads ~12 Hz) — recorded to compare
  pos p2p   encoder ticks between the extremes — is the joint physically moving, or only fighting electrically?
  peak Hz   where most of I hp sits, AS SEEN at 250 samples/s (a whine at f reads |f - k*250|)

Three ways to run it:

    .venv/bin/python api_tests/59_hold_noise.py --listen 5      # record the arm as it is NOW, 5 s. Moves
                                                                 # nothing, takes no control. Use it the
                                                                 # moment you hear the buzz.
    .venv/bin/python api_tests/59_hold_noise.py                 # dry run: prints the plan, moves nothing
    .venv/bin/python api_tests/59_hold_noise.py --go            # the cycles below
    .venv/bin/python api_tests/59_hold_noise.py --freedrive     # you pose the arm by hand, it holds, you listen
    .venv/bin/python api_tests/59_hold_noise.py --selftest      # check the analysis on synthetic data

--go: takes control (without forcing), enters EXEC, and --cycles times moves ONE joint (--joint, default J3) by
+/- --step degrees (default 5) at --speed % (default 20), waits until the move has finished, waits --settle s and
records --hold s of the hold. After each hold it asks whether you heard the buzz (y / n / Enter = not sure), so
quiet and buzzing holds can be compared. At the end it moves the joint back to where it started and leaves the
arm holding in EXEC, control released. Ctrl-C stops the robot. The robot must be ENABLED, homed, in IDLE or EXEC,
without errors. Every record: temp_data/hold_noise/<time>/ (can0.log + report.json);
`--analyse <that folder>` prints it again.

--freedrive (2026-10-06, the user's idea after --go caught no buzz): the arm goes to FREEDRIVE (IDLE with gravity
compensation); you move it by hand, let go and press Enter; it switches to POSITION HOLD (EXEC holds the pose it is
in), and you say whether it buzzes — the hold is recorded from the switch until your answer (at least --hold s).
Then it goes back to freedrive for the next try; `q` finishes (the arm is left in freedrive, control released).
Hands OFF the arm while it holds: a hand on it is recorded as a push (the --go run's hold 4 looked exactly like
that). Needs gravity compensation ON (without it IDLE is a velocity hold, not freedrive) and a terminal.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

RESP_DATA_PACK_1 = 3            # drive -> RTI: position 24 b, speed 24 b, current 16 b, big-endian signed
JOINT_NODES = 6                 # CAN nodes 0..5 = J1..J6
# candump filter: every frame whose command bits (id bits 1..6) are RESP_DATA_PACK_1, from any node, error bit
# either way — the replies only, about half the bus.
CAN_FILTER = f"{RESP_DATA_PACK_1 << 1:03X}:07E"
OUT_DIR = os.path.join(_ROOT, "temp_data", "hold_noise")
LIMIT_CLEARANCE_DEG = 3.0       # refuse a target closer than this to a soft limit
HP_WINDOW = 63                  # ticks (0.25 s): "I hp" = the current minus its own moving mean over this window


# ----------------------------------------------------------------------------- CAN log -> replies
def parse_replies(lines):
    """candump -L lines -> {node: (t, pos_ticks, speed, current_mA)} arrays, DATA_PACK_1 replies of nodes 0..5."""
    rows = {n: [] for n in range(JOINT_NODES)}
    for ln in lines:
        ln = ln.strip()
        if not ln.startswith("("):
            continue
        try:
            ts, _iface, frame = ln.split()
            cid_hex, data_hex = frame.split("#", 1)
            cid = int(cid_hex, 16)
            data = bytes.fromhex(data_hex)
        except ValueError:
            continue
        node, cmd = (cid >> 7) & 0xF, (cid >> 1) & 0x3F
        if cmd != RESP_DATA_PACK_1 or node >= JOINT_NODES or len(data) != 8:
            continue
        rows[node].append((float(ts.strip("()")),
                           int.from_bytes(data[0:3], "big", signed=True),
                           int.from_bytes(data[3:6], "big", signed=True),
                           int.from_bytes(data[6:8], "big", signed=True)))
    out = {}
    for n, r in rows.items():
        a = np.asarray(sorted(r), float).reshape(-1, 4)
        out[n] = (a[:, 0], a[:, 1], a[:, 2], a[:, 3])
    return out


# ----------------------------------------------------------------------------- analysis (pure)
def hold_metrics(t, pos, spd, cur):
    """The numbers for one joint over one hold window (arrays of that joint's replies in the window)."""
    n = t.size
    if n < 20:
        return {"n": int(n)}
    dt = np.median(np.diff(t))
    fast = float(np.sqrt(np.mean(np.diff(cur) ** 2) / 2.0))
    w = min(HP_WINDOW, n // 3 * 2 + 1 if n < 3 * HP_WINDOW else HP_WINDOW)
    hp = cur[w // 2: n - w // 2] - np.convolve(cur, np.ones(w) / w, mode="valid")
    # Where I hp sits: its spectrum, replies treated as evenly spaced at their median interval (RTI ticks are,
    # within jitter).
    d = hp - np.mean(hp)
    spec = np.abs(np.fft.rfft(d * np.hanning(d.size))) ** 2
    freqs = np.fft.rfftfreq(d.size, dt)
    spec[freqs < 1.0] = 0.0
    k = int(np.argmax(spec))
    lo, hi = max(k - 2, 0), k + 3
    share = float(spec[lo:hi].sum() / spec.sum()) if spec.sum() > 0 else 0.0
    return {
        "n": int(n), "rate_hz": round(1.0 / dt, 1),
        "i_mean": round(float(np.mean(cur)), 1), "i_sd": round(float(np.std(cur)), 2),
        "i_hp": round(float(np.sqrt(np.mean(hp ** 2))), 2), "i_fast": round(fast, 2),
        "i_p2p": round(float(np.ptp(cur)), 1),
        "pos_p2p": round(float(np.ptp(pos)), 1), "pos_sd": round(float(np.std(pos)), 2),
        "spd_sd": round(float(np.std(spd)), 2),
        "peak_hz": round(float(freqs[k]), 1), "peak_share": round(share, 2),
    }


def analyse(replies, windows):
    """Every window x every joint. windows: [{"t0", "t1", ...}] (epoch seconds, candump's clock)."""
    out = []
    for w in windows:
        per = {}
        for n, (t, p, s, c) in replies.items():
            m = (t >= w["t0"]) & (t < w["t1"])
            per[f"J{n + 1}"] = hold_metrics(t[m], p[m], s[m], c[m])
        out.append({**w, "joints": per})
    return out


def separation(results, joint_name):
    """I hp of `joint_name` on the holds labelled buzz vs quiet: (quiet list, buzz list, suggested threshold or
    None). The threshold sits geometrically between the loudest quiet and the quietest buzzing hold — only when
    every buzzing hold is above every quiet one."""
    quiet = [r["joints"][joint_name].get("i_hp") for r in results if r.get("buzz") is False]
    buzz = [r["joints"][joint_name].get("i_hp") for r in results if r.get("buzz") is True]
    quiet = [v for v in quiet if v is not None]
    buzz = [v for v in buzz if v is not None]
    thr = None
    if quiet and buzz and min(buzz) > max(quiet):
        thr = round(float(np.sqrt(max(quiet) * min(buzz))), 1)
    return quiet, buzz, thr


def print_results(results, focus=None):
    for r in results:
        lab = {True: "BUZZ heard", False: "quiet", None: "not labelled"}[r.get("buzz")]
        print(f"\n{r['name']}  ({r['t1'] - r['t0']:.1f} s, {lab})")
        print("  joint  replies   I mean    I sd    I hp  I fast   I p2p  pos p2p  spd sd  peak Hz (share)")
        for j, m in r["joints"].items():
            if m.get("n", 0) < 20:
                print(f"  {j:5s}  {m.get('n', 0):7d}   (too few replies)")
                continue
            star = "  <" if focus == j else ""
            print(f"  {j:5s}  {m['n']:7d}  {m['i_mean']:7.1f}  {m['i_sd']:6.2f}  {m['i_hp']:6.2f}  {m['i_fast']:6.2f}  "
                  f"{m['i_p2p']:6.1f}  {m['pos_p2p']:7.1f}  {m['spd_sd']:6.2f}  {m['peak_hz']:6.1f} "
                  f"({m['peak_share']:.2f}){star}")
    if focus:
        quiet, buzz, thr = separation(results, focus)
        if quiet or buzz:
            print(f"\n{focus} I hp (mA): quiet holds {quiet}, buzzing holds {buzz}")
            if thr is not None:
                print(f"  -> they separate: a warning threshold near {thr} mA would tell them apart "
                      f"(x{min(buzz) / max(max(quiet), 1e-9):.1f} between the closest pair).")
            elif quiet and buzz:
                print("  -> they OVERLAP on this number: paste the whole printout, another measure is needed.")


# ----------------------------------------------------------------------------- selftest
def selftest():
    fails = []

    def chk(name, cond, detail=""):
        print(("  PASS  " if cond else "  FAIL  ") + name + (f"   [{detail}]" if detail and not cond else ""))
        if not cond:
            fails.append(name)

    rng = np.random.default_rng(1)
    # Reply times as MEASURED on the arm (2026-10-06 recording): 4 ms ticks on an absolute grid (RTI paces to
    # deadlines), interval sd 0.21 ms -> ~0.15 ms of jitter per reply.
    t = 1000.0 + 0.004 * np.arange(1500) + rng.normal(0, 0.00015, 1500)
    drift = np.linspace(0, 40, t.size)                               # the integral term settling: 40 mA
    quiet = -600 + drift + rng.normal(0, 4, t.size)
    buzz = quiet + 80 * np.sin(2 * np.pi * 262.0 * (t - t[0]))       # a 262 Hz whine, 80 mA
    pos = np.round(rng.normal(0, 0.3, t.size))
    mq = hold_metrics(t, pos, pos, quiet)
    mb = hold_metrics(t, pos, pos, buzz)
    chk("quiet: I fast reads the noise (4 mA), not the 40 mA drift", 3.0 < mq["i_fast"] < 5.5, mq["i_fast"])
    chk("quiet: I sd DOES include the drift (why it is not the number)", mq["i_sd"] > 10, mq["i_sd"])
    chk("quiet: I hp reads the noise too (the drift removed)", 3.0 < mq["i_hp"] < 5.5, mq["i_hp"])
    chk("buzz: a 262 Hz whine sampled at 250 Hz is seen in I hp (>> quiet; 80 mA sine = 57 RMS)",
        mb["i_hp"] > 8 * mq["i_hp"], (mq["i_hp"], mb["i_hp"]))
    chk("buzz: ...while I fast sees much less of it (it folds to a LOW frequency) — why I hp is the number",
        mb["i_fast"] < 0.6 * mb["i_hp"], (mb["i_fast"], mb["i_hp"]))
    chk("buzz: its peak reads folded, near 12 Hz (262 - 250)", abs(mb["peak_hz"] - 12.0) < 2.5, mb["peak_hz"])
    m501 = hold_metrics(t, pos, pos, quiet + 80 * np.sin(2 * np.pi * 501.0 * (t - t[0])))
    chk("buzz near 2 x 250 Hz (folds to ~1 Hz): still seen in I hp (the replies' small jitter spreads it) — but "
        "only about half of it", 4 * mq["i_hp"] < m501["i_hp"] < 0.6 * mb["i_hp"], (m501["i_hp"], mb["i_hp"]))
    te = 1000.0 + 0.004 * np.arange(1500)                            # perfectly even sampling: the blind spot
    m501e = hold_metrics(te, pos, pos, -600 + rng.normal(0, 4, te.size) + 80 * np.sin(2 * np.pi * 501.0 * (te - te[0])))
    chk("(documented blind spot) with PERFECTLY even sampling the same 501 Hz hides from I hp, not from I sd",
        m501e["i_hp"] < 3 * mq["i_hp"] and m501e["i_sd"] > 40, (m501e["i_hp"], m501e["i_sd"]))
    chk("too few replies: no numbers", hold_metrics(t[:10], pos[:10], pos[:10], quiet[:10]) == {"n": 10})

    # candump round trip: node 2 (J3), DATA_PACK_1 reply, error bit set; a command frame and the gripper ignored.
    def frame(node, cmd, err, p, s, c, ts):
        cid = (node << 7) | (cmd << 1) | err
        data = p.to_bytes(3, "big", signed=True) + s.to_bytes(3, "big", signed=True) + c.to_bytes(2, "big", signed=True)
        return f"({ts:.6f}) can0 {cid:03X}#{data.hex().upper()}"
    lines = [frame(2, 3, 1, -150, -187, -3047, 5.0), frame(2, 2, 0, 1, 2, 3, 5.001), frame(6, 3, 0, 9, 9, 9, 5.002),
             frame(0, 3, 0, 8388607, -8388608, 32767, 5.003)]
    r = parse_replies(lines)
    chk("candump: J3's reply decoded (signed position, speed, current)",
        r[2][0].tolist() == [5.0] and (r[2][1][0], r[2][2][0], r[2][3][0]) == (-150, -187, -3047), r[2])
    chk("candump: command frames and the gripper node are not replies", r[2][0].size == 1 and 6 not in r)
    chk("candump: 24/16-bit extremes", (r[0][1][0], r[0][2][0], r[0][3][0]) == (8388607, -8388608, 32767), r[0])
    chk("the candump filter selects command id 3", CAN_FILTER == "006:07E", CAN_FILTER)

    res = [{"name": f"h{i}", "t0": 0, "t1": 1, "buzz": b, "joints": {"J3": {"i_hp": v}}}
           for i, (b, v) in enumerate([(False, 4.0), (False, 6.0), (True, 40.0), (True, 55.0), (None, 30.0)])]
    q, b, thr = separation(res, "J3")
    chk("separation: labelled holds only; threshold between max quiet and min buzz", q == [4.0, 6.0]
        and b == [40.0, 55.0] and thr is not None and 6.0 < thr < 40.0, (q, b, thr))
    res[1]["joints"]["J3"]["i_hp"] = 45.0
    chk("separation: overlap -> no threshold", separation(res, "J3")[2] is None)
    print(f"\n{len(fails)} failed" + (f": {fails}" if fails else " - all passed"))
    return 1 if fails else 0


# ----------------------------------------------------------------------------- recording
def start_candump(out_dir, iface):
    os.makedirs(out_dir, exist_ok=True)
    fh = open(os.path.join(out_dir, "can0.log"), "w")
    proc = subprocess.Popen(["candump", "-L", f"{iface},{CAN_FILTER}"], stdout=fh, stderr=subprocess.DEVNULL)
    time.sleep(0.3)
    return proc, fh


def stop_candump(proc, fh):
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
    fh.close()


def ask_buzz(prompt):
    if not sys.stdin.isatty():
        return None
    a = input(prompt).strip().lower()
    return True if a.startswith("y") else False if a.startswith("n") else None


def finish(out_dir, windows, meta, focus):
    with open(os.path.join(out_dir, "can0.log")) as fh:
        replies = parse_replies(fh)
    results = analyse(replies, windows)
    with open(os.path.join(out_dir, "report.json"), "w") as fh:
        json.dump({"meta": meta, "windows": windows, "results": results}, fh, indent=2)
    print_results(results, focus)
    print(f"\nSaved {os.path.relpath(out_dir, _ROOT)}/ (can0.log, report.json)")


def listen_session(args):
    """Record the arm as it is, nothing else."""
    out_dir = os.path.join(OUT_DIR, time.strftime("%Y%m%d_%H%M%S") + "_listen")
    proc, fh = start_candump(out_dir, args.iface)
    print(f"Recording {args.listen:.0f} s of what the drives report (nothing moves)...")
    t0 = time.time()
    try:
        time.sleep(args.listen)
    finally:
        t1 = time.time()
        stop_candump(proc, fh)
    buzz = ask_buzz("Was a joint buzzing during that? [y/n, Enter = not sure] ")
    focus = None
    if buzz:
        j = input("Which joint (1-6, Enter = J3)? ").strip() if sys.stdin.isatty() else ""
        focus = f"J{int(j) if j.isdigit() and 1 <= int(j) <= 6 else 3}"
    windows = [{"name": "listen", "t0": t0, "t1": t1, "buzz": buzz}]
    finish(out_dir, windows, {"kind": "listen", "focus": focus}, focus)
    return 0


def not_ready(st, need_gravity=False):
    """Why the robot cannot start a session, or None."""
    if (st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") not in ("IDLE", "EXEC")
            or st.get("motion_aborted") or st.get("err")):
        return (f"Robot must be ENABLED + homed, in IDLE or EXEC, without errors (got mode={st.get('mode')} "
                f"state={st.get('state')} homed={st.get('homed')} errors={st.get('errors') or st.get('err')} "
                f"aborted={st.get('motion_aborted')}).")
    if need_gravity and not st.get("gravity_comp"):
        return "Gravity compensation is OFF, so IDLE is a velocity hold, not freedrive — switch it on first."
    return None


def freedrive_session(args):
    """You pose the arm in freedrive; it switches to position hold; you say whether it buzzes."""
    from robotclient import RobotClient
    base = __import__("46_monitor_baseline")
    if not sys.stdin.isatty():
        print("--freedrive asks you questions: run it in a terminal.")
        return 2
    if shutil.which("candump") is None:
        print("candump not found (apt package can-utils).")
        return 2
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed = False
    proc = fh = None
    windows = []
    out_dir = os.path.join(OUT_DIR, time.strftime("%Y%m%d_%H%M%S") + "_freedrive")
    try:
        why = not_ready(robot.status(), need_gravity=True)
        if why:
            print(why + " Nothing changed.")
            return 2
        a = input("The arm goes into FREEDRIVE (soft, it floats) now. Hand on the arm, then Enter (q = quit): ")
        if a.strip().lower().startswith("q"):
            return 0
        proc, fh = start_candump(out_dir, args.iface)
        base.claim_control(robot)
        claimed = True
        robot.enter_idle_mode(wait=True)
        n = 0
        while True:
            a = input("\nFREEDRIVE: move the arm by hand, LET GO, then Enter = position hold (q = finish): ")
            if a.strip().lower().startswith("q"):
                break
            robot.enter_exec_mode(wait=True)
            t_hold = time.time()
            n += 1
            q = [round(float(x), 2) for x in robot.get_joints()]
            time.sleep(args.settle)
            t0 = time.time()
            buzz = ask_buzz(f"  HOLDING (hands off) at J1-J6 {q} deg. Listen — buzzing? [y/n, Enter = not sure] ")
            t1 = max(time.time(), t0 + args.hold)
            time.sleep(max(0.0, t1 - time.time()))
            windows.append({"name": f"hold {n} (J3 {q[2]:.2f} deg)", "t0": t0, "t1": t1, "buzz": buzz,
                            "t_hold": t_hold, "joints_deg": q})
            print("  back to FREEDRIVE — hand on the arm")
            robot.enter_idle_mode(wait=True)
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the robot.")
        try:
            robot.stop()
        except Exception:
            pass
    finally:
        if proc is not None:
            stop_candump(proc, fh)
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()
    if windows:
        finish(out_dir, windows, {"kind": "freedrive", "settle": args.settle, "hold": args.hold, "focus": "J3"}, "J3")
    return 0


def go_session(args):
    from robotclient import RobotClient
    base = __import__("46_monitor_baseline")                   # claim_control (waits for the dashboard's release)
    j0 = args.joint - 1
    lo, hi = (np.degrees(v) for v in base._limits()[:2])
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed = False
    proc = fh = None
    windows = []
    out_dir = os.path.join(OUT_DIR, time.strftime("%Y%m%d_%H%M%S") + f"_J{args.joint}")
    try:
        st = robot.status()
        q0 = np.asarray(robot.get_joints(), float)
        targets = []
        for c in range(args.cycles):
            q = q0.copy()
            q[j0] = q0[j0] + (args.step if c % 2 == 0 else -args.step)
            targets.append(q)
        far = [q0[j0] + args.step, q0[j0] - args.step]
        print(f"J{args.joint} now {q0[j0]:.2f} deg (soft limits {lo[j0]:.1f} .. {hi[j0]:.1f}).")
        print(f"Plan: {args.cycles} EXEC moves of J{args.joint} to {far[0]:.2f} / {far[1]:.2f} deg alternately at "
              f"{args.speed:.0f} %, each followed by {args.settle:.1f} s settle + {args.hold:.1f} s recorded hold; "
              f"then back to {q0[j0]:.2f}. Only J{args.joint} moves.")
        if min(far) < lo[j0] + LIMIT_CLEARANCE_DEG or max(far) > hi[j0] - LIMIT_CLEARANCE_DEG:
            print(f"A target is within {LIMIT_CLEARANCE_DEG} deg of a soft limit — move J{args.joint} away first or "
                  f"use a smaller --step. Nothing moved.")
            return 2
        if not args.go:
            print("DRY RUN: re-run with --go to execute. Nothing moved.")
            return 0
        if shutil.which("candump") is None:
            print("candump not found (apt package can-utils). Nothing moved.")
            return 2
        why = not_ready(st)
        if why:
            print(why + " Nothing moved.")
            return 2

        proc, fh = start_candump(out_dir, args.iface)
        base.claim_control(robot)
        claimed = True
        if st.get("mode") != "EXEC":
            robot.enter_exec_mode(wait=True)
        for c, q in enumerate(targets):
            robot.move_joints(*[float(x) for x in q], speed=args.speed)
            fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
            if fails:
                print(f"Move failed: {fails}")
                break
            time.sleep(args.settle)
            t0 = time.time()
            print(f"  hold {c + 1}/{args.cycles} at J{args.joint} = {q[j0]:.2f} deg — recording {args.hold:.1f} s...")
            time.sleep(args.hold)
            t1 = time.time()
            buzz = ask_buzz("    buzz heard? [y/n, Enter = not sure] ")
            windows.append({"name": f"hold {c + 1} (J{args.joint} {q[j0]:.2f} deg)", "t0": t0, "t1": t1,
                            "buzz": buzz, "target_deg": float(q[j0])})
        robot.move_joints(*[float(x) for x in q0], speed=args.speed)
        robot.wait_idle(timeout=60.0, raise_on_error=False)
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the robot.")
        try:
            robot.stop()
        except Exception:
            pass
    finally:
        if proc is not None:
            stop_candump(proc, fh)
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()
    if windows:
        meta = {"kind": "go", "joint": args.joint, "step": args.step, "speed": args.speed, "settle": args.settle,
                "hold": args.hold, "focus": f"J{args.joint}"}
        finish(out_dir, windows, meta, f"J{args.joint}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--freedrive", action="store_true", help="you pose the arm by hand; it holds; you listen")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--listen", type=float, default=None, metavar="SECONDS", help="record now, move nothing")
    ap.add_argument("--analyse", default=None, metavar="DIR", help="print a saved record again")
    ap.add_argument("--joint", type=int, default=3, choices=range(1, 7))
    ap.add_argument("--step", type=float, default=5.0, help="degrees each way (default 5)")
    ap.add_argument("--speed", type=float, default=20.0, help="EXEC speed percent (default 20)")
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--settle", type=float, default=0.3, help="seconds after the move before recording")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds recorded per hold (--freedrive: at least)")
    ap.add_argument("--iface", default="can0")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.analyse:
        with open(os.path.join(args.analyse, "report.json")) as fh:
            rep = json.load(fh)
        with open(os.path.join(args.analyse, "can0.log")) as fh:
            print_results(analyse(parse_replies(fh), rep["windows"]), rep["meta"].get("focus"))
        return 0
    if args.speed <= 1.0:
        print("--speed is a percent (20 = 20 %); a value <= 1 would be read as a fraction. Nothing moved.")
        return 2
    if args.freedrive:
        return freedrive_session(args)
    if args.listen is not None:
        if shutil.which("candump") is None:
            print("candump not found (apt package can-utils).")
            return 2
        return listen_session(args)
    return go_session(args)


if __name__ == "__main__":
    sys.exit(main())
