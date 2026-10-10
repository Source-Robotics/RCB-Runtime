#!/usr/bin/env python3
"""66: how well each joint TRACKS its commanded motion — and how two control setups compare (2026-10-10).

TODO/dynamics_feedforward_plan.md, steps T1 + T2. The same EXEC moves are run with each gain set in turn, recorded tick by
tick, and compared per joint and speed. Step T2: the PID tune before 2026-10-09 ("old", read from the "Was: KPP ..."
comments in robots/PAR6.xml) against the current one ("new", the XML values). Later the same tool A/B-tests the dynamics
feedforward (F1-F3) — `--compare` puts two saved runs side by side.

WHAT IS MEASURED, per move and joint (error = commanded - measured position, positive = the joint lags behind):
  rms / peak        tracking error while the command moves
  acc / cru / dec   RMS error while the command accelerates / cruises / decelerates (from the commanded acceleration)
  lag               mean error while cruising, signed along the motion (the steady lag)
  overshoot         how far the joint goes PAST the target after the command stops
  settle            ms after the command stops until |error| stays under 0.05 deg
  pid               RMS of measured torque - commanded torque while moving: the torque the drive's own loops had to add
                    beyond our feedforward (gravity today) — what a dynamics feedforward would take over
  others            the largest error on the joints that were NOT commanded to move (coupling)
  delay             the time shift (0-20 ms) that best lines the measured motion up with the command
  rms_d / peak_d    the error LEFT after that shift — the path error. The plain error is dominated by a constant
                    ~6 ms (first run 2026-10-10: lag / speed = 6.1 ms on every joint, both gain sets): the measured
                    position is ~2.4 ms old CAN data and the command reaches the drive at the end of the tick. A pure
                    delay (and any lag proportional to speed) only shifts the motion in time; what remains — error
                    while accelerating, friction at reversals, ripple, overshoot — is what control changes can improve.
Each speed's value is the median of its four moves (out, back, out the other way, back).

WHY TELEMETRY. Shared memory sampled from Python pairs values from different ticks (TODO/servo_mode_options.md §15.4);
the robot's UDP state packet is built inside one tick. The script switches the recipe to `tracking` for the run.

MOTION. Around the arm's CURRENT pose — put it somewhere free first. Each joint alone (default J1 +/-20, J2 +/-10,
J3 +/-15, J4 +/-30, J5 +/-30, J6 +/-45 deg, kept 10 deg inside the soft limits): out, back, out the other way, back, at
10 / 30 / 60 / 100 % of its EXEC limits; then all joints together at half those amplitudes at 30 and 60 %. The planner
refuses anything outside the joint limits, zones or the self-collision margin, and a refused or failed move ends the
session. Gains are switched with set_pid_gains (KPP, KPV, KIV, KIIQ, KPIQ — the current and velocity LIMITS stay
the XML's), and the XML gains are ALWAYS put back at the end, whatever happens. Ctrl+C stops the robot.
~3 minutes per gain set. The robot must be ENABLED and homed; the script enters EXEC (no motion) and takes control.

    python3 api_tests/66_tracking.py --selftest                 # the analysis on synthetic data with known answers
    python3 api_tests/66_tracking.py                            # dry run: prints the plan and both gain tables
    python3 api_tests/66_tracking.py --go                       # new, old, new (the 2nd new = repeatability)
    python3 api_tests/66_tracking.py --go --sets new --joints 4,5,6 --speeds 30,60
    python3 api_tests/66_tracking.py --compare temp_data/tracking/A.json temp_data/tracking/B.json
    python3 api_tests/66_tracking.py --reanalyse temp_data/tracking/A.json   # recompute a saved run (no robot)

Results: printed, and saved to temp_data/tracking/<time>.npz (raw ticks) + .json (every metric).
"""
import argparse
import json
import os
import re
import socket
import sys
import threading
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
from robotclient.rti_client import parse_state_packet                 # noqa: E402

RECIPE = "tracking"
TELEMETRY_PORT = 30003            # config/networking.py TELEMETRY_PORT (the import would pull in zmq setup)
GAIN_KEYS = ("KPP", "KPV", "KIV", "KIIQ", "KPIQ")
DEFAULT_AMP = {1: 20.0, 2: 10.0, 3: 15.0, 4: 30.0, 5: 30.0, 6: 45.0}
DEFAULT_SPEEDS = (10.0, 30.0, 60.0, 100.0)
ALL_JOINT_SPEEDS = (30.0, 60.0)
CLEARANCE_DEG = 10.0
HOLD_S = 0.6                      # recorded after every move: overshoot, settling, final error
MOVING = 1e-3                     # rad/s — a commanded speed above this is "moving"
ACC_FRACTION = 0.1                # |commanded accel| above this x the move's peak counts as accelerating / decelerating
SETTLE_DEG = 0.05
FINAL_S = 0.1
NEED = ("j_p", "j_pc", "j_s", "j_sc", "j_ac", "j_t", "j_tc", "tick")
DELAY_MAX_S, DELAY_STEP_S = 0.020, 0.00025   # the time shift searched for each move
METRICS = ("rms", "peak", "acc", "cru", "dec", "lag", "overshoot", "settle_ms", "final", "pid", "others", "vpeak",
           "delay_ms", "rms_d", "peak_d")


# ----------------------------------------------------------------------------- gains
def read_gain_sets(xml_path, joint_num=6):
    """{"new": [per-joint {KPP..KPIQ}], "old": [...]} — new = the XML values, old = the "Was: KPP a, KPV b, KIV c,
    KIIQ d, KPIQ e" comment written above each joint's gains on 2026-10-09 (the tune before it)."""
    import xml.etree.ElementTree as ET
    text = open(xml_path).read()
    old = [dict(zip(GAIN_KEYS, map(float, m)))
           for m in re.findall(r"Was: KPP ([\d.]+), KPV ([\d.]+), KIV ([\d.]+), KIIQ ([\d.]+), KPIQ ([\d.]+)", text)]
    joints = ET.parse(xml_path).getroot().findall(".//joint")
    new = [{k: float(j.findtext(k)) for k in GAIN_KEYS} for j in joints if j.findtext("KPP") is not None]
    if len(old) < joint_num or len(new) < joint_num:
        raise RuntimeError(f"found {len(old)} old and {len(new)} new gain sets in {xml_path}, need {joint_num}")
    return {"new": new[:joint_num], "old": old[:joint_num]}


def apply_gains(robot, gains):
    for j, g in enumerate(gains):
        robot.set_pid_gains(j, **g)
    time.sleep(0.3)                                       # RTI reloads each joint's config on its tick


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

    def last_tick(self):
        """The newest tick received (scans back past packets of another recipe)."""
        for raw in reversed(self.packets[-50:]):
            try:
                r = parse_state_packet(raw.decode())
            except Exception:
                continue
            if "tick" in r and "j_ac" in r:
                return int(r["tick"])
        return None

    def stop(self):
        self._run = False
        self._t.join(timeout=1.0)
        self._sock.close()


def to_arrays(packets, joint_num=6):
    """Raw packets -> arrays ordered by tick, duplicates and packets of another recipe dropped."""
    rows = {}
    for raw in packets:
        try:
            r = parse_state_packet(raw.decode() if isinstance(raw, bytes) else raw)
        except Exception:
            continue
        if all(k in r for k in NEED):
            rows[int(r["tick"])] = r
    ticks = sorted(rows)
    out = {"tick": np.array(ticks, dtype=np.int64),
           "mode": np.array([str(rows[t].get("mode", "")) for t in ticks])}
    for k in NEED[:-1]:
        out[k] = np.array([np.resize(np.asarray(rows[t][k], float), joint_num) for t in ticks]) if ticks \
            else np.zeros((0, joint_num))
    return out


# ----------------------------------------------------------------------------- analysis (pure)
def segment(d, t0, t1):
    m = (d["tick"] >= t0) & (d["tick"] <= t1)
    return {k: v[m] for k, v in d.items()}


def move_metrics(s, j, direction, dt):
    """Metrics of joint j (0-based) in one move's segment. direction: +1 / -1, the sign of target - start."""
    sc, ac = s["j_sc"][:, j], s["j_ac"][:, j]
    moving = np.abs(sc) > MOVING
    if not moving.any():
        return None
    err = np.degrees(s["j_pc"][:, j] - s["j_p"][:, j])
    along = err * direction                                   # > 0: behind the command
    a_ref = np.abs(ac[moving]).max()
    signed_acc = ac * np.sign(sc)
    acc = moving & (signed_acc > ACC_FRACTION * a_ref)
    dec = moving & (signed_acc < -ACC_FRACTION * a_ref)
    cru = moving & ~acc & ~dec
    rms = lambda m: float(np.sqrt(np.mean(err[m] ** 2))) if m.any() else None
    end = int(np.flatnonzero(moving)[-1])
    post = slice(end + 1, None)
    over = float(max(0.0, (-along[post]).max())) if end + 1 < err.size else 0.0
    settle = None
    if end + 1 < err.size:
        inside = np.abs(err[post]) < SETTLE_DEG
        bad = np.flatnonzero(~inside)
        if inside.size and inside[-1]:
            first_ok = 0 if bad.size == 0 else int(bad[-1]) + 1
            settle = 1000.0 * (first_ok + 1) * dt
    nfin = max(1, int(round(FINAL_S / dt)))
    others = [k for k in range(s["j_p"].shape[1]) if k != j and not (np.abs(s["j_sc"][:, k]) > MOVING).any()]
    oth = float(np.degrees(np.abs(s["j_pc"][moving][:, others] - s["j_p"][moving][:, others])).max()) if others else None
    pid = s["j_t"][:, j] - s["j_tc"][:, j]
    delay, res = best_delay(s["tick"] * dt, s["j_pc"][:, j], s["j_p"][:, j], moving)
    return {"delay_ms": 1000.0 * delay, "rms_d": float(np.sqrt(np.mean(res ** 2))), "peak_d": float(np.abs(res).max()),
            "rms": rms(moving), "peak": float(np.abs(err[moving]).max()), "acc": rms(acc), "cru": rms(cru),
            "dec": rms(dec), "lag": float(along[cru].mean()) if cru.any() else None, "overshoot": over,
            "settle_ms": settle, "final": float(err[-nfin:].mean()), "pid": float(np.sqrt(np.mean(pid[moving] ** 2))),
            "others": oth, "vpeak": float(np.degrees(np.abs(sc).max()))}


def best_delay(t, cmd, meas, mask):
    """The shift D (0..DELAY_MAX_S) minimising the RMS of cmd(t - D) - meas(t) over `mask`, and that residual (deg)."""
    tm = t[mask]
    best = (np.inf, 0.0, None)
    for D in np.arange(0.0, DELAY_MAX_S + 1e-12, DELAY_STEP_S):
        r = np.degrees(np.interp(tm - D, t, cmd) - meas[mask])
        e = float(np.mean(r ** 2))
        if e < best[0]:
            best = (e, float(D), r)
    return best[1], best[2]


def analyse(d, records, dt):
    """records: [{set, joint (1-based) or 'all', speed, t0, t1, start (deg), target (deg)}] -> per-move metrics."""
    out = []
    for r in records:
        s = segment(d, r["t0"], r["t1"])
        if s["tick"].size == 0:
            continue
        joints = range(len(r["start"])) if r["joint"] == "all" else [int(r["joint"]) - 1]
        for j in joints:
            dirn = float(np.sign(r["target"][j] - r["start"][j]))
            if dirn == 0:
                continue
            m = move_metrics(s, j, dirn, dt)
            if m is not None:
                out.append({"set": r["set"], "move": r["label"], "group": "all" if r["joint"] == "all" else "alone",
                            "joint": j + 1, "speed": r["speed"], **m})
    return out


def summarise(moves):
    """{set: {"J3": {"alone": {"30": {metric: median}}, "all": {...}}}}"""
    summ = {}
    for m in moves:
        b = summ.setdefault(m["set"], {}).setdefault(f"J{m['joint']}", {}).setdefault(m["group"], {}) \
            .setdefault(f"{m['speed']:g}", [])
        b.append(m)
    for st in summ.values():
        for jt in st.values():
            for grp in jt.values():
                for sp, lst in list(grp.items()):
                    grp[sp] = {k: (float(np.median([x[k] for x in lst if x[k] is not None]))
                                   if any(x[k] is not None for x in lst) else None) for k in METRICS}
                    grp[sp]["n"] = len(lst)
    return summ


# ----------------------------------------------------------------------------- report
def _fmt(v, nd=3):
    return "    -  " if v is None else f"{v:7.{nd}f}"


def print_report(summ, set_names, title=""):
    if title:
        print(title)
    cols = [("rms", 3), ("delay_ms", 1), ("rms_d", 3), ("peak_d", 3), ("overshoot", 3), ("settle_ms", 0), ("pid", 3),
            ("others", 3)]
    head = "  ".join(f"{c[0][:7]:>7s}" for c in cols)
    print("  (degrees, ms, Nm; median of the moves at that speed; error = commanded - measured;"
          " rms_d / peak_d = after the best time shift 'delay')")
    joints = sorted({j for s in set_names for j in summ.get(s, {})}, key=lambda x: int(x[1:]))
    for jn in joints:
        for grp in ("alone", "all"):
            speeds = sorted({sp for s in set_names for sp in summ.get(s, {}).get(jn, {}).get(grp, {})}, key=float)
            if not speeds:
                continue
            print(f"\n  {jn} ({'moving alone' if grp == 'alone' else 'all joints together'})")
            print(f"    {'speed':>6s} {'set':>8s} {'vpeak':>7s}  {head}")
            for sp in speeds:
                for s in set_names:
                    v = summ.get(s, {}).get(jn, {}).get(grp, {}).get(sp)
                    if v is None:
                        continue
                    print(f"    {sp + '%':>6s} {s:>8s} {_fmt(v['vpeak'], 1)}  " + "  ".join(_fmt(v[c], nd) for c, nd in cols))
    # one line per joint and metric: the mean over speeds, each set relative to the first
    if len(set_names) > 1:
        for key, what in (("rms", "RMS tracking error"), ("rms_d", "RMS path error after the time shift")):
            print(f"\n  Mean {what} over all speeds (joint moving alone), relative to the first set:")
            for jn in joints:
                vals = []
                for s in set_names:
                    g = summ.get(s, {}).get(jn, {}).get("alone", {})
                    xs = [g[sp][key] for sp in g if g[sp].get(key) is not None]
                    vals.append(float(np.mean(xs)) if xs else None)
                if vals[0]:
                    print(f"    {jn}: " + "   ".join(f"{s} {v:.3f}" + ("" if i == 0 or v is None else f" ({v / vals[0]:.2f}x)")
                                                     for i, (s, v) in enumerate(zip(set_names, vals)) if v is not None))


def label_sets(names):
    """'new','old','new' -> 'new','old','new2' (each run keeps its own name)."""
    seen, out = {}, []
    for n in names:
        seen[n] = seen.get(n, 0) + 1
        out.append(n if seen[n] == 1 else f"{n}{seen[n]}")
    return out


# ----------------------------------------------------------------------------- self-test
def selftest():
    """Synthetic moves with KNOWN tracking behaviour, through the real packet parser and the analysis."""
    dt = 0.004
    ok_all = True

    def chk(name, cond, detail=""):
        nonlocal ok_all
        ok_all &= bool(cond)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail and not cond else ""))

    def synth(dist, dec_extra, post):
        """J3 moves `dist` deg (sign = direction) on a trapezoid (100 deg/s^2, 40 deg/s), lagging 0.004 deg per deg/s
        along the motion, `dec_extra` deg MORE while decelerating; then 1 s still with the error sequence `post`
        (deg, along the motion: negative = past the target) and 0 after it. Returns cmd, meas, vel, acc (deg units)."""
        a, v, D = 100.0, 40.0, abs(dist)
        ta = v / a
        tc = (D - a * ta * ta) / v
        T = 2 * ta + tc
        t = np.arange(0, T + 1.0, dt)
        pos = np.where(t < ta, 0.5 * a * t ** 2, np.where(t < ta + tc, 0.5 * a * ta ** 2 + v * (t - ta),
                       np.where(t < T, D - 0.5 * a * (T - t) ** 2, D)))
        vel = np.where(t < ta, a * t, np.where(t < ta + tc, v, np.where(t < T, a * (T - t), 0.0)))
        acc = np.where(t < ta, a, np.where(t < ta + tc, 0.0, np.where(t < T, -a, 0.0)))
        along = 0.004 * vel + np.where((t >= ta + tc) & (t < T), dec_extra, 0.0)
        k0 = int(np.flatnonzero(vel > 0)[-1]) + 1                  # the first still sample
        tail = np.zeros(t.size - k0)
        tail[:len(post)] = post
        along[k0:] = tail
        sg = np.sign(dist)
        return sg * pos, sg * (pos - along), sg * vel, sg * acc

    # move 1: J3 +20 deg, the error decaying past the target (0.08 deg, tau 50 ms); move 2: J3 back -20 deg with
    # 0.05 deg extra lag only while DECELERATING and a RINGING settle: 0.08 x2, 0.03 x5 (inside), 0.07 x5, then 0
    tt = np.arange(0, 1.0, dt)
    c1, m1, v1, a1 = synth(+20.0, 0.0, -0.08 * np.exp(-tt / 0.05))
    c2, m2, v2, a2 = synth(-20.0, 0.05, -np.array([0.08] * 2 + [0.03] * 5 + [0.07] * 5))
    cmd = np.concatenate([c1, 20.0 + c2]); meas = np.concatenate([m1, 20.0 + m2])
    vel = np.concatenate([v1, v2]); acc = np.concatenate([a1, a2])
    n, n1 = cmd.size, c1.size
    jp = np.zeros((n, 6)); jpc = np.zeros((n, 6)); js = np.zeros((n, 6)); jsc = np.zeros((n, 6)); jac = np.zeros((n, 6))
    jp[:, 2], jpc[:, 2] = np.radians(meas), np.radians(cmd)
    jsc[:, 2], js[:, 2], jac[:, 2] = np.radians(vel), np.radians(vel), np.radians(acc)
    jp[:, 3] = np.radians(0.02)                            # J4 not commanded, sits 0.02 deg off: the coupling figure
    jt = np.zeros((n, 6)); jtc = np.zeros((n, 6))
    jtc[:, 2] = 2.0
    jt[:, 2] = 2.0 + 0.3 * np.sign(vel)                    # the drive adds 0.3 Nm along the motion while moving
    f = lambda arr, k=5: ",".join(f"{x:.{k}f}" for x in arr)
    pkts = [(f"j_p:{f(jp[i])},j_pc:{f(jpc[i])},j_s:{f(js[i], 4)},j_sc:{f(jsc[i], 4)},j_ac:{f(jac[i], 3)},"
             f"j_t:{f(jt[i], 3)},j_tc:{f(jtc[i], 3)},tick:{5000 + i},mode:EXEC").encode() for i in range(n)]
    pkts.append(b"j_p:0,0,0,0,0,0,tick:5001,mode:EXEC")    # another recipe's packet: dropped
    d = to_arrays(pkts[::-1])
    chk("packets parsed, ordered, foreign recipe dropped", d["tick"].size == n and d["tick"][0] == 5000, d["tick"].size)
    z = [0.0] * 6
    rec = [{"set": "A", "label": "J3 +", "joint": 3, "speed": 30.0, "t0": 5000, "t1": 5000 + n1 - 1,
            "start": z, "target": [0, 0, 20.0, 0, 0, 0]},
           {"set": "A", "label": "J3 back", "joint": 3, "speed": 30.0, "t0": 5000 + n1, "t1": 5000 + n - 1,
            "start": [0, 0, 20.0, 0, 0, 0], "target": z}]
    m = analyse(d, rec, dt)
    chk("two moves found for J3", len(m) == 2 and all(x["joint"] == 3 for x in m), len(m))
    x, y = m[0], m[1]
    chk("cruise lag = +0.16 deg on the way out (0.004 x 40 deg/s)", abs(x["lag"] - 0.16) < 0.002, x["lag"])
    chk("cruise lag = +0.16 deg on the way BACK too (signed along the motion)", abs(y["lag"] - 0.16) < 0.002, y["lag"])
    chk("peak error = 0.16 deg", abs(x["peak"] - 0.16) < 0.002, x["peak"])
    chk("overshoot = 0.08 deg, both directions", abs(x["overshoot"] - 0.08) < 0.003 and abs(y["overshoot"] - 0.08) < 0.003,
        (x["overshoot"], y["overshoot"]))
    expect = 1000 * 0.05 * np.log(0.08 / SETTLE_DEG)        # exp decay below 0.05 deg: ~23.5 ms
    chk(f"settle ~{expect:.0f} ms (smooth decay)", abs(x["settle_ms"] - expect) <= 1000 * dt + 0.5, x["settle_ms"])
    chk("settle 52 ms for the ringing one (from the LAST sample outside 0.05 deg, not the first inside)",
        abs(y["settle_ms"] - 52.0) < 1e-6, y["settle_ms"])
    chk("accelerating / decelerating RMS < cruising (lag grows with speed)", x["acc"] < x["cru"] and x["dec"] < x["cru"],
        (x["acc"], x["cru"], x["dec"]))
    chk("the extra decelerating lag lands in 'dec', not 'acc' (phases follow the motion's direction)",
        y["dec"] > y["acc"] + 0.02, (y["acc"], y["dec"]))
    chk("final error ~0 (settled)", abs(x["final"]) < 1e-3 and abs(y["final"]) < 1e-3, (x["final"], y["final"]))
    chk("drive torque beyond the feedforward = 0.3 Nm", abs(x["pid"] - 0.3) < 1e-3, x["pid"])
    chk("coupling: J4 sat 0.02 deg off", abs(x["others"] - 0.02) < 1e-4, x["others"])
    chk("commanded peak speed 40 deg/s", abs(x["vpeak"] - 40.0) < 0.01, x["vpeak"])
    # a pure 6 ms delay plus an error proportional to the ACCELERATION (what inertia does): the shift finds 6 ms, the
    # residual is exactly the acceleration part, the plain error is much larger (the delay x the speed)
    ta_ = np.arange(0, 1.2, dt)
    cm = np.where(ta_ < 0.4, 0.5 * 100 * ta_ ** 2, np.where(ta_ < 0.8, 8 + 40 * (ta_ - 0.4), 24 + 40 * (ta_ - 0.8)
                  - 0.5 * 100 * (ta_ - 0.8) ** 2))
    cm = np.where(ta_ < 1.2, cm, cm[-1])
    vv = np.gradient(cm, dt); aa = np.gradient(vv, dt)
    me = np.interp(ta_ - 0.006, ta_, cm) - 0.0005 * np.interp(ta_ - 0.006, ta_, aa)
    D, r = best_delay(ta_, np.radians(cm), np.radians(me), vv > 0.5)
    want = np.sqrt(np.mean((0.0005 * np.interp(ta_ - 0.006, ta_, aa))[vv > 0.5] ** 2))
    chk("time shift: a pure 6 ms delay is found (0.25 ms steps)", abs(D - 0.006) <= 0.00026, D)
    chk("after the shift only the acceleration-proportional error is left", abs(np.sqrt(np.mean(r ** 2)) - want) < 0.003,
        (np.sqrt(np.mean(r ** 2)), want))
    chk("a lag of 0.004 deg per deg/s IS a 4 ms shift: the first move's delay ~4 ms, almost nothing left after it",
        abs(x["delay_ms"] - 4.0) <= 0.25 + 1e-9 and x["rms_d"] < 0.005, (x["delay_ms"], x["rms_d"]))
    s = summarise([x] * 4)
    chk("summary: median per speed", s["A"]["J3"]["alone"]["30"]["lag"] == x["lag"] and s["A"]["J3"]["alone"]["30"]["n"] == 4,
        s["A"]["J3"]["alone"]["30"])
    pr = room_problems([0, -138.0, 180, 0, 0, 180], [1, 2], [-160, -140, -160, -160, -160, -160], [160, -6.4, 376, 160, 160, 400], {1: 20.0, 2: 10.0})
    chk("room check: J2 2 deg above a -140 limit is refused, J1 in the middle is not",
        len(pr) == 1 and pr[0].startswith("J2"), pr)
    chk("set labels: new, old, new2", label_sets(["new", "old", "new"]) == ["new", "old", "new2"])
    # the gain tables come out of the real robot XML
    try:
        g = read_gain_sets(os.path.join(_ROOT, "robots", "PAR6.xml"))
        chk("PAR6.xml: 6 old + 6 new gain sets, J1 old KPP 5, new KPP 88",
            len(g["old"]) == 6 and g["old"][0]["KPP"] == 5.0 and g["new"][0]["KPP"] == 88.0, g["old"][0])
    except Exception as exc:
        chk("PAR6.xml gain sets readable", False, exc)
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


def room_problems(q0_deg, joints1, lo_deg, hi_deg, amps):
    """Joints whose full +/- amplitude does not fit CLEARANCE_DEG inside the soft limits from where the arm is: the
    moves must be symmetric to compare, so the session refuses rather than shrinking them."""
    out = []
    for j1 in joints1:
        j = j1 - 1
        lo_room = q0_deg[j] - (lo_deg[j] + CLEARANCE_DEG)
        hi_room = (hi_deg[j] - CLEARANCE_DEG) - q0_deg[j]
        if min(lo_room, hi_room) < amps[j1]:
            out.append(f"J{j1} at {q0_deg[j]:.1f} deg has {lo_room:.1f} / {hi_room:.1f} deg of room below / above "
                       f"(soft limits {lo_deg[j]:.1f} .. {hi_deg[j]:.1f}, {CLEARANCE_DEG:g} deg clearance), needs "
                       f"{amps[j1]:g} each way")
    return out


def plan_moves(q0_deg, joints1, lo_deg, hi_deg, amps, speeds):
    """[{label, joint, speed, start, target}] in degrees — each joint alone, then all together."""
    moves = []
    q0 = np.asarray(q0_deg, float)

    def reach(j, a):
        return min(q0[j] + a, hi_deg[j] - CLEARANCE_DEG), max(q0[j] - a, lo_deg[j] + CLEARANCE_DEG)

    for j1 in joints1:
        j = j1 - 1
        up, dn = reach(j, amps[j1])
        for sp in speeds:
            cur = q0.copy()
            for tag, v in (("+", up), ("back", q0[j]), ("-", dn), ("back", q0[j])):
                tgt = cur.copy()
                tgt[j] = v
                moves.append({"label": f"J{j1} {tag} @{sp:g}%", "joint": j1, "speed": sp, "start": cur.tolist(),
                              "target": tgt.tolist()})
                cur = tgt
    if len(joints1) > 1:
        up, dn = q0.copy(), q0.copy()
        for j1 in joints1:
            u, d = reach(j1 - 1, amps[j1] / 2.0)
            up[j1 - 1], dn[j1 - 1] = u, d
        for sp in [s for s in ALL_JOINT_SPEEDS if s <= max(speeds)]:
            cur = q0.copy()
            for tag, tgt in (("+", up), ("back", q0), ("-", dn), ("back", q0)):
                moves.append({"label": f"all {tag} @{sp:g}%", "joint": "all", "speed": sp, "start": cur.tolist(),
                              "target": np.asarray(tgt, float).tolist()})
                cur = np.asarray(tgt, float).copy()
    return moves


def run_session(args, cfg, gain_sets, set_order):
    from robotclient import RobotClient
    lo = np.degrees(np.asarray(cfg["software_joint_limit_negative"], float))
    hi = np.degrees(np.asarray(cfg["software_joint_limit_positive"], float))
    joints1 = [int(x) for x in args.joints.split(",") if x.strip()]
    speeds = [float(x) for x in args.speeds.split(",") if x.strip()]
    amps = {j: DEFAULT_AMP[j] * args.amp_scale for j in joints1}
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    capture, prev_recipe, claimed, gains_changed = None, None, False, False
    records, ok = [], True
    names = label_sets(set_order)
    try:
        st = robot.status()
        q0 = np.asarray(robot.get_joints(), float)
        moves = plan_moves(q0, joints1, lo, hi, amps, speeds)
        print(f"Joints now (deg): {np.round(q0, 2).tolist()}")
        for j1 in joints1:
            j = j1 - 1
            u = max(m["target"][j] for m in moves if m["joint"] == j1)
            dn = min(m["target"][j] for m in moves if m["joint"] == j1)
            print(f"  J{j1}: {dn:7.1f} .. {u:7.1f} deg (around {q0[j]:.1f})")
        print(f"Plan per gain set: {len(moves)} moves at {speeds} % of the EXEC limits, {HOLD_S} s hold after each; "
              f"sets in order: {names}.")
        print("Gains (the current and velocity LIMITS stay the XML's):")
        for nm in dict.fromkeys(set_order):
            print(f"  {nm:4s} " + "  ".join(f"J{j + 1} " + "/".join(f"{gain_sets[nm][j][k]:g}" for k in GAIN_KEYS)
                                           for j in range(6)))
        print(f"  ({'/'.join(GAIN_KEYS)})")
        problems = room_problems(q0, joints1, lo, hi, amps)
        if problems:
            print("\nNOT ENOUGH ROOM from this pose:")
            for pr in problems:
                print(f"  {pr}")
            print("Move the arm to a free working pose first — e.g. where homing leaves it (J1 90, J2 -106, J3 163, "
                  "J4 0, J5 -29, J6 180 deg) — or lower --amp-scale. Nothing moved.")
            return 2, None
        if not args.go:
            print("DRY RUN: re-run with --go to execute. Nothing moved.")
            return 0, None
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") not in ("IDLE", "EXEC"):
            print(f"Robot must be ENABLED + homed, in IDLE or EXEC (got mode={st.get('mode')} "
                  f"state={st.get('state')} homed={st.get('homed')}). Nothing moved.")
            return 2, None
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
        if capture.last_tick() is None:
            print("No telemetry with the tracking fields is arriving on UDP 30003. Nothing moved.")
            return 2, None
        for set_name, label in zip(set_order, names):
            print(f"\n=== gain set '{label}' ===")
            apply_gains(robot, gain_sets[set_name])
            gains_changed = gains_changed or set_name != "new"
            time.sleep(1.0)                               # let the hold settle on the new gains
            for i, mv in enumerate(moves, 1):
                t0 = capture.last_tick()
                robot.move_joints(*[float(x) for x in mv["target"]], speed=mv["speed"])
                fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
                time.sleep(HOLD_S)
                t1 = capture.last_tick()
                print(f"  [{i:3d}/{len(moves)}] {mv['label']}")
                if fails:
                    print(f"Move failed: {fails}. Stopping the session.")
                    ok = False
                    break
                records.append({"set": label, **mv, "t0": t0, "t1": t1})
            if not ok:
                break
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the robot.")
        ok = False
        try:
            robot.stop()
        except Exception:
            pass
    finally:
        if gains_changed:
            try:
                apply_gains(robot, gain_sets["new"])
                print("XML gains restored.")
            except Exception as exc:
                print(f"!!! COULD NOT RESTORE THE XML GAINS ({exc}) — re-apply them from the dashboard's Tune Joint, "
                      f"or restart the service, before using the arm.")
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
    if capture is None or not records:
        return (0 if ok else 1), None
    return (0 if ok else 1), (capture.packets, records, names)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true", help="actually move the arm")
    ap.add_argument("--selftest", action="store_true", help="check the analysis on synthetic data and exit")
    ap.add_argument("--sets", default="new,old,new", help="gain sets in order (new = the XML, old = before 2026-10-09)")
    ap.add_argument("--joints", default="1,2,3,4,5,6", help="1-based joints to move")
    ap.add_argument("--speeds", default=",".join(f"{s:g}" for s in DEFAULT_SPEEDS), help="percent of the EXEC limits")
    ap.add_argument("--amp-scale", type=float, default=1.0, help="scale the default amplitudes (<= 1 to be gentler)")
    ap.add_argument("--compare", nargs=2, metavar=("A.json", "B.json"), help="put two saved runs side by side")
    ap.add_argument("--reanalyse", metavar="RUN.json", help="recompute a saved run's metrics from its .npz (no robot)")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.reanalyse:
        run = json.load(open(args.reanalyse))
        raw = np.load(args.reanalyse[:-5] + ".npz")
        d = {k: raw[k] for k in raw.files}
        moves = analyse(d, run["records"], 0.004)
        summ = summarise(moves)
        names = [n for n in run["set_names"] if n in summ]
        print_report(summ, names, f"=== {args.reanalyse} (re-analysed) ===")
        run.update({"summary": summ, "moves": moves, "set_names": names})
        out = args.reanalyse[:-5] + "_reanalysed.json"
        with open(out, "w") as fh:
            json.dump(run, fh, indent=1)
        print(f"\nSaved {out}")
        return 0
    if args.compare:
        a, b = (json.load(open(p)) for p in args.compare)
        merged, order = {}, []
        for tag, run in (("A", a), ("B", b)):
            for s in run["set_names"]:
                merged[f"{tag}:{s}"] = run["summary"][s]
                order.append(f"{tag}:{s}")
        print_report(merged, order, f"A = {args.compare[0]}\nB = {args.compare[1]}")
        return 0
    if args.amp_scale > 1.0:
        print("--amp-scale above 1 is refused: these moves are meant to stay small.")
        return 2
    set_order = [s.strip() for s in args.sets.split(",") if s.strip()]
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    gain_sets = read_gain_sets(str(sysc["robot_xml_path"]))
    if any(s not in gain_sets for s in set_order):
        print(f"Unknown gain set in {set_order}; known: {list(gain_sets)}.")
        return 2
    rc, got = run_session(args, cfg, gain_sets, set_order)
    if got is None:
        return rc
    packets, records, names = got
    dt = float(cfg.get("delta_t", 0.004))
    d = to_arrays(packets)
    moves = analyse(d, records, dt)
    summ = summarise(moves)
    print_report(summ, [n for n in names if n in summ], "\n=== tracking (commanded - measured) ===")
    out_dir = os.path.join(_ROOT, "temp_data", "tracking")
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    np.savez_compressed(os.path.join(out_dir, f"{stamp}.npz"), **d)
    with open(os.path.join(out_dir, f"{stamp}.json"), "w") as fh:
        json.dump({"set_names": [n for n in names if n in summ], "summary": summ, "moves": moves, "records": records,
                   "gains": {k: v for k, v in gain_sets.items()}, "args": vars(args)}, fh, indent=1)
    print(f"\nSaved temp_data/tracking/{stamp}.npz and .json")
    return rc


if __name__ == "__main__":
    sys.exit(main())
