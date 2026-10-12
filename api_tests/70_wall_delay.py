#!/usr/bin/env python3
"""70: does the loop delay decide whether a stiff freedrive wall is stable? One joint's wall, option B OFF / ON / OFF
(2026-10-12, TODO/can_tx_timing_plan.md §9.6). You flick the joint into its wall by hand; nothing is commanded to move.

Why: the haptic walls' stability table (tools/haptic_stability_table.py) is proven with the wall's force computed from a
state 1 tick old. api_tests/69 measured the real loop (a joint's reply -> the command computed from it): 7.8 ms, ~2 ticks,
in the normal tick order; 3.8 ms, ~1 tick, with option B (RTI reads CAN first, sends early — `set_can_read_first`). In the
table's own model a wall above ~7 Hz (200 N/m at 0.1 kg) ADDS energy at 2 ticks whatever its damping — 10 Hz: a bounce
leaves 1.35x faster than it came in — and is passive at 1 tick (0.83x). The defaults (7.1 Hz 'auto') are fine either way.

What it does: sets the joint's wall LIVE to the test frequency (default 10 Hz, the shipped damping ratio, the 4 ms speed
filter; the stability table must allow it — the executor refuses otherwise), puts the robot in IDLE gravity comp
(freedrive), then per phase (default off, on, off): switches option B, and for --seconds you flick the joint towards
its wall and LET GO BEFORE IT GETS THERE, several times — it hits the wall on its own and bounces out. After each phase
it asks whether you felt a buzz. At the end the wall setting, option B (off), friction compensation and the telemetry
recipe are put back (also on Ctrl-C). Telemetry (recipe haptic_wall) is recorded throughout.

Per contact (the joint inside its wall for >= 3 ticks): the speed going in (v_in) and coming out (v_out) — out / in
above 1 = the wall ADDED energy; only free bounces count (v_in >= 10 deg/s: a flick, not a push); the speed's
reversals inside (1 = a clean bounce, more = ringing); the deepest point. Per phase: the median out / in, the share
of bounces above 1, ringing contacts.

    python3 api_tests/70_wall_delay.py --selftest          # the analysis on simulated bounces (1 and 2 ticks)
    python3 api_tests/70_wall_delay.py                     # dry run: the joint, its wall, the test setting
    python3 api_tests/70_wall_delay.py --go                # J5, 10 Hz, off / on / off, 20 s each
    python3 api_tests/70_wall_delay.py --go --joint 6 --hz 9
    python3 api_tests/70_wall_delay.py --reanalyse temp_data/wall_delay/<time>.npz

Light joints only (J4-J6): their walls push fractions of a Nm. Keep the other hand near the arm.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FS = 250.0
DT = 1.0 / FS
RECIPE = "haptic_wall"
VIN_MIN = 10.0          # deg/s: a contact entered slower than this was a push, not a free bounce
MIN_TICKS = 3           # a contact shorter than this is noise at the boundary
AFTER = 3               # ticks after the exit in which the outgoing speed is taken: with a delay the wall still pushes
#                         (its force comes from a state that was still inside) — the push it adds counts
SAFE_JOINTS = (4, 5, 6)


# ----------------------------------------------------------------------------- analysis (pure)
def contacts(j_p, j_s, side, limit_deg):
    """Every stretch with the joint inside its wall (>= MIN_TICKS ticks): entry / exit speeds (deg/s, positive = into
    / out of the wall; the exit speed the fastest in the AFTER ticks after leaving — the delayed push included), out /
    in, reversals of the speed inside, the deepest point (deg)."""
    q = np.degrees(np.asarray(j_p, float))
    v = side * np.degrees(np.asarray(j_s, float))            # + = into the wall
    depth = side * (q - limit_deg)
    inside = depth > 0
    edges = np.flatnonzero(np.diff(np.r_[0, inside.astype(int), 0]))
    out = []
    for a, b in zip(edges[::2], edges[1::2]):
        if b - a < MIN_TICKS or a == 0 or b + AFTER > q.size:
            continue                                          # too short, or cut by the recording's ends
        v_in = float(max(v[a - 1], v[a]))
        v_out = float(-v[b - 1:b + AFTER].min())
        sv = np.sign(v[a:b])
        sv = sv[sv != 0]
        rev = int(np.sum(sv[1:] != sv[:-1])) if sv.size > 1 else 0
        out.append({"i0": int(a), "i1": int(b), "ticks": int(b - a), "v_in": v_in, "v_out": v_out,
                    "ratio": v_out / v_in if v_in > 0 else float("nan"), "reversals": rev,
                    "depth_max": float(depth[a:b].max()), "free": v_in >= VIN_MIN})
    return out


def summarise(cs):
    free = [c for c in cs if c["free"] and math.isfinite(c["ratio"])]
    r = np.array([c["ratio"] for c in free])
    return {"contacts": len(cs), "free": len(free),
            "ratio_med": float(np.median(r)) if r.size else float("nan"),
            "ratio_p90": float(np.percentile(r, 90)) if r.size else float("nan"),
            "above_1": int(np.sum(r > 1.0)), "ringing": int(sum(1 for c in cs if c["reversals"] > 1)),
            "depth_med": float(np.median([c["depth_max"] for c in cs])) if cs else float("nan")}


def analyse(d, meta):
    side, limit, jj = meta["side"], meta["limit_deg"], meta["joint"] - 1
    res = []
    for ph in meta["phases"]:
        sel = (d["tick"] >= ph["tick0"]) & (d["tick"] <= ph["tick1"])
        cs = contacts(d["j_p"][sel, jj], d["j_s"][sel, jj], side, limit)
        res.append({"name": ph["name"], "read_first": ph["read_first"], "felt": ph.get("felt"), **summarise(cs),
                    "list": cs})
    return res


def print_report(res, meta):
    print(f"J{meta['joint']}'s wall at {meta['hz']:g} Hz (damping ratio {meta['ratio']:.2f}, filter {meta['filter_ms']:g} ms), "
          f"{'upper' if meta['side'] > 0 else 'lower'} side at {meta['limit_deg']:.1f} deg. Free bounces: entered at "
          f">= {VIN_MIN:g} deg/s. out/in > 1 = the wall added energy.")
    print(f"  {'phase':10s} {'B':>3s} {'contacts':>8s} {'free':>5s} {'out/in med':>10s} {'p90':>6s} {'>1':>4s} "
          f"{'ringing':>7s} {'depth med':>9s}  felt a buzz")
    for r in res:
        print(f"  {r['name']:10s} {'ON' if r['read_first'] else 'off':>3s} {r['contacts']:8d} {r['free']:5d} "
              f"{r['ratio_med']:10.2f} {r['ratio_p90']:6.2f} {r['above_1']:4d} {r['ringing']:7d} "
              f"{r['depth_med']:8.2f}°  {r.get('felt') or '-'}")
    print("  per free bounce (v_in -> v_out deg/s, reversals):")
    for r in res:
        cells = [f"{c['v_in']:.0f}->{c['v_out']:.0f}{'*' if c['reversals'] > 1 else ''}" for c in r["list"] if c["free"]]
        print(f"    {r['name']:10s} " + ("  ".join(cells) if cells else "(none)"))


# ----------------------------------------------------------------------------- self-test
def simulate_bounce(hz, ratio, filter_ms, delay, v0_dps=40.0, inertia=0.002, seconds=0.8, sub=20):
    """A free joint hitting its wall at +v0, the wall law of utility/haptic_walls.py (spring + two-sided damping on an EMA
    of the raw speed, never pulling), the force computed from the state `delay` ticks old and held for a tick — the model
    of tools/haptic_stability_table.py, in joint terms. Wall at 0 rad. -> (j_p, j_s) per tick."""
    w = 2 * math.pi * hz
    k, dmp = w * w * inertia, 2 * ratio * w * inertia
    a = 1.0 if filter_ms <= 0 else 1 - math.exp(-DT / (filter_ms / 1000.0))
    x, v = -math.radians(1.0), math.radians(v0_dps)
    vf = v
    hist = [(x, v)] * (delay + 2)
    qs, vs = [], []
    for _ in range(int(seconds * FS)):
        xs, vs_ = hist[-1 - delay]
        vf += a * (vs_ - vf)
        tau = -max(k * xs + dmp * vf, 0.0) if xs > 0 else 0.0
        for _ in range(sub):
            v += tau / inertia * (DT / sub)
            x += v * (DT / sub)
        hist.append((x, v))
        hist = hist[-(delay + 3):]
        qs.append(x)
        vs.append(v)
    return np.array(qs), np.array(vs)


def selftest():
    ok_all = True

    def chk(name, cond, detail=""):
        nonlocal ok_all
        ok_all &= bool(cond)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}" + ("" if cond else f"  [{detail}]"))

    import importlib
    sys.path.insert(0, os.path.join(_ROOT, "tools"))
    hst = importlib.import_module("haptic_stability_table")
    # the stability model at 400 N/m on 0.1 kg = 10.07 Hz: our joint-terms simulation must give the same ratio
    for delay in (1, 2):
        # the model's approach: 0.02 m at 0.3 m/s = 67 ms; here 1 deg at 15 deg/s (the ratio is scale-free)
        q, v = simulate_bounce(math.sqrt(400 / 0.1) / (2 * math.pi), 0.67, 4, delay, v0_dps=15.0)
        r_model = hst.light_bounce(400, 2 * 0.67 * math.sqrt(400 * 0.1), 0.1, 0.004, delay=delay, v0=0.3)
        i_out = int(np.flatnonzero((q < 0) & (v < 0))[0])      # the model's own measure: the first sample back outside
        chk(f"the joint-terms simulation = the stability model at {delay} tick(s) ({r_model:.2f})",
            abs(-v[i_out] / math.radians(15.0) - r_model) < 0.02, (-v[i_out] / math.radians(15.0), r_model))
        cs = contacts(q, v, 1.0, 0.0)
        chk(f"...contacts() takes the FINAL outgoing speed, the delayed push after the exit included ({delay} tick(s))",
            len(cs) == 1 and abs(cs[0]["v_out"] - math.degrees(-v[-1])) < 1e-9, (cs, math.degrees(-v[-1])))
    q1, v1 = simulate_bounce(10.0, 0.67, 4, 1)
    q2, v2 = simulate_bounce(10.0, 0.67, 4, 2)
    s1, s2 = summarise(contacts(q1, v1, 1.0, 0.0)), summarise(contacts(q2, v2, 1.0, 0.0))
    chk("10 Hz wall, 1 tick (option B): the bounce LOSES energy (out/in < 1)", s1["free"] == 1 and s1["ratio_med"] < 1.0,
        s1)
    chk("10 Hz wall, 2 ticks (the old order): the bounce GAINS energy (out/in > 1)", s2["free"] == 1
        and s2["ratio_med"] > 1.0, s2)
    qd, vd = simulate_bounce(7.1, 0.67, 4, 2)
    sd = summarise(contacts(qd, vd, 1.0, 0.0))
    chk("the default 7.1 Hz wall at 2 ticks still loses energy (the defaults are fine either way)",
        sd["ratio_med"] < 1.0, sd)
    # the lower side: mirrored
    ql, vl = simulate_bounce(10.0, 0.67, 4, 2)
    sl = summarise(contacts(-ql + math.radians(-50.0), -vl, -1.0, -50.0))
    chk("the lower side (mirrored) reads the same", abs(sl["ratio_med"] - s2["ratio_med"]) < 1e-9, (sl, s2))
    # a slow push is not a free bounce; a contact cut by the recording's end is dropped
    qp = np.radians(np.r_[np.linspace(-1, 2, 250), np.full(30, 2.0), np.linspace(2, -1, 250)])   # 3 deg/s
    vp = np.gradient(qp) * FS
    cp = contacts(qp, vp, 1.0, 0.0)
    chk("a slow push in and out: one contact, not free (v_in < 10 deg/s)", len(cp) == 1 and not cp[0]["free"], cp)
    mix = summarise(contacts(q2, v2, 1.0, 0.0) + cp)
    chk("a phase with a free bounce and a push: only the bounce is judged (free 1 of 2, its ratio)",
        mix["contacts"] == 2 and mix["free"] == 1 and abs(mix["ratio_med"] - s2["ratio_med"]) < 1e-12, mix)
    chk("a contact still open at the recording's end is not judged", contacts(q1[:20], v1[:20], 1.0, 0.0) == [] or
        all(c["i1"] < 20 for c in contacts(q1[:20], v1[:20], 1.0, 0.0)))
    # ringing: a speed that reverses several times inside
    qr = np.radians(np.r_[np.linspace(-1, 0.5, 10), 0.5 + 0.2 * np.sin(np.linspace(0, 6 * np.pi, 40)), np.linspace(0.5, -1, 10)])
    vr = np.gradient(qr) * FS
    cr = contacts(qr, vr, 1.0, 0.0)
    chk("ringing inside the wall is counted (reversals > 1)", cr and cr[0]["reversals"] > 1, cr)
    print("selftest:", "ALL PASSED" if ok_all else "FAILED")
    return 0 if ok_all else 1


# ----------------------------------------------------------------------------- robot session
def save(d, meta):
    from project_paths import TEMP_DATA_DIR
    out = TEMP_DATA_DIR / "wall_delay"
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d_%H%M%S')}_J{meta['joint']}_{meta['hz']:g}Hz.npz"
    np.savez(path, meta=json.dumps(meta, default=str), **d)
    return path


def reanalyse(path):
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z["meta"]))
    d = {k: z[k] for k in z.files if k != "meta"}
    print(f"=== {path} ===")
    print_report(analyse(d, meta), meta)
    return 0


def newest_tick(w51, capture):
    """The newest telemetry tick received (None before any)."""
    if not capture.packets:
        return None
    t = w51.to_arrays(capture.packets[-50:])["tick"]
    return float(t.max()) if t.size else None


def run(args):
    from robotclient import RobotClient
    from robotclient.exceptions import CommandRejected
    w51 = __import__("51_haptic_wall")
    base = __import__("46_monitor_baseline")
    N, j = args.joint, args.joint - 1
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed, capture, prev_recipe, restore_spec, fric_was, rf_touched = False, None, None, None, False, False
    meta = {"joint": N, "hz": args.hz, "filter_ms": args.filter, "phases": []}
    try:
        w = robot.get_joint_walls()["walls"][j]
        if not w["enabled"]:
            print(f"J{N} has no freedrive wall. Nothing to test.")
            return 2, None
        q = float(robot.get_joints()[j])
        upper = (w["hi_deg"] - q) <= (q - w["lo_deg"])
        side, limit = (1.0, w["hi_deg"]) if upper else (-1.0, w["lo_deg"])
        ratio = args.ratio if args.ratio is not None else float(w["damping_ratio"])
        meta.update({"side": side, "limit_deg": limit, "ratio": ratio, "wall_before": w})
        bt, room = w["breakthrough_deg"], w["breakthrough_max_deg"]
        test_bt = bt if room is None or bt <= room - 1.0 else max(2.0, room - 1.0)
        print(f"J{N} now {q:.1f} deg; its wall: {w['frequency_hz']:.2f} Hz, ratio {w['damping_ratio']:.2f}, filter "
              f"{w.get('filter_ms', '?')} ms; nearest side {'upper' if upper else 'lower'} at {limit:.1f} deg "
              f"({abs(limit - q):.0f} deg away). Test setting: {args.hz:g} Hz, ratio {ratio:.2f}, filter {args.filter:g} ms"
              + (f"; breakthrough lowered {bt:.1f} -> {test_bt:.1f} deg (endstop)" if test_bt != bt else "")
              + f". Phases: {args.order}, {args.seconds:g} s each.")
        if not args.go:
            print("DRY RUN: re-run with --go. Nothing moved, nothing changed.")
            return 0, None
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True:
            print(f"Robot must be ENABLED and homed (got state={st.get('state')} homed={st.get('homed')}).")
            return 2, None
        base.claim_control(robot)
        claimed = True
        fric_was = bool(st.get("friction_comp"))
        if fric_was:
            robot.set_friction_comp(False)
        restore_spec = {key: ("auto" if v is None else v) for key, v in w["spec"].items()}
        try:
            robot.set_joint_wall(N, frequency_hz=args.hz, damping_ratio=ratio, filter_ms=args.filter,
                                 **({"breakthrough_deg": test_bt} if test_bt != bt else {}))
        except CommandRejected as e:
            print(f"The executor refused the test setting: {e}. Nothing moved.")
            return 2, None
        capture = base.Capture()
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        if st.get("mode") == "EXEC":
            robot.enter_idle_mode(wait=True)
        robot.set_gravity_comp(True)
        time.sleep(0.5)
        if newest_tick(w51, capture) is None:
            print(f"No {RECIPE} telemetry is arriving on this machine. Nothing moved.")
            return 2, None
        for i, name in enumerate(args.order.split(",")):
            rf = name.strip() == "on"
            robot.set_can_read_first(rf)
            rf_touched = True
            robot.release_control()
            claimed = False
            input(f"\nPhase {i + 1} — option B {'ON' if rf else 'off'}. Press Enter, then for {args.seconds:g} s flick J{N} "
                  f"towards {limit:.1f} deg and LET GO before it gets there, several times. ")
            t0 = newest_tick(w51, capture)
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                print(f"   {end - time.monotonic():4.1f} s left   J{N} {float(robot.get_joints()[j]):7.2f} deg "
                      f"(wall at {limit:.1f})", end="\r")
                time.sleep(0.1)
            t1 = newest_tick(w51, capture)
            felt = input("\n   Did you feel a buzz / vibration in the wall? [y/n] ").strip().lower()[:1] or "-"
            meta["phases"].append({"name": f"{name.strip()}_{i + 1}", "read_first": rf, "tick0": float(t0),
                                   "tick1": float(t1), "felt": felt})
            base.claim_control(robot)
            claimed = True
        d = w51.to_arrays(capture.packets)
        return 0, (d, meta)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1, None
    finally:
        try:
            if not claimed:
                base.claim_control(robot)
                claimed = True
            if rf_touched:
                robot.set_can_read_first(False)
            if restore_spec is not None:
                robot.set_joint_wall(N, **restore_spec)
                print(f"J{N}'s wall setting restored.")
            if fric_was:
                robot.set_friction_comp(True)
            if prev_recipe:
                robot.set_recipe(prev_recipe)
        except Exception as e:
            print(f"!! could not restore everything ({e}) — check J{N} in the Walls tab; option B goes off at the next "
                  f"RTI start (the robot XML was never changed)")
        if capture is not None:
            capture.stop()
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--joint", type=int, default=5, help=f"one of {SAFE_JOINTS}")
    ap.add_argument("--hz", type=float, default=10.0, help="the wall's frequency for the test")
    ap.add_argument("--ratio", type=float, default=None, help="damping ratio (default: the wall's own)")
    ap.add_argument("--filter", type=float, default=4.0, help="speed filter, ms")
    ap.add_argument("--order", default="off,on,off", help="phases: off / on (option B)")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--reanalyse")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.reanalyse:
        return reanalyse(args.reanalyse)
    if args.joint not in SAFE_JOINTS or any(p.strip() not in ("on", "off") for p in args.order.split(",")) \
            or not 3.0 <= args.hz <= 13.0:
        print(f"Refused: --joint must be one of {SAFE_JOINTS}, --order of on / off, --hz 3..13. Nothing moved.")
        return 2
    rc, got = run(args)
    if got is None:
        return rc
    d, meta = got
    path = save(d, meta)
    rel = os.path.relpath(path, _ROOT)
    print(f"\nSaved {rel} ({d['tick'].size} ticks). Not analysed while the arm is enabled; disable it, then:\n"
          f"  cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/70_wall_delay.py --reanalyse {rel}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
