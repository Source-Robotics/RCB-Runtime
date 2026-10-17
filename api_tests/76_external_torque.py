#!/usr/bin/env python3
"""76: the external torque estimate on the arm (2026-10-16) — how much "external" torque the momentum observer
(utility/momentum_observer.py) reads when NOTHING touches the arm, per joint and speed. That is the floor any contact /
collision threshold must stay above; this measures it before any check acts on the estimate.

ON THE ARM (--go; the workspace clear, nothing touching the arm during the moves):
  1. the start pose (START_POSE_DEG);
  2. per joint of --joints: a capture (the recorder: every tick), moves of +-AMP deg and back at each speed of --speeds,
     a rest between, then Stop & save;
  3. --push S: one more capture of S seconds while the arm holds still in EXEC — push the tool by hand in a few
     directions: the estimate (t_ext, the TCP force) should follow your push.
  The capture files are copied to temp_data/external_torque/<UTC>/. The analysis runs right away only with the robot
  DISABLED (it replays the observer over every tick — CPU on this Pi slows RTI's ticks); otherwise run
  `--reanalyse <that folder>` after disabling the arm.

THE ANALYSIS, per capture (the joint that moved, and every joint while it moved):
  - at rest: the estimate's offset (a gravity-model / torque-calibration error at that pose, the same in the old
    estimate) and its noise — the BIAS, reported apart: a threshold acts on the estimate minus it;
  - moving: |estimate - offset| p50 / p95 / p99 / max — the false "external" torque the motion adds, which a
    threshold must stay above — next to the OLD static estimate (measured - gravity, minus its own offset) recomputed
    from the same ticks;
  - the observer replayed offline on the recorded inputs with gains 25 / 50 / 100 and friction on / off;
  - for the joint that moved: a fit estimate ~ a·tanh(v/0.02) + b·v + c·acc + d over its moving ticks — friction the
    model lacks (a > 0: more Coulomb friction than the robot XML's model, utility/friction_model.py; b: viscous,
    Nm·s/rad), an inertia error (c) — and the <friction_coulomb_nm> it suggests;
  - suggested thresholds per joint, from its own capture: contact ~ 2 x the moving p99, collision ~ the larger of
    2 x the moving max and 1.5 x contact (on the estimate minus the offset; suggestions only).

    .venv/bin/python api_tests/76_external_torque.py --go                        # every joint, 20/50/100 %
    .venv/bin/python api_tests/76_external_torque.py --go --joints 1,2 --push 15
    .venv/bin/python api_tests/76_external_torque.py --reanalyse temp_data/external_torque/20261016_130000
    .venv/bin/python api_tests/76_external_torque.py --reanalyse FOLDER --recompute   # the estimate recomputed offline
        from the recorded inputs with today's code (when the robot ran an older one — e.g. before a fix)
    .venv/bin/python api_tests/76_external_torque.py --selftest                  # no arm: the PAR6 model simulated
"""
from __future__ import annotations

import glob
import json
import math
import os
import shutil
import struct
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

AMP_DEG = {1: 30.0, 2: 15.0, 3: 15.0, 4: 30.0, 5: 25.0, 6: 45.0}
MOVING_RAD_S = 0.02            # a joint moves above this filtered speed (~1.1 deg/s)
REST_RAD_S = 0.005             # every joint below this = the arm at rest
# A joint's capture keeps the recorder's 5 s BEFORE its start: the move to the start pose (J1's) or the previous joint's
# last fast moves — another joint's OWN motion at other poses, which read as this joint's "coupling" and set thresholds
# (2026-10-17: J3 1.77 instead of 1.49). So the moving numbers of a joint's capture are taken where that joint moves
# and every other joint is below the larger of this (rad/s, 0.5 deg/s) and DOMINANT x its speed — on the arm the
# held joints sit at ~0 (p90 under 0.3 deg/s), the pre-roll's moves are 4-125 deg/s.
STILL_RAD_S = math.radians(0.5)
DOMINANT = 0.1
GAINS = (25.0, 50.0, 100.0)
OUT_DIR = os.path.join(ROOT, "temp_data", "external_torque")


# ---------------------------------------------------------------------------------------------------------------
class FrictionList(list):
    """The Coulomb friction per joint (a list, as 76 always used it) + the model's viscous and load terms (robot XML
    <friction_viscous_nm_s> / <friction_load_per_nm>, 2026-10-17) — what RTI's observer uses."""

    def __init__(self, coulomb, viscous=None, load=None):
        super().__init__(float(x) for x in coulomb)
        self.viscous = [float(x) for x in (viscous if viscous is not None else [0.0] * len(self))]
        self.load = [float(x) for x in (load if load is not None else [0.0] * len(self))]


def load_model():
    """(momentum_fn, gravity_fn, friction, robot) for the active robot, as RTI builds them; friction: a FrictionList
    (THE friction model, utility/friction_model.py)."""
    from config import xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from utility.pinocchio_attach import _make_pinocchio_gravity, build_pinocchio_dynamics, make_pinocchio_momentum_terms
    from utility.robot_factory import create_robot
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    robot = create_robot(cfg["robot_name"], gripper_name=cfg["gripper_name"])
    model, data = build_pinocchio_dynamics(robot)
    friction = FrictionList(cfg["friction_coulomb_nm"], cfg["friction_viscous_nm_s"], cfg["friction_load_per_nm"])
    return make_pinocchio_momentum_terms(robot), _make_pinocchio_gravity(model, data), friction, robot


def replay(ch, n, momentum_fn, gain, friction, gravity_fn=None):
    """The observer run offline over a recording's ticks (its inputs: j_p, j_s_f, j_t, timestamp) -> (ticks, n), in
    the PUBLISHED convention (-r: measured - model, as t_ext in the file). friction: the Coulomb list, or a FrictionList
    (+ viscous, + load — its gravity torque from gravity_fn per tick, as RTI's joint_gravity_torque)."""
    from utility.momentum_observer import MomentumObserver
    viscous, load = getattr(friction, "viscous", None), getattr(friction, "load", None)
    obs = MomentumObserver(n, gain=gain, friction_coulomb=list(friction), friction_viscous=viscous, friction_load=load)
    use_g = gravity_fn is not None and obs.model.has_load
    q, v, tau, ts = (np.ascontiguousarray(ch[k][:, :n] if ch[k].ndim == 2 else ch[k], dtype=float)
                     for k in ("j_p", "j_s_f", "j_t", "timestamp"))
    out = np.zeros((len(ts), n))
    for i in range(len(ts)):
        dt = ts[i] - ts[i - 1] if i else 0.0
        obs.step(q[i], v[i], tau[i], dt, momentum_fn, gravity=gravity_fn(q[i]) if use_g else None)
        out[i] = -obs.r
    return out


def static_estimate(ch, n, gravity_fn):
    """The estimate before 2026-10-16: the filtered measured torque minus gravity."""
    q, tf = ch["j_p"][:, :n], ch["j_t_f"][:, :n]
    return np.array([tf[i] - gravity_fn(np.ascontiguousarray(q[i], dtype=float)) for i in range(len(q))])


def _stats(x):
    a = np.abs(np.asarray(x, dtype=float))
    a = a[np.isfinite(a)]
    if not len(a):
        return None
    return {"p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)), "p99": float(np.percentile(a, 99)),
            "max": float(a.max()), "n": int(len(a))}


def fit_residual(r, v, acc):
    """r ~ a·tanh(v/0.02) + b·v + c·acc + d (least squares) -> (a, b, c, rms left) or None — only over a joint that
    really moved both ways (a fit on a still joint is meaningless)."""
    ok = np.isfinite(r) & np.isfinite(v) & np.isfinite(acc) & (np.abs(v) > MOVING_RAD_S)
    if ok.sum() < 50 or (v[ok] > 0).sum() < 10 or (v[ok] < 0).sum() < 10:
        return None
    A = np.column_stack([np.tanh(v[ok] / 0.02), v[ok], acc[ok], np.ones(int(ok.sum()))])
    coef, *_ = np.linalg.lstsq(A, r[ok], rcond=None)
    left = r[ok] - A @ coef
    return float(coef[0]), float(coef[1]), float(coef[2]), float(np.sqrt(np.mean(left ** 2)))


def analyse_capture(ch, n, momentum_fn, gravity_fn, friction, moved=None, recompute=False):
    """The numbers for one recording. `moved`: the joint (1-based) the capture moved, or None (a push / rest capture).
    recompute: the estimate from the observer replayed on the recorded inputs (today's code, gain 50) instead of the
    robot's own t_ext — for a recording made by an older RTI."""
    v = ch["j_s_f"][:, :n]
    acc = ch["j_a_f"][:, :n] if "j_a_f" in ch else np.gradient(v, axis=0) / 0.004
    est = replay(ch, n, momentum_fn, 50.0, friction, gravity_fn) if recompute else ch["t_ext"][:, :n]
    rest = np.all(np.abs(v) < REST_RAD_S, axis=1)
    out = {"ticks": int(len(v)), "moved": moved, "rest_ticks": int(rest.sum()), "joints": {},
           "estimate": "recomputed offline (gain 50)" if recompute else "the robot's own (t_ext)"}
    if "TCP_f_ext" in ch:
        out["tcp_force_peak_n"] = float(np.nanmax(np.linalg.norm(ch["TCP_f_ext"][:, :3], axis=1)))
    old = static_estimate(ch, n, gravity_fn)
    replays = {g: replay(ch, n, momentum_fn, g, friction, gravity_fn) for g in GAINS}
    no_fric = replay(ch, n, momentum_fn, 50.0, [0.0] * n)
    if moved is None:
        clean = None
    else:                                   # the moved joint moving, every other joint (nearly) still
        others = [k for k in range(n) if k != moved - 1]
        vm = np.abs(v[:, moved - 1])
        clean = (vm > MOVING_RAD_S) & np.all(np.abs(v[:, others]) < np.maximum(STILL_RAD_S, DOMINANT * vm)[:, None],
                                             axis=1)
    for j in range(n):
        mv = np.abs(v[:, j]) > MOVING_RAD_S if clean is None else clean
        off = float(np.median(est[rest, j])) if rest.any() else 0.0
        off_old = float(np.median(old[rest, j])) if rest.any() else 0.0
        off_nf = float(np.median(no_fric[rest, j])) if rest.any() else 0.0
        row = {"rest_offset": off if rest.any() else None,
               "rest_noise": float(np.std(est[rest, j])) if rest.sum() > 2 else None,
               "moving": _stats(est[mv, j] - off) if mv.any() else None,
               "moving_old": _stats(old[mv, j] - off_old) if mv.any() else None,
               "replay": {f"gain {g:g}": _stats(replays[g][mv, j] - (float(np.median(replays[g][rest, j])) if rest.any()
                                                                     else 0.0)) for g in GAINS} if mv.any() else None,
               "replay_no_friction": _stats(no_fric[mv, j] - off_nf) if mv.any() else None,
               "fit": fit_residual(est[:, j], v[:, j], acc[:, j]) if moved == j + 1 else None,
               "friction_model": float(friction[j]),
               "peak": float(np.nanmax(np.abs(est[:, j] - off))) if len(est) else None}
        out["joints"][f"J{j + 1}"] = row
    return out


def suggest(results, n):
    """Per joint, from the captures that moved a joint (its own and the others — a joint is shaken by its neighbours'
    motion too): contact ~ 2 x the largest moving p99, collision ~ the larger of 2 x the largest moving max and
    1.5 x contact — on the estimate minus its at-rest offset. Plus the offset and the friction its own fit suggests."""
    out = {}
    for j in range(n):
        rows = [r["joints"][f"J{j + 1}"] for r in results if r["moved"]]
        mov = [x["moving"] for x in rows if x["moving"]]
        if not mov:
            continue
        contact = 2.0 * max(m["p99"] for m in mov)
        collision = max(2.0 * max(m["max"] for m in mov), 1.5 * contact)
        own = [r["joints"][f"J{j + 1}"] for r in results if r["moved"] == j + 1]
        offsets = [x["rest_offset"] for x in rows if x["rest_offset"] is not None]
        s = {"contact_nm": round(contact, 2), "collision_nm": round(collision, 2),
             "offset_nm": round(float(np.median(offsets)), 3) if offsets else None}
        if own and own[0]["fit"]:
            a = own[0]["fit"][0]
            s["friction_lacking_nm"] = round(a, 3)
            s["friction_sliding_suggested_nm"] = round(max(0.0, own[0]["friction_model"] + a), 3)
        out[f"J{j + 1}"] = s
    return out


def report(results, n):
    lines = []
    for r in results:
        lines.append(f"\n=== {r['name']}: {r['ticks']} ticks, moved J{r['moved']}" if r["moved"] else
                     f"\n=== {r['name']}: {r['ticks']} ticks (push / rest): the peak per joint "
                     + ", ".join(f"J{j + 1} {r['joints'][f'J{j + 1}']['peak']:.2f}" for j in range(n)) + " Nm"
                     + (f"; the TCP force up to {r['tcp_force_peak_n']:.1f} N" if "tcp_force_peak_n" in r else ""))
        lines.append("  joint | at rest: offset  noise | moving |est-offset| p99   max | OLD (measured-gravity) p99   max | "
                     "no friction model p99 | its own fit: friction a  viscous b  inertia c")
        for j in range(n):
            x = r["joints"][f"J{j + 1}"]
            m, o, nf, fit = x["moving"], x["moving_old"], x["replay_no_friction"], x["fit"]
            fmt = lambda v: "   -  " if v is None else f"{v:6.3f}"                                  # noqa: E731
            lines.append(f"  J{j + 1}    |        {fmt(x['rest_offset'])} {fmt(x['rest_noise'])} | "
                         f"{fmt(m['p99'] if m else None)} {fmt(m['max'] if m else None)} | "
                         f"{fmt(o['p99'] if o else None)} {fmt(o['max'] if o else None)} | "
                         f"{fmt(nf['p99'] if nf else None)} | "
                         + (f"{fit[0]:+.3f} {fit[1]:+.3f} {fit[2]:+.4f}" if fit else "   -"))
    lines.append("\nPer joint (Nm) — suggestions only; thresholds act on the estimate MINUS the at-rest offset:")
    for k, s in suggest(results, n).items():
        fr = (f"; friction the model lacks {s['friction_lacking_nm']:+.3f} -> <friction_coulomb_nm> "
              f"{s['friction_sliding_suggested_nm']:.3f}" if "friction_lacking_nm" in s else "")
        off = f"offset at rest {s['offset_nm']:+.3f}; " if s.get("offset_nm") is not None else ""
        lines.append(f"  {k}: {off}contact {s['contact_nm']:.2f}, collision {s['collision_nm']:.2f}{fr}")
    return "\n".join(lines)


def analyse_folder(folder, recompute=False):
    from utility.incident_dump import load
    momentum_fn, gravity_fn, friction, robot = load_model()
    n = len(friction)
    results = []
    for path in sorted(glob.glob(os.path.join(folder, "*.npz"))):
        meta, ch = load(path)
        moved = None
        name = os.path.basename(path)
        if "_ext_j" in name:
            try:
                moved = int(name.split("_ext_j")[1][0])
            except ValueError:
                moved = None
        r = analyse_capture(ch, n, momentum_fn, gravity_fn, friction, moved, recompute)
        r["name"] = name
        results.append(r)
    text = ("the estimate: " + ("RECOMPUTED offline from the recorded inputs (today's code)" if recompute
                                else "the robot's own, as recorded") + "\n" + report(results, n))
    with open(os.path.join(folder, "report.json"), "w") as fh:
        json.dump({"results": results, "suggested": suggest(results, n)}, fh, indent=1)
    with open(os.path.join(folder, "report.txt"), "w") as fh:
        fh.write(text + "\n")
    print(text)
    print(f"\nwritten: {folder}/report.txt, report.json")
    return results


# ---------------------------------------------------------------------------------------------------------------
def robot_state():
    try:
        with open("/dev/shm/robot_state", "rb") as f:
            raw = f.read()
        return raw[4:4 + struct.unpack("<I", raw[:4])[0]].decode(errors="replace")
    except (OSError, struct.error):
        return None


def _capture_file(name, before, timeout=20.0):
    from project_paths import TEMP_DATA_DIR
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        new = set(glob.glob(os.path.join(str(TEMP_DATA_DIR), "captures", f"*_{name}.npz"))) - before
        if new:
            time.sleep(0.5)                       # the file is renamed into place when complete
            return sorted(new)[-1]
        time.sleep(0.2)
    return None


def run_arm(args):
    from _common import START_POSE_DEG, connect, finish, goto_start, prepare_exec
    from project_paths import TEMP_DATA_DIR
    robot = connect(args)
    folder = os.path.join(OUT_DIR, time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
    os.makedirs(folder, exist_ok=True)
    joints = [int(x) for x in args.joints.split(",") if x.strip()]
    speeds = [float(x) for x in args.speeds.split(",") if x.strip()]
    try:
        prepare_exec(robot, args)
        goto_start(robot, args)
        for j in joints:
            name = f"ext_j{j}"
            before = set(glob.glob(os.path.join(str(TEMP_DATA_DIR), "captures", f"*_{name}.npz")))
            robot.capture_telemetry(60.0, name)
            time.sleep(1.0)
            for sp in speeds:
                target = list(START_POSE_DEG)
                for sign in (+1.0, -1.0, 0.0):
                    target[j - 1] = START_POSE_DEG[j - 1] + sign * AMP_DEG[j]
                    print(f"  J{j} -> {target[j - 1]:.1f} deg at {sp:g} %")
                    robot.move_joints(list(target), speed=sp, timeout=args.timeout)
                    robot.wait_idle(timeout=args.timeout)
                    time.sleep(0.5)
            robot.stop_capture()
            path = _capture_file(name, before)
            if path is None:
                print(f"!!! no capture file for J{j} — see the activity log (CAPTURE_*)")
                continue
            shutil.copy(path, folder)
            print(f"  J{j}: {os.path.basename(path)}")
        if args.push:
            name = "ext_push"
            before = set(glob.glob(os.path.join(str(TEMP_DATA_DIR), "captures", f"*_{name}.npz")))
            print(f"\nPUSH: the arm holds still for {args.push:g} s — push the TOOL by hand, a few seconds each way "
                  f"(up, down, sideways), not hard.")
            robot.capture_telemetry(float(args.push), name)
            time.sleep(float(args.push) + 1.0)
            path = _capture_file(name, before, timeout=10.0)
            if path:
                shutil.copy(path, folder)
    finally:
        finish(robot)
    print(f"\nsaved: {folder}")
    if robot_state() == "DISABLED":
        analyse_folder(folder)
    else:
        print(f"The robot is not DISABLED: the analysis waits (it replays the observer over every tick — CPU that "
              f"slows RTI on this Pi). Disable the arm, then:\n  .venv/bin/python api_tests/76_external_torque.py "
              f"--reanalyse {os.path.relpath(folder, ROOT)}")
    return 0


# ---------------------------------------------------------------------------------------------------------------
def simulate(robot, seconds=6.0, moving_joint=1, amp=0.5, freq=0.4, friction=None, extra_friction=0.0,
             push=None, seed=3, tick=0.004, substeps=32, rest_s=0.0):
    """The PAR6 model simulated (Pinocchio forward dynamics, friction), a PD + gravity controller following a sine on
    one joint — run every sub-step, as the drives' own fast loops do — and the motor torque recorded once per tick as a
    capture holds it: {j_p, j_s_f, j_t, j_t_f, j_a_f, timestamp, tick}.
    friction: the ARM's friction — a FrictionList (THE model: Coulomb + load x |gravity| rounded through zero speed +
    viscous x speed; a plain list = Coulomb only) — + extra_friction (Coulomb) on every joint the model does not know;
    push: (t0, t1, joint (1-based), Nm) an external torque during [t0, t1); rest_s: the arm held still first."""
    import pinocchio as pin
    from utility.pinocchio_attach import build_pinocchio_dynamics
    model, data = build_pinocchio_dynamics(robot)
    n = model.nv
    rng = np.random.default_rng(seed)
    fc = np.asarray(list(friction) if friction is not None else [0.0] * n, dtype=float) + extra_friction
    fv = np.asarray(getattr(friction, "viscous", None) or [0.0] * n, dtype=float)
    fl = np.asarray(getattr(friction, "load", None) or [0.0] * n, dtype=float)
    q0 = np.radians([90.0, -105.0, 165.0, 0.0, -30.0, 180.0])[:n]
    q, v = q0.copy(), np.zeros(n)
    kp = np.array([400, 600, 400, 20, 20, 10], dtype=float)[:n]
    kd = np.array([20, 30, 20, 1.0, 1.0, 0.5], dtype=float)[:n]
    steps = int(round(seconds / tick))
    rec = {k: np.zeros((steps, n)) for k in ("j_p", "j_s_f", "j_t", "j_t_f", "j_a_f", "tau_ext")}
    rec["timestamp"] = np.zeros(steps)
    rec["tick"] = np.arange(steps, dtype=float)
    h = tick / substeps
    tau_m = np.zeros(n)
    tf = np.zeros(n)
    v_prev = v.copy()
    w = 2 * np.pi * freq
    for k in range(steps):
        t = k * tick
        ext = np.zeros(n)
        if push and push[0] <= t < push[1]:
            ext[push[2] - 1] = push[3]
        for i in range(substeps):
            ts = t + i * h
            qd = q0.copy()
            vd = np.zeros(n)
            if ts >= rest_s:
                qd[moving_joint - 1] += amp * np.sin(w * (ts - rest_s))
                vd[moving_joint - 1] = amp * w * np.cos(w * (ts - rest_s))
            g = pin.computeGeneralizedGravity(model, data, q)
            ga = np.abs(g)
            tau_m = kp * (qd - q) + kd * (vd - v) + g + (fc - extra_friction + fl * ga) * np.tanh(vd / 0.02) + fv * vd
            fr = (fc + fl * ga) * np.tanh(v / 0.02) + fv * v
            a = pin.aba(model, data, q, v, tau_m + ext - fr)
            v = v + a * h
            q = q + v * h
        meas_tau = tau_m + rng.normal(0, 0.02, n)
        tf = 0.8 * tf + 0.2 * meas_tau
        rec["j_p"][k], rec["j_s_f"][k] = q, v + rng.normal(0, 0.001, n)
        rec["j_t"][k], rec["j_t_f"][k] = meas_tau, tf
        rec["j_a_f"][k] = (v - v_prev) / tick
        rec["tau_ext"][k] = ext
        rec["timestamp"][k] = t
        v_prev = v.copy()
    return rec


def selftest():
    momentum_fn, gravity_fn, friction, robot = load_model()
    n = len(friction)
    fails = []

    def check(name, cond, detail=""):
        print(("  PASS  " if cond else "  FAIL  ") + name + ("" if cond else f"   [{detail}]"))
        if not cond:
            fails.append(name)

    rec = simulate(robot, seconds=5.0, moving_joint=2, amp=0.4, freq=0.5, friction=friction)
    rec["t_ext"] = replay(rec, n, momentum_fn, 50.0, friction, gravity_fn)
    r = analyse_capture(rec, n, momentum_fn, gravity_fn, friction, moved=2)
    j2 = r["joints"]["J2"]
    check("free motion (J2 +-23 deg at 0.5 Hz, the friction modelled): the estimate stays small (p99 < 0.15 Nm)",
          j2["moving"]["p99"] < 0.15, j2["moving"])
    check("... while the OLD static estimate reads the motion as force (p99 > 5 x the observer's)",
          j2["moving_old"]["p99"] > 5 * j2["moving"]["p99"], (j2["moving_old"], j2["moving"]))
    rec2 = simulate(robot, seconds=5.0, moving_joint=2, amp=0.4, freq=0.5, friction=friction, extra_friction=0.15)
    rec2["t_ext"] = replay(rec2, n, momentum_fn, 50.0, friction, gravity_fn)
    fit = analyse_capture(rec2, n, momentum_fn, gravity_fn, friction, moved=2)["joints"]["J2"]["fit"]
    check("no fit for a joint that did not move (it would be meaningless)",
          analyse_capture(rec2, n, momentum_fn, gravity_fn, friction, moved=2)["joints"]["J4"]["fit"] is None)
    check("friction the model lacks (0.15 Nm on J2) shows in the fit as a = +0.15 (+-0.03)",
          fit is not None and abs(fit[0] - 0.15) < 0.03, fit)
    rec3 = simulate(robot, seconds=4.0, moving_joint=1, amp=0.3, freq=0.5, friction=friction, push=(2.0, 3.0, 3, 1.5))
    est = replay(rec3, n, momentum_fn, 50.0, friction, gravity_fn)
    t = rec3["timestamp"]
    inside = (t > 2.15) & (t < 3.0)
    check("a 1.5 Nm push on J3 while J1 moves: read as -1.5 (the joint's reaction) +- 0.1 Nm after 0.15 s",
          abs(np.median(est[inside, 2]) + 1.5) < 0.1, np.median(est[inside, 2]))
    k = int(np.argmax(t >= 2.0))
    rise = int(np.argmax(est[k:, 2] < -0.63 * 1.5))
    check("... rising with the observer's 20 ms time constant (63 % within 3-8 ticks)", 3 <= rise <= 8, rise)
    check("... and not on the joints it does not push (J1 / J2 under 0.2 Nm then)",
          np.abs(est[inside, 0]).max() < 0.2 and np.abs(est[inside, 1]).max() < 0.2,
          (np.abs(est[inside, 0]).max(), np.abs(est[inside, 1]).max()))
    sug = suggest([r], n)
    check("thresholds are suggested from the moving numbers, collision above contact",
          "J2" in sug and 0 < sug["J2"]["contact_nm"] < sug["J2"]["collision_nm"], sug)
    rec4 = simulate(robot, seconds=5.0, moving_joint=2, amp=0.4, freq=0.5, friction=friction, extra_friction=0.15,
                    rest_s=1.0)
    rec4["t_ext"] = replay(rec4, n, momentum_fn, 50.0, friction, gravity_fn) + 0.8          # a constant offset (a gravity error)
    r4 = analyse_capture(rec4, n, momentum_fn, gravity_fn, friction, moved=2)
    check("a constant offset is reported apart and taken out of the moving numbers",
          abs(r4["joints"]["J2"]["rest_offset"] - 0.8) < 0.1 and r4["joints"]["J2"]["moving"]["p99"] < 0.6,
          (r4["joints"]["J2"]["rest_offset"], r4["joints"]["J2"]["moving"]))
    s4 = suggest([r4], n)["J2"]
    check("... and the friction the model lacks becomes a suggested <friction_sliding_nm> (model + 0.15)",
          abs(s4["friction_sliding_suggested_nm"] - (friction[1] + 0.15)) < 0.04, s4)
    # A capture keeps 5 s BEFORE its start: here J1's own motion (with 0.15 Nm of friction the model lacks — it reads
    # as torque) before J2's capture. J2's numbers must come from where J2 moves and the rest are still (2026-10-17).
    pre = simulate(robot, seconds=4.0, moving_joint=1, amp=0.4, freq=0.5, friction=friction, extra_friction=0.15)
    own = simulate(robot, seconds=5.0, moving_joint=2, amp=0.4, freq=0.5, friction=friction, rest_s=1.0)
    both = {k: np.concatenate([pre[k], own[k]]) for k in ("j_p", "j_s_f", "j_t", "j_t_f", "j_a_f")}
    both["timestamp"] = np.concatenate([pre["timestamp"], own["timestamp"] + pre["timestamp"][-1] + 0.004])
    both["tick"] = np.arange(len(both["timestamp"]), dtype=float)
    both["t_ext"] = replay(both, n, momentum_fn, 50.0, friction, gravity_fn)
    r5 = analyse_capture(both, n, momentum_fn, gravity_fn, friction, moved=2)
    vb = both["j_s_f"][:, :n]
    any_mv = np.any(np.abs(vb) > MOVING_RAD_S, axis=1)
    at_rest = np.all(np.abs(vb) < REST_RAD_S, axis=1)
    j1_all = float(np.percentile(np.abs(both["t_ext"][any_mv, 0] - np.median(both["t_ext"][at_rest, 0])), 99))
    check("a capture's pre-roll (another joint's own motion before it) is left out: J1 during J2's capture reads under "
          "0.05 Nm, where counting every moving tick reads its own missing friction (> 0.1)",
          r5["joints"]["J1"]["moving"]["p99"] < 0.05 and j1_all > 0.1, (r5["joints"]["J1"]["moving"], j1_all))
    print(f"\n{len(fails)} failed" + (f": {fails}" if fails else " - all passed"))
    return 1 if fails else 0


# ---------------------------------------------------------------------------------------------------------------
def main():
    from _common import parser
    p = parser("76: the external torque estimate on the arm — its floor when nothing touches it")
    p.add_argument("--joints", default="1,2,3,4,5,6")
    p.add_argument("--speeds", default="20,50,100", help="joint speed percents")
    p.add_argument("--push", type=float, default=0.0, help="seconds of a hand-push capture at the end (0 = none)")
    p.add_argument("--reanalyse", metavar="FOLDER", help="analyse saved captures (the arm DISABLED)")
    p.add_argument("--recompute", action="store_true",
                   help="with --reanalyse: recompute the estimate offline from the recorded inputs (an older RTI)")
    p.add_argument("--selftest", action="store_true", help="no arm: the PAR6 model simulated through the analysis")
    args = p.parse_args()
    if args.selftest:
        return selftest()
    if args.reanalyse:
        if robot_state() not in ("DISABLED", None):
            print("The robot is not DISABLED — the analysis loads the CPU; disable the arm first.")
            return 2
        analyse_folder(args.reanalyse, recompute=args.recompute)
        return 0
    if not args.go:
        print("DRY RUN: would move each joint of --joints +-its amplitude at --speeds and back, capturing every tick. "
              "Re-run with --go (workspace clear, nothing touching the arm).")
        return 0
    return run_arm(args)


if __name__ == "__main__":
    sys.exit(main())
