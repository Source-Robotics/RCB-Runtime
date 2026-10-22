#!/usr/bin/env python3
"""78: each joint's TORQUE SCALE calibrated with known weights hung at the TCP, the CAD gravity model checked with it,
and the external force estimate checked against the weights (TODO/friction_compensation_plan.md F4, 2026-10-17).

Why: a joint's torque is computed from its motor current as current x Kt x gear ratio x <gear_efficiency>, and the same
factor turns the gravity model's torque into the current RTI sends. The efficiencies are nominal (J2 0.90, J3 0.95, the
rest 0.95-0.98). 77's slides (friction cancels) found J2's gravity error growing with the load: measured = 0.946 x
model + 0.13 Nm — a 5.4 % SCALE, which a 10 g CAD error on a 6 kg arm (0.2 %) cannot make. A known weight is a reference
that needs no CAD mass: its torque on each joint is exactly m x g x dz_hook / dq (the kinematics alone).

How: the weights hang at the TCP (a short hook or loop held between the fingertips; `--hook` for another point, mm in
the tool frame). Each is DECLARED to the robot (set_payload, a point mass at the hook — statically the same as a weight
hanging from it) while it hangs, so gravity compensation carries it in every mode and the arm never sags in freedrive
between poses. At a few poses chosen to load J2 / J3 / J4 / J5 (the joints a vertical force at the TCP can twist: J1's
axis is vertical, J6's runs through the TCP), each of those joints slides both ways at a steady speed (54's slide, as
77: friction cancels): the drive's own correction, (t+ + t-) / 2 of measured - commanded torque, is
        k x (true gravity + weight) - (model gravity + declared weight)      (k = the joint's measured / true torque)
so against the same pose without a weight it changes by (k - 1) x the weight's torque: k per joint by least squares
over the weights and poses. The new <gear_efficiency> = the old / k. With k known, the no-weight error left against the
CAD model is the model's own (+ offsets, cables): reported per pose.

Then the force check (the original F4): with the largest weight still hanging, a slow EXEC path recorded three times —
declared (the estimate should read ~0), undeclared (it should read the weight: TCP force z = +m x g, the joints'
reaction sign), and without the weight (the reference).

`--apply` (with --reanalyse, the arm disabled) writes the confident joints' scale into the robot XML: <gear_efficiency>
= old / k AND every value that was measured in the old torque units divided by k — <friction_coulomb_nm>,
<friction_viscous_nm_s>, <friction_load_per_nm> (77), <contact_torque_nm>, <collision_torque_nm> (76). Text-level, a
backup first, refused for a joint this run already applied. RTI uses it from its next start.

Safety: dry run by default (the plan: poses, each joint's weight torque, the TCP's lowest point over the slides with the
weight's hang below it). Robot ENABLED, homed, IDLE, no payload declared; the space below the TCP clear for the weight.
Weights are hung and changed only while the arm HOLDS in EXEC (the tool asks). Weights up to the robot's
<max_payload_kg>. While an undeclared weight hangs, collision detection (warn) may warn — it is an external force; a
`stop` setting is set to `warn` for the run and put back.
    .venv/bin/python api_tests/78_weight_calibration.py --selftest
    .venv/bin/python api_tests/78_weight_calibration.py                        # dry run: the plan, nothing moves
    .venv/bin/python api_tests/78_weight_calibration.py --go                   # 0 / 50 g / 100 g / 0.5 kg
    .venv/bin/python api_tests/78_weight_calibration.py --reanalyse temp_data/weight_calibration/RUN        # disabled
    .venv/bin/python api_tests/78_weight_calibration.py --reanalyse temp_data/weight_calibration/RUN --apply
Results: temp_data/weight_calibration/<UTC>/run.json (+ the force-check captures) + report.txt.
"""
import argparse
import glob
import itertools
import json
import math
import os
import re
import shutil
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

G = 9.81                                  # the robot's dynamics model's g — what RTI subtracts for a declared weight
FS = 250.0
RECIPE = "friction_id"                    # j_p, j_s, j_t, j_tc, m_temp, tick, mode (77's)
WEIGHTS_KG = (0.0, 0.05, 0.1, 0.5)
SPEEDS_DPS = (10.0,)
JOINTS = (2, 3, 4, 5)                     # the joints a vertical force at the TCP can twist
MIN_SIGNAL_NM = 0.25                      # slide a joint at a pose only where the largest weight gives it this much
POSES_PER_JOINT = 2
HANG_M = 0.15                             # the hook + the weight below the TCP (m)
MIN_Z_M = 0.10                            # the weight's lowest point stays this high above the base (m)
LIMIT_MARGIN_DEG = 3.0                    # the joints not sliding stay this far inside their soft limits
HANG_CLEAR_M = 0.06                       # the hanging weight's line keeps this far from every link (a link's radius +)
HANG_SKIP_M = 0.03                        # ... below this much under the hook (the hook itself sits at the gripper)
MIN_K_SIGNAL_NM = 0.4                     # a joint's scale is trusted only where its largest weight torque reached this
K_RANGE = (0.8, 1.2)                      # a scale outside this is a measurement problem, never applied
MOVE_SPEED = 10                           # EXEC moves between poses, percent
FORCE_SPEED = 20                          # the force check's path, percent
# The candidate poses (deg) around the api_tests' start pose: J2 / J3 / J4 / J5 on these values, J1 / J6 at the start's.
GRID = {2: (-105.0, -90.0, -75.0), 3: (165.0, 195.0, 225.0), 4: (0.0, 90.0, -90.0),
        5: (-30.0, -55.0, -80.0, 30.0, 55.0, 80.0)}
SCALED_TAGS = ("friction_coulomb_nm", "friction_viscous_nm_s", "friction_load_per_nm", "contact_torque_nm",
               "collision_torque_nm")
MOVING_RAD_S = 0.02


def m77():
    return __import__("77_friction_id")


# ------------------------------------------------------------------------------------------------------------ the model
def hook_z_fn(robot, hook_m=(0.0, 0.0, 0.0)):
    """z (m, the base frame, up) of the hook point — hook_m in the TCP frame (the tool link's frame, rtb's fkine)."""
    h = np.asarray(hook_m, dtype=float)

    def z(q_rad):
        T = robot.fkine(np.asarray(q_rad, dtype=float))
        return float(T.t[2] + (np.asarray(T.R) @ h)[2])
    return z


def weight_torque(z_fn, pose_deg, mass_kg, h=1e-6):
    """The torque (Nm, per joint) holding a weight that hangs from the hook: m x g x dz_hook / dq (the gradient of its
    potential energy — the sign the gravity model's torque has: what the motors must add)."""
    q = np.radians(np.asarray(pose_deg, dtype=float))
    out = np.zeros(len(q))
    for j in range(len(q)):
        qp, qm = q.copy(), q.copy()
        qp[j] += h
        qm[j] -= h
        out[j] = mass_kg * G * (z_fn(qp) - z_fn(qm)) / (2.0 * h)
    return out


def hang_clearance(robot, pose_deg, hook_m=(0.0, 0.0, 0.0), hang=HANG_M, n=12):
    """The smallest distance (m) between the weight's hanging line (straight down from the hook, from HANG_SKIP_M to
    `hang` below it) and the parts of the arm BELOW the hook: the segments between consecutive link frames' origins and
    the flange -> TCP (the gripper). A weight resting on the arm would spoil the measurement silently; one hanging
    through the gripper (the tool pointing up) too. inf: nothing of the arm below the hook."""
    q = np.radians(np.asarray(pose_deg, dtype=float))
    frames = robot.fkine_all(q)
    pts = [np.asarray(T.t, dtype=float) for T in frames]
    T = robot.fkine(q)
    hook = np.asarray(T.t, dtype=float) + np.asarray(T.R) @ np.asarray(hook_m, dtype=float)
    pts.append(np.asarray(T.t, dtype=float))                     # the flange -> TCP: the gripper's body
    line = np.array([hook - np.array([0.0, 0.0, z]) for z in np.linspace(HANG_SKIP_M, hang, n)])
    best = float("inf")
    for a, b in zip(pts[:-1], pts[1:]):
        seg = np.array([a + (b - a) * t for t in np.linspace(0.0, 1.0, n)])
        seg = seg[seg[:, 2] < hook[2] - HANG_SKIP_M]           # only what lies BELOW the hook can touch the weight
        if len(seg):
            best = min(best, float(np.linalg.norm(line[:, None, :] - seg[None, :, :], axis=2).min()))
    return best


def speeds_plan(speeds):
    m = m77()
    return [{"v": v, "amp": m.amplitude_deg(v), "past": m.past_deg(v), "legs": m.legs_for(v)} for v in speeds]


def make_plan(lo_deg, hi_deg, start_deg, tcp_z, z_fn, weights=WEIGHTS_KG, joints=JOINTS, speeds=SPEEDS_DPS,
              min_z=MIN_Z_M, hang=HANG_M, per_joint=POSES_PER_JOINT, min_signal=MIN_SIGNAL_NM, clearance=None,
              torque_targets=None):
    """[{pose, joints, tau (per joint at the largest weight), z_min}] — per joint the poses (of GRID) where the largest
    weight twists it most: inside the soft limits (the sliding joint with the room its slide needs), the weight's lowest
    point over every slide at least min_z above the base, the weight hanging free (clearance(pose) >= HANG_CLEAR_M, when
    given); poses shared between joints merged. torque_targets (Nm): instead of the largest, per joint the pose whose
    torque is closest to each target (a scale is the same at half and full lever; a saturating motor is not)."""
    m = m77()
    room = m.room_needed_deg(speeds)
    reach = m.amplitude_deg(max(speeds)) + m.past_deg(max(speeds))
    wmax = max(weights)
    cands = []
    for combo in itertools.product(*(GRID[k] for k in (2, 3, 4, 5))):
        p = [float(v) for v in start_deg]
        for k, v in zip((2, 3, 4, 5), combo):
            p[k - 1] = float(v)
        if not all(lo_deg[i] + LIMIT_MARGIN_DEG <= p[i] <= hi_deg[i] - LIMIT_MARGIN_DEG for i in range(len(p))):
            continue
        if clearance is not None and clearance(p) < HANG_CLEAR_M:
            continue
        cands.append((p, weight_torque(z_fn, p, wmax)))
    chosen = {}
    for jj in joints:
        j = jj - 1
        ok = []
        for p, tau in cands:
            if abs(tau[j]) < min_signal or not (lo_deg[j] + room <= p[j] <= hi_deg[j] - room):
                continue
            z_lo, _ = m.tcp_z_range(tcp_z, p, j, reach)
            if z_lo - hang < min_z:
                continue
            ok.append((abs(tau[j]), p, tau, z_lo))
        if torque_targets:
            picks = []
            for t in torque_targets:
                for c in sorted(ok, key=lambda x: abs(x[0] - t)):
                    if all(c[1] != d[1] for d in picks):
                        picks.append(c)
                        break
        else:
            ok.sort(key=lambda x: -x[0])
            picks = ok[:per_joint]
        for _, p, tau, z_lo in picks:
            key = tuple(round(v, 3) for v in p)
            item = chosen.setdefault(key, {"pose": p, "joints": [], "tau": [float(x) for x in tau], "z_min": z_lo})
            item["joints"].append(jj)
            item["z_min"] = min(item["z_min"], z_lo)
    plan = [chosen[k] for k in sorted(chosen)]
    for item in plan:
        item["joints"].sort()
        item["speeds"] = speeds_plan(speeds)
    return plan


def plan_seconds(plan, weights=WEIGHTS_KG):
    per = sum(8.0 + sum(len(it["joints"]) * m77().slide_seconds(s) for s in it["speeds"]) for it in plan)
    return len(weights) * (per + 20.0)


def print_plan(plan, weights, missing, force=True):
    print("poses (deg) and the joints slid there; the largest weight's torque on each joint (Nm); the TCP's lowest point:")
    for it in plan:
        print("  " + " ".join(f"{x:6.1f}" for x in it["pose"]) + "   slide " + ", ".join(f"J{j}" for j in it["joints"])
              + "   weight torque " + " ".join(f"J{j + 1} {it['tau'][j]:+.2f}" for j in range(len(it["tau"])))
              + f"   TCP down to {1000 * it['z_min']:.0f} mm")
    if missing:
        print("  no pose found for: " + ", ".join(f"J{j}" for j in missing))
    print(f"weights {', '.join(f'{1000 * w:g} g' for w in weights)}"
          + ("; then the force check (the largest weight declared / undeclared / none)" if force else "; no force check")
          + f". About {plan_seconds(plan, weights) / 60.0:.0f} min + your weight changes.")


# ------------------------------------------------------------------------------------------------------------ analysis
def mean_correction(rec):
    """The drive's own correction at the pose: (t+ + t-) / 2 of measured - commanded torque (Nm) — friction cancels."""
    return 0.5 * (rec["t_plus"] + rec["t_minus"])


def analyse(records, z_fn, joints=JOINTS, min_k_signal=MIN_K_SIGNAL_NM):
    """records: 77's slide records + weight_kg, pose_deg, joint (1-based), v_cmd_dps. -> {joint: result}:
    k (the torque scale), its standard error, n, the largest weight torque, the residual RMS, k per weight, and per pose
    the no-weight error against the CAD model before / after the scale."""
    by = {}
    for r in records:
        if "t_plus" not in r:
            continue
        key = (int(r["joint"]), tuple(round(x, 3) for x in r["pose_deg"]), float(r["v_cmd_dps"]))
        by.setdefault(key, []).append(r)
    out = {}
    for jj in joints:
        j = jj - 1
        dd, tt, per_w, poses = [], [], {}, []
        for (kj, pose, v), recs in sorted(by.items()):
            zero = sorted((int(r.get("round", 0)), mean_correction(r), r) for r in recs if float(r["weight_kg"]) <= 0.0)
            if kj != jj or not zero:
                continue
            zr = [z[0] for z in zero]
            zv = [z[1] for z in zero]
            base = zv[0]
            poses.append({"pose": list(pose), "v_dps": v, "model_gravity": float(zero[0][2]["model_gravity"]),
                          "base": base, "drift": zv[-1] - zv[0] if len(zv) > 1 else None})
            for r in recs:
                w = round(float(r["weight_kg"]), 4)
                if w <= 0.0:
                    continue
                tau = float(weight_torque(z_fn, pose, w)[j])
                # the 0 g baseline at this record's round: interpolated between the 0 g rounds (constant with one)
                d = mean_correction(r) - float(np.interp(int(r.get("round", 0)), zr, zv))
                dd.append(d)
                tt.append(tau)
                if abs(tau) > 1e-3:
                    per_w.setdefault(w, []).append(1.0 + d / tau)
        res = {"n": len(dd), "poses": poses}
        if len(dd) >= 1 and float(np.sum(np.square(tt))) > 0.0:
            d, t = np.array(dd), np.array(tt)
            km1 = float(np.dot(d, t) / np.dot(t, t))
            left = d - km1 * t
            se = float(math.sqrt(float(np.sum(left ** 2)) / max(1, len(d) - 1) / float(np.dot(t, t)))) if len(d) > 1 \
                else float("nan")
            k = 1.0 + km1
            res.update(k=k, k_se=se, rms=float(np.sqrt(np.mean(left ** 2))), max_tau=float(np.max(np.abs(t))),
                       per_weight={f"{1000 * w:g} g": float(np.median(v)) for w, v in sorted(per_w.items())})
            res["confident"] = bool(res["max_tau"] >= min_k_signal and K_RANGE[0] <= k <= K_RANGE[1])
            for p in poses:                      # the CAD model's own error, before / after the scale
                g_true = (p["model_gravity"] + p["base"]) / k
                p["error_before"] = -p["base"]
                p["error_after"] = p["model_gravity"] - g_true
        out[jj] = res
    return out


def report(results, efficiencies, weights):
    lines = ["torque scale per joint from the hung weights: k = measured / true torque (1 = right); the new "
             "<gear_efficiency> = the old / k. Only --apply writes it."]
    for jj, r in results.items():
        if "k" not in r:
            lines.append(f"\nJ{jj}: no measurement")
            continue
        eff = efficiencies[jj - 1] if efficiencies is not None else None
        lines.append(f"\nJ{jj}: k {r['k']:.4f} +- {r['k_se']:.4f} from {r['n']} slides, the largest weight torque "
                     f"{r['max_tau']:.2f} Nm, what the line leaves {r['rms']:.3f} Nm"
                     + (f"; <gear_efficiency> {eff:g} -> {eff / r['k']:.4f}" if eff else "")
                     + ("" if r["confident"] else "  — NOT confident (too little torque or k out of range): not applied"))
        drifts = [p["drift"] for p in r["poses"] if p.get("drift") is not None]
        if drifts:
            lines.append("  the 0 g reading drifted over the run by " + ", ".join(f"{x:+.3f}" for x in drifts)
                         + " Nm (corrected: the baseline interpolated in time)")
        lines.append("  k per weight: " + ", ".join(f"{w} {k:.3f}" for w, k in r["per_weight"].items()))
        lines.append("  the CAD model's error per pose without a weight (model - true, Nm): "
                     + ", ".join(f"[{' '.join(f'{x:.0f}' for x in p['pose'])}] {p['error_before']:+.3f} -> "
                                 f"{p['error_after']:+.3f} (model {p['model_gravity']:+.2f})" for p in r["poses"]))
    return "\n".join(lines)


def force_stats(ch, n=6):
    """A force-check capture -> the TCP force (world, N) while the arm moves and at rest: medians and the spread."""
    v = ch["j_s_f"][:, :n]
    f = ch["TCP_f_ext"][:, :3]
    mv = np.any(np.abs(v) > MOVING_RAD_S, axis=1)
    rest = ~mv
    out = {}
    for name, m in (("moving", mv), ("rest", rest)):
        if m.sum() >= 10:
            out[name] = {"median": [float(x) for x in np.median(f[m], axis=0)],
                         "p95_spread_z": float(np.percentile(np.abs(f[m, 2] - np.median(f[m, 2])), 95)),
                         "n": int(m.sum())}
    return out


def force_report(stats, mass):
    if not {"declared", "undeclared", "none"} <= set(stats):
        return "force check: not complete"
    lines = [f"\nforce check, {1000 * mass:g} g hanging at the TCP (expected while undeclared: Fz = +{mass * G:.2f} N, "
             f"the joints' reaction sign; declared: ~0), TCP force in the base frame, minus the run without the weight:"]
    for part in ("moving", "rest"):
        if not all(part in stats[k] for k in stats):
            continue
        ref = np.array(stats["none"][part]["median"])
        for k in ("undeclared", "declared"):
            d = np.array(stats[k][part]["median"]) - ref
            lines.append(f"  {part:6s} {k:10s}: dF = ({d[0]:+.2f}, {d[1]:+.2f}, {d[2]:+.2f}) N"
                         + (f" — {d[2] / (mass * G):.3f} x m g" if k == "undeclared" else "")
                         + f"; spread (p95, z) {stats[k][part]['p95_spread_z']:.2f} N")
    return "\n".join(lines)


# ------------------------------------------------------------------------------------------------------------ --apply
def _line_re(tag):
    return re.compile(rf"^([ \t]*)<{tag}>([^<]*)</{tag}>([^\n]*)\n", flags=re.M)


def select_scales(results, joints):
    """{joint: k} for the joints NAMED (--joints): each must have a confident k, else ValueError — nothing is written
    for a joint the user did not choose (2026-10-17: J4 / J5 read high only under the largest weight)."""
    out = {}
    for jj in joints:
        r = results.get(jj) or {}
        if not r.get("confident"):
            raise ValueError(f"J{jj}: no confident scale in this run" + (f" (k {r['k']:.4f}, the largest weight "
                                                                        f"torque {r['max_tau']:.2f} Nm)" if "k" in r else ""))
        out[jj] = float(r["k"])
    return out


def write_scale_xml(xml_path, scales, backup_dir, run_name):
    """scales {joint 1-based: k}: <gear_efficiency> = old / k, every SCALED_TAGS value / k, each line marked with the run
    (a joint this run already applied is skipped — never scaled twice). Text-level; must parse; backup first.
    Returns (backup or None, applied joints, skipped joints)."""
    import xml.etree.ElementTree as ET
    with open(xml_path) as fh:
        text = fh.read()
    day = time.strftime("%Y-%m-%d", time.gmtime())
    marker = f"78 {run_name}".replace("--", "- -")
    applied, skipped = [], []
    for jj, k in sorted(scales.items()):
        m = re.search(rf'<joint id="{jj - 1}">(.*?)</joint>', text, flags=re.S)
        if m is None:
            raise ValueError(f"{os.path.basename(xml_path)}: no <joint id=\"{jj - 1}\"> block")
        body = m.group(1)
        if marker in body:
            skipped.append(jj)
            continue
        e = _line_re("gear_efficiency").search(body)
        if e is None:
            raise ValueError(f"joint {jj}: no <gear_efficiency>")
        old = float(e.group(2))
        body = body[:e.start()] + (f"{e.group(1)}<gear_efficiency>{old / k:.4f}</gear_efficiency> <!-- calibrated with "
                                   f"hung weights ({marker}, {day}): was {old:g}, the torque scale k {k:.4f} (measured / "
                                   f"true) — Kt x ratio x this is what counts -->\n") + body[e.end():]
        for tag in SCALED_TAGS:
            lm = _line_re(tag).search(body)
            if lm is None:
                continue
            val = float(lm.group(2)) / k
            digits = 2 if tag.endswith("torque_nm") else 3
            rest = lm.group(3)
            note = f" [/{k:.4f}: the torque scale of {marker}]"
            rest = (rest[:rest.rindex("-->")].rstrip() + note + " -->") if "-->" in rest else f" <!--{note} -->"
            body = body[:lm.start()] + f"{lm.group(1)}<{tag}>{val:.{digits}f}</{tag}>{rest}\n" + body[lm.end():]
        text = text[:m.start(1)] + body + text[m.end(1):]
        applied.append(jj)
    ET.fromstring(text)
    backup = None
    if applied:
        if backup_dir is not None:
            backup = os.path.join(str(backup_dir), f"robot_xml_backup_{time.strftime('%Y%m%d_%H%M%S', time.gmtime())}_"
                                                   f"{os.path.basename(xml_path)}")
            shutil.copy2(xml_path, backup)
        with open(xml_path, "w") as fh:
            fh.write(text)
    return backup, applied, skipped


# ------------------------------------------------------------------------------------------------------------ self-test
def selftest():
    bad = []

    def chk(name, cond, detail=""):
        print(("  PASS  " if cond else "  FAIL  ") + name + ("" if cond else f"   [{detail}]"))
        if not cond:
            bad.append(name)

    cfg, sysc, robot = load_model()
    z_tcp = hook_z_fn(robot)
    print("A. the weight's torque")
    from utility.pinocchio_attach import _make_pinocchio_gravity, build_pinocchio_dynamics
    from utility.payload import PayloadModel
    from utility.robot_factory import create_robot
    r2 = create_robot(cfg["robot_name"], gripper_name=cfg["gripper_name"])
    g0 = _make_pinocchio_gravity(*build_pinocchio_dynamics(r2))
    rng = np.random.default_rng(4)
    worst = {}
    for hook in ((0.0, 0.0, 0.0), (0.02, -0.01, 0.03)):
        pm = PayloadModel(r2)
        pm.apply(0.5, hook)
        gw = _make_pinocchio_gravity(*build_pinocchio_dynamics(r2))
        zf = hook_z_fn(robot, hook)
        w = 0.0
        for _ in range(20):
            q = np.radians(rng.uniform(-150, 150, 6))
            q[1], q[2] = math.radians(rng.uniform(-130, -60)), math.radians(rng.uniform(150, 240))
            ref = np.asarray(gw(q)) - np.asarray(g0(q))
            w = max(w, float(np.abs(weight_torque(zf, np.degrees(q), 0.5) - ref).max()))
        pm.restore()
        worst[hook] = w
    chk("m g dz_hook/dq = the payload model's added gravity (a point mass at the hook, the robot's own set_payload "
        "path), 20 random poses, the hook at the TCP and off it", max(worst.values()) < 1e-5, worst)
    start = __import__("_common").START_POSE_DEG
    tau = weight_torque(z_tcp, start, 0.5)
    chk("J1 feels nothing of a hanging weight (its axis is vertical); the weight pulls J2 / J3 hardest at the start pose",
        abs(tau[0]) < 1e-6 and abs(tau[1]) > 1.0 and abs(tau[1]) >= abs(tau[4]), np.round(tau, 3))

    print("B. the plan on the real model")
    lo = [math.degrees(x) for x in cfg["software_joint_limit_negative"]]
    hi = [math.degrees(x) for x in cfg["software_joint_limit_positive"]]
    plan = make_plan(lo, hi, start, z_tcp, z_tcp, clearance=lambda p: hang_clearance(robot, p))
    covered = sorted({j for it in plan for j in it["joints"]})
    chk("every calibrated joint (J2 / J3 / J4 / J5) has a pose", covered == [2, 3, 4, 5], covered)
    room = m77().room_needed_deg(SPEEDS_DPS)
    good = all(lo[j - 1] + room <= it["pose"][j - 1] <= hi[j - 1] - room for it in plan for j in it["joints"])
    good = good and all(lo[i] + LIMIT_MARGIN_DEG <= it["pose"][i] <= hi[i] - LIMIT_MARGIN_DEG for it in plan
                        for i in range(6))
    chk("...inside the soft limits, the sliding joint with its slide's room", good)
    chk("...the weight's lowest point over the slides above the minimum height, each joint's torque above the minimum",
        all(it["z_min"] - HANG_M >= MIN_Z_M and all(abs(it["tau"][j - 1]) >= MIN_SIGNAL_NM for j in it["joints"])
            for it in plan), [(it["z_min"], it["tau"]) for it in plan])
    chk("...and the weight hangs free at every pose (its line >= 6 cm from every link and the gripper)",
        all(hang_clearance(robot, it["pose"]) >= HANG_CLEAR_M for it in plan),
        [round(hang_clearance(robot, it["pose"]), 3) for it in plan])
    blocked = plan[0]["pose"]
    plan_b = make_plan(lo, hi, start, z_tcp, z_tcp,
                       clearance=lambda p: 0.0 if [round(x, 3) for x in p] == [round(x, 3) for x in blocked] else 1.0)
    chk("a pose where the weight would touch the arm is never used (the clearance check rejects it)",
        all([round(x, 3) for x in it["pose"]] != [round(x, 3) for x in blocked] for it in plan_b) and plan_b,
        [it["pose"] for it in plan_b])
    up = list(start)
    up[4] = 80.0                                 # J5 bent the other way: the tool points up, the TCP above the flange
    qa = np.radians(up)
    flange_z, tcp_up = float(robot.fkine_all(qa)[-1].t[2]), float(robot.fkine(qa).t[2])
    chk("a weight that would hang through the gripper (the tool pointing up) is caught",
        tcp_up <= flange_z or hang_clearance(robot, up) < HANG_CLEAR_M, (tcp_up, flange_z, hang_clearance(robot, up)))

    plan_t = make_plan(lo, hi, start, z_tcp, z_tcp, joints=(4, 5), clearance=lambda p: hang_clearance(robot, p),
                       torque_targets=(0.35, 0.70))
    got = {j: sorted(abs(it["tau"][j - 1]) for it in plan_t if j in it["joints"]) for j in (4, 5)}
    chk("torque targets 0.35 / 0.70 Nm (half / full lever): J4 and J5 each get one pose near each",
        all(len(v) == 2 and abs(v[0] - 0.35) < 0.1 and abs(v[1] - 0.70) < 0.1 for v in got.values()), got)

    print("C. slides on simulated joints (77's plant: friction, a CAD error, a torque scale)")
    m = m77()
    sp = speeds_plan(SPEEDS_DPS)

    def fric(v):                                             # 77's J2-like joint: heavy, 0.60 + 0.11 x v
        return 0.60 + 0.11 * v

    recs = []
    k_true, g_model, cad = 0.946, -2.0, 0.08                 # J2-like: 5.4 % low, 0.08 Nm of CAD error
    lever = 0.45
    for w in WEIGHTS_KG:
        tau_w = w * G * lever
        g_true = g_model - cad
        # measured - commanded = k (true gravity + weight + friction) - (model gravity + declared weight): 77's plant
        # holds e (a push besides the drive) and its friction; its commanded is g_model + the declared weight
        e = (g_model + tau_w) - k_true * (g_true + tau_w)
        send, read, pace, st = m._plant(1, lambda v: k_true * fric(v), e, x0=0.5, inertia=0.05,
                                        g_model=g_model + tau_w)
        out, why = m.slide_pose(1, sp, send, read, pace)
        for r in out:
            r.update(joint=2, pose_deg=[0.0] * 6, weight_kg=w)
            recs.append(r)
    zf_lever = lambda q: lever * math.sin(q[1])                                  # noqa: E731  (a lever along J2)
    res = analyse(recs, zf_lever, joints=(2,))[2]
    chk("k recovered from the slides within 0.5 % (true 0.946)", "k" in res and abs(res["k"] - k_true) < 0.005,
        res.get("k"))
    chk("...the CAD error left after the scale: 0.08 Nm (before it: the scale and the CAD mixed)",
        "k" in res and abs(res["poses"][0]["error_after"] - cad) < 0.02, res.get("poses"))

    print("D. the analysis on synthetic records")
    def synthetic(weights, k=0.95):
        out = []
        for w in weights:
            for pose_off in (0.0, 0.3):
                pose = [0.0, math.degrees(0.3 + pose_off), 0.0, 0.0, 0.0, 0.0]
                tau = float(weight_torque(zf_lever, pose, w)[1])
                corr = -0.05 + (k - 1.0) * tau
                out.append({"joint": 2, "pose_deg": pose, "v_cmd_dps": 10.0, "weight_kg": w, "t_plus": corr + 0.5,
                            "t_minus": corr - 0.5, "model_gravity": -1.0})
        return out
    syn = synthetic(WEIGHTS_KG)
    res = analyse(syn, zf_lever, joints=(2,))[2]
    chk("a scale of 0.95 exactly, every weight's own k 0.95, nothing left over",
        abs(res["k"] - 0.95) < 1e-6 and res["rms"] < 1e-6 and all(abs(v - 0.95) < 1e-6 for v in res["per_weight"].values()),
        res)
    res_w = analyse(synthetic([w * 0.1 for w in WEIGHTS_KG]), zf_lever, joints=(2,))[2]
    chk("too little torque (the largest weight under 0.4 Nm on the joint): not confident — though its k is right",
        res_w["confident"] is False and abs(res_w["k"] - 0.95) < 1e-6 and res_w["max_tau"] < MIN_K_SIGNAL_NM, res_w)
    drifted = []
    for rnd, w in enumerate((0.0, 0.15, 0.5, 0.0)):                 # the reading drifts 0.03 Nm a round
        for r in synthetic([w]):
            r = dict(r, round=rnd)
            r["t_plus"] += 0.03 * rnd
            r["t_minus"] += 0.03 * rnd
            drifted.append(r)
    res_d = analyse(drifted, zf_lever, joints=(2,))[2]
    chk("a drifting reading with a 0 g round at both ends: k still exactly 0.95 (the baseline interpolated in time), the "
        "drift reported (+0.09 Nm)", abs(res_d["k"] - 0.95) < 1e-6
        and all(abs(p["drift"] - 0.09) < 1e-9 for p in res_d["poses"]), res_d)
    res_o = analyse(synthetic(WEIGHTS_KG, k=0.7), zf_lever, joints=(2,))[2]
    chk("a scale out of range (0.7): not confident — a measurement problem, never applied", res_o["confident"] is False,
        res_o.get("k"))
    text = report({2: res}, [0.98, 0.90, 0.95, 0.98, 0.98, 0.95], WEIGHTS_KG)
    chk("the report: k, the new efficiency 0.90 -> 0.9474, the CAD error per pose",
        "k 0.9500" in text and "0.9 -> 0.9474" in text and "model - true" in text, text[:300])

    try:
        sel = select_scales({2: res, 4: res_w}, [2])
        refused = False
        try:
            select_scales({2: res, 4: res_w}, [2, 4])
        except ValueError:
            refused = True
        chk("--apply writes only the joints named (J2), and refuses a named joint without a confident scale (J4 here)",
            sel == {2: res["k"]} and refused and select_scales({2: res}, []) == {}, sel)
    except ValueError as e:
        chk("--apply writes only the joints named", False, e)

    print("E. --apply on a fixture")
    import tempfile
    tmp = tempfile.mkdtemp(prefix="rcb78_", dir=os.environ.get("RCB_TMPDIR") or None)
    try:
        path = os.path.join(tmp, "ROBOT.xml")
        joint = """        <joint id="{i}">
            <name>J{n}</name> <!-- joint {n} -->
            <gear_efficiency>{eff}</gear_efficiency> <!-- Gear efficiency -->
            <friction_coulomb_nm>0.578</friction_coulomb_nm> <!-- 77 -->
            <friction_viscous_nm_s>0.625</friction_viscous_nm_s> <!-- 77 -->
            <friction_load_per_nm>0.041</friction_load_per_nm> <!-- 77 -->
            <contact_torque_nm>2.82</contact_torque_nm> <!-- 76 -->
            <collision_torque_nm>4.24</collision_torque_nm> <!-- 76 -->
        </joint>
"""
        with open(path, "w") as fh:
            fh.write("<robot>\n    <joints>\n" + "".join(joint.format(i=i, n=i + 1, eff=e)
                                                       for i, e in enumerate((0.98, 0.90, 0.95))) + "    </joints>\n</robot>\n")
        before = open(path).read()
        backup, applied, skipped = write_scale_xml(path, {2: 0.95}, tmp, "run--1")
        after = open(path).read()
        import xml.etree.ElementTree as ET
        j2 = [e for e in ET.fromstring(after).iter("joint") if e.get("id") == "1"][0]
        vals = {t: float(j2.find(t).text) for t in ("gear_efficiency",) + SCALED_TAGS}
        chk("J2: efficiency 0.90 / 0.95 = 0.9474, the friction and thresholds / 0.95",
            abs(vals["gear_efficiency"] - 0.9474) < 1e-9 and abs(vals["friction_coulomb_nm"] - 0.608) < 1e-9
            and abs(vals["friction_load_per_nm"] - 0.043) < 1e-9 and abs(vals["contact_torque_nm"] - 2.97) < 1e-9
            and abs(vals["collision_torque_nm"] - 4.46) < 1e-9 and applied == [2], vals)
        bl, al = before.splitlines(), after.splitlines()
        changed = [i for i, (a, b) in enumerate(zip(bl, al)) if a != b]
        j2_lines = [i for i, line in enumerate(bl) if 'joint id="1"' in line]
        chk("...text-level: only J2's six lines changed (same line count), every other line kept",
            len(bl) == len(al) and len(changed) == 6 and all(j2_lines[0] < i < j2_lines[0] + 9 for i in changed),
            changed)
        chk("...a backup of the old file", backup is not None and os.path.exists(backup) and open(backup).read() == before)
        b2, a2, s2 = write_scale_xml(path, {2: 0.95}, None, "run--1")
        chk("...the same run applied again: refused for that joint (never scaled twice), the file unchanged",
            a2 == [] and s2 == [2] and open(path).read() == after and b2 is None)
        chk("...no '--' in any XML comment", "--" not in "".join(re.findall(r"<!--(.*?)-->", after, flags=re.S)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("F. the force check's numbers")
    n = 400
    v = np.zeros((n, 6))
    v[100:300, 0] = 0.3
    mk = lambda fz: {"j_s_f": v, "TCP_f_ext": np.column_stack([np.full(n, 0.1), np.zeros(n), np.full(n, fz),  # noqa: E731
                                                              np.zeros((n, 3))])}
    st = {"none": force_stats(mk(0.3)), "undeclared": force_stats(mk(0.3 + 0.5 * G)), "declared": force_stats(mk(0.32))}
    text = force_report(st, 0.5)
    chk("the weight's force against the run without it: +m g = 1.000 x m g undeclared, ~0 declared",
        f"{0.5 * G:+.2f}) N — 1.000 x m g" in text and "declared  : dF = (+0.00, +0.00, +0.02)" in text, text)
    print(f"\n{len(bad)} failed" + (f": {bad}" if bad else " - all passed"))
    return 1 if bad else 0


# ------------------------------------------------------------------------------------------------------------ the arm
def load_model():
    from config import xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from utility.robot_factory import create_robot
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    return cfg, sysc, create_robot(cfg["robot_name"], gripper_name=cfg["gripper_name"])


def robot_state():
    return __import__("76_external_torque").robot_state()


def run_arm(args, cfg, sysc, plan, hook, weights):
    from robotclient import RobotClient
    from _rti_session import rti_session, Pacer
    from project_paths import TEMP_DATA_DIR
    base = __import__("46_monitor_baseline")
    e76 = __import__("76_external_torque")
    m = m77()
    out_dir = TEMP_DATA_DIR / "weight_calibration" / time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    out_dir.mkdir(parents=True, exist_ok=True)
    run = {"robot": cfg.get("robot_name"), "gripper": sysc.get("active_gripper"), "hook_m": list(hook),
           "weights_kg": list(weights), "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "efficiency": [float(x) for x in cfg["gear_efficiency"]], "plan": plan, "slides": [], "force": {}}

    def save():
        with open(out_dir / "run.json", "w") as fh:
            json.dump(run, fh)

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed, prev = False, {}
    declared = [0.0]
    try:
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") != "IDLE":
            print(f"Robot must be ENABLED, homed and in IDLE (got state={st.get('state')} homed={st.get('homed')} "
                  f"mode={st.get('mode')}).")
            return 2
        pay = st.get("payload") or {}
        if float(pay.get("mass_kg", pay.get("mass", 0.0)) or 0.0) > 0.0:
            print(f"A payload is declared ({pay}): set_payload(0) and nothing hanging first.")
            return 2
        base.claim_control(robot)
        claimed = True
        prev = {"recipe": st.get("recipe"), "ff": bool(st.get("friction_ff")), "coll": st.get("collision_detection")}
        if prev["ff"]:
            robot.set_friction_ff(False)
        if prev["coll"] == "stop":
            robot.set_collision_detection("warn")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        robot.set_gravity_comp(True)

        def hold_at(pose):
            robot.enter_exec_mode(wait=True)
            robot.move_joints(*[float(x) for x in pose], speed=args.move_speed)
            return robot.wait_idle(timeout=120.0, raise_on_error=False)

        def declare(w):
            robot.set_payload(float(w), [float(x) for x in hook])
            declared[0] = float(w)
            time.sleep(0.3)

        def ok_to_go():
            s = robot.status()
            errs = [e for e in (s.get("errors") or []) if not e.endswith("_WARNING")]
            if errs:
                return f"errors: {errs}"
            return None if s.get("mode") == "RTI" else f"the robot left RTI (mode {s.get('mode')})"

        first = plan[0]["pose"]
        for r_idx, w in enumerate(weights):
            fails = hold_at(first)
            if fails:
                print(f"the move to the first pose was refused: {str(fails)[:160]}")
                return 2
            what = (f"ONLY the {1000 * w:g} g weight at the TCP (take the previous one off)" if w > 0
                    else "NOTHING at the TCP")
            input(f"\n== the arm HOLDS in EXEC. Hang {what}, let it hang still, then press Enter ")
            declare(w)
            for it in plan:
                fails = hold_at(it["pose"])
                if fails:
                    print(f"  pose {it['pose']}: the move was refused — skipped: {str(fails)[:160]}")
                    continue
                robot.enter_idle_mode(wait=True)
                time.sleep(0.5)
                with rti_session(robot) as (rti, _first):
                    pace = Pacer(FS)
                    for jj in it["joints"]:
                        recs, why = m.slide_pose(jj - 1, it["speeds"], rti.send_joint_velocity, rti.read_state, pace,
                                                 ok_to_go)
                        for r in recs:
                            r.update(joint=jj, pose_deg=it["pose"], weight_kg=w, round=r_idx, t=time.time())
                            run["slides"].append(r)
                            print(f"  {1000 * w:5g} g  J{jj} at [{' '.join(f'{x:.0f}' for x in it['pose'])}]: "
                                  + (f"correction {mean_correction(r):+.3f} Nm, friction {r['friction']:.3f}"
                                     if "t_plus" in r else f"no number — {r.get('why')}"))
                        if why and "stick-slip" not in why:
                            print(f"  stopped: {why}")
                            save()
                            return 2
                save()
        if not args.no_force_check and weights[-1] == max(weights) > 0.0:   # it must still hang
            mass = max(weights)
            from project_paths import TEMP_DATA_DIR as TD
            fpose = list(first)
            targets = []                      # J1 +-30 deg (the tool keeps its tilt), then J2 / J3 together, and back
            for d1 in (30.0, -30.0, 0.0):
                p = list(fpose)
                p[0] = fpose[0] + d1
                targets.append(p)
            p = list(fpose)
            p[1], p[2] = fpose[1] + 8.0, fpose[2] + 8.0
            targets += [p, list(fpose)]

            def capture_path(name):
                hold_at(fpose)
                before = set(glob.glob(os.path.join(str(TD), "captures", f"*_{name}.npz")))
                robot.capture_telemetry(60.0, name)
                time.sleep(1.0)
                for t in targets:
                    robot.move_joints(*t, speed=FORCE_SPEED)
                    robot.wait_idle(timeout=120.0, raise_on_error=False)
                    time.sleep(0.8)
                robot.stop_capture()
                f = e76._capture_file(name, before)
                if f:
                    shutil.copy(f, out_dir)
                    run["force"][name.split("_")[-1]] = os.path.basename(f)
                    print(f"  force check '{name}': {os.path.basename(f)}")
                save()

            print(f"\nforce check with the {1000 * mass:g} g still hanging: declared")
            capture_path("w78_declared")
            hold_at(fpose)
            declare(0.0)
            print("...undeclared (collision detection may warn: the weight is an external force now)")
            capture_path("w78_undeclared")
            hold_at(fpose)
            input("\n== the arm HOLDS in EXEC. Take the weight OFF, then press Enter ")
            capture_path("w78_none")
        elif declared[0] > 0.0:
            hold_at(first)
            input("\n== the arm HOLDS in EXEC. Take the weight OFF, then press Enter ")
            declare(0.0)
    finally:
        if declared[0] > 0.0:
            print(f"!! {1000 * declared[0]:g} g still DECLARED: with the arm holding (EXEC), take the weight off, then "
                  f"robot.set_payload(0) (an RTI restart also clears it)")
        if claimed:
            try:
                if prev.get("recipe"):
                    robot.set_recipe(prev["recipe"])
                if prev.get("ff"):
                    robot.set_friction_ff(True)
                if prev.get("coll") == "stop":
                    robot.set_collision_detection("stop")
                robot.release_control()
            except Exception as e:                                  # noqa: BLE001 — report and go on
                print(f"!! could not restore everything: {e}")
        robot.disconnect()
    save()
    print(f"\nsaved {os.path.relpath(out_dir, _ROOT)}/run.json")
    if robot_state() == "DISABLED":
        reanalyse(str(out_dir))
    else:
        print(f"Not analysed yet (the arm is enabled). Disable it, then:\n  cd ~/Desktop/RCB-Runtime && .venv/bin/python "
              f"api_tests/78_weight_calibration.py --reanalyse {os.path.relpath(out_dir, _ROOT)}")
    return 0


def reanalyse(folder, apply=False, apply_joints=None):
    from utility.incident_dump import load
    with open(os.path.join(folder, "run.json")) as fh:
        run = json.load(fh)
    cfg, sysc, robot = load_model()
    z_fn = hook_z_fn(robot, run.get("hook_m", (0.0, 0.0, 0.0)))
    results = analyse(run["slides"], z_fn)
    text = report(results, run.get("efficiency"), run.get("weights_kg"))
    stats = {}
    for k, name in (run.get("force") or {}).items():
        p = os.path.join(folder, name)
        if os.path.exists(p):
            stats[k] = force_stats(load(p)[1])
    if stats:
        text += "\n" + force_report(stats, max(run.get("weights_kg") or [0.0]))
    with open(os.path.join(folder, "report.txt"), "w") as fh:
        fh.write(text + "\n")
    print(text + f"\n\nsaved {folder}/report.txt")
    if apply:
        try:
            scales = select_scales(results, apply_joints or [])
        except ValueError as e:
            print(f"--apply: {e} — nothing written")
            return 2
        if not scales:
            print("--apply needs --joints (the joints to write, e.g. --joints 2) — nothing written")
            return 2
        backup, applied, skipped = write_scale_xml(str(sysc["robot_xml_path"]), scales, folder,
                                                   os.path.basename(os.path.normpath(folder)))
        print(f"\nwritten into {sysc['robot_xml_path']}: " + (", ".join(f"J{j}" for j in applied) or "nothing")
              + (f"; already applied by this run: {', '.join(f'J{j}' for j in skipped)}" if skipped else "")
              + (f" (the old file: {backup})" if backup else "") + ". RTI uses it from its next start (restart + re-home).")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--weights", default=",".join(f"{w:g}" for w in WEIGHTS_KG),
                    help="kg, in order (0 first; may repeat, e.g. 0,0.15,0.5,0 — a closing 0 corrects the drift)")
    ap.add_argument("--joints", default=None, help="the joints to calibrate (default 2,3,4,5); with --apply: the joints "
                                                   "to WRITE (required)")
    ap.add_argument("--torques", default=None, help="Nm: per joint the poses whose largest-weight torque is closest to "
                                                    "each (e.g. 0.35,0.7 — half and full lever)")
    ap.add_argument("--hook", default="0,0,0", help="mm in the TCP frame: where the weight hangs (default the TCP)")
    ap.add_argument("--hang", type=float, default=HANG_M, help="m the hook + weight reach below the hook (default 0.15)")
    ap.add_argument("--min-z", type=float, default=MIN_Z_M)
    ap.add_argument("--move-speed", type=float, default=MOVE_SPEED)
    ap.add_argument("--no-force-check", action="store_true")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--reanalyse", metavar="FOLDER")
    ap.add_argument("--apply", action="store_true", help="with --reanalyse: write the confident joints' scale")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.reanalyse:
        if args.apply and robot_state() not in ("DISABLED", None):
            print("--apply: disable the arm first.")
            return 2
        try:
            aj = [int(x) for x in args.joints.split(",") if x.strip()] if args.joints else None
        except ValueError:
            print("--joints: joint numbers, comma separated")
            return 2
        return reanalyse(args.reanalyse, apply=args.apply, apply_joints=aj)
    if args.apply:
        print("--apply works on a saved run: --reanalyse FOLDER --apply --joints 2")
        return 2
    try:
        weights = [float(x) for x in args.weights.split(",") if x.strip()]
        hook = [float(x) / 1000.0 for x in args.hook.split(",")]
        joints = tuple(int(x) for x in args.joints.split(",") if x.strip()) if args.joints else JOINTS
        torques = [float(x) for x in args.torques.split(",") if x.strip()] if args.torques else None
    except ValueError:
        print("--weights: kg, comma separated; --hook: x,y,z mm; --joints: numbers; --torques: Nm")
        return 2
    if not set(joints) <= set(JOINTS):
        print(f"--joints: only {', '.join(f'J{j}' for j in JOINTS)} can be calibrated with a hung weight")
        return 2
    if len(hook) != 3 or not weights or weights[0] != 0.0 or any(w < 0 for w in weights):
        print("--hook needs x,y,z; --weights must start with 0")
        return 2
    cfg, sysc, robot = load_model()
    wmax = float(cfg.get("max_payload_kg") or cfg.get("safety", {}).get("max_payload_kg", 0.75) or 0.75)
    if max(weights) > wmax:
        print(f"the largest weight {max(weights):g} kg is above the robot's max payload {wmax:g} kg")
        return 2
    start = __import__("_common").START_POSE_DEG
    lo = [math.degrees(x) for x in cfg["software_joint_limit_negative"]]
    hi = [math.degrees(x) for x in cfg["software_joint_limit_positive"]]
    plan = make_plan(lo, hi, start, hook_z_fn(robot), hook_z_fn(robot, hook), weights, joints=joints, min_z=args.min_z,
                     hang=args.hang, clearance=lambda p: hang_clearance(robot, p, hook, args.hang), torque_targets=torques)
    missing = [j for j in joints if j not in {jj for it in plan for jj in it["joints"]}]
    print(f"weight calibration on {cfg.get('robot_name')}: {len(plan)} poses")
    print_plan(plan, weights, missing, force=not args.no_force_check and weights[-1] == max(weights) > 0.0)
    if not plan:
        return 2
    if not args.go:
        print("DRY RUN: nothing moved. The space below the TCP clear, the weights ready, then re-run with --go.")
        return 0
    return run_arm(args, cfg, sysc, plan, hook, weights)


if __name__ == "__main__":
    sys.exit(main())
