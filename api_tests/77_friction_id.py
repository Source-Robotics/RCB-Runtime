#!/usr/bin/env python3
"""77: friction identification — every joint's SLIDING friction against speed, at several poses, in both directions,
with the motor temperature, so that one friction model can serve every mode but homing (TODO/friction_compensation_plan.md
"Friction identification", F1; user 2026-10-16: "definitive friction values across a lot of tests … for other arms users
could run that code and just use them").

Why: the robot XML's <friction_sliding_nm> is the LOWEST friction api_tests/54 saw at 2-3 angles and two slow speeds — a
freedrive safety choice, 10-25 % under the typical value; 76's fit of the external-torque observer while moving found the
same, and J2's +0.62 Nm (0 in the XML). Friction also changes with speed (not along a straight line), angle, load and
temperature. This measures all of that, so the robot can use the typical values and know their spread.

How (per joint): EXEC moves the arm to each test pose (the other joints at the api_tests' start pose; J2 also where
gravity loads it), then in ONE hard-RTI session per pose the drive's own speed loop slides the joint back and forth at
every speed of the sweep (54's run_slide: both directions, only the steady part of each leg, a stick-slipping slide gets
no number), stepping back to the pose after each speed. At a steady speed the drive's own correction — the measured
torque minus what RTI commanded (the model's gravity) — is friction + the gravity model's error:
    sliding +: F(v) - e        sliding -: -F(v) - e        so  F = (t+ - t-) / 2,  e = -(t+ + t-) / 2
(at one pose a direction asymmetry of the friction reads as e: only the sum per direction is measured).
--mode quick: 3 poses, 5 / 15 / 40 deg/s, ~10 min for the six joints — for any arm. --mode thorough: 5 poses,
2-40 deg/s, J2 at 3 loaded poses, ~35 min. Run it cold and again warm (after use): --reanalyse merges runs.

Analysis per joint: the friction against speed (a table: median / lowest / highest over poses and runs), a fit
F = Coulomb + viscous x speed over every slide and what the line misses at each speed (a curve it does not follow shows
there), the spread over poses, the effect of load (the model's gravity) and of temperature when they varied, the
gravity model's error per pose (reported only — user: "measure only, decide later"), and today's XML value next to the
measured one. Then the VALUES FOR THE ROBOT XML (F2, 2026-10-17 — the model utility/friction_model.py uses everywhere):
a pose whose friction is over 1.5x the median of the others' is left out (PAR6's J3 at 354 deg: ~1.0 Nm where the rest
read 0.5-0.65 — something like a cable acts there); Coulomb + viscous x speed fitted over the rest, the load term
(friction per Nm of the model's gravity) only where it halves what the line leaves over; the gravity error the largest
measured at any pose (freedrive compensation stays under the friction minus it). `--apply` (with --reanalyse) writes
<friction_coulomb_nm>, <friction_viscous_nm_s>, <friction_load_per_nm> (replacing the older <friction_sliding_nm>) and
<gravity_error_nm> for every joint measured — text-level, every comment kept, a copy of the XML in the first run's folder
first. RTI reads it at its next start. Measure with the friction feedforward OFF: 77 switches it off and back itself.

Safety: the dry run first (the plan: every pose, the TCP's lowest point over the slide's travel, the model's gravity, the
time). Poses whose TCP would come under --min-z (default 0.10 m above the base, by the model) are skipped; EXEC's checks
(limits, zones, self-collision) apply to every move between poses; while sliding, every other joint is held at speed 0
and a slide stops if another joint moves more than 2 deg, the joint runs past its travel, a (non-warning) error latches
or the robot leaves RTI. Robot ENABLED, homed, in IDLE, nothing else holding control; keep the space around the arm clear
— a pose and its slides move the arm, J1 / J2 / J3 the whole of it.
    .venv/bin/python api_tests/77_friction_id.py --selftest
    .venv/bin/python api_tests/77_friction_id.py                              # dry run: the plan, nothing moves
    .venv/bin/python api_tests/77_friction_id.py --go                         # quick, every joint
    .venv/bin/python api_tests/77_friction_id.py --mode thorough --joints 2,3 --go
    .venv/bin/python api_tests/77_friction_id.py --reanalyse temp_data/friction_id/RUN_A temp_data/friction_id/RUN_B
    .venv/bin/python api_tests/77_friction_id.py --reanalyse RUN_A RUN_B --apply   # writes the robot XML (arm disabled)
Results: temp_data/friction_id/<UTC>_<mode>/run.json (every slide, saved as it goes) + report.txt.
"""
import argparse
import json
import math
import os
import re
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FS = 250.0
RECIPE = "friction_id"                  # j_p, j_s, j_t, j_tc, m_temp, tick, mode
SPEEDS = {"quick": (5.0, 15.0, 40.0), "thorough": (2.0, 5.0, 10.0, 20.0, 40.0)}   # deg/s
N_POSES = {"quick": 3, "thorough": 5}
MIN_AMP_DEG = 4.0                       # a slide's travel each way: at least this ...
AMP_S = 0.4                             # ... and the speed x this (each leg keeps >= ~0.4 s of steady sliding)
PAST_S = 0.1                            # travel allowed past it: the speed x this, at least 54's 3 deg (a fast turn overshoots)
SLOW_DPS, SLOW_LEGS = 2.0, 4            # a slow slide takes long: 2 legs each way instead of 3
ROOM_EXTRA_DEG = 2.0                    # a pose stays this much more than the fastest slide's reach from the soft limits
# Where gravity loads a joint the spread poses do not reach, per robot: (angle, {other joint: angle}), deg. PAR6's J2 with
# J3 ~230 deg (TODO/friction_compensation_plan.md item 4: model gravity 2.3 / 3.9 / 5.3 Nm, TCP z >= 123 mm by the model).
# quick takes the first one only. A robot not listed gets the spread poses only.
LOADED = {"PAR6": {2: ((-65.0, {3: 230.0}), (-80.0, {3: 230.0}), (-50.0, {3: 230.0}))}}
MIN_Z_M = 0.10                          # skip a pose whose TCP comes under this (m, above the base, by the model)
MOVE_SPEED = 10                         # EXEC moves between poses, percent
RETURN_MAX_DPS, RETURN_MIN_DPS = 10.0, 2.0   # back to the pose after a slide: a speed proportional to the distance,
RETURN_GAIN = 3.0                            # 1/s, at least the minimum (a sticking joint still gets there) ...
RETURN_TOL_DEG, RETURN_TIMEOUT_S = 1.0, 10.0  # ... until within this (the poses keep ROOM_EXTRA_DEG spare) ...
RETURN_SETTLE_S = 0.1                         # ... for this long (a sticking joint's drive may still push it on)
NO_STATE_S = 0.5                              # no state packet this long: RTI stopped streaming (an error) — give up
# A slide whose slower direction stayed under this share of its speed was still speeding up in its "steady" part: the
# acceleration torque is in its number, so it is left out (2026-10-17 on the arm: J3 loaded at 40 deg/s reached 35.1 in
# one direction and read 1.513 Nm; the slow slides of J1 / J4 / J5 at 95-96 % are fine — a speed loop's small shortfall).
MIN_SPEED_SHARE = 0.9
POSE_OVERHEAD_S = 8.0                   # per pose: the EXEC move + the RTI session (the time estimate)
OUTLIER_RATIO = 1.5                     # a pose whose median friction is over this x the others' median: left out
LOAD_PAYS = 0.5                         # the load term is used only when it leaves at most this share of the line's RMS


def m54():
    """api_tests/54: run_slide (the slide) and split (friction / gravity error) are used as they are."""
    return __import__("54_joint_friction")


def amplitude_deg(v_dps):
    return max(MIN_AMP_DEG, AMP_S * v_dps)


def past_deg(v_dps):
    return max(m54().SLIDE_PAST_DEG, PAST_S * v_dps)


def legs_for(v_dps):
    return SLOW_LEGS if v_dps <= SLOW_DPS else m54().SLIDE_LEGS


def room_needed_deg(speeds):
    """How far from the soft limits a pose must be for the fastest slide of the sweep (its travel + overshoot + extra)."""
    v = max(speeds)
    return amplitude_deg(v) + past_deg(v) + ROOM_EXTRA_DEG


def test_poses(j, mode, lo_deg, hi_deg, start_deg, robot_name):
    """The poses (deg, every joint) joint j (0-based) is measured at: the other joints at start_deg, joint j spread evenly
    over its soft range minus the room the fastest slide needs, + where gravity loads it (LOADED; quick: the first)."""
    room = room_needed_deg(SPEEDS[mode])
    a, b = lo_deg[j] + room, hi_deg[j] - room
    if b < a:
        return []
    out = []
    for x in np.linspace(a, b, N_POSES[mode]):
        p = [float(v) for v in start_deg]
        p[j] = float(x)
        out.append({"pose": p, "kind": "spread"})
    loaded = LOADED.get(robot_name, {}).get(j + 1, ())
    for x, others in (loaded[:1] if mode == "quick" else loaded):
        if not a <= x <= b:
            continue
        p = [float(v) for v in start_deg]
        p[j] = float(x)
        for k, val in others.items():
            p[k - 1] = float(val)
        out.append({"pose": p, "kind": "loaded"})
    return out


def tcp_z_range(fk_z, pose_deg, j, reach_deg, k=9):
    """The TCP's lowest and highest point (m) while joint j moves +/- reach_deg around the pose (the other joints fixed)."""
    zs = []
    for d in np.linspace(-reach_deg, reach_deg, k):
        q = np.radians(np.asarray(pose_deg, dtype=float))
        q[j] += math.radians(d)
        zs.append(float(fk_z(q)))
    return min(zs), max(zs)


def make_plan(joints, mode, lo_deg, hi_deg, start_deg, robot_name, fk_z, gravity_fn, min_z=MIN_Z_M):
    """Every pose of the run with its sweep, the TCP's lowest point over the slides and the model's gravity there;
    a pose whose TCP would come under min_z carries a "skip" reason."""
    speeds = SPEEDS[mode]
    reach = amplitude_deg(max(speeds)) + past_deg(max(speeds))
    plan = []
    for jj in joints:
        j = jj - 1
        for pz in test_poses(j, mode, lo_deg, hi_deg, start_deg, robot_name):
            z_lo, _ = tcp_z_range(fk_z, pz["pose"], j, reach)
            item = {"joint": jj, "pose": pz["pose"], "kind": pz["kind"], "z_min": z_lo,
                    "gravity": float(gravity_fn(np.radians(pz["pose"]))[j]),
                    "speeds": [{"v": v, "amp": amplitude_deg(v), "past": past_deg(v), "legs": legs_for(v)}
                               for v in speeds]}
            if z_lo < min_z:
                item["skip"] = f"the TCP would come to {1000 * z_lo:.0f} mm above the base (under {1000 * min_z:.0f})"
            plan.append(item)
    return plan


def slide_seconds(sp):
    return sp["legs"] * 2.0 * sp["amp"] / sp["v"] + 1.5          # the legs + the way back to the pose


def plan_seconds(plan):
    return sum(POSE_OVERHEAD_S + sum(slide_seconds(s) for s in p["speeds"]) for p in plan if "skip" not in p)


def fresh_state(send_velocity, read_state, pace, n=6, timeout_s=0.5):
    """Speed 0 to every joint until a state packet comes (None after timeout_s)."""
    vel = [0.0] * n
    for _ in range(int(timeout_s * FS)):
        send_velocity(vel)
        pace()
        s = read_state()
        if s is not None:
            return s
    return None


TEMP_S = 0.2                            # the motor temperature: the median over this long (single readings glitch)


def motor_temp(j, send_velocity, read_state, pace, n=6):
    """Joint j's motor temperature (C): the median of the packets of TEMP_S at speed 0, or None. One reading is not
    enough — on the arm (2026-10-17) single readings jumped 36 -> 45 -> 36 C between slides a few seconds apart."""
    vel, temps = [0.0] * n, []
    for _ in range(int(TEMP_S * FS)):
        send_velocity(vel)
        pace()
        s = read_state()
        if s is not None and s.get("m_temp") is not None:
            t = float(s["m_temp"][j])
            if math.isfinite(t):
                temps.append(t)
    return float(np.median(temps)) if temps else None


def return_to(j, x_target, send_velocity, read_state, pace, n=6):
    """Back to the pose after a slide (54's slide ends a whole travel away), by the drive's speed loop: a speed
    proportional to the distance, at most RETURN_MAX_DPS and — outside RETURN_TOL_DEG — at least RETURN_MIN_DPS (a
    sticking joint still gets there); done once it has stayed within RETURN_TOL_DEG for RETURN_SETTLE_S (a drive whose
    integral wound up while the joint stuck can push it on after it got there — it is corrected again). Measured on
    simulated joints (2026-10-16): a sticking light joint jumps 0.4-0.8 deg after its command stops, so a tighter
    tolerance never settles; inside it the proportional speed still brings a normal joint near the pose. Returns None,
    or why it gave up (speed 0 commanded either way)."""
    vel = [0.0] * n
    tol = math.radians(RETURN_TOL_DEG)
    v_max, v_min = math.radians(RETURN_MAX_DPS), math.radians(RETURN_MIN_DPS)
    settle, inside, silent = int(RETURN_SETTLE_S * FS), 0, 0
    why = f"J{j + 1} did not get back to its pose within {RETURN_TIMEOUT_S:g} s"
    for _ in range(int(RETURN_TIMEOUT_S * FS)):
        s = read_state()
        silent = silent + 1 if s is None else 0
        if silent >= int(NO_STATE_S * FS):          # 2026-10-16 on the arm: an error stopped RTI's stream, 10 s wasted
            why = f"no state packet from RTI for {NO_STATE_S:g} s — the robot left RTI (an error?)"
            break
        if s is not None:
            err = x_target - float(s["j_p"][j])
            if abs(err) < tol:
                vel[j] = math.copysign(min(v_max, RETURN_GAIN * abs(err)), err)
                inside += 1
                if inside >= settle:
                    why = None
                    break
            else:
                inside = 0
                vel[j] = math.copysign(min(v_max, max(v_min, RETURN_GAIN * abs(err))), err)
        send_velocity(vel)
        pace()
    vel[j] = 0.0
    for _ in range(int(0.15 * FS)):
        send_velocity(vel)
        pace()
    return why


def slide_pose(j, speeds, send_velocity, read_state, pace, ok_to_go=lambda: None, n=6):
    """Every slide of the sweep at the pose the joint is at, back to that pose after each. ok_to_go() -> None, or why to
    stop (an error, a mode change). Returns (records, why it stopped or None). A stick-slipping slide gets no number and
    the sweep goes on; any other refusal ends it (after the way back)."""
    s = fresh_state(send_velocity, read_state, pace, n)
    if s is None:
        return [], "no state packet from RTI"
    x_pose = float(s["j_p"][j])
    recs = []
    for sp in speeds:
        why = ok_to_go()
        if why:
            return recs, why
        temp = motor_temp(j, send_velocity, read_state, pace, n)
        res, why = m54().run_slide(j, sp["v"], sp["amp"], send_velocity, read_state, pace, n=n, legs=sp["legs"],
                                   past_deg=sp["past"])
        rec = {"v_cmd_dps": sp["v"], "amp_deg": sp["amp"], "legs": sp["legs"], "temp_c": temp}
        if res is None:
            rec["why"] = why
        else:
            (tp, vp, shp, gp), (tm, vm, shm, gm) = res[1.0], res[-1.0]
            f, e = m54().split(tp, tm)
            rec.update(t_plus=tp, t_minus=tm, v_plus_dps=math.degrees(vp), v_minus_dps=math.degrees(vm),
                       steady=[shp, shm], friction=f, gravity_error=e, model_gravity=0.5 * (gp + gm))
        recs.append(rec)
        back = return_to(j, x_pose, send_velocity, read_state, pace, n)
        if back:
            return recs, back
        if res is None and "stick-slip" not in why:
            return recs, why
    return recs, None


# ------------------------------------------------------------------------------------------------------------ analysis
def speed_rad(rec):
    """The measured sliding speed (rad/s), both directions averaged."""
    return math.radians(0.5 * (rec["v_plus_dps"] - rec["v_minus_dps"]))


def speed_share(rec):
    """The slower direction's measured speed / the commanded speed (1.0 = it slid at its speed)."""
    return min(rec["v_plus_dps"], -rec["v_minus_dps"]) / rec["v_cmd_dps"]


def usable(rec):
    """A slide with a number that slid at its speed (see MIN_SPEED_SHARE)."""
    return "friction" in rec and speed_share(rec) >= MIN_SPEED_SHARE


def not_usable_why(rec):
    if "friction" not in rec:
        return rec.get("why")
    return (f"did not reach its speed ({min(rec['v_plus_dps'], -rec['v_minus_dps']):.1f} of {rec['v_cmd_dps']:g} deg/s:"
            f" acceleration in the measurement) — left out")


def _fit(cols, y):
    a = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(a, y, rcond=None)
    return coef, y - a @ coef


def analyse_joint(recs):
    """One joint's slides (any runs, any poses) -> the table, the fit, the spread, load / temperature, gravity errors."""
    ok = [r for r in recs if usable(r)]
    out = {"n_slides": len(recs), "n_measured": len(ok),
           "refused": [not_usable_why(r) for r in recs if not usable(r)]}
    if not ok:
        return out
    f = np.array([r["friction"] for r in ok])
    v = np.array([speed_rad(r) for r in ok])
    g = np.array([abs(r["model_gravity"]) for r in ok])
    t = np.array([r["temp_c"] if r.get("temp_c") is not None else np.nan for r in ok])
    one = np.ones_like(f)
    speeds = sorted({r["v_cmd_dps"] for r in ok})
    if len(speeds) >= 2:
        (fc, b), left = _fit([one, v], f)
        out["fit"] = {"coulomb": float(fc), "viscous": float(b), "rms": float(np.sqrt(np.mean(left ** 2)))}
    else:
        left = f - np.median(f)
        out["fit"] = {"coulomb": float(np.median(f)), "viscous": None, "rms": float(np.sqrt(np.mean(left ** 2)))}
    out["table"] = []
    for vc in speeds:
        sel = np.array([r["v_cmd_dps"] == vc for r in ok])
        out["table"].append({"v_cmd_dps": vc, "n": int(sel.sum()), "v_dps": float(np.degrees(np.median(v[sel]))),
                             "median": float(np.median(f[sel])), "min": float(f[sel].min()), "max": float(f[sel].max()),
                             "left": float(np.median(left[sel]))})
    out["spread"] = float(max(row["max"] - row["min"] for row in out["table"]))
    out["lowest"] = float(f.min())
    base = [one, v] if len(speeds) >= 2 else [one]
    if np.ptp(g) > 0.5:                                  # the model's gravity varied enough to see a load effect
        coef, left_g = _fit(base + [g], f)
        out["load"] = {"per_nm": float(coef[-1]), "gravity": [float(g.min()), float(g.max())],
                       "coulomb": float(coef[0]), "viscous": float(coef[1]) if len(base) == 2 else None,
                       "rms": float(np.sqrt(np.mean(left_g ** 2)))}
    fin = np.isfinite(t)
    if fin.sum() >= 3 and np.ptp(t[fin]) > 3.0:          # warmed up by more than 3 C during the runs
        coef, _ = _fit([c[fin] for c in base] + [t[fin]], f[fin])
        out["temperature"] = {"per_c": float(coef[-1]), "temp_c": [float(t[fin].min()), float(t[fin].max())]}
    poses = {}
    for r in ok:
        poses.setdefault(round(float(r.get("angle_deg", float("nan"))), 1), []).append(r)
    out["gravity_error"] = [{"angle_deg": k, "error": float(np.mean([r["gravity_error"] for r in rs])),
                             "model_gravity": float(np.mean([r["model_gravity"] for r in rs])),
                             "kind": rs[0].get("kind", "")} for k, rs in sorted(poses.items())]
    return out


def analyse(slides):
    """Every slide record (with its "joint") -> {joint: analyse_joint(...)}."""
    by = {}
    for r in slides:
        if "joint" in r:
            by.setdefault(int(r["joint"]), []).append(r)
    return {jj: analyse_joint(rs) for jj, rs in sorted(by.items())}


def _pose_key(r):
    return round(float(r.get("angle_deg", float("nan"))), 1)


def typical(recs):
    """One joint's slides -> the values for the robot XML (see the module docstring), or None without a usable slide:
    {coulomb, viscous, load, gravity_error, rms, rms_line, n, poses, outliers}."""
    ok = [r for r in recs if usable(r)]
    if not ok:
        return None
    by = {}
    for r in ok:
        by.setdefault(_pose_key(r), []).append(r)
    med = {k: float(np.median([r["friction"] for r in rs])) for k, rs in by.items()}
    outliers = []
    if len(med) >= 3:                                    # two poses cannot say which one is odd
        for k, m in med.items():
            if m > OUTLIER_RATIO * float(np.median([x for kk, x in med.items() if kk != k])):
                outliers.append(k)
    keep = [r for r in ok if _pose_key(r) not in outliers]
    f = np.array([r["friction"] for r in keep])
    v = np.array([speed_rad(r) for r in keep])
    g = np.array([abs(r["model_gravity"]) for r in keep])
    one = np.ones_like(f)
    two_speeds = len({r["v_cmd_dps"] for r in keep}) >= 2
    c, b = float(np.median(f)), 0.0
    if two_speeds:
        (c1, b1), _ = _fit([one, v], f)
        if b1 >= 0.0:
            c, b = float(c1), float(b1)
    left = f - c - b * v
    rms_line = float(np.sqrt(np.mean(left ** 2)))
    out = {"coulomb": c, "viscous": b, "load": 0.0, "rms": rms_line, "rms_line": rms_line, "n": len(keep),
           "poses": len(med) - len(outliers), "outliers": sorted(outliers)}
    if np.ptp(g) > 0.5:
        cols = [one, v] if (two_speeds and b > 0.0) else [one]
        coef, left_g = _fit(cols + [g], f)
        rms_g = float(np.sqrt(np.mean(left_g ** 2)))
        if coef[-1] > 0.0 and coef[0] >= 0.0 and (len(cols) == 1 or coef[1] >= 0.0) and rms_g <= LOAD_PAYS * rms_line:
            out.update(coulomb=float(coef[0]), viscous=float(coef[1]) if len(cols) == 2 else 0.0, load=float(coef[-1]),
                       rms=rms_g)
    out["coulomb"] = max(0.0, out["coulomb"])
    errs = {}
    for r in ok:                                         # every pose, the outliers too: the freedrive rule's margin
        errs.setdefault(_pose_key(r), []).append(r["gravity_error"])
    out["gravity_error"] = float(max(abs(float(np.mean(e))) for e in errs.values()))
    return out


def typical_all(slides):
    """Every slide -> {joint: typical(...)} (joints with no usable slide left out)."""
    by = {}
    for r in slides:
        if "joint" in r:
            by.setdefault(int(r["joint"]), []).append(r)
    out = {}
    for jj, rs in sorted(by.items()):
        t = typical(rs)
        if t is not None:
            out[jj] = t
    return out


def typical_report(values, xml_now=None):
    lines = ["\nvalues for the robot XML (utility/friction_model.py: (Coulomb + load x |gravity|) x sign + viscous x speed;"
             " --apply writes them):"]
    for jj, t in values.items():
        now = (f"  (XML today: Coulomb {float(xml_now[jj - 1]):.3f})" if xml_now is not None and jj - 1 < len(xml_now)
               else "")
        lines.append(f"  J{jj}: Coulomb {t['coulomb']:.3f} Nm + viscous {t['viscous']:.3f} Nm.s/rad x speed"
                     + (f" + {t['load']:.3f} x |gravity|" if t["load"] else "")
                     + f"; gravity error {t['gravity_error']:.3f} Nm; leftover RMS {t['rms']:.3f}"
                     + (f" (the line alone {t['rms_line']:.3f})" if t["load"] else "")
                     + f"; {t['n']} slides at {t['poses']} poses"
                     + (f"; left out (over {OUTLIER_RATIO:g}x the others): "
                        + ", ".join(f"{k:+.1f} deg" for k in t["outliers"]) if t["outliers"] else "") + now)
    return "\n".join(lines)


def _xml_line(pad, tag, value, comment):
    comment = comment.replace("--", "- -")              # an XML comment may not hold "--" (a run folder name might)
    return f"{pad}<{tag}>{value}</{tag}> <!-- {comment} -->\n"


def write_friction_xml(xml_path, values, backup_dir=None, source=""):
    """Write the typical values into the robot XML: per measured joint (values: {joint 1-based: typical(...)})
    <friction_coulomb_nm> / <friction_viscous_nm_s> / <friction_load_per_nm> in place of <friction_sliding_nm> (or the
    earlier ones), and <gravity_error_nm>. Text-level — every other line and comment unchanged; the result must parse
    before it is written; a copy of the old file into backup_dir first. Returns (the backup's path or None, joints)."""
    import re
    import shutil
    import xml.etree.ElementTree as ET
    with open(xml_path) as fh:
        text = fh.read()
    day = time.strftime("%Y-%m-%d", time.gmtime())
    written = []
    for jj, t in sorted(values.items()):
        m = re.search(rf'<joint id="{jj - 1}">(.*?)</joint>', text, flags=re.S)
        if m is None:
            raise ValueError(f"{os.path.basename(xml_path)}: no <joint id=\"{jj - 1}\"> block")
        body = m.group(1)
        line_re = lambda tag: re.compile(rf"^([ \t]*)<{tag}>([^<]*)</{tag}>[^\n]*\n", flags=re.M)  # noqa: E731
        old = line_re("friction_sliding_nm").search(body)
        was = f" Was <friction_sliding_nm>{old.group(2).strip()} (api_tests/54's lowest)." if old else ""
        if old is None and line_re("friction_coulomb_nm").search(body):    # applied before: keep where it came from
            mw = re.search(r" Was <friction_sliding_nm>\S* \(api_tests/54's lowest\)\.",
                           line_re("friction_coulomb_nm").search(body).group(0))
            was = mw.group(0) if mw else ""
        anchor = old or line_re("friction_coulomb_nm").search(body) or line_re("gravity_error_nm").search(body)
        pad = anchor.group(1) if anchor else "            "
        out_note = (f"; poses left out (over {OUTLIER_RATIO:g}x the others): "
                    + ", ".join(f"{k:+.1f} deg" for k in t["outliers"]) if t["outliers"] else "")
        new = (_xml_line(pad, "friction_coulomb_nm", f"{t['coulomb']:.3f}",
                         f"Nm, sliding (Coulomb) friction, the typical value: {t['n']} slides at {t['poses']} poses "
                         f"(api_tests/77, applied {day}{(', ' + source) if source else ''}{out_note}).{was} The friction "
                         f"model of utility/friction_model.py: feedforward, external torque, freedrive")
               + _xml_line(pad, "friction_viscous_nm_s", f"{t['viscous']:.3f}",
                           "Nm.s/rad: the friction grows by this x speed (api_tests/77)")
               + _xml_line(pad, "friction_load_per_nm", f"{t['load']:.3f}",
                           "Nm of friction per Nm of the gravity model's torque on the joint (api_tests/77; 0 = no "
                           "load effect worth a term)"))
        for tag in ("friction_coulomb_nm", "friction_viscous_nm_s", "friction_load_per_nm"):
            if tag != "friction_coulomb_nm" or old is not None:
                body = line_re(tag).sub("", body, count=1)
        if old is not None:
            body = line_re("friction_sliding_nm").sub(lambda _m: new, body, count=1)
        elif line_re("friction_coulomb_nm").search(body):
            body = line_re("friction_coulomb_nm").sub(lambda _m: new, body, count=1)
        elif line_re("gravity_error_nm").search(body):
            ge = line_re("gravity_error_nm").search(body)
            body = body[:ge.start()] + new + body[ge.start():]
        else:
            content = body.rstrip()
            body = content + "\n" + new.rstrip("\n") + body[len(content):]
        gline = _xml_line(pad, "gravity_error_nm", f"{t['gravity_error']:.3f}",
                          f"Nm, the largest gravity-model error measured while sliding, any pose and speed (api_tests/77, "
                          f"applied {day}). Freedrive friction compensation gives back 0.5 x (sliding friction - this)")
        if line_re("gravity_error_nm").search(body):
            body = line_re("gravity_error_nm").sub(lambda _m: gline, body, count=1)
        else:
            i = body.find(new) + len(new)
            body = body[:i] + gline + body[i:]
        text = text[:m.start(1)] + body + text[m.end(1):]
        written.append(jj)
    ET.fromstring(text)                                  # never write a file that no longer parses
    backup = None
    if backup_dir is not None:
        backup = os.path.join(str(backup_dir), f"robot_xml_backup_{time.strftime('%Y%m%d_%H%M%S', time.gmtime())}_"
                                               f"{os.path.basename(xml_path)}")
        shutil.copy2(xml_path, backup)
    with open(xml_path, "w") as fh:
        fh.write(text)
    return backup, written


def report(results, xml_now=None, runs=1):
    lines = [f"friction identification: {runs} run(s). Sliding friction at a steady speed (Nm), over every pose and run;"
             " 'left' = what the line Coulomb + viscous x speed misses at that speed. The robot XML is written only by"
             " --apply."]
    for jj, a in results.items():
        lines.append(f"\nJ{jj}: {a['n_measured']} of {a['n_slides']} slides measured"
                     + (f"; no number: {', '.join(sorted({str(w)[:60] for w in a['refused']}))}" if a["refused"] else ""))
        if not a.get("table"):
            continue
        lines.append("  deg/s  measured   median   lowest  highest    left")
        for row in a["table"]:
            lines.append(f"  {row['v_cmd_dps']:5.0f}  {row['v_dps']:8.1f}  {row['median']:7.3f}  {row['min']:7.3f}  "
                         f"{row['max']:7.3f}  {row['left']:+7.3f}   ({row['n']} slides)")
        fit = a["fit"]
        lines.append(f"  fit: Coulomb {fit['coulomb']:.3f} Nm" + (f" + viscous {fit['viscous']:.3f} Nm.s/rad x speed"
                                                                  if fit["viscous"] is not None else "")
                     + f" (leftover RMS {fit['rms']:.3f}); spread over poses / runs at one speed up to {a['spread']:.3f};"
                       f" lowest seen {a['lowest']:.3f}")
        if "load" in a:
            ld = a["load"]
            lines.append(f"  load: {ld['per_nm']:+.3f} Nm of friction per Nm of the model's gravity "
                         f"(gravity {ld['gravity'][0]:.2f}-{ld['gravity'][1]:.2f} Nm); with it the fit is Coulomb "
                         f"{ld['coulomb']:.3f}" + (f" + viscous {ld['viscous']:.3f} x speed" if ld["viscous"] is not None
                                                   else "") + f" + {ld['per_nm']:.3f} x |gravity| (leftover RMS {ld['rms']:.3f})")
        else:
            lines.append("  load: not seen (the model's gravity varied under 0.5 Nm)")
        if "temperature" in a:
            tp = a["temperature"]
            lines.append(f"  temperature: {tp['per_c']:+.4f} Nm per C ({tp['temp_c'][0]:.0f}-{tp['temp_c'][1]:.0f} C)")
        else:
            lines.append("  temperature: not seen (it varied under 3 C — run again warm)")
        lines.append("  gravity-model error per pose (reported only): " + ", ".join(
            f"{x['angle_deg']:+.1f} deg {x['error']:+.3f} (model {x['model_gravity']:+.2f}"
            + (", loaded" if x["kind"] == "loaded" else "") + ")" for x in a["gravity_error"]))
        if xml_now is not None and jj - 1 < len(xml_now):
            lines.append(f"  robot XML today: Coulomb {float(xml_now[jj - 1]):.3f}; this fit {fit['coulomb']:.3f}")
    return "\n".join(lines)


def load_runs(folders):
    """run.json of each folder -> (every slide, how many runs)."""
    slides, runs = [], 0
    for folder in folders:
        with open(os.path.join(folder, "run.json")) as fh:
            run = json.load(fh)
        runs += 1
        for r in run.get("slides", []):
            r = dict(r)
            r.setdefault("run", os.path.basename(os.path.normpath(folder)))
            slides.append(r)
    return slides, runs


# ------------------------------------------------------------------------------------------------------------ self-test
def _plant(j, fric, e, x0=0.0, inertia=0.002, bw_hz=30.0, ki_hz=3.0, temp=35.0, n=6, ripple=0.0, g_model=0.3,
           i_max=3.0, temp_glitch=0.0):
    """A simulated joint under its drive's PI speed loop (5 kHz on the true speed; the command applied one tick late,
    the state one tick old), as 54's self-test, with a SPEED-DEPENDENT sliding friction fric(|v| rad/s) -> Nm and a
    breakaway 1.3 x fric(0); every other joint held."""
    kp = inertia * 2.0 * math.pi * bw_hz
    ki = kp * 2.0 * math.pi * ki_hz
    st = {"q": x0, "v": 0.0, "i": 0.0, "stuck": True, "cmd": 0.0, "pending": 0.0, "hist": [], "last_cmd": 0.0,
          "temp": temp}
    fs = 1.3 * fric(0.0)
    sub = 20
    dt = 1.0 / FS / sub

    def send_velocity(vel):
        st["pending"] = float(vel[j])
        st["last_cmd"] = float(vel[j])

    def pace():
        vc, st["cmd"] = st["cmd"], st["pending"]
        u_sum = 0.0
        for _ in range(sub):
            err = vc - st["v"]
            st["i"] = max(-i_max, min(i_max, st["i"] + ki * err * dt))
            u = max(-i_max, min(i_max, kp * err + st["i"]))
            u_sum += u
            net = u + e
            if st["stuck"]:
                if abs(net) > fs:
                    st["stuck"] = False
                else:
                    continue
            sgn = math.copysign(1.0, st["v"]) if st["v"] else math.copysign(1.0, net)
            v_new = st["v"] + (net - fric(abs(st["v"])) * sgn) / inertia * dt
            if st["v"] and v_new * st["v"] <= 0.0 and abs(net) <= fs:
                st["v"], st["stuck"] = 0.0, True
            else:
                st["v"] = v_new
            st["q"] += st["v"] * dt
        q = np.zeros(n)
        q[j] = st["q"]
        rip = ripple * math.sin(900.0 * st["q"])
        jt, js, jtc = np.zeros(n), np.zeros(n), np.zeros(n)
        jtc[j] = g_model
        jt[j] = g_model + u_sum / sub + rip
        js[j] = st["v"] + 20.0 * rip
        glitch = temp_glitch if len(st["hist"]) % 7 == 3 else 0.0       # a reading off now and then, as on the arm
        st["hist"].append({"j_p": q, "j_s": js, "j_t": jt, "j_tc": jtc, "m_temp": np.full(n, st["temp"] + glitch)})

    def read_state():
        return st["hist"][-2] if len(st["hist"]) > 1 else None

    return send_velocity, read_state, pace, st


def selftest():
    bad = []

    def chk(name, cond, detail=""):
        print(("  PASS  " if cond else "  FAIL  ") + name + (f"   [{detail}]" if detail and not cond else ""))
        if not cond:
            bad.append(name)

    print("A. the sweep's rules")
    chk("travel each way: 4 deg up to 10 deg/s, then 0.4 s worth (16 deg at 40)",
        amplitude_deg(5.0) == 4.0 and amplitude_deg(10.0) == 4.0 and abs(amplitude_deg(40.0) - 16.0) < 1e-12)
    chk("allowed past it: 54's 3 deg, 4 deg at 40 deg/s", past_deg(15.0) == 3.0 and abs(past_deg(40.0) - 4.0) < 1e-12)
    chk("legs: 4 at 2 deg/s, 54's 6 otherwise", legs_for(2.0) == 4 and legs_for(5.0) == m54().SLIDE_LEGS == 6)
    chk("a pose keeps 22 deg from the soft limits (16 + 4 + 2) for both modes",
        abs(room_needed_deg(SPEEDS["quick"]) - 22.0) < 1e-12 and abs(room_needed_deg(SPEEDS["thorough"]) - 22.0) < 1e-12)

    print("B. the poses")
    lo = [-164.14, -139.84, 114.09, -149.81, -99.12, -48.70]
    hi = [164.14, -6.43, 376.02, 146.38, 91.67, 409.09]
    start = [90.0, -105.0, 165.0, 0.0, -30.0, 180.0]
    p1 = test_poses(0, "quick", lo, hi, start, "PAR6")
    chk("J1 quick: 3 poses spread over its range 22 deg from the limits, the others at the start pose",
        len(p1) == 3 and abs(p1[0]["pose"][0] - (lo[0] + 22)) < 1e-9 and abs(p1[-1]["pose"][0] - (hi[0] - 22)) < 1e-9
        and all(p["pose"][1:] == start[1:] for p in p1), [p["pose"][0] for p in p1])
    p2q = test_poses(1, "quick", lo, hi, start, "PAR6")
    p2t = test_poses(1, "thorough", lo, hi, start, "PAR6")
    chk("J2 quick: 3 spread + ONE loaded pose (J2 -65, J3 230)",
        len(p2q) == 4 and p2q[-1]["kind"] == "loaded" and p2q[-1]["pose"][1] == -65.0 and p2q[-1]["pose"][2] == 230.0)
    chk("J2 thorough: 5 spread + 3 loaded (-65 / -80 / -50 with J3 230)",
        len(p2t) == 8 and [p["pose"][1] for p in p2t if p["kind"] == "loaded"] == [-65.0, -80.0, -50.0])
    chk("another robot: no loaded poses", all(p["kind"] == "spread" for p in test_poses(1, "thorough", lo, hi, start, "X")))
    chk("a joint with less than 2 x 22 deg of range: no pose", test_poses(0, "quick", [-20.0] * 6, [20.0] * 6, start, "X") == [])

    print("C. the plan")
    fk_z = lambda q: 0.30 - 0.25 * math.sin(q[1] + math.radians(105.0))   # a fake TCP height: low with J2 far forward
    grav = lambda q: np.array([0.0, 2.0 * math.cos(q[1]), 1.0, 0.1, 0.1, 0.0])
    plan = make_plan([1, 2, 3, 4, 5, 6], "quick", lo, hi, start, "PAR6", fk_z, grav)
    skipped = [p for p in plan if "skip" in p]
    chk("a pose whose TCP would come under 0.10 m is marked skipped, with its height",
        skipped and all(p["joint"] == 2 and p["z_min"] < MIN_Z_M and "mm above the base" in p["skip"] for p in skipped),
        [(p["joint"], p["pose"][1], round(p["z_min"], 3)) for p in skipped])
    chk("every pose carries the model's gravity of its joint and the sweep (5 / 15 / 40 deg/s)",
        all(len(p["speeds"]) == 3 and [s["v"] for s in p["speeds"]] == [5.0, 15.0, 40.0] for p in plan)
        and abs(plan[0]["gravity"] - 0.0) < 1e-12)
    tq = plan_seconds(make_plan([1, 2, 3, 4, 5, 6], "quick", lo, hi, start, "PAR6", lambda q: 1.0, grav)) / 60.0
    tt = plan_seconds(make_plan([1, 2, 3, 4, 5, 6], "thorough", lo, hi, start, "PAR6", lambda q: 1.0, grav)) / 60.0
    chk(f"time: quick ~{tq:.0f} min, thorough ~{tt:.0f} min (the docstring says ~10 / ~35)", 6 <= tq <= 14 and 25 <= tt <= 45)

    print("D. slides on a simulated joint (54's run_slide + the way back)")
    # Inertias as the arm's (link + rotor, joint side): a joint with more friction is a heavier one. A light joint with
    # a lot of friction under a 30 Hz speed loop really stick-slips — refused by design (the last case of this part).
    for name, fric, e, inertia, mode in (
            ("J1-like: linear 0.20 + 0.10 x v", lambda v: 0.20 + 0.10 * v, 0.03, 0.02, "thorough"),
            ("J3-like: LESS friction faster (a dip, as J3 showed) + 0.05 x v",
             lambda v: 0.37 + 0.10 * math.exp(-(v / 0.15) ** 2) + 0.05 * v, -0.04, 0.04, "thorough"),
            ("J5-like: light, 0.08 + 0.06 x v", lambda v: 0.08 + 0.06 * v, 0.03, 0.002, "quick"),
            ("J2-like: heavy, 0.60 + 0.11 x v", lambda v: 0.60 + 0.11 * v, 0.07, 0.05, "quick")):
        sv, rs, pc, st = _plant(4, fric, e, x0=0.5, inertia=inertia)
        sweep = [{"v": v, "amp": amplitude_deg(v), "past": past_deg(v), "legs": legs_for(v)} for v in SPEEDS[mode]]
        after = []

        def ok_to_go(st=st, after=after):
            after.append(st["q"])
            return None
        recs, why = slide_pose(4, sweep, sv, rs, pc, ok_to_go)
        meas = [r for r in recs if "friction" in r]
        errs = [abs(r["friction"] - fric(speed_rad(r))) / fric(speed_rad(r)) for r in meas]
        chk(f"{name}: every speed measured, friction within 5 % of the plant's at the measured speed (worst "
            f"{100 * max(errs, default=1):.1f} %), gravity error {e:+} within 0.004",
            why is None and len(meas) == len(sweep) and max(errs) < 0.05
            and all(abs(r["gravity_error"] - e) < 0.004 for r in meas), (why, [r.get("why") for r in recs]))
        chk(f"{name}: back at the pose (within {RETURN_TOL_DEG + 0.1:g} deg) before every speed, speed 0 at the end",
            all(abs(math.degrees(x - 0.5)) < RETURN_TOL_DEG + 0.1 for x in after)
            and abs(math.degrees(st["q"] - 0.5)) < RETURN_TOL_DEG + 0.1 and st["last_cmd"] == 0.0,
            [round(math.degrees(x - 0.5), 2) for x in after])
        chk(f"{name}: the motor temperature recorded per slide", all(r["temp_c"] == 35.0 for r in recs))
    sv, rs, pc, st = _plant(4, lambda v: 0.08 + 0.06 * v, 0.03, inertia=0.002, temp_glitch=9.0)
    recs, why = slide_pose(4, [{"v": 15.0, "amp": 6.0, "past": 3.0, "legs": 6}] * 3, sv, rs, pc)
    chk("a motor temperature whose single readings glitch +9 C now and then is recorded as it is (the median), not as "
        "a glitch (the arm, 2026-10-17: 36 -> 45 -> 36 C)", len(recs) == 3 and all(r["temp_c"] == 35.0 for r in recs),
        [r.get("temp_c") for r in recs])
    # The way back must SETTLE: a drive whose integral wound up while the joint stuck can push it on after it got there.
    # An ideal joint that is kicked 2.5 deg back out 3 ticks after it first comes within the tolerance.
    kick = {"q": math.radians(8.0), "cmd": 0.0, "inside_ticks": 0, "kicked": False}

    def k_send(vel):
        kick["cmd"] = vel[4]

    def k_pace():
        kick["q"] += kick["cmd"] / FS
        if not kick["kicked"] and abs(kick["q"]) < math.radians(RETURN_TOL_DEG):
            kick["inside_ticks"] += 1
            if kick["inside_ticks"] == 3:
                kick["kicked"] = True
                kick["q"] += math.copysign(math.radians(2.5), kick["q"])

    def k_read():
        q = np.zeros(6)
        q[4] = kick["q"]
        return {"j_p": q}
    why = return_to(4, 0.0, k_send, k_read, k_pace)
    chk(f"the way back settles: a joint pushed back out after it got there is brought back (within {RETURN_TOL_DEG:g} "
        f"deg, not left {math.degrees(abs(kick['q'])):.1f} deg off)",
        why is None and kick["kicked"] and abs(kick["q"]) < math.radians(RETURN_TOL_DEG), (why, math.degrees(kick["q"])))
    # The stream stops (on the arm 2026-10-16: LOOP_CRITICAL disabled the robot mid-sweep; the old way back waited 10 s).
    sv, rs, pc, st = _plant(4, lambda v: 0.2, 0.0, x0=0.3)
    ticks = []

    def silent_pace(pc=pc):
        pc()
        ticks.append(1)
    why = return_to(4, 0.5, sv, lambda: None, silent_pace)
    chk(f"the way back gives up within {NO_STATE_S:g} s when RTI stops streaming (not after {RETURN_TIMEOUT_S:g} s), "
        f"speed 0 at the end", why and "no state packet" in why and len(ticks) <= int((NO_STATE_S + 0.2) * FS)
        and st["last_cmd"] == 0.0, (why, len(ticks)))
    sv, rs, pc, st = _plant(4, lambda v: 0.2, 0.0)
    calls = []

    def stop_after_two():
        calls.append(1)
        return "errors: ['J5:WATCHDOG']" if len(calls) > 2 else None
    sweep = [{"v": v, "amp": amplitude_deg(v), "past": past_deg(v), "legs": legs_for(v)} for v in SPEEDS["quick"]]
    recs, why = slide_pose(4, sweep, sv, rs, pc, stop_after_two)
    chk("an error during the sweep stops it before the next slide, keeping what was measured",
        len(recs) == 2 and why == "errors: ['J5:WATCHDOG']", (len(recs), why))
    sv, rs, pc, st = _plant(4, lambda v: 5.0, 0.0)
    recs, why = slide_pose(4, sweep, sv, rs, pc)
    chk("a joint the drive cannot move: no number, the sweep stops with the reason, speed 0 at the end",
        len(recs) == 1 and "friction" not in recs[0] and why and "stuck" in why and st["last_cmd"] == 0.0, why)
    sv, rs, pc, st = _plant(4, lambda v: 0.2 + 0.2 * v, 0.0, inertia=0.02, i_max=0.3)   # 40 deg/s needs 0.34 Nm
    recs, why = slide_pose(4, sweep, sv, rs, pc)
    chk("a drive that cannot reach 40 deg/s (its current limit): that slide is marked as not at its speed and left out "
        "of the analysis, the slower ones kept",
        len(recs) == 3 and "friction" in recs[-1] and speed_share(recs[-1]) < MIN_SPEED_SHARE and not usable(recs[-1])
        and all(usable(r) for r in recs[:2]), [round(speed_share(r), 2) for r in recs if "friction" in r])
    sv, rs, pc, st = _plant(4, lambda v: 0.09, -0.04, inertia=0.0008)
    recs, why = slide_pose(4, [{"v": 5.0, "amp": 4.0, "past": 3.0, "legs": 6},
                               {"v": 40.0, "amp": 16.0, "past": 4.0, "legs": 6}], sv, rs, pc)
    chk("a stick-slipping slide (light joint, 5 deg/s) gets no number and the sweep goes on to the next speed",
        len(recs) == 2 and "stick-slip" in recs[0].get("why", "") and "friction" in recs[1] and why is None,
        [r.get("why") for r in recs])

    print("E. the analysis")

    def rec(v_dps, f, e=0.0, g=1.0, temp=30.0, angle=0.0, kind="spread"):
        return {"joint": 2, "v_cmd_dps": v_dps, "v_plus_dps": v_dps, "v_minus_dps": -v_dps, "friction": f,
                "gravity_error": e, "model_gravity": g, "temp_c": temp, "angle_deg": angle, "kind": kind}
    lin = [rec(v, 0.3 + 0.2 * math.radians(v), angle=a) for v in (5, 15, 40) for a in (-90.0, -60.0)]
    a = analyse(lin)[2]
    chk("a straight line: Coulomb 0.300, viscous 0.200 Nm.s/rad, nothing left over",
        abs(a["fit"]["coulomb"] - 0.3) < 1e-9 and abs(a["fit"]["viscous"] - 0.2) < 1e-9 and a["fit"]["rms"] < 1e-9, a["fit"])
    chk("...the table per speed (3 rows, 2 slides each, lowest 0.3 + 0.2 x 5 deg/s)",
        [row["n"] for row in a["table"]] == [2, 2, 2] and abs(a["lowest"] - (0.3 + 0.2 * math.radians(5))) < 1e-12)
    dip = [rec(v, 0.15 + 0.10 * math.exp(-(math.radians(v) / 0.05) ** 2) + 0.05 * math.radians(v)) for v in (2, 5, 10, 20, 40)]
    a = analyse(dip)[2]
    chk("a dip at low speed: the line misses most at the slowest speed (left > 0 there) — the table keeps the curve",
        a["table"][0]["left"] > 0.02 and a["table"][0]["left"] == max(row["left"] for row in a["table"]),
        [round(row["left"], 3) for row in a["table"]])
    load = [rec(v, 0.5 + 0.05 * g + 0.1 * math.radians(v), g=g) for v in (5, 40) for g in (0.5, 2.0, 5.0)]
    a = analyse(load)[2]
    chk("load: +0.050 Nm per Nm of the model's gravity found (gravity 0.5-5 Nm), with the fit's Coulomb 0.5 and "
        "viscous 0.1 beside it", "load" in a and abs(a["load"]["per_nm"] - 0.05) < 1e-9
        and abs(a["load"]["coulomb"] - 0.5) < 1e-9 and abs(a["load"]["viscous"] - 0.1) < 1e-9, a.get("load"))
    slow = dict(rec(40, 1.5), v_minus_dps=-35.1)
    a = analyse(lin + [slow])[2]
    chk("a slide that did not reach its speed (35.1 of 40 deg/s, the arm's J3 on 2026-10-17) is left out, with the reason",
        a["n_measured"] == 6 and a["refused"] and "did not reach its speed (35.1 of 40" in a["refused"][0], a["refused"])
    chk("...and no load line when the gravity varied under 0.5 Nm", "load" not in analyse(lin)[2])
    warm = [rec(v, 0.6 - 0.004 * (tc - 30.0) + 0.1 * math.radians(v), temp=tc) for v in (5, 40) for tc in (30.0, 38.0, 45.0)]
    a = analyse(warm)[2]
    chk("temperature: -0.004 Nm per C found (30-45 C)", "temperature" in a and abs(a["temperature"]["per_c"] + 0.004) < 1e-9,
        a.get("temperature"))
    gerr = [rec(5, 0.3, e=0.1, g=2.0, angle=-80.0, kind="loaded"), rec(40, 0.4, e=0.3, g=2.0, angle=-80.0, kind="loaded"),
            rec(5, 0.3, e=-0.05, g=0.5, angle=-110.0)]
    a = analyse(gerr)[2]
    chk("the gravity error per pose (the mean of its slides), loaded poses marked",
        [(x["angle_deg"], round(x["error"], 3), x["kind"]) for x in a["gravity_error"]]
        == [(-110.0, -0.05, "spread"), (-80.0, 0.2, "loaded")], a["gravity_error"])
    a = analyse(lin + [{"joint": 2, "v_cmd_dps": 2.0, "why": "did not slide steadily (stick-slip)"}])[2]
    chk("a refused slide is counted with its reason", a["n_slides"] == 7 and a["n_measured"] == 6
        and "stick-slip" in a["refused"][0])
    text = report(analyse(lin + load), xml_now=[0.177, 0.0, 0.369, 0.163, 0.08, 0.07])
    chk("the report: the table, the fit, the load line, today's XML value, and that only --apply writes the XML",
        "J2:" in text and "fit: Coulomb" in text and "load:" in text and "XML today: Coulomb 0.000" in text
        and "written only by --apply" in text)

    print("F. several runs merged")
    import tempfile
    tmp = tempfile.mkdtemp(prefix="rcb77_", dir=os.environ.get("RCB_TMPDIR") or None)
    for k, rows in enumerate((lin, warm)):
        d = os.path.join(tmp, f"run{k}")
        os.makedirs(d)
        with open(os.path.join(d, "run.json"), "w") as fh:
            json.dump({"slides": rows}, fh)
    slides, runs = load_runs([os.path.join(tmp, "run0"), os.path.join(tmp, "run1")])
    chk("--reanalyse: two run.json files merged (12 slides, 2 runs, each slide knows its run)",
        len(slides) == 12 and runs == 2 and {r.get("run") for r in slides} == {"run0", "run1"})
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)

    print("G. the values for the robot XML, and --apply")
    t = typical(lin)
    chk("the line's values: Coulomb 0.300, viscous 0.200, no load term, nothing left out",
        abs(t["coulomb"] - 0.3) < 1e-9 and abs(t["viscous"] - 0.2) < 1e-9 and t["load"] == 0.0 and not t["outliers"], t)
    odd = lin + [rec(v, 1.0 + 0.2 * math.radians(v), e=0.3, angle=354.0) for v in (5, 15, 40)] \
        + [rec(v, 0.33 + 0.2 * math.radians(v), angle=0.0) for v in (5, 15, 40)]
    t = typical(odd)
    chk("a pose over 1.5x the others' median (J3 at 354 deg) is left out; the rest give the line",
        t["outliers"] == [354.0] and t["poses"] == 3 and abs(t["coulomb"] - 0.31) < 0.01, t)
    chk("...but its gravity error still counts (the freedrive margin takes the largest anywhere: 0.3 there)",
        abs(t["gravity_error"] - 0.3) < 1e-9, t["gravity_error"])
    t = typical(lin + [rec(v, 1.0, angle=354.0) for v in (5, 15, 40)])
    chk("two poses are never judged (which one would be odd?); with a third the odd one is found",
        typical(lin)["outliers"] == [] and t["outliers"] == [354.0], t["outliers"])
    t = typical(load)
    chk("the load term when it pays (the J2-like data: 0.050 per Nm, Coulomb 0.5, viscous 0.1, nothing left over)",
        abs(t["load"] - 0.05) < 1e-9 and abs(t["coulomb"] - 0.5) < 1e-9 and abs(t["viscous"] - 0.1) < 1e-9
        and t["rms"] < 1e-9, t)
    rng = np.random.default_rng(3)
    noisy = [rec(v, 0.5 + 0.002 * g + 0.1 * math.radians(v) + float(rng.normal(0, 0.02)), g=g)
             for v in (5, 15, 40) for g in (0.5, 2.0, 5.0)]
    t = typical(noisy)
    chk("...and NOT when it does not (a tiny load effect under noise: the line alone)", t["load"] == 0.0, t)
    neg = [rec(v, 0.4 - 0.05 * math.radians(v), angle=a) for v in (5, 15, 40) for a in (0.0, 30.0)]
    t = typical(neg)
    chk("friction that falls with speed: viscous 0, the Coulomb the median (no negative term in the XML)",
        t["viscous"] == 0.0 and abs(t["coulomb"] - float(np.median([r["friction"] for r in neg]))) < 1e-12, t)
    t = typical(gerr)
    chk("the gravity error: the largest of the poses' means (0.2 at -80 deg)", abs(t["gravity_error"] - 0.2) < 1e-9, t)
    chk("no usable slide: no values", typical([{"joint": 2, "v_cmd_dps": 5.0, "why": "stick-slip"}]) is None)

    import tempfile
    tmp = tempfile.mkdtemp(prefix="rcb77x_", dir=os.environ.get("RCB_TMPDIR") or None)
    try:
        path = os.path.join(tmp, "ROBOT.xml")
        joint = """        <joint id="{i}">
            <name>J{n}</name> <!-- joint {n}'s own comment -->
{body}            <tracking_limit_deg>5</tracking_limit_deg>
        </joint>
"""
        old_form = ("            <friction_sliding_nm>{f}</friction_sliding_nm> <!-- api_tests/54's lowest -->\n"
                    "            <gravity_error_nm>{e}</gravity_error_nm> <!-- while sliding -->\n")
        new_form = ("            <friction_coulomb_nm>0.400</friction_coulomb_nm> <!-- applied before. Was "
                    "<friction_sliding_nm>0.333 (api_tests/54's lowest). -->\n"
                    "            <friction_viscous_nm_s>0.050</friction_viscous_nm_s> <!-- applied before -->\n"
                    "            <friction_load_per_nm>0.000</friction_load_per_nm> <!-- applied before -->\n"
                    "            <gravity_error_nm>0.090</gravity_error_nm> <!-- applied before -->\n")
        bodies = [old_form.format(f=0.177, e=0.035), old_form.format(f=0.0, e=0.0), "",
                  "            <gravity_error_nm>0.031</gravity_error_nm> <!-- only this -->\n", new_form,
                  old_form.format(f=0.070, e=0.019)]
        with open(path, "w") as fh:
            fh.write('<?xml version="1.0"?>\n<!-- the file\'s own header comment -->\n<robot>\n    <joints count="6">\n'
                     + "".join(joint.format(i=i, n=i + 1, body=b) for i, b in enumerate(bodies))
                     + "    </joints>\n</robot>\n")
        before = open(path).read()
        lt = dict(typical(lin), gravity_error=0.117)
        vals = {2: dict(typical(load), gravity_error=0.441), 3: lt, 4: lt, 5: lt}
        backup, written = write_friction_xml(path, vals, tmp, "selftest run--a")   # a '--' as a folder name may hold
        after = open(path).read()
        import xml.etree.ElementTree as ET
        jx = {int(e.get("id")): e for e in ET.fromstring(after).iter("joint") if e.get("id") is not None}

        def val(j, tag):
            e = jx[j].find(tag)
            return None if e is None else float(e.text)

        def tags(j):
            return [e.tag for e in jx[j]]
        chk("--apply, a joint in the old form (J2): Coulomb, viscous, load and gravity error written, "
            "<friction_sliding_nm> replaced in its place, where it came from in the comment",
            written == [2, 3, 4, 5] and abs(val(1, "friction_coulomb_nm") - 0.5) < 1e-9
            and abs(val(1, "friction_viscous_nm_s") - 0.1) < 1e-9 and abs(val(1, "friction_load_per_nm") - 0.05) < 1e-9
            and abs(val(1, "gravity_error_nm") - 0.441) < 1e-9 and val(1, "friction_sliding_nm") is None
            and tags(1) == ["name", "friction_coulomb_nm", "friction_viscous_nm_s", "friction_load_per_nm",
                            "gravity_error_nm", "tracking_limit_deg"]
            and "Was <friction_sliding_nm>0.0 (api_tests/54's lowest)" in after, tags(1))
        chk("...a joint with no friction tags at all (J3): all four added at its end",
            tags(2) == ["name", "tracking_limit_deg", "friction_coulomb_nm", "friction_viscous_nm_s",
                        "friction_load_per_nm", "gravity_error_nm"] and abs(val(2, "friction_coulomb_nm") - 0.3) < 1e-9,
            tags(2))
        chk("...a joint with a gravity error only (J4): the three before it, the error replaced",
            tags(3) == ["name", "friction_coulomb_nm", "friction_viscous_nm_s", "friction_load_per_nm",
                        "gravity_error_nm", "tracking_limit_deg"] and abs(val(3, "gravity_error_nm") - 0.117) < 1e-9,
            tags(3))
        chk("...a joint applied before (J5): replaced in place (no duplicates), the old provenance kept",
            tags(4) == ["name", "friction_coulomb_nm", "friction_viscous_nm_s", "friction_load_per_nm",
                        "gravity_error_nm", "tracking_limit_deg"] and abs(val(4, "friction_coulomb_nm") - 0.3) < 1e-9
            and "Was <friction_sliding_nm>0.333 (api_tests/54's lowest)" in after, tags(4))
        chk("...the joints not measured untouched (J1 keeps its <friction_sliding_nm>, no new tags)",
            val(0, "friction_sliding_nm") == 0.177 and val(0, "friction_coulomb_nm") is None
            and val(0, "gravity_error_nm") == 0.035)
        kept = [x for x in before.splitlines() if "friction" not in x and "gravity_error" not in x]
        chk("...text-level: every other line kept exactly (the comments included), the result parses",
            all(x in after.splitlines() for x in kept) and "the file's own header comment" in after
            and after.count("joint 2's own comment") == 1, [x for x in kept if x not in after.splitlines()][:3])
        chk("...an XML comment never gets a '--' (it would not parse; a run folder name may hold one)",
            "--" not in "".join(re.findall(r"<!--(.*?)-->", after, flags=re.S)))
        chk("...the old file kept as a backup", backup is not None and os.path.exists(backup)
            and open(backup).read() == before, backup)
        backup2, _ = write_friction_xml(path, vals, None, "selftest run--a")
        again = open(path).read()
        chk("...applied again: the same text (no duplicate tags, no backup asked)", again == after and backup2 is None
            and again.count("<friction_coulomb_nm>") == 4 and again.count("<friction_viscous_nm_s>") == 4)
        # the parser's own reading of the friction tags (the whole config also needs the robot's other files beside it)
        from config.xml_parser import robot as xr
        from config.xml_parser.xml_access import get_all_values, parse_robot_config
        _root, xj = parse_robot_config(path)
        coul = xr._friction_coulomb(get_all_values(xj, "friction_coulomb_nm"), get_all_values(xj, "friction_sliding_nm"))
        load_v = xr._nonneg(get_all_values(xj, "friction_load_per_nm"), "friction_load_per_nm", xr.FRICTION_LOAD_MAX)
        visc = xr._nonneg(get_all_values(xj, "friction_viscous_nm_s"), "friction_viscous_nm_s", xr.FRICTION_NM_MAX)
        chk("...the parser reads the written values (J2 0.5 + 0.05 x |g| + 0.1 x v; J1 / J6 their old sliding value)",
            abs(coul[1] - 0.5) < 1e-9 and abs(load_v[1] - 0.05) < 1e-9 and abs(visc[1] - 0.1) < 1e-9
            and abs(coul[0] - 0.177) < 1e-9 and abs(coul[5] - 0.070) < 1e-9 and load_v[0] == 0.0,
            (list(coul), list(load_v)))
        bad_xml = os.path.join(tmp, "broken.xml")
        with open(bad_xml, "w") as fh:
            fh.write('<robot><joint id="1"><friction_sliding_nm>0.1</friction_sliding_nm></joint></robot>')
        try:
            write_friction_xml(bad_xml, {9: vals[2]}, None)
            chk("a joint the XML does not have: refused, nothing written", False)
        except ValueError:
            chk("a joint the XML does not have: refused, nothing written",
                open(bad_xml).read().count("friction_coulomb") == 0)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n{len(bad)} failed" + (f": {bad}" if bad else " - all passed"))
    return 1 if bad else 0


# ------------------------------------------------------------------------------------------------------------ the arm
def load_robot():
    """(cfg, system config, fk_z(q) -> TCP height m, gravity_fn(q) -> Nm) for the active robot."""
    from config import xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    base76 = __import__("76_external_torque")
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    _momentum, gravity_fn, _friction, robot = base76.load_model()
    return cfg, sysc, (lambda q: float(robot.fkine(q).t[2])), gravity_fn


def print_plan(plan, mode):
    for p in plan:
        j = p["joint"] - 1
        lines = (f"  J{p['joint']} at {p['pose'][j]:7.1f} deg ({p['kind']}; pose "
                 + " ".join(f"{x:.0f}" for x in p["pose"]) + f"): model gravity {p['gravity']:+.2f} Nm, TCP down to "
                 f"{1000 * p['z_min']:.0f} mm")
        print(lines + (f" — SKIPPED: {p['skip']}" if "skip" in p else ""))
    sp = plan[0]["speeds"] if plan else []
    print(f"sweep ({mode}): " + ", ".join(f"{s['v']:g} deg/s +/-{s['amp']:g} deg x{s['legs']} legs" for s in sp)
          + f". About {plan_seconds(plan) / 60.0:.0f} min.")


def run_arm(args, cfg, sysc, plan):
    from robotclient import RobotClient, ControlLost
    from _rti_session import rti_session, Pacer
    from project_paths import TEMP_DATA_DIR
    base = __import__("46_monitor_baseline")
    out_dir = TEMP_DATA_DIR / "friction_id" / f"{time.strftime('%Y%m%d_%H%M%S', time.gmtime())}_{args.mode}"
    out_dir.mkdir(parents=True, exist_ok=True)
    run = {"mode": args.mode, "robot": cfg.get("robot_name"), "gripper": sysc.get("active_gripper"),
           "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "plan": plan, "slides": []}

    def save():
        with open(out_dir / "run.json", "w") as fh:
            json.dump(run, fh)

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed, prev_recipe, prev_ff = False, None, False
    try:
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") != "IDLE":
            print(f"Robot must be ENABLED, homed and in IDLE (got state={st.get('state')} homed={st.get('homed')} "
                  f"mode={st.get('mode')}).")
            return 2
        base.claim_control(robot)
        claimed = True
        prev_recipe = st.get("recipe")
        # the friction feedforward would hide what this measures (measured - commanded torque): off, back at the end
        prev_ff = bool(st.get("friction_ff"))
        if prev_ff:
            robot.set_friction_ff(False)
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        robot.set_gravity_comp(True)

        def go_to(pose):
            robot.enter_exec_mode(wait=True)
            robot.move_joints(*[float(x) for x in pose], speed=args.move_speed)
            fails = robot.wait_idle(timeout=120.0, raise_on_error=False)
            robot.enter_idle_mode(wait=True)
            time.sleep(0.5)
            return fails

        def ok_to_go():
            s = robot.status()
            errs = [e for e in (s.get("errors") or []) if not e.endswith("_WARNING")]
            if errs:
                return f"errors: {errs}"
            if s.get("mode") != "RTI":
                return f"the robot left RTI (mode {s.get('mode')})"
            return None

        for item in plan:
            if "skip" in item:
                continue
            j = item["joint"] - 1
            fails = go_to(item["pose"])
            if fails:
                print(f"\nJ{item['joint']} at {item['pose'][j]:.1f} deg: the move there was refused — skipped: "
                      f"{str(fails)[:160]}")
                run["slides"].append({"joint": item["joint"], "pose_deg": item["pose"], "angle_deg": item["pose"][j],
                                      "kind": item["kind"], "why": f"move refused: {str(fails)[:160]}"})
                continue
            print(f"\nJ{item['joint']} at {item['pose'][j]:.1f} deg ({item['kind']}, model gravity "
                  f"{item['gravity']:+.2f} Nm):")
            with rti_session(robot) as (rti, _first):
                pace = Pacer(FS)
                try:
                    recs, why = slide_pose(j, item["speeds"], rti.send_joint_velocity, rti.read_state, pace, ok_to_go)
                except ControlLost as e:
                    recs, why = [], f"control lost: {e}"
            for r in recs:
                r.update(joint=item["joint"], pose_deg=item["pose"], angle_deg=item["pose"][j], kind=item["kind"])
                run["slides"].append(r)
                print(f"   {r['v_cmd_dps']:4.0f} deg/s: " + (
                    f"friction {r['friction']:.3f} Nm at {0.5 * (r['v_plus_dps'] - r['v_minus_dps']):.1f} deg/s, "
                    f"gravity error {r['gravity_error']:+.3f}"
                    + (f", motor {r['temp_c']:.0f} C" if r.get("temp_c") is not None else "")
                    + ("" if usable(r) else f" — {not_usable_why(r)}") if "friction" in r
                    else f"no number — {r['why']}"))
            save()
            if why and "stick-slip" not in why:
                print(f"   stopped: {why}")
                break
    finally:
        if claimed:
            try:
                if prev_recipe:
                    robot.set_recipe(prev_recipe)
                if prev_ff:
                    robot.set_friction_ff(True)
                s = robot.status()
                errs = [e for e in (s.get("errors") or []) if not e.endswith("_WARNING")]
                if errs:                             # IDLE cannot be entered while an error is latched
                    print(f"!! the robot has errors {errs}: Clear Errors (and Enable) before running again")
                elif s.get("mode") != "IDLE":
                    robot.enter_idle_mode(wait=True)
                robot.release_control()
            except Exception as e:
                print(f"!! could not restore everything: {e}")
        robot.disconnect()
    save()
    results = analyse(run["slides"])
    text = (report(results, xml_now=cfg.get("friction_coulomb_nm"))
            + "\n" + typical_report(typical_all(run["slides"]), cfg.get("friction_coulomb_nm")))
    (out_dir / "report.txt").write_text(text + "\n")
    print("\n" + text + f"\n\nsaved {out_dir.relative_to(_ROOT)}/run.json + report.txt")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--joints", default="1,2,3,4,5,6", help="1-based, comma separated (default all six)")
    ap.add_argument("--mode", choices=("quick", "thorough"), default="quick")
    ap.add_argument("--min-z", type=float, default=MIN_Z_M, help="m: skip a pose whose TCP comes under this (default 0.10)")
    ap.add_argument("--move-speed", type=float, default=MOVE_SPEED, help="percent, the EXEC moves between poses (default 10)")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--reanalyse", nargs="+", metavar="FOLDER", help="analyse saved runs (merged)")
    ap.add_argument("--apply", action="store_true",
                    help="with --reanalyse: write the typical values into the robot XML (a backup in the first folder)")
    args = ap.parse_args()
    if args.apply and not args.reanalyse:
        print("--apply works on saved runs: --reanalyse FOLDER [FOLDER ...] --apply")
        return 2
    if args.selftest:
        return selftest()
    if args.reanalyse:
        slides, runs = load_runs(args.reanalyse)
        from config import xml_parser as xp
        from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
        sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
        cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                        active_gripper=sysc["active_gripper"])
        values = typical_all(slides)
        text = (report(analyse(slides), xml_now=cfg.get("friction_coulomb_nm"), runs=runs)
                + "\n" + typical_report(values, cfg.get("friction_coulomb_nm")))
        path = os.path.join(args.reanalyse[0], f"report_merged_{time.strftime('%Y%m%d_%H%M%S', time.gmtime())}.txt")
        with open(path, "w") as fh:
            fh.write(text + "\n")
        print(text + f"\n\nsaved {path}")
        if args.apply:
            if not values:
                print("--apply: nothing measured, nothing written")
                return 2
            src = "runs " + " + ".join(os.path.basename(os.path.normpath(f)) for f in args.reanalyse)
            backup, written = write_friction_xml(str(sysc["robot_xml_path"]), values, args.reanalyse[0], src)
            print(f"\nwritten into {sysc['robot_xml_path']}: " + ", ".join(f"J{jj}" for jj in written)
                  + f" (the old file: {backup}). RTI uses it from its next start (restart + re-home).")
        return 0
    try:
        joints = sorted({int(x) for x in args.joints.split(",") if x.strip()})
    except ValueError:
        print(f"--joints: {args.joints!r} is not a list of joint numbers")
        return 2
    if not joints or not all(1 <= jj <= 6 for jj in joints):
        print("--joints must be joint numbers 1-6")
        return 2
    cfg, sysc, fk_z, gravity_fn = load_robot()
    start = __import__("_common").START_POSE_DEG
    lo = [math.degrees(x) for x in cfg["software_joint_limit_negative"]]
    hi = [math.degrees(x) for x in cfg["software_joint_limit_positive"]]
    plan = make_plan(joints, args.mode, lo, hi, start, cfg.get("robot_name"), fk_z, gravity_fn, args.min_z)
    print(f"friction identification ({args.mode}) on {cfg.get('robot_name')}: {len(plan)} poses, "
          f"{sum('skip' in p for p in plan)} skipped")
    print_plan(plan, args.mode)
    if not args.go:
        print("DRY RUN: nothing moved. Clear the space around the arm, then re-run with --go.")
        return 0
    return run_arm(args, cfg, sysc, plan)


if __name__ == "__main__":
    sys.exit(main())
