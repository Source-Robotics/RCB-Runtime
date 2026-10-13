#!/usr/bin/env python3
"""Self-collision worker: the measured pose against the arm's convex pieces, on another core, into shared memory.

TODO/self_collision_plan.md, docs/self_collision.md (2026-10-06). Started by supervisor.py next to RTI, the GUI and
command_executor. Every cycle (<self_collision><worker_hz>, 100 Hz):

  1. reads the MEASURED joint angles (joint['joint_position']) and runs utility/self_collision.SelfCollisionModel
     .check() with the MONITOR margin -> rti['sc_pair_gaps'][a * 7 + b] (each flagged pair's exact gap, m, negative
     = overlap; +inf for the others — RTI's monitor applies the escape rule with them), rti['sc_hit'] (1 = some pair
     within the margin), rti['sc_pair_a' / 'sc_pair_b'] / rti['sc_min_gap'] (the deepest pair and its gap; the
     planner margin when clear);
  2. while a jog runs, it scans the jog's RAY (RTI's jog handler publishes it: rti['jog_ref'] + s * rti['jog_dir'],
     for rti['jog_session']; one joint or several) with SelfCollisionModel.ray_start / ray_advance — a check every
     0.5 deg, at most 20 per cycle (twice the fastest jog), the first pose within the JOG margin bisected to 0.02 deg
     (from a start already inside it: the first pose that closes a gap or brings a new pair in) — and publishes the
     free length along it: rti['sc_ray_free'] (a verified frontier, never decreasing, then the exact entry) and
     rti['sc_ray_done'], then rti['sc_ray_session']. RTI's jog plans its motion with Ruckig to ARRIVE at that length
     with zero speed: a jog comes to rest at the jog margin at any speed (2026-10-07, the user's rule; before, a
     triggered block + the jog's fixed braking rested it 5-22 mm away depending on the speed);
  3. rti['sc_heartbeat'] += 1. RTI's safety monitor turns a stalled heartbeat into SELF_COLLISION_STALE.

The jaws (2026-10-07, S3f; <self_collision><jaws>): "swept" — the gripper file's pieces, each jaw over its whole stroke.
"measured" — a second model with the jaws unswept, placed every cycle where the gripper says they are
(jaw_closed_fraction: firmware mode — calibrated — or motor mode — homed toward its open end; fresh data either way;
anything else -> the swept model for that cycle), widened by <jaw_band_mm>. A jog's scan keeps the model and the jaws it
started with. rti['sc_jaw_closed'] says which (-1 = swept).
The dashboard's readout (S3g): every 20th cycle (5 Hz, the badge's own refresh) the pair nearest its own jog margin, if
within <readout_mm> of it ->
rti['sc_near_a' / 'sc_near_b' / 'sc_near_gap' / 'sc_near_margin']; rti['sc_cycle_ms'] = the slowest recent cycle.

Until the robot is HOMED (every rti['joint_homed'] AND homed_state "HOMED" — a lost drive clears only the latter) the
measured angles mean nothing — at the first start on the arm the
worker "saw" the base 26 mm inside the upper arm at q = -343 246 deg, and homing's endstop seeking reads as folded
poses — so it then publishes clear / no block and only the heartbeat (RTI's monitor judges nothing unhomed either).

Why a process and not RTI's tick: a folded pose costs the check 1-2.5 ms, too much for the 4 ms tick; measured
2026-10-06, a worker on cores 0-2 at 100 Hz adds ~0.07 ms to RTI's execution time and no overruns. Why not a
thread in command_executor: Pinocchio's bindings hold the GIL. Latency: one cycle (10 ms) — 3 mm of gap at jog
speed, 14 mm with every joint at its ceiling; the monitor margin and the jog look-ahead cover it.

Disabled (<self_collision><enabled>0</enabled>) it idles so the supervisor does not restart it. Logs to
logs/self_collision.log.
"""
from __future__ import annotations

import logging
import math
import os
import signal
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _ROOT)

from project_paths import GRIPPERS_DIR, HIGH_LEVEL_LOG, ROBOTS_DIR, SYSTEM_XML   # noqa: E402
from utility.logger_setup import setup_logging                                   # noqa: E402

LOG_FILE = os.path.join(os.path.dirname(str(HIGH_LEVEL_LOG)), "self_collision.log")
from config.cpu_cores import GENERAL_CPU_CORES                            # noqa: E402
CORES = set(GENERAL_CPU_CORES)    # never RTI's core (3), nor the CAN core (2) — config/cpu_cores.py
from robotics.jog import ray_soft_free                                   # noqa: E402  the jog's own soft-limit length
STATS_EVERY_S = 60.0
_stop = False


def _on_term(_s, _f):
    global _stop
    _stop = True


STROKE_MATCH_MM = 3.0             # the gripper XML's <Stroke> vs its collision file's (both jaws): measured jaws need them to agree


def measured_jaws_refusal(cfg: dict, tool_geo: dict | None):
    """Why the jaws must stay swept for this gripper, or None. Measured jaws need a CAN gripper whose collision file lists
    its jaws, with the stroke the gripper XML declares: the MSG grippers with 100 / 200 mm rails share the 150 mm rail's
    file (105 mm of stroke) — placed by THEIR 0-255 the jaws would land in the wrong place (the 200 mm one beyond the
    model)."""
    can = cfg.get("gripper_CAN_gripper")
    if can is None or (isinstance(can, float) and math.isnan(can)) or int(can) != 1:
        return "the gripper is not a CAN gripper"
    jaws = (tool_geo or {}).get("jaws")
    if not jaws:
        return "its collision file lists no jaws (re-export it: tools/self_collision_capsules.py fit + export)"
    try:
        xml_mm = float(cfg.get("gripper_stroke"))      # type: ignore[arg-type]  # None -> TypeError, caught below
    except (TypeError, ValueError):
        return "the gripper XML declares no <Stroke>"
    file_mm = 1e3 * sum(float(j["stroke_m"]) for j in jaws)
    if not abs(xml_mm - file_mm) <= STROKE_MATCH_MM:
        return (f"its XML's <Stroke> ({xml_mm:g} mm) is not its collision file's ({file_mm:.1f} mm, both jaws) — that file is "
                f"another rail's shape")
    return None


# The gripper's position as firmware mode reports it: gripper['gripper_position'] 0-255 = the motor's encoder mapped
# linearly between the two endstops the gripper found when it calibrated, 0 = open, 255 = closed (STEPFOC and Spectral
# BLDC firmware: mapAndConstrain(Position_Ticks, max_closed_position, max_open_position), read 2026-10-07).
GRIPPER_RAW_CLOSED = 255.0
MOTOR_MODE, FIRMWARE_MODE = 1, 2  # rti['gripper_ctrl_mode']: 0 none, 1 motor mode, 2 firmware mode
READOUT_EVERY = 20                # cycles between two readouts (5 Hz at 100 Hz: a readout costs ~2 ms, the badge refreshes 5x/s)
OPENING_SLACK_M = 0.005           # motor mode: an opening this far outside 0..stroke means the reference is wrong -> swept


def jaw_closed_fraction(rti, gripper, node: int, homed_state: str = "", motor_ok: bool = False):
    """The jaws' closing fraction (0 open .. 1 closed) when the gripper's report can be trusted, else None. The node's
    data must be fresh, then:
      firmware mode — calibrated (its endstops found): gripper['gripper_position'] / 255;
      motor mode (2026-10-07, user: both modes) — only for a gripper that homes toward its OPEN end (`motor_ok`:
        <homing_direction> 0, positive speed, which is how the firmware's own calibration opens), with the robot HOMED
        and the gripper's own homing done (gripper_homed) and its scale known (ticks per metre): then
        gripper['gripper_joint_position'] is the opening in metres, 0 closed .. <Stroke> open (data/update_shared_data.py:
        the endstop it homed against = the stroke), so 1 - opening / stroke. An opening outside 0..stroke (by more than
        OPENING_SLACK_M) means the reference is wrong -> None."""
    if int(rti["old_data_warning"][node]) != 0:
        return None
    mode = int(rti["gripper_ctrl_mode"][0])
    if mode == FIRMWARE_MODE:
        if int(gripper["gripper_calibrated"][0]) != 1:
            return None
        return min(max(float(gripper["gripper_position"][0]) / GRIPPER_RAW_CLOSED, 0.0), 1.0)
    if mode == MOTOR_MODE and motor_ok:
        if homed_state != "HOMED" or int(gripper["gripper_homed"][0]) != 1:
            return None
        stroke, tpm = float(gripper["gripper_stroke_m"][0]), float(gripper["gripper_ticks_per_meter"][0])
        if not (stroke > 0.0 and tpm > 0.0):
            return None
        opening = float(gripper["gripper_joint_position"][0])
        if not (-OPENING_SLACK_M <= opening <= stroke + OPENING_SLACK_M):
            return None
        return min(max(1.0 - opening / stroke, 0.0), 1.0)
    return None


def ray_cycle(model, rti, joint_num, soft_lo, soft_hi, state) -> None:
    """One worker cycle of the jog's ray scan. A new rti['jog_session'] starts a scan of its ray (the soft limits
    along it are the horizon: beyond them the jog stops anyway); each cycle advances it and publishes the free length
    — sc_ray_free and sc_ray_done FIRST, sc_ray_session LAST, so the jog never pairs a session with another one's
    length. state = {"session": the session scanned, "scan": the scan or None}. `model` is the one in use this cycle;
    a scan keeps the model (and the jaws) it started with."""
    sess = float(rti["jog_session"][0])
    if sess != state["session"]:
        state["session"], state["scan"] = sess, None
        d = rti["jog_dir"][:joint_num]
        if float(np.dot(d, d)) > 0.25:                    # a unit vector: a jog direction (zeros: none)
            ref = np.array(rti["jog_ref"][:joint_num])
            state["scan"] = model.ray_start(ref, d, min(ray_soft_free(ref, d, soft_lo, soft_hi), 2.0 * math.pi))
            state["model"] = model
            rti["sc_ray_free"][0], rti["sc_ray_done"][0] = 0.0, 0.0
            rti["sc_ray_session"][0] = sess
    scan = state["scan"]
    if scan is None or scan["done"] and state.get("published") == (sess, scan["s_free"]):
        return
    model = state.get("model") or model
    model.ray_advance(scan)
    rti["sc_ray_free"][0] = scan["s_free"]
    rti["sc_ray_done"][0] = 1.0 if scan["done"] else 0.0
    rti["sc_ray_session"][0] = sess
    state["published"] = (sess, scan["s_free"])
    if scan["done"] and scan["hit"]:
        seen = ", ".join(f"{model.describe(a_, b_)} {1e3 * g_:.1f} mm" for (a_, b_), g_ in scan["hit"].items())
        stop = np.degrees(scan["q_ref"] + scan["s_free"] * scan["dir"])
        jaw_q = scan.get("jaw_q")
        jaws = (f"jaws {100 * float(jaw_q[0] / model.jaw_strokes[0]):.0f} % closed (as at the jog's start)"
                if model.jaws == "measured" and jaw_q is not None and jaw_q.size else "jaws swept")
        logging.info(f"[SC] jog session {int(sess)}: the jog margin begins {math.degrees(scan['s_free']):.2f} deg along its ray "
                     f"({seen}){' — escaping: the start is inside the margin' if scan['inside'] else ''}; stops at "
                     f"{np.round(stop, 2).tolist()} deg, {jaws}")


def main() -> int:
    setup_logging(LOG_FILE, log_to_console=False)
    try:
        os.sched_setaffinity(0, CORES)
    except (AttributeError, OSError) as e:                                        # pragma: no cover
        logging.warning(f"[SC] could not pin to cores {sorted(CORES)}: {e}")
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGINT, _on_term)

    import config.xml_parser as xp
    _sys = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(_sys["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=_sys["active_gripper"])
    policy = _sys["self_collision"]
    joint_num = int(cfg["joint_num"])
    from data import shared_data
    shared_data.setup(joints=joint_num)
    rti, joint, gripper = shared_data.rti, shared_data.joint, shared_data.gripper

    if not policy["enabled"]:
        logging.info("[SC] self-collision checking is OFF in config/system.xml — idling")
        while not _stop:
            time.sleep(1.0)
        return 0

    from utility.robot_factory import create_robot
    from utility.self_collision import SelfCollisionModel, load_geometry
    try:
        robot = create_robot(_sys["active_robot"], gripper_name=cfg.get("gripper_name", ""))
        robot_geo, tool_geo = load_geometry(ROBOTS_DIR, GRIPPERS_DIR, cfg)
        pairs = cfg["self_collision"]["pairs"] + cfg["gripper_collision"]["pairs"]
        model = SelfCollisionModel(robot, robot_geo, tool_geo, policy, pairs)
    except Exception as e:
        logging.critical(f"[SC] cannot build the self-collision model: {e} — fix the geometry files named in the robot / "
                         f"gripper XML, or set <self_collision><enabled>0</enabled>. Nothing is watching self-collisions.")
        raise SystemExit(2)
    # the measured-jaws model (S3f): only for a CAN gripper whose file lists its jaws; the swept model stays the fallback
    measured = None
    _hd = cfg.get("gripper_homing_direction")
    motor_ok = _hd is not None and not (isinstance(_hd, float) and math.isnan(_hd)) and int(_hd) == 0   # homes toward OPEN
    if policy.get("jaws") == "measured":
        why_not = measured_jaws_refusal(cfg, tool_geo)
        if why_not:
            logging.warning(f"[SC] <jaws>measured</jaws> but {why_not} — the jaws stay swept over their stroke")
        else:
            measured = SelfCollisionModel(robot, robot_geo, tool_geo, policy, pairs, jaws="measured",
                                          jaw_band_m=float(policy["jaw_band_mm"]) / 1000.0)

    period = 1.0 / float(policy["worker_hz"])
    planner_margin = float(policy["planner_margin_mm"]) / 1000.0
    soft_lo = np.asarray(cfg["software_joint_limit_negative"], dtype=np.float64)[:joint_num]
    soft_hi = np.asarray(cfg["software_joint_limit_positive"], dtype=np.float64)[:joint_num]
    logging.info(f"[SC] worker up: {model.geom.ngeoms} pieces, {model.n_pairs} pairs, {policy['worker_hz']:g} Hz, margins planner "
                 f"{1e3 * planner_margin:g} / jog {policy['jog_margin_mm']:g} / monitor {policy['monitor_margin_mm']:g} mm, jaws "
                 + (f"measured (band {policy['jaw_band_mm']:g} mm, else swept)" if measured is not None else "swept")
                 + f", cores {sorted(CORES)}, log {os.path.relpath(LOG_FILE, _ROOT)}")

    q = np.zeros(joint_num)
    homed_view = rti["joint_homed"]
    pair_gaps = np.full(49, np.inf)                       # written whole into rti['sc_pair_gaps'] each cycle
    rti["sc_pair_gaps"][:] = np.inf
    rti["sc_near_a"][0], rti["sc_near_b"][0], rti["sc_near_gap"][0], rti["sc_near_margin"][0] = -1.0, -1.0, np.inf, 0.0
    rti["sc_jaw_closed"][0], rti["sc_cycle_ms"][0] = -1.0, 0.0
    heartbeat = 0.0
    hit_live, homed_live = False, None
    jaws_live = None                                      # the jaw source last logged: "measured" / "swept"
    cycle_ms, n_cycle = 0.0, 0
    ray_state = {"session": -1.0, "scan": None}       # the jog's ray being scanned
    cycle_times, next_stats = [], time.monotonic() + STATS_EVERY_S
    t_next = time.monotonic()
    while not _stop:
        t0 = time.perf_counter()
        homed = bool(np.all(homed_view[:joint_num] == 1)) and shared_data.homed_state.get() == "HOMED"  # type: ignore[union-attr]  # set by shared_data.setup() at start
        if homed != homed_live:
            homed_live = homed
            logging.info("[SC] robot homed — watching the measured pose" if homed else
                         "[SC] robot not homed — the measured angles mean nothing until it is: clear / no block, heartbeat only")
        if not homed:
            rti["sc_hit"][0], rti["sc_min_gap"][0] = 0.0, planner_margin
            rti["sc_pair_gaps"][:] = np.inf
            rti["sc_near_a"][0], rti["sc_near_b"][0], rti["sc_near_gap"][0] = -1.0, -1.0, np.inf
            hit_live = False
            heartbeat += 1.0
            rti["sc_heartbeat"][0] = heartbeat
            t_next += period
            sleep = t_next - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            elif sleep < -period:
                t_next = time.monotonic()
            continue
        q[:] = joint["joint_position"][:joint_num]
        # 0. the jaws: where the gripper says they are (measured), else swept over their stroke
        active = model
        if measured is not None:
            c = jaw_closed_fraction(rti, gripper, joint_num, shared_data.homed_state.get(), motor_ok)  # type: ignore[union-attr]  # (the same)
            src = "measured" if c is not None else "swept"
            if src != jaws_live:
                jaws_live = src
                logging.info(f"[SC] jaws: placed where the gripper reports them ({'firmware' if int(rti['gripper_ctrl_mode'][0]) == FIRMWARE_MODE else 'motor'} mode)"
                             if c is not None else
                             "[SC] jaws: swept over their stroke — the gripper's position is not trustworthy now (firmware mode "
                             "uncalibrated, motor mode not homed" + ("" if motor_ok else " or homing toward its closed end") + ", or stale)")
            if c is not None:
                measured.set_jaws(c)
                active = measured
            rti["sc_jaw_closed"][0] = -1.0 if c is None else c
        # 1. the measured pose against the monitor margin
        hits = active.check(q, "monitor")
        if hits:
            gaps = active.hit_gaps("monitor")             # only the flagged piece pairs: cheap even in contact
            pair_gaps.fill(np.inf)
            for (pa, pb), g in gaps.items():
                pair_gaps[pa * 7 + pb] = g
            (a, b), gap = min(gaps.items(), key=lambda kv: kv[1])
            rti["sc_pair_gaps"][:] = pair_gaps            # the gaps BEFORE the flag: a hit always has its gaps
            rti["sc_pair_a"][0], rti["sc_pair_b"][0], rti["sc_min_gap"][0] = a, b, gap
            rti["sc_hit"][0] = 1.0
            if not hit_live:
                hit_live = True
                logging.warning(f"[SC] within the monitor margin: {active.describe(a, b)} {1e3 * gap:.1f} mm apart "
                                f"(q = {np.round(np.degrees(q), 1).tolist()} deg"
                                + (f", jaws {100 * active.jaw_closed:.0f} % closed" if active.jaws == "measured" else "") + ")")  # type: ignore[operator]  # jaws == 'measured' -> jaw_closed is a float
        else:
            rti["sc_hit"][0] = 0.0                        # the flag BEFORE the gaps (see above)
            rti["sc_pair_gaps"][:] = np.inf
            rti["sc_min_gap"][0] = planner_margin
            if hit_live:
                hit_live = False
                logging.info("[SC] clear again")
        # 1b. the dashboard's readout (every 20th cycle; before the ray, which may move the jaws to its session's)
        n_cycle += 1
        if n_cycle % READOUT_EVERY == 0:
            near = active.readout(q)
            if near is None:
                rti["sc_near_gap"][0] = np.inf
                rti["sc_near_a"][0], rti["sc_near_b"][0] = -1.0, -1.0
            else:
                rti["sc_near_a"][0], rti["sc_near_b"][0], rti["sc_near_gap"][0], rti["sc_near_margin"][0] = near
        # 2. the jog's ray: how far it is free of the jog margin (RTI's jog plans its stop to arrive there)
        ray_cycle(active, rti, joint_num, soft_lo, soft_hi, ray_state)
        # 3. the heartbeat
        heartbeat += 1.0
        rti["sc_heartbeat"][0] = heartbeat
        dt = time.perf_counter() - t0
        cycle_ms = max(1e3 * dt, 0.99 * cycle_ms)          # the slowest recent cycle, decaying over ~1 s
        rti["sc_cycle_ms"][0] = cycle_ms
        cycle_times.append(dt)
        now = time.monotonic()
        if now >= next_stats:
            ct = np.array(cycle_times) * 1e3
            logging.info(f"[SC] {len(ct)} cycles: median {np.median(ct):.2f} ms, p99 {np.percentile(ct, 99):.2f}, max {ct.max():.2f} ms "
                         f"({100 * ct.mean() / (1e3 * period):.0f} % of the {1e3 * period:.0f} ms period)")
            cycle_times.clear(); next_stats = now + STATS_EVERY_S
        t_next += period
        sleep = t_next - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        elif sleep < -period:
            t_next = time.monotonic()                              # far behind: re-base, no catch-up burst
    logging.info("[SC] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
