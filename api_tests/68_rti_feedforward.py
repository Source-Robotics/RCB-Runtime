#!/usr/bin/env python3
"""68: does inertia / Coriolis feedforward help RTI and SOFT_RTI streaming? Measured on the arm (2026-10-11).

TODO/dynamics_feedforward_plan.md F3. RTI / SOFT_RTI never feed forward their GENERATOR's acceleration (it switches
between its limits as the planner chases each target — simulated up to ±1.8 Nm on J1). The opt-in source "stream"
(`rti_set_dynamics_ff("stream")`) feeds forward the STREAM's own acceleration: the change of the target velocity
between packets over their interval, smoothed at 8 Hz. Simulated against the torque the commanded motion needs
(1.0 = no better than none): hard RTI 250 Hz 0.12, SOFT_RTI 50 Hz 0.27, wandering 50-60 Hz 0.33, 10 Hz 0.69.
This test measures it. Every SENDER runs every SET in turn:

  senders  hard250 (hard RTI, a command every tick) / soft50 (SOFT_RTI 50 Hz) / wander (SOFT_RTI, 50-60 Hz wandering,
           like a VR device) / soft10 (SOFT_RTI 10 Hz)
  sets     off / stream (default order off,stream,off — the second off checks the measurement repeats)
           The robot-wide inertia and Coriolis switches are ON for every set (put back as they were at the end).

Each block: a session (connect — the robot must really be in the sender's contract, RTI or SOFT_RTI, or the block is
refused —, configure, claim), 1 s still, then a J1 sine around where the arm is (default ±15 deg,
1.5 s period, faded in and out over 1 s so it starts and ends at rest), 1 s still, release, DISCONNECT (back to IDLE:
RTI <-> SOFT_RTI only change through IDLE — the first run, 2026-10-11, stayed in RTI after the hard blocks, so every
'soft' block ran hard RTI fed at 50 Hz, and the 10 Hz one tripped the 40 ms watchdog). Any failure ends the run, and
what was recorded is still saved. Recorded tick by tick from
the robot's own UDP telemetry (recipe `tracking`, as api_tests/66).

WHAT IS MEASURED, per block and moving joint (error = commanded - measured position; the commanded motion is the
generator's, which is what the arm is asked to follow):
  rms_d / peak_d  the tracking error after the best time shift (0-20 ms): the PATH error the feedforward should cut
  delay_ms        that shift
  buzz            RMS of the measured joint torque minus its 108 ms moving mean (above ~10 Hz, Nm), only where the
                  joint moves steadily (> 30 % of its peak speed, the filter's width away from the rest): vibration —
                  what a bad feedforward would raise; the torque's step at each reversal (friction) is left out
  ff / ff_hf      RMS of the feedforward torque we added (inertia + Coriolis, Nm), and its part above ~10 Hz
  pid             RMS of measured - commanded torque while moving: what the drive's own loops had to add
  acc_hf          the published acceleration's part above ~10 Hz (rad/s^2): the generator's (off) or the stream's
  cmd_vhf /       the COMMANDED / MEASURED speed's part above ~10 Hz during steady motion (deg/s): jerkiness — the
  meas_vhf        generator's chatter in what the drive is told, and what the arm really did (speed, not position:
                  the slow sine leaks ~0.09 deg through the filter, its speed only ~0.5 deg/s — the floor here)
One line per sender and set; then each joint's mean over the senders, relative to `off`.

The robot must be ENABLED and homed; the arm moves J1 only by default (`--joints 1,4,5,6` allowed — J2 / J3 are not,
as every servo test). The planner is not involved: RTI streams are bounded by the soft joint limits and the RTI limits,
so put the arm where J1 has room (the start pose of the other tests: J1 90 deg). Ctrl+C stops the arm.

    python3 api_tests/68_rti_feedforward.py --selftest           # the analysis on synthetic data
    python3 api_tests/68_rti_feedforward.py                      # dry run: the plan
    python3 api_tests/68_rti_feedforward.py --go                 # ~4 min, J1
    python3 api_tests/68_rti_feedforward.py --go --senders soft50,wander --sets off,stream
    python3 api_tests/68_rti_feedforward.py --reanalyse temp_data/rti_ff/<time>.json   # with the arm DISABLED

As api_tests/66: the RAW packets are saved the moment the run ends; with the arm still ENABLED the analysis waits
for --reanalyse (it is heavy enough to slow RTI's loop). Run it inside tmux.
"""
import argparse
import json
import math
import os
import random
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
from _tracking_common import Capture, to_arrays, best_delay, robot_enabled  # noqa: E402

RECIPE = "tracking"
SENDERS = {                       # name: (lo Hz, hi Hz, soft)
    "hard250": (250.0, 250.0, False),
    "soft50": (50.0, 50.0, True),
    "wander": (50.0, 60.0, True),
    "soft10": (10.0, 10.0, True),
}
SETS = ("off", "stream")
SAFE_JOINTS1 = (1, 4, 5, 6)       # 1-based; the servo tests' rule (api_tests/_servo_common.SAFE_JOINTS)
CLEARANCE_DEG = 10.0
STILL_S = 1.0
FADE_S = 1.0
MOVING = 1e-3                     # rad/s commanded: "moving"
HF_W = 13                         # moving-mean half width (ticks): 27 ticks = 108 ms -> above ~10 Hz
METRICS = ("rms_d", "peak_d", "delay_ms", "buzz", "ff", "ff_hf", "pid", "acc_hf", "cmd_vhf", "meas_vhf", "vpeak")


# ----------------------------------------------------------------------------- the motion (pure)
def envelope(t, total):
    """0 -> 1 over FADE_S (raised cosine), 1, then -> 0 over the last FADE_S: the sine starts and ends at rest."""
    if t <= 0.0 or t >= total:
        return 0.0
    if t < FADE_S:
        return 0.5 - 0.5 * math.cos(math.pi * t / FADE_S)
    if t > total - FADE_S:
        return 0.5 - 0.5 * math.cos(math.pi * (total - t) / FADE_S)
    return 1.0


def offset(t, total, amp_rad, period_s):
    return amp_rad * envelope(t, total) * math.sin(2.0 * math.pi * t / period_s)


# ----------------------------------------------------------------------------- analysis (pure)
def hf(x, w=HF_W):
    """x minus its centred (2w+1)-sample moving mean (api_tests/40's high-pass), edges dropped."""
    if x.size <= 2 * w + 1:
        return np.zeros(0)
    k = np.ones(2 * w + 1) / (2 * w + 1)
    return x[w:-w] - np.convolve(x, k, mode="valid")


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if x.size else None


def steady_mask(speed, frac=0.3, w=HF_W):
    """For hf()'s output (2w shorter than `speed`): True where the joint moves at > frac of its peak speed and no
    excluded sample lies within the filter's half width — a reversal's torque step cannot leak into it."""
    sp = np.abs(speed)
    bad = (sp < frac * sp.max()).astype(float)
    near = np.convolve(bad, np.ones(2 * w + 1), mode="same") > 0
    return ~near[w:-w]


def block_metrics(s, j, dt):
    """Metrics of joint j (0-based) in one block's segment (arrays from to_arrays)."""
    moving = np.abs(s["j_sc"][:, j]) > MOVING
    if moving.sum() < 50:
        return None
    first, last = int(np.flatnonzero(moving)[0]), int(np.flatnonzero(moving)[-1])
    win = slice(first, last + 1)
    delay, res = best_delay(s["tick"] * dt, s["j_pc"][:, j], s["j_p"][:, j], moving)
    ff = (s["j_t_inertia"][:, j] + s["j_t_coriolis"][:, j])[win]
    return {"rms_d": float(np.sqrt(np.mean(res ** 2))), "peak_d": float(np.abs(res).max()),
            "delay_ms": 1000.0 * delay, "buzz": rms(hf(s["j_t"][win, j])[steady_mask(s["j_sc"][win, j])]),
            "ff": rms(ff), "ff_hf": rms(hf(ff)),
            "pid": rms((s["j_t"][:, j] - s["j_tc"][:, j])[moving]), "acc_hf": rms(hf(s["j_ac"][win, j])),
            "cmd_vhf": rms(hf(np.degrees(s["j_sc"][win, j]))[steady_mask(s["j_sc"][win, j])]),
            "meas_vhf": rms(hf(np.degrees(s["j_s"][win, j]))[steady_mask(s["j_sc"][win, j])]),
            "vpeak": float(np.degrees(np.abs(s["j_sc"][:, j]).max()))}


def analyse(d, records, dt):
    out = []
    for r in records:
        m = (d["tick"] >= r["t0"]) & (d["tick"] <= r["t1"])
        s = {k: v[m] for k, v in d.items()}
        if s["tick"].size == 0:
            continue
        for j1 in r["joints"]:
            mm = block_metrics(s, j1 - 1, dt)
            if mm is not None:
                out.append({"sender": r["sender"], "set": r["set"], "joint": j1, **mm})
    return out


def label_sets(names):
    """'off','stream','off' -> 'off','stream','off2'."""
    seen, out = {}, []
    for n in names:
        seen[n] = seen.get(n, 0) + 1
        out.append(n if seen[n] == 1 else f"{n}{seen[n]}")
    return out


def _fmt(v, nd=3):
    return "    -  " if v is None else f"{v:7.{nd}f}"


def print_report(moves, senders, set_labels, title=""):
    if title:
        print(title)
    cols = [("vpeak", 1), ("rms_d", 4), ("peak_d", 3), ("delay_ms", 1), ("buzz", 4), ("ff", 3), ("ff_hf", 4),
            ("pid", 3), ("acc_hf", 2), ("cmd_vhf", 2), ("meas_vhf", 2)]
    print("  (J = joint; deg, ms, Nm, rad/s^2; rms_d / peak_d = tracking error after the best time shift 'delay';"
          " buzz = torque above ~10 Hz)")
    joints = sorted({m["joint"] for m in moves})
    for j in joints:
        print(f"\n  J{j}")
        print(f"    {'sender':>8s} {'set':>8s}  " + "  ".join(f"{c[0][:7]:>7s}" for c in cols))
        for snd in senders:
            for lab in set_labels:
                v = next((m for m in moves if m["joint"] == j and m["sender"] == snd and m["set"] == lab), None)
                if v is not None:
                    print(f"    {snd:>8s} {lab:>8s}  " + "  ".join(_fmt(v.get(c), nd) for c, nd in cols))
    base = [lab for lab in set_labels if lab.startswith("off")]
    others = [lab for lab in set_labels if not lab.startswith("off")]
    if base and others:
        for key, what in (("rms_d", "tracking error after the time shift"), ("buzz", "buzz (torque above ~10 Hz)"),
                          ("pid", "torque the drive's own loops added"),
                          ("meas_vhf", "jerkiness (measured speed above ~10 Hz)")):
            print(f"\n  {what}, per sender, relative to 'off' (mean of the off sets):")
            for j in joints:
                for snd in senders:
                    offs = [m[key] for m in moves if m["joint"] == j and m["sender"] == snd and m["set"] in base
                            and m.get(key) is not None]
                    if not offs:
                        continue
                    b = float(np.mean(offs))
                    cells = []
                    for lab in others:
                        v = next((m[key] for m in moves if m["joint"] == j and m["sender"] == snd and m["set"] == lab
                                  and m.get(key) is not None), None)
                        if v is not None and b > 0:
                            cells.append(f"{lab} {v / b:.2f}x")
                    print(f"    J{j} {snd:>8s}: off {b:.4f}   " + "   ".join(cells))
        if not any(m["ff"] for m in moves if m["set"] in others and m.get("ff") is not None):
            print("\n  !!! The 'stream' sets added NO feedforward — the running RTI does not know rti_set_dynamics_ff "
                  "(restart the service), or the inertia switch was off.")


# ----------------------------------------------------------------------------- self-test
def selftest():
    dt = 0.004
    ok_all = True

    def chk(name, cond, detail=""):
        nonlocal ok_all
        ok_all &= bool(cond)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail and not cond else ""))

    total, amp, per = 6.0, math.radians(15.0), 1.5
    chk("envelope: at rest at both ends, 1 in the middle, continuous",
        envelope(0.0, total) == 0.0 and envelope(total, total) == 0.0 and envelope(3.0, total) == 1.0
        and abs(envelope(FADE_S - 1e-9, total) - 1.0) < 1e-6 and abs(envelope(0.5, total) - 0.5) < 1e-9)
    tt = np.arange(0, total + 2 * STILL_S, dt)
    cmd = np.array([offset(x - STILL_S, total, amp, per) for x in tt])
    vel = np.gradient(cmd, dt)
    acc = np.gradient(vel, dt)
    chk("the commanded motion starts and ends still (|v| < 1e-6 outside the sine)",
        np.abs(vel[tt < STILL_S - dt]).max() < 1e-6 and np.abs(vel[tt > STILL_S + total + dt]).max() < 1e-6)
    # synthetic block: measured = commanded 6 ms late; the feedforward is 0.1 * acc plus a 40 Hz 0.01 Nm ripple; the
    # measured torque = the commanded one (so it carries that ripple too) + 0.05 Nm friction + a 60 Hz 0.02 Nm buzz
    meas = np.interp(tt - 0.006, tt, cmd)
    n = tt.size
    jp = np.zeros((n, 6)); jpc = np.zeros((n, 6)); jsc = np.zeros((n, 6)); jac = np.zeros((n, 6))
    jt = np.zeros((n, 6)); jtc = np.zeros((n, 6)); jti = np.zeros((n, 6)); jtco = np.zeros((n, 6)); js = np.zeros((n, 6))
    jp[:, 0], jpc[:, 0], jsc[:, 0], jac[:, 0] = meas, cmd, vel, acc
    js[:, 0] = vel + 0.1 * np.sin(2 * np.pi * 10 * tt)                # the arm's speed wobbles 0.1 rad/s at 10 Hz
    ripple = 0.01 * np.sin(2 * np.pi * 40 * tt)
    jti[:, 0] = 0.1 * acc + ripple
    jtc[:, 0] = 1.0 + jti[:, 0]
    jt[:, 0] = jtc[:, 0] + 0.05 * np.sign(vel) + 0.02 * np.sin(2 * np.pi * 60 * tt)
    f = lambda a, k=5: ",".join(f"{x:.{k}f}" for x in a)
    pk = [(f"j_p:{f(jp[i])},j_pc:{f(jpc[i])},j_s:{f(js[i], 4)},j_sc:{f(jsc[i], 4)},j_ac:{f(jac[i], 3)},"
           f"j_t:{f(jt[i], 3)},j_tc:{f(jtc[i], 3)},j_t_inertia:{f(jti[i], 3)},j_t_coriolis:{f(jtco[i], 3)},"
           f"tick:{1000 + i},mode:SOFT_RTI").encode() for i in range(n)]
    d = to_arrays(pk)
    chk("packets parsed in tick order", d["tick"].size == n, d["tick"].size)
    moves = analyse(d, [{"sender": "soft50", "set": "stream", "joints": [1], "t0": 1000, "t1": 1000 + n - 1}], dt)
    chk("one result for J1", len(moves) == 1 and moves[0]["joint"] == 1, moves)
    m = moves[0]
    chk("a pure 6 ms delay: found, and almost nothing left after it", abs(m["delay_ms"] - 6.0) <= 0.25 + 1e-9
        and m["rms_d"] < 0.002, (m["delay_ms"], m["rms_d"]))
    want_buzz = math.sqrt(0.02 ** 2 / 2 + 0.01 ** 2 / 2)
    chk(f"buzz = the 60 Hz wobble + the feedforward's 40 Hz ripple (RMS {want_buzz:.4f}), not the slow torque",
        abs(m["buzz"] - want_buzz) < 0.0025, m["buzz"])
    mv = np.abs(vel) > MOVING
    want_ff = float(np.sqrt(np.mean((0.1 * acc[mv] + ripple[mv]) ** 2)))
    chk("ff = RMS of the added torque while moving", abs(m["ff"] - want_ff) < 0.02 * want_ff + 1e-3, (m["ff"], want_ff))
    w0, w1 = int(np.flatnonzero(mv)[0]), int(np.flatnonzero(mv)[-1]) + 1
    want_hf = rms(hf((0.1 * acc + ripple)[w0:w1]))                        # the added torque's own high part
    other_hf = rms(hf(jt[w0:w1, 0]))                                       # what the MEASURED torque's would be
    chk(f"ff_hf = the high part of the added torque ({want_hf:.4f}: the 40 Hz ripple + the slow part's leak), "
        f"clearly not the measured torque's ({other_hf:.4f})",
        abs(m["ff_hf"] - want_hf) < 0.1 * want_hf and abs(other_hf - want_hf) > 0.3 * want_hf, (m["ff_hf"], want_hf))
    chk("pid = the drive's extra 0.05 Nm (+ the buzz)", abs(m["pid"] - math.sqrt(0.05 ** 2 + 0.02 ** 2 / 2)) < 0.004,
        m["pid"])
    k10 = math.degrees(0.1) / math.sqrt(2) * (1 + abs(math.sin(math.pi * 10 * (2 * HF_W + 1) * dt))
                                                / (math.pi * 10 * (2 * HF_W + 1) * dt))
    chk(f"meas_vhf = the measured speed's 10 Hz wobble (~{k10:.2f} deg/s); cmd_vhf of the smooth command = only the "
        f"filter's leak of the 63 deg/s sine (< 0.7 deg/s — the floor of this measure)",
        abs(m["meas_vhf"] - k10) < 0.15 * k10 and m["cmd_vhf"] < 0.7, (m["meas_vhf"], m["cmd_vhf"], k10))
    chk("set labels: off, stream, off2", label_sets(["off", "stream", "off"]) == ["off", "stream", "off2"])
    sp = np.abs(np.sin(np.linspace(0, 4 * np.pi, 1000)))
    mk = steady_mask(sp)
    chk("steady mask: away from the reversals by the filter's width, and most of the motion kept",
        mk.size == 1000 - 2 * HF_W and not mk[np.argmin(sp[HF_W:-HF_W] + 0)] and 0.5 < mk.mean() < 0.8, mk.mean())
    print("selftest:", "ALL PASSED" if ok_all else "FAILED")
    return 0 if ok_all else 1


# ----------------------------------------------------------------------------- robot session
def stream_block(rti, q0, joints0, amp, period, lo, hi, seconds, rng):
    """Still, the faded sine, still — sent at lo..hi Hz (each interval drawn from [1/hi, 1/lo])."""
    total = STILL_S + seconds + STILL_S
    t0 = time.monotonic()
    nxt = t0
    while True:
        now = time.monotonic()
        t = now - t0
        if t >= total:
            break
        q = list(q0)
        off = offset(t - STILL_S, seconds, amp, period)
        for j in joints0:
            q[j] = q0[j] + off
        rti.send_joint_position(q)
        nxt += 1.0 / rng.uniform(lo, hi)
        slp = nxt - time.monotonic()
        if slp > 0:
            time.sleep(slp)


def claim_control(robot, seconds=15.0):
    """Take control WITHOUT forcing; wait for the dashboard's 5 s release if it holds it (as api_tests/66)."""
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


def run_session(args, cfg, senders, set_order):
    from robotclient import RobotClient, RTIClient, ControlLost
    from api_tests import _servo_common as sc
    joints1 = [int(x) for x in args.joints.split(",") if x.strip()]
    joints0 = [j - 1 for j in joints1]
    amp = math.radians(args.amp)
    lo_lim = np.degrees(np.asarray(cfg["software_joint_limit_negative"], float))
    hi_lim = np.degrees(np.asarray(cfg["software_joint_limit_positive"], float))
    labels = label_sets(set_order)
    robot = RobotClient()
    robot.connect(args.host)
    capture, records, ok, prev_recipe, comp0, claimed = None, [], True, None, None, False
    try:
        why = sc.preflight(robot)
        if why:
            print(f"Not ready: {why}. Nothing moved.")
            return 2, None
        st = robot.status()
        comp0 = (bool(st.get("inertia_comp", True)), bool(st.get("coriolis_comp", True)))
        if st.get("mode") not in ("IDLE", "EXEC"):
            print(f"The robot must be in IDLE or EXEC (it is in {st.get('mode')}). Nothing moved.")
            return 2, None
        q_now = np.asarray(robot.get_joints(), float)
        for j in joints0:
            room = min(q_now[j] - (lo_lim[j] + CLEARANCE_DEG), (hi_lim[j] - CLEARANCE_DEG) - q_now[j])
            if room < args.amp:
                print(f"J{j + 1} at {q_now[j]:.1f} deg has only {room:.1f} deg of room for +/-{args.amp:g} deg "
                      f"(soft limits {lo_lim[j]:.1f} .. {hi_lim[j]:.1f}, {CLEARANCE_DEG:g} deg clearance). Nothing moved.")
                return 2, None
        claim_control(robot)
        claimed = True
        if st.get("mode") == "EXEC":
            print("Leaving EXEC for IDLE (RTI sessions start from IDLE).")
            robot.enter_idle_mode()
            time.sleep(0.5)
        capture = Capture()
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        time.sleep(0.5)
        if capture.last_tick() is None:
            print("No telemetry with the tracking fields is arriving on UDP 30003. Nothing moved.")
            return 2, None
        robot.set_inertia_comp(True)
        robot.set_coriolis_comp(True)
        for snd in senders:
            lo, hi, soft = SENDERS[snd]
            for set_name, label in zip(set_order, labels):
                print(f"  {snd:>8s} {label:>7s} ... ", end="", flush=True)
                rng = random.Random(args.seed)
                rti = RTIClient(robot, clock="pc")
                want = "SOFT_RTI" if soft else "RTI"
                try:
                    rti.connect(rate_hz=lo if soft else None)
                    try:
                        robot.wait(lambda st_: st_.get("mode") == want, 3.0, f"mode {want}")
                    except Exception:
                        raise RuntimeError(f"the robot is in {robot.status().get('mode')}, not {want} — refusing to "
                                           f"run this block under the wrong contract") from None
                    time.sleep(0.4)
                    sc.apply_config(robot, {"gen": "ruckig_advancing" if soft else "ruckig"})
                    robot.rti_set_dynamics_ff(set_name)          # after the connect: every RTI entry starts 'off'
                    rti.read_once(timeout=2.0)
                    rti.claim()
                    q0 = list(rti.read_once(timeout=1.0)["j_p"])
                    t0 = capture.last_tick()
                    stream_block(rti, q0, joints0, amp, args.period, lo, hi, args.seconds, rng)
                    t1 = capture.last_tick()
                    sc.keep_alive(rti, q0, lo, 0.3)
                    rti.release()
                    time.sleep(0.6)                           # the robot holds 0.5 s after a release
                    robot.rti_disconnect(wait=True)           # back to IDLE: the next block may be the other contract
                except (ControlLost, Exception) as exc:      # any failure ends the run; what was recorded is saved
                    print(f"FAILED: {type(exc).__name__}: {exc}")
                    ok = False
                    try:
                        robot.rti_disconnect()
                    except Exception:
                        pass
                    break
                finally:
                    try:
                        rti.close()
                    except Exception:
                        pass
                records.append({"sender": snd, "set": label, "joints": joints1, "t0": t0, "t1": t1})
                print("done")
                time.sleep(0.3)
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
        try:
            robot.rti_set_dynamics_ff("off")
            sc.clear_config(robot)
            robot.enter_idle_mode()
        except Exception as exc:
            print(f"(cleanup: {exc})")
        if comp0 is not None:
            try:
                robot.set_inertia_comp(comp0[0])
                robot.set_coriolis_comp(comp0[1])
            except Exception as exc:
                print(f"!!! COULD NOT RESTORE THE FEEDFORWARD SWITCHES ({exc}) — check the dashboard's buttons.")
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
    return (0 if ok else 1), (capture.packets, records, labels)


def analyse_saved(json_path):
    run = json.load(open(json_path))
    stem = json_path[:-5]
    if os.path.exists(stem + ".npz"):
        raw = np.load(stem + ".npz")
        d = {k: raw[k] for k in raw.files}
    else:
        with open(stem + ".raw", "rb") as fh:
            d = to_arrays(fh.read().split(b"\n"))
        np.savez_compressed(stem + ".npz", **d)
    moves = analyse(d, run["records"], float(run.get("delta_t", 0.004)))
    senders = list(dict.fromkeys(r["sender"] for r in run["records"]))
    print_report(moves, senders, run["set_labels"], f"=== {json_path} ===")
    run.update({"moves": moves, "analysed": True})
    with open(stem + "_reanalysed.json", "w") as fh:
        json.dump(run, fh, indent=1)
    print(f"\nSaved {stem}_reanalysed.json")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true", help="actually move the arm")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--senders", default=",".join(SENDERS), help=f"in order, of {list(SENDERS)}")
    ap.add_argument("--sets", default="off,stream,off", help=f"in order, of {list(SETS)}")
    ap.add_argument("--joints", default="1", help=f"1-based, of {SAFE_JOINTS1}")
    ap.add_argument("--amp", type=float, default=15.0, help="deg, around where the arm is (at most 20)")
    ap.add_argument("--period", type=float, default=1.5, help="s (at least 1.0)")
    ap.add_argument("--seconds", type=float, default=8.0, help="of sine per block, faded in / out over 1 s each")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--reanalyse", metavar="RUN.json")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.reanalyse:
        if robot_enabled(args.host, args.port):
            print("The arm is ENABLED: the analysis is heavy for the Pi and would slow RTI's loop. Disable it first.")
            return 2
        return analyse_saved(args.reanalyse)
    senders = [s.strip() for s in args.senders.split(",") if s.strip()]
    set_order = [s.strip() for s in args.sets.split(",") if s.strip()]
    joints1 = [int(x) for x in args.joints.split(",") if x.strip()]
    bad = [s for s in senders if s not in SENDERS] + [s for s in set_order if s not in SETS] \
        + [f"J{j}" for j in joints1 if j not in SAFE_JOINTS1]
    if bad or args.amp > 20.0 or args.period < 1.0 or args.seconds < 2.5 * 1.0:
        print(f"Refused: unknown / not allowed {bad}, or --amp > 20, --period < 1, --seconds < 2.5. Nothing moved.")
        return 2
    w = 2 * math.pi / args.period
    print(f"Plan: J{','.join(map(str, joints1))} sine +/-{args.amp:g} deg, period {args.period:g} s "
          f"(peak {math.degrees(math.radians(args.amp) * w):.0f} deg/s, {math.radians(args.amp) * w * w:.1f} rad/s^2), "
          f"{args.seconds:g} s per block + {2 * STILL_S:g} s still")
    print(f"  senders {senders}, sets in order {label_sets(set_order)}: {len(senders) * len(set_order)} blocks, "
          f"~{len(senders) * len(set_order) * (args.seconds + 2 * STILL_S + 1.5) / 60:.1f} min")
    if not args.go:
        print("DRY RUN: re-run with --go to move the arm. Nothing moved.")
        return 0
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    rc, got = run_session(args, cfg, senders, set_order)
    if got is None:
        return rc
    packets, records, labels = got
    out_dir = os.path.join(_ROOT, "temp_data", "rti_ff")
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, time.strftime("%Y%m%d_%H%M%S"))
    with open(base + ".raw", "wb") as fh:
        fh.write(b"\n".join(packets))
    with open(base + ".json", "w") as fh:
        json.dump({"set_labels": labels, "records": records, "args": vars(args),
                   "delta_t": float(cfg.get("delta_t", 0.004)), "analysed": False}, fh, indent=1)
    rel = os.path.relpath(base, _ROOT)
    print(f"\nRaw recording saved: {rel}.raw + .json ({len(packets)} packets).")
    if robot_enabled(args.host, args.port) is not False:
        print("Not analysed yet (heavy for the Pi while the arm is enabled). Disable the arm, then:\n"
              f"  cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/68_rti_feedforward.py --reanalyse {rel}.json")
        return rc
    return analyse_saved(base + ".json") or rc


if __name__ == "__main__":
    sys.exit(main())
