#!/usr/bin/env python3
"""54: how hard ONE joint has to be pushed before it breaks loose — its static friction — and how wrong the
gravity model is for it. Only the tested joint gets a push (any joint; default J5). J2/J3: the servo-test rule kept
them out until the user asked for them (2026-10-02, friction compensation for the big joints).

Why (2026-10-02): in freedrive the walls push exactly what they are set to (measured == commanded), yet a joint
released inside a wall often stays there — J5 sat still under 0.53 Nm, J6 stopped 3.8 deg inside its wall at
0.05 Nm, both stuck and jumped (stick-slip). A wall can only move the arm back once its push beats the joint's
breakaway friction. This measures that number, per joint and pose, and separates it from a gravity-model error
(which looks the same from one side: the joint does not move, or moves on its own).

How: the robot is put in hard RTI torque mode for a few seconds at a time. Every joint gets ZERO extra torque —
the robot adds gravity, so all of them float exactly as in freedrive — except the tested joint, whose extra
torque ramps slowly (default 0.1 Nm/s) until it moves 0.08 deg. Then every joint gets a velocity-ZERO command
(the drives hold still, as IDLE does with gravity comp off) for 0.4 s, and the robot goes back to IDLE. Each ramp
is its own short RTI session. It ramps + and - at each angle, several times:

    breaks loose at t+ (pushing +) and t- (pushing -):   t+ = Fs - e,   t- = -Fs - e
    static friction Fs = (t+ - t-) / 2;   gravity-model error e = -(t+ + t-) / 2
    (e > 0: the robot's gravity comp pushes the joint + by e more than gravity needs; in freedrive it drifts +
     whenever e > Fs)

Then SLIDING friction (2026-10-02 — what a MOVING joint has, which is what freedrive friction compensation may give
back): the drive's own speed loop moves the joint back and forth, ±4 deg at 5 deg/s and ±8 deg at 15 deg/s (every
other joint held at speed 0); RTI adds the gravity model, so the drive's own correction (measured - commanded torque)
is friction + the model's error, at a steady speed: + F - e, - -F - e. Two speeds split it into a constant (Coulomb)
part and a part growing with speed (viscous). A joint that sticks and jumps on the way gets NO number.

Moves: the tested joint only — by EXEC to each test angle (10 % speed), by under 1 deg at each breakaway, and by
±8 deg (+3 deg at most) around each test angle while sliding.
Stops: a tested joint that moves more than 5 deg, or ANY other joint more than 2 deg, ends the test (brake).
The extra torque never exceeds --max (default 1 Nm, at most 80 % of what the drive can give). Every test angle
stays 15 deg from the soft limits. Robot ENABLED, homed, in IDLE; nothing else may hold control.

    .venv/bin/python api_tests/54_joint_friction.py --selftest          # the ramp logic on a simulated sticky joint
    .venv/bin/python api_tests/54_joint_friction.py                     # dry run: the plan, nothing moves
    .venv/bin/python api_tests/54_joint_friction.py --go                # J5 at its angle now and +/-30 deg
    .venv/bin/python api_tests/54_joint_friction.py --joint 6 --angles -20 0 20 --go
    .venv/bin/python api_tests/54_joint_friction.py --joint 6 --slide-only --go   # sliding only (no ramps)

Results print per angle and go to temp_data/friction/<time>_J<n>.json (every sample of every ramp).
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
ALLOWED = (1, 2, 3, 4, 5, 6)        # J2/J3 too since 2026-10-02 (user request); otherwise they float as in freedrive
OFFSET_DEG = {2: 15.0, 3: 15.0}      # default test angles: now and now +/- this (30 elsewhere) — the shoulder/elbow
                                     # move the whole arm, so smaller steps by default
ONSET_DEG = 0.08                    # moved this far IN THE PUSH'S DIRECTION = it broke loose (from the furthest point
                                    # the other way: on the arm a joint often first unwinds AGAINST the push, 2026-10-02)
RUNAWAY_DEG = 5.0                   # the tested joint further than this = something is wrong: stop
OTHER_DEG = 2.0                     # any other joint further than this = stop
ROOM_DEG = 15.0                     # every test angle this far from the soft limits
BRAKE_S = 0.4
RECIPE = "haptic_wall"              # TCP_p, TCP_s, j_p, j_s, j_t, j_tc, tick, mode


def split(t_plus, t_minus):
    """Breakaway torques pushing + and - (Nm, extra on top of the model's gravity) -> (static friction Fs,
    gravity-model error e). The joint breaks loose when extra + e = +Fs or -Fs."""
    return (t_plus - t_minus) / 2.0, -(t_plus + t_minus) / 2.0


def run_ramp(j, direction, ramp, t_max, send_torque, send_brake, read_q, pace, n=6):
    """Ramp joint j's extra torque (Nm) in `direction` at `ramp` Nm/s until it moves ONSET_DEG, then brake.

    send_torque(list) / send_brake() send one command; read_q() returns the newest measured joint vector (rad)
    or None; pace() waits for the next tick. The ramp is by TICK COUNT, so it is the same whatever the wall clock
    does. Returns (breakaway torque or None, why, samples [(t, extra, q_j)]). The torque is the one commanded on the
    tick the motion was SEEN — a few ticks after it started, i.e. at most ~ramp x 5 / FS (0.002 Nm at 0.1 Nm/s)
    high."""
    tau = [0.0] * n
    q0, last, log, k, back = None, None, [], 0, None
    t_end = t_max / ramp + 1.0                     # hold at t_max for 1 s before giving up
    why, found = "no breakaway up to the maximum", None
    while True:
        x = min(ramp * k / FS, t_max)
        tau[j] = direction * x
        send_torque(tau)
        q = read_q()
        if q is not None:
            last = np.asarray(q, dtype=float)
            if q0 is None:
                q0 = last.copy()
                back = float(q0[j])
            log.append((k / FS, x, float(last[j])))
            if direction * (last[j] - back) < 0.0:
                back = float(last[j])              # unwinding against the push (after a move): not a breakaway
            moved = direction * (last[j] - back)
            others = max((abs(last[i] - q0[i]) for i in range(n) if i != j), default=0.0)
            if others > math.radians(OTHER_DEG):
                why = f"another joint moved {math.degrees(others):.1f} deg — stopped"
                break
            if moved > math.radians(ONSET_DEG):
                found, why = x, "broke loose"
                break
        if k / FS > t_end:
            break
        k += 1
        pace()
    for _ in range(int(BRAKE_S * FS)):
        send_brake()
        pace()
        k += 1
        q = read_q()
        if q is not None:
            last = np.asarray(q, dtype=float)
            log.append((k / FS, 0.0, float(last[j])))
    if q0 is not None and last is not None and abs(last[j] - q0[j]) > math.radians(RUNAWAY_DEG):
        found, why = None, f"moved {math.degrees(abs(last[j] - q0[j])):.1f} deg — more than {RUNAWAY_DEG:g}: stopped"
    return found, why, log


SLIDES = ((5.0, 4.0), (15.0, 8.0))     # (deg/s, ±deg): the sliding measurement's two speeds
SLIDE_LEGS = 6                         # back-and-forth legs per speed (3 each way)
SLIDE_SKIP_S = 0.35                    # after each turn: not measured (still getting up to speed)
SLIDE_RAMP_S = 0.1                     # the speed command ramps to its new value over this (gentle turns)
SLIDE_WIN = 10                         # stick-slip check: per 40 ms, it must advance at least 25 % of the speed asked
SLIDE_STEADY = 0.9                     # ... in at least this share of a direction's 40 ms windows
SLIDE_PAST_DEG = 3.0                   # further than this past its back-and-forth: stop


def run_slide(j, v_dps, amp_deg, send_velocity, read_state, pace, n=6, legs=SLIDE_LEGS):
    """SLIDING friction, the way robot friction is usually identified: the DRIVE's own velocity loop (fast, on its
    encoder — no stick-slip from our 250 Hz loop) moves the joint back and forth at a constant v_dps between
    ±amp_deg; every other joint is held at speed 0. RTI adds the gravity model to the command, so the drive's own
    correction — the measured torque minus what was commanded (j_t - j_tc) — is what friction and the gravity
    model's error need: + at a steady speed: F - e, -: -F - e (e as in split()). Averaged over EVERY tick of a leg
    after its first 0.35 s (the motor's ripple averages out over whole cycles; at 5 deg/s it swings the measured
    speed far outside any band, so judging ticks by their speed threw most of them away), and only believed when
    the joint really slid: in 90 % of its 40 ms windows it advanced at least a quarter of
    what the speed asks (a joint that sticks and jumps needs more than its sliding friction to break loose again).
    Ends with speed 0 commanded on every path. read_state() -> dict with "j_p", "j_s", "j_t", "j_tc" (rad, rad/s,
    Nm) or None. Returns ({+1: (mean extra torque Nm, mean speed rad/s, share steady, mean model gravity Nm),
    -1: (...)} or None, why stopped or None)."""
    vel = [0.0] * n

    def to_zero(why):
        vel[j] = 0.0
        for _ in range(int(0.3 * FS)):
            send_velocity(vel)
            pace()
        return None, why

    first = None
    while first is None:
        send_velocity(vel)
        pace()
        first = read_state()
    q0 = np.asarray(first["j_p"], dtype=float).copy()
    x0 = float(q0[j])
    v, a = math.radians(v_dps), math.radians(amp_deg)
    step = v / (SLIDE_RAMP_S * FS)
    d, since, done, cmd, k = 1.0, 0.0, 0, 0.0, 0
    acc = {1.0: [[], [], 0, 0, []], -1.0: [[], [], 0, 0, []]}  # extra torques, speeds, 40 ms windows, stuck, gravity
    xs = []
    k_max = int(3.0 * (0.5 + legs) * 2.0 * a / v * FS) + int(2 * FS)
    while done < legs:
        k += 1
        if k > k_max:
            return to_zero(f"J{j + 1} did not get through {legs} legs in {k_max / FS:.0f} s (stuck?) — stopped")
        s = read_state()
        if s is not None:
            q = np.asarray(s["j_p"], dtype=float)
            x = float(q[j])
            if abs(x - x0) > a + math.radians(SLIDE_PAST_DEG):
                return to_zero(f"J{j + 1} {math.degrees(x - x0):+.1f} deg from its start — past its back-and-forth: "
                               f"stopped")
            others = max((abs(q[i] - q0[i]) for i in range(n) if i != j), default=0.0)
            if others > math.radians(OTHER_DEG):
                return to_zero(f"another joint moved {math.degrees(others):.1f} deg — stopped")
            if (d > 0 and x >= x0 + a) or (d < 0 and x <= x0 - a):
                d, since, xs = -d, 0.0, []
                done += 1
            elif since > SLIDE_SKIP_S:
                r = acc[d]
                r[0].append(float(s["j_t"][j]) - float(s["j_tc"][j]))
                r[1].append(float(s["j_s"][j]))
                r[4].append(float(s["j_tc"][j]))
                xs.append(x)
                if len(xs) == SLIDE_WIN:
                    r[2] += 1
                    if d * (xs[-1] - xs[0]) < 0.25 * v * (SLIDE_WIN - 1) / FS:
                        r[3] += 1
                    xs = [x]
        since += 1.0 / FS
        cmd = min(cmd + step, d * v) if d > 0 else max(cmd - step, d * v)
        vel[j] = cmd
        send_velocity(vel)
        pace()
    to_zero(None)
    res = {}
    for dd, (tq, sp, wins, stuck, gm) in acc.items():
        share = 1.0 - stuck / wins if wins else 0.0
        if wins < 10 or share < SLIDE_STEADY:
            return None, (f"J{j + 1} did not slide steadily {'+' if dd > 0 else '-'} at {v_dps:g} deg/s: it advanced "
                          f"in only {100 * share:.0f} % of its 40 ms windows (stick-slip) — no number")
        res[dd] = (float(np.mean(tq)), float(np.mean(sp)), share, float(np.mean(gm)))
    return res, None


def xml_values(angles):
    """The robot XML's values from one run's per-angle records: (lowest sliding friction, largest gravity error,
    largest model gravity measured under) or None. The LOWEST friction a moving joint showed — at any angle and either
    speed, and the Coulomb part extrapolated to zero speed — and the LARGEST single gravity error. (J3, 2026-10-02:
    less friction at 15 than at 5 deg/s, and its gravity error +0.124 at one speed vs +0.003 at the other: the
    5 deg/s value and a per-angle average would have given 0.186 Nm of compensation instead of 0.122.)"""
    slides = [x for r in angles for x in r.get("slides", []) if "friction" in x]
    if not slides:
        return None
    fs_all = [x["friction"] for x in slides] + [r["sliding"] for r in angles if r.get("sliding") is not None]
    return (float(min(fs_all)), float(max(abs(x["gravity_error"]) for x in slides)),
            float(max(abs(x.get("model_gravity", 0.0)) for x in slides)))


def coulomb_viscous(slides):
    """[(v_dps, F_slide)] at two speeds -> (Coulomb Nm, viscous Nm.s/rad): F = Fc + b x v."""
    (v1, f1), (v2, f2) = slides
    b = (f2 - f1) / (math.radians(v2) - math.radians(v1))
    b = max(0.0, b)
    return f1 - b * math.radians(v1), b


def selftest():
    """The ramp logic against a simulated joint with static friction Fs, sliding friction 0.6 Fs and a gravity
    error e, the state one tick old and the command applied one tick late — as on the arm."""
    bad = []

    def chk(name, cond, detail=""):
        print(("  PASS  " if cond else "  FAIL  ") + name + (f"   [{detail}]" if detail else ""))
        if not cond:
            bad.append(name)

    fs_, e_ = split(0.5, -0.3)
    chk("split: t+ 0.5, t- -0.3 -> Fs 0.4, e -0.1", abs(fs_ - 0.4) < 1e-12 and abs(e_ + 0.1) < 1e-12)

    def plant(fs, e, inertia=0.003, j=4):
        st = {"q": np.zeros(6), "v": 0.0, "stuck": True, "cmd": None, "pending": None, "hist": [np.zeros(6)],
              "brakes": 0, "tail": []}

        def send_torque(tau):
            st["pending"] = ("t", tau[j])
            st["tail"].append("t")

        def send_brake():
            st["pending"] = ("b", 0.0)
            st["brakes"] += 1
            st["tail"].append("b")

        def pace():
            cmd, st["cmd"] = st["cmd"], st["pending"]          # applied one tick late
            if cmd is not None:
                kind, x = cmd
                if kind == "b":
                    st["v"], st["stuck"] = 0.0, True
                else:
                    net = x + e
                    if st["stuck"]:
                        if abs(net) > fs:
                            st["stuck"] = False
                    if not st["stuck"]:
                        drag = 0.6 * fs * (math.copysign(1.0, st["v"]) if st["v"] else math.copysign(1.0, net))
                        st["v"] += (net - drag) / inertia / FS
                        st["q"][j] += st["v"] / FS
            st["hist"].append(st["q"].copy())

        def read_q():
            return st["hist"][-2] if len(st["hist"]) > 1 else st["hist"][-1]   # one tick old

        return send_torque, send_brake, read_q, pace, st

    for fs, e in ((0.45, 0.10), (0.30, -0.12), (0.05, 0.0), (0.5, 0.42)):     # last: e almost Fs
        res = []
        for d in (1.0, -1.0):
            tq, br, rq, pc, st = plant(fs, e)
            t, why, log = run_ramp(4, d, 0.1, 1.0, tq, br, rq, pc)
            res.append((t, why, abs(st["q"][4])))
            nb = int(BRAKE_S * FS)
            if not (st["brakes"] == nb and st["tail"][-nb:] == ["b"] * nb and st["v"] == 0.0):
                chk(f"Fs {fs}, e {e:+}, push {d:+}: braked for {BRAKE_S} s right after it broke loose, and stopped",
                    False, f"{st['brakes']} brake commands, last {st['tail'][-3:]}, v {st['v']:.3f}")
        if res[0][0] is None or res[1][0] is None:
            chk(f"Fs {fs}, e {e:+}: both directions broke loose", False, str(res))
            continue
        fs_m, e_m = split(res[0][0], -res[1][0])
        chk(f"Fs {fs}, e {e:+}: measured Fs {fs_m:.4f}, e {e_m:+.4f} (within 0.005 Nm), moved under 1 deg",
            abs(fs_m - fs) < 0.005 and abs(e_m - e) < 0.005 and max(r[2] for r in res) < math.radians(1.0),
            f"moved {math.degrees(max(r[2] for r in res)):.2f} deg")
    tq, br, rq, pc, st = plant(0.6, 0.55)
    t, why, log = run_ramp(4, -1.0, 0.1, 1.0, tq, br, rq, pc)
    chk("a direction that needs more than the maximum (Fs 0.6 + e 0.55 = 1.15 > 1 Nm) is REPORTED, not guessed",
        t is None and "no breakaway" in why and abs(st["q"][4]) < 1e-12, why)
    tq, br, rq, pc, st = plant(5.0, 0.0)
    t, why, log = run_ramp(4, 1.0, 0.1, 1.0, tq, br, rq, pc)
    chk("friction above the maximum: no breakaway reported, gave up after holding the maximum 1 s, NEVER pushed "
        "more than the maximum", t is None and "no breakaway" in why and abs(log[-1][0] - (1.0 / 0.1 + 1.0 + BRAKE_S))
        < 0.05 and max(x for _, x, _ in log) <= 1.0 + 1e-12, f"{why}; largest push {max(x for _, x, _ in log):.3f} Nm")
    tq, br, rq, pc, st = plant(0.3, 0.0)

    def drifting():
        q = rq()
        q = q.copy()
        q[1] += math.radians(3.0) * min(1.0, len(st["hist"]) / 200.0)   # J2 drifts 3 deg
        return q
    t, why, log = run_ramp(4, 1.0, 0.1, 1.0, tq, br, drifting, pc)
    chk("another joint moving more than 2 deg stops the ramp (and brakes)", t is None and "another joint" in why, why)
    for d in (1.0, -1.0):
        tq, br, rq, pc, st = plant(0.3, 0.05)

        def unwinding():
            q = rq().copy()
            q[4] -= d * math.radians(0.1) * min(1.0, len(st["hist"]) / 15.0)   # gearing unwinds 0.1 deg AGAINST it
            return q
        t, why, log = run_ramp(4, d, 0.1, 1.0, tq, br, unwinding, pc)
        want = 0.3 - d * 0.05
        chk(f"a joint that first unwinds 0.1 deg AGAINST the push ({'+' if d > 0 else '-'}; seen on the arm after a "
            f"move, read as 'broke loose at 0.003 Nm'): breaks loose at {want:.2f} within 0.005",
            t is not None and abs(t - want) < 0.005, f"{t}")
    # The sliding measurement, against a simulated DRIVE: its own PI velocity loop at 5 kHz on the true speed, the
    # speed command applied one tick late, the state one tick old; the joint sticks below its breakaway friction
    # and slides at `sliding` (+ viscous x speed), re-sticking whenever it stops; the measured torque and speed
    # carry a motor ripple. Every joint but the tested one is held.
    def vplant(fs, sliding, e, inertia=0.002, visc=0.0, bw_hz=30.0, ki_hz=3.0, ripple=0.0, j=4, drift=False):
        kp = inertia * 2.0 * math.pi * bw_hz
        ki = kp * 2.0 * math.pi * ki_hz
        st = {"q": 0.0, "v": 0.0, "i": 0.0, "stuck": True, "cmd": 0.0, "pending": 0.0, "hist": [], "last_cmd": 0.0}
        g_model, sub = 0.3, 20
        dt = 1.0 / FS / sub

        def send_velocity(vel):
            st["pending"] = float(vel[j])
            st["last_cmd"] = float(vel[j])

        def pace():
            vc, st["cmd"] = st["cmd"], st["pending"]
            u_sum = 0.0
            for _ in range(sub):
                err = vc - st["v"]
                st["i"] = max(-1.5, min(1.5, st["i"] + ki * err * dt))
                u = max(-1.5, min(1.5, kp * err + st["i"]))
                u_sum += u
                net = u + e
                if st["stuck"]:
                    if abs(net) > fs:
                        st["stuck"] = False
                    else:
                        continue
                sgn = math.copysign(1.0, st["v"]) if st["v"] else math.copysign(1.0, net)
                v_new = st["v"] + (net - sliding * sgn - visc * st["v"]) / inertia * dt
                if st["v"] and v_new * st["v"] <= 0.0 and abs(net) <= fs:
                    st["v"], st["stuck"] = 0.0, True              # it stopped and the push cannot break it loose
                else:
                    st["v"] = v_new
                st["q"] += st["v"] * dt
            q = np.zeros(6)
            q[j] = st["q"]
            if drift:
                q[1] = math.radians(3.0) * min(1.0, len(st["hist"]) / 300.0)
            rip = ripple * math.sin(900.0 * st["q"])
            jt, js, jtc = np.zeros(6), np.zeros(6), np.zeros(6)
            jtc[j] = g_model
            jt[j] = g_model + u_sum / sub + rip
            js[j] = st["v"] + 20.0 * rip
            st["hist"].append({"j_p": q, "j_s": js, "j_t": jt, "j_tc": jtc})

        def read_state():
            return st["hist"][-2] if len(st["hist"]) > 1 else None

        return send_velocity, read_state, pace, st

    cases = (  # breakaway, sliding, gravity error, inertia, viscous, ripple, drive loop Hz, note
        (0.09, 0.054, 0.03, 0.002, 0.0, 0.0, 30.0, "J5-like"),
        (0.15, 0.09, -0.04, 0.0008, 0.0, 0.02, 100.0, "J6-like, a 100 Hz drive loop, 0.02 Nm motor ripple"),
        (0.19, 0.12, 0.13, 0.01, 0.0, 0.02, 30.0, "J4-like, gravity error as large as the friction, ripple"),
        (0.12, 0.07, 0.02, 0.002, 0.05, 0.0, 30.0, "with viscous friction 0.05 Nm.s/rad"),
    )
    for fs, sl, e, inertia, visc, rip, bw, note in cases:
        slides = []
        for v_dps, amp in SLIDES:
            sv, rs, pc, st = vplant(fs, sl, e, inertia=inertia, visc=visc, ripple=rip, bw_hz=bw)
            res, why = run_slide(4, v_dps, amp, sv, rs, pc)
            if res is None:
                chk(f"{note}: sliding at {v_dps:g} deg/s measured", False, why)
                break
            f_sl, e_sl = split(res[1.0][0], res[-1.0][0])
            v_meas = 0.5 * (res[1.0][1] - res[-1.0][1])
            want = sl + visc * v_meas
            slides.append((math.degrees(v_meas), f_sl))
            chk(f"{note}: at {v_dps:g} deg/s sliding friction {f_sl:.4f} (true {want:.4f}) within 5 %, gravity "
                f"error {e_sl:+.4f} (true {e:+}) within 0.003 Nm, ended with speed 0 commanded",
                abs(f_sl - want) < 0.05 * want and abs(e_sl - e) < 0.003 and st["last_cmd"] == 0.0,
                f"speed {math.degrees(v_meas):.2f} deg/s, steady {100 * min(res[1.0][2], res[-1.0][2]):.0f} %")
        if len(slides) == 2:
            fc, bv = coulomb_viscous(slides)
            chk(f"{note}: the two speeds split it into Coulomb {sl} (within 5 %) and viscous {visc} (within 0.01)",
                abs(fc - sl) < 0.05 * sl and abs(bv - visc) < 0.01, f"Coulomb {fc:.4f}, viscous {bv:.4f}")
    sv, rs, pc, st = vplant(0.09, 0.054, 0.03)
    res, why = run_slide(4, 5.0, 4.0, sv, rs, pc)
    chk("it reports the model gravity it measured under (0.3 Nm in the plant: a gravity error found where the model "
        "carries little gravity says little about loaded poses)",
        res is not None and abs(res[1.0][3] - 0.3) < 1e-9 and abs(res[-1.0][3] - 0.3) < 1e-9, why)
    j3 = [{"sliding": 0.435, "slides": [{"friction": 0.435, "gravity_error": 0.061, "model_gravity": 2.43},
                                        {"friction": 0.388, "gravity_error": -0.016, "model_gravity": 2.43}]},
          {"sliding": 0.467, "slides": [{"friction": 0.467, "gravity_error": 0.124, "model_gravity": 2.48},
                                        {"friction": 0.369, "gravity_error": 0.003, "model_gravity": 2.48}]},
          {"sliding": 0.437, "slides": [{"friction": 0.437, "gravity_error": -0.010, "model_gravity": 2.22},
                                        {"v_dps": 15.0, "why": "did not slide steadily"}]}]
    v = xml_values(j3)
    chk("the XML values from J3's real run: friction 0.369 (the LOWEST, at 15 deg/s — not the 5 deg/s 0.435), gravity "
        "error 0.124 (the largest single one, not an average), measured under 2.48 Nm; a refused slide is skipped",
        v is not None and abs(v[0] - 0.369) < 1e-12 and abs(v[1] - 0.124) < 1e-12 and abs(v[2] - 2.48) < 1e-12, v)
    chk("...and with no slide measured: no values", xml_values([{"slides": [{"why": "stick-slip"}]}]) is None)
    # The same light joint (friction 0.15 -> 0.09 once moving) under a 30 Hz drive loop really stick-slips (40 % of its ticks stopped): the average torque would mix in the breakaway — no number.
    sv, rs, pc, st = vplant(0.15, 0.09, -0.04, inertia=0.0008)
    stopped = []
    pc_ = pc

    def pc_count():
        pc_()
        stopped.append(st["v"] == 0.0)
    res, why = run_slide(4, 5.0, 4.0, sv, rs, pc_count)
    chk("a joint that really stick-slips (light, 0.15 breakaway / 0.09 sliding): NO number, said why",
        res is None and "stick-slip" in why and np.mean(stopped) > 0.2,
        f"{why}; stopped {100 * np.mean(stopped):.0f} % of the ticks")
    # A weak drive loop on a strongly sticking joint. Never a wrong number: right or refused.
    for bw, ki_hz in ((3.0, 0.0), (1.5, 0.0), (4.0, 0.5), (8.0, 1.0)):
        sv, rs, pc, st = vplant(0.2, 0.06, 0.0, inertia=0.002, bw_hz=bw, ki_hz=ki_hz)
        res, why = run_slide(4, 5.0, 4.0, sv, rs, pc)
        f_sl = None if res is None else split(res[1.0][0], res[-1.0][0])[0]
        chk(f"a weak drive loop ({bw:g} Hz, integral {ki_hz:g} Hz) on a sticking joint (0.2 breakaway / 0.06 sliding): the "
            f"sliding friction within 10 % or NO number", res is None or abs(f_sl - 0.06) < 0.006,
            why if res is None else f"{f_sl:.4f}")
    sv, rs, pc, st = vplant(5.0, 3.0, 0.0)
    res, why = run_slide(4, 5.0, 4.0, sv, rs, pc)
    chk("a joint the drive cannot move: stopped with a reason, speed 0 commanded at the end",
        res is None and "stuck" in why and st["last_cmd"] == 0.0, why)
    sv, rs, pc, st = vplant(0.09, 0.054, 0.0, drift=True)
    res, why = run_slide(4, 5.0, 4.0, sv, rs, pc)
    chk("another joint moving more than 2 deg stops it", res is None and "another joint" in why, why)
    print(f"\n{len(bad)} failed" + (f": {bad}" if bad else " - all passed"))
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--joint", type=int, default=5, help="1-based, 1-6 (default 5)")
    ap.add_argument("--angles", type=float, nargs="*", help="test angles in deg (default: now and now +/-30; "
                                                              "J2/J3 +/-15)")
    ap.add_argument("--repeats", type=int, default=3, help="ramps per direction per angle (default 3)")
    ap.add_argument("--ramp", type=float, default=0.1, help="Nm/s (default 0.1)")
    ap.add_argument("--max", type=float, default=None, help="Nm, the largest push (default 1, <= 80%% of the drive)")
    ap.add_argument("--slide-only", action="store_true", help="only the sliding measurement (no breakaway ramps)")
    ap.add_argument("--no-slide", action="store_true", help="only the breakaway ramps")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.joint not in ALLOWED:
        print(f"--joint must be one of {ALLOWED}.")
        return 2
    j = args.joint - 1

    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML, TEMP_DATA_DIR
    from robotclient import RobotClient, ControlLost
    from _rti_session import rti_session, Pacer
    from utility.haptic_walls import joint_drive_torque
    base = __import__("46_monitor_baseline")
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    lo = float(np.degrees(cfg["software_joint_limit_negative"][j]))
    hi = float(np.degrees(cfg["software_joint_limit_positive"][j]))
    drive = float(joint_drive_torque(cfg)[j])
    t_max = min(1.0 if args.max is None else args.max, 0.8 * drive)

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed, prev_recipe = False, None
    out = {"joint": args.joint, "ramp": args.ramp, "max": t_max, "angles": []}
    try:
        q_now = np.asarray(robot.get_joints(), dtype=float)
        off = OFFSET_DEG.get(args.joint, 30.0)
        angles = args.angles if args.angles else [q_now[j], q_now[j] - off, q_now[j] + off]
        ok = [a for a in angles if lo + ROOM_DEG <= a <= hi - ROOM_DEG]
        print(f"J{args.joint} now {q_now[j]:.1f} deg, soft limits [{lo:.1f}, {hi:.1f}]. Test angles "
              f"{', '.join(f'{a:.1f}' for a in ok) or 'none'}"
              + (f" (skipped, under {ROOM_DEG:g} deg from a limit: {', '.join(f'{a:.1f}' for a in angles if a not in ok)})"
                 if len(ok) < len(angles) else "")
              + ("" if args.slide_only else f". Push up to {t_max:.2f} Nm (drive {drive:.2f}) at {args.ramp:g} Nm/s, "
                                           f"{args.repeats}x each way")
              + ("" if args.no_slide else "; then slide " + ", ".join(f"±{a:g} deg at {v:g} deg/s" for v, a in SLIDES)
                 + " (its drive's speed loop)") + ".")
        if not ok:
            return 2
        if not args.go:
            print("DRY RUN: re-run with --go. Nothing moved.")
            return 0
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") != "IDLE":
            print(f"Robot must be ENABLED, homed and in IDLE (got state={st.get('state')} homed={st.get('homed')} "
                  f"mode={st.get('mode')}).")
            return 2
        base.claim_control(robot)
        claimed = True
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        robot.set_gravity_comp(True)
        walls = {w["joint"]: w for w in robot.get_joint_walls()["walls"]}

        def go_to(angle):
            target = robot.get_joints()
            if abs(target[j] - angle) < 0.5:
                return True
            target[j] = float(angle)
            robot.enter_exec_mode(wait=True)
            robot.move_joints(*[float(x) for x in target], speed=10)
            fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
            robot.enter_idle_mode(wait=True)
            time.sleep(0.5)
            if fails:
                print(f"   could not move J{args.joint} to {angle:.1f} deg: {str(fails)[:120]}")
            return not fails

        def one_ramp(direction):
            with rti_session(robot) as (rti, first):
                grav = float(first["j_tc"][j])                 # extra 0 so far: what RTI adds = the model's gravity
                pace = Pacer(FS)

                def read_q():
                    s = rti.read_state()
                    return None if s is None else s["j_p"]

                try:
                    t, why, log = run_ramp(j, direction, args.ramp, t_max, rti.send_joint_torque,
                                           lambda: rti.send_joint_velocity([0.0] * 6), read_q, pace)
                except ControlLost as e:
                    t, why, log = None, f"control lost: {e}", []
            time.sleep(0.3)
            return t, why, log, grav

        def one_slide(v_dps, amp):
            with rti_session(robot) as (rti, _first):
                pace = Pacer(FS)
                try:
                    res, why = run_slide(j, v_dps, amp, rti.send_joint_velocity, rti.read_state, pace)
                except ControlLost as e:
                    res, why = None, f"control lost: {e}"
            time.sleep(0.3)
            return res, why

        for angle in ok:
            if not go_to(angle):
                break
            q_at = float(robot.get_joints()[j])
            print(f"\nJ{args.joint} at {q_at:.1f} deg:")
            ups, downs, rec, stop = [], [], {"angle": q_at, "ramps": []}, False
            for rep in range(0 if args.slide_only else args.repeats):
                for direction in (1.0, -1.0):
                    errs = [e for e in (robot.status().get("errors") or []) if not e.endswith("_WARNING")]
                    if errs or robot.status().get("mode") != "IDLE":
                        print(f"   stopping: {errs or robot.status().get('mode')}")
                        stop = True
                        break
                    t, why, log, grav = one_ramp(direction)
                    rec["ramps"].append({"dir": direction, "breakaway": t, "why": why, "gravity_model": grav,
                                         "samples": log})
                    print(f"   push {'+' if direction > 0 else '-'}: " + (f"broke loose at {t:.3f} Nm" if t is not None
                                                                         else why) + f"   (model gravity {grav:+.3f} Nm)")
                    if t is None:
                        stop = "maximum" not in why
                        if stop:
                            break
                    else:
                        (ups if direction > 0 else downs).append(t)
                if stop:
                    break
            if ups and downs:
                tp, tm = float(np.median(ups)), -float(np.median(downs))
                fs, e = split(tp, tm)
                w = walls.get(args.joint, {})
                k_here = float(w.get("stiffness_nm_per_deg") or 0.0) * float(w.get("scale_here") or 1.0)
                rec.update(t_plus=tp, t_minus=tm, friction=fs, gravity_error=e)
                print(f"   breaks loose at +{tp:.3f} / {tm:.3f} Nm (spread {np.ptp(ups):.3f} / {np.ptp(downs):.3f})"
                      f"  ->  static friction {fs:.3f} Nm, gravity-model error {e:+.3f} Nm"
                      + (" (the model pushes + more than gravity needs)" if e > 0.02 else
                         " (the model pushes - more than gravity needs)" if e < -0.02 else " (model OK here)"))
                if k_here > 0:
                    print(f"   its freedrive wall ({k_here:.4f} Nm/deg here) beats that friction {fs / k_here:.1f} deg in")
            if not args.no_slide and not stop:
                slides, rec["slides"] = [], []
                for v_dps, amp in SLIDES:
                    errs = [e for e in (robot.status().get("errors") or []) if not e.endswith("_WARNING")]
                    if errs or robot.status().get("mode") != "IDLE":
                        print(f"   stopping: {errs or robot.status().get('mode')}")
                        stop = True
                        break
                    res, why = one_slide(v_dps, amp)
                    if res is None:
                        print(f"   slide at {v_dps:g} deg/s: {why}")
                        rec["slides"].append({"v_dps": v_dps, "amp_deg": amp, "why": why})
                        if "stick-slip" not in why:
                            stop = True
                            break
                        continue
                    (tp, vp, shp, gp), (tm, vm, shm, gmn) = res[1.0], res[-1.0]
                    f_sl, e_sl = split(tp, tm)
                    v_meas = math.degrees(0.5 * (vp - vm))
                    slides.append((v_meas, f_sl))
                    rec["slides"].append({"v_dps": v_dps, "amp_deg": amp, "t_plus": tp, "t_minus": tm,
                                          "v_plus_dps": math.degrees(vp), "v_minus_dps": math.degrees(vm),
                                          "steady": [shp, shm], "friction": f_sl, "gravity_error": e_sl,
                                          "model_gravity": 0.5 * (gp + gmn)})
                    print(f"   slides at {math.degrees(vp):.1f} / {math.degrees(vm):.1f} deg/s: needs +{tp:.3f} / {tm:.3f} Nm"
                          f"  ->  sliding friction {f_sl:.3f} Nm, gravity-model error {e_sl:+.3f} Nm"
                          f"   (model gravity here {0.5 * (gp + gmn):+.2f} Nm)")
                if slides:
                    if len(slides) == 2:
                        fc, bv = coulomb_viscous(slides)
                    else:
                        fc, bv = slides[0][1], None
                    rec.update(sliding=fc, viscous=bv)
                    es = [x["gravity_error"] for x in rec["slides"] if "gravity_error" in x]
                    rec["gravity_error_sliding"] = float(np.mean(es))
                    line = f"   sliding friction {fc:.3f} Nm" + (f" + {bv:.3f} Nm.s/rad x speed" if bv is not None else "")
                    if rec.get("friction"):
                        line += (f"  ({100 * fc / rec['friction']:.0f} % of the breakaway {rec['friction']:.3f}; gravity "
                                 f"error sliding {rec['gravity_error_sliding']:+.3f} vs breakaway {rec['gravity_error']:+.3f})")
                    print(line)
            out["angles"].append(rec)
            if stop:
                break
    finally:
        if claimed:
            try:
                if prev_recipe:
                    robot.set_recipe(prev_recipe)
                if robot.status().get("mode") not in ("IDLE",):
                    robot.enter_idle_mode(wait=True)
                robot.release_control()
            except Exception as e:
                print(f"!! could not restore everything: {e}")
        robot.disconnect()
    vals = xml_values(out["angles"])
    if vals is not None:
        fsl, emax, gmax = vals
        frac = float(sysc["friction_comp"]["fraction"])
        out.update(sliding=fsl, gravity_error_max=emax, model_gravity_max=gmax)
        print(f"\nJ{args.joint}: lowest sliding friction {fsl:.3f} Nm (any angle, either speed, or extrapolated to 0 speed), "
              f"largest gravity-model error while sliding {emax:.3f} Nm — measured where the model's gravity on "
              f"J{args.joint} was at most {gmax:.2f} Nm"
              f"\n  -> freedrive friction compensation {frac:g} x ({fsl:.3f} - {emax:.3f}) = "
              f"{frac * max(0.0, fsl - emax):.3f} Nm. For the robot XML (joint {args.joint}):"
              f"\n     <friction_sliding_nm>{fsl:.3f}</friction_sliding_nm>  <gravity_error_nm>{emax:.3f}</gravity_error_nm>"
              f"\n  A gravity-loaded joint (J2, J3): only if the angles tested include where it carries the MOST gravity "
              f"— a model error tends to grow with the load.")
    if out["angles"]:
        path = TEMP_DATA_DIR / "friction"
        path.mkdir(parents=True, exist_ok=True)
        path = path / f"{time.strftime('%Y%m%d_%H%M%S')}_J{args.joint}.json"
        path.write_text(json.dumps(out))
        print(f"\nsaved {path.relative_to(_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
