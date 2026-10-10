#!/usr/bin/env python3
"""56: one joint's frequency response — how it moves per Nm of torque from 15 to 80 Hz — and from it the most
wall DAMPING it can take before it rings. Only the tested joint gets a push (J1, J4, J5 or J6; default J5).

Why (2026-10-02): the box demo (55) shook at ~30 Hz — J5, with J3 moving too — because the wall's damping reaches
the motor ~10 ms late, and at 30 Hz that pushes WITH the motion. The stability table is computed for a RIGID arm
and cannot see it: between motor and link are gears and belts that act like springs, and around 30 Hz the motor
and the link start to swing against each other. This measures what the table is missing.

How: hard RTI torque mode, ~2.5 min. Every other joint gets zero extra torque (they float as in freedrive).
  1. Sweep (~35 s): a sine torque on the tested joint, one frequency at a time (15 ... 100 Hz), at least 0.2 Nm
     (below that J5's friction holds it and the answer would be the friction), adjusted towards ±0.3 deg of motion
     (at most 30 deg/s), never more than --max Nm or faster than the robot's torque-rate limit; plus a gentle
     centring spring (the torque the robot SENT is what the analysis uses, so it does not bias the result).
     -> per frequency: motion per Nm, the APPARENT inertia (rigid: constant; a springy drive: it changes around the
     drivetrain's frequency), the phase; and a fitted two-mass model (link inertia, drivetrain stiffness, delay; the
     motor's own inertia fixed to the model's) — where the link starts to swing against the motor.
  2. Damping limit (~30 s per speed filter: 0 / 4 / 8 ms): a wall's damping law on the joint while it SLIDES slowly
     back and forth (±3 deg at 5 deg/s — as a hand moving the arm), the damping raised until it starts to ring, then
     confirmed: the largest damping that stays calm for 6 s. Keep walls below HALF of it (friction can slow a ringing
     so much that 6 s misses it — the self-test has such a joint; half stayed calm for 30 s).
     Compared with what the joint's freedrive wall and the box used.
Predicting the limit from the fitted model instead was tried and dropped: it hinges on how the light motor behaves
near 125 Hz, which a sweep through friction cannot pin down (10-20x off in the self-test).

Stops (velocity-zero brake): the tested joint 3 deg off where it should be, any other joint 2 deg, or the tested
joint faster than 200 deg/s; a ringing is stopped within ~75 ms of being seen (D back to 0). Robot ENABLED, homed, IDLE; the arm somewhere the joint has 15 deg of room.

    .venv/bin/python api_tests/56_joint_resonance.py --selftest      # the sweep + analysis on a simulated springy joint
    .venv/bin/python api_tests/56_joint_resonance.py                 # dry run: the plan, nothing moves
    .venv/bin/python api_tests/56_joint_resonance.py --go            # J5 at the pose it is in now
    .venv/bin/python api_tests/56_joint_resonance.py --joint 6 --go

Saved to temp_data/resonance/<time>_J<n>.json (+ .npz of the telemetry).
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
ALLOWED = (1, 4, 5, 6)                 # J2/J3 never get a push (servo-test rule); they float as in freedrive
FREQS = (15, 18, 21, 24, 27, 30, 33, 36, 40, 45, 50, 56, 63, 70, 80, 90, 100)
# Not below 15 Hz: there ±0.3 deg needs less torque than J5's friction (~0.08 Nm) — the joint sticks and jumps,
# and the answer is the friction, not the drivetrain (seen in the self-test at 8 Hz).
X_TARGET_DEG = 0.3                     # the motion aimed for at each frequency ...
V_MAX_DEG = 30.0                       # ... but never more than this speed amplitude
TAU_MIN = 0.2                          # Nm: never less (2.5x J5's friction): below it the joint sticks and the
                                       # response is the friction's, not the drivetrain's (seen in the self-test)
SETTLE_S, MEASURE_S = 0.8, 1.2         # per frequency: amplitude control, then measured
CENTRE_K, CENTRE_I, CENTRE_D = 1.0, 1.0, 0.06    # Nm/rad, Nm/(rad s), Nm.s/rad on a 3 Hz filtered speed
CENTRE_TAU = 0.053                     # s: that filter — it damps the ~3.6 Hz centring, and is 1/10 at 30 Hz
DRIFT_DEG, OTHER_DEG, V_ABORT_DEG = 3.0, 2.0, 200.0
BRAKE_S = 0.4
RECIPE = "haptic_wall"                 # TCP_p, TCP_s, j_p, j_s, j_t, j_tc, tick, mode


# ----------------------------------------------------------------------------------------------- the sweep
def run_sweep(j, freqs, send_torque, send_brake, read_state, pace, tau_max, rate_limit, tau_min=TAU_MIN, n=6, log=None):
    """Excite joint j with a stepped sine; returns (windows [(f, tick_from, tick_to, amplitude)], why stopped or
    None). read_state() -> dict with "tick", "j_p", "j_s" (rad, rad/s) or None; the torque sent is the robot's
    business (the analysis reads it back from telemetry). Ramp/amplitude by tick count, not the wall clock."""
    tau = [0.0] * n
    first = None
    while first is None:
        send_torque(tau)
        pace()
        first = read_state()
    q0 = np.asarray(first["j_p"], dtype=float).copy()
    x0, integ, vlp = float(q0[j]), 0.0, 0.0
    a_v = 1.0 - math.exp(-1.0 / (FS * CENTRE_TAU))
    windows, k = [], 0
    last = first
    amp = tau_min
    for f in freqs:
        w = 2 * math.pi * f
        x_t = min(math.radians(X_TARGET_DEG), math.radians(V_MAX_DEG) / w)
        a_cap = min(tau_max, 0.8 * rate_limit / w)          # the sine's slope stays under the torque-rate limit
        amp = min(max(amp, tau_min), a_cap)                 # start from the last frequency's amplitude
        hist = []                                          # (tick, x) of the last 0.2 s, for the amplitude control
        n_settle, n_meas = int(SETTLE_S * FS), int(MEASURE_S * FS)
        t_from = None
        for i in range(n_settle + n_meas):
            s = read_state()
            if s is not None:
                last = s
            q = np.asarray(last["j_p"], dtype=float)
            x, v = float(q[j]), float(last["j_s"][j])
            if abs(x - x0) > math.radians(DRIFT_DEG):
                return windows, f"J{j + 1} drifted {math.degrees(x - x0):+.1f} deg at {f} Hz — stopped"
            others = max((abs(q[i2] - q0[i2]) for i2 in range(n) if i2 != j), default=0.0)
            if others > math.radians(OTHER_DEG):
                return windows, f"another joint moved {math.degrees(others):.1f} deg at {f} Hz — stopped"
            if abs(v) > math.radians(V_ABORT_DEG):
                return windows, f"J{j + 1} at {math.degrees(v):.0f} deg/s at {f} Hz — stopped"
            hist.append((int(last["tick"]), x))
            hist = hist[-int(0.2 * FS):]
            if i < n_settle and i % 12 == 11 and len(hist) > 25:      # every ~50 ms: steer the amplitude
                tt = np.array([h[0] for h in hist], dtype=float) / FS
                xx = np.array([h[1] for h in hist])
                xx = xx - xx.mean()
                got = 2.0 * abs(np.mean(xx * np.exp(-1j * w * tt)))
                ratio = 1.5 if got < 1e-7 else min(max(x_t / got, 0.5), 1.5)
                amp = min(max(amp * ratio, tau_min), a_cap)
            if i == n_settle:
                t_from = int(last["tick"])
            integ += (x - x0) / FS
            vlp += a_v * (v - vlp)
            centre = -CENTRE_K * (x - x0) - CENTRE_I * integ - CENTRE_D * vlp
            tau[j] = amp * math.sin(w * k / FS) + centre
            tau[j] = max(-tau_max, min(tau_max, tau[j]))
            send_torque(tau)
            if log is not None:
                log.append((k, tau[j]))
            k += 1
            pace()
        windows.append((f, t_from, int(last["tick"]), amp))
    return windows, None


SLIDE_DEG, SLIDE_DPS = 4.0, 6.0        # the damping ramp keeps the joint SLIDING: back and forth ±4 deg at 6 deg/s
TRACK_K, TRACK_I = 4.0, 4.0            # Nm/rad, Nm/(rad s): the spring that makes it follow (~7 Hz)
FRICTION_FF = 0.06                     # Nm pushed in the direction of travel (J5's friction, api_tests/54) —
                                       # 2026-10-02 on the arm: without it J5 stuck and jumped (stick-slip), and the
                                       # 10-20 Hz bursts of every jump read as ringing at ANY damping
D_START, D_STEP, D_MAX, STEP_S = 0.005, 1.10, 2.0, 0.4
RING_HZ = 25.0                         # ringing is looked for above this (2nd-order high-pass): the drivetrain mode
                                       # (~32 Hz on J5) and the motor's own (to 125 Hz) — not the 10-20 Hz slips
RING_X, RING_FLOOR_DEG, RING_WINDOWS, RING_WIN = 3.0, 5.0, 5, 6   # 3x the start's 90th percentile, 5 windows of 24 ms


def slide_ref(t):
    """(position offset rad, speed rad/s, seconds since the last turn) of the back-and-forth."""
    a, v = math.radians(SLIDE_DEG), math.radians(SLIDE_DPS)
    period = 4.0 * a / v
    ph = (t + a / v) % period                               # starts at 0, moving +
    if ph < period / 2:
        return -a + v * ph, v, ph
    return a - v * (ph - period / 2), -v, ph - period / 2


def run_damping_ramp(j, tau_s, send_torque, read_state, pace, tau_max, n=6, d_fixed=None, max_s=None,
                     friction=FRICTION_FF):
    """The most damping the joint takes, measured: a wall's damping law (-D x speed through the wall's EMA filter)
    on the joint while it SLIDES slowly back and forth (±3 deg at 5 deg/s — as a hand moving the arm: friction is
    then a constant force and cannot hold a small ringing back). D is raised x1.15 every 0.4 s until the speed
    above ~10 Hz grows to 3x its start (and > 3 deg/s) for 3 windows of 50 ms in a row (only while sliding, not at
    the turns, where sticking makes its own bursts) — then D goes straight back to 0. d_fixed (+ max_s): after the
    same baseline at the starting D, hold that one D instead (the self-test's check of the ramp). The damping acts on the speed
    AROUND the sliding speed (stability does not change; the joint is not pushed back by D x 5 deg/s).
    Returns (D_max — the last D that did not ring —, ringing Hz, why stopped otherwise)."""
    tau = [0.0] * n
    first = None
    while first is None:
        send_torque(tau)
        pace()
        first = read_state()
    q0 = np.asarray(first["j_p"], dtype=float).copy()
    x0, integ, vlp, vf, lo1, lo2 = float(q0[j]), 0.0, 0.0, 0.0, 0.0, 0.0
    a_c = 1.0 - math.exp(-1.0 / (FS * CENTRE_TAU))
    a_w = 1.0 if tau_s <= 0 else 1.0 - math.exp(-1.0 / (FS * tau_s))
    a_hp = 1.0 - math.exp(-2.0 * math.pi * RING_HZ / FS)  # two first-order high-passes at RING_HZ
    D, k, last, d_step = D_START, 0, first, (1.0 if d_fixed is not None else D_STEP)
    win, base, hot, hp_hist, base_w = [], None, 0, [], []
    per_step = int(STEP_S * FS)
    while D <= D_MAX:
        s = read_state()
        if s is not None:
            last = s
        q = np.asarray(last["j_p"], dtype=float)
        x, v = float(q[j]), float(last["j_s"][j])
        dx, vr, since = slide_ref(k / FS)
        xr = x0 + dx
        if abs(x - xr) > math.radians(DRIFT_DEG):
            return None, None, f"J{j + 1} {math.degrees(x - xr):+.1f} deg off its slow back-and-forth at D {D:.3f} — stopped"
        others = max((abs(q[i] - q0[i]) for i in range(n) if i != j), default=0.0)
        if others > math.radians(OTHER_DEG):
            return None, None, f"another joint moved {math.degrees(others):.1f} deg at D {D:.3f} — stopped"
        if abs(v) > math.radians(V_ABORT_DEG):
            return D / d_step, None, f"J{j + 1} at {math.degrees(v):.0f} deg/s at D {D:.3f} — stopped"
        lo1 += a_hp * (v - lo1)
        h1 = v - lo1
        lo2 += a_hp * (h1 - lo2)
        hp = h1 - lo2
        sliding = since > 0.25
        if sliding:
            win.append(hp)
            hp_hist.append(hp)
            hp_hist = hp_hist[-128:]
        else:
            win = []
        if len(win) == RING_WIN:                            # a 24 ms window while sliding
            rms = math.sqrt(sum(h * h for h in win) / len(win))
            win = []
            if base is None:
                base_w.append(rms)
                if len(base_w) >= 50:                       # ~1.2 s of sliding at the first D: the baseline
                    base = max(float(np.percentile(base_w, 90)), 1e-4)
                    if d_fixed is not None:
                        D, k_fix = d_fixed, k
            elif rms > max(RING_X * base, math.radians(RING_FLOOR_DEG)):
                hot += 1
                if hot >= RING_WINDOWS:
                    tr = np.array(hp_hist)
                    sp = np.abs(np.fft.rfft(tr * np.hanning(len(tr))))
                    ff = np.fft.rfftfreq(len(tr), 1 / FS)
                    sp[ff < RING_HZ] = 0.0
                    fr = float(ff[int(np.argmax(sp))])
                    tau[j] = 0.0
                    send_torque(tau)
                    return D / d_step, fr, None
            else:
                hot = 0
        integ += (xr - x) / FS
        vlp += a_c * (v - vlp)
        vf += a_w * (v - vf)
        ff_fric = friction * math.copysign(min(1.0, since / 0.05), vr)     # eased in over 50 ms after each turn
        track = TRACK_K * (xr - x) + TRACK_I * integ - CENTRE_D * (vlp - vr) + ff_fric
        tau[j] = track - D * (vf - vr)
        tau[j] = max(-tau_max, min(tau_max, tau[j]))
        send_torque(tau)
        k += 1
        if base is not None and k % per_step == 0:
            D *= d_step
        if d_fixed is not None and base is not None and k - k_fix > (max_s or 5.0) * FS:
            return None, None, f"no ringing in {max_s:g} s at D {D:.3f}"
        pace()
    return None, None, f"no ringing up to {D_MAX} Nm.s/rad"


def damping_limit_measured(j, tau_s, send_torque, send_brake, read_state, pace, tau_max, n=6, hold_s=6.0, tries=5):
    """The ramp finds where ringing starts; it then CONFIRMS: hold 0.8x that D for hold_s while sliding, stepping
    down x0.8 until a D stays calm for the whole hold. A slowly growing ringing (friction fights it) can take longer
    than one 0.4 s ramp step to show — the self-test found the ramp alone 25 % too high for such a joint.
    Returns (confirmed D or None, the D it rang at, ringing Hz, why)."""
    d_ring, fr, why = run_damping_ramp(j, tau_s, send_torque, read_state, pace, tau_max, n=n)
    brake(send_brake, pace)                                   # let any ringing die before the next phase
    if d_ring is None:
        return None, None, None, why
    d_ring *= D_STEP                                          # the ramp returns the last D BEFORE the ringing one
    d = 0.8 * d_ring
    for _ in range(tries):
        rang, fr2, why2 = run_damping_ramp(j, tau_s, send_torque, read_state, pace, tau_max, n=n, d_fixed=d,
                                           max_s=hold_s)
        brake(send_brake, pace)
        if why2 and ("drifted" in why2 or "another joint" in why2 or "deg/s" in why2 or "off its" in why2):
            return None, d_ring, fr, why2
        if rang is None:
            return d, d_ring, fr, None
        d *= 0.8
    return None, d_ring, fr, f"still ringing at {d / 0.8:.3f} after {tries} steps down"


def brake(send_brake, pace):
    for _ in range(int(BRAKE_S * FS)):
        send_brake()
        pace()


# -------------------------------------------------------------------------------------------- the analysis
def bin_at(t, x, f):
    """Complex amplitude of x at f (Hann window, linear trend removed). t in s."""
    x = np.asarray(x, dtype=float)
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    win = np.hanning(len(x))
    return 2.0 * np.sum(win * x * np.exp(-2j * np.pi * f * t)) / np.sum(win)


def analyse(d, windows, j):
    """Per frequency: torque sent, position and speed measured (complex amplitudes), from telemetry arrays
    d = {"tick", "j_p", "j_s", "j_tc"} (one row per tick)."""
    rows = []
    tick = np.asarray(d["tick"], dtype=float)
    for f, a, b, amp in windows:
        s = (tick >= a) & (tick <= b)
        if s.sum() < 0.8 * FS:
            continue
        t = tick[s] / FS
        T = bin_at(t, d["j_tc"][s, j], f)
        X = bin_at(t, d["j_p"][s, j], f)
        V = bin_at(t, d["j_s"][s, j], f)
        rows.append({"f": f, "T": T, "X": X, "V": V, "amp": amp})
    return rows


def ema_response(f, tau_s):
    """The walls' speed filter (EMA at 250 Hz) at frequency f."""
    if tau_s <= 0.0:
        return 1.0 + 0j
    a = 1.0 - math.exp(-1.0 / (FS * tau_s))
    z = np.exp(-2j * np.pi * f / FS)
    return a / (1.0 - (1.0 - a) * z)


def two_mass(params, f):
    """Motor-side position per Nm at the motor (joint-referred): Jm (motor), Jl (link), k, c (drivetrain), T (delay)."""
    jm, jl, k, c, td = params
    s = 2j * np.pi * np.asarray(f, dtype=float)
    num = jl * s * s + c * s + k
    den = s * s * (jm * jl * s * s + c * (jm + jl) * s + k * (jm + jl))
    return num / den * np.exp(-s * td)


def fit_two_mass(rows, j_total, j_motor):
    """Fit the measured position response (log magnitude and phase). The motor's own inertia is FIXED to the robot
    model's (Jm x G^2, a datasheet value): below the drivetrain's own resonance (~100+ Hz) the data barely shows it
    (the self-test's free fit ran into its bound), and it is what sets the high-frequency limit. Returns
    ([Jm, Jl, k, c, T], rms)."""
    from scipy.optimize import least_squares
    f = np.array([r["f"] for r in rows], dtype=float)
    H = np.array([r["X"] / r["T"] for r in rows])

    def res(p):
        e = np.log(H / two_mass([j_motor] + np.exp(p[:3]).tolist() + [p[3]], f))
        return np.concatenate([e.real, np.angle(np.exp(1j * e.imag))])

    best = None
    jl0 = max(j_total - j_motor, 0.5 * j_total)
    for fa in (15.0, 25.0, 35.0, 50.0, 80.0, 150.0):        # starts: anti-resonance guesses
        p0 = [math.log(jl0), math.log(jl0 * (2 * math.pi * fa) ** 2), math.log(0.01), 0.006]
        try:
            r = least_squares(res, p0, bounds=([math.log(1e-6), math.log(0.1), math.log(1e-6), 0.0],
                                               [math.log(1.0), math.log(1e7), math.log(10.0), 0.03]))
        except Exception:
            continue
        if best is None or r.cost < best.cost:
            best = r
    return [j_motor] + np.exp(best.x[:3]).tolist() + [float(best.x[3])], float(np.sqrt(np.mean(best.fun ** 2)))


def grows(tr, kick, ticks):
    """(unstable?, its frequency): a kicked loop is UNSTABLE if its speed ends above twice the kick (a stable one
    only loses energy; a free joint without damping just coasts at the kick). The same test for the measured model
    and the self-test's plant."""
    nw = min(256, len(tr))
    spec = np.abs(np.fft.rfft((tr[-nw:] - np.mean(tr[-nw:])) * np.hanning(nw)))
    fr = float(np.fft.rfftfreq(nw, 1 / FS)[1 + int(np.argmax(spec[1:]))])
    if len(tr) < ticks or not np.all(np.isfinite(tr)):
        return True, fr
    return bool(np.max(np.abs(tr[-100:])) > 2.0 * kick), fr


def report(rows, fit, j_total, j_motor, limits, walls_d):
    print("\n    f Hz   torque Nm   motion deg   apparent inertia   phase (x per Nm)")
    for r in rows:
        w = 2 * np.pi * r["f"]
        Hx = r["X"] / r["T"]
        print(f"   {r['f']:5.0f}    {abs(r['T']):7.3f}     {math.degrees(abs(r['X'])):7.3f}      "
              f"{1.0 / (w * w * abs(Hx)):.5f} kg m^2   {math.degrees(np.angle(Hx)):+7.0f} deg")
    print(f"   (the model: {j_total:.5f} kg m^2 with the other joints free; the motor's own share {j_motor:.5f})")
    if fit is not None:
        (jm, jl, k, c, td), rms = fit
        fa = math.sqrt(k / jl) / (2 * math.pi)
        print(f"\n   Two-mass fit (rms {rms:.2f}; motor inertia fixed to the model's): link {jl:.2e} kg m^2 (total "
              f"{jm + jl:.5f}), drivetrain {k:.1f} Nm/rad, delay {1000 * td:.1f} ms -> the link swings against the "
              f"motor from ~{fa:.0f} Hz" + (" (above the band: stiff)" if fa > 110 else ""))
    print("\n   The most wall damping it took (measured: sliding slowly, D raised until it rang, then confirmed calm "
          "for 6 s):")
    for tau_ms, (d, d_ring, fr, why) in limits.items():
        if d is None:
            print(f"      speed filter {tau_ms} ms: not found ({why})")
        else:
            print(f"      speed filter {tau_ms} ms: {d:.3f} Nm.s/rad calm (rang at {d_ring:.3f}, ~{fr:.0f} Hz) "
                  f"-> keep walls below ~{0.5 * d:.3f}")
    for name, dv in walls_d:
        print(f"      for comparison — {name}: {dv:.3f} Nm.s/rad")
    print("   (Half: friction slows some ringing down so much that a 6 s hold can miss it — in the self-test a springy\n"
          "    joint held calm for 4 s at its 'limit' and rang after 8; half of it stayed calm for 30 s.)")


# ---------------------------------------------------------------------------------------------- self-test
def sim_plant(jm, jl, k, c, fric=0.07, bias=0.03, sub=40, stick=None, ripple=0.0):
    """Motor (jm, Coulomb friction, the encoder) -spring k, damper c- link (jl). Each tick's packet holds the
    state at the end of the tick and the command sent then, which acts during the NEXT tick — the PC and the
    telemetry see the same packet, as on the arm. stick: the torque to break loose (default = fric, the sliding
    friction); ripple: stepper torque ripple (Nm) at 4x the electrical frequency (50 pole pairs, gear 4) — together
    they make the stick-slip bursts the real J5 showed (2026-10-02: they fooled the first ringing detector)."""
    stick = fric if stick is None else stick
    st = {"xm": 0.0, "vm": 0.0, "xl": 0.0, "vl": 0.0, "cmd": 0.0, "pend": 0.0, "tick": 0, "rows": [], "hist": []}
    dt = 1.0 / FS / sub

    def send_torque(tau):
        st["pend"] = tau[4]

    def send_brake():
        st["pend"] = None

    def pace():
        # RTI's packet at tick n holds the state RTI READ at n and the command it computed from it and SENT at n;
        # that command acts during the next tick. (The first version logged the state AFTER the command's tick,
        # which hid one tick of the loop's delay from the measured response.)
        q = np.zeros(6)
        q[4] = st["xm"]
        qd = np.zeros(6)
        qd[4] = st["vm"]
        tc = np.zeros(6)
        tc[4] = 0.0 if st["pend"] is None else st["pend"]
        st["tick"] += 1
        st["rows"].append((st["tick"], q, qd, tc))
        cmd = st["cmd"]                                       # sent last tick: acts during this one
        st["cmd"] = st["pend"]
        for _ in range(sub):
            if cmd is None:                                   # velocity-zero brake
                st["vm"], st["vl"] = 0.0, 0.0
                break
            fs = k * (st["xm"] - st["xl"]) + c * (st["vm"] - st["vl"])
            net = cmd + bias - fs + ripple * math.sin(800.0 * st["xm"])
            if abs(st["vm"]) < 1e-6 and abs(net) <= stick:
                am = 0.0
                st["vm"] = 0.0
            else:
                am = (net - math.copysign(fric, st["vm"] if st["vm"] else net)) / jm
            al = fs / jl
            st["vm"] += am * dt
            st["vl"] += al * dt
            st["xm"] += st["vm"] * dt
            st["xl"] += st["vl"] * dt
        q2 = np.zeros(6)
        q2[4] = st["xm"]
        qd2 = np.zeros(6)
        qd2[4] = st["vm"]
        st["hist"].append({"tick": st["tick"], "j_p": q2, "j_s": qd2})

    def read_state():
        return st["hist"][-1] if st["hist"] else None         # the same packet the telemetry has

    return send_torque, send_brake, read_state, pace, st



def sim_ring(D, jm, jl, k, c, tau_s=0.004, ticks=1000):
    """The self-test's plant (no friction) with a wall's damping on it, kicked: (unstable?, frequency) by grows()."""
    tq_, br_, rs_, pc_, st_ = sim_plant(jm, jl, k, c, fric=0.0, bias=0.0)
    a = 1.0 if tau_s <= 0 else 1.0 - math.exp(-1.0 / (FS * tau_s))
    vf, tau = 0.0, [0.0] * 6
    st_["vm"] = 0.05
    trace = []
    for _ in range(ticks):
        s = rs_()
        v = 0.0 if s is None else float(s["j_s"][4])
        vf += a * (v - vf)
        tau[4] = -D * vf
        tq_(tau)
        pc_()
        trace.append(st_["vm"])
        if not math.isfinite(st_["vm"]) or abs(st_["vm"]) > 1e3:
            break
    return grows(np.array(trace), 0.05, ticks)


def selftest():
    bad = []

    def chk(name, cond, detail=""):
        print(("  PASS  " if cond else "  FAIL  ") + name + (f"   [{detail}]" if detail else ""))
        if not cond:
            bad.append(name)

    def arrays(st):
        r = st["rows"]
        return {"tick": np.array([x[0] for x in r], float), "j_p": np.array([x[1] for x in r]),
                "j_s": np.array([x[2] for x in r]), "j_tc": np.array([x[3] for x in r])}

    def truth(jm, jl, k, c, tau_s):
        """The plant's own damping limit: bisection on the simulated wall (sim_ring), no friction."""
        lo, hi = 0.0, 4.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if sim_ring(mid, jm, jl, k, c, tau_s=tau_s)[0]:
                hi = mid
            else:
                lo = mid
        return lo

    # 1. The sweep and the fit (the resonance itself).
    cases = (("a springy joint (drivetrain ~30 Hz), friction 0.07 Nm, 0.03 Nm gravity error", 1.0e-4, 0.0019, 70.0, 0.01),
             ("a stiffer joint (drivetrain ~60 Hz)", 1.0e-4, 0.0019, 270.0, 0.02),
             ("a near-RIGID joint (drivetrain ~200 Hz, above the band)", 1.0e-4, 0.0019, 3000.0, 0.2))
    for name, jm, jl, k, c in cases:
        print(f"\n  Sweep — {name}:")
        tq, br, rs, pc, st = sim_plant(jm, jl, k, c, fric=0.07, bias=0.03)
        wins, why = run_sweep(4, FREQS, tq, br, rs, pc, tau_max=0.6, rate_limit=173.0)
        chk("the sweep runs every frequency without tripping a stop", why is None and len(wins) == len(FREQS), why)
        drift = max(abs(math.degrees(x[1][4])) for x in st["rows"])
        chk("it stays near its start (under 3 deg)", drift < 3.0, f"{drift:.2f} deg")
        (fjm, fjl, fk, fc, ftd), rms = fit_two_mass(analyse(arrays(st), wins, 4), jm + jl, jm)
        fa_true, fa_fit = math.sqrt(k / jl) / (2 * math.pi), math.sqrt(fk / fjl) / (2 * math.pi)
        if k < 1000:
            chk("the fit finds the drivetrain: its frequency within 10 %, the total inertia within 15 %",
                abs(fa_fit - fa_true) < 0.1 * fa_true and abs(fjm + fjl - (jm + jl)) < 0.15 * (jm + jl),
                f"{fa_fit:.1f} vs {fa_true:.1f} Hz; total {fjm + fjl:.5f} vs {jm + jl:.5f}; delay {1000 * ftd:.1f} ms")
        else:
            chk("a near-rigid joint: the fit puts the drivetrain above the band (>120 Hz)", fa_fit > 120.0, f"{fa_fit:.0f} Hz")

    # 2. The measured damping limit, against the plant's own (frictionless) limit.
    def truth(jm, jl, k, c, tau_s):
        lo, hi = 0.0, 4.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if sim_ring(mid, jm, jl, k, c, tau_s=tau_s)[0]:
                hi = mid
            else:
                lo = mid
        return lo

    plants = (("J5 as measured (drivetrain 73 Nm/rad, damping 0.125), HARSH stick-slip + stepper ripple — the real "
               "J5's sliding fooled the first detector", (9.12e-5, 0.00181, 73.0, 0.125), dict(fric=0.05, stick=0.12, ripple=0.03),
               False),
              ("a stiffer joint (drivetrain ~60 Hz)", (1.0e-4, 0.0019, 270.0, 0.02), dict(fric=0.07), False),
              ("a near-rigid joint", (1.0e-4, 0.0019, 3000.0, 0.2), dict(fric=0.07), False),
              ("an extreme joint: light motor, almost NO drivetrain damping (0.01; the real J5 has 0.125)",
               (1.0e-4, 0.0019, 70.0, 0.01),
               dict(fric=0.07), True))
    for name, p, fr_kw, extreme in plants:
        for tau_s in (0.0, 0.004):
            print(f"\n  Damping limit — {name}, filter {1000 * tau_s:.0f} ms:")
            tq, br, rs, pc, st = sim_plant(*p, bias=0.03, **fr_kw)
            d, d_ring, fr, why = damping_limit_measured(4, tau_s, tq, br, rs, pc, tau_max=0.6)
            true = truth(*p, tau_s)
            if extreme:                           # its barely damped motor rings from the slide itself
                chk("a joint the slide alone makes ring: STOPPED by the speed guard, no value reported (never a guess)",
                    d is None and why is not None and "deg/s" in why, why)
                continue
            chk("found and confirmed calm (no false alarm from the sliding)", d is not None and d > 0.1 * true,
                f"{why}; measured {d}, true {true:.4f}")
            if "HARSH" in name and tau_s == 0.0:
                v = np.abs(np.array([x[2][4] for x in st["rows"]]))
                med = math.degrees(float(np.median(v[: int(20 * FS)])))
                chk(f"the friction push makes it really SLIDE (median speed > 60 % of the {SLIDE_DPS:g} deg/s asked; the "
                    f"real J5 without it: 1.3 deg/s, stuck-and-jump)", med > 0.6 * SLIDE_DPS, f"{med:.1f} deg/s")
                tq2, br2, rs2, pc2, st2 = sim_plant(*p, bias=0.03, **fr_kw)
                d2, _, why_2 = run_damping_ramp(4, tau_s, tq2, rs2, pc2, tau_max=0.6, friction=0.0)
                chk("...and with the friction push OFF (a joint whose friction is set wrong), the 25 Hz detector alone "
                    "still raises no false alarm", d2 is None or d2 > 0.3 * true, f"ramp says {d2} ({why_2}), true {true:.4f}")
            if d is None:
                continue
            if 0.5 * d < true:
                chk("the recommended ceiling (half) is below the joint's own limit, and the measurement is not needlessly "
                    "low (>= 0.7x)", d >= 0.7 * true, f"measured {d:.4f} (rang {d_ring:.3f} at {fr:.0f} Hz), half "
                    f"{0.5 * d:.4f}, true {true:.4f}")
            else:
                print(f"     half of the measured {d:.3f} is above this joint's own limit without friction ({true:.4f}): "
                      f"friction slows its ringing so much that the hold misses it")
            tq, br, rs, pc, st = sim_plant(*p, bias=0.03, **fr_kw)
            rang, _, why2 = run_damping_ramp(4, tau_s, tq, rs, pc, tau_max=0.6, d_fixed=0.5 * d, max_s=30.0)
            chk("half of it stays calm for 30 s, sliding, with the friction", rang is None, why2)
    print(f"\n{len(bad)} failed" + (f": {bad}" if bad else " - all passed"))
    return 1 if bad else 0


# --------------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--joint", type=int, default=5, help="1-based: 1, 4, 5 or 6 (default 5)")
    ap.add_argument("--max", type=float, default=None, help="Nm, the largest torque (default 0.6, <= 40%% of the drive)")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.joint not in ALLOWED:
        print(f"--joint must be one of {ALLOWED} (J2/J3 are never pushed by a servo test).")
        return 2
    j = args.joint - 1

    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML, TEMP_DATA_DIR
    from robotclient import RobotClient, ControlLost
    from _rti_session import rti_session, Pacer
    from utility.haptic_walls import joint_drive_torque, make_effective_inertia
    from utility.robot_factory import create_robot
    base = __import__("46_monitor_baseline")
    w51 = __import__("51_haptic_wall")
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    model = create_robot(sysc["active_robot"], gripper_name=cfg.get("gripper_name", ""))
    lo = float(np.degrees(cfg["software_joint_limit_negative"][j]))
    hi = float(np.degrees(cfg["software_joint_limit_positive"][j]))
    drive = float(joint_drive_torque(cfg)[j])
    tau_max = min(0.6 if args.max is None else args.max, 0.4 * drive)
    rate = float(np.asarray(cfg["torque_rate_limit"], dtype=float)[j])
    j_motor = float(model.links[j].Jm) * float(model.links[j].G) ** 2

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed = False
    capture = prev_recipe = None
    windows, why, limits = [], None, {}
    try:
        qd = np.asarray(robot.get_joints(), dtype=float)
        ieff = np.zeros(6)
        make_effective_inertia(model)(np.radians(qd), ieff)
        j_total = float(ieff[j])
        room = min(qd[j] - lo, hi - qd[j])
        print(f"J{args.joint} at {qd[j]:.1f} deg (soft limits [{lo:.1f}, {hi:.1f}], {room:.0f} deg of room); model inertia "
              f"{j_total:.5f} kg m^2 (motor's share {j_motor:.5f}). Sine {FREQS[0]}-{FREQS[-1]} Hz, ~±{X_TARGET_DEG} deg "
              f"(<= {V_MAX_DEG:g} deg/s), torque <= {tau_max:.2f} Nm (drive {drive:.2f}), ~{len(FREQS) * (SETTLE_S + MEASURE_S):.0f} s; "
              f"then the damping limit for 0 / 4 / 8 ms filters, ~30 s each.")
        if room < 15.0:
            print("Less than 15 deg of room to a soft limit — move the joint first. Nothing moved.")
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
        capture = base.Capture()
        with rti_session(robot) as (rti, _first):
            pace = Pacer(FS)

            print("1/2 Sweeping 15-100 Hz (~35 s) — keep clear of the arm; it buzzes softly.")
            limits = {}
            try:
                windows, why = run_sweep(j, FREQS, rti.send_joint_torque, lambda: rti.send_joint_velocity([0.0] * 6),
                                         rti.read_state, pace, tau_max, rate)
                for tau_ms in (() if why else (0, 4, 8)):
                    print(f"2/2 Damping limit, speed filter {tau_ms} ms (~30 s): it slides slowly back and forth; "
                          f"a short buzz when it starts to ring is expected.")
                    limits[tau_ms] = damping_limit_measured(j, tau_ms / 1000.0, rti.send_joint_torque,
                                                            lambda: rti.send_joint_velocity([0.0] * 6), rti.read_state,
                                                            pace, tau_max)
                    if limits[tau_ms][3] and ("drifted" in limits[tau_ms][3] or "another joint" in limits[tau_ms][3]
                                              or "deg/s" in limits[tau_ms][3] or "off its" in limits[tau_ms][3]):
                        why = limits[tau_ms][3]
                        break
                brake(lambda: rti.send_joint_velocity([0.0] * 6), pace)
            except ControlLost as e:
                why = f"control lost: {e}"
    finally:
        if capture is not None:
            time.sleep(0.2)
            capture.stop()
        if claimed:
            try:
                if prev_recipe:
                    robot.set_recipe(prev_recipe)
                if robot.status().get("mode") != "IDLE":
                    robot.enter_idle_mode(wait=True)
                robot.release_control()
            except Exception as e:
                print(f"!! could not restore everything: {e}")
        robot.disconnect()
    if why:
        print(f"STOPPED: {why}")
    if capture is None or not capture.packets or not windows:
        return 1 if why else 0
    d = w51.to_arrays(capture.packets)
    rows = analyse(d, windows, j)
    w = walls.get(args.joint, {})
    deg = 180.0 / math.pi
    walls_d = [(f"J{args.joint}'s freedrive wall at its lightest pose ({w.get('frequency_hz', 0):.1f} Hz)",
                float(w.get("damping_nm_s_per_deg", 0.0)) * deg)] if w else []
    if args.joint == 5:
        walls_d.append(("the box wall's share on J5 when the box demo shook", 0.156))
    fit = None
    try:
        fit = fit_two_mass(rows, j_total, j_motor)
    except Exception as e:
        print(f"(two-mass fit failed: {e})")
    report(rows, fit, j_total, j_motor, limits, walls_d)
    out = TEMP_DATA_DIR / "resonance"
    out.mkdir(parents=True, exist_ok=True)
    stem = out / f"{time.strftime('%Y%m%d_%H%M%S')}_J{args.joint}"
    np.savez(f"{stem}.npz", **d)
    Path = __import__("pathlib").Path
    Path(f"{stem}.json").write_text(json.dumps({
        "joint": args.joint, "pose_deg": qd.tolist(), "model_inertia": j_total, "motor_inertia": j_motor,
        "rows": [{"f": r["f"], "T": [r["T"].real, r["T"].imag], "X": [r["X"].real, r["X"].imag],
                  "V": [r["V"].real, r["V"].imag], "amp": r["amp"]} for r in rows],
        "damping_limit": {str(k): {"calm": v[0], "rang": v[1], "hz": v[2], "why": v[3]} for k, v in limits.items()},
        "fit": None if fit is None else {"motor": fit[0][0], "link": fit[0][1], "k": fit[0][2], "c": fit[0][3],
                                         "delay": fit[0][4], "rms": fit[1]}}))
    print(f"\nsaved {Path(str(stem) + '.json').relative_to(_ROOT)} (+ .npz)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
