"""Standalone current, velocity, and position loop tuner for a single Spectral motor driver.

Independent of the RCB runtime — it talks to the driver directly and shares nothing with
RTI except the CAN interface, which is exactly why it refuses to start while RTI is alive.

  ONE motor on the bus, CAN id 0, SECURELY MOUNTED, SHAFT FREE TO SPIN.
  Velocity and position tuning both make the motor turn. Remove anything that can be
  thrown, and do not clamp the shaft — a blocked rotor is the current-loop test, not
  this one.

Typical session:

    # see how it behaves as configured
    python3 tools/tune_motor_loops.py --mode velocity --excite step --label kpv_stock

    # sweep the proportional gain and overlay the results
    python3 tools/tune_motor_loops.py --mode velocity --excite step --sweep 0.005,0.01,0.02,0.04

    # frequency response of the tuned result
    python3 tools/tune_motor_loops.py --mode velocity --excite chirp --kpv 0.02 --kiv 0.01

    # then the outer loop, which must be slower than the inner one
    python3 tools/tune_motor_loops.py --mode position --excite step --sweep 5,10,20,40

Targets (each loop 3-10x slower than the one inside it):
    current  500-1000 Hz   (current mode observes it through CAN; firmware loop is 6.25 kHz)
    velocity  50-150 Hz
    position   10-30 Hz

WHY THERE IS NO LIVE PLOT. Sampling is a CAN poll-response: one Send_data_pack_1 out,
one 8-byte reply back carrying position+speed+current together. At 1 Mbit that is ~260 us
of wire time per sample, so ~1 kHz is honest and ~3.8 kHz is the hard ceiling. Rendering
during capture would cost more than the measurement. Capture fast, plot afterwards.

Command and response are recorded as SEPARATE timestamped series and aligned during
analysis. They are not paired per-iteration on purpose: the reply arriving in tick N is
the answer to tick N-1's command, so pairing them would silently bake one CAN round trip
into every measurement. Keeping them apart lets the transport delay be reported as its own
number ("dead time") instead of inflating the rise time.
"""

import argparse
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")           # headless Pi - must precede pyplot
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import Spectral_BLDC as Spectral

RESULTS = Path(__file__).resolve().parent.parent / "temp_data" / "motor_tuning"
# Read from the library rather than copied here - a stale duplicate of this constant
# captures zero samples and looks exactly like a dead motor.
RESPOND_DATA_PACK_1_ID = Spectral.SpectralCAN.RESPOND_DATA_PACK_1_ID
ENCODER_TICKS_PER_REV = 1 << 14
CURRENT_LOOP_TIME_S = 0.00016      # firmware: #define LOOP_TIME 0.00016
CURRENT_LOOP_FREQ_HZ = 1.0 / CURRENT_LOOP_TIME_S
CURRENT_PI_KP_DIVISOR = 7.0        # empirical firmware scaling in PID.Kp_iq


def calculate_current_pi_gains(resistance_ohm, inductance_h, bandwidth_hz):
    """Reproduce the firmware's R/L current-loop PI-gain calculation exactly.

    ``bandwidth_hz`` is ``current_control_bandwidth``. The resulting ``kpiq`` and
    ``kiiq`` are the values accepted by ``Send_Current_Gains``. Inductance is in
    henries here; the GUI uses the same unit directly.
    """
    values = (resistance_ohm, inductance_h, bandwidth_hz)
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("resistance, inductance, and bandwidth must all be positive")

    discrete_ki = 1.0 - math.exp(-resistance_ohm * CURRENT_LOOP_TIME_S / inductance_h)
    crossover = bandwidth_hz / (CURRENT_LOOP_FREQ_HZ / (2.0 * math.pi))
    kpiq = resistance_ohm * (crossover / discrete_ki) / CURRENT_PI_KP_DIVISOR
    kiiq = discrete_ki * kpiq
    return {
        "kpiq": kpiq,
        "kiiq": kiiq,
        "discrete_ki": discrete_ki,
        "crossover": crossover,
    }


# --------------------------------------------------------------------------- safety

def _rti_is_running():
    """RTI owns the CAN bus. Two processes polling it would interleave replies and
    quietly corrupt both sets of measurements."""
    try:
        with open("/dev/shm/loop_tick", "rb") as f:
            a = struct.unpack("<d", f.read(8))[0]
        time.sleep(0.3)
        with open("/dev/shm/loop_tick", "rb") as f:
            b = struct.unpack("<d", f.read(8))[0]
        return b > a
    except (OSError, struct.error):
        return False            # no shm segment = RTI has never run


def preflight(node_id):
    if _rti_is_running():
        sys.exit(
            "REFUSING TO START: RTI.py is running (loop_tick is advancing).\n"
            "It owns the CAN bus. Stop it first:  sudo systemctl stop <supervisor unit>\n"
            "or kill the supervisor, then re-run this tool."
        )
    print("  RTI not running - bus is free")


def survey_bus(comm, node_id, seconds=1.0):
    """Anything other than our motor answering is a wiring mistake worth catching now."""
    seen = set()
    end = time.time() + seconds
    while time.time() < end:
        message, msg_id = comm.receive_can_messages(timeout=0.05)
        if message is not None and msg_id is not None:
            seen.add(msg_id.node_id)
    if seen - {node_id}:
        print(f"  WARNING: other CAN nodes are talking: {sorted(seen - {node_id})}")
        print("  This tool expects ONE motor on the bus. Their traffic will steal bandwidth.")
    return seen


def read_state(comm, motor, node_id, tries=20):
    """Return the driver's current (position, speed, current), or None.

    Deliberately insists on a RESPOND_DATA_PACK_1 rather than accepting any frame from the
    node. Position mode steps RELATIVE to this reading, so a caller that treated "some
    frame arrived" as success could end up with position=None -> a default of 0 -> an
    ABSOLUTE move to tick 0 from wherever the shaft actually sits. That is a slam, not a step.
    """
    for _ in range(tries):
        motor.Send_data_pack_1(Position=None, Speed=None, Current=0)
        end = time.time() + 0.05
        while time.time() < end:
            message, msg_id = comm.receive_can_messages(timeout=0.01)
            if message is None or msg_id is None or msg_id.node_id != node_id:
                continue
            if msg_id.command_id != RESPOND_DATA_PACK_1_ID:
                continue
            motor.UnpackData(message, msg_id)
            if motor.position is not None:
                return float(motor.position), float(motor.speed), float(motor.current)
    return None


# ----------------------------------------------------------------------- excitation

def hold_at_rest(comm, motor, seconds, rate):
    """Command zero velocity for `seconds`, at the normal loop rate.

    Not a bare sleep. Silence longer than the watchdog period latches a watchdog error on
    the driver, which then has to be cleared over CAN before it will move again - so a
    quiet pause between sweep runs would poison every run after the first. Commanding
    zero also brakes the shaft instead of letting it coast into the next capture.
    """
    period = 1.0 / rate
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
        while comm.receive_can_messages(timeout=0)[0] is not None:
            pass                            # drain so the socket cannot fall behind
        time.sleep(period)


def _step(t, amplitude, pre):
    return 0.0 if t < pre else float(amplitude)


def _sine(t, amplitude, pre, frequency):
    """Zero-centred sine setpoint; amplitude is the positive and negative peak."""
    if t < pre:
        return 0.0
    return amplitude * math.sin(2.0 * math.pi * frequency * (t - pre))


def _pwm(t, amplitude, pre, frequency, duty_pct):
    """Pulse setpoint between zero and amplitude; this is not the driver's FOC PWM."""
    if t < pre:
        return 0.0
    phase = ((t - pre) * frequency) % 1.0
    return float(amplitude) if phase < duty_pct / 100.0 else 0.0


def _chirp(t, amplitude, pre, f0, f1, sweep_time):
    """Linear frequency sweep. Instantaneous phase is the integral of the frequency ramp,
    hence the t^2/2 term - differentiate it and you get f0 + (f1-f0)*t/T."""
    if t < pre:
        return 0.0
    tau = min(t - pre, sweep_time)
    phase = 2.0 * math.pi * (f0 * tau + 0.5 * (f1 - f0) / sweep_time * tau * tau)
    return amplitude * math.sin(phase)


# -------------------------------------------------------------------------- capture

def capture(comm, motor, node_id, setpoint_fn, duration, rate, mode, pos_base, ff_current):
    """Drive the setpoint and record. Returns command and response series separately.

    The RX drain is unbounded-until-empty every iteration. A fixed number of reads would
    let the socket fall behind the bus and every sample would age silently - the exact
    failure that once put RTI 50 ms behind reality.
    """
    period = 1.0 / rate
    cmd_t, cmd_u, rx_t, rx_pos, rx_spd, rx_cur = [], [], [], [], [], []

    start = time.perf_counter()
    next_send = start
    while True:
        now = time.perf_counter()
        elapsed = now - start
        if elapsed >= duration:
            break

        u = setpoint_fn(elapsed)
        if mode == "current":
            # A Current-only packet selects pure current / torque mode. Do not include
            # Speed=0, because that would activate the velocity loop and hide this response.
            motor.Send_data_pack_1(Position=None, Speed=None, Current=int(u))
        elif mode == "velocity":
            motor.Send_data_pack_1(Position=None, Speed=int(u), Current=int(ff_current))
        else:
            motor.Send_data_pack_1(Position=int(pos_base + u), Speed=0, Current=int(ff_current))
        cmd_t.append(time.time())       # CLOCK_REALTIME, same epoch as msg.timestamp
        cmd_u.append(u)

        while True:
            message, msg_id = comm.receive_can_messages(timeout=0)
            if message is None:
                break
            if msg_id is None or msg_id.node_id != node_id:
                continue
            if msg_id.command_id != RESPOND_DATA_PACK_1_ID:
                continue
            motor.UnpackData(message, msg_id)
            if motor.position is None:
                continue
            rx_t.append(message.timestamp)      # kernel RX time, not when we got round to it
            rx_pos.append(motor.position)
            rx_spd.append(motor.speed)
            rx_cur.append(motor.current)

        next_send += period
        slack = next_send - time.perf_counter()
        if slack > 3e-4:
            time.sleep(slack - 2e-4)            # coarse sleep, then spin for the last bit
        while time.perf_counter() < next_send:
            pass

    return {
        "cmd_t": np.asarray(cmd_t), "cmd_u": np.asarray(cmd_u, dtype=float),
        "t": np.asarray(rx_t), "pos": np.asarray(rx_pos, dtype=float),
        "spd": np.asarray(rx_spd, dtype=float), "cur": np.asarray(rx_cur, dtype=float),
    }


# ------------------------------------------------------------------------- analysis

def _to_relative(run):
    """Put both series on one clock, t=0 at the first command."""
    if run["cmd_t"].size == 0 or run["t"].size == 0:
        return None
    t0 = run["cmd_t"][0]
    run["cmd_t"] = run["cmd_t"] - t0
    run["t"] = run["t"] - t0
    return run


def _quantum(y):
    """Smallest step the reported signal can take. The driver estimates velocity over a
    fixed window, so its speed channel is coarsely quantised (312.5 ticks/s = a 3.2 ms
    window on PAR6 hardware). Feeding a quantised signal to a high gain is itself a cause
    of limit cycling, so it is worth showing rather than silently averaging away."""
    u = np.unique(y)
    if u.size < 3:
        return 0.0
    gaps = np.diff(u)
    gaps = gaps[gaps > 0]
    return float(np.median(gaps)) if gaps.size else 0.0


def _oscillation(t, y):
    """Ripple amplitude and frequency about the local mean, from zero crossings."""
    if t.size < 20:
        return 0.0, float("nan")
    centred = y - np.mean(y)
    amp = float((np.max(y) - np.min(y)) / 2.0)
    crossings = np.count_nonzero(np.diff(np.sign(centred)) != 0)
    span = t[-1] - t[0]
    freq = crossings / 2.0 / span if span > 0 else float("nan")
    return amp, freq


def _velocity_from_position(t, pos, window_s=0.025):
    """Independent velocity estimate. The speed channel is quantisation-limited, so a
    conclusion drawn from it alone cannot distinguish a real oscillation from a reporting
    artifact - differentiating position settles that question."""
    if t.size < 10:
        return None, None
    dt = np.median(np.diff(t))
    w = max(int(window_s / dt), 2)
    if t.size <= w:
        return None, None
    return t[w:], (pos[w:] - pos[:-w]) / (t[w:] - t[:-w])


def analyze_step(run, mode, pre, amplitude):
    """Dead time, rise, overshoot, settling - measured from the response series.

    Distinguishes a decaying transient from a SUSTAINED oscillation. Reporting "overshoot"
    and "settling time" for a limit cycle is actively misleading: the peak is ripple rather
    than transient overshoot, settling time degenerates to the capture length, and the
    advice that follows ("lower the proportional gain") points at the wrong gain.
    """
    t = run["t"]
    if mode == "velocity":
        y = run["spd"]
    elif mode == "current":
        y = run["cur"]
    else:
        y = run["pos"] - run["pos"][0]
    if t.size < 20:
        return {"error": "too few samples"}

    pre_mask = t < pre
    baseline = float(np.mean(y[pre_mask])) if pre_mask.sum() > 3 else 0.0
    post = t >= pre
    if post.sum() < 10:
        return {"error": "no post-step samples"}

    tail = t >= (t[-1] - 0.15)
    final = float(np.mean(y[tail]))
    delta = final - baseline
    if abs(delta) < 1e-9 or abs(delta) < 0.05 * abs(amplitude):
        return {"error": "motor barely responded - check enable, limits, wiring"}

    yp, tp = y[post], t[post]

    def _first_cross(frac):
        thr = baseline + frac * delta
        hit = np.nonzero((yp >= thr) if delta > 0 else (yp <= thr))[0]
        return float(tp[hit[0]] - pre) if hit.size else float("nan")

    peak = float(np.max(yp) if delta > 0 else np.min(yp))
    overshoot = (peak - final) / delta * 100.0

    band = 0.02 * abs(delta)
    outside = np.nonzero(np.abs(yp - final) > band)[0]
    settling = float(tp[outside[-1]] - pre) if outside.size else 0.0

    # Is the ripple still there at the end? A decaying transient has largely gone by the
    # final third; a limit cycle has not.
    tail_mask = t >= (t[-1] - 0.35 * (t[-1] - pre))
    tail_amp, tail_freq = _oscillation(t[tail_mask], y[tail_mask])
    sustained = abs(delta) > 0 and (tail_amp / abs(delta)) > 0.10

    metrics = {
        "baseline": baseline, "final": final, "delta": delta,
        "dead_time_ms": _first_cross(0.10) * 1e3,
        "rise_10_90_ms": (_first_cross(0.90) - _first_cross(0.10)) * 1e3,
        "overshoot_pct": overshoot if overshoot > 0 else 0.0,
        "settling_2pct_ms": settling * 1e3,
        "steady_state_err": float(amplitude - final),
        "bandwidth_est_hz": (0.35 / ((_first_cross(0.90) - _first_cross(0.10)))
                             if _first_cross(0.90) > _first_cross(0.10) else float("nan")),
        "quantum": _quantum(y),
        "sustained_osc": bool(sustained),
        "ripple_amp": tail_amp,
        "ripple_pct": tail_amp / abs(delta) * 100.0 if delta else 0.0,
        "osc_freq_hz": tail_freq,
    }

    # Cross-check the ripple against position, which is not quantisation-limited.
    if mode == "velocity":
        tv, vp = _velocity_from_position(t, run["pos"])
        if tv is not None:
            m = tv >= (tv[-1] - 0.35 * (tv[-1] - pre))
            if m.sum() > 10:
                amp_pos, _ = _oscillation(tv[m], vp[m])
                metrics["ripple_amp_from_pos"] = amp_pos
                metrics["ripple_pct_from_pos"] = amp_pos / abs(delta) * 100.0 if delta else 0.0
    return metrics


def analyze_chirp(run, mode, pre, rate):
    """Bode from the transfer-function estimate H = Y/U on a uniform grid.

    Only reported where the excitation actually has energy - dividing by a near-zero U
    manufactures enormous gains out of noise.
    """
    grid_dt = 1.0 / rate
    t_end = min(run["cmd_t"][-1], run["t"][-1])
    grid = np.arange(pre, t_end, grid_dt)
    if grid.size < 128:
        return None

    u = np.interp(grid, run["cmd_t"], run["cmd_u"])
    y_src = run["spd"] if mode == "velocity" else (run["cur"] if mode == "current" else run["pos"])
    y = np.interp(grid, run["t"], y_src)
    if mode == "position":
        y = y - y[0]

    u = u - np.mean(u)
    y = y - np.mean(y)
    win = np.hanning(u.size)
    U = np.fft.rfft(u * win)
    Y = np.fft.rfft(y * win)
    f = np.fft.rfftfreq(u.size, grid_dt)

    keep = np.abs(U) > 0.05 * np.max(np.abs(U))
    keep[0] = False
    if keep.sum() < 8:
        return None

    H = Y[keep] / U[keep]
    mag_db = 20.0 * np.log10(np.abs(H))
    phase = np.unwrap(np.angle(H)) * 180.0 / math.pi

    dc = float(np.mean(mag_db[:3]))
    below = np.nonzero(mag_db < dc - 3.0)[0]
    return {
        "f": f[keep], "mag_db": mag_db, "phase_deg": phase,
        "bandwidth_hz": float(f[keep][below[0]]) if below.size else float("nan"),
    }


# -------------------------------------------------------------------------- plotting

def plot_runs(runs, mode, excite, out_path, pre):
    unit = "ticks/s" if mode == "velocity" else ("mA" if mode == "current" else "ticks")
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    for label, run in runs:
        t = run["t"]
        y = run["spd"] if mode == "velocity" else (run["cur"] if mode == "current" else (run["pos"] - run["pos"][0]))
        axes[0].plot(t, y, linewidth=1.0, label=label)
        axes[1].plot(t, run["spd"] if mode == "current" else run["cur"], linewidth=0.8, label=label)

    ref_label, ref = runs[0]
    axes[0].plot(ref["cmd_t"], ref["cmd_u"], "k--", linewidth=1.0, alpha=0.6, label="setpoint")
    axes[0].set_ylabel(f"{mode} [{unit}]")
    axes[0].set_title(f"{mode} loop - {excite} response")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].set_ylabel("speed [ticks/s]" if mode == "current" else "current [mA]")
    axes[1].set_xlabel("time [s]")
    axes[1].axhline(0, color="k", linewidth=0.5)
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_bode(bodes, out_path):
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for label, b in bodes:
        axes[0].semilogx(b["f"], b["mag_db"], linewidth=1.1, label=label)
        axes[1].semilogx(b["f"], b["phase_deg"], linewidth=1.1, label=label)
    axes[0].axhline(-3.0, color="r", linestyle=":", linewidth=1.0, label="-3 dB")
    axes[0].set_ylabel("|H| [dB]")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3, which="both")
    axes[1].axhline(-180.0, color="r", linestyle=":", linewidth=1.0)
    axes[1].set_ylabel("phase [deg]")
    axes[1].set_xlabel("frequency [Hz]")
    axes[1].grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


# ------------------------------------------------------------------------------ main

def format_metrics(metrics, mode=None):
    """Metrics as display lines, with tuning advice for the tested loop."""
    if "error" in metrics:
        return [metrics["error"]]
    out = [
        f"dead time      {metrics['dead_time_ms']:.2f} ms   (CAN round trip + firmware)",
        f"rise 10-90%    {metrics['rise_10_90_ms']:.2f} ms",
        f"mean level     {metrics['final']:.0f}   (steady error {metrics['steady_state_err']:.0f})",
    ]
    if metrics["quantum"] > 0:
        source = "driver's velocity quantum" if mode == "velocity" else "reported feedback increment"
        out.append(f"feedback step  {metrics['quantum']:.1f}   ({source})")
    if metrics["sustained_osc"]:
        out.append(f"SUSTAINED OSCILLATION at {metrics['osc_freq_hz']:.1f} Hz, "
                   f"ripple +-{metrics['ripple_amp']:.0f} ({metrics['ripple_pct']:.0f}% of step)")
        if "ripple_pct_from_pos" in metrics:
            out.append(f"ripple from position (unquantised): "
                       f"+-{metrics['ripple_amp_from_pos']:.0f} "
                       f"({metrics['ripple_pct_from_pos']:.0f}%)")
        out.append("! limit cycle, not overshoot - it never settles.")
        if mode == "current":
            out.append("  Lower KIIQ first; only touch KPIQ if the ripple survives that.")
        elif mode == "position":
            out.append("  Lower KPP first; confirm the velocity loop is stable before retesting.")
        else:
            out.append("  Lower KIV first; only touch KPV if the ripple survives that.")
        if metrics["quantum"] > 0.02 * abs(metrics["delta"]):
            out.append(f"  Feedback quantum is "
                       f"{metrics['quantum'] / abs(metrics['delta']) * 100:.0f}% of the step - "
                       "high gain on a coarse signal limit-cycles by itself.")
    else:
        out.append(f"overshoot      {metrics['overshoot_pct']:.1f} %")
        out.append(f"settling (2%)  {metrics['settling_2pct_ms']:.1f} ms")
        bw = metrics["bandwidth_est_hz"]
        if bw == bw:
            out.append(f"bandwidth ~    {bw:.1f} Hz   (0.35/t_rise, first-order approximation)")
        if metrics["overshoot_pct"] > 25:
            gain = {"current": "KPIQ", "velocity": "KPV", "position": "KPP"}.get(mode, "proportional gain")
            out.append(f"! heavy overshoot - back off {gain}")
        if abs(metrics["steady_state_err"]) > 0.05 * abs(metrics["delta"]):
            if mode == "position":
                out.append("! steady-state error - check load/friction, then raise KPP cautiously")
            else:
                gain = "KIIQ" if mode == "current" else "KIV"
                out.append(f"! steady-state error - raise {gain}")
    return out


def _report(label, metrics, mode):
    print(f"\n  --- {label} ---")
    for line in format_metrics(metrics, mode):
        print(f"    {line}")

def main():
    ap = argparse.ArgumentParser(
        description="Current, velocity, and position loop tuner. ONE motor, id 0, mounted, shaft free.")
    ap.add_argument("--mode", choices=("current", "velocity", "position"), required=True)
    ap.add_argument("--excite", choices=("step", "sine", "pwm", "chirp"), default="step")
    ap.add_argument("--node", type=int, default=0)
    ap.add_argument("--rate", type=float, default=1000.0, help="sample/command rate [Hz]")
    ap.add_argument("--amplitude", type=float, default=None,
                    help="current: mA (default 500). velocity: ticks/s (default 8000). position: ticks (default 2000)")
    ap.add_argument("--duration", type=float, default=1.2, help="total capture [s]")
    ap.add_argument("--pre", type=float, default=0.2, help="quiet time before excitation [s]")
    ap.add_argument("--f0", type=float, default=1.0, help="chirp start [Hz]")
    ap.add_argument("--f1", type=float, default=200.0, help="chirp end [Hz]")
    ap.add_argument("--frequency", type=float, default=5.0, help="sine/PWM frequency [Hz]")
    ap.add_argument("--duty", type=float, default=50.0, help="PWM high time [%%]")
    ap.add_argument("--kpv", type=float, default=None)
    ap.add_argument("--kiv", type=float, default=None)
    ap.add_argument("--kpp", type=float, default=None)
    ap.add_argument("--kpiq", type=float, default=None, help="current-loop Iq proportional gain")
    ap.add_argument("--kiiq", type=float, default=None, help="current-loop Iq integral gain")
    ap.add_argument("--sweep", type=str, default=None, help="comma list of gain values")
    ap.add_argument("--sweep-param", choices=("kpv", "kiv", "kpp", "kpiq", "kiiq"), default=None,
                    help="which gain --sweep varies (default kpiq in current, kpv in velocity, kpp in position)")
    ap.add_argument("--current-limit", type=float, default=1500.0, help="[mA]")
    ap.add_argument("--velocity-limit", type=float, default=40000.0, help="[ticks/s]")
    ap.add_argument("--ff-current", type=float, default=0.0, help="feedforward current [mA]")
    # int, not float: Send_Watchdog packs this with a bitwise mask, which rejects floats.
    ap.add_argument("--watchdog", type=int, default=5000,
                    help="driver watchdog, project default 5000; 0 disables (not advised)")
    ap.add_argument("--label", type=str, default=None)
    ap.add_argument("--yes", action="store_true", help="skip the motion confirmation prompt")
    args = ap.parse_args()

    if args.excite in ("sine", "pwm") and args.frequency <= 0:
        ap.error("--frequency must be > 0 for sine/PWM excitation")
    if args.excite == "pwm" and not 0.0 <= args.duty <= 100.0:
        ap.error("--duty must be between 0 and 100")

    if args.amplitude is None:
        args.amplitude = 500.0 if args.mode == "current" else (8000.0 if args.mode == "velocity" else 2000.0)

    print("\n  THE MOTOR WILL MOVE. Mounted? Shaft clear? Nothing attached to be thrown?")
    if not args.yes:
        if input("  type 'go' to continue: ").strip().lower() != "go":
            sys.exit("  aborted")

    preflight(args.node)
    comm = Spectral.CanCommunication(bustype="socketcan", channel="can0", bitrate=1000000)
    motor = Spectral.SpectralCAN(node_id=args.node, communication=comm)

    sweep_vals = [float(v) for v in args.sweep.split(",")] if args.sweep else [None]
    runs, bodes, summary = [], [], []

    # Everything past bus creation is inside the guard, including setup. A failure while
    # arming the watchdog or setting limits still has to idle the motor and close the bus.
    try:
        survey_bus(comm, args.node, seconds=0.5)
        state = read_state(comm, motor, args.node)
        if state is None:
            sys.exit(f"  node {args.node} did not answer - check power, wiring and CAN id")
        print(f"  node {args.node} responding: "
              f"pos={state[0]:.0f} spd={state[1]:.0f} cur={state[2]:.0f}")

        # Watchdog first: if this process dies mid-spin the driver idles on its own rather
        # than holding the last commanded velocity forever. 5000 matches the project
        # default in hardware/can_hardware.py; anything near the inter-run rest would trip
        # on its own.
        # A zero timeout is an explicit firmware disable command, not "skip setup".
        motor.Send_Watchdog(int(args.watchdog), "Idle")
        if args.watchdog > 0:
            print(f"  watchdog armed: {args.watchdog} (Idle on timeout)")
        else:
            print("  WARNING: watchdog explicitly disabled - a crash mid-spin leaves the motor running")
        motor.Send_Limits(args.velocity_limit, args.current_limit)
        time.sleep(0.05)

        for val in sweep_vals:
            # Clear anything latched from a previous run or a prior session. A latched
            # watchdog error blocks motion, and the symptom is a flat trace that reads as
            # "gains too low" rather than "driver is refusing to move".
            motor.Send_Clear_Error()
            time.sleep(0.05)

            kpiq, kiiq = args.kpiq, args.kiiq
            kpv, kiv, kpp = args.kpv, args.kiv, args.kpp
            swept = args.sweep_param or {"current": "kpiq", "velocity": "kpv", "position": "kpp"}[args.mode]
            if val is not None:
                if swept == "kpiq":
                    kpiq = val
                elif swept == "kiiq":
                    kiiq = val
                elif swept == "kpv":
                    kpv = val
                elif swept == "kiv":
                    kiv = val
                else:
                    kpp = val
            if kpiq is not None or kiiq is not None:
                motor.Send_Current_Gains(kpiq if kpiq is not None else 0.012,
                                         kiiq if kiiq is not None else 0.012)
            if kpv is not None or kiv is not None:
                motor.Send_Velocity_Gains(kpv if kpv is not None else 0.012,
                                          kiv if kiv is not None else 0.012)
            if kpp is not None:
                motor.Send_Position_Gains(kpp)
            time.sleep(0.1)

            gain_txt = f"{swept.upper()}={val:g}" if val is not None else "as-configured"

            # Position steps must be relative to where the shaft actually is - an absolute
            # target would slam it across whatever distance happens to separate them.
            pos_base = 0.0
            if args.mode == "position":
                state = read_state(comm, motor, args.node)
                if state is None:
                    print("    could not read shaft position - skipping (will NOT guess)")
                    continue
                pos_base = state[0]
            if args.excite == "step":
                fn = lambda t: _step(t, args.amplitude, args.pre)
            elif args.excite == "sine":
                fn = lambda t: _sine(t, args.amplitude, args.pre, args.frequency)
            elif args.excite == "pwm":
                fn = lambda t: _pwm(t, args.amplitude, args.pre, args.frequency, args.duty)
            else:
                sweep_time = max(args.duration - args.pre, 0.1)
                fn = lambda t: _chirp(t, args.amplitude, args.pre, args.f0, args.f1, sweep_time)

            print(f"\n  running: {gain_txt}")
            run = capture(comm, motor, args.node, fn, args.duration, args.rate,
                          args.mode, pos_base, args.ff_current)

            hold_at_rest(comm, motor, 0.4, args.rate)
            run = _to_relative(run)
            if run is None:
                print("    no data captured")
                continue
            achieved = run["t"].size / args.duration
            print(f"    {run['t'].size} samples, {achieved:.0f} Hz achieved")
            if achieved < 0.6 * args.rate:
                print("    ! sample rate well under target - results are coarse")

            runs.append((gain_txt, run))
            if args.excite == "step":
                m = analyze_step(run, args.mode, args.pre, args.amplitude)
                _report(gain_txt, m, args.mode)
                summary.append({"label": gain_txt, **{k: v for k, v in m.items()}})
            elif args.excite == "chirp":
                b = analyze_chirp(run, args.mode, args.pre, args.rate)
                if b:
                    bodes.append((gain_txt, b))
                    print(f"    -3 dB bandwidth {b['bandwidth_hz']:.1f} Hz")
                    summary.append({"label": gain_txt, "bandwidth_hz": b["bandwidth_hz"]})
            else:
                print("    visual excitation - inspect the trace; no step/chirp metrics")
                summary.append({"label": gain_txt, "excitation": args.excite,
                                "frequency_hz": args.frequency})
    finally:
        motor.Send_Idle()
        try:
            comm.bus.shutdown()             # otherwise python-can complains on exit
        except Exception:
            pass
        print("\n  motor idled, bus closed")

    if not runs:
        sys.exit("  nothing captured")

    RESULTS.mkdir(parents=True, exist_ok=True)
    stamp = args.label or f"{args.mode}_{args.excite}_{int(time.time())}"
    png = RESULTS / f"{stamp}.png"
    plot_runs(runs, args.mode, args.excite, png, args.pre)
    print(f"  plot  -> {png}")
    if bodes:
        bode_png = RESULTS / f"{stamp}_bode.png"
        plot_bode(bodes, bode_png)
        print(f"  bode  -> {bode_png}")

    np.savez_compressed(RESULTS / f"{stamp}.npz",
                        **{f"{lbl}__{k}": v for lbl, r in runs for k, v in r.items()})
    (RESULTS / f"{stamp}.json").write_text(json.dumps(summary, indent=2))
    print(f"  data  -> {RESULTS / (stamp + '.npz')}")


if __name__ == "__main__":
    main()
