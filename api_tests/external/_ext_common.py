#!/usr/bin/env python3
"""Shared helpers for the OFF-ROBOT SOFT_RTI test suite.

Everything here exists because these tests run on a SEPARATE MACHINE, which changes two
things fundamentally compared with api_tests/_servo_common.py:

1. **THERE IS NO /dev/shm.** Every metric the on-robot tests read from shared memory has to
   come from the robot's state stream instead. That is why `external_test` exists as a
   recipe: no shipped one carried j_p AND j_t AND rti_seq AND rti_state together.

2. **THE STREAM IS BETTER DATA THAN SHARED MEMORY WAS.** A state packet is built and sent
   INSIDE the robot's tick, so its contents are self-consistent and it carries its own tick
   number — an atomic (tick, data) pair. Shared memory cannot provide that: `loop_tick`
   advances at RTI.py:1351 and the commanded position is written at :1680, so an off-robot
   sampler reading both gets a mismatched pair about a third of the time. That race
   (servo_mode_options.md 15.4) is what voided test 39's ripple numbers. **Per-tick
   differencing is valid here and invalid on the robot**, which is the opposite of what you
   would guess.

The two rules from _servo_common still apply and are, if anything, sharper over a network:

  * **NEVER GO SILENT WHILE CLAIMED.** A claimed session has no idle state. Sleeping,
    printing, or analysing between phases exceeds the command timeout and the watchdog
    correctly stops the arm. Analysis happens AFTER release, always — that mistake latched
    RTI_LINK_LOST on 2026-08-21 and disabled the robot mid-test.
  * **PACING IS STICKY.** vel_scale, pace and min_duration persist until changed, so every
    config must write every knob or it inherits the previous one's.
"""

import math
import time

from robotclient.rti_client import parse_state_packet

LOOP_HZ = 250.0
DT = 1.0 / LOOP_HZ

# The recipe these tests need. Robot-side file: data_recipe/external_test.xml
RECIPE = "external_test"

# J2 (shoulder) and J3 (elbow) carry the gravity load and are EXCLUDED. J3 additionally has
# a known velocity-loop oscillation unrelated to servo mode that would confound any torque
# measurement taken near it (servo_mode_options.md 14.6).
SAFE_JOINTS = (0, 3, 4, 5)

CONFIGS = {
    "advancing":         dict(gen="ruckig_advancing"),
    "advancing+mindur2": dict(gen="ruckig_advancing", min_duration=2.0),
    "ruckig":            dict(gen="ruckig"),
    "tracker g8":        dict(gen="tracker", gain=8.0),
    "tracker g15":       dict(gen="tracker", gain=15.0),
}

JN = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6"}


def deg(r):
    return r * 180.0 / math.pi


def tri(t, amp, period):
    """0 -> +amp -> -amp -> 0. Starts AND ends at zero offset.

    Not cosmetic: a shape starting at -amp makes the first command of every block order the
    arm a full amplitude away from where it stands, and it snaps there at the acceleration
    limit. A test measuring smoothness must not open with a step.
    """
    ph = (t / period) % 1.0
    if ph < 0.25:
        return amp * (4.0 * ph)
    if ph < 0.75:
        return amp * (2.0 - 4.0 * ph)
    return amp * (4.0 * ph - 4.0)


def apply_config(robot, cfg):
    """Write EVERY knob, defaults included — pacing is sticky."""
    robot.rti_set_vel_scale(cfg.get("vel_scale", 1.0))
    robot.rti_set_pace(cfg.get("pace", 0.0))
    robot.rti_set_min_duration(cfg.get("min_duration", 0.0))
    robot.rti_set_lowpass(cfg.get("lowpass", 0.0))
    robot.rti_set_servo_generator(cfg["gen"], gain=cfg.get("gain"),
                                  lookahead=cfg.get("lookahead"),
                                  self_lookahead=cfg.get("self_lookahead", 0.0))


def clear_config(robot):
    robot.rti_set_vel_scale(1.0)
    robot.rti_set_pace(0.0)
    robot.rti_set_min_duration(0.0)
    robot.rti_set_lowpass(0.0)
    # The state rate is sticky too, and forgetting it is not cosmetic: a leftover 25 Hz
    # throttle from a previous run makes the next test read a 10-tick gap between state
    # packets and report it as 90% PACKET LOSS. Found exactly that way on 2026-08-21.
    try:
        robot.rti_set_state_rate(0.0)
    except Exception:
        pass          # older robot without the command — nothing to clear


# ---------------------------------------------------------------------------------------
# THE CLOCK. Found 2026-09-30 from the first PC runs: every send interval printed as a
# multiple of 15.6 ms (p50 16.0 / p99 47.0), where the same suite on the robot printed
# 20.0 / 20.4 / 20.7. Cause: on Windows, Python <= 3.12's time.monotonic() is GetTickCount64,
# which only ADVANCES every 15.625 ms. This module read it for the path time, the send
# schedule and the packet stamps, so the sender computed each target from a staircase clock
# and slept against a stale "now" — bursts of catch-up sends. An offline run of the real
# generator (scratchpad sim_clock.py, checked against the measured intervals) shows what that
# does to ruckig_advancing, which derives target velocity from consecutive targets: commanded
# error 0.08 -> 0.27-0.45 deg, commanded-velocity ripple 0.06 -> ~0.29 rad/s (as large as the
# 0.26 rad/s path speed), and the size depends on how the clock ticks line up with the start —
# i.e. it varies run to run. perf_counter is QueryPerformanceCounter on Windows (sub-us) and
# CLOCK_MONOTONIC on Linux, so nothing changes on the robot.
now = time.perf_counter


def _sleep_granularity():
    """How long time.sleep(1 ms) really takes here. ~1 ms on Linux and Python 3.11+ Windows;
    ~15.6 ms on Windows before 3.11 (the default system timer)."""
    worst = 0.0
    for _ in range(3):
        t = time.perf_counter()
        time.sleep(0.001)
        worst = max(worst, time.perf_counter() - t)
    return worst


# Busy-wait for this long before each deadline. 0 where sleep() is accurate (the robot); on a
# coarse-timer Windows it is long enough to cover the overshoot. Costs one core, briefly, in a
# TEST sender — acceptable; a stalled or bursty sender is not.
_SPIN_S = 0.0
_g = _sleep_granularity()
if _g > 0.003:
    _SPIN_S = min(0.020, 1.5 * _g)


def sleep_until(deadline, spin=None):
    """Return at `deadline` (a perf_counter time), however coarse time.sleep() is."""
    spin = _SPIN_S if spin is None else spin
    while True:
        rem = deadline - now()
        if rem <= 0:
            return
        if rem > spin:
            time.sleep(rem - spin)
        # else: spin the last stretch


def clock_report():
    """One line for the top of a run, and a warning if the OS clock is coarse."""
    mono = time.get_clock_info("monotonic").resolution
    perf = time.get_clock_info("perf_counter").resolution
    line = (f"clock: perf_counter {perf * 1e6:.2g} us; sleep granularity "
            f"{_g * 1e3:.1f} ms" + (f" (spin-waiting last {_SPIN_S * 1e3:.0f} ms)"
                                     if _SPIN_S else ""))
    if mono > 0.002:
        line += (f"\n  note: time.monotonic() here ticks every {mono * 1e3:.1f} ms — the suite "
                 f"does not use it for timing")
    return line


def keep_alive(rti, q, rate_hz, seconds):
    """Stream `q` for `seconds`. The ONLY correct way to wait while claimed."""
    period = 1.0 / rate_hz
    end, nxt = now() + seconds, now()
    while now() < end:
        rti.send_joint_position(q)
        rti.read_state()          # also keeps the TCP socket drained
        nxt += period
        sleep_until(nxt)


def stream(rti, q0, joints, amp, period, rate_hz, seconds, rng=None, lo=None, hi=None):
    """Stream the triangle and CAPTURE every state packet. Returns (rows, t0, cmd_hz).

    `rng`/`lo`/`hi` make the send interval wander, simulating a real VR or haptic sender
    that cannot hold a fixed rate. lo==hi (or rng=None) is a metronome.
    """
    rti.capture(True)
    sent = 0
    # (seq, PC send time) for every command, so command_timing() can measure the UPLINK — the
    # state stream's tick gap only proves the robot->PC direction was clean.
    rti.sent_log = sent_log = []
    t0 = nxt = now()
    while True:
        t = now() - t0
        if t >= seconds:
            break
        q = list(q0)
        off = tri(t, amp, period)
        for j in joints:
            q[j] = q0[j] + off
        rti.send_joint_position(q)
        sent_log.append((rti.seq, now()))
        rti.read_state()                    # drains -> capture appends every packet
        sent += 1
        nxt += (1.0 / rng.uniform(lo, hi)) if rng else (1.0 / rate_hz)
        sleep_until(nxt)
    # Walk any leftover offset back to exactly zero so the caller's hold does not start with
    # a step either.
    off = tri(now() - t0, amp, period)
    steps = int(abs(off) / max(4.0 * amp / period * (1.0 / rate_hz), 1e-9)) + 1
    for i in range(steps - 1, -1, -1):
        q = list(q0)
        for j in joints:
            q[j] = q0[j] + off * i / steps
        rti.send_joint_position(q)
        rti.read_state()
        sleep_until(now() + 1.0 / rate_hz)
    el = now() - t0
    rti.capture(False)
    return list(rti.captured), t0, sent / el


def retime(rows):
    """Replace drain-time stamps with times rebuilt from the robot's own tick.

    THE FLAW THIS REMOVES. A packet is stamped when the PC DRAINS the socket, not when the
    robot made it. The robot emits every 4 ms; a sender at 50 Hz drains every 20 ms, so five
    packets share (almost) one stamp — the oldest is up to 16 ms stale. The accuracy metric
    compares each measured position with the analytic path AT ITS STAMP, and a stale stamp
    reads as error: v x dt = 15 deg/s x ~10 ms is ~0.15 deg on average, up to 0.24 deg. On the
    robot this cannot happen — the sampler stamps at the instant it reads.

    HOW. The tick is the robot's clock and packets are unique in it, so time is a straight
    line in tick: t = a + b*tick. Fit it through the LAST packet of each drain (the
    freshest — delayed by at most one packet interval, where the first is delayed by a whole
    drain period). The slope is FITTED rather than assumed to be 4.000 ms: the observed rate
    was 249.2 pkt/s (4.013 ms), and assuming 4.000 would drift 50 ms over a 16 s block.

    Falls back to the raw stamps — never invents times — when there is too little data or the
    fit is implausible (slope not within 5% of the loop period).
    """
    pts = [(ts, s["tick"]) for ts, s in rows if isinstance(s.get("tick"), (int, float))]
    if len(pts) < 50:
        return rows
    # A drain's packets arrive back-to-back (microseconds apart); separate drains are >= a
    # send period apart. 1 ms sits between the two.
    last = [(tk, ts) for i, (ts, tk) in enumerate(pts)
            if i + 1 == len(pts) or pts[i + 1][0] - ts > 0.001]
    if len(last) < 20:
        return rows
    n = len(last)
    mx = sum(x for x, _ in last) / n
    my = sum(y for _, y in last) / n
    sxx = sum((x - mx) ** 2 for x, _ in last)
    if sxx == 0:
        return rows
    b = sum((x - mx) * (y - my) for x, y in last) / sxx
    if not (0.95 * DT < b < 1.05 * DT):
        return rows
    a = my - b * mx
    return [((a + b * s["tick"]) if isinstance(s.get("tick"), (int, float)) else ts, s)
            for ts, s in rows]


def parse_raw(captured):
    """[(arrival_time, payload_bytes)] -> [(arrival_time, {tag: value})]. Skips junk."""
    rows = []
    for ts, data in captured:
        try:
            rows.append((ts, parse_state_packet(data.decode())))
        except Exception:
            continue
    return rows


def parse(captured):
    """Like parse_raw, but with times rebuilt from the robot's tick — see retime()."""
    return retime(parse_raw(captured))


GLITCH_JUMP_DEG = 3.0


def deglitch(rows, jump_deg=GLITCH_JUMP_DEG, half=3):
    """Drop packets whose j_p is impossible, and say how many. Returns (rows, n_dropped).

    WHY. The robot's own `motor_position` occasionally flips to a different set of six numbers
    for exactly one loop period (api_tests/43_glitch_hunt.py, TODO/position_glitch.md), and a
    fraction of those reach `joint_position` and so the state packet: 9 packets in 6 runs of 16 s,
    each 10-99 deg off on several joints, with perfectly smooth neighbours. A handful of such
    packets is enough to make tracking error read 1-3 deg on an arm that tracked at 0.4-0.5.

    HOW. An arm cannot move `jump_deg` in 4 ms — at 15 deg/s it moves 0.06 deg — so a packet
    whose joint reading differs from the MEDIAN of its neighbours by more than that is not the
    arm. The median (not the mean, and not a fixed error threshold) is what keeps this honest:
    it does not depend on how well the arm tracks, so ruckig/tracker, whose REAL error is 4 deg,
    are not trimmed, and it survives up to `half` consecutive bad packets.

    This removes a robot bug from the measurement; it does not fix the robot.
    """
    n = len(rows)
    if n < 2 * half + 3:
        return rows, 0
    thr = math.radians(jump_deg)
    keep = []
    dropped = 0
    for i, (ts, st) in enumerate(rows):
        jp = st.get("j_p")
        if not isinstance(jp, list):
            keep.append((ts, st))
            continue
        lo, hi = max(0, i - half), min(n, i + half + 1)
        bad = False
        for k in range(len(jp)):
            nb = sorted(r[1]["j_p"][k] for j, r in enumerate(rows[lo:hi], start=lo)
                        if j != i and isinstance(r[1].get("j_p"), list))
            if nb and abs(jp[k] - nb[len(nb) // 2]) > thr:
                bad = True
                break
        if bad:
            dropped += 1
        else:
            keep.append((ts, st))
    return keep, dropped


def window(rows, t0):
    """The rows a run is JUDGED on: from the moment the stream started, with a usable j_p.

    ONE definition, used by measure() AND anatomy(). They used different sample sets (all
    packets vs every 4th packet after the start) and disagreed by up to 3x on the same run —
    e.g. 0.47 vs 1.447 deg — which made the breakdown useless as an explanation. If the error
    is concentrated in a few samples, any difference in which samples are used shows.
    """
    return [(ts, s) for ts, s in rows
            if ts >= t0 and isinstance(s.get("j_p"), list)
            and isinstance(s.get("j_t"), list)]


def _pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else float("nan")


def command_timing(rows, sent_log):
    """The UPLINK, from data the run already has: how regularly the PC sent, and how long each
    command took to be reflected in a state packet.

    WHY. `tick gap 1` says the robot->PC state stream was clean. It says NOTHING about the
    other direction, and the generator that matters most (`ruckig_advancing`) is driven by
    the ARRIVAL TIMING of commands. A stalled sender or a delayed packet would leave every
    state packet intact and still show up as tracking error. rms error is dominated by a few
    spikes, so a handful of stalls in 16 s can double it.

    send_dt_ms   interval between consecutive sends. A stall is a spike here, on the PC.
    lat_ms       send -> first state packet whose `rti_seq` echo has reached that command,
                 on the PC clock (states are retimed from the tick). Includes the network
                 both ways, up to one tick of alignment and the robot's processing. What
                 matters is not its level but its SPREAD: a late packet is a spike here.

    Returns {"send_dt": (p50, p99, max), "lat": (p50, p99, max), "n": commands} in ms, or
    None when there is nothing to measure (no rti_seq in the stream).
    """
    sends = sorted(sent_log)
    pk = [(t, s["rti_seq"]) for t, s in rows if isinstance(s.get("rti_seq"), (int, float))]
    if len(sends) < 20 or len(pk) < 20:
        return None
    dts = [(b[1] - a[1]) * 1e3 for a, b in zip(sends, sends[1:])]
    lat, j = [], 0
    for seq, ts in sends:
        while j < len(pk) and pk[j][1] < seq:
            j += 1
        if j >= len(pk):
            break
        lat.append((pk[j][0] - ts) * 1e3)
    if len(lat) < 20:
        return None
    return {"send_dt": (_pct(dts, .5), _pct(dts, .99), max(dts)),
            "lat": (_pct(lat, .5), _pct(lat, .99), max(lat)), "n": len(sends)}


def ready_or_recover(robot, recover=False):
    """Is the robot fit to start the next transport? Returns (ok, message).

    A transport that ends CONTROL LOST leaves the robot DISABLED in an error hold. e2 then ran the
    NEXT transport on it and reported a raw `CommandRejected: robot is DISABLED` — indistinguishable
    from that transport failing. (Seen 2026-09-30: UDP tripped the watchdog, TCP "failed" on a
    disabled robot, and the TCP half of three burst-loss levels measured nothing.)
    With recover=True it clears errors and enables the arm — which ENABLES THE MOTORS, so opt-in.
    """
    def fine(st):
        return (st.get("state") == "ENABLED" and not st.get("errors")
                and st.get("mode") != "ACTIVE_ERROR")
    st = robot.status()
    if fine(st):
        return True, ""
    if not recover:
        return False, (f"SKIPPED — the robot is {st.get('state')} / {st.get('mode')} "
                       f"(errors {st.get('errors') or 'none'}) after the previous transport's stop. "
                       f"Clear Errors + Enable, or rerun with --recover.")
    try:
        robot.clear_errors(wait=True)
        robot.enable(wait=True)
        time.sleep(0.5)
    except Exception as exc:
        return False, f"SKIPPED — automatic recovery failed: {exc!r}"
    st = robot.status()
    return (True, "") if fine(st) else (False, f"SKIPPED — still {st.get('state')} / {st.get('mode')} after recovery")


def stream_health(rows, t0, seconds, min_cover=0.9):
    """Did the ROBOT keep driving for the whole block? Returns (ok, message).

    THE TRAP. When SOFT_RTI's watchdog fires (3 command intervals without a setpoint) the robot
    stops the arm, disables itself and stops streaming state — and NOTHING is raised on the PC:
    the socket stays open and read_state() just returns nothing. e2 then analysed the first few
    seconds and printed a normal-looking table. Found 2026-09-30: TCP under 3% loss + 8 ms jitter
    delivered 875 of ~3990 state packets; the robot log said "No command for 16 ticks (limit 15) —
    stopping, latching RTI_LINK_LOST" 3.5 s in, and e2 reported trk_err 0.681 as if nothing had
    happened. A block that did not run to the end is a FAILURE of that transport under those
    conditions — and the most important result of the test — not a data point.

    Two independent signals, either is enough:
      * coverage — the last packet's time vs the block length
      * rti_state — the packets carry the robot's sub-state; 4 = stopping_error
    """
    if not rows:
        return False, "no state packets at all"
    last_t = rows[-1][0] - t0
    for ts, s_ in rows:
        if s_.get("rti_state") == 4:
            return False, (f"the robot STOPPED itself {ts - t0:.1f} s into the block "
                           f"(rti_state = stopping_error: RTI_LINK_LOST) — control was lost")
    if last_t < seconds * min_cover:
        return False, (f"state stream ended {last_t:.1f} s into a {seconds:g} s block "
                       f"({len(rows)} packets) — the robot stopped driving (link lost)")
    return True, ""


def loss_counts(rows, sent_log):
    """What the link actually dropped, counted from the data — not inferred from a median.

    state_lost  packets missing from the robot's tick sequence (robot -> PC). A median tick gap
                of 1 is COMPATIBLE with 3% loss, which is how an earlier version of e2 printed
                "no packet loss" under injected loss.
    cmd_skipped commands whose sequence number the robot never echoed as the latest accepted
                (PC -> robot): lost in transit OR overtaken (two arriving inside one 4 ms tick,
                where only the newest is used). An upper bound on command loss.
    Returns dict(state_lost, state_span, cmd_skipped, cmd_sent) or None.
    """
    ticks = [s["tick"] for _, s in rows if isinstance(s.get("tick"), (int, float))]
    seqs = [s["rti_seq"] for _, s in rows if isinstance(s.get("rti_seq"), (int, float))]
    if len(ticks) < 50 or len(seqs) < 50:
        return None
    span = int(ticks[-1] - ticks[0]) + 1
    skipped = sum(max(0, int(b - a) - 1) for a, b in zip(seqs, seqs[1:]) if b > a)
    return dict(state_lost=max(0, span - len(ticks)), state_span=span,
                cmd_skipped=skipped, cmd_sent=len(sent_log))


def fmt_timing(ct):
    if ct is None:
        return "no command timing (stream carries no rti_seq)"
    d, l = ct["send_dt"], ct["lat"]
    return (f"send interval p50/p99/max {d[0]:.1f}/{d[1]:.1f}/{d[2]:.1f} ms | "
            f"command->state latency p50/p99/max {l[0]:.1f}/{l[1]:.1f}/{l[2]:.1f} ms")


def measure(rows, t0, joints, q0, amp, period):
    """Per-joint (t_rms Nm, trk_err deg). NEVER call while a claim is held.

    Same two metrics the on-robot tests use, so the numbers are directly comparable:
      trk_err  rms |measured - the analytic path the PC meant| — total system accuracy
      t_rms    high-frequency joint torque — the vibration an operator hears
    Neither differentiates, so both are robust to a dropped sample.
    """
    good = window(rows, t0)
    if len(good) < 200:
        return None
    out, w = {}, 13
    for j in joints:
        meas = [s["j_p"][j] for _, s in good]
        trq = [s["j_t"][j] for _, s in good]
        err = [meas[i] - (q0[j] + tri(good[i][0] - t0, amp, period))
               for i in range(len(good))]
        trk = deg(math.sqrt(sum(e * e for e in err) / len(err)))
        if len(trq) > w * 3:
            hf = [trq[i] - sum(trq[i - w:i + w + 1]) / (2 * w + 1)
                  for i in range(w, len(trq) - w)]
            trms = math.sqrt(sum(x * x for x in hf) / len(hf))
        else:
            trms = float("nan")
        out[j] = (trms, trk)
    return out


def anatomy(rows, t0, joints, q0, amp, period, lag_stride=4):
    """WHERE does the tracking error come from? Per joint:

      rms / bias    total rms and the signed mean (a constant offset)
      early / rest  rms over the first 2.5 s vs the rest (a start-up transient)
      lag_ms, res   the best time shift d such that measured(t) ~ path(t - d), and the rms
                    LEFT after that shift. If a big error collapses at some d, the arm is
                    simply LATE by d ms (15 deg/s x d); if it does not, the trouble is shape.
      peak, at      the single worst |error| and when it happened
      top1          the share of ALL squared error carried by the worst 1% of samples. Near
                    100% means a few glitched samples, NOT a tracking failure: rms is
                    dominated by them, and lag/bias/early/rest then mean nothing.

    Uses window() — the SAME samples as measure(), all of them. (The first version subsampled
    every 4th packet for speed and disagreed with measure() by up to 3x on identical runs.)
    Only the lag SEARCH is subsampled: it is smooth, and costs 100 passes.
    NEVER call while a claim is held.
    """
    good = window(rows, t0)
    if len(good) < 100:
        return None
    ts = [t - t0 for t, _ in good]
    out = {}
    rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v)) if v else float("nan")
    for j in joints:
        m = [s["j_p"][j] for _, s in good]
        base = q0[j]
        err = [m[i] - (base + tri(ts[i], amp, period)) for i in range(len(ts))]
        early = [e for e, t in zip(err, ts) if t < 2.5]
        rest = [e for e, t in zip(err, ts) if t >= 2.5]
        sq = sorted((e * e for e in err), reverse=True)
        tot = sum(sq) or 1e-30
        top1 = sum(sq[:max(1, len(sq) // 100)]) / tot
        k_pk = max(range(len(err)), key=lambda i: abs(err[i]))
        sub_t, sub_m = ts[::lag_stride], m[::lag_stride]
        best = (float("inf"), 0.0)
        for k in range(-10, 91):                 # -50 ms .. +450 ms in 5 ms steps
            d = k * 0.005
            r = rms([sub_m[i] - (base + tri(sub_t[i] - d, amp, period)) for i in range(len(sub_t))])
            if r < best[0]:
                best = (r, d)
        out[j] = dict(rms=deg(rms(err)), bias=deg(sum(err) / len(err)),
                      early=deg(rms(early)), rest=deg(rms(rest)),
                      lag_ms=best[1] * 1e3, resid=deg(best[0]),
                      peak=deg(abs(err[k_pk])), peak_t=ts[k_pk], top1=top1, n=len(err))
    return out


def fmt_anatomy(an, joints):
    if an is None:
        return "  (too few samples for an error breakdown)"
    rows = []
    for j in joints:
        a = an[j]
        rows.append(f"    {JN[j]}: rms {a['rms']:.2f} = bias {a['bias']:+.2f} | first 2.5 s "
                    f"{a['early']:.2f} / rest {a['rest']:.2f} | best lag {a['lag_ms']:.0f} ms "
                    f"-> residual {a['resid']:.2f} | peak {a['peak']:.1f} @ {a['peak_t']:.2f} s, "
                    f"worst 1% carries {a['top1'] * 100:.0f}% of the error")
    return "\n".join(rows)


def dump_csv(path, raw_rows, rows, t0, q0, amp, period, joints):
    """Write every packet of the run, RAW arrival time and retimed time side by side, so a
    bad run can be examined sample by sample instead of argued about from summary numbers."""
    by_id = {id(s): ts for ts, s in raw_rows}
    with open(path, "w") as f:
        f.write(f"# t0={t0!r} amp={amp!r} period={period!r} joints={list(joints)} q0={list(q0)}\n")
        f.write("arrival_s,t_s,tick,rti_seq," + ",".join(f"jp{i}" for i in range(6)) + ","
                + ",".join(f"jt{i}" for i in range(6)) + "\n")
        for ts, s in rows:
            jp, jt = s.get("j_p"), s.get("j_t")
            if not (isinstance(jp, list) and isinstance(jt, list)):
                continue
            arr = by_id.get(id(s), float("nan"))
            f.write(f"{arr - t0:.6f},{ts - t0:.6f},{s.get('tick', '')},{s.get('rti_seq', '')},"
                    + ",".join(repr(v) for v in jp) + "," + ",".join(repr(v) for v in jt) + "\n")


def state_rate(rows):
    """Packets per second actually received, from the ARRIVAL times."""
    if len(rows) < 2:
        return float("nan")
    return (len(rows) - 1) / max(rows[-1][0] - rows[0][0], 1e-9)


def tick_gaps(rows):
    """(median, max) gap in robot ticks between consecutive packets.

    A gap of 1 means nothing was lost. This is the closest thing to a LOSS measurement the
    suite has, and it only means anything off-robot — the tick comes from inside the packet,
    so a missing tick is a missing packet, full stop.
    """
    ticks = [s.get("tick") for _, s in rows if isinstance(s.get("tick"), (int, float))]
    d = [b - a for a, b in zip(ticks, ticks[1:]) if b > a]
    if not d:
        return float("nan"), float("nan")
    d.sort()
    return d[len(d) // 2], d[-1]


def wait_for_status(robot, timeout=10.0):
    """Block until the robot has pushed at least one real status event.

    NOT optional off-robot. `RobotClient.status()` returns the latest PUSHED event, not a
    request/response — so immediately after connect() it is an EMPTY DICT and every field
    reads None. On loopback the first event lands before anyone looks and the race is
    invisible; across a network it is not, and the first thing a remote PC sees is
    "robot is None, need ENABLED" from a perfectly healthy robot.

    Returns None on success, or a message.
    """
    try:
        robot.wait(lambda s: s.get("mode") is not None, timeout, "first status event")
    except Exception as exc:
        return (f"connected, but no status event arrived within {timeout:g} s ({exc!r}). "
                f"The TCP session is up — the robot is not publishing status.")
    return None


def preflight(robot, need_enabled=True):
    """Common gate. Returns None if OK, else a message."""
    why = wait_for_status(robot)
    if why:
        return why
    # Exclusive control ownership: the first command that moves or arms the robot claims it,
    # and every one of these tests issues such commands. The DASHBOARD is a holder too — any
    # click on it claims control for 30 s idle — so "I clicked Enable a moment ago, then ran
    # the suite" is the likeliest first failure. Say so here, before a cryptic CONTROL_HELD
    # from deep inside rti_connect.
    holder, me = robot.control_holder(), robot.local_label()
    if holder is not None and holder != me:
        return (f"control is held by {holder} — most likely the dashboard, where any click "
                f"claims control and it lets go after 5 s idle (a script holds for 30 s). Wait "
                f"a few seconds, or click Release in the dashboard's status bar, then re-run. (A script on the robot itself shows up the same "
                f"way, as 127.0.0.1:<port>.)")
    st = robot.status()
    if need_enabled and st.get("state") != "ENABLED":
        return f"robot is {st.get('state')}, need ENABLED (clear errors and enable)"
    if not st.get("homed"):
        return "robot is not homed"
    errs = st.get("errors") or []
    if errs:
        return f"errors latched: {errs}"
    return None
