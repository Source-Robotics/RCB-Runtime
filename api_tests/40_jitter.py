#!/usr/bin/env python3
"""40: with a sender whose rate WANDERS -- a real VR headset or haptic handle -- which
generator should SOFT_RTI use? The decisive test.

Everything before this measured a metronome. Real devices do not hold a rate; they drift
around 50-60 or 90-100 Hz. Off-hardware that changes the answer completely:

    fixed 55 Hz (worst fixed rate)   24% velocity ripple
    WANDERING 50-60 Hz               46%     -- worse than the worst fixed rate

and it reverses which generator to use. On a steady 5 Hz stream `ruckig_advancing` +
min_duration is the measured frontier (servo_mode_options.md 14). Under jitter the tracker
beats it on BOTH axes at once -- 22x smoother AND 4x more accurate -- because Ruckig re-plans
a point-to-point profile every tick and a jittery interval corrupts the target-velocity
estimate that plan is built on, while the tracker is a continuous law with no arrival phase
and simply does not care when the next packet lands.

WHAT IS MEASURED, AND WHY ONLY THESE TWO. Shared memory has no atomic (tick, data)
publication: RTI.py increments `loop_tick` at :1351 and writes the commanded position at
:1680, so a Python sampler reads the new tick with the old data on roughly a third of its
reads. Anything computed by DIFFERENCING per-tick shm data is therefore measuring the
sampler, not the robot -- that is what made test 39 report 60-70% at every rate including
the exact divisors. Both metrics here avoid differencing entirely:

    t_rms      high-frequency joint TORQUE (Nm). The vibration -- what you hear, what heats
               the motor, what wears the gearbox. Off-hardware, the difference between these
               configs is a generator that saturates 100% of the joint's acceleration limit
               on every command versus one that uses 1%. Torque is where that shows up.
    trk_err    rms |measured - the analytic triangle| (deg). TOTAL system accuracy against
               what the PC actually meant, arm included. Not measured-vs-commanded, which
               only tells you how well the arm follows a command that may itself be wrong.

Each config runs against a STEADY sender and a WANDERING one, so the jitter penalty is
measured per config rather than assumed.

    python3 api_tests/40_jitter.py --host <ip> --go
    python3 api_tests/40_jitter.py --host <ip> --go --band 90,100
"""
import argparse
import math
import os
import random
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost

JN = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6"}

CANDIDATES = {
    "advancing":            dict(gen="ruckig_advancing"),
    "advancing+mindur2":    dict(gen="ruckig_advancing", min_duration=2.0),
    "ruckig":               dict(gen="ruckig"),
    "tracker g8":           dict(gen="tracker", gain=8.0),
    "tracker g15":          dict(gen="tracker", gain=15.0),
}


class Sampler(threading.Thread):
    """Timestamped shm sampler. The timestamp, not the loop tick, is what the analysis uses:
    the tick label is unreliable (see the module docstring) but the wall clock is not, and
    neither metric here needs to know which tick a row came from."""

    FIELDS = ("joint_position", "joint_torque")
    MAX_ROWS = 8000

    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self.error = [], None
        self._stop, self._on = threading.Event(), False

    def run(self):
        while not self._stop.is_set():
            if self._on:
                try:
                    self.rows.append((time.monotonic(),) + tuple(sc.rd(f) for f in self.FIELDS))
                    if len(self.rows) > self.MAX_ROWS:
                        del self.rows[:len(self.rows) - self.MAX_ROWS]
                except Exception as exc:
                    self.error = self.error or repr(exc)
            time.sleep(sc.DT * 0.5)

    def start_window(self):
        self.rows = []
        self._on = True

    def end_window(self):
        self._on = False
        return list(self.rows)

    def stop(self):
        self._stop.set()


def tri(t, amp, period):
    """0 -> +amp -> -amp -> 0. Starts and ends at zero so no block begins with a step."""
    ph = (t / period) % 1.0
    if ph < 0.25:
        return amp * (4.0 * ph)
    if ph < 0.75:
        return amp * (2.0 - 4.0 * ph)
    return amp * (4.0 * ph - 4.0)


def stream(rti, q0, joints, amp, period, lo_hz, hi_hz, seconds, smp, rng):
    """Stream the triangle. Each interval is drawn from [1/hi, 1/lo], so lo==hi is a
    metronome and lo<hi is a device whose rate wanders."""
    smp.start_window()
    sent, gaps = 0, []
    t0 = time.monotonic()
    nxt = t0
    prev = t0
    while True:
        now = time.monotonic()
        t = now - t0
        if t >= seconds:
            break
        q = list(q0)
        off = tri(t, amp, period)
        for j in joints:
            q[j] = q0[j] + off
        rti.send_joint_position(q)
        sent += 1
        gaps.append(now - prev)
        prev = now
        nxt += 1.0 / rng.uniform(lo_hz, hi_hz)
        slp = nxt - time.monotonic()
        if slp > 0:
            time.sleep(slp)
    # ease back to exactly q0 so the caller's hold does not start with a step
    off = tri(time.monotonic() - t0, amp, period)
    steps = int(abs(off) / max(4.0 * amp / period * (1.0 / hi_hz), 1e-9)) + 1
    for i in range(steps - 1, -1, -1):
        q = list(q0)
        for j in joints:
            q[j] = q0[j] + off * i / steps
        rti.send_joint_position(q)
        time.sleep(1.0 / hi_hz)
    rows = smp.end_window()
    g = gaps[1:]
    mean = sum(g) / max(len(g), 1)
    sd = math.sqrt(sum((x - mean) ** 2 for x in g) / max(len(g), 1))
    return rows, t0, sent / max(time.monotonic() - t0, 1e-9), 100.0 * sd / max(mean, 1e-9)


def measure(rows, t0, joints, q0, amp, period):
    """(t_rms Nm, trk_err deg) per joint. No differencing anywhere."""
    if len(rows) < 200:
        return None
    out = {}
    w = 13
    for j in joints:
        meas = [r[1][j] for r in rows]
        trq = [r[2][j] for r in rows]
        # accuracy: measured against what the PC MEANT at that instant
        err = [meas[i] - (q0[j] + tri(rows[i][0] - t0, amp, period)) for i in range(len(rows))]
        trk = sc.deg(math.sqrt(sum(e * e for e in err) / len(err)))
        if len(trq) > w * 3:
            hf = [trq[i] - sum(trq[i - w:i + w + 1]) / (2 * w + 1)
                  for i in range(w, len(trq) - w)]
            trms = math.sqrt(sum(x * x for x in hf) / len(hf))
        else:
            trms = float("nan")
        out[j] = (trms, trk)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true", help="required: this MOVES the arm")
    p.add_argument("--band", default="50,60", help="lo,hi Hz for the WANDERING sender")
    p.add_argument("--steady", type=float, default=50.0, help="the metronome control rate")
    p.add_argument("--joints", default="3,4,5")
    p.add_argument("--amp", type=float, default=15.0, help="degrees")
    p.add_argument("--period", type=float, default=4.0, help="seconds per triangle")
    p.add_argument("--seconds", type=float, default=16.0, help="per block")
    p.add_argument("--only", default="", help="comma substrings to filter configs")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--transport", default="udp", choices=("udp", "tcp"),
                   help="SOFT_RTI transport. On a loss-free link both measure the same; "
                        "the transports only diverge under packet loss.")
    p.add_argument("--state-rate", type=float, default=0.0,
                   help="0 = state every tick (250 Hz). Worth lowering on --transport tcp, "
                        "where state is 20-40x the command traffic and is the direction "
                        "that builds backpressure.")
    a = p.parse_args()

    joints = [int(x) for x in a.joints.split(",") if x.strip()]
    bad = [j for j in joints if j not in sc.SAFE_JOINTS]
    if bad:
        print(f"ABORT: joints {bad} not in SAFE_JOINTS {sc.SAFE_JOINTS} (J2/J3 excluded).")
        return 1
    lo, hi = (float(x) for x in a.band.split(","))
    names = [n for n in CANDIDATES
             if not a.only or any(s.strip() in n for s in a.only.split(","))]
    amp = math.radians(a.amp)
    profiles = (("steady", a.steady, a.steady), (f"jitter {lo:g}-{hi:g}", lo, hi))

    print("=" * 96)
    print("40: WHICH GENERATOR SURVIVES A SENDER WHOSE RATE WANDERS?")
    print("=" * 96)
    print(f"  transport {a.transport.upper()}"
          + (f"   state rate {a.state_rate:g} Hz" if a.state_rate else "   state every tick"))
    print(f"  joints {[JN[j] for j in joints]}   triangle 0 -> +/-{a.amp:g} deg over "
          f"{a.period:g}s   {sc.deg(4*amp/a.period):.1f} deg/s")
    print(f"  {len(names)} configs x 2 senders x {a.seconds:g}s -> about "
          f"{len(names)*2*(a.seconds+5)/60:.1f} min")
    print(f"  metrics: t_rms = vibration (hf torque, Nm)   trk_err = rms error vs the "
          f"analytic path (deg)\n")
    if not a.go:
        print("Refusing without --go. This MOVES the arm. Clear the workspace and keep the")
        print("E-stop in reach. Each block starts and ends at the pose it claims.")
        return 1

    smp = Sampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
    res, holds, fails = {}, {}, {}
    try:
        why = sc.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 1
        for name in names:
            for plabel, plo, phi in profiles:
                rng = random.Random(a.seed)
                print(f"  {name:>18}  {plabel:>14} ... ", end="", flush=True)
                rti = RTIClient(robot, clock="pc", transport=a.transport)
                try:
                    # A session per block: the rate contract is fixed at connect, and the
                    # analysis below must run UNCLAIMED or it is a gap the watchdog would
                    # rightly stop the arm for.
                    rti.connect(rate_hz=plo, transport=a.transport)
                    time.sleep(0.4)
                    if a.state_rate:
                        robot.rti_set_state_rate(a.state_rate)
                    sc.apply_config(robot, CANDIDATES[name])   # 5 TCP calls, before claiming
                    rti.read_once(timeout=2.0)
                    rti.claim()
                    q0 = rti.read_once(timeout=1.0)["j_p"]
                    smp.start_window()
                    sc.keep_alive(rti, q0, plo, 1.0)
                    hold = smp.end_window()
                    hold_err = {j: sc.deg(math.sqrt(sum((r[1][j] - q0[j]) ** 2
                                                        for r in hold) / max(len(hold), 1)))
                                for j in joints} if hold else {j: float("nan") for j in joints}
                    rows, t0, achieved, jit = stream(rti, q0, joints, amp, a.period,
                                                     plo, phi, a.seconds, smp, rng)
                    sc.keep_alive(rti, q0, plo, 0.6)
                    rti.release()
                except ControlLost as exc:
                    fails[(name, plabel)] = str(exc)
                    print("CONTROL LOST")
                    try:
                        robot.enter_idle_mode()
                    except Exception:
                        pass
                    continue
                if smp.error:
                    print(f"\n*** sampler failed: {smp.error}")
                    return 1
                m = measure(rows, t0, joints, q0, amp, a.period)
                if m is None:
                    fails[(name, plabel)] = f"only {len(rows)} rows"
                    print(f"too few rows ({len(rows)})")
                    continue
                res[(name, plabel)] = m
                holds[(name, plabel)] = hold_err
                print(f"{achieved:6.2f} Hz, sd {jit:4.1f}%, hold "
                      + "/".join(f"{hold_err[j]:.3f}" for j in joints)
                      + ", trk " + "/".join(f"{m[j][1]:.3f}" for j in joints))
        sc.clear_config(robot); robot.enter_idle_mode()
    except ControlLost as exc:
        print(f"\n*** CONTROL LOST: {exc}")
        return 1
    finally:
        smp.stop()
        try:
            sc.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    if not res:
        print("\nNothing completed.")
        return 1
    print("\n" + "=" * 96)
    print("RESULTS — worst joint of the three. Lower is better on both.")
    print("=" * 96)
    print(f"{'config':>18} | " + " ".join(f"{p[0]:>13}" for p in profiles)
          + f" | {'jitter cost':>12}")
    print(f"{'':>18} | " + " ".join(f"{'t_rms / err':>13}" for _ in profiles) + "")
    for name in names:
        cells, vals = [], {}
        for plabel, _, _ in profiles:
            m = res.get((name, plabel))
            if m is None:
                cells.append(f"{'--':>13}")
                continue
            t = max(m[j][0] for j in joints)
            e = max(m[j][1] for j in joints)
            vals[plabel] = (t, e)
            cells.append(f"{t:6.4f}/{e:6.3f}")
        cost = ""
        if len(vals) == 2:
            s, jv = vals[profiles[0][0]], vals[profiles[1][0]]
            cost = f"x{jv[0]/max(s[0],1e-9):.2f} vib"
        print(f"{name:>18} | " + " ".join(cells) + f" | {cost:>12}")

    print("\n" + "-" * 96)
    print("PER-JOINT DETAIL — trk_err (deg). 'hold' is the error with the arm STANDING STILL:")
    print("a large hold value means trk_err is a fixed offset, not a tracking failure.")
    print(f"{'config':>18} {'sender':>14} | "
          + " ".join(f"{JN[j]+' hold/trk':>16}" for j in joints))
    for name in names:
        for plabel, _, _ in profiles:
            m = res.get((name, plabel)); h = holds.get((name, plabel))
            if m is None:
                continue
            print(f"{name:>18} {plabel:>14} | " + " ".join(
                f"{h[j]:7.3f}/{m[j][1]:7.3f}" for j in joints))

    jlab = profiles[1][0]
    live = {n: res[(n, jlab)] for n in names if (n, jlab) in res}
    if len(live) >= 2:
        score = {n: (max(m[j][0] for j in joints), max(m[j][1] for j in joints))
                 for n, m in live.items()}
        best_v = min(score, key=lambda n: score[n][0])
        best_e = min(score, key=lambda n: score[n][1])
        print("\n" + "-" * 96)
        print(f"  under a wandering sender:")
        print(f"    quietest       {best_v:>18}   t_rms {score[best_v][0]:.4f} Nm")
        print(f"    most accurate  {best_e:>18}   err   {score[best_e][1]:.3f} deg")
        if best_v == best_e:
            print(f"\n  CONCLUSIVE: {best_v} wins BOTH axes -- it is not a trade-off.")
            adv = score.get("advancing")
            if adv:
                print(f"  vs advancing: {adv[0]/max(score[best_v][0],1e-9):.1f}x quieter, "
                      f"{adv[1]/max(score[best_v][1],1e-9):.1f}x more accurate.")
        else:
            print(f"\n  A TRADE-OFF, not a winner: '{best_v}' is quietest and '{best_e}' is")
            print(f"  most accurate. Pick per application; neither dominates.")
    if fails:
        print("\n  did not complete:")
        for k, v in fails.items():
            print(f"    {k}  {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
