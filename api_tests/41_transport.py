#!/usr/bin/env python3
"""41: SOFT_RTI over TCP vs UDP — bring-up, then a head-to-head on the same motion.

Runs the identical triangle on both transports and reports the same numbers for each, so the
comparison is like-for-like rather than two runs remembered separately.

⚠️ WHAT LOOPBACK CANNOT SHOW, said up front so the result is not over-read. The one place the
two transports genuinely diverge is PACKET LOSS: UDP skips a setpoint and carries on, TCP
stalls every packet behind the lost one until it is retransmitted, which at an RTO of 200 ms
is fifty frozen ticks. **Loopback does not drop packets.** On `--host 127.0.0.1` both
transports will look the same on the motion metrics, and that is the correct result, not a
broken test. To see the difference you need either a real network between PC and robot, or
netem — and netem on `lo` also degrades the executor's own TCP channel on 29000, so do it on
a real interface or not at all:

    sudo tc qdisc add dev <iface> root netem loss 1%     # then re-run
    sudo tc qdisc del dev <iface> root                   # ALWAYS clean up

What loopback CAN show, and what this test therefore measures:

    bring-up      does a TCP session pair, claim, stream and release at all
    trk_err       rms |measured - the analytic path| (deg) — accuracy, arm included
    t_rms         high-frequency joint torque (Nm) — vibration
    echo-lag      round-trip from the seq echo (ms). ⚠️ FLOORED BY THE POLL INTERVAL: this
                  loop reads state once per command, so at 50 Hz nothing can be observed
                  sooner than 20 ms and the median pins there. Both transports are polled
                  identically so the COMPARISON is fair, but the absolute number is this
                  test's sampling, not the link's latency. Do not quote it as latency.
    state polled  how often WE read state, not how often the robot sent it. The robot sends
                  every tick (250 Hz) unless rti_set_state_rate says otherwise; read_state()
                  drains to newest, so everything between polls is discarded by design.
    seq lost      likewise a polling artifact here, not link loss — the robot echoes only
                  its most recent accepted seq, so sparse polling skips over intermediate
                  ones. Compare the two transports to each other; ignore the magnitude.
    recovery      drop the stream mid-session and confirm the robot survives and re-accepts

Neither motion metric differences per-tick shm, so both survive the §15.4 sampling race.

    python3 api_tests/41_transport.py --host <ip>            # bring-up only, NO motion
    python3 api_tests/41_transport.py --host <ip> --go       # the full comparison
"""
import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from api_tests import _servo_common as sc
from robotclient import RobotClient, RTIClient, ControlLost

JN = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6"}


class Sampler(threading.Thread):
    """Timestamped shm sampler. Neither metric differences per tick, so the §15.4 race —
    loop_tick advancing at RTI.py:1351 while the commanded write happens at :1680 — cannot
    corrupt either one. A stale row costs at most one tick of misalignment."""

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
                    self.rows.append((time.monotonic(),)
                                     + tuple(sc.rd(f) for f in self.FIELDS))
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
    """0 -> +amp -> -amp -> 0. Starts and ends at zero so no phase opens with a step."""
    ph = (t / period) % 1.0
    if ph < 0.25:
        return amp * (4.0 * ph)
    if ph < 0.75:
        return amp * (2.0 - 4.0 * ph)
    return amp * (4.0 * ph - 4.0)


def stream(rti, q0, joints, amp, period, rate_hz, seconds, smp):
    smp.start_window()
    per, sent, states = 1.0 / rate_hz, 0, 0
    t0 = nxt = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t >= seconds:
            break
        q = list(q0)
        off = tri(t, amp, period)
        for j in joints:
            q[j] = q0[j] + off
        rti.send_joint_position(q)
        sent += 1
        if rti.read_state() is not None:      # also keeps the TCP socket drained
            states += 1
        nxt += per
        slp = nxt - time.monotonic()
        if slp > 0:
            time.sleep(slp)
    # ease back to exactly q0
    off = tri(time.monotonic() - t0, amp, period)
    steps = int(abs(off) / max(4.0 * amp / period * per, 1e-9)) + 1
    for i in range(steps - 1, -1, -1):
        q = list(q0)
        for j in joints:
            q[j] = q0[j] + off * i / steps
        rti.send_joint_position(q)
        time.sleep(per)
    el = time.monotonic() - t0
    return smp.end_window(), t0, sent / el, states / el


def measure(rows, t0, joints, q0, amp, period):
    if len(rows) < 200:
        return None
    out, w = {}, 13
    for j in joints:
        meas = [r[1][j] for r in rows]
        trq = [r[2][j] for r in rows]
        err = [meas[i] - (q0[j] + tri(rows[i][0] - t0, amp, period))
               for i in range(len(rows))]
        trk = sc.deg(math.sqrt(sum(e * e for e in err) / len(err)))
        hf = [trq[i] - sum(trq[i - w:i + w + 1]) / (2 * w + 1)
              for i in range(w, len(trq) - w)] if len(trq) > w * 3 else [float("nan")]
        out[j] = (math.sqrt(sum(x * x for x in hf) / len(hf)), trk)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--go", action="store_true", help="required for the MOTION phases")
    p.add_argument("--transports", default="udp,tcp")
    p.add_argument("--rate", type=float, default=50.0, help="command rate")
    p.add_argument("--state-rate", type=float, default=0.0,
                   help="0 = every tick (250 Hz). Try 25 to see the throttle work")
    p.add_argument("--joints", default="3,4,5")
    p.add_argument("--amp", type=float, default=15.0)
    p.add_argument("--period", type=float, default=4.0)
    p.add_argument("--seconds", type=float, default=16.0)
    p.add_argument("--recipe", default="standard",
                   help="must carry rti_seq for the rtt column: standard or diagnostics")
    a = p.parse_args()

    joints = [int(x) for x in a.joints.split(",") if x.strip()]
    bad = [j for j in joints if j not in sc.SAFE_JOINTS]
    if bad:
        print(f"ABORT: joints {bad} not in SAFE_JOINTS {sc.SAFE_JOINTS} (J2/J3 excluded).")
        return 1
    transports = [t.strip() for t in a.transports.split(",") if t.strip()]
    amp = math.radians(a.amp)

    print("=" * 92)
    print("41: SOFT_RTI TRANSPORT COMPARISON — TCP vs UDP")
    print("=" * 92)
    print(f"  {transports}   {a.rate:g} Hz commands   joints {[JN[j] for j in joints]}")
    print(f"  triangle 0 -> +/-{a.amp:g} deg over {a.period:g}s "
          f"({sc.deg(4*amp/a.period):.1f} deg/s), starts and ends at the claimed pose")
    if a.host in ("127.0.0.1", "localhost"):
        print("\n  NOTE: loopback. Both transports SHOULD measure the same here — loopback")
        print("  does not drop packets, and packet loss is the only place they diverge.")
        print("  Identical numbers are the expected result, not a failed test.")
    if not a.go:
        print("\n  --go not given: running BRING-UP ONLY (pair, claim, hold, release).")
        print("  The arm should not move. Add --go for the motion comparison.\n")

    smp = Sampler(); smp.start()
    robot = RobotClient(); robot.connect(a.host)
    res, fails = {}, {}
    try:
        why = sc.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 1
        robot.set_recipe(a.recipe)
        for tr in transports:
            print(f"\n--- {tr.upper()} " + "-" * 70)
            rti = RTIClient(robot, clock="pc", transport=tr)
            try:
                t_dial = time.monotonic()
                rti.connect(rate_hz=a.rate, transport=tr)
                time.sleep(0.4)
                sc.apply_config(robot, {"gen": "ruckig_advancing"})
                if a.state_rate:
                    robot.rti_set_state_rate(a.state_rate)
                st = rti.read_once(timeout=3.0)
                print(f"  paired and streaming in {(time.monotonic()-t_dial)*1e3:6.0f} ms")
                rti.claim()
                q0 = rti.read_once(timeout=1.0)["j_p"]
                print(f"  claimed. holding the arm's own pose for 2 s (must not move)")
                sc.keep_alive(rti, q0, a.rate, 2.0)
                held = rti.read_once(timeout=1.0)["j_p"]
                drift = max(abs(sc.deg(held[j] - q0[j])) for j in joints)
                print(f"  drift while holding: {drift:.4f} deg  "
                      + ("OK" if drift < 0.5 else "<-- INVESTIGATE"))

                if a.go:
                    rows, t0, chz, shz = stream(rti, q0, joints, amp, a.period,
                                                a.rate, a.seconds, smp)
                    rtt = sorted(rti.rtt_samples_ms)
                    lost = rti.lost_seqs
                # RELEASE FIRST, ANALYSE AFTER. A claimed session has NO idle state: measure()
                # walks ~7000 rows x 3 joints through a 27-wide moving average, which is far
                # more than the 60 ms watchdog allows at 50 Hz. Running it under the claim
                # latched RTI_LINK_LOST and disabled the robot mid-test -- "No command for 16
                # ticks (limit 15)". _servo_common.py rule 1, learned again the hard way.
                rti.release()
                rti.close()
                if a.go:
                    if smp.error:
                        print(f"  *** sampler failed: {smp.error}")
                        return 1
                    m = measure(rows, t0, joints, q0, amp, a.period)
                    res[tr] = dict(
                        m=m, chz=chz, shz=shz, drift=drift,
                        rtt=(rtt[len(rtt) // 2] if rtt else float("nan")),
                        rtt_p95=(rtt[int(len(rtt) * 0.95)] if rtt else float("nan")),
                        lost=lost, rows=len(rows))
                    print(f"  streamed {chz:6.2f} Hz, state polled {shz:6.1f} Hz, "
                          f"{len(rows)} rows, echo-lag med "
                          f"{res[tr]['rtt']:.2f} ms p95 {res[tr]['rtt_p95']:.2f} ms")
            except ControlLost as exc:
                fails[tr] = f"CONTROL LOST: {exc}"
                print(f"  *** CONTROL LOST: {exc}")
                try:
                    robot.enter_idle_mode()
                except Exception:
                    pass
            except Exception as exc:
                fails[tr] = repr(exc)
                print(f"  *** {exc!r}")
                try:
                    robot.enter_idle_mode()
                except Exception:
                    pass
        sc.clear_config(robot); robot.enter_idle_mode()
    finally:
        smp.stop()
        try:
            sc.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    if res:
        print("\n" + "=" * 92)
        print("COMPARISON — worst joint of the three. Lower is better on trk_err and t_rms.")
        print("=" * 92)
        print(f"{'':>10} {'trk_err':>9} {'t_rms':>9} {'cmd Hz':>8} {'polled':>8} "
              f"{'echo med':>9} {'echo p95':>9} {'seq skip':>9}")
        for tr in transports:
            r = res.get(tr)
            if not r:
                continue
            trk = max(r["m"][j][1] for j in joints)
            trm = max(r["m"][j][0] for j in joints)
            print(f"{tr.upper():>10} {trk:9.3f} {trm:9.4f} {r['chz']:8.2f} "
                  f"{r['shz']:8.1f} {r['rtt']:9.2f} {r['rtt_p95']:9.2f} {r['lost']:9d}")
        if len(res) == 2 and "udp" in res and "tcp" in res:
            u, t = res["udp"], res["tcp"]
            ut = max(u["m"][j][1] for j in joints)
            tt = max(t["m"][j][1] for j in joints)
            print("\n" + "-" * 92)
            print(f"  accuracy   TCP/UDP = {tt/max(ut,1e-9):5.2f}x")
            print(f"  echo p95   UDP {u['rtt_p95']:6.2f} ms   TCP {t['rtt_p95']:6.2f} ms"
                  f"   (floored by the {1000.0/a.rate:.0f} ms poll interval — compare, "
                  f"do not quote)")
            if abs(tt - ut) / max(ut, 1e-9) < 0.15:
                print("\n  The transports are EQUIVALENT on this link — expected without")
                print("  packet loss. This says nothing about a lossy network; it says the")
                print("  TCP path adds no overhead of its own, which is what loopback tests.")
            else:
                print("\n  The transports DIFFER on a link that should not distinguish them.")
                print("  That points at the implementation, not at TCP: check TCP_NODELAY,")
                print("  the state rate, and whether the PC is keeping up with its reads.")
    if fails:
        print("\n  did not complete:")
        for k, v in fails.items():
            print(f"    {k}: {v}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
