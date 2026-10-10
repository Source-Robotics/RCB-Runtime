#!/usr/bin/env python3
"""E2: UDP vs TCP from THIS machine, on the same motion. MOVES THE ARM.

The off-robot twin of api_tests/41_transport.py. Same triangle, same two metrics, so the
numbers are directly comparable with the on-robot runs.

WHY RUNNING IT FROM HERE IS THE POINT. On the robot, loopback never drops a packet, so UDP
and TCP measured identical — correctly, and uselessly, because packet loss is the ONLY place
they diverge. Over a real network they can finally differ: UDP skips a lost setpoint and
carries on, TCP stalls every packet behind it until retransmission. This test reports the
loss it actually saw (from the robot's tick, carried inside each packet) alongside the motion
metrics, so a difference can be attributed rather than guessed at.

    python3 api_tests/external/e2_transport.py --host 192.168.1.50 --go
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from api_tests.external import _ext_common as ec
from robotclient import RobotClient, RTIClient, ControlLost


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", required=True)
    p.add_argument("--go", action="store_true", help="required: this MOVES the arm")
    p.add_argument("--transports", default="udp,tcp")
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--state-rate", type=float, default=0.0, help="0 = every tick")
    p.add_argument("--joints", default="3,4,5")
    p.add_argument("--amp", type=float, default=15.0)
    p.add_argument("--period", type=float, default=4.0)
    p.add_argument("--seconds", type=float, default=16.0)
    p.add_argument("--gen", default="ruckig_advancing")
    p.add_argument("--recover", action="store_true",
                   help="after a transport ends CONTROL LOST, clear errors and ENABLE the arm before the "
                        "next one (enables the motors, so opt-in)")
    p.add_argument("--dump", default="", metavar="FILE",
                   help="write every packet of each run to FILE (transport name inserted before "
                        "the extension), so a bad run can be examined sample by sample")
    a = p.parse_args()

    joints = [int(x) for x in a.joints.split(",") if x.strip()]
    bad = [j for j in joints if j not in ec.SAFE_JOINTS]
    if bad:
        print(f"ABORT: joints {bad} not in SAFE_JOINTS {ec.SAFE_JOINTS} (J2/J3 excluded).")
        return 1
    transports = [t.strip() for t in a.transports.split(",") if t.strip()]
    amp = math.radians(a.amp)

    print("=" * 92)
    print(f"E2: TRANSPORT COMPARISON from this PC -> {a.host}")
    print("=" * 92)
    print(f"  {ec.clock_report()}")
    print(f"  {transports}   {a.rate:g} Hz   {a.gen}   joints {[ec.JN[j] for j in joints]}")
    print(f"  triangle 0 -> +/-{a.amp:g} deg over {a.period:g}s "
          f"({ec.deg(4*amp/a.period):.1f} deg/s), starts and ends at the claimed pose")
    if not a.go:
        print("\n  Refusing without --go. This MOVES the arm. Run e1_link.py first.")
        return 1

    robot = RobotClient(); robot.connect(a.host)
    res, fails = {}, {}
    try:
        why = ec.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 3          # 3 = could not even start; the loss script stops instead of looping
        try:
            robot.reload_recipes(); time.sleep(0.3)
        except Exception:
            pass
        robot.set_recipe(ec.RECIPE)
        for tr in transports:
            print(f"\n--- {tr.upper()} " + "-" * 68)
            ok_ready, why_not = ec.ready_or_recover(robot, a.recover)
            if not ok_ready:
                fails[tr] = why_not
                print(f"  *** {why_not}")
                continue
            rti = RTIClient(robot, clock="pc", transport=tr)
            try:
                rti.connect(rate_hz=a.rate, transport=tr)
                time.sleep(0.5)
                ec.apply_config(robot, {"gen": a.gen})
                robot.rti_set_state_rate(a.state_rate)
                first = rti.read_once(timeout=5.0)
                if not isinstance(first.get("j_t"), list):
                    fails[tr] = "recipe lacks j_t — run e1_link.py to diagnose"
                    print(f"  *** {fails[tr]}")
                    continue
                rti.claim()
                q0 = rti.read_once(timeout=2.0)["j_p"]
                ec.keep_alive(rti, q0, a.rate, 1.0)
                cap, t0, chz = ec.stream(rti, q0, joints, amp, a.period,
                                         a.rate, a.seconds)
                ec.keep_alive(rti, q0, a.rate, 1.0)
                # RELEASE BEFORE ANALYSING. A claimed session has no idle state; parsing
                # thousands of packets under the claim exceeds the command timeout and the
                # watchdog stops the arm. That latched RTI_LINK_LOST on 2026-08-21.
                rti.release(); rti.close()
                raw_rows = ec.parse_raw(cap)
                rows = ec.retime(raw_rows)
                ok, why_stopped = ec.stream_health(rows, t0, a.seconds)
                if not ok:
                    # The most important thing this test can find, so it is REPORTED, not analysed
                    # as if it were data. Numbers from a block the robot abandoned mean nothing.
                    fails[tr] = f"CONTROL LOST — {why_stopped}"
                    print(f"  *** {fails[tr]}")
                    print(f"      (the robot has stopped the arm and is in an error hold: press "
                          f"Clear Errors, then Enable, before the next run)")
                    continue
                if a.dump:
                    base, ext = os.path.splitext(a.dump)
                    ec.dump_csv(f"{base}_{tr}{ext or '.csv'}", raw_rows, rows, t0, q0, amp,
                                a.period, joints)
                m = ec.measure(rows, t0, joints, q0, amp, a.period)
                clean_rows, n_glitch = ec.deglitch(rows)
                mc = ec.measure(clean_rows, t0, joints, q0, amp, a.period)
                med, mx = ec.tick_gaps(rows)
                res[tr] = dict(m=m, mc=mc, nglitch=n_glitch, chz=chz, shz=ec.state_rate(rows), n=len(rows),
                               gap=med, gapmax=mx)
                print(f"  sent {chz:6.2f} Hz, {len(rows)} state packets at "
                      f"{res[tr]['shz']:6.1f}/s, tick gap med {med:.0f} worst {mx:.0f}")
                ct = ec.command_timing(rows, rti.sent_log)
                res[tr]["ct"] = ct
                lc = ec.loss_counts(rows, rti.sent_log)
                res[tr]["lc"] = lc
                print(f"  {ec.fmt_timing(ct)}")
                an = ec.anatomy(rows, t0, joints, q0, amp, a.period)
                res[tr]["an"] = an
                print("  error anatomy (deg):")
                print(ec.fmt_anatomy(an, joints))
            except ControlLost as exc:
                fails[tr] = f"CONTROL LOST: {exc}"
                print(f"  *** {fails[tr]}")
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
        ec.clear_config(robot); robot.enter_idle_mode()
    finally:
        try:
            ec.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    if res:
        print("\n" + "=" * 92)
        print("COMPARISON — worst joint. 'gap' is the robot tick spacing between packets:")
        print("1 = nothing lost. This is the number loopback could never show you.")
        print("=" * 92)
        print(f"{'':>8} {'trk_err':>9} {'trk_clean':>10} {'glitches':>9} {'t_rms':>9} "
              f"{'cmd Hz':>8} {'state/s':>9} {'gap med':>8} {'gap max':>8}")
        for tr in transports:
            r = res.get(tr)
            if not r:
                continue
            cl = "n/a" if not r.get("mc") else f"{max(r['mc'][j][1] for j in joints):.3f}"
            print(f"{tr.upper():>8} {max(r['m'][j][1] for j in joints):9.3f} {cl:>10} "
                  f"{r.get('nglitch', 0):9d} "
                  f"{max(r['m'][j][0] for j in joints):9.4f} {r['chz']:8.2f} "
                  f"{r['shz']:9.1f} {r['gap']:8.0f} {r['gapmax']:8.0f}")
        if any(r.get("nglitch") for r in res.values()):
            print("\n  'glitches' = state packets the ROBOT reported with an impossible position "
                  "(arm cannot move\n  >3 deg in 4 ms). A robot-side defect, not the link or the "
                  "generator: see TODO/position_glitch.md.\n  'trk_clean' excludes them; 'trk_err' "
                  "includes them.")
        if len(res) == 2:
            u, t = res.get("udp"), res.get("tcp")
            if u and t:
                # CLEAN numbers: the raw ones carry the robot's own position glitch, which is
                # unrelated to the transport (TODO/position_glitch.md).
                ut = max((u.get("mc") or u["m"])[j][1] for j in joints)
                tt = max((t.get("mc") or t["m"])[j][1] for j in joints)
                def lost(r):
                    lc = r.get("lc") or {}
                    return lc.get("state_lost", 0), lc.get("cmd_skipped", 0)
                (us, uc), (ts_, tc) = lost(u), lost(t)
                lossy = (us + uc + ts_ + tc) > 0 or u["gapmax"] > 1 or t["gapmax"] > 1
                print("\n" + "-" * 92)
                print(f"  accuracy  TCP/UDP = {tt/max(ut,1e-9):5.2f}x")
                if not lossy:
                    print("\n  NO packet loss on this link (no missing ticks, no skipped commands), so the")
                    print("  two transports SHOULD measure the same and did. To see them differ you need")
                    print("  real loss: sudo tools/lossy_transport_test.sh run   (on the robot).")
                else:
                    def line(name, r):
                        lc, ct = r.get("lc") or {}, r.get("ct") or {}
                        span = max(lc.get("state_span", 1), 1)
                        sent = max(lc.get("cmd_sent", 1), 1)
                        lat = ct.get("lat", (float("nan"),) * 3)
                        return (f"  {name}: state packets lost {lc.get('state_lost', 0):4d} "
                                f"({100 * lc.get('state_lost', 0) / span:4.1f}%) | commands skipped "
                                f"{lc.get('cmd_skipped', 0):3d} ({100 * lc.get('cmd_skipped', 0) / sent:4.1f}%) "
                                f"| latency p50/p99/max {lat[0]:.1f}/{lat[1]:.1f}/{lat[2]:.1f} ms")
                    print("\n  WHAT THE LINK DID:")
                    print(line("UDP", u))
                    print(line("TCP", t))
                    print("  UDP loses the packet and moves on; TCP delivers every packet and pays in")
                    print("  LATENCY (retransmission). Compare the latency max to SOFT_RTI's watchdog of")
                    print("  3 command intervals (60 ms at 50 Hz): beyond it, the robot stops the arm.")
    if fails:
        print("\n  did not complete:")
        for k, v in fails.items():
            print(f"    {k}: {v}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
