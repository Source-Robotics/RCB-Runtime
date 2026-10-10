#!/usr/bin/env python3
"""E3: which generator survives a wandering sender — run from THIS machine. MOVES THE ARM.

The off-robot twin of api_tests/40_jitter.py, with the same five configs and the same two
metrics, so the tables line up directly with the on-robot runs.

Running it from here adds the variable the robot could not supply: a real network between
the sender and the arm. On loopback a "jittery" sender is only as jittery as time.sleep()
makes it, and no packet is ever lost. Here the wander is real and so is the loss.

Established on the robot (servo_mode_options.md 15.3), for comparison:
    ruckig_advancing  trk_err 0.51 deg   -- 8x more accurate than anything else
    ruckig / tracker  trk_err 4.2-4.4    -- both lag badly on motion WITH REVERSALS
    jitter cost       0.98-1.01x         -- a wandering rate cost nothing at the joint

    python3 api_tests/external/e3_jitter.py --host 192.168.1.50 --go
    python3 api_tests/external/e3_jitter.py --host 192.168.1.50 --go --transport tcp
"""
import argparse
import math
import os
import random
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
    p.add_argument("--transport", default="udp", choices=("udp", "tcp"))
    p.add_argument("--band", default="50,60", help="lo,hi Hz for the WANDERING sender")
    p.add_argument("--steady", type=float, default=50.0)
    p.add_argument("--state-rate", type=float, default=0.0)
    p.add_argument("--joints", default="3,4,5")
    p.add_argument("--amp", type=float, default=15.0)
    p.add_argument("--period", type=float, default=4.0)
    p.add_argument("--seconds", type=float, default=16.0)
    p.add_argument("--only", default="")
    p.add_argument("--seed", type=int, default=7)
    a = p.parse_args()

    joints = [int(x) for x in a.joints.split(",") if x.strip()]
    bad = [j for j in joints if j not in ec.SAFE_JOINTS]
    if bad:
        print(f"ABORT: joints {bad} not in SAFE_JOINTS {ec.SAFE_JOINTS} (J2/J3 excluded).")
        return 1
    lo, hi = (float(x) for x in a.band.split(","))
    names = [n for n in ec.CONFIGS
             if not a.only or any(s.strip() in n for s in a.only.split(","))]
    amp = math.radians(a.amp)
    profiles = (("steady", a.steady, a.steady), (f"jitter {lo:g}-{hi:g}", lo, hi))

    print("=" * 92)
    print(f"E3: GENERATOR vs WANDERING SENDER — this PC -> {a.host}  [{a.transport.upper()}]")
    print("=" * 92)
    print(f"  {ec.clock_report()}")
    print(f"  {len(names)} configs x 2 senders x {a.seconds:g}s -> about "
          f"{len(names)*2*(a.seconds+6)/60:.1f} min")
    if not a.go:
        print("\n  Refusing without --go. This MOVES the arm. Run e1_link.py first.")
        return 1

    robot = RobotClient(); robot.connect(a.host)
    res, fails = {}, {}
    try:
        why = ec.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 1
        try:
            robot.reload_recipes(); time.sleep(0.3)
        except Exception:
            pass
        robot.set_recipe(ec.RECIPE)
        for name in names:
            for plabel, plo, phi in profiles:
                rng = random.Random(a.seed)
                print(f"  {name:>18} {plabel:>14} ... ", end="", flush=True)
                rti = RTIClient(robot, clock="pc", transport=a.transport)
                try:
                    rti.connect(rate_hz=plo, transport=a.transport)
                    time.sleep(0.5)
                    ec.apply_config(robot, ec.CONFIGS[name])
                    robot.rti_set_state_rate(a.state_rate)
                    first = rti.read_once(timeout=5.0)
                    if not isinstance(first.get("j_t"), list):
                        fails[(name, plabel)] = "recipe lacks j_t — run e1_link.py"
                        print("recipe missing j_t"); continue
                    rti.claim()
                    q0 = rti.read_once(timeout=2.0)["j_p"]
                    ec.keep_alive(rti, q0, plo, 1.0)
                    cap, t0, chz = ec.stream(rti, q0, joints, amp, a.period, plo,
                                             a.seconds, rng=rng, lo=plo, hi=phi)
                    ec.keep_alive(rti, q0, plo, 1.0)
                    rti.release(); rti.close()          # release BEFORE analysing
                except ControlLost as exc:
                    fails[(name, plabel)] = f"CONTROL LOST: {exc}"
                    print("CONTROL LOST")
                    try:
                        robot.enter_idle_mode()
                    except Exception:
                        pass
                    continue
                rows = ec.parse(cap)
                ok, why_stopped = ec.stream_health(rows, t0, a.seconds)
                if not ok:
                    fails[(name, plabel)] = f"CONTROL LOST — {why_stopped}"
                    print(f"CONTROL LOST — {why_stopped}")
                    try:
                        robot.enter_idle_mode()
                    except Exception:
                        pass
                    continue
                # Compare configs on GLITCH-FREE data: the robot occasionally reports one
                # impossible position packet, which is unrelated to any config and swamps the
                # differences being measured (TODO/position_glitch.md). The count is printed.
                rows, n_glitch = ec.deglitch(rows)
                m = ec.measure(rows, t0, joints, q0, amp, a.period)
                if m is None:
                    fails[(name, plabel)] = f"only {len(rows)} usable packets"
                    print(f"too few packets ({len(rows)})"); continue
                med, mx = ec.tick_gaps(rows)
                res[(name, plabel)] = (m, med)
                ct = ec.command_timing(rows, rti.sent_log)
                print(f"{chz:6.2f} Hz, {len(rows)} pkts, gap {med:.0f}, trk "
                      + "/".join(f"{m[j][1]:.3f}" for j in joints)
                      + f" [{n_glitch} glitch pkt removed]"
                      + ("" if ct is None else
                         f" | dt max {ct['send_dt'][2]:.0f} ms, lat p99/max "
                         f"{ct['lat'][1]:.0f}/{ct['lat'][2]:.0f} ms"))
        ec.clear_config(robot); robot.enter_idle_mode()
    finally:
        try:
            ec.clear_config(robot); robot.enter_idle_mode(); robot.disconnect()
        except Exception:
            pass

    if res:
        print("\n" + "=" * 92)
        print("RESULTS — worst joint. Lower is better on both.")
        print("=" * 92)
        print(f"{'config':>18} | " + " ".join(f"{p[0]:>16}" for p in profiles))
        print(f"{'':>18} | " + " ".join(f"{'t_rms / err':>16}" for _ in profiles))
        for name in names:
            cells = []
            for plabel, _, _ in profiles:
                got = res.get((name, plabel))
                cells.append("--".rjust(16) if not got else
                             f"{max(got[0][j][0] for j in joints):6.4f}/"
                             f"{max(got[0][j][1] for j in joints):7.3f}")
            print(f"{name:>18} | " + " ".join(cells))
        jl = profiles[1][0]
        live = {n: res[(n, jl)] for n in names if (n, jl) in res}
        if len(live) >= 2:
            sc = {n: (max(m[j][0] for j in joints), max(m[j][1] for j in joints))
                  for n, (m, _) in live.items()}
            bv = min(sc, key=lambda n: sc[n][0])
            be = min(sc, key=lambda n: sc[n][1])
            print("\n" + "-" * 92)
            print(f"  under a wandering sender:  quietest {bv!r} ({sc[bv][0]:.4f} Nm), "
                  f"most accurate {be!r} ({sc[be][1]:.3f} deg)")
            if bv == be:
                print(f"  CONCLUSIVE: {bv} wins BOTH axes.")
            else:
                print("  A TRADE-OFF, not a winner — pick per application.")
            gaps = {n: g for n, (_, g) in live.items()}
            if any(g > 1 for g in gaps.values()):
                print(f"\n  NOTE: this link LOST packets (tick gaps {sorted(set(gaps.values()))}).")
                print("  That is a real-network result the on-robot suite cannot produce.")
    if fails:
        print("\n  did not complete:")
        for k, v in fails.items():
            print(f"    {k}: {v}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
