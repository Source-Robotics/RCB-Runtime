#!/usr/bin/env python3
"""E1: can this machine reach and drive the robot at all? NO MOTION. Run this first.

Everything the on-robot suite takes for granted has to be established here: the ports are
open, the access list allows this PC, state packets actually arrive, and the link is good
enough to hold a claim. It moves nothing — phase 4 streams back the arm's own pose, so a
correct system sits perfectly still.

WHAT IT MEASURES THAT THE ON-ROBOT TESTS CANNOT: **packet loss**. Each state packet carries
the robot's own tick, so a gap of more than one tick between consecutive packets is a packet
that did not arrive. On loopback that number is always 1 and the on-robot suite could never
see otherwise. Over a real network it is the single most important number here, because
packet loss is the ONLY place UDP and TCP genuinely diverge.

    python3 api_tests/external/e1_link.py --host 192.168.1.50
    python3 api_tests/external/e1_link.py --host 192.168.1.50 --transports udp
"""
import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from api_tests.external import _ext_common as ec
from robotclient import RobotClient, RTIClient, ControlLost

EXEC_PORT = 29000
UDP_CMD_PORT = 30001
UDP_STATE_PORT = 30002
TCP_STREAM_PORT = 30003

_ok, _bad = 0, []


def check(name, cond, detail="", fail_detail=""):
    """`detail` is informational and shown either way; `fail_detail` only on failure.

    Without the split, a check whose detail explains what went WRONG prints that
    explanation next to a PASS, which reads as a contradiction.
    """
    global _ok
    if cond:
        _ok += 1
        print(f"  PASS  {name}" + (f"  [{detail}]" if detail else ""))
    else:
        _bad.append(name)
        msg = fail_detail or detail
        print(f"  FAIL  {name}" + (f"  [{msg}]" if msg else ""))
    return cond


def port_open(host, port, timeout=3.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return True
    except Exception:
        return False
    finally:
        s.close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", required=True, help="the ROBOT's IP address")
    p.add_argument("--transports", default="udp,tcp")
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--hold", type=float, default=3.0, help="seconds to hold the claim")
    a = p.parse_args()
    transports = [t.strip() for t in a.transports.split(",") if t.strip()]

    print("=" * 84)
    print(f"E1: LINK CHECK — this PC -> robot at {a.host}.  NO MOTION.")
    print("=" * 84)

    print("\n1. Ports")
    check(f"executor TCP {EXEC_PORT} reachable", port_open(a.host, EXEC_PORT),
          "if this fails: robot down, wrong IP, or a firewall")
    if "tcp" in transports:
        check(f"RTI stream TCP {TCP_STREAM_PORT} reachable", port_open(a.host, TCP_STREAM_PORT),
              "needs an RTI.py built after 2026-08-21")
    if _bad:
        print("\nABORT: cannot reach the robot. Nothing below would mean anything.")
        return 1

    print("\n2. Session and access control")
    robot = RobotClient()
    try:
        robot.connect(a.host)
    except Exception as exc:
        print(f"  FAIL  RobotClient.connect -> {exc!r}")
        print("\n  If this is a refusal rather than a timeout, this PC's IP is not in the")
        print("  robot's access list. On the ROBOT, restart the executor with:")
        print("      RCB_ALLOWED_IPS=<this-pc-ip> ...")
        print("  Do NOT edit ALLOWED_IPS in the source — it is a tracked file in a public")
        print("  repo and developer machine IPs must not be committed to it.")
        return 1
    check("RobotClient connected", True)
    my_ip = robot.local_ip()
    print(f"        this PC appears to the robot as {my_ip}")
    # status() is the latest PUSHED event, empty until one arrives. Wait before reading, or
    # a healthy robot reports mode/state/homed as None.
    _why = ec.wait_for_status(robot)
    check("robot is publishing status events", _why is None, _why or "")
    st = robot.status()
    print(f"        robot: mode {st.get('mode')}  state {st.get('state')}  "
          f"homed {st.get('homed')}  errors {st.get('errors') or 'none'}")
    why = ec.preflight(robot)
    if not check("robot is ENABLED, homed and error-free", why is None, why or ""):
        robot.disconnect()
        return 1

    print(f"\n3. Telemetry recipe")
    # RecipeManager scans data_recipe/ ONCE at startup, so a recipe copied to the robot
    # afterwards is invisible until a reload — and set_recipe for an unknown name is refused
    # robot-side with only a log line the caller never sees. Ask for a reload first, and
    # VERIFY below by looking at what the packets actually contain rather than trusting that
    # the command was accepted.
    try:
        robot.reload_recipes()
        time.sleep(0.3)
    except Exception as exc:
        print(f"        (reload_recipes unavailable: {exc!r} — needs a newer robot build)")
    try:
        robot.set_recipe(ec.RECIPE)
        check(f"set_recipe({ec.RECIPE!r}) accepted", True)
    except Exception as exc:
        check(f"set_recipe({ec.RECIPE!r}) accepted", False, repr(exc))
        print(f"  The robot needs data_recipe/{ec.RECIPE}.xml. Copy it over and reload.")
        robot.disconnect()
        return 1

    rc = 0
    for tr in transports:
        print(f"\n4. {tr.upper()} — pair, stream, claim, hold, release")
        rti = RTIClient(robot, clock="pc", transport=tr)
        try:
            t = time.monotonic()
            rti.connect(rate_hz=a.rate, transport=tr)
            time.sleep(0.5)
            first = rti.read_once(timeout=5.0)
            check(f"{tr}: state packets arriving",
                  isinstance(first.get("j_p"), list),
                  f"paired in {(time.monotonic()-t)*1e3:.0f} ms, j_p={len(first.get('j_p', []))} joints")
            # THE REAL VERIFICATION. set_recipe returns nothing on refusal, so the only way
            # to know it took is to look at what arrives.
            if not check(f"{tr}: recipe carries j_t (torque)",
                         isinstance(first.get("j_t"), list),
                         fail_detail="set_recipe silently did NOT take — is "
                                     "external_test.xml on the robot, and has RTI "
                                     "reloaded?"):
                print(f"        packet tags actually present: {sorted(first.keys())}")
            check(f"{tr}: recipe carries the robot tick", "tick" in first)

            # Full state rate for the loss measurement. The throttle is STICKY across
            # sessions, and a leftover 25 Hz from an earlier run shows up as a 10-tick gap
            # that reads exactly like 90% packet loss. Set it explicitly rather than hope.
            try:
                robot.rti_set_state_rate(0.0)
                time.sleep(0.2)
            except Exception:
                pass
            # measure the incoming stream WITHOUT a claim: pure link quality
            rti.capture(True)
            t = time.monotonic()
            while time.monotonic() - t < 3.0:
                rti.read_state()
                time.sleep(0.002)
            rti.capture(False)
            rows = ec.parse(rti.captured)
            hz = ec.state_rate(rows)
            med, mx = ec.tick_gaps(rows)
            check(f"{tr}: state rate is sane", hz > 50, f"{hz:.1f} pkt/s over 3 s")
            # THE number this suite exists to produce
            lost_pct = 100.0 * (1.0 - 1.0 / med) if med and med == med and med > 0 else float("nan")
            print(f"        tick gap between packets: median {med:.0f}, worst {mx:.0f}")
            if med == 1:
                print(f"        -> NO packet loss on this link ({len(rows)} packets)")
            else:
                print(f"        -> roughly {lost_pct:.1f}% of packets are not arriving")
            check(f"{tr}: no sustained packet loss", med == 1,
                  f"median tick gap {med:.0f} (1 = perfect)")

            q0 = rti.read_once(timeout=2.0)["j_p"]
            rti.claim()
            ec.keep_alive(rti, q0, a.rate, a.hold)
            held = rti.read_once(timeout=2.0)["j_p"]
            drift = max(abs(ec.deg(held[j] - q0[j])) for j in ec.SAFE_JOINTS)
            check(f"{tr}: held the claim for {a.hold:g} s without the watchdog firing", True)
            check(f"{tr}: arm did not move while holding its own pose",
                  drift < 0.5, f"{drift:.4f} deg")
            rti.release()
            rti.close()
        except ControlLost as exc:
            check(f"{tr}: session survived", False, f"CONTROL LOST: {exc}")
            rc = 1
            try:
                robot.enter_idle_mode()
            except Exception:
                pass
        except Exception as exc:
            check(f"{tr}: session survived", False, repr(exc))
            rc = 1
            try:
                robot.enter_idle_mode()
            except Exception:
                pass
    try:
        robot.enter_idle_mode()
        robot.disconnect()
    except Exception:
        pass

    print("\n" + "=" * 84)
    if _bad:
        print(f"RESULT: {len(_bad)} FAILED: " + ", ".join(_bad))
        return 1
    print(f"RESULT: ALL PASS ({_ok} checks) — this machine can drive the robot.")
    print("Next: e2_transport.py (MOVES the arm).")
    return rc


if __name__ == "__main__":
    sys.exit(main())
