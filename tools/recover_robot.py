#!/usr/bin/env python3
"""Clear errors and Enable the arm from the command line — what "Clear Errors, then Enable" on the
dashboard does. Exit 0 if the robot ends ENABLED, IDLE, error-free and homed; 1 otherwise.

    python3 tools/recover_robot.py --check     # READ-ONLY: report state, send nothing
    python3 tools/recover_robot.py             # clear errors, enable, verify

Used by tools/lossy_transport_test.sh --recover between levels. A test that ends in CONTROL LOST
leaves the robot DISABLED in an error hold (ACTIVE_ERROR, where J3 can oscillate); the next level
cannot start until someone recovers it, and the sooner that happens the less time the arm spends
there. It ENABLES THE MOTORS, so it is opt-in and never runs by itself.

It claims control only for as long as it is connected and releases on exit (a closed connection
releases immediately), so it does not leave the robot "held" the way a dashboard click does.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robotclient import RobotClient


def summary(st):
    return (f"mode {st.get('mode')}  state {st.get('state')}  homed {st.get('homed')}  "
            f"errors {st.get('errors') or 'none'}  holder {st.get('control_holder')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--check", action="store_true", help="read-only: report and exit")
    a = ap.parse_args()

    r = RobotClient()
    r.connect(a.host)
    try:
        r.wait(lambda s: s.get("mode") is not None, 10.0, "first status event")
        st = r.status()
        print(f"  before: {summary(st)}")
        if a.check:
            ok = st.get("state") == "ENABLED" and st.get("mode") == "IDLE" and not st.get("errors") and st.get("homed")
            print("  (check only — nothing sent)  " + ("robot is ready" if ok else "robot needs recovery"))
            return 0 if ok else 1

        holder = st.get("control_holder")
        if holder is not None and holder != r.local_label():
            print(f"  control is held by {holder} (probably the dashboard, which lets go after 5 s idle) "
                  f"— waiting up to 10 s")
            end = time.monotonic() + 10.0
            while time.monotonic() < end and r.control_holder() not in (None, r.local_label()):
                time.sleep(0.25)

        if st.get("errors") or st.get("mode") == "ACTIVE_ERROR":
            r.clear_errors(wait=True)
        if r.status().get("state") != "ENABLED":
            r.enable(wait=True)
        time.sleep(0.5)
        st = r.status()
        print(f"  after : {summary(st)}")
        ok = st.get("state") == "ENABLED" and not st.get("errors") and st.get("mode") != "ACTIVE_ERROR"
        if ok and not st.get("homed"):
            print("  ⚠️ not homed — re-home before motion")
            ok = False
        return 0 if ok else 1
    finally:
        try:
            r.release_control()
        except Exception:
            pass
        r.disconnect()


if __name__ == "__main__":
    sys.exit(main())
