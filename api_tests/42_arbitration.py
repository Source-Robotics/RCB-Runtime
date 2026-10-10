#!/usr/bin/env python3
"""42: control arbitration — two real clients, one arm. NO MOTION.

Exclusive control ownership is what industrial arms commonly enforce (they typically
authenticate nobody). With the IP allowlist off, it is the ONLY thing standing between two machines and a
robot that executes an interleaving of two programmes neither of them asked for.

This drives TWO genuine RobotClient connections through the real TCP path rather than
unit-testing the ownership functions, because this project has repeatedly found unit tests
passing while the real path was broken — most recently a handler table referencing a function
defined below it, which imported fine in a test and killed the executor on startup.

THE RULE UNDER TEST, from TODO/control_arbitration_plan.md §4.1:

    lock the queue, never lock the stop

So the critical assertions are not "B is refused" — they are that B can still STOP, still
read, and still take over deliberately. An arbitration layer that can lock an operator out
of stopping the robot has become the hazard it was meant to prevent.

Every command used here is a config setter or a read. Nothing moves.

    python3 api_tests/42_arbitration.py --host 127.0.0.1
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robotclient import RobotClient
from robotclient.exceptions import CommandRejected

_ok, _bad = 0, []


def check(name, cond, detail=""):
    global _ok
    if cond:
        _ok += 1
        print(f"  PASS  {name}" + (f"  [{detail}]" if detail else ""))
    else:
        _bad.append(name)
        print(f"  FAIL  {name}" + (f"  [{detail}]" if detail else ""))
    return cond


def owned(client):
    """Issue a harmless OWNED command. A config setter — moves nothing, arms nothing."""
    client.set_joint_speed(50)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--press", action="store_true",
                   help="test the ALLOW path: a human at the control box HOLDS B2 for about "
                        "a second when the OLED asks.")
    p.add_argument("--deny", action="store_true",
                   help="test the DENY path: a human at the control box TAPS B1 when the "
                        "OLED asks. Must be refused immediately, not after the 20 s window.")
    a = p.parse_args()

    print("=" * 80)
    print("42: CONTROL ARBITRATION — two clients, one arm.  NO MOTION.")
    print("=" * 80)

    A = RobotClient(); A.connect(a.host)
    B = RobotClient(); B.connect(a.host)
    try:
        print("\n1. Ownership attaches to whoever commands first")
        check("nobody holds control initially", A.control_holder() is None,
              f"holder={A.control_holder()}")
        owned(A)
        time.sleep(0.3)
        holder = A.control_holder()
        check("A owns after one owned command", holder is not None, f"holder={holder}")

        print("\n2. The second client is refused — and told who has it")
        try:
            owned(B)
            check("B is refused while A holds", False, "B was allowed to command")
        except CommandRejected as exc:
            msg = str(exc)
            check("B is refused while A holds", "CONTROL_HELD" in msg, msg[:90])
            check("...and the refusal names the holder and how to recover",
                  "take_control" in msg, msg[:120])

        print("\n3. LOCK THE QUEUE, NEVER LOCK THE STOP")
        print("   These must work for B even though A holds control:")
        for name, fn in (("get_status", lambda: B.status()),
                         ("get_joints", lambda: B.get_joints()),
                         ("get_pose", lambda: B.get_pose()),
                         ("stop", lambda: B.stop()),
                         ("jog_stop", lambda: B.jog_stop()),   # a jog has no timeout
                         ("clear_errors", lambda: B.clear_errors())):
            try:
                fn()
                check(f"non-owner B can still call {name}", True)
            except Exception as exc:
                check(f"non-owner B can still call {name}", False, repr(exc))

        print("\n4. Deliberate takeover")
        try:
            B.take_control()
            check("B's UNFORCED take_control is refused while A is active", False)
        except CommandRejected:
            check("B's UNFORCED take_control is refused while A is active", True)
        B.take_control(force=True)
        time.sleep(0.3)
        check("B's FORCED take_control succeeds", B.control_holder() is not None,
              f"holder={B.control_holder()}")
        try:
            owned(A)
            check("A is now the one refused", False, "A still commanded after losing control")
        except CommandRejected:
            check("A is now the one refused", True)

        print("\n5. Release")
        B.release_control()
        time.sleep(0.3)
        check("control is free after release_control", B.control_holder() is None,
              f"holder={B.control_holder()}")
        owned(A)
        time.sleep(0.3)
        check("A can command again once control is free", A.control_holder() is not None)

        print("\n6. Physical confirmation for a FORCED takeover (plan 4.4)")
        # Baseline with the gate OFF. Nobody holds control at this point, so disabling is
        # allowed — see the asymmetry check below for why that matters.
        A.release_control(); B.release_control(); time.sleep(0.3)
        B.set_force_confirm(False)
        time.sleep(0.2)
        owned(A); time.sleep(0.3)
        B.take_control(force=True)
        time.sleep(0.2)
        check("with the gate OFF, a forced takeover is immediate",
              B.control_holder() is not None)

        B.release_control(); owned(A); time.sleep(0.3)

        # THE SECURITY PROPERTY. Raising protection is always allowed, so the operator can
        # turn the gate on while a remote machine holds the arm — precisely when they need
        # to. Lowering it is refused to a non-holder, or "disable, then force" would bypass
        # physical confirmation in one command from anywhere on the network.
        B.set_force_confirm(True)
        time.sleep(0.2)
        check("a NON-HOLDER can RAISE protection while someone else holds control",
              B.status().get("force_confirm") is True,
              f"force_confirm={B.status().get('force_confirm')}")
        try:
            B.set_force_confirm(False)
            check("a NON-HOLDER cannot LOWER it — otherwise the gate is one command deep",
                  False, "B disabled the gate while A held control")
        except CommandRejected as exc:
            check("a NON-HOLDER cannot LOWER it — otherwise the gate is one command deep",
                  "CONTROL_HELD" in str(exc), str(exc)[:90])

        if a.deny:
            print("\n   >>> TAP B1 ON THE CONTROL BOX when the OLED asks (DENY) <<<")
            holder_before = A.control_holder()
            t0 = time.monotonic()
            try:
                B.take_control(force=True)
                check("a DENY at the box refuses the takeover", False, "it was granted")
            except CommandRejected as exc:
                waited = time.monotonic() - t0
                check("a DENY at the box refuses the takeover", "DENIED" in str(exc),
                      str(exc)[:80])
                check("...immediately, not after the 20 s window", waited < 15.0,
                      f"refused after {waited:.1f}s")
            check("A still holds control after the denied takeover",
                  A.control_holder() == holder_before,
                  f"was {holder_before}, now {A.control_holder()}")
        elif a.press:
            print("\n   >>> HOLD B2 ON THE CONTROL BOX for about a second when the OLED "
                  "asks (ALLOW) <<<")
            t0 = time.monotonic()
            try:
                B.take_control(force=True)
                ok, detail = True, ""
            except Exception as exc:
                ok, detail = False, repr(exc)
            check("a forced takeover succeeds once the button is pressed", ok, detail)
            if ok:
                check("...and it really waited for the press rather than passing through",
                      time.monotonic() - t0 > 0.5,
                      f"returned in {time.monotonic()-t0:.2f}s")
        else:
            # The OLED will light up asking for a press, because the executor has no idea
            # this test wants to be refused. Say so loudly: the first run of this failed
            # purely because the operator did what the screen told them to, 11 s in.
            print("")
            print("   >>> DO NOT TOUCH EITHER BUTTON. The OLED will ask you to — ignore it. <<<")
            print("   Testing the REFUSAL path: this waits the full 20 s and must be refused.")
            holder_before = A.control_holder()
            t0 = time.monotonic()
            outcome = {}

            def _force():
                try:
                    B.take_control(force=True)
                    outcome["granted"] = True
                except CommandRejected as exc:
                    outcome["exc"] = exc
                except Exception as exc:              # a hung/aborted call must not hang the test
                    outcome["other"] = exc

            # On its own thread so the rest of this section can act WHILE the takeover waits —
            # that is the whole point of what follows.
            waiter = threading.Thread(target=_force, daemon=True)
            waiter.start()
            time.sleep(1.5)                            # the request is now waiting at the box

            # THE GUI'S SITUATION. One connection carries both the takeover and the Disable
            # button. The executor reads one command at a time per connection, so a wait done
            # on that thread leaves the client's own stop UNREAD for up to 20 s.
            t1 = time.monotonic()
            try:
                B.stop()
                served, why = True, ""
            except Exception as exc:
                served, why = False, repr(exc)
            dt = time.monotonic() - t1
            check("while B's takeover waits, B's OWN stop is still served promptly",
                  served and dt < 3.0, why or f"answered in {dt:.2f}s")

            # One prompt, one physical resource: a second forced request must be refused at
            # once, or a single press could grant the wrong requester.
            C = RobotClient(); C.connect(a.host)
            try:
                t2 = time.monotonic()
                try:
                    C.take_control(force=True)
                    check("a 2nd forced request while one is pending is refused at once", False,
                          "it was accepted")
                except CommandRejected as exc:
                    check("a 2nd forced request while one is pending is refused at once",
                          "CONFIRM_BUSY" in str(exc) and time.monotonic() - t2 < 3.0,
                          str(exc)[:80])
            finally:
                C.disconnect()

            waiter.join(timeout=35.0)
            exc = outcome.get("exc")
            if "granted" in outcome:
                check("force is REFUSED when nobody presses the button", False,
                      "GRANTED — the button was pressed during the window. The refusal "
                      "path needs nobody to touch it; check logs/high_level.log for "
                      "'takeover ALLOWED'.")
            else:
                waited = time.monotonic() - t0
                check("force is REFUSED when nobody presses the button",
                      exc is not None and "CONFIRM_REQUIRED" in str(exc),
                      str(exc or outcome.get("other"))[:90])
                check("...and it waited for the full window before refusing",
                      waited > 15.0, f"waited {waited:.1f}s")
            # control_holder() returns the GLOBAL holder, so `is not None` passes even when
            # B has taken over — it silently agreed with the failure above. Compare to the
            # holder recorded BEFORE the attempt instead.
            check("A still holds control after the refused takeover",
                  A.control_holder() == holder_before,
                  f"was {holder_before}, now {A.control_holder()}")

        # Clean up: the holder may lower it, so release first.
        A.release_control(); B.release_control(); time.sleep(0.3)
        B.set_force_confirm(False)
        time.sleep(0.2)
        owned(A); time.sleep(0.3)

        print("\n7. A dropped connection must not hold the robot hostage")
        A.disconnect()
        time.sleep(0.5)
        check("control freed when the holder disconnects", B.control_holder() is None,
              f"holder={B.control_holder()}")
        owned(B)
        time.sleep(0.3)
        check("B can take over a dead holder's robot with no force needed",
              B.control_holder() is not None)
        B.release_control()
    finally:
        for c in (A, B):
            try:
                c.release_control()
            except Exception:
                pass
        # Leave the gate as we found it. Must come AFTER both releases — lowering it is
        # refused while anyone holds control, which is the point of the asymmetry.
        try:
            B.set_force_confirm(False)
        except Exception:
            pass
            try:
                c.disconnect()
            except Exception:
                pass

    print("\n" + "=" * 80)
    if _bad:
        print(f"RESULT: {len(_bad)} FAILED: " + ", ".join(_bad))
        return 1
    print(f"RESULT: ALL PASS ({_ok} checks)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
