"""S3 by hand (`tools/first_run.py --by-hand`) — one joint at a time goes limp, the user turns it the way the check
says, the encoder must count that way, about as far as it was turned (2026-10-18, TODO/first_run_plan.md; user first:
"prompt the user to move the robot by hand ... no currents", then: "this sounds tiresome — automatic move detection",
so stage 3 is the powered first_run/stage3.py and this stays an option, e.g. for a builder not ready to power an
unhomed arm).

Per joint, wrist first (ORDER): robot.free_joint(n) (robotics/free_joint.py: its drive's current limit 0, every other
joint still held), the user turns it about TURN_DEG so that <positive_direction> (the robot XML's words) happens and
holds it still, robot.free_joint(0). Judged:

    the direction   the encoder counted the + way (else the robot XML's <dir> for that joint is wrong — or the words)
    counting at all no reading while the user turned it: the motor does not turn with the joint (the coupling, a belt,
                    a grub screw) or the encoder (its magnet, the drive)
    the amount      the user confirms the angle read is about what they turned (a wrong gear ratio or encoder setting
                    reads a multiple; a slipping coupling reads less)

What turning by hand cannot check: that the MOTOR can drive the joint (its phase wiring, the drive's calibration) —
homing is the first powered motion and has its own checks.
"""
from __future__ import annotations

from first_run.core import FAIL, INFO, PASS, SKIP, Aborted, Cancelled, claim, result, step

ORDER = (5, 4, 3, 0, 2, 1)       # J6, J5, J4, J1, J3, J2: the wrist first, the joints carrying the arm last
TURN_DEG = 30.0                  # what the user is asked to turn
MIN_TURN_DEG = 15.0              # a turn the check accepts as done
STILL_DEG = 0.5                  # ... once the joint stays within this
STILL_S = 1.0                    # ... for this long
TURN_TIMEOUT_S = 90.0
POLL_S = 0.1


def _joints(ctx):
    return list((ctx["robot"].get_status() or {}).get("joints") or [])


def turn_watcher(read, still_samples):
    """poll() for ui.wait: None until the joint moved >= MIN_TURN_DEG from an extreme it reached and then stayed within
    STILL_DEG for `still_samples` polls; then (excursion_deg, signed: + when it ended above where it had been lowest).
    Tracking the extremes lets the user first back off a little (an endstop) and then turn the asked way."""
    st = {"lo": None, "hi": None, "recent": []}

    def poll():
        q = read()
        if q is None:
            return None
        st["lo"] = q if st["lo"] is None else min(st["lo"], q)
        st["hi"] = q if st["hi"] is None else max(st["hi"], q)
        st["recent"] = (st["recent"] + [q])[-still_samples:]
        up, down = q - st["lo"], st["hi"] - q
        moved = max(up, down)
        still = len(st["recent"]) == still_samples and max(st["recent"]) - min(st["recent"]) <= STILL_DEG
        if moved >= MIN_TURN_DEG and still:
            return up if up >= down else -down
        return None
    poll.state = st
    return poll


def _check_joint(ctx, ui, st, j, words):
    name = f"J{j + 1}"
    robot = ctx["robot"]
    ask = (f"{name} next: it goes limp (no motor force) while every other joint keeps holding. Hold the arm beyond "
           f"{name} so it cannot fall. Ready?")
    if ui.choose(ask, ["ready", "skip this joint"]) != "ready":
        return [result(st, SKIP, f"{name}: skipped by the user")]
    robot.free_joint(j + 1)
    try:
        if ui.wait(f"Freeing {name} ...", lambda: True if (robot.status() or {}).get("free_joint") == j + 1 else None,
                   3.0, 0.1) is None:
            return [result(st, FAIL, f"{name}: the robot did not free it", hints=["logs/RTI.log says why (IDLE only)"])]
        how = (f"so that {words}" if words else "either way (the robot XML has no <positive_direction> for it)")
        poll = turn_watcher(lambda: (lambda q: q[j] if len(q) > j else None)(_joints(ctx)),
                            max(2, int(round(STILL_S / POLL_S))))
        try:
            got = ui.wait(f"Turn {name} by hand about {TURN_DEG:.0f} degrees {how}, then hold it still. (At its end "
                          "already? Back it off a little first.)", poll, TURN_TIMEOUT_S, POLL_S)
        except Cancelled:                    # skips this joint only; the next one follows
            return [result(st, SKIP, f"{name}: skipped by the user")]
    finally:
        robot.free_joint(0)
    if got is None:
        if ui.choose(f"The robot read no turn of {name}. Did you turn it?", ["yes", "no"]) == "no":
            return [result(st, SKIP, f"{name}: not turned")]
        seen = (poll.state["hi"] - poll.state["lo"]) if poll.state["lo"] is not None else 0.0
        return [result(st, FAIL, f"{name} turned by hand but the encoder read only {seen:.1f} deg",
                       value=round(seen, 2), expected=f">= {MIN_TURN_DEG:.0f} deg",
                       hints=["the motor does not turn with the joint: the coupling between motor and gearbox, a "
                              "belt, a loose grub screw",
                              "the encoder: its magnet on the motor shaft, the drive's encoder setting"])]
    out = []
    if words and got < 0:
        out.append(result(st, FAIL, f"{name} counts BACKWARDS: turned so that {words}, the robot read {got:+.1f} deg",
                          value=round(got, 2), expected="a + reading",
                          hints=[f"{name}'s <dir> in the robot XML has the wrong value (0 <-> 1) — fix it and restart "
                                 "the robot service",
                                 f"or the words in {name}'s <positive_direction> do not match this arm — fix those "
                                 "instead (the kinematic model decides which way + is)"]))
    amount = ui.choose(f"The robot read {name} {got:+.0f} degrees. Is that about what you turned it?",
                       ["yes", "it read much less", "it read much more"])
    if amount == "it read much less":
        out.append(result(st, FAIL, f"{name}: read {abs(got):.0f} deg, the user turned it clearly further",
                          value=round(got, 2), hints=[
                              "the motor turns less than the joint: the coupling or a belt slips",
                              f"the robot XML's gear ratio or encoder bits for {name} too small"]))
    elif amount == "it read much more":
        out.append(result(st, FAIL, f"{name}: read {abs(got):.0f} deg, the user turned it clearly less",
                          value=round(got, 2), hints=[f"the robot XML's gear ratio or encoder bits for {name} too "
                                                      "large"]))
    if not out:
        out.append(result(st, PASS if words else INFO,
                          f"{name}: {'counts the + way' if words else 'counts (direction not judged)'}, "
                          f"{got:+.0f} deg read, about what was turned", value=round(got, 2)))
    return out


def check_by_hand(ctx, ui):
    st = BY_HAND
    status = ctx["robot"].get_status() or {}
    if status.get("mode") != "IDLE":
        return result(st, SKIP, f"needs IDLE (the robot is in {status.get('mode')}: clear its errors first)")
    why = claim(ctx)
    if why:
        return result(st, SKIP, why)
    words = list((ctx.get("config") or {}).get("positive_direction") or [])
    n = int((ctx.get("config") or {}).get("joint_num") or 6)
    out: list = []
    try:
        for j in (k for k in ORDER if k < n):
            out += _check_joint(ctx, ui, st, j, words[j] if j < len(words) else None)
    except Aborted as e:
        e.partial = out
        raise
    return out


BY_HAND = step("S3.by_hand", 3, "Each joint by hand", check_by_hand, needs=("robot",),
               about="one joint at a time limp; you turn it the way the check says; the encoder must count that way")
STEPS = [BY_HAND]
