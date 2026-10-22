"""S3 — small powered test moves before homing (2026-10-18, TODO/first_run_plan.md; user: "automatic move detection",
5 deg at 5 deg/s, the current limited to 1.5 x the joint's worst gravity torque, a blocked joint tried the other way).
The by-hand variant is first_run/stage3_hand.py (`tools/first_run.py --by-hand`).

The arm ENABLED, homed or not. Per joint, wrist first (ORDER): robot.joint_test (robotics/joint_test.py) moves it the
robot XML's <first_run_move> amplitude from where it stands and back — position mode in the drive's own encoder frame,
its current limited, every other joint holding — and stops it at once if it falls 2 deg behind. A joint that cannot
move the + way (at its end, blocked) is tried the other way once. The user then says which way it moved FIRST, against
the robot XML's <positive_direction> words. Judged:

    the motor drives the joint   it followed the command (else: the motor's wiring, the drive, the joint blocked, the
                                 test current too low for its load)
    the direction                the user saw it move the + way (else <dir> — or the words)
    the coupling                 the encoder moved but the user saw nothing: the coupling / belt / grub screw slips
    effort                       the peak current against the test limit (binding, a tight belt or gearbox), how far it
                                 fell behind, whether it came back to its start
"""
from __future__ import annotations

import math

from first_run.core import FAIL, INFO, PASS, SKIP, WARN, Aborted, Cancelled, claim, result, step

ORDER = (5, 4, 3, 0, 2, 1)       # J6, J5, J4, J1, J3, J2: the wrist first, the joints carrying the arm last
HOLD_S = 0.5                     # robotics/joint_test.py HOLD_S: still at each end
WAIT_MARGIN_S = 3.0
BLOCKED_FRACTION = 0.5           # moved less than this share of the amplitude: blocked
CURRENT_WARN_FRACTION = 0.85     # peak current above this share of the test limit: working hard
FOLLOW_WARN_DEG = 1.0            # fell behind more than this (but under the 2 deg stop): loose
RETURN_WARN_DEG = 0.5            # ended further than this from where it started

FAIL_HINTS = [
    "the motor's wiring: its phase wires / connector to the drive",
    "the drive's motor calibration (the drive's configuration tool)",
    "the joint blocked: something in the way, a cable caught, the joint against its endstop both ways",
    "the test current too low for this joint's load: <first_run_move current_ma> in the robot XML",
]


def _jt(ctx):
    """status()["joint_test"] (the status stream): joint, state, moved_deg, max_error_deg, peak_current_ma, ..."""
    return (ctx["robot"].status() or {}).get("joint_test") or {}


def _move(ctx, ui, j, amp, mv):
    """One test move of joint j (0-based) by amp deg; the finished result, or None when it did not finish in time."""
    robot = ctx["robot"]
    seq0 = int(_jt(ctx).get("seq", 0))
    ramp = 0.5 * math.pi * abs(amp) / float(mv["speed_dps"])
    robot.joint_test(j + 1, amp, float(mv["speed_dps"]), float(mv["current_ma"]))
    try:
        return ui.wait(f"J{j + 1} is moving {amp:+g} deg and back — watch which way it goes FIRST",
                       lambda: (lambda r: r if int(r.get("seq", 0)) > seq0 else None)(_jt(ctx)),
                       2 * ramp + 2 * HOLD_S + WAIT_MARGIN_S, 0.1)
    finally:
        if int(_jt(ctx).get("seq", 0)) <= seq0:      # not finished (a skip, a timeout): stop it
            robot.joint_test(0, 0, 0, 0)


def _blocked(res, amp):
    return res is None or res.get("state") == "fell_behind" or abs(float(res.get("moved_deg", 0))) < BLOCKED_FRACTION * abs(amp)


def _check_joint(ctx, ui, st, j, words, mv):
    name = f"J{j + 1}"
    if not mv:
        return [result(st, SKIP, f"{name}: no <first_run_move> in the robot XML — its test move is not defined")]
    amp, cur = float(mv["amplitude_deg"]), float(mv["current_ma"])
    if ui.choose(f"{name}: moves {amp:g} deg and back at {float(mv['speed_dps']):g} deg/s, its force limited "
                 f"({cur:.0f} mA). Go?", ["go", "skip this joint"]) != "go":
        return [result(st, SKIP, f"{name}: skipped by the user")]
    try:
        res, sign = _move(ctx, ui, j, amp, mv), 1
        if res is not None and res.get("state") == "stopped":
            return [result(st, SKIP, f"{name}: the test move was stopped (the arm disabled, an error, a mode change)",
                           value=res)]
        if _blocked(res, amp):
            ui.say(f"{name} could not move the + way (at its end, or blocked) — trying the other way")
            first = res
            res, sign = _move(ctx, ui, j, -amp, mv), -1
            if res is not None and res.get("state") == "stopped":
                return [result(st, SKIP, f"{name}: the test move was stopped (the arm disabled, an error, a mode "
                                         "change)", value=res)]
            if _blocked(res, amp):
                return [result(st, FAIL, f"{name} did not follow its test move either way (moved "
                                         f"{float((res or {}).get('moved_deg', 0)):+.2f} deg of {amp:g}, behind up to "
                                         f"{float((res or {}).get('max_error_deg', 0)):.2f} deg)",
                               value={"plus": first, "minus": res}, expected=f"{amp:g} deg each way", hints=FAIL_HINTS)]
    except Cancelled:
        return [result(st, SKIP, f"{name}: skipped by the user (its test move stopped)")]
    return _judge(st, ui, name, words, amp, cur, sign, res)


def _judge(st, ui, name, words, amp, cur, sign, res):
    moved, err = float(res.get("moved_deg", 0)), float(res.get("max_error_deg", 0))
    peak, end = float(res.get("peak_current_ma", 0)), float(res.get("end_error_deg", 0))
    way = "+" if sign > 0 else "-"
    if words:
        answer = ui.choose(f"Which way did {name} move FIRST?", [words, "the other way", "I saw no movement"])
        right = words if sign > 0 else "the other way"
    else:
        answer = ui.choose(f"Did you see {name} move?", ["yes", "I saw no movement"])
        right = "yes"
    if answer == "I saw no movement":
        return [result(st, FAIL, f"{name}: the encoder read {moved:+.1f} deg but the joint did not move",
                       value=res, hints=["the motor turns but the joint does not: the coupling between motor and "
                                         "gearbox, a belt, a loose grub screw"])]
    out = []
    if answer != right:
        out.append(result(st, FAIL, f"{name} moves BACKWARDS: commanded {way}{abs(amp):g} deg, seen the opposite",
                          value=res, expected=words if sign > 0 else f"the opposite of: {words}",
                          hints=[f"{name}'s <dir> in the robot XML has the wrong value (0 <-> 1) — fix it and restart "
                                 "the robot service",
                                 f"or the words in {name}'s <positive_direction> do not match this arm — fix those "
                                 "instead (the kinematic model decides which way + is)"]))
    if peak >= CURRENT_WARN_FRACTION * cur:
        out.append(result(st, WARN, f"{name} needed {peak:.0f} of its {cur:.0f} mA test current", value=res,
                          hints=["binding: a belt or gearbox too tight, a bearing, a cable pulling",
                                 "or a heavy load at this pose (the test current is 1.5 x its worst gravity torque)"]))
    if err > FOLLOW_WARN_DEG:
        out.append(result(st, WARN, f"{name} fell up to {err:.2f} deg behind its test move", value=res,
                          hints=["the joint's gains (the dashboard's Tune Joint), or friction / binding"]))
    if abs(end) > RETURN_WARN_DEG:
        out.append(result(st, WARN, f"{name} ended {end:+.2f} deg from where it started", value=res,
                          hints=["it sagged or slipped while held: a load the test current cannot hold, or a slipping "
                                 "coupling"]))
    if not out:
        out.append(result(st, PASS if words else INFO,
                          f"{name}: moved {moved:+.1f} deg the {way} way and back"
                          + ("" if words else " (direction not judged: no <positive_direction>)")
                          + f", peak {peak:.0f} of {cur:.0f} mA, within {err:.2f} deg", value=res))
    return out


def check_moves(ctx, ui):
    st = MOVES
    status = ctx["robot"].get_status() or {}
    if status.get("mode") != "IDLE":
        return result(st, SKIP, f"needs IDLE (the robot is in {status.get('mode')}: clear its errors first)")
    if ui.choose("The robot now moves each joint a few degrees and back, slowly, its force limited — J6 first, J2 "
                 "last. Clear the space around the arm and keep a hand on the E-stop.", ["start", "skip"]) != "start":
        return result(st, SKIP, "not started (the user's choice)")
    why = claim(ctx)
    if why:
        return result(st, SKIP, why)
    cfg = ctx.get("config") or {}
    words = list(cfg.get("positive_direction") or [])
    moves = list(cfg.get("first_run_move") or [])
    n = int(cfg.get("joint_num") or 6)
    out: list = []
    try:
        for j in (k for k in ORDER if k < n):
            out += _check_joint(ctx, ui, st, j, words[j] if j < len(words) else None,
                                moves[j] if j < len(moves) else None)
    except Aborted as e:
        e.partial = out
        raise
    return out


MOVES = step("S3.moves", 3, "Small test moves", check_moves, needs=("robot", "enabled"),
             about="each joint moves a few degrees and back, slowly, its force limited; you say which way it went")
STEPS = [MOVES]
