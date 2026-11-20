"""S3 — small powered test moves before homing (2026-10-18, TODO/first_run_plan.md; user: "automatic move detection",
5 deg at 5 deg/s, the current limited to 1.5 x the joint's worst gravity torque, a blocked joint tried the other way).
The by-hand variant is first_run/stage3_hand.py (`tools/first_run.py --by-hand`).

The arm ENABLED, homed or not. Per joint, wrist first (ORDER): robot.joint_test (robotics/joint_test.py) moves it the
robot XML's <first_run_move> amplitude from where it stands and back — position mode in the drive's own encoder frame,
its current limited, every other joint holding — and stops it at once if it falls 2 deg behind. WHICH WAY (2026-11-19,
the user: "it can crash into itself", J2 / J3 rest against an endstop):
    homed        both ways are planned first from where the arm stands, as dry-run joint moves in EXEC (the robot's own
                 joint-limit, safety-zone and self-collision checks; nothing moves); only a way that passes is moved,
                 + first; none passes: the joint is SKIPPED with the reason
    not homed    nothing says where the arm stands: the user confirms it rests folded (its REST pose, as it lies
                 switched off), and each joint moves only its <first_run_move from_rest> way ("+" by default), never the
                 other way automatically; blocked that way: a FAIL with the hints (or the arm is not at rest)
A refusal saying "try again" (no position read from the drive yet) is retried the same way. The user then says which
way it moved FIRST, by the + and - signs on the joint (2026-11-20, the user: the words were confusing; every PAR6
joint carries its signs) — judged only where the robot XML says where + is (<positive_direction>: the words the signs
must match, named in a FAIL's hint); "show me again" runs the same move once more and asks again (up to MAX_REPEATS;
the move judged is the last one that moved it). Judged:

    the motor drives the joint   it followed the command (else: the motor's wiring, the drive, the joint blocked, the
                                 test current too low for its load)
    the direction                the user saw it move the + way (else <dir> — or the signs on the arm)
    the coupling                 the encoder moved but the user saw nothing: the coupling / belt / grub screw slips
    effort                       the peak current against the test limit (binding, a tight belt or gearbox), how far it
                                 fell behind, whether it came back to its start
"""
from __future__ import annotations

import math

from first_run.core import FAIL, INFO, PASS, SKIP, WARN, Aborted, Cancelled, claim, result, step
from robotclient import CommandRejected

ORDER = (5, 4, 3, 0, 2, 1)       # J6, J5, J4, J1, J3, J2: the wrist first, the joints carrying the arm last
HOLD_S = 0.5                     # robotics/joint_test.py HOLD_S: still at each end
WAIT_MARGIN_S = 3.0
BLOCKED_FRACTION = 0.5           # moved less than this share of the amplitude: blocked
CURRENT_WARN_FRACTION = 0.85     # peak current above this share of the test limit: working hard
FOLLOW_WARN_DEG = 1.0            # fell behind more than this (but under the 2 deg stop): loose
RETURN_WARN_DEG = 0.5            # ended further than this from where it started

NOT_PLANNED = ("JOINT_LIMIT", "SELF_COLLISION", "ZONE_LIMIT", "ZONE_CHECK_UNAVAILABLE")   # a planned way refused
TRY_AGAIN_TRIES = 5              # a "try again" refusal is retried the same way this often ...
TRY_AGAIN_PAUSE_S = 0.1          # ... this far apart
DRY_RUN_TIMEOUT_S = 10.0
PLAN_SPEED_PCT = 10.0
NO_MOVEMENT = "I saw no movement"
DIRECTION_ANSWERS = ["+", "-", NO_MOVEMENT]      # the + and - signs on the joint (2026-11-20, the user)
BY_SIGNS = "by the + and - signs on the joint"
SHOW_AGAIN = "show me again"     # the same test move once more, then the question again (2026-11-20, the user)
MAX_REPEATS = 3                  # ... at most this often per joint
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
    """One test move of joint j (0-based) by amp deg; the finished result, None when it did not finish in time, or
    {"state": "refused", "why": ...} when the robot refused to start it — nothing moved (RTI answers since 1.1.0; e.g.
    a homed arm whose move would pass a soft limit)."""
    robot = ctx["robot"]
    seq0 = int(_jt(ctx).get("seq", 0))
    ramp = 0.5 * math.pi * abs(amp) / float(mv["speed_dps"])
    try:
        robot.joint_test(j + 1, amp, float(mv["speed_dps"]), float(mv["current_ma"]))
    except CommandRejected as e:
        return {"state": "refused", "why": e.message}
    try:
        return ui.wait(f"J{j + 1} is moving {amp:+g} deg and back — watch which way it goes FIRST",
                       lambda: (lambda r: r if int(r.get("seq", 0)) > seq0 else None)(_jt(ctx)),
                       2 * ramp + 2 * HOLD_S + WAIT_MARGIN_S, 0.1)
    finally:
        if int(_jt(ctx).get("seq", 0)) <= seq0:      # not finished (a skip, a timeout): stop it
            robot.joint_test(0, 0, 0, 0)


def _move_retrying(ctx, ui, j, amp, mv):
    """_move, with a refusal that says "try again" (no position read from the drive yet: a passing moment, RTI's
    robotics/joint_test.py) retried the same way — it was taken for blocked and the joint moved the other way."""
    sleep = ctx.get("sleep") or __import__("time").sleep
    res = _move(ctx, ui, j, amp, mv)
    for _ in range(TRY_AGAIN_TRIES - 1):
        if not (_refused(res) and "try again" in str(res.get("why", ""))):
            break
        sleep(TRY_AGAIN_PAUSE_S)
        res = _move(ctx, ui, j, amp, mv)
    return res


def _planned_ways(ctx, ui, moves, n):
    """Homed arm: per joint (0-based) {+1: None | why not, -1: None | why not} — both test moves planned from where the
    arm stands as dry-run joint moves in EXEC (nothing moves), then back to IDLE (the test moves need it). A refused
    plan aborts the program, so each refusal is cleared before the next plan. None when it could not be planned at all
    (then every joint is SKIPPED with that reason)."""
    robot = ctx["robot"]
    try:
        robot.enter_exec_mode(wait=True)
        q = [float(x) for x in robot.get_joints()]
        ways: dict = {}
        for j in (k for k in ORDER if k < n):
            mv = moves[j] if j < len(moves) else None
            if not mv:
                continue
            ways[j] = {}
            for sign in (1, -1):
                target = list(q)
                target[j] += sign * float(mv["amplitude_deg"])
                try:
                    robot.move_joints(*target, speed=PLAN_SPEED_PCT, dry_run=True)
                    robot.wait_motion_complete(timeout=DRY_RUN_TIMEOUT_S)
                    ways[j][sign] = None
                except CommandRejected as e:
                    ways[j][sign] = f"{e.code}: {e.message}"
                    robot.clear_errors(wait=True)           # the refused plan aborted the program: free it again
        return ways
    except Exception as e:      # noqa: BLE001 — no plan, no move: the caller skips the joints with the reason
        return f"the test moves could not be planned ({type(e).__name__}: {e})"
    finally:
        try:
            robot.enter_idle_mode(wait=True)
        except Exception:   # noqa: BLE001 — the joints' own start then refuses with the reason
            pass


def _ways(mv, plan, homed):
    """The signs this joint may move, in order, and why the others may not: homed — the planned ways that passed, +
    first; not homed — its from_rest way only."""
    if not homed:
        sign = -1 if float(mv.get("from_rest", 1.0)) < 0 else 1
        return [sign], {-sign: "not tried: the arm is not homed, so only the joint's free way from the rest pose moves"}
    ok = [sgn for sgn in (1, -1) if plan.get(sgn) is None]
    return ok, {sgn: why for sgn, why in plan.items() if why is not None}


def _blocked(res, amp):
    return res is None or res.get("state") == "fell_behind" or abs(float(res.get("moved_deg", 0))) < BLOCKED_FRACTION * abs(amp)


def _refused(res):
    return res is not None and res.get("state") == "refused"


def _check_joint(ctx, ui, st, j, words, mv, plan=None, homed=False):
    name = f"J{j + 1}"
    if not mv:
        return [result(st, SKIP, f"{name}: no <first_run_move> in the robot XML — its test move is not defined")]
    amp, cur = float(mv["amplitude_deg"]), float(mv["current_ma"])
    signs, why_not = _ways(mv, plan or {}, homed)
    if not signs:
        return [result(st, SKIP, f"{name}: no safe room for its test move from this pose — "
                                 + "; ".join(f"{'+' if sgn > 0 else '-'}: {w}" for sgn, w in sorted(why_not.items(),
                                                                                                reverse=True)),
                       hints=["move the arm to a pose with room around this joint (jog it), then run stage 3 again"])]
    if ui.choose(f"{name}: moves {amp:g} deg and back at {float(mv['speed_dps']):g} deg/s, its force limited "
                 f"({cur:.0f} mA). Go?", ["go", "skip this joint"]) != "go":
        return [result(st, SKIP, f"{name}: skipped by the user")]
    tried: list = []                   # (sign, result) of each way that did not move it
    try:
        for k, sign in enumerate(signs):
            if k:
                ui.say(f"{name} did not move the {'+' if signs[k - 1] > 0 else '-'} way — trying the other way "
                       "(planned clear of the limits, zones and the arm itself)")
            res = _move_retrying(ctx, ui, j, sign * amp, mv)
            if res is not None and res.get("state") == "stopped":
                return [result(st, SKIP, f"{name}: the test move was stopped (the arm disabled, an error, a mode "
                                         "change)", value=res)]
            if not _blocked(res, amp):
                return _judge(st, ui, name, words, amp, cur, sign, res, first=tried[0][1] if tried else None,
                              again=lambda s=sign: _move_retrying(ctx, ui, j, s * amp, mv))
            tried.append((sign, res))
    except Cancelled:
        return [result(st, SKIP, f"{name}: skipped by the user (its test move stopped)")]
    value = {("plus" if sgn > 0 else "minus"): r for sgn, r in tried}
    if all(_refused(r) for _, r in tried):          # not tested: the robot's state, not the build
        whys = dict.fromkeys(r["why"] for _, r in tried)
        return [result(st, SKIP, f"{name}: the robot refused its test move: " + "; ".join(whys), value=value)]
    ran = [(sgn, r) for sgn, r in tried if not _refused(r)]
    refused = [r for _, r in tried if _refused(r)]
    if len(ran) > 1:
        how = "either way"
    else:
        how = f"the {'+' if ran[0][0] > 0 else '-'} way"
        if refused:
            how += f", and the robot refused the other way: {refused[0]['why']}"
        elif len(signs) == 1 and why_not:
            how += " (the only way tried: " + "; ".join(why_not.values()) + ")"
    ran = ran[0][1]
    hints = list(FAIL_HINTS)
    if not homed:
        hints.append("or the arm is not at its rest pose: the joint stands against its end the way it was moved — fold "
                     "it to rest (support it, disabled), or check its <first_run_move from_rest> in the robot XML")
    return [result(st, FAIL, f"{name} did not follow its test move {how} (moved "
                             f"{float((ran or {}).get('moved_deg', 0)):+.2f} deg of {amp:g}, behind up to "
                             f"{float((ran or {}).get('max_error_deg', 0)):.2f} deg)",
                   value=value, expected=f"{amp:g} deg", hints=hints)]


def backwards_hints(name, words):
    """A joint seen (or counted) going the wrong way: its <dir>, or the signs on the arm on the wrong sides — + is
    where the robot XML's <positive_direction> words say (the kinematic model decides; stage3_hand too)."""
    return [f"{name}'s <dir> in the robot XML has the wrong value (0 <-> 1) — fix it and restart the robot service",
            f"or {name}'s + and - signs are on the wrong sides — + is: {words} (its <positive_direction>; the kinematic "
            "model decides which way + is) — move the signs instead"]


def _question(name, words, first, repeated):
    """Both ways by the signs on the joint (2026-11-20, the user: the words — "counter-clockwise, seen from ..." — were
    confusing; 2026-11-19 before: "the other way" was abstract). A first try that did not move it is said as it was,
    without which way it went; after "show me again" the question says the joint moved again."""
    if not words:
        return f"Did you see {name} move{' this time' if repeated else ''}?"
    ask = f"Which way did {'it' if repeated or first is not None else name} move FIRST, {BY_SIGNS}?"
    if repeated:
        return f"{name} moved again. {ask}"
    if first is None:
        return ask
    if _refused(first):
        return (f"{name}'s first try was refused by the robot ({first.get('why')}) and did not move it; then it moved. "
                f"{ask}")
    return f"{name}'s first try was blocked and stopped at once; then it moved. {ask}"


def _ask_seen(ui, name, words, amp, res, first, again):
    """The question; "show me again" (offered up to MAX_REPEATS times) runs `again()` — the same test move — and asks
    again. A repeat that did not move the joint is said and ends the repeats; the answer then is about the move seen
    before. Returns (answer, the result to judge: the last move that moved the joint)."""
    options = list(DIRECTION_ANSWERS) if words else ["yes", NO_MOVEMENT]
    repeats, repeated = 0, False
    while True:
        offer = options + ([SHOW_AGAIN] if again is not None and repeats < MAX_REPEATS else [])
        answer = ui.choose(_question(name, words, first, repeated), offer)
        if answer != SHOW_AGAIN:
            return answer, res
        repeats += 1
        rep = again()
        if rep is None or rep.get("state") == "stopped" or _blocked(rep, amp):
            why = (f"refused: {rep.get('why')}" if _refused(rep) else "stopped" if rep and rep.get("state") == "stopped"
                   else "it did not get far")
            ui.say(f"{name}'s repeat did not move it ({why}) — answer for the move you saw before")
            repeats = MAX_REPEATS
            continue
        res, repeated = rep, True


def _judge(st, ui, name, words, amp, cur, sign, res, first=None, again=None):
    way = "+" if sign > 0 else "-"
    answer, res = _ask_seen(ui, name, words, amp, res, first, again)
    right = way if words else "yes"
    moved, err = float(res.get("moved_deg", 0)), float(res.get("max_error_deg", 0))
    peak, end = float(res.get("peak_current_ma", 0)), float(res.get("end_error_deg", 0))
    if answer == NO_MOVEMENT:
        return [result(st, FAIL, f"{name}: the encoder read {moved:+.1f} deg but the joint did not move",
                       value=res, hints=["the motor turns but the joint does not: the coupling between motor and "
                                         "gearbox, a belt, a loose grub screw"])]
    out = []
    if answer != right:
        out.append(result(st, FAIL, f"{name} moves BACKWARDS: commanded {way}{abs(amp):g} deg, seen the opposite",
                          value=res, expected=way, hints=backwards_hints(name, words)))
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
    homed = bool(status.get("homed"))
    if not homed and ui.choose(
            "The arm is not homed, so the robot does not know where it stands: each joint moves only the way it is "
            "free from the REST pose — the arm folded down as it lies switched off (J2 and J3 against their ends). "
            "Is the arm resting folded like that?", ["yes, it rests folded", "no"]) != "yes, it rests folded":
        return result(st, SKIP, "the arm is not at its rest pose: fold it to rest (support it, disabled), or home it "
                                "first (stage 4), then run stage 3 again")
    why = claim(ctx, ui)
    if why:
        return result(st, SKIP, why)
    cfg = ctx.get("config") or {}
    words = list(cfg.get("positive_direction") or [])
    moves = list(cfg.get("first_run_move") or [])
    n = int(cfg.get("joint_num") or 6)
    plans: dict = {}
    if homed:
        ui.say("Planning each joint's test move both ways from where the arm stands (limits, zones, the arm itself) ...")
        plans = _planned_ways(ctx, ui, moves, n)
        if isinstance(plans, str):
            return result(st, SKIP, plans)
    out: list = []
    try:
        for j in (k for k in ORDER if k < n):
            out += _check_joint(ctx, ui, st, j, words[j] if j < len(words) else None,
                                moves[j] if j < len(moves) else None, plan=plans.get(j), homed=homed)
    except Aborted as e:
        e.partial = out
        raise
    return out


MOVES = step("S3.moves", 3, "Small test moves", check_moves, needs=("robot", "enabled"),
             about="each joint moves a few degrees and back, slowly, its force limited; you say which way it went")
STEPS = [MOVES]
