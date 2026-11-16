"""S4 — homing, explained for a builder (2026-10-27, TODO/first_run_plan.md P3; user: "Homing, explained").

The arm ENABLED, in IDLE: the normal homing (robot.home), each joint to its endstop, then its start pose. When it fails,
the reason RTI latched with the error (its description: shared memory error_state — the robot's status carries only
the key) is shown with what a builder checks for that kind of failure. Every homing failure RTI can report is matched
(robotics/homing/joint.py, controller.py, moves.py, gripper.py and the arm check, robotics/homing_arms.py —
test_first_run J reads their reason texts and fails on one nothing matches); an unmatched one gets the general hints.
"""
from __future__ import annotations

import time

from first_run import shm_files
from first_run.core import FAIL, INFO, PASS, SKIP, Cancelled, claim, result, step
from robotclient import CommandRejected

HOME_TIMEOUT_S = 180.0           # the client's home(wait=True) waits as long
START_TIMEOUT_S = 5.0            # the robot did not enter HOMING within this: it never started
POLL_S = 0.2
HOMING_KEYS = ("HOMING_FAILED", "GRIPPER_HOMING_FAILED", "HOMING_ABORTED")

# (words in RTI's reason, the builder's hints) — the first match wins; the reasons' own wording in robotics/homing/
REASON_HINTS = [
    (("did not reach its endstop", "timeout ("),
     ["the joint started far from its endstop: move the arm to its park pose by hand before homing",
      "something in the way, or a cable caught",
      "the joint homes the wrong way: its direction in the robot's homing sequence",
      "a Hall-sensor joint: the sensor's cable and its magnet (stage 2 checks the sensor)"]),
    (("Hall sensor still triggered",),
     ["the Hall sensor stays triggered: the magnet too close, or the sensor's output shorted (stage 2 checks it)"]),
    (("passes stopped", "ticks apart"),
     ["the joint stopped at two different places on its two passes: a loose or flexible endstop",
      "a coupling or belt that slips (stage 3 checks the coupling)",
      "a cable or part that sometimes catches before the endstop"]),
    # the arm check (robotics/homing_arms.py) before "false stall": its texts name a false stall as one cause
    (("controlled approach stopped",),
     ["something in the way before the endstop: the arm against itself or its base, a cable",
      "the joint could not carry the arm there: move the arm to its park pose and home again"]),
    (("always homes",),
     ["after a deliberate mechanical change (a new belt, the joint taken apart): accept the new reference in the "
      "dashboard's System Info, Arms (homing)",
      "otherwise: a belt that skipped, a coupling that slipped, or a false stall — start from the park pose"]),
    (("known arm",),
     ["a false stall: a joint stopped before its endstop — could it move from where it was? Move the arm to its park "
      "pose and home again",
      "a different arm on this controller: the dashboard's System Info, Arms (homing), 'Learn a new arm on the next "
      "homing'"]),
    (("during its release", "false stall"),
     ["the joint stalled before its endstop: something stiff (a tight belt, a cable), or a load the homing current "
      "cannot move — start from the park pose",
      "the joint's homing current in the robot's homing sequence"]),
    (("never read valid", "no position was read", "without a home reference"),
     ["the drive read no position: its encoder (the magnet on the motor shaft, the drive's encoder setting)",
      "the CAN connection to that drive (stage 1 checks it)"]),
    (("firmware calibration", "no endstop"),
     ["the gripper's jaws move freely, nothing between them",
      "the gripper's power and its CAN cable (stage 1 checks it answers)"]),
]
ABORTED_HINTS = ["the homing was interrupted — Stop, Disable, the E-stop or a mode change ends it, and the arm is "
                 "then not homed: home it again",
                 "an error ended it: the dashboard's Errors tab names it"]
GENERAL_HINTS = ["logs/RTI.log has each joint's homing steps (search for the joint, e.g. 'J3 [CANID 2]')"]


def hints_for(key, reason):
    """The builder's hints for one homing error: by the words of RTI's reason, else by its code."""
    text = (reason or "").lower()
    for words, hints in REASON_HINTS:
        if any(w.lower() in text for w in words):
            return list(hints)
    if key.endswith("HOMING_ABORTED"):
        return list(ABORTED_HINTS)
    return list(GENERAL_HINTS)


def _shm(ctx):
    return ctx.get("shm") or shm_files


def _description(ctx, key, fallback):
    """RTI's own line for an error key (its reason in it), else the robot's title."""
    for e in _shm(ctx).errors():
        if e.get("key") == key and e.get("description"):
            return str(e["description"])
    return fallback


def _stop(robot):
    """Stop the homing (robot.stop(): RTI leaves HOMING for IDLE; the arm is then not homed). Never raises."""
    try:
        robot.stop()
    except Exception:
        pass


def _answer_question(robot, ui, question):
    """Homing asks (2026-11-16, user: "pause and ask"): a joint homed off where this arm's joint always homes, and
    homing holds the arm. Asks the user; sends the answer; waits until the robot no longer reports the question (its
    status follows within a tick). -> what was answered, or None = stop (the caller stops the homing)."""
    listed = ", ".join(f"{k} {float(v):+.2f} deg" for k, v in question.items())
    joints = ", ".join(question)
    pick = ui.choose(f"Homing asks: {listed} off where this arm always homes it. Changed something on purpose (the "
                     f"gripper re-fitted, a belt or a drive replaced)? Then accept it as the new reference. Otherwise "
                     f"(a false stall, a skipped belt, a slipping coupling) stop the homing, check {joints}, bring the "
                     f"arm to its park pose and run this stage again.", ["accept", "stop"])
    if pick != "accept":
        return None
    try:
        robot.accept_homing_reference(wait=False)
    except CommandRejected:               # answered meanwhile (the dashboard): the wait goes on with what happens next
        pass
    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end and (robot.status() or {}).get("homing_question"):
        time.sleep(POLL_S)
    return f"{joints}: accepted as the new reference"


def check_homing(ctx, ui):
    st = HOME
    robot = ctx["robot"]
    status = robot.get_status() or {}
    if status.get("mode") != "IDLE":
        return result(st, SKIP, f"needs IDLE (the robot is in {status.get('mode')}: clear its errors first)")
    homed = bool(status.get("homed"))
    ask = ("The arm is homed already. Home it again? Each joint moves slowly to its endstop, then to its start pose "
           "(about half a minute)." if homed else
           "The arm homes now: each joint moves slowly to its endstop, then to its start pose (about half a minute). "
           "Clear the space around the arm and keep a hand on the E-stop.")
    if ui.choose(ask, ["home", "skip"]) != "home":
        return result(st, INFO if homed else SKIP, "already homed (not homed again)" if homed else
                      "not homed (the user's choice): stage 5 needs a homed arm")
    why = claim(ctx, ui)
    if why:
        return result(st, SKIP, why)
    before = {e.get("key") for e in (robot.errors() or [])}
    t0 = time.monotonic()
    try:
        robot.home(wait=False)
    except CommandRejected as e:          # RTI answers since 1.1.0: not homed, the robot's state says why
        return result(st, SKIP, f"the robot refused to home: {e.message}")
    seen = {"homing": False, "polls": 0}

    def ended():
        s = robot.status() or {}
        seen["polls"] += 1
        if s.get("mode") == "HOMING":
            seen["homing"] = True
            return s if s.get("homing_question") else None     # homing asks (2026-11-16): answered below
        if not seen["homing"] and seen["polls"] * POLL_S < START_TIMEOUT_S:
            return None                   # not begun yet (the status follows within ~20 ms; a few polls of slack)
        return s

    answers = []
    while True:
        try:
            s = ui.wait("Homing — each joint to its endstop, then the start pose", ended, HOME_TIMEOUT_S, POLL_S)
        except Cancelled:                     # a skip stops the homing, like a skip of any other move
            _stop(robot)
            return result(st, SKIP, "skipped by the user: the homing was stopped, the arm is not homed")
        if s is None or not s.get("homing_question"):
            break
        asked = _answer_question(robot, ui, s["homing_question"])
        if asked is None:
            _stop(robot)
            return result(st, FAIL, "stopped at homing's question: a joint homed off where this arm always homes; "
                          "the homing was stopped, the arm is not homed", hints=list(GENERAL_HINTS))
        answers.append(asked)
    took = time.monotonic() - t0
    if s is None:
        _stop(robot)
        return result(st, FAIL, f"homing did not end within {HOME_TIMEOUT_S:.0f} s (stopped)",
                      hints=GENERAL_HINTS)
    errs = [e for e in (robot.errors() or []) if e.get("key") not in before]
    failed = [e for e in errs if str(e.get("key", "")).endswith(HOMING_KEYS)]
    if not failed and not seen["homing"]:      # before IDLE + homed: an arm homed before would read as a PASS
        return result(st, FAIL, "homing never started (the robot stayed in " + str(s.get("mode")) + ")",
                      value=s.get("mode"), hints=GENERAL_HINTS)
    if s.get("mode") == "IDLE" and s.get("homed") and not failed:
        return result(st, PASS, "homed" + (f" in {took:.0f} s" if took >= 1 else "")
                      + (f" ({'; '.join(answers)})" if answers else ""), value=round(took, 1))
    out = []
    for e in failed or errs:
        key = str(e.get("key"))
        reason = _description(ctx, key, e.get("title") or key)
        who = key.split(":", 1)[0] if ":" in key else ""     # "J3:HOMING_FAILED" -> J3 (RTI's line names no joint)
        out.append(result(st, FAIL, f"{who}: {reason}" if who and not reason.startswith(who) else reason, value=key,
                          hints=list(dict.fromkeys(hints_for(key, reason) + GENERAL_HINTS))))
    others = [str(e.get("key")) for e in errs if e not in failed]
    if failed and others:
        out.append(result(st, INFO, "also latched while homing: " + ", ".join(others), value=others))
    if not out:
        out.append(result(st, FAIL, f"homing ended in {s.get('mode')}, not homed", value=s.get("mode"),
                          hints=ABORTED_HINTS + GENERAL_HINTS))
    return out


HOME = step("S4.home", 4, "Homing", check_homing, needs=("robot", "enabled"),
            about="each joint finds its endstop; a failure is explained for a builder")
STEPS = [HOME]
