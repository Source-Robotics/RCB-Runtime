"""S2 — the E-stop, the box's buttons, the isolated I/O and the Hall sensor; nothing moves (2026-10-18,
TODO/first_run_plan.md).

    estop     the user presses the E-stop: the channel(s) the PCB version uses read pressed and the robot latches
              ESTOP; released: clear again (the error cleared at the end)
    buttons   a short press of B1, then B2 (a short press only turns the screen's page)
    inputs    optional: the user switches each isolated input on and off
    outputs   optional: each isolated output on for OUTPUT_ON_S, the user says whether it switched ("again" switches
              it once more — a mis-typed answer is redone, not a FAIL)

Every single button, input and output can be skipped on its own (a build without the front-panel buttons, or with
only some I/O wired): Cancelled from a wait (Ctrl-C in the console) skips that one, and an output's question has
"not wired". A skipped one is a SKIP in the report, never a FAIL.
    hall      the Hall-sensor joint limp (robot.hall_check), the user turns it by hand through the magnet: the sensor
              changes and changes back; optionally twice, to see it trigger at the same angle

The arm is DISABLED for this stage (a pressed E-stop disables it anyway). Read through robot.get_hardware(can=False)
— the polls skip the CAN interface's counters.
"""
from __future__ import annotations

import math

from first_run.core import FAIL, INFO, PASS, SKIP, WARN, Cancelled, claim, result, step

PRESS_TIMEOUT_S = 60.0
BUTTON_TIMEOUT_S = 30.0
INPUT_TIMEOUT_S = 45.0
HALL_TIMEOUT_S = 120.0
POLL_S = 0.04               # a short button press lasts ~0.1-0.3 s; the supervisor refreshes it every ~20 ms
SETTLE_S = 0.5              # after the press, before the channels are read again (contact bounce, the debounce)
ROBOT_REACT_S = 1.0         # RTI latches ESTOP on the tick it reads it: allow a second for the status to show it
OUTPUT_ON_S = 2.0
OUTPUT_ANSWERS = ("yes", "no", "again", "not wired")   # "again": switch it once more, ask again (a mis-typed answer)
HALL_REPEAT_DEG = 2.0       # the same edge twice further apart than this: the magnet or the sensor is loose
ESTOP_KEYS = ("ESTOP",)
# The control box's terminal block (docs/pin_map.md): isolated inputs 1-3, outputs 1-3; E-Stop 1 = E2 + E4.
INPUT_TERMINALS = (11, 10, 9)
OUTPUT_TERMINALS = (6, 7, 8)

ESTOP_HINTS = [
    "the E-stop's two wires to the control box: E-Stop 1 is terminals E2 + E4 (E-Stop 2: E1 + E3, docs/pin_map.md)",
    "the E-stop's contact type: the box expects a normally-CLOSED contact that OPENS when pressed",
    "the E-stop button itself: a contact block not clipped on, or a broken contact",
]


def _io(ctx):
    return ctx["robot"].get_hardware(can=False)["io"]


def _errors(ctx):
    """The latched error keys, from the status stream (the executor pushes it on every change)."""
    return list((ctx["robot"].status() or {}).get("errors") or [])


def check_estop(ctx, ui):
    st = ESTOP
    robot = ctx["robot"]
    io = _io(ctx)
    used = io["estop_channels_used"]
    if io["software_estop"]:
        if ui.wait("The software E-stop (the dashboard's E-STOP button) is on — switch it off",
                   lambda: True if not _io(ctx)["software_estop"] else None, PRESS_TIMEOUT_S) is None:
            return result(st, SKIP, "the software E-stop stayed on")
    if io["estop_pressed"]:
        if ui.wait("The E-stop reads PRESSED — release it (turn or pull it to unlock)",
                   lambda: True if not _io(ctx)["estop_pressed"] else None, PRESS_TIMEOUT_S) is None:
            return result(st, FAIL, "the E-stop reads pressed and did not release", value=io["estop_raw"],
                          expected="released", hints=ESTOP_HINTS + ["a wire broken anywhere in the E-stop circuit "
                                                                    "reads as pressed — that is on purpose"])
    pressed = ui.wait("Press the E-stop now", lambda: (lambda x: x if x["estop_pressed"] else None)(_io(ctx)),
                      PRESS_TIMEOUT_S, POLL_S)
    if pressed is None:
        return result(st, FAIL, f"pressing the E-stop changed nothing within {PRESS_TIMEOUT_S:.0f} s",
                      expected="pressed", hints=ESTOP_HINTS)
    ctx["sleep"](SETTLE_S)
    held = _io(ctx)
    out = []
    if used >= 2 and not all(held["estop_channel_pressed"][:2]):
        bad = 1 + held["estop_channel_pressed"].index(False)
        out.append(result(st, FAIL, f"E-stop channel {bad} does not read pressed (the other does)",
                          value=held["estop_raw"], expected="both channels pressed",
                          hints=[f"channel {bad}'s wire or contact (a dual-channel E-stop has two separate contacts)"]))
    elif used < 2:
        out.append(result(st, INFO, f"channel 2 reads {'pressed' if held['estop_channel_pressed'][1] else 'released'}"
                                    f" — not used on PCB v{held['pcb_version']}", value=held["estop_raw"]))
    reacted = ui.wait("Waiting for the robot to latch the E-stop ...",
                      lambda: True if any(k in _errors(ctx) for k in ESTOP_KEYS) else None, ROBOT_REACT_S, 0.1)
    if reacted is None:
        out.append(result(st, FAIL, "the inputs read pressed but the robot did not latch ESTOP",
                          hints=["RTI reads the E-stop every tick: is RTI running (S0)? logs/RTI.log"]))
    released = ui.wait("Now release the E-stop", lambda: True if not _io(ctx)["estop_pressed"] else None,
                       PRESS_TIMEOUT_S, POLL_S)
    if released is None:
        out.append(result(st, FAIL, "the E-stop still reads pressed after the release", hints=ESTOP_HINTS))
        return out
    robot.clear_errors(wait=False)
    gone = ui.wait("Clearing the E-stop error ...",
                   lambda: True if not any(k in _errors(ctx) for k in ESTOP_KEYS) else None, 3.0, 0.1)
    if gone is None:
        out.append(result(st, WARN, "the ESTOP error did not clear after the release",
                          hints=["press Clear Errors in the dashboard; if it stays, an E-stop channel still reads "
                                 "pressed"]))
    if not any(r["verdict"] == FAIL for r in out):
        out.insert(0, result(st, PASS, f"pressed: read on {used} channel{'s' if used != 1 else ''} and latched by "
                                       "the robot; released: clear", value=held["estop_raw"]))
    return out


def check_buttons(ctx, ui):
    st = BUTTONS
    out = []
    for name in ("B1", "B2"):
        try:
            got = ui.wait(f"Give the control box's button {name} a SHORT press (it only turns the screen's page; skip "
                          "it if your box has no such button)",
                          lambda n=name: True if _io(ctx)["buttons_pressed"][n] else None, BUTTON_TIMEOUT_S, POLL_S)
        except Cancelled:
            out.append(result(st, SKIP, f"button {name}: skipped by the user"))
            continue
        if got is None:
            out.append(result(st, FAIL, f"button {name}: no press seen in {BUTTON_TIMEOUT_S:.0f} s",
                              hints=[f"{name}'s two wires to the Pi's header (docs/pin_map.md)",
                                     "is the supervisor running? It reads the buttons (logs/supervisor.log)"]))
            continue
        back = ui.wait(f"... and release {name}", lambda n=name: True if not _io(ctx)["buttons_pressed"][n] else None,
                       BUTTON_TIMEOUT_S, POLL_S)
        out.append(result(st, PASS if back else FAIL, f"button {name}: " + ("pressed and released" if back else
                                                                             "stays pressed"),
                          hints=[] if back else [f"{name} is stuck or its wire is shorted to ground"]))
    return out


def check_inputs(ctx, ui):
    st = INPUTS
    if not ui.confirm("Test the isolated inputs? You need a way to switch each one (a 24 V signal or a switch wired to "
                      "it, docs/pin_map.md); you can skip any input that has nothing wired to it."):
        return result(st, SKIP, "not tested (the user's choice)")
    out = []
    for k in range(3):
        base = _io(ctx)["inputs"][k]
        try:
            on = ui.wait(f"Switch isolated input {k + 1} (terminal {INPUT_TERMINALS[k]}; it reads {base} now) — skip it "
                         "if nothing is wired to it",
                         lambda k=k, b=base: True if _io(ctx)["inputs"][k] != b else None, INPUT_TIMEOUT_S, POLL_S)
        except Cancelled:
            out.append(result(st, SKIP, f"input {k + 1}: skipped by the user (not wired)"))
            continue
        if on is None:
            out.append(result(st, FAIL, f"input {k + 1}: no change seen", value=base,
                              hints=[f"input {k + 1}'s wiring and its supply (an isolated input needs its own "
                                     "signal voltage)"]))
            continue
        off = ui.wait(f"Switch input {k + 1} back", lambda k=k, b=base: True if _io(ctx)["inputs"][k] == b else None,
                      INPUT_TIMEOUT_S, POLL_S)
        what = "switches both ways" if off else "changed, did not go back"
        out.append(result(st, PASS if off else WARN, f"input {k + 1}: {what}", value=base))
    return out


def check_outputs(ctx, ui):
    st = OUTPUTS
    if not ui.confirm("Test the isolated outputs? Each switches on for 2 s — have something on it you can see (a "
                      "lamp, a relay, a meter), and nothing that moves; answer 'not wired' for one with nothing on it."):
        return result(st, SKIP, "not tested (the user's choice)")
    why = claim(ctx)
    if why:
        return result(st, SKIP, why)
    robot = ctx["robot"]
    out = []
    for k in range(3):
        seen = "again"
        while seen == "again":
            before = _io(ctx)["outputs"][k]
            try:
                robot.set_digital_output(k + 1, 1)
                ctx["sleep"](OUTPUT_ON_S)
            finally:
                robot.set_digital_output(k + 1, before)
            seen = ui.choose(f"Did output {k + 1} (terminal {OUTPUT_TERMINALS[k]}) switch on for ~2 s?",
                             list(OUTPUT_ANSWERS))
        if seen == "not wired":
            out.append(result(st, SKIP, f"output {k + 1}: not wired (the user)"))
            continue
        out.append(result(st, PASS if seen == "yes" else FAIL, f"output {k + 1}: "
                          + ("switched, seen by the user" if seen == "yes" else "the user saw nothing"),
                          hints=[] if seen == "yes" else [f"output {k + 1}'s wiring and the supply of what it drives "
                                                          "(an isolated output switches an external voltage)"]))
    return out


def _hall(ctx):
    """status()["hall_check"]: {"joint", "pin", "changes", "edge_deg"} (the status stream)."""
    return (ctx["robot"].status() or {}).get("hall_check") or {}


def check_hall(ctx, ui):
    st = HALL
    joint = (ctx.get("config") or {}).get("hall_joint")
    if joint is None:
        return result(st, SKIP, "this robot has no Hall-sensor joint")
    name = f"J{joint + 1}"
    status = ctx["robot"].get_status() or {}
    if status.get("mode") != "IDLE":
        return result(st, SKIP, f"the Hall check needs IDLE (the robot is in {status.get('mode')})")
    why = claim(ctx)
    if why:
        return result(st, SKIP, why)
    robot = ctx["robot"]
    robot.hall_check(True)
    try:
        started = ui.wait("Starting the Hall check ...", lambda: True if _hall(ctx).get("joint") == joint + 1 else None,
                          3.0, 0.1)
        if started is None:
            return result(st, FAIL, "the robot did not start the Hall check", hints=["logs/RTI.log"])
        robot.hall_check(True)          # renews the check's 2 minutes for the wait below (2026-10-24)
        first = ui.wait(f"{name} is limp now (hold the gripper if its weight could turn it). Turn {name} SLOWLY by "
                        "hand through its magnet and a little past it",
                        lambda: (lambda h: h if h.get("changes", 0) >= 2 else None)(_hall(ctx)), HALL_TIMEOUT_S,
                        0.1)
        if first is None:
            h = _hall(ctx)
            stuck = h.get("changes", 0) == 1
            return result(st, FAIL, "the sensor " + ("changed once and stayed" if stuck else "never changed"),
                          value=h, expected="it changes at the magnet and changes back after it",
                          hints=(["the sensor stays triggered: the magnet too close, or the sensor's output shorted"]
                                 if stuck else
                                 ["the sensor's cable to the drive (its connector on the drive)",
                                  "the magnet: present, its gap to the sensor (a few mm), the right pole facing it",
                                  "turn through the whole range once: the magnet may sit elsewhere"]))
        out = [result(st, PASS, f"the sensor changes at the magnet and back ({name} at "
                                f"{_fmt(first.get('edge_deg'))})", value=first)]
        if ui.confirm(f"Check that it triggers at the same place? Turn {name} back past the magnet, then through it "
                      "again the SAME way as before."):
            robot.hall_check(True)      # renewed again: the question above may have taken a while
            again = ui.wait(f"Turn {name} back past the magnet, then through it again the same way",
                            lambda: (lambda h: h if h.get("changes", 0) >= 6 else None)(_hall(ctx)), HALL_TIMEOUT_S,
                            0.1)
            if again is None:
                out.append(result(st, WARN, "the repeat was not completed", value=_hall(ctx)))
            else:
                a, b = (first.get("edge_deg") or [None])[0], (again.get("edge_deg") or [None])[0]
                if a is None or b is None or not (math.isfinite(a) and math.isfinite(b)):
                    out.append(result(st, WARN, "the trigger angle was not recorded both times", value=again))
                else:
                    d = abs(a - b)
                    out.append(result(st, PASS if d <= HALL_REPEAT_DEG else WARN,
                                      f"triggered {d:.2f} deg apart on the two passes", value=round(d, 3),
                                      expected=f"within {HALL_REPEAT_DEG} deg",
                                      hints=[] if d <= HALL_REPEAT_DEG else
                                      ["the magnet or the sensor is loose, or the gap is so large the sensor "
                                       "triggers late; homing relies on this point"]))
        return out
    finally:
        robot.hall_check(False)


def _fmt(edges):
    return ", ".join(f"{e:.1f} deg" for e in (edges or []) if e is not None and math.isfinite(e)) or "?"


ESTOP = step("S2.estop", 2, "The E-stop", check_estop, needs=("robot", "disabled"),
             about="press and release the E-stop: the box reads it and the robot latches it")
BUTTONS = step("S2.buttons", 2, "The box's buttons", check_buttons, needs=("robot", "disabled"),
               about="a short press of B1 and B2")
INPUTS = step("S2.inputs", 2, "Isolated inputs (optional)", check_inputs, needs=("robot", "disabled"),
              about="you switch each isolated input on and off")
OUTPUTS = step("S2.outputs", 2, "Isolated outputs (optional)", check_outputs, needs=("robot", "disabled"),
               about="each isolated output switches on for 2 s; you say whether it did")
HALL = step("S2.hall", 2, "The Hall sensor", check_hall, needs=("robot", "disabled"),
            about="the Hall-sensor joint limp, turned by hand through its magnet")
STEPS = [ESTOP, BUTTONS, INPUTS, OUTPUTS, HALL]
