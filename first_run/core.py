"""The first-run check's core: what a step is, what it returns, the runner, and the UI contract (2026-10-18).

A STEP is a dict:

    {"id": "S1.nodes", "stage": 1, "title": "Every drive answers", "needs": ("robot", "disabled"),
     "about": "what it does, in a sentence", "run": fn}

`run(ctx, ui)` returns one result or a list of them (`result()`); it raises Cancelled when the user skips it. `ctx` is
a plain dict the steps share: "connect" (-> a RobotClient), "robot" (set by S0.service), "config" (set by
S0.config), "hardware" (the last get_hardware() read), "sleep", "monotonic", "run" (subprocess.run). Needs:

    "robot"     a connection to the robot's executor (S0.service made one)
    "disabled"  the arm DISABLED — the runner asks the user to disable it and waits, or skips the step
    "enabled"   the arm ENABLED (powered moves) — the runner asks the user to enable it and waits, or skips the step

A step that raises Aborted may attach the results it already has as `e.partial`: the runner keeps them (and a SKIP for
the rest), so stopping the run mid-step loses nothing that was measured.

THE UI CONTRACT — what a front-end (tools/first_run.py's console, the dashboard later, a test's script) provides:

    ui.stage(number, title, about)      a stage begins
    ui.say(text)                        information for the user
    ui.confirm(question) -> bool        yes / no
    ui.choose(question, options) -> str one of options
    ui.wait(prompt, poll, timeout_s, interval_s=0.1) -> value | None
                                        show `prompt`, call poll() every interval_s until it returns something other
                                        than None (returned) or timeout_s passed (None); the user can skip (Cancelled)
    ui.result(res)                      one result line

and raises Cancelled (skip this step) or Aborted (stop the whole run) when the user asks to.
"""
from __future__ import annotations

import traceback

PASS, WARN, FAIL, SKIP, INFO = "PASS", "WARN", "FAIL", "SKIP", "INFO"
VERDICTS = (PASS, WARN, FAIL, SKIP, INFO)
STAGE_TITLES = {0: "The Pi and the configuration", 1: "Power and CAN", 2: "E-stop, buttons, I/O, Hall sensor",
                3: "Small test moves"}
STAGE_ABOUT = {
    0: "Nothing needs power yet: the Pi's setup, the configuration files, the control box version, the robot service.",
    1: "The drives powered, the arm DISABLED: every drive answers, the CAN bus is clean, firmware, supply voltage, "
       "temperatures and the drives' own fault flags.",
    2: "Nothing moves: you press the E-stop and the box's buttons, optionally switch the isolated inputs and watch "
       "the outputs, and turn the Hall-sensor joint by hand.",
    3: "The arm ENABLED, homed or not: each joint moves a few degrees and back, slowly, its force limited, the "
       "others holding; you say which way it went. (--by-hand: you turn each joint yourself instead.)",
}
DISABLE_WAIT_S = 180.0


class Cancelled(Exception):
    """The user skipped the step."""


class Aborted(Exception):
    """The user stopped the whole run."""


def step(sid, stage, title, run, needs=(), about="") -> dict:
    return {"id": sid, "stage": int(stage), "title": title, "run": run, "needs": tuple(needs), "about": about}


def result(st, verdict, summary, value=None, expected=None, hints=(), details=None) -> dict:
    """One finding of step `st`: the verdict, one line saying why, the measured value and what was expected, and —
    for a WARN / FAIL — what to check, most likely first."""
    if verdict not in VERDICTS:
        raise ValueError(f"unknown verdict {verdict!r}")
    return {"id": st["id"], "stage": st["stage"], "title": st["title"], "verdict": verdict, "summary": summary,
            "value": value, "expected": expected, "hints": list(hints), "details": dict(details or {})}


def worst(results) -> str:
    """FAIL over WARN over the rest; PASS for none."""
    verdicts = {r["verdict"] for r in results}
    return FAIL if FAIL in verdicts else WARN if WARN in verdicts else PASS


def counts(results) -> dict:
    return {v: sum(1 for r in results if r["verdict"] == v) for v in VERDICTS}


def claim(ctx):
    """None once this connection holds control (the owned commands: outputs, hall_check, free_joint), else why not."""
    try:
        ctx["robot"].take_control()
    except Exception as e:   # CONTROL_HELD: another client holds it
        return (f"another client holds control ({getattr(e, 'message', e)}) — the dashboard keeps it ~5 s after its "
                "last command: wait, or release it there, then run this step again")
    return None


def _arm_state(robot) -> str:
    return str((robot.get_status() or {}).get("state", ""))


def _meet_needs(st, ctx, ui):
    """None when the step may run, else the reason it is skipped. 'disabled': asks the user to disable the arm and
    waits for it (never disables it itself: an arm that drops on a disable must be supported first)."""
    robot = ctx.get("robot")
    if ("robot" in st["needs"] or "disabled" in st["needs"] or "enabled" in st["needs"]) and robot is None:
        return "no connection to the robot (S0's 'Robot service' step)"
    if "enabled" in st["needs"] and _arm_state(robot) != "ENABLED":
        got = ui.wait("The arm is DISABLED. Clear the space around it, then press Enable in the dashboard (it holds "
                      "where it is; nothing moves yet) — waiting for the arm to be enabled",
                      lambda: True if _arm_state(robot) == "ENABLED" else None, DISABLE_WAIT_S, 0.5)
        if got is None:
            return "the arm stayed disabled (this step needs it enabled)"
    if "disabled" in st["needs"] and _arm_state(robot) == "ENABLED":
        got = ui.wait("The arm is ENABLED. Support it (or park it), then press Disable in the dashboard — "
                      "waiting for the arm to be disabled", lambda: True if _arm_state(robot) != "ENABLED" else None,
                      DISABLE_WAIT_S, 0.5)
        if got is None:
            return "the arm stayed enabled (this step needs it disabled)"
    return None


def run_step(st, ctx, ui) -> list:
    """Run one step; never raises but Aborted. A crash of the check itself is a FAIL naming it (and logged)."""
    try:
        why = _meet_needs(st, ctx, ui)
        if why is not None:
            return [result(st, SKIP, why)]
        out = st["run"](ctx, ui)
    except Cancelled:
        return [result(st, SKIP, "skipped by the user")]
    except Aborted as e:      # keep what the step measured before the stop, and say where it stopped
        e.results = list(getattr(e, "partial", None) or []) + [result(st, SKIP, "the run was stopped here")]
        raise
    except Exception as e:   # a broken check must not end the run: it is reported as a FAIL with its traceback
        return [result(st, FAIL, f"the check itself failed: {type(e).__name__}: {e}",
                       hints=["this is a fault of the check, not necessarily of the robot — send the report"],
                       details={"traceback": traceback.format_exc()})]
    return list(out) if isinstance(out, (list, tuple)) else [out]


def run(steps, ctx, ui, stages=None) -> tuple:
    """Run `steps` stage by stage (only `stages` if given). A stage with a FAIL asks whether to go on to the next.
    Returns (results, completed: bool) — completed False when the user stopped the run."""
    results: list = []
    wanted = sorted({s["stage"] for s in steps} if stages is None else set(stages))
    for k, stage in enumerate(wanted):
        ui.stage(stage, STAGE_TITLES.get(stage, f"Stage {stage}"), STAGE_ABOUT.get(stage, ""))
        stage_results: list = []
        try:
            for st in (s for s in steps if s["stage"] == stage):
                for res in run_step(st, ctx, ui):
                    ui.result(res)
                    stage_results.append(res)
        except Aborted as e:
            return results + stage_results + list(getattr(e, "results", None) or []), False
        results += stage_results
        if k + 1 < len(wanted) and worst(stage_results) == FAIL:
            try:
                go_on = ui.confirm(f"Stage {stage} has a FAIL — fix it first (the hints say what to check). "
                                   f"Go on to stage {wanted[k + 1]} anyway?")
            except Aborted:
                go_on = False
            if not go_on:
                return results, False
    return results, True
