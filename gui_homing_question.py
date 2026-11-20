"""Homing's question in the dashboard (2026-11-16, user: "pause and ask"; robotics/homing/controller.py).

When a joint homes more than 1 deg from where the recognised arm's joint always homes, homing no longer fails: it holds
the arm and waits for an answer — RTI's rti['homing_question'] ([0] = 1 while it waits, [1 + j] = joint j's deg off,
NaN when not asked). This module shows that question as a dialog on every dashboard page and sends the answer:
"Accept as new reference" (robot.accept_homing_reference()) or "Stop homing" (robot.stop(): the homing ends, not
homed). A "Home J<n> again" answer (re-home the joint where it stood) was removed the same day: J6 homes to a Hall
sensor, and re-homed from its sensor it turned ~0.9 turn and wound its cables.

The dialog is non-modal (Quasar `seamless`) and sits at the bottom of the page: the top bar's E-STOP, Stop and Disable
stay reachable while it waits — either ends the homing (the question goes with it).
"""
from __future__ import annotations

import html
import os
import re

import robotics.homing_arms as ha


def md_plain(text) -> str:
    """`text` shown as itself inside ui.markdown (review G9, 2026-11-20): HTML escaped, Markdown's special characters
    backslash-escaped — an arm named left_arm_2 rendered in italics, and raw HTML in a name passed through."""
    return re.sub(r"([\\`*_{}\[\]()#+\-.!|~])", r"\\\1", html.escape(str(text), quote=False))


def question_now(rti, joint_num: int) -> dict:
    """{joint index: deg off} while homing asks, {} otherwise (also when the field does not exist yet: a dashboard
    started before the field existed)."""
    q = rti.get("homing_question") if isinstance(rti, dict) else None
    if q is None or float(q[0]) < 0.5:
        return {}
    return {j: float(d) for j, d in enumerate(q[1:1 + joint_num]) if d == d}


def arm_name(path) -> str:
    """The arm the question is about: the arm table's last recorded fault (check_so_far records it when it asks)."""
    try:
        table = ha.load_table(path)
        last = table.get("last_failed") or {}
        arm = next((a for a in table["arms"] if a["id"] == last.get("arm")), None)
        return f"arm {md_plain(ha.arm_label(arm))}" if arm else "this arm"     # shown in Markdown (review G9)
    except Exception:       # noqa: BLE001 — a name is a nicety; the question still shows
        return "this arm"


def question_text(question: dict, arm: str) -> str:
    """The dialog's text (Markdown)."""
    joints = ", ".join(f"J{j + 1}" for j in sorted(question))
    lines = "\n".join(f"- **J{j + 1}** homed **{d:+.2f}°** from where {arm} always homes it."
                      for j, d in sorted(question.items()))
    return (f"{lines}\n\n"
            f"**Changed something on purpose** (the gripper re-fitted, a belt or a drive replaced)? "
            f"**Accept as new reference**: the robot keeps this spot for {joints} and homing goes on.\n\n"
            f"**Otherwise** it may be a false stall (something in the way), a skipped belt or a slipping coupling: "
            f"**Stop homing** — the arm stays where it is, not homed. Check {joints}, bring the arm to its park pose "
            f"and press Home.\n\n"
            f"The arm holds meanwhile.")


def add_homing_question(shared_data, joint_num: int, exec_fn, path=None):
    """The dialog for one dashboard page. Returns refresh() — call it on a timer. exec_fn(fn, label) sends a
    RobotClient call (nicegui_test._exec)."""
    from nicegui import ui
    if path is None:
        from project_paths import TEMP_DATA_DIR
        path = os.path.join(str(TEMP_DATA_DIR), "homing_arms.json")
    state = {"dlg": None, "shown": None}

    def _close():
        if state["dlg"] is not None:
            state["dlg"].close()
            state["dlg"] = None

    def _answer(fn, label):
        exec_fn(fn, label)
        state["shown"] = None
        _close()

    def refresh():
        question = question_now(shared_data.rti, joint_num)
        if not question:
            state["shown"] = None
            _close()
            return
        key = tuple(sorted((j, round(d, 2)) for j, d in question.items()))
        if state["shown"] == key:
            return
        _close()
        state["shown"] = key
        with ui.dialog().props("seamless persistent position=bottom") as dlg, \
                ui.card().classes("max-w-2xl border-2 border-orange-500"):
            ui.label("Homing asks — it waits for your answer").classes("text-lg font-bold text-orange-700")
            ui.markdown(question_text(question, arm_name(path))).classes("text-sm")
            with ui.row().classes("gap-2 justify-end w-full mt-2"):
                ui.button("Stop homing", icon="stop",
                          on_click=lambda: _answer(lambda r: r.stop(), "stop")).props("outline")
                ui.button("Accept as new reference", icon="check",
                          on_click=lambda: _answer(lambda r: r.accept_homing_reference(wait=False),
                                                   "accept_homing_reference")).props("color=orange")
        state["dlg"] = dlg
        dlg.open()

    return refresh
