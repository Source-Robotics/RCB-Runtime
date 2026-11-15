"""First run tab — the first-run check in the dashboard (2026-10-27, TODO/first_run_plan.md P4).

The same program as the command line: the tab starts `tools/first_run.py --ui json` as its own process and shows what
it says — its questions become buttons, its waits a Skip button (first_run/json_ui.py is the protocol). Why a process
and not a thread in the dashboard: a fault in the check cannot take the dashboard down; the run belongs to this module
(NiceGUI's script mode re-runs nicegui_test.py on every page load, the imported modules stay), so a reloaded or
second page finds the run again and can answer it; and the dashboard going away closes the check's input, which the
check treats as a stop — the move in progress stopped, control released, the report written.

  * Start: the stages (0-5), stage 3 by hand or powered, the support bundle at the end (ask / yes / no). One run at a
    time, for every page.
  * Stop: the wait in progress is skipped (its motion stopped), the next question ends the run; the report keeps what
    was done.
  * The transcript: stages, what the check says, its questions and the answers given, every result with its hints
    (WARN / FAIL), the check's own output (stderr, grey: a crash shows here).
  * At the end: the result, and downloads of report.txt / report.json / the support bundle. Earlier reports below.

Self-contained, so the GUI redesign can move it as one piece (TODO/gui_redesign_preparation_plan.md): Session,
row_of() and recent_reports() have no NiceGUI; add_first_run_tab() builds the view.
"""
from __future__ import annotations

import glob
import json
import logging
import os
import subprocess
import sys
import threading
import time

from project_paths import PROJECT_ROOT, TEMP_DATA_DIR

logger = logging.getLogger("gui")

CLI = os.path.join(str(PROJECT_ROOT), "tools", "first_run.py")
REPORTS_DIR = os.path.join(str(TEMP_DATA_DIR), "first_run")
STAGES = {0: "The Pi and the configuration", 1: "Power and CAN", 2: "E-stop, buttons, I/O, Hall sensor",
          3: "Small test moves", 4: "Homing", 5: "The homed arm"}      # first_run.core.STAGE_TITLES (a net compares)
BUNDLE_MODES = {"ask": "ask at the end", "yes": "yes", "no": "no"}
VERDICT_COLOURS = {"PASS": "positive", "WARN": "warning", "FAIL": "negative", "SKIP": "grey", "INFO": "info"}
RECENT = 10


class Session:
    """One run of the check as a child process: its transcript and its open question, for any page.

    events: every message in order — the protocol's (first_run/json_ui.py) plus {"t": "answered", "id", "a"} (an
    answer this tab sent), {"t": "log", "text"} (a line of the check's stderr, or a stdout line that is not JSON) and
    {"t": "exit", "code"} (the process ended). question: the open "ask" / "wait" message, or None."""

    def __init__(self, stages, by_hand=False, bundle="ask", popen=subprocess.Popen, python=sys.executable,
                 cli=CLI):
        self.stages = sorted({int(s) for s in stages})
        self.argv = [python, "-u", cli, "--ui", "json", "--stages", ",".join(str(s) for s in self.stages),
                     "--bundle", bundle] + (["--by-hand"] if by_hand else [])
        self.events: list = []
        self.question = None
        self.done = None
        self.returncode = None
        self.stopping = False
        self.started = time.time()
        self._lock = threading.Lock()
        self.proc = popen(self.argv, cwd=str(PROJECT_ROOT), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True, bufsize=1, start_new_session=True,
                          env=dict(os.environ, PYTHONUNBUFFERED="1"))
        logger.info("first-run check started: stages %s%s, bundle %s (pid %s)", self.stages,
                    " (stage 3 by hand)" if by_hand else "", bundle, getattr(self.proc, "pid", "?"))
        readers = [threading.Thread(target=self._read_out, name="first-run-out", daemon=True),
                   threading.Thread(target=self._read_err, name="first-run-err", daemon=True)]
        for t in readers:
            t.start()
        threading.Thread(target=self._reap, args=(readers,), name="first-run-reap", daemon=True).start()

    @property
    def running(self) -> bool:
        return self.returncode is None

    # ---- what the check says
    def _add(self, ev):
        with self._lock:
            t = ev.get("t")
            if t in ("ask", "wait"):
                self.question = ev
            elif t == "wait_end" and self.question is not None and self.question.get("id") == ev.get("id"):
                self.question = None
            elif t == "done":
                self.done, self.question = ev, None
            self.events.append(ev)

    def _read_out(self):
        for line in self.proc.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                msg = None
            self._add(msg if isinstance(msg, dict) and "t" in msg else {"t": "log", "text": line.rstrip("\n")})

    def _read_err(self):
        for line in self.proc.stderr:
            if line.strip():
                self._add({"t": "log", "text": line.rstrip("\n")})

    def _reap(self, readers):
        code = self.proc.wait()
        for t in readers:
            t.join(timeout=5.0)
        with self._lock:
            self.question = None
            self.events.append({"t": "exit", "code": code})
            self.returncode = code
        logger.info("first-run check ended (exit code %s)%s", code,
                    f": {self.done.get('verdict')}, report {self.done.get('report')}" if self.done else "")

    # ---- what the tab sends
    def _send(self, obj) -> bool:
        try:
            self.proc.stdin.write(json.dumps(obj) + "\n")
            self.proc.stdin.flush()
            return True
        except (OSError, ValueError):   # the check has ended
            return False

    def answer(self, qid, value) -> bool:
        """Answer the open question `qid`; False when it is not open any more (answered on another page, ended)."""
        with self._lock:
            q = self.question
            if q is None or q.get("t") != "ask" or q.get("id") != qid:
                return False
            self.question = None
            self.events.append({"t": "answered", "id": qid, "a": value})
        return self._send({"id": qid, "a": value})

    def skip(self, qid) -> bool:
        """Skip the open wait `qid` (the check stops what it was waiting for and skips the step)."""
        q = self.question
        if q is None or q.get("t") != "wait" or q.get("id") != qid:
            return False
        return self._send({"id": qid, "skip": True})

    def stop(self) -> bool:
        """Stop the run: the wait in progress is skipped, the next question ends it (the report is still written)."""
        with self._lock:
            self.stopping = True
            if self.question is not None and self.question.get("t") == "ask":   # the check ends the run at it
                self.question = None
        return self._send({"stop": True})


_current: dict = {"session": None}


def current():
    """The run every page shows: the one in progress, else the last one (None before the first)."""
    return _current["session"]


def start(stages, by_hand=False, bundle="ask", **kw):
    """Start a run; RuntimeError while one is in progress (one at a time, for every page)."""
    s = _current["session"]
    if s is not None and s.running:
        raise RuntimeError("a first-run check is already running")
    if not stages:
        raise ValueError("choose at least one stage")
    _current["session"] = Session(stages, by_hand=by_hand, bundle=bundle, **kw)
    return _current["session"]


def _yes_no(a):
    return ("yes" if a else "no") if isinstance(a, bool) else str(a)


def row_of(ev, done=None):
    """One transcript row for event `ev` — {"kind", "text", "verdict"?, "hints"?, "sub"?} — or None (not shown).
    kind: stage, say, ask, answer, wait, note, result, log, done, exit. done: the session's "done" message (an exit
    after it is normal and not shown)."""
    t = ev.get("t")
    if t == "stage":
        return {"kind": "stage", "text": f"Stage {ev.get('n')}: {ev.get('title', '')}", "sub": ev.get("about", "")}
    if t == "say":
        return {"kind": "say", "text": str(ev.get("text", ""))}
    if t == "ask":
        return {"kind": "ask", "text": str(ev.get("q", ""))}
    if t == "answered":
        return {"kind": "answer", "text": "answered: " + _yes_no(ev.get("a"))}
    if t == "wait":
        return {"kind": "wait", "text": str(ev.get("prompt", ""))}
    if t == "wait_end":
        how = ev.get("how")
        return None if how == "done" else {"kind": "note", "text": str(how)}
    if t == "result":
        r = ev.get("res") or {}
        v = r.get("verdict", "")
        hints = list(r.get("hints") or []) if v in ("WARN", "FAIL") else []
        exp = r.get("expected")
        return {"kind": "result", "verdict": v, "text": f"{r.get('title', '')}: {r.get('summary', '')}",
                "hints": ([f"expected: {exp}"] if exp is not None and v in ("WARN", "FAIL") else []) + hints}
    if t == "log":
        return {"kind": "log", "text": str(ev.get("text", ""))}
    if t == "done":
        return {"kind": "done", "verdict": ev.get("verdict", ""), "text": summary_text(ev)}
    if t == "exit":
        if done is not None:
            return None
        return {"kind": "exit", "text": f"The check ended without its result (exit code {ev.get('code')}) — the grey "
                                        "lines above say why; logs/gui.log has the start and the end."}
    return None


def summary_text(done) -> str:
    c = done.get("counts") or {}
    return (f"Result: {done.get('verdict', '?')} — " + ", ".join(f"{n} {v}" for v, n in c.items() if n)
            + ("" if done.get("completed", True) else " (stopped before the end)"))


def report_files(folder) -> list:
    """[(label, path)] of a report folder's files that exist: report.txt, report.json, its support bundle."""
    out = [(n, os.path.join(folder, n)) for n in ("report.txt", "report.json")
           if os.path.isfile(os.path.join(folder, n))]
    return out + [("support bundle", p) for p in sorted(glob.glob(os.path.join(folder, "support_*.zip")))]


def recent_reports(root=REPORTS_DIR, n=RECENT) -> list:
    """The newest `n` reports: [{"folder", "name", "verdict", "summary", "files"}] (a folder without a readable
    report.json is listed with verdict "?")."""
    out = []
    for folder in sorted(glob.glob(os.path.join(str(root), "*")), reverse=True):
        if not os.path.isdir(folder):
            continue
        try:
            with open(os.path.join(folder, "report.json")) as f:
                doc = json.load(f)
            done = {"verdict": doc.get("verdict"), "counts": doc.get("counts"),
                    "completed": (doc.get("meta") or {}).get("completed", True)}
        except (OSError, ValueError, AttributeError):
            done = {"verdict": "?", "counts": {}, "completed": True}
        out.append({"folder": folder, "name": os.path.basename(folder), "verdict": done["verdict"],
                    "summary": summary_text(done), "files": report_files(folder)})
        if len(out) >= n:
            break
    return out


# ---------------------------------------------------------------------------------------------------------------
# The tab
# ---------------------------------------------------------------------------------------------------------------
def add_first_run_tab(is_visible):
    """Build the tab inside the current tab panel. is_visible() says whether it is on screen (a question asked while
    it is not raises a notification). Returns its handles (for the net)."""
    from nicegui import ui

    form = {"stages": {k: True for k in STAGES}, "by_hand": False, "bundle": "ask"}
    st = {"session": None, "shown": 0, "question": None, "done_shown": False, "recent_at": 0.0}

    ui.label("First-run check").classes("text-sm font-semibold text-gray-600 uppercase")
    ui.label("A guided check that the robot was built and wired correctly, stage by stage: every step says what it "
             "checks, the measured value and PASS / WARN / FAIL — and, for a WARN or FAIL, what to check. Stages 0-2 "
             "move nothing (the arm DISABLED); stage 3 moves each joint a few degrees, stage 4 homes the arm and "
             "stage 5 moves the homed arm (the arm ENABLED — clear the space, keep a hand on the E-stop). The check "
             "asks you to press Enable / Disable in the bar above when a stage needs it; it never does either itself. "
             "The same check runs from a terminal: tools/first_run.py.").classes("text-xs text-gray-500 mb-2")

    with ui.card().classes("w-full"):
        with ui.row().classes("items-center gap-4 flex-wrap"):
            for k, title in STAGES.items():
                ui.checkbox(f"{k} {title}").bind_value(form["stages"], k).classes("text-sm")
        with ui.row().classes("items-center gap-4 mt-1"):
            ui.switch("Stage 3 by hand (turn each joint yourself, no motor current)").bind_value(form, "by_hand") \
                .classes("text-sm")
            ui.select(BUNDLE_MODES, label="Support bundle at the end").bind_value(form, "bundle").classes("w-48")
        with ui.row().classes("gap-3 mt-2"):
            start_btn = ui.button("Start", icon="play_arrow", on_click=lambda: _start()).props("color=primary size=sm")
            stop_btn = ui.button("Stop the run", icon="stop", on_click=lambda: _stop()) \
                .props("color=negative outline size=sm") \
                .tooltip("The wait in progress is skipped (its move stopped), the next question ends the run; the "
                         "report keeps what was done. To stop the ARM at once: Disable or the E-stop.")
        status_lbl = ui.label("").classes("text-xs text-gray-500 font-mono")

    question_card = ui.card().classes("w-full mt-3 border-2 border-orange-400")
    question_card.visible = False

    with ui.card().classes("w-full mt-3"):
        ui.label("Transcript").classes("text-xs font-semibold text-gray-600 uppercase")
        scroll = ui.scroll_area().classes("w-full h-[55vh]")
        with scroll:
            transcript = ui.column().classes("w-full gap-1")

    result_card = ui.card().classes("w-full mt-3")
    result_card.visible = False

    with ui.expansion("Earlier reports", icon="folder").classes("w-full mt-3") as recent_box:
        recent_col = ui.column().classes("w-full gap-1")

    def _start():
        stages = [k for k, on in form["stages"].items() if on]
        try:
            start(stages, by_hand=bool(form["by_hand"]), bundle=form["bundle"])
        except (RuntimeError, ValueError) as e:
            ui.notify(str(e), type="warning")
            return
        except OSError as e:
            ui.notify(f"The check could not start: {e}", type="negative")
            return
        refresh()

    def _stop():
        s = current()
        if s is not None and s.running:
            s.stop()
            ui.notify("Stopping the run: the wait in progress is skipped, the next question ends it.", type="info")

    def _answer(qid, value):
        s = current()
        if s is None or not s.answer(qid, value):
            ui.notify("That question is not open any more.", type="info")
        refresh()

    def _skip(qid):
        s = current()
        if s is not None:
            s.skip(qid)

    def _render_row(row):
        kind = row["kind"]
        with transcript:
            if kind == "stage":
                ui.label(row["text"]).classes("text-base font-bold mt-3")
                if row.get("sub"):
                    ui.label(row["sub"]).classes("text-xs text-gray-500")
            elif kind == "result":
                with ui.row().classes("items-start gap-2 no-wrap w-full"):
                    ui.badge(row["verdict"], color=VERDICT_COLOURS.get(row["verdict"], "grey")).classes("mt-0.5")
                    with ui.column().classes("gap-0"):
                        ui.label(row["text"]).classes("text-sm")
                        for h in row.get("hints", []):
                            ui.label(("" if h.startswith("expected:") else "check: ") + h) \
                                .classes("text-xs text-gray-600 ml-2")
            elif kind == "ask":
                ui.label("? " + row["text"]).classes("text-sm font-semibold text-orange-800")
            elif kind == "answer":
                ui.label(row["text"]).classes("text-xs text-gray-600 ml-4")
            elif kind == "wait":
                ui.label("> " + row["text"]).classes("text-sm text-blue-800")
            elif kind == "note":
                ui.label(row["text"]).classes("text-xs text-gray-500 ml-4")
            elif kind == "log":
                ui.label(row["text"]).classes("text-xs font-mono text-gray-400 whitespace-pre-wrap")
            elif kind in ("done", "exit"):
                ui.label(row["text"]).classes("text-base font-bold mt-3"
                                              + (" text-negative" if kind == "exit" else ""))
            else:
                ui.label(row["text"]).classes("text-sm")

    def _show_question(q):
        question_card.clear()
        question_card.visible = q is not None
        if q is None:
            return
        with question_card:
            if q.get("t") == "ask":
                ui.label(str(q.get("q", ""))).classes("text-base font-semibold")
                with ui.row().classes("gap-2 mt-2 flex-wrap"):
                    if q.get("kind") == "confirm":
                        ui.button("Yes", on_click=lambda: _answer(q["id"], True)).props("color=primary")
                        ui.button("No", on_click=lambda: _answer(q["id"], False)).props("outline")
                    else:
                        for opt in q.get("options") or []:
                            ui.button(str(opt), on_click=lambda o=opt: _answer(q["id"], o)).props("no-caps outline")
            else:
                with ui.row().classes("items-center gap-3 no-wrap"):
                    ui.spinner(size="md")
                    ui.label(str(q.get("prompt", ""))).classes("text-base")
                ui.label(f"up to {float(q.get('timeout_s', 0)):.0f} s").classes("text-xs text-gray-500")
                ui.button("Skip", icon="skip_next", on_click=lambda: _skip(q["id"])).props("outline size=sm") \
                    .tooltip("Skip this step (a move in progress is stopped)")
        if q.get("t") == "ask" and not is_visible():    # a question needs the user; a wait mostly a move (stage 5: dozens)
            ui.notify("The first-run check asks you something (First run tab).", type="warning")

    def _show_result(s):
        result_card.clear()
        result_card.visible = s.done is not None
        if s.done is None:
            return
        folder = s.done.get("report")
        with result_card:
            ui.label(summary_text(s.done)).classes("text-base font-bold")
            if folder:
                ui.label(f"Report: {folder}").classes("text-xs font-mono text-gray-500")
                with ui.row().classes("gap-2"):
                    for label, path in report_files(folder):
                        ui.button(label, icon="download", on_click=lambda p=path: ui.download(p)) \
                            .props("outline size=sm no-caps")

    def _show_recent():
        recent_col.clear()
        with recent_col:
            reports = recent_reports()
            if not reports:
                ui.label("No reports yet.").classes("text-xs text-gray-500")
            for rep in reports:
                with ui.row().classes("items-center gap-2"):
                    ui.badge(rep["verdict"], color=VERDICT_COLOURS.get(rep["verdict"], "grey"))
                    ui.label(f"{rep['name']} UTC — {rep['summary']}").classes("text-xs font-mono")
                    for label, path in rep["files"]:
                        ui.button(label, on_click=lambda p=path: ui.download(p)).props("flat dense size=sm no-caps")

    def refresh():
        s = current()
        if s is not st["session"]:              # a new run (or the page's first look at one): render from the start
            st.update(session=s, shown=0, question=None, done_shown=False)
            transcript.clear()
            _show_question(None)
            result_card.visible = False
        running = s is not None and s.running
        start_btn.set_enabled(not running)
        stop_btn.set_enabled(running and not s.stopping)
        if s is None:
            status_lbl.text = "No run yet."
        else:
            status_lbl.text = (("running" + (" — stopping" if s.stopping else "")) if running else
                               f"ended (exit code {s.returncode})") + \
                f"; stages {','.join(str(k) for k in s.stages)}; started " \
                f"{time.strftime('%H:%M:%S', time.localtime(s.started))}"
        if s is not None:
            new = s.events[st["shown"]:]
            st["shown"] += len(new)
            for ev in new:
                row = row_of(ev, s.done)
                if row is not None:
                    _render_row(row)
            if new:
                scroll.scroll_to(percent=1.0)
            q = s.question
            if (q or {}).get("id") != (st["question"] or {}).get("id") or (q is None) != (st["question"] is None):
                st["question"] = q
                _show_question(q)
            if s.done is not None and not st["done_shown"]:
                st["done_shown"] = True
                _show_result(s)
                st["recent_at"] = 0.0
        if recent_box.value and time.monotonic() - st["recent_at"] > 10.0:
            st["recent_at"] = time.monotonic()
            _show_recent()

    recent_box.on_value_change(lambda e: (st.update(recent_at=0.0), refresh()))
    ui.timer(0.25, refresh)
    return {"form": form, "state": st, "refresh": refresh, "start": _start, "stop": _stop, "answer": _answer,
            "skip": _skip, "transcript": transcript, "question_card": question_card, "result_card": result_card,
            "status": status_lbl, "start_btn": start_btn, "stop_btn": stop_btn}
