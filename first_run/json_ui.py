"""The first-run check's UI contract as JSON lines (2026-10-27, TODO/first_run_plan.md P4): what the dashboard's First
run tab (gui_first_run_tab.py) runs, as `tools/first_run.py --ui json`.

Every message to the user is one JSON object on a line of `out` (the process's real stdout); every answer is one on a
line of `inp` (its stdin).

    out  {"t": "stage", "n": 3, "title": ..., "about": ...}                    a stage begins
         {"t": "say", "text": ...}                                           information for the user
         {"t": "ask", "id": 7, "kind": "confirm", "q": ...}                   a yes / no question
         {"t": "ask", "id": 8, "kind": "choose", "q": ..., "options": [...]}  one of the options
         {"t": "wait", "id": 9, "prompt": ..., "timeout_s": 180.0}            waiting for something (the user may skip)
         {"t": "wait_end", "id": 9, "how": "done" | "timed out" | "skipped"}
         {"t": "result", "res": {...}}                                       one result (first_run.core.result())
         {"t": "done", "verdict": ..., "counts": {...}, "completed": ..., "report": folder, "bundle": path | null}

    inp  {"id": 7, "a": true}           the answer to question 7 (a bool for confirm, one of the options for choose)
         {"id": 9, "skip": true}        skip wait 9: the step is skipped (its motion stopped), as Ctrl-C does
         {"stop": true}                 stop the run: a wait in progress is skipped, the next question stops the run
                                        (Aborted) — the report keeps what was done

The end of `inp` (the dashboard went away) is a stop, and so is a broken `out`: the run then ends the way a stop does —
motion stopped, control released, the report written. Waits raise only Cancelled and questions only Aborted, exactly
like the console (tools/first_run.py's ConsoleUI): the steps stop their motion on Cancelled. An answer to anything
but the open question, and a line that is not a JSON object, are ignored.
"""
from __future__ import annotations

import json
import queue
import threading
import time

from first_run.core import Aborted, Cancelled


class JsonUI:
    def __init__(self, inp, out, monotonic=time.monotonic):
        self.out, self.monotonic = out, monotonic
        self.stopping = False
        self._q: queue.Queue = queue.Queue()
        self._lock = threading.Lock()
        self._next_id = 0
        threading.Thread(target=self._read, args=(inp,), name="first-run-answers", daemon=True).start()

    # ---- the two streams
    def _read(self, inp):
        try:
            for line in inp:
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                if isinstance(msg, dict):
                    self._q.put(msg)
        except (OSError, ValueError):
            pass
        self._q.put({"stop": True, "eof": True})

    def emit(self, obj):
        with self._lock:
            try:
                self.out.write(json.dumps(obj, default=str) + "\n")
                self.out.flush()
            except (OSError, ValueError):   # the dashboard is gone: end the run the way a stop does
                self.stopping = True

    def _new_id(self):
        self._next_id += 1
        return self._next_id

    # ---- the UI contract (first_run.core)
    def stage(self, number, title, about):
        self.emit({"t": "stage", "n": number, "title": title, "about": about})

    def say(self, text):
        self.emit({"t": "say", "text": text})

    def result(self, res):
        self.emit({"t": "result", "res": res})

    def finished(self, summary):
        """The run's end (tools/first_run.py): verdict, counts, completed, report, bundle."""
        self.emit(dict(summary, t="done"))

    def _ask(self, kind, question, valid, **extra):
        if self.stopping:
            raise Aborted()
        qid = self._new_id()
        self.emit(dict({"t": "ask", "id": qid, "kind": kind, "q": question}, **extra))
        while True:
            if self.stopping:
                raise Aborted()
            msg = self._q.get()
            if msg.get("stop"):
                self.stopping = True
                raise Aborted()
            if msg.get("id") == qid and "a" in msg and valid(msg["a"]):
                return msg["a"]

    def confirm(self, question) -> bool:
        return bool(self._ask("confirm", question, lambda a: isinstance(a, bool)))

    def choose(self, question, options) -> str:
        options = list(options)
        return self._ask("choose", question, lambda a: a in options, options=options)

    def wait(self, prompt, poll, timeout_s, interval_s=0.1):
        if self.stopping:
            raise Cancelled()
        wid = self._new_id()
        self.emit({"t": "wait", "id": wid, "prompt": prompt, "timeout_s": float(timeout_s)})
        end = self.monotonic() + float(timeout_s)
        while True:
            if self.stopping:               # a stop, or `out` broke while emitting
                self.emit({"t": "wait_end", "id": wid, "how": "skipped"})
                raise Cancelled()
            value = poll()
            if value is not None:
                self.emit({"t": "wait_end", "id": wid, "how": "done"})
                return value
            if self.monotonic() >= end:
                self.emit({"t": "wait_end", "id": wid, "how": "timed out"})
                return None
            try:
                msg = self._q.get(timeout=interval_s)
            except queue.Empty:
                continue
            if msg.get("stop"):
                self.stopping = True
            elif msg.get("id") == wid and msg.get("skip"):
                self.emit({"t": "wait_end", "id": wid, "how": "skipped"})
                raise Cancelled()
