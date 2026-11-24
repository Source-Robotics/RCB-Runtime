"""Hang detection: a managed program still running but stuck (review I4, 2026-11-22; the user: "Detect + error, no
restart").

The supervisor already notices a program that EXITS (process_manager.monitor_scripts, with a crash back-off). A program
that is alive but no longer works was invisible. Each program has a liveness signal that its WORKING loop advances —
not a helper thread, which can keep beating while the loop is stuck:

    RTI             rti['loop_tick']        its superloop, 250 Hz in every mode (also IDLE, HOMING, FLASHING)
    HIGH_LEVEL      rti['exec_watch_beat']  the executor's status watcher, 50 Hz (rti['exec_heartbeat'] is a
                                            thread of its own: it beats while the rest of the executor is stuck)
    SELF_COLLISION  rti['sc_heartbeat']     the worker's loop, 100 Hz; 1 Hz while the check is off in system.xml
    RECORDER        the trace head's recorder beat (utility/trace_ring.read_head), 20 Hz once it follows RTI's
                    trace — judged only while RTI's loop ticks (before that the recorder has nothing to follow)
    GUI             an HTTP answer from the dashboard (GET /favicon.ico — a route registered at start; any unknown
                    path would build a whole page), probed every GUI_PROBE_S on a thread of its own
    CAMERA          the camera server's "beat" in its status JSON (shared memory camera_status), a few times a second
                    whether a camera is plugged in or not (2026-11-24, TODO/camera_plan.md D1)

A program is ARMED at the first change of its signal after its start (a change, not an increase: shared memory keeps
the last run's values, and RTI counts from 0 again). Armed and unchanged for its LIMIT, or never armed within its
START limit: hung. Each hang logs one ERROR (with how long, the pid and the process's state), its end one INFO, and
rti['process_hung'] carries it: [0] = how many programs are hung, [1..6] = seconds hung per program in PROGRAMS' order
(0 = fine). RTI's error pass shows P_<NAME>:PROCESS_HUNG_WARNING (a warning: live, gone when the program works again,
nothing stops — a stuck recorder must not stop the arm; the arm's own watchdogs cover RTI's links); the health card
reads the flag itself, so it still shows a stuck RTI. Nothing is restarted.
"""
from __future__ import annotations

import logging
import threading
import time
import urllib.request

logger = logging.getLogger("sup")

# (name in supervisor.SCRIPTS, limit s, start limit s, words) — the order of rti['process_hung'][1..6]
PROGRAMS = (
    ("RTI", 2.0, 180.0, "RTI (the real-time loop)"),           # a cold boot's CAN wait took 51 s before tick 1
    ("HIGH_LEVEL", 10.0, 60.0, "the executor"),                # a stuck client's send can block it 2 s
    ("SELF_COLLISION", 5.0, 60.0, "the self-collision worker"),
    ("RECORDER", 30.0, 60.0, "the recorder"),                  # it saves a 65 s capture inside its loop
    ("GUI", 30.0, 120.0, "the dashboard"),                     # a page build blocks its event loop for seconds
    ("CAMERA", 10.0, 60.0, "the camera server"),               # its loop waits at most 0.2 s; a camera open ~0.5 s
)
GUI_PROBE_S = 10.0
GUI_PROBE_TIMEOUT_S = 2.0


class _Track:
    __slots__ = ("pid", "started", "value", "changed", "armed", "hung")

    def __init__(self, pid, now, value):
        self.pid, self.started, self.value, self.changed, self.armed, self.hung = pid, now, value, now, False, 0.0


class HangCheck:
    """One per supervisor. step(processes) on every monitor pass; `signals` maps a program name to a function returning
    its current signal value (None = not available yet); `flag` is rti['process_hung'] (or a stand-in)."""

    def __init__(self, signals, flag, clock=time.monotonic):
        self.signals, self.flag, self.clock = signals, flag, clock
        self.tracks: dict = {}
        self.zero()

    def zero(self):
        """Nothing hung (the supervisor's start: an old run's flag must not survive in /dev/shm)."""
        for i in range(len(self.flag)):
            self.flag[i] = 0.0

    def step(self, processes) -> dict:
        """One pass. Returns {name: seconds hung} of the hung ones. Never raises."""
        now = self.clock()
        hung = {}
        rti_moving = False
        for slot, (name, limit, start_limit, words) in enumerate(PROGRAMS, start=1):
            proc = processes.get(name)
            seconds = 0.0
            if proc is not None and proc.poll() is None:
                try:
                    value = self.signals[name]() if name in self.signals else None
                except Exception:  # noqa: BLE001 — a signal that cannot be read is no hang
                    value = None
                seconds = self._judge(name, proc, value, now, limit, start_limit, words,
                                      gate=(name != "RECORDER" or rti_moving))
                if name == "RTI":
                    t = self.tracks.get("RTI")
                    rti_moving = bool(t and t.armed and seconds == 0.0)
            else:
                self.tracks.pop(name, None)
            self.flag[slot] = seconds
            if seconds > 0.0:
                hung[name] = seconds
        self.flag[0] = float(len(hung))
        return hung

    def _judge(self, name, proc, value, now, limit, start_limit, words, gate=True):
        pid = getattr(proc, "pid", None)
        t = self.tracks.get(name)
        if t is None or t.pid != pid:
            t = self.tracks[name] = _Track(pid, now, value)
            return 0.0
        if value is not None and value != t.value:
            t.value, t.changed = value, now
            if not t.armed:
                t.armed = True
            if t.hung:
                logger.info(f"{words} works again (pid {pid}): its liveness signal moves after {t.hung:.0f} s "
                            f"(PROCESS_HUNG_WARNING cleared)")
                t.hung = 0.0
            return 0.0
        if not gate:                               # the recorder while RTI does not tick: nothing to follow
            t.changed = now
            if not t.armed:
                t.started = now                    # its start limit counts from when there is something to follow
            return 0.0
        if t.armed:
            stuck = now - t.changed
            over = stuck > limit
        else:
            stuck = now - t.started
            over = stuck > start_limit
        if not over:
            return 0.0
        if not t.hung:
            logger.error(f"{words} is not responding (pid {pid}): "
                         f"{'its liveness signal has not moved for' if t.armed else 'no sign of work since its start,'}"
                         f" {stuck:.0f} s{_proc_state(pid)} — PROCESS_HUNG_WARNING; it is not restarted "
                         f"(make the arm safe, then restart it from the control box or restart the service)")
        t.hung = max(stuck, 0.001)
        return t.hung


def _proc_state(pid):
    """', state sleeping, 3 % CPU' — busy loop or blocked; '' when psutil cannot tell."""
    try:
        import psutil
        p = psutil.Process(pid)
        return f", state {p.status()}, {p.cpu_percent(interval=0.05):.0f} % CPU"
    except Exception:  # noqa: BLE001
        return ""


class GuiProbe:
    """The dashboard's liveness: a counter that moves each time it answered an HTTP request (GUI_PROBE_S apart, on its
    own daemon thread, so a stuck dashboard never delays the supervisor's loop)."""

    def __init__(self, port, interval=GUI_PROBE_S, timeout=GUI_PROBE_TIMEOUT_S):
        self.url = f"http://127.0.0.1:{int(port)}/favicon.ico"
        self.interval, self.timeout = interval, timeout
        self.answers = 0
        self._thread = None

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="gui-probe", daemon=True)
            self._thread.start()
        return self

    def value(self):
        return self.answers

    def _run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=self.timeout) as r:  # noqa: S310  # nosec B310 — our own http://127.0.0.1 URL
                    r.read(64)
                self.answers += 1
            except Exception:  # noqa: BLE001  # nosec B110 — no answer: the counter stands still (that IS the signal)
                pass
            time.sleep(self.interval)
