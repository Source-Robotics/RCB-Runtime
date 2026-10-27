"""Serialize GUI commands onto command_executor from one background thread.

The live GUI used to PUSH commands straight to RTI's ZMQ_CMD_SOCKET, bypassing command_executor
(no validation, no arbitration). This routes them through the executor's TCP JSON protocol via a
RobotClient instead -- see TODO/gui_executor_migration_plan.md.

The RobotClient calls block on the executor ack and raise on rejection/timeout, which a GUI event
handler must not do. So handlers stay trivial -- they submit(fn) and return -- while ONE worker
thread owns the RobotClient and:

  - calls each fn(robot) in submission ORDER, so a jog_stop can never overtake the jog_set that
    preceded it (a real hazard with thread-per-call), and rapid slider drags simply queue;
  - connects lazily and RECONNECTS if the executor restarts (supervisor may bounce it), so the
    GUI recovers without a restart;
  - never lets an exception escape -- failures go to the on_error callback (the GUI logs and
    shows a toast), exactly the feedback the old fire-and-forget push could never give.

This module has no NiceGUI dependency so it can be unit-tested without a UI.
"""
import logging
logger = logging.getLogger("gui")  # docs/logging.md "Logger names"
import threading
from collections import deque

from robotclient import RobotClient
from robotclient.constants import DEFAULT_PORT

_QUEUE_MAX = 256          # bound the backlog so a wedged executor can't grow it without limit
_POLL_TIMEOUT = 0.5       # s; how often the idle worker re-checks the stop flag
_STOP_LABELS = frozenset({"jog_stop", "gripper_stop", "stop", "disable", "safety_stop"})


# How long the DASHBOARD may sit idle holding control before it lets go. The executor's own idle
# timeout (30 s) exists mainly because of the dashboard — one permanent connection that would
# otherwise own the robot forever after a single click — and 30 s is a long time to be locked out
# after clicking Enable. A person is idle between clicks by nature; a script is not.
GUI_IDLE_RELEASE_S = 5.0


def auto_release_due(state, idle_s, jog_active=False, rti_active=False,
                     threshold=GUI_IDLE_RELEASE_S):
    """Should the dashboard hand control back now? Pure, so the rules can be tested.

    Only ever releases what THIS connection holds, and never while anything it started is still
    running: a queued program (the executor would keep the claim anyway; releasing would let a
    second client interleave commands into the same queue), a held jog button, or a live RTI
    session. Releasing is otherwise harmless: `jog_stop`, `stop`, `disable` and `safety_stop` are
    never gated, so nothing that is already moving depends on holding control to be stopped.
    """
    if not state.get("mine"):
        return False
    if state.get("busy") or jog_active or rti_active:
        return False
    return idle_s >= threshold


class ExecCommandSender:
    """Single-threaded, ordered, self-reconnecting command channel to command_executor.

    Usage:
        sender = ExecCommandSender(on_error=lambda label, exc: ...)
        sender.submit(lambda r: r.enable(), label="enable")
    """

    def __init__(self, ip="127.0.0.1", port=DEFAULT_PORT, on_error=None, robot=None):
        self._ip = ip
        self._port = port
        self._on_error = on_error or (lambda label, exc: None)
        self._robot = robot if robot is not None else RobotClient(name="dashboard")   # its name in the activity log
        self._connected = False
        self._pending = deque()
        self._pending_cv = threading.Condition()
        self._queue_max = _QUEUE_MAX
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, name="gui-exec-sender", daemon=True)
        self._worker.start()

    def submit(self, fn, label=""):
        """Queue fn(robot) without blocking the UI.

        Stops discard stale pending actions and follow any command already in flight.
        Other commands still use the bounded FIFO and report a full backlog.
        """
        full = False
        closed = False
        superseded = 0
        with self._pending_cv:
            if self._stop.is_set():
                closed = True
            elif label in _STOP_LABELS:
                kept = deque()
                for pending_fn, pending_label in self._pending:
                    if pending_label in _STOP_LABELS and pending_label != label:
                        kept.append((pending_fn, pending_label))
                    elif pending_label not in _STOP_LABELS:
                        superseded += 1
                self._pending = kept
                self._pending.append((fn, label))
                self._pending_cv.notify()
            elif len(self._pending) >= self._queue_max:
                full = True
            else:
                self._pending.append((fn, label))
                self._pending_cv.notify()
        if full:
            logger.warning("command '%s' dropped: sender backlog full", label)
            self._on_error(label, RuntimeError("command sender backlog full"))
        if closed:
            self._on_error(label, RuntimeError("command sender closed"))
        if superseded:
            logger.warning("%d queued GUI command(s) superseded by '%s'", superseded, label)
            self._on_error("sender", RuntimeError(f"{superseded} queued command(s) superseded by {label}"))

    def submit_async(self, fn, label=""):
        """Run fn(robot) on its OWN thread, outside the ordered queue.

        For a call that blocks for a long time ON PURPOSE. A forced takeover waits up to 30 s
        for someone to answer at the control box; on the ordered worker that would hold every
        later command behind it — including Disable — for the whole wait. This shares the same
        connection (ownership is per connection, so it must) but not the worker: RobotClient
        serialises its sends and matches replies by id, so concurrent calls are safe, and the
        executor now answers a takeover from a helper thread so it does not stall the
        connection either.

        Do NOT use this for commands whose ORDER matters (jog_set / jog_stop): submit() exists
        precisely so a stop can never overtake the start that preceded it.
        """
        def _one():
            if self._stop.is_set():
                return
            if not (self._connected and self._robot.connected):
                self._on_error(label, RuntimeError("command_executor unreachable"))
                return
            try:
                fn(self._robot)
            except Exception as exc:
                logger.warning("command '%s' failed: %s", label, exc)
                self._on_error(label, exc)
        threading.Thread(target=_one, name=f"gui-exec-{label or 'async'}", daemon=True).start()

    def control_state(self):
        """Who holds control, from this connection's point of view — for a status display.

        {"connected", "holder", "mine", "other"}. `other` is True only when SOMEONE ELSE
        demonstrably holds it: not connected, or nobody holding, both read as False, so a UI
        built on this can never grey itself out just because the link is down. Reads the
        status stream the executor already pushes on every change; no polling of the robot.
        """
        r = self._robot
        if not (self._connected and r.connected):
            return {"connected": False, "holder": None, "mine": False, "busy": False, "other": False}
        try:
            holder = r.control_holder()
            mine = holder is not None and holder == r.local_label()
        except Exception:
            return {"connected": False, "holder": None, "mine": False, "busy": False, "other": False}
        try:
            pl = r.status().get("planner") or {}
            busy = bool((pl.get("queue_depth") or 0) > 0 or pl.get("active_command"))
        except Exception:
            busy = True            # cannot tell -> assume busy: never release on a guess
        return {"connected": True, "holder": holder, "mine": mine, "busy": busy,
                "other": holder is not None and not mine}

    def close(self):
        self._stop.set()
        with self._pending_cv:
            self._pending_cv.notify_all()

    @property
    def closed(self):
        return self._stop.is_set()

    def abort(self):
        """End this page's channel at once, cancelling pending sends.

        Used when a browser disappears mid-jog. Closing its TCP connection lets
        the executor stop only a jog that this connection actually started.
        A reconnect gets a new sender; no queued jog can revive after the close.
        """
        self._stop.set()
        with self._pending_cv:
            self._pending.clear()
            self._pending_cv.notify_all()
        self._robot.disconnect()

    # -- worker internals --------------------------------------------------------------------
    def _ensure_connected(self):
        if self._connected and self._robot.connected:
            return True
        try:
            self._robot.connect(self._ip, self._port)
            self._connected = True
            logger.info("connected to command_executor at %s:%s", self._ip, self._port)
        except Exception as exc:                       # connection refused / timeout / etc.
            self._connected = False
            logger.warning("command_executor connect failed: %s", exc)
        return self._connected

    def _run(self):
        while not self._stop.is_set():
            with self._pending_cv:
                if not self._pending and not self._stop.is_set():
                    self._pending_cv.wait(timeout=_POLL_TIMEOUT)
                if self._stop.is_set():
                    break
                if not self._pending:
                    continue
                fn, label = self._pending.popleft()
            if not self._ensure_connected():
                self._on_error(label, RuntimeError("command_executor unreachable"))
                continue
            if self._stop.is_set():
                self._robot.disconnect()        # abort may have raced a slow connect
                break
            try:
                fn(self._robot)
            except Exception as exc:                    # rejection, timeout, dropped socket
                logger.warning("command '%s' failed: %s", label, exc)
                if not self._robot.connected:           # force a reconnect on the next submit
                    self._connected = False
                self._on_error(label, exc)
