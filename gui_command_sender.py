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
import queue
import threading

from robotclient import RobotClient
from robotclient.constants import DEFAULT_PORT

_QUEUE_MAX = 256          # bound the backlog so a wedged executor can't grow it without limit
_POLL_TIMEOUT = 0.5       # s; how often the idle worker re-checks the stop flag


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
        self._robot = robot if robot is not None else RobotClient()
        self._connected = False
        self._q = queue.Queue(maxsize=_QUEUE_MAX)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, name="gui-exec-sender", daemon=True)
        self._worker.start()

    def submit(self, fn, label=""):
        """Queue fn(robot) for the worker. Never blocks meaningfully, never raises into the
        caller. If the backlog is full (executor wedged), the command is dropped and reported."""
        try:
            self._q.put_nowait((fn, label))
        except queue.Full:
            logging.warning("[GUI] command '%s' dropped: sender backlog full", label)
            self._on_error(label, RuntimeError("command sender backlog full"))

    def close(self):
        self._stop.set()

    # -- worker internals --------------------------------------------------------------------
    def _ensure_connected(self):
        if self._connected and self._robot.connected:
            return True
        try:
            self._robot.connect(self._ip, self._port)
            self._connected = True
            logging.info("[GUI] connected to command_executor at %s:%s", self._ip, self._port)
        except Exception as exc:                       # connection refused / timeout / etc.
            self._connected = False
            logging.warning("[GUI] command_executor connect failed: %s", exc)
        return self._connected

    def _run(self):
        while not self._stop.is_set():
            try:
                fn, label = self._q.get(timeout=_POLL_TIMEOUT)
            except queue.Empty:
                continue
            if not self._ensure_connected():
                self._on_error(label, RuntimeError("command_executor unreachable"))
                continue
            try:
                fn(self._robot)
            except Exception as exc:                    # rejection, timeout, dropped socket
                logging.warning("[GUI] command '%s' failed: %s", label, exc)
                if not self._robot.connected:           # force a reconnect on the next submit
                    self._connected = False
                self._on_error(label, exc)
