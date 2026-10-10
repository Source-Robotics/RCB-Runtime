"""TCP connection with a background receiver thread.

Exactly one thread reads the socket. It matches each reply to the pending command by
``id`` and signals that command's events; replies without an ``id`` are unsolicited
events routed to callbacks. This lets multiple caller threads issue commands and block
on their own results without ever reading the socket directly.

A queued motion command receives TWO replies: ``{"status":"queued"}`` (ack) then later
``{"status":"complete"}``. Immediate commands receive one terminal reply (``ok``/``error``).
"""

import json
import socket
import threading
import time

from .constants import CONNECT_TIMEOUT
from .exceptions import RobotConnectionError, RobotTimeout, CommandRejected


class _Pending:
    """Tracks the lifecycle of one in-flight command id."""
    __slots__ = ("ack", "done", "data", "error", "queued")

    def __init__(self):
        self.ack = threading.Event()    # first reply seen (queued / ok / error)
        self.done = threading.Event()   # terminal reply seen (ok / complete / error)
        self.data = None
        self.error = None               # (code, message) or None
        self.queued = False


class Connection:
    def __init__(self):
        self._sock = None
        self._recv_thread = None
        self._connected = False

        self._pending = {}                     # id -> _Pending
        self._lock = threading.Lock()          # guards _pending / _outstanding
        self._send_lock = threading.Lock()     # serialize sendall
        self._outstanding = set()              # queued motion ids not yet complete
        self._idle_cv = threading.Condition()  # notified when _outstanding shrinks

        self._rejected = False                 # executor refused us on the access list
        self._status = {}                      # latest {"type":"status",...} from the robot
        self._status_cv = threading.Condition()  # notified on every status event

        # Failures of commands that were already acked as "queued". In async mode
        # _queued() has long since returned, so nothing is waiting on their _Pending
        # and the error would otherwise be silently dropped. Held here until
        # take_motion_errors() (wait_idle) reports them.
        self._motion_errors = []               # [{"id", "code", "message"}, ...]

        # (command_id, object_detection) of the most recently SETTLED gripper
        # command. Gripper completions are the one queued result that carries a
        # payload, because "complete" alone cannot distinguish a pick from a
        # close on empty air.
        self._last_gripper = None

        self.on_error = None                   # callback(dict) for unsolicited errors
        self.on_status_change = None           # callback(dict) for unsolicited status
        self.on_motion_complete = None         # callback(command_id) on queued-command success
        self.on_motion_error = None            # callback(dict) on queued-command failure

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def remote_ip(self):
        """Robot's IP, or None if not connected. RTIClient needs it to aim UDP."""
        if self._sock is None:
            return None
        return self._sock.getpeername()[0]

    def local_ip(self):
        """Our IP *on the route to the robot*, or None if not connected.

        Taken from the live socket rather than a hostname lookup, so it is correct on
        a multi-homed PC — the robot must be told the interface it can actually
        reach us on, not whichever address resolves first.
        """
        if self._sock is None:
            return None
        return self._sock.getsockname()[0]

    def local_label(self):
        """Our address exactly as the executor labels this connection ("ip:port"), or None.

        The executor identifies a client by its TCP connection and publishes the holder as
        that connection's peer address, so "is control mine?" is a string comparison against
        this. Taken from the live socket, not a hostname lookup, for the same reason as
        local_ip(). (Direct connections only: behind a NAT the server sees the translated
        address and this will not match — the answer is then "not mine", the safe side.)
        """
        if self._sock is None:
            return None
        try:
            ip, port = self._sock.getsockname()[:2]
        except OSError:
            return None
        return f"{ip}:{port}"

    def connect(self, ip, port, timeout=CONNECT_TIMEOUT):
        try:
            self._sock = socket.create_connection((ip, port), timeout=timeout)
        except OSError as e:
            raise RobotConnectionError(f"could not connect to {ip}:{port}: {e}")
        self._sock.settimeout(None)
        self._connected = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True,
                                              name="RobotClientReceiver")
        self._recv_thread.start()
        # The TCP handshake succeeding does NOT mean we are allowed in: the executor accepts
        # the socket, checks the source IP, then sends FORBIDDEN and closes. Give that one
        # line time to arrive so the failure is reported HERE, at its cause, rather than as
        # a puzzling silence several calls later.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if self._rejected:
                self.disconnect()
                raise RobotConnectionError(
                    f"{ip} refused this machine: not in the executor's access list. "
                    f"Restart command_executor.py on the robot with "
                    f"RCB_ALLOWED_IPS=<this-pc-ip> (comma-separated for several). Do NOT "
                    f"edit ALLOWED_IPS in the source -- it is tracked in a public repo.")
            if self._status:
                break
            time.sleep(0.02)

    def disconnect(self):
        self._connected = False
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None
        self._fail_all("DISCONNECTED", "connection closed")

    @property
    def connected(self):
        return self._connected

    # ------------------------------------------------------------------ #
    # Sending
    # ------------------------------------------------------------------ #

    def send_command(self, cid, cmd, args=None, options=None):
        """Register a pending entry and send {"id","cmd","args", optional "options"}."""
        if not self._connected:
            raise RobotConnectionError("not connected")
        p = _Pending()
        with self._lock:
            self._pending[cid] = p
        msg = {"id": cid, "cmd": cmd, "args": args or []}
        if options:
            msg["options"] = options
        payload = json.dumps(msg) + "\n"
        try:
            with self._send_lock:
                self._sock.sendall(payload.encode())
        except OSError as e:
            with self._lock:
                self._pending.pop(cid, None)
            raise RobotConnectionError(f"send failed: {e}")
        return p

    # ------------------------------------------------------------------ #
    # Receiving / dispatch
    # ------------------------------------------------------------------ #

    def _receive_loop(self):
        buf = b""
        try:
            while self._connected:
                data = self._sock.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if line:
                        self._dispatch(line)
        except OSError:
            pass
        finally:
            self._connected = False
            self._fail_all("CONNECTION_LOST", "receiver stopped")

    def _dispatch(self, line):
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return

        cid = msg.get("id")
        if cid is None:
            # The executor's access-list refusal: it sends this ONE line and closes. It
            # carries no id, so before 2026-09-29 it fell through _fire_event and was
            # discarded -- connect() appeared to succeed, then nothing worked and the caller
            # saw "no status event arrived" from a perfectly healthy robot. A refusal that
            # the robot states plainly must not be silently dropped by the client.
            if msg.get("code") == "FORBIDDEN":
                self._rejected = True
            self._fire_event(msg)
            return

        status = msg.get("status")
        with self._lock:
            p = self._pending.get(cid)
            if p is None:
                return
            if status == "queued":
                p.queued = True
                self._outstanding.add(cid)
                p.ack.set()
                return
            # terminal replies below → drop the pending entry
            self._pending.pop(cid, None)
            if status == "ok":
                p.data = msg.get("data")
            elif status == "error":
                p.error = (msg.get("code", "ERROR"), msg.get("message", ""))
            # "complete" carries no data of its own EXCEPT for gripper commands,
            # which report WHY they settled: 1/2 = an object stopped the jaws
            # (opening/closing), 3 = reached the target with nothing there. Kept
            # per-command so a caller can ask about a specific pick rather than
            # only the most recent one.
            if "gripper_object" in msg:
                p.data = {"gripper_object": int(msg["gripper_object"])}
                self._last_gripper = (cid, int(msg["gripper_object"]))
        # clear from outstanding + wake wait_idle (outside _lock to avoid nesting)
        if status in ("complete", "error"):
            with self._idle_cv:
                self._outstanding.discard(cid)
                self._idle_cv.notify_all()
            # p.queued (set on the earlier "queued" ack) confirms this was a motion
            # command, not just any pending happening to see status=="complete".
            if status == "complete" and p.queued and self.on_motion_complete:
                self.on_motion_complete(cid)
            elif status == "error" and p.queued:
                failure = {"id": cid, "code": p.error[0], "message": p.error[1]}
                with self._lock:
                    self._motion_errors.append(failure)
                if self.on_motion_error:
                    self.on_motion_error(dict(failure))
        p.ack.set()
        p.done.set()

    def _fire_event(self, msg):
        mtype = msg.get("type")
        if mtype == "status":
            with self._status_cv:
                self._status.update(msg)
                self._status_cv.notify_all()
            if self.on_status_change:
                self.on_status_change(dict(self._status))
        elif mtype == "error":
            if self.on_error:
                self.on_error(msg)

    def _fail_all(self, code, message):
        """Wake every waiter with a connection error (on disconnect / socket loss)."""
        with self._lock:
            pendings = list(self._pending.items())
            self._pending.clear()
            # Queued motion that never reached a terminal reply is a failure, not a
            # success — record it so the _outstanding.clear() below can't make
            # wait_idle() look like everything completed.
            self._motion_errors.extend(
                {"id": cid, "code": code, "message": message}
                for cid, p in pendings if p.queued)
        for _, p in pendings:
            if p.error is None and p.data is None:
                p.error = (code, message)
            p.ack.set()
            p.done.set()
        with self._idle_cv:
            self._outstanding.clear()
            self._idle_cv.notify_all()
        with self._status_cv:
            self._status_cv.notify_all()   # wake status waiters so they don't hang

    # ------------------------------------------------------------------ #
    # Synchronization helpers
    # ------------------------------------------------------------------ #

    def wait_idle(self, timeout=None):
        """Block until no queued motion commands remain outstanding.

        Note a failed command also leaves _outstanding (it is terminal), so this
        returning True does NOT mean every command succeeded — check
        take_motion_errors().
        """
        with self._idle_cv:
            return self._idle_cv.wait_for(lambda: not self._outstanding, timeout=timeout)

    def last_gripper_result(self):
        """(command_id, object_detection) of the last settled gripper command, or None."""
        with self._lock:
            return self._last_gripper

    def take_motion_errors(self):
        """Return and clear the queued-command failures seen since the last call."""
        with self._lock:
            failures = self._motion_errors
            self._motion_errors = []
        return failures

    def wait_status(self, predicate, timeout=None):
        """Block until predicate(status) is true. Event-driven on status events.

        Returns the status dict, or None on timeout / disconnect.
        """
        with self._status_cv:
            self._status_cv.wait_for(
                lambda: predicate(self._status) or not self._connected, timeout=timeout)
            if not self._connected or not predicate(self._status):
                return None
            return dict(self._status)

    def latest_status(self):
        with self._status_cv:
            return dict(self._status)

    @staticmethod
    def wait_ack(p, timeout):
        if not p.ack.wait(timeout):
            raise RobotTimeout("no ack within timeout")
        if p.error is not None:
            raise CommandRejected(*p.error)

    @staticmethod
    def wait_done(p, timeout):
        if not p.done.wait(timeout):
            raise RobotTimeout("command did not complete within timeout")
        if p.error is not None:
            raise CommandRejected(*p.error)
        return p.data
