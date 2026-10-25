"""TCP connection with a background receiver thread.

Exactly one thread reads the socket. It matches each reply to the pending command by
``id`` and signals that command's events; replies without an ``id`` are unsolicited
events routed to callbacks. This lets multiple caller threads issue commands and block
on their own results without ever reading the socket directly.

A queued motion command receives TWO replies: ``{"status":"queued"}`` (ack) then later
``{"status":"complete"}``. Immediate commands receive one terminal reply (``ok``/``error``).
"""

import json
import queue
import socket
import sys
import threading
import time
import traceback

from .constants import CONNECT_TIMEOUT
from .exceptions import RobotConnectionError, RobotTimeout, CommandRejected
from .version import PROTOCOL_VERSION, __version__


class _Pending:
    """Tracks the lifecycle of one in-flight command id."""
    __slots__ = ("ack", "done", "data", "error", "error_info", "queued", "cid", "owner")

    def __init__(self, cid=None, owner=None):
        self.cid = cid                  # its command id, and the Connection that keeps its failure for wait_idle()
        self.owner = owner
        self.ack = threading.Event()    # first reply seen (queued / ok / error)
        self.done = threading.Event()   # terminal reply seen (ok / complete / error)
        self.data = None
        self.error = None               # (code, message) or None
        self.error_info = {}            # the catalog's number / title / fix of that error, when sent (2026-10-15)
        self.queued = False


def default_client_name():
    """"<script> on <host>" — how the robot names a client that gives no name (e.g. "pick_place.py on lab-pc")."""
    import os
    import sys
    script = os.path.basename(sys.argv[0]) if sys.argv and sys.argv[0] not in ("", "-c") else "python"
    return f"{script} on {socket.gethostname()}"


class Connection:
    def __init__(self, name=None):
        # Sent as "client" with every message: the robot's activity log names who asked (2026-10-15).
        self.name = name or default_client_name()
        self._sock = None
        self._recv_thread = None
        self._connected = False

        self._pending = {}                     # id -> _Pending
        self._lock = threading.Lock()          # guards _pending / _outstanding
        self._send_lock = threading.Lock()     # serialize sendall
        self._outstanding = set()              # queued motion ids not yet complete
        self._idle_cv = threading.Condition()  # notified when _outstanding shrinks

        self._rejected = False                 # executor refused us on the access list
        self.wire_names = {}                   # command ID -> the ID to send instead (an older robot, 2026-10-23)
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

        # The program's callbacks run on a thread of their own, in arrival order (2026-10-24): on the receiver
        # thread a command sent from a callback could not get its reply — only the receiver delivers replies.
        self._callbacks = queue.SimpleQueue()  # (name, fn, arg)
        self._callback_thread = None
        self._callback_lock = threading.Lock()
        self._closing = False                  # disconnect() runs: the receiver's end is DISCONNECTED, not a loss

        self.on_error = None                   # callback(dict) for unsolicited errors
        self.on_status_change = None           # callback(dict) for unsolicited status
        self.on_motion_complete = None         # callback(command_id) on queued-command success
        self.on_motion_error = None            # callback(dict) on queued-command failure

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def remote_ip(self):
        """Robot's IP, or None if not connected. RTIClient needs it to aim UDP."""
        if self._sock is None or not self._connected:      # also after a lost connection (2026-10-24)
            return None
        try:
            return self._sock.getpeername()[0]
        except OSError:
            return None

    def local_ip(self):
        """Our IP *on the route to the robot*, or None if not connected.

        Taken from the live socket rather than a hostname lookup, so it is correct on
        a multi-homed PC — the robot must be told the interface it can actually
        reach us on, not whichever address resolves first.
        """
        if self._sock is None or not self._connected:
            return None
        try:
            return self._sock.getsockname()[0]
        except OSError:
            return None

    def local_label(self):
        """Our address exactly as the executor labels this connection ("ip:port"), or None.

        The executor identifies a client by its TCP connection and publishes the holder as
        that connection's peer address, so "is control mine?" is a string comparison against
        this. Taken from the live socket, not a hostname lookup, for the same reason as
        local_ip(). (Direct connections only: behind a NAT the server sees the translated
        address and this will not match — the answer is then "not mine", the safe side.)
        """
        if self._sock is None or not self._connected:
            return None
        try:
            ip, port = self._sock.getsockname()[:2]
        except OSError:
            return None
        return f"{ip}:{port}"

    def connect(self, ip, port, timeout=CONNECT_TIMEOUT):
        if self._connected:                    # 2026-10-24: a second receiver read the new socket beside the old one
            raise RobotConnectionError("already connected: call disconnect() first")
        try:
            sock = socket.create_connection((ip, port), timeout=timeout)
        except OSError as e:
            raise RobotConnectionError(f"could not connect to {ip}:{port}: {e}")
        sock.settimeout(None)
        _keepalive(sock)
        # Nothing of an earlier connection carries over (2026-10-24): its access-list refusal, its status (the version
        # check read another robot's after an IP change), its failures and its last gripper result.
        with self._lock:
            self._pending.clear()
            self._motion_errors = []
            self._last_gripper = None
            self._outstanding.clear()
        with self._status_cv:
            self._status = {}
        self._rejected = False
        self._closing = False
        self._sock = sock
        self._connected = True
        self._recv_thread = threading.Thread(target=self._receive_loop, args=(sock,), daemon=True,
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
        self._closing = True                   # the receiver, ending now, reports DISCONNECTED too (2026-10-24)
        self._connected = False
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        self._fail_all("DISCONNECTED", "connection closed")

    @property
    def connected(self):
        return self._connected

    @property
    def rejected(self):
        """True after the robot refused this machine (its access list)."""
        return self._rejected

    # ------------------------------------------------------------------ #
    # Sending
    # ------------------------------------------------------------------ #

    def send_command(self, cid, cmd, args=None, options=None):
        """Register a pending entry and send {"id","cmd","args", optional "options"}."""
        if not self._connected:
            raise RobotConnectionError("not connected")
        p = _Pending(cid, self)
        with self._lock:
            self._pending[cid] = p
        cmd = self.wire_names.get(cmd, cmd)      # a robot older than a rename gets the ID it knows (client.connect)
        # + this client's version and command protocol (2026-10-21, versioning V3): the executor logs them per
        # connection and lists them in get_versions; an older executor ignores both keys.
        msg = {"id": cid, "cmd": cmd, "args": args or [], "client": self.name,
               "client_version": __version__, "protocol": PROTOCOL_VERSION}
        if options:
            msg["options"] = options
        payload = json.dumps(msg) + "\n"
        try:
            with self._send_lock:
                self._sock.sendall(payload.encode())
        except (OSError, AttributeError) as e:          # AttributeError: disconnect() ran meanwhile (no socket)
            with self._lock:
                self._pending.pop(cid, None)
            raise RobotConnectionError(f"send failed: {e}")
        return p

    # ------------------------------------------------------------------ #
    # Receiving / dispatch
    # ------------------------------------------------------------------ #

    def _receive_loop(self, sock):
        buf = b""
        try:
            while self._connected:
                data = sock.recv(4096)          # its own socket: disconnect() clears self._sock meanwhile
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
            if self._closing:                   # our own disconnect(): it reports DISCONNECTED (2026-10-24)
                self._fail_all("DISCONNECTED", "connection closed")
            else:
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
            if msg.get("code") and msg.get("type") is None:
                # An error with no command id — the access-list refusal, a line the robot could not read (BAD_JSON) —
                # goes to on_error (2026-10-24: they carried no "type", so on_error never fired).
                msg = {**msg, "type": "error"}
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
                p.error_info = {k: msg[k] for k in ("number", "title", "fix") if msg.get(k) is not None}
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
                self._call_back("on_motion_complete", self.on_motion_complete, cid)
            elif status == "error" and p.queued:
                failure = {"id": cid, "code": p.error[0], "message": p.error[1], **p.error_info}
                with self._lock:
                    self._motion_errors.append(failure)
                if self.on_motion_error:
                    self._call_back("on_motion_error", self.on_motion_error, dict(failure))
        p.ack.set()
        p.done.set()

    def _fire_event(self, msg):
        mtype = msg.get("type")
        if mtype == "status":
            with self._status_cv:
                self._status.update(msg)
                self._status_cv.notify_all()
            if self.on_status_change:
                self._call_back("on_status_change", self.on_status_change, dict(self._status))
        elif mtype == "error":
            if self.on_error:
                self._call_back("on_error", self.on_error, msg)

    def _call_back(self, name, fn, arg):
        """Hand a program's callback to the callback thread (2026-10-24): on the receiver thread a command sent from a
        callback (robot.stop() in on_motion_error) waited 5 s for a reply only the busy receiver could deliver."""
        with self._callback_lock:
            if self._callback_thread is None or not self._callback_thread.is_alive():
                self._callback_thread = threading.Thread(target=self._run_callbacks, daemon=True,
                                                         name="RobotClientCallbacks")
                self._callback_thread.start()
        self._callbacks.put((name, fn, arg))

    def _run_callbacks(self):
        """The callback thread: every callback in arrival order. An exception one raises is printed and dropped
        (2026-10-24): on the receiver thread it ended the receiver, so the client called itself disconnected and
        refused every command, stop() too, while the robot kept the connection and this client's control."""
        while True:
            name, fn, arg = self._callbacks.get()
            try:
                fn(arg)
            except Exception:
                print(f"robotclient: the {name} callback raised; the client goes on:", file=sys.stderr)
                traceback.print_exc()

    def _forget_failure(self, cid):
        """A failure already raised to the program (a sync call, a wait_*_complete) is not raised again by the next
        wait_idle() (2026-10-24: one failure was reported twice)."""
        with self._lock:
            self._motion_errors = [f for f in self._motion_errors if f.get("id") != cid]

    def _fail_all(self, code, message):
        """Wake every waiter with a connection error (on disconnect / socket loss)."""
        with self._lock:
            pendings = list(self._pending.items())
            self._pending.clear()
            # Queued motion that never reached a terminal reply is a failure, not a
            # success — record it so the _outstanding.clear() below can't make
            # wait_idle() look like everything completed.
            lost = [{"id": cid, "code": code, "message": message} for cid, p in pendings if p.queued]
            self._motion_errors.extend(lost)
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
        if self.on_motion_error:           # the lost connection's failures reach the callback too (2026-10-24)
            for failure in lost:
                self._call_back("on_motion_error", self.on_motion_error, dict(failure))

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
        # The predicate gets a copy (2026-10-24): it was handed the client's own dict, which it could change.
        with self._status_cv:
            self._status_cv.wait_for(
                lambda: predicate(dict(self._status)) or not self._connected, timeout=timeout)
            if not self._connected or not predicate(dict(self._status)):
                return None
            return dict(self._status)

    def latest_status(self):
        with self._status_cv:
            return dict(self._status)

    @staticmethod
    def wait_ack(p, timeout):
        if not p.ack.wait(timeout):
            raise RobotTimeout("no ack within timeout")
        # A rejection is an error INSTEAD of "queued". A command that was queued and then failed before this thread
        # looked is a motion failure — reported once, by wait_done / wait_idle (MotionFailed) — not ALSO here
        # (2026-10-13: the simulated suite saw one JOINT_LIMIT counted twice).
        if p.error is not None and not p.queued:
            raise CommandRejected(*p.error, **p.error_info)

    @staticmethod
    def wait_done(p, timeout):
        if not p.done.wait(timeout):
            raise RobotTimeout("command did not complete within timeout")
        if p.error is not None:
            if p.queued and p.owner is not None:
                p.owner._forget_failure(p.cid)      # raised here, so not again by wait_idle()
            raise CommandRejected(*p.error, **p.error_info)
        return p.data


# TCP keepalive (2026-10-24): a network that dies silently (a cable pulled, no reset) was noticed only when a send
# failed — minutes of retransmission, or never without a send — so wait_idle(None) could block that long. Probes after
# 5 s idle, every 2 s, 3 lost: a dead robot is found in about 11 s. The robot does the same on its side.
KEEPALIVE_IDLE_S, KEEPALIVE_INTERVAL_S, KEEPALIVE_PROBES = 5, 2, 3


def _keepalive(sock):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for opt, value in (("TCP_KEEPIDLE", KEEPALIVE_IDLE_S), ("TCP_KEEPINTVL", KEEPALIVE_INTERVAL_S),
                       ("TCP_KEEPCNT", KEEPALIVE_PROBES)):
        if hasattr(socket, opt):                    # Linux (and macOS for some); elsewhere the system's timing
            try:
                sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, opt), value)
            except OSError:
                pass
