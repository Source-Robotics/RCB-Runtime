"""RobotClient — high-level Python API over the EXEC JSON protocol.

Async by default (queued motion returns after the "queued" ack; use wait_idle() to
synchronize). In sync mode (RobotClient(sync=True)), state-changers block until the target
state is reached and motion blocks until complete. See docs/robotclient_transport.md.

    # async
    robot = RobotClient()
    robot.connect("127.0.0.1")
    robot.enable();           robot.wait_enabled()
    robot.home();             robot.wait_homed()
    robot.enter_exec_mode();  robot.wait_mode("EXEC")
    robot.move_joints(0, -60, 60, 0, 0, 0)
    robot.wait_idle()

    # sync — same calls, each blocks until its target
    robot = RobotClient(sync=True)
    robot.connect("127.0.0.1")
    robot.enable(); robot.home(); robot.enter_exec_mode()
    robot.move_joints(0, -60, 60, 0, 0, 0)

Joint angles are DEGREES at the API (converted to radians server-side).

TCP poses are [x, y, z, rx, ry, rz]: metres, and three angles in RADIANS by default, or degrees with
angle_unit="deg" on the call (2026-10-21; converted here, so the robot and its logs always see radians). The angles:
R = Rx(rx) Ry(ry) Rz(rz) — about base X by rx, then the tool's new Y by ry, then its own Z by rz (rz = the tool's
spin about its own axis).
"""
from __future__ import annotations
import math
from typing import Any, Callable, Sequence

from .connection import Connection
from .commands import generate_id
from .constants import (
    DEFAULT_PORT, IMMEDIATE_TIMEOUT, FORCE_TAKE_TIMEOUT, ACK_TIMEOUT, DEFAULT_MOVE_TIMEOUT,
    WAIT_STATE_TIMEOUT, WAIT_HOMED_TIMEOUT,
)
from .exceptions import (RobotError, RobotTimeout, MotionFailed, GripFailed, HomingFailed, VersionMismatch,
                         RobotConnectionError)
from .version import check_protocols, wire_names_for

# Pose angles (2026-10-21): radians unless a call says angle_unit="deg".
_RAD_PER = {"rad": 1.0, "deg": 3.141592653589793 / 180.0}


def _whole(value, what):
    """A joint number or a direction as a whole number: 2.7 is refused, not cut to 2 (2026-10-24: set_joint_wall(2.7, ...)
    changed joint 2). A value that is no number raises as int() did."""
    f = float(value)
    if not (math.isfinite(f) and f == int(f)):
        raise ValueError(f"{what} must be a whole number, got {value!r}")
    return int(f)


def _rad_per(angle_unit):
    """Radians per one `angle_unit` ("rad" | "deg"). Anything else is refused: a quiet fallback would turn the tool
    57 times too far, or 57 times too little."""
    try:
        return _RAD_PER[angle_unit]
    except (KeyError, TypeError):
        raise ValueError(f"angle_unit must be 'rad' or 'deg', got {angle_unit!r}") from None


def _pose_in(values, angle_unit):
    """Whole poses [x, y, z, rx, ry, rz] (one, or several end to end) as floats, the angles in radians."""
    k = _rad_per(angle_unit)
    return [float(v) * k if i % 6 >= 3 else float(v) for i, v in enumerate(values)]


def _pose_out(values, angle_unit):
    """A pose from the robot (radians) with its angles in `angle_unit`."""
    k = _rad_per(angle_unit)
    return [float(v) / k if i % 6 >= 3 else float(v) for i, v in enumerate(values)]


class GripperResult:
    """One gripper command's outcome: why its jaws stopped.

    `gripper_result()`, `wait_holding_part()` and `wait_released_part()`
    return one. Its code tells a part between the jaws from jaws that reached
    their target with nothing there; "complete" alone cannot.

    Attributes:
        code: Why the jaws stopped. `1`: something stopped them while opening.
            `2`: something stopped them while closing. `3`: they reached the
            target with nothing in the way, also when they came within 5 of it
            while the gripper still reported motion. A settled command never
            reports `0`, the gripper's "still moving".
        command_id: The id of the gripper command, as the call that queued it
            returned it.

    Examples:
        >>> robot.close_gripper()
        >>> robot.wait_gripper_move_complete()
        >>> result = robot.gripper_result()
        >>> result.code, result.name
        (3, 'reached target, no object')
    """

    __slots__ = ("code", "command_id")

    _NAMES = {
        0: "still moving",
        1: "object detected while opening",
        2: "object detected while closing",
        3: "reached target, no object",
    }

    def __init__(self, code, command_id=None):
        self.code = int(code)
        self.command_id = command_id

    @property
    def object_detected(self) -> bool:
        """Whether something stopped the jaws before their target.

        `True` for codes 1 and 2: the gripper reported that the jaws pressed on
        something while opening or closing. After a close that is the part being
        held; after an open it means something blocked the jaws.

        Examples:
            >>> robot.close_gripper()
            >>> robot.wait_gripper_move_complete()
            >>> robot.gripper_result().object_detected
            False
        """
        return self.code in (1, 2)

    @property
    def reached_target(self) -> bool:
        """Whether the jaws reached their target with nothing in the way.

        `True` for code 3 only. Code 0, the jaws within 5 of the target while the
        gripper still reported motion, reads `False`.

        Examples:
            >>> robot.open_gripper()
            >>> robot.wait_gripper_move_complete()
            >>> robot.gripper_result().reached_target
            True
        """
        return self.code == 3

    @property
    def name(self) -> str:
        """Short text for the code, such as "reached target, no object".

        Code 0 reads "still moving", 1 "object detected while opening", 2 "object
        detected while closing" and 3 "reached target, no object". Any other code
        reads "unknown(N)", with the code for N.

        Examples:
            >>> robot.close_gripper()
            >>> robot.wait_gripper_move_complete()
            >>> robot.gripper_result().name
            'reached target, no object'
        """
        return self._NAMES.get(self.code, f"unknown({self.code})")

    def __repr__(self):
        return f"GripperResult(code={self.code}, {self.name!r})"


class RobotClient:
    def __init__(self, sync=False, name=None):
        """name: how the robot shows this client in its activity log and control messages ("dashboard",
        "pick_place"). Default: "<script> on <host>", e.g. "pick_place.py on lab-pc"."""
        self._conn = Connection(name=name)
        self._sync = sync
        # What the robot reported at connect() (versioning V3): its runtime version ("0.9.0") and protocol versions
        # ({"commands", "rti", "telemetry"}); None until connected, or when the robot reports none.
        self.runtime_version: str | None = None
        self.protocols: dict | None = None
        self._last_queued = None   # most recent queued command's _Pending, for wait_motion_complete()
        # Most recent GRIPPER command's _Pending, tracked separately: queue a
        # gripper command then a joint move and _last_queued is the MOVE, so
        # wait_motion_complete() would wait for the wrong thing.
        self._last_gripper_queued = None

    # ------------------------------------------------------------------ #
    # Connection
    # ------------------------------------------------------------------ #

    def connect(self, ip: str, port: int = DEFAULT_PORT, *, allow_mismatch: bool = False) -> RobotClient:
        """Connect to the robot and check its command protocol version.

        The client opens a connection to the robot software, waiting up to 5 s,
        then waits for the status the robot sends every new client: up to 2 s,
        after which it asks the robot for it, up to 5 s more. It compares the
        robot's command protocol version with its own. A different version raises
        `VersionMismatch` and the client disconnects again; a robot that reports
        none gives a `VersionWarning` (a Python warning) and the connection goes
        ahead. Afterwards the attributes `runtime_version` (the robot software's
        version) and `protocols` (its protocol versions: `commands`, `rti` and
        `telemetry`) hold what the robot reported, or `None`.

        Every message the client sends carries its name, which the robot's
        activity log shows: the name given to `RobotClient()`, or "<script> on
        <host>". With older robot software that knows some commands by an earlier
        name, the client sends them under that name, so a program runs on either.
        A command the robot software does not have is refused with `UNKNOWN_CMD`.
        Connecting takes no control: the first command that needs it does.

        Args:
            ip: The robot's IP address or host name.
            port: The TCP port of the robot software; the robot listens on the
                default.
            allow_mismatch: `True` skips the version check: no `VersionMismatch`
                and no `VersionWarning`. The robot can then misread commands: use
                it only when you know the two versions work together.

        Returns:
            The client itself, so that `robot = RobotClient().connect(ip)` works.

        Raises:
            RobotConnectionError: The connection could not be opened within 5 s:
                nothing listens at `ip` and `port`, the host name is unknown, or
                there is no route. Also when the robot refused this computer
                because its address is not on the robot's access list, when the
                connection closed or nothing answered as a robot, and when this
                client is already connected.
            VersionMismatch: The robot speaks another command protocol version,
                and `allow_mismatch` is `False`. The client is disconnected again.

        Note:
            `VersionMismatch` is a kind of `RobotConnectionError`, so
            `except RobotConnectionError` catches both. To connect again, call
            `disconnect()` first: nothing of the earlier connection carries over.

        See also:
            `disconnect()`, `get_versions()`, `take_control()`

        Examples:
            >>> from robotclient import RobotClient
            >>> robot = RobotClient(name="pick_place")
            >>> robot.connect("192.168.0.84")
            >>> robot.protocols
            {'commands': 2, 'rti': 1, 'telemetry': 1}
        """
        self._conn.connect(ip, port)
        self._last_queued = self._last_gripper_queued = None     # an earlier connection's commands (2026-10-24)
        # The executor pushes a status snapshot on connect; wait briefly for it so
        # status() is populated and the first wait_*() has a baseline.
        self._conn.wait_status(lambda s: bool(s), timeout=1.0)
        st = self._conn.latest_status()
        if not st:          # no snapshot within 1 s (a slow link): ask (get_status carries the same fields)
            try:
                st = self._immediate("get_status") or {}
            except RobotError:
                st = {}
        if not self._conn.connected or not st:
            # 2026-10-24: connect() returned "successfully" here — after a late access-list refusal, a connection
            # that closed, or a TCP server on that port that is not the robot (nothing ever answered).
            rejected = self._conn.rejected
            self._conn.disconnect()
            raise RobotConnectionError(
                f"{ip} refused this machine: not in the robot's access list (RCB_ALLOWED_IPS on the robot)" if rejected
                else f"{ip}:{port} closed the connection or never answered as a robot")
        self.runtime_version = st.get("runtime_version")
        # A robot older than the command renames (version.COMMANDS_RENAMED_IN) gets the IDs it knows, for every
        # command this client sends from here on; a current robot gets the documented names (2026-10-23).
        self._conn.wire_names = wire_names_for(self.runtime_version)
        self.protocols = dict(st["protocols"]) if isinstance(st.get("protocols"), dict) else None
        try:
            check_protocols(st, ("commands",), allow_mismatch)
        except VersionMismatch:
            self._conn.disconnect()
            raise
        return self

    def disconnect(self) -> None:
        """Close the connection to the robot.

        The robot releases control at once if this client holds it, so another
        client can take it. On the client, every call still waiting for the robot
        ends: a command's call raises `CommandRejected` with the code
        `DISCONNECTED`, and a status wait such as `wait_mode()` raises
        `RobotConnectionError`. Queued commands that had not finished count as
        failed: `on_motion_error()` gets them, and the next `wait_idle()` raises
        `MotionFailed` for them.

        Calling it again, or before `connect()`, does nothing. Leaving a
        `with RobotClient() as robot:` block calls it.

        Warning:
            When this client holds control and moves it queued still run, the
            robot stops the arm as `stop()` does; a lost connection does the same.
            Call `wait_idle()` first to let them finish. A jog stops too. The
            robot stays in its mode, and a real-time session opened with
            `rti_connect()` stays open.

        See also:
            `connect()`, `stop()`, `release_control()`

        Examples:
            >>> robot.wait_idle()
            >>> robot.disconnect()
            >>> from robotclient import RobotClient
            >>> with RobotClient() as robot:
            ...     robot.connect("192.168.0.84")
            ...     robot.wait_homed()
        """
        self._conn.disconnect()

    @property
    def connected(self) -> bool:
        """Whether the client is connected to the robot.

        It is `True` from `connect()` until `disconnect()`, or until the
        connection is lost: the robot software closed it, or the network failed.
        It reads the client's own state and asks the robot nothing. A network
        that fails silently is noticed within about 11 s (the connection's
        keepalive probes); until then commands raise `RobotTimeout`.

        Returns:
            `True` while the connection is open; `False` before `connect()`, after
            `disconnect()` and after the connection was lost.

        See also:
            `connect()`, `disconnect()`

        Examples:
            >>> robot.connected
            True
        """
        return self._conn.connected

    def on_error(self, fn: Callable[[dict], None]) -> None:
        """Set a function to call when the robot sends an error event.

        The function receives the event, a dict with `code` and `message`, for
        an error message from the robot that answers no command: its refusal of
        this computer (`FORBIDDEN`) and a line it could not read (`BAD_JSON`).
        Other errors reach a program through the answers to its commands,
        `wait_idle()`, the status (`errors()`, `on_status_change()`) and
        `on_motion_error()`. A client has one such function: a new call replaces
        it, and `None` removes it.

        Args:
            fn: The function, called with the event dict. `None` removes it.

        Note:
            The function runs on the client's callback thread, one call at a
            time, in the order the events arrived, so a command or a wait called
            from it works. Keep it short: the next events wait for it. If the
            function raises, the client prints the exception and goes on.

        See also:
            `on_status_change()`, `on_motion_error()`, `errors()`

        Examples:
            >>> robot.on_error(lambda event: print("robot error:", event))
        """
        self._conn.on_error = fn

    def on_status_change(self, fn: Callable[[dict], None]) -> None:
        """Set a function to call each time the robot's status arrives.

        The function receives a copy of the latest status, the dict
        `latest_status()` returns. The robot sends its status once when the
        client connects, then whenever it changes, checking 50 times a second,
        and at once after `take_control()`, `release_control()` and
        `set_force_confirm()`. Set the function before `connect()` to receive the
        first status too. A client has one such function: a new call replaces it,
        and `None` removes it.

        Args:
            fn: The function, called with the status dict. `None` removes it.

        Note:
            The function runs on the client's callback thread, one call at a
            time, in the order the events arrived, so a command or a wait called
            from it works. Keep it short: the next events wait for it. If the
            function raises, the client prints the exception and goes on.

        See also:
            `latest_status()`, `wait()`, `on_error()`

        Examples:
            >>> robot.on_status_change(lambda s: print(s.get("mode"), s.get("state")))
            >>> robot.enter_exec_mode(wait=True)
        """
        self._conn.on_status_change = fn

    def on_motion_complete(self, fn: Callable[[str], None]) -> None:
        """Set a function to call when a queued command of this client finishes.

        The function receives the command's id, the value the queued call
        returned, each time a command this client queued finishes: a move, a
        delay, or a gripper command once the jaws have settled. Other clients'
        commands do not call it, nor do failed ones: see `on_motion_error()`.

        It runs on the callback thread, so `wait_motion_complete()` and
        `wait_idle()` can return before it has run. A client has one such
        function: a new call replaces it, and `None` removes it.

        Args:
            fn: The function, called with the command id. `None` removes it.

        Note:
            The function runs on the client's callback thread, one call at a
            time, in the order the events arrived, so a command or a wait called
            from it works. Keep it short: the next events wait for it. If the
            function raises, the client prints the exception and goes on.

        See also:
            `on_motion_error()`, `wait_idle()`, `wait_motion_complete()`

        Examples:
            >>> done = []
            >>> robot.on_motion_complete(done.append)
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_motion_complete()
        """
        self._conn.on_motion_complete = fn

    def on_motion_error(self, fn: Callable[[dict], None]) -> None:
        """Set a function to call when a queued command of this client fails.

        The function is called as soon as a command this client queued fails
        after the robot accepted it, `STOPPED` and `ABORTED` included. It
        receives a dict with `id` (the value the queued call returned), `code`
        and `message`, and the error catalog's `number`, `title` and `fix` when
        the robot sent them. The next `wait_idle()` reports the same failure as
        `MotionFailed`, unless this client's `stop()` discards it first or a
        call already raised it; this function lets a program react at once.

        A command the robot refused when it was sent raises in its own call
        instead. The failures a lost connection causes reach it too, with the
        code `CONNECTION_LOST`, or `DISCONNECTED` after `disconnect()`. A client
        has one such function: a new call replaces it, and `None` removes it.

        Args:
            fn: The function, called with the failure dict. `None` removes it.

        Note:
            The function runs on the client's callback thread, one call at a
            time, in the order the events arrived, so a command or a wait called
            from it works. Keep it short: the next events wait for it. If the
            function raises, the client prints the exception and goes on.

        See also:
            `wait_idle()`, `on_motion_complete()`, `stop()`

        Examples:
            >>> failures = []
            >>> robot.on_motion_error(failures.append)
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle(raise_on_error=False)
        """
        self._conn.on_motion_error = fn

    def latest_status(self) -> dict:
        """Return the latest status the robot sent, without asking it.

        The robot checks its state 50 times a second and sends the whole status to
        every connected client whenever a field changes, and once when a client
        connects. This returns a copy of the newest one, so it never waits and
        sends nothing. It is empty until the first status arrives, and after a
        disconnect it keeps the last values it had. Use `get_status()` to ask the
        robot, and `wait()` to wait until a field changes.

        Returns:
            The status, a dict. The fields used most: `mode` (the robot mode, such
            as `"IDLE"` or `"EXEC"`), `state` (`"ENABLED"` or `"DISABLED"`),
            `homed`, `err` (1 while an error is latched), `errors` (the latched
            error keys) and `error_details` (as `errors()` returns them),
            `control_holder`, `exec_paused` and `exec_stopping` (1 while a program
            is paused or brakes for a stop), `queue_depth` (queued moves not started
            yet), `active_command`, `motion_aborted` (why new moves are refused, or
            `None`), `recipe`, `payload` (`mass` in kilograms, `com` in metres),
            `joint_test`, `free_joint` and `hall_check`. Every field, with its
            meaning and unit, is in the Status fields reference.

        See also:
            `get_status()`, `errors()`, `wait()`, `on_status_change()`

        Examples:
            >>> robot.latest_status()["mode"]
            'IDLE'
            >>> robot.latest_status()["errors"]
            []
        """
        return self._conn.latest_status()

    status = latest_status      # its other name (2026-10-23): it is not get_status(), which asks the robot

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _should_wait(self, wait):
        """Per-call override wins; otherwise follow the client's sync flag."""
        return self._sync if wait is None else wait

    def _immediate(self, cmd, args=None, timeout=IMMEDIATE_TIMEOUT):
        cid = generate_id()
        p = self._conn.send_command(cid, cmd, args)
        return Connection.wait_done(p, timeout)

    def _state_change(self, cmd, wait, predicate, timeout=WAIT_STATE_TIMEOUT, what=None,
                      args=None):
        """Send an immediate command; if waiting, block until predicate(status) is true.

        Shared shape behind enable/disable/clear_errors/enter_exec_mode/stop/pause/
        resume. home() stays separate — its two-stage HOMING→done wait doesn't fit.
        `args` is for the few state changes that carry one (enter_flashing_mode's
        force flag); most pass none.
        """
        self._immediate(cmd, args)
        if self._should_wait(wait):
            self.wait(predicate, timeout, what or cmd)

    def _queued(self, cmd, args=None, timeout=DEFAULT_MOVE_TIMEOUT, options=None, gripper=False):
        cid = generate_id()
        p = self._conn.send_command(cid, cmd, args, options=options)
        Connection.wait_ack(p, ACK_TIMEOUT)          # raises if rejected at validation
        self._last_queued = p                        # for wait_motion_complete()
        if gripper:
            # Before the sync wait (2026-10-24): a failing gripper command raised before it was recorded, so the next
            # wait_gripper_move_complete() judged the gripper command before it.
            self._last_gripper_queued = p
        if self._sync:
            Connection.wait_done(p, timeout)
        return cid

    # ------------------------------------------------------------------ #
    # State-changers (immediate ack; block until target state if sync/wait)
    # ------------------------------------------------------------------ #

    def enable(self, wait: bool | None = None) -> None:
        """Enable the robot, so that it can home and move.

        The robot accepts it in any mode. While an error is latched, a pressed
        E-stop included, the robot refuses to enable and stays disabled: the
        call raises `CommandRejected` (`ENABLE_REJECTED`). Call `clear_errors()`
        first.

        In IDLE mode, on a homed arm with gravity compensation on (as at every
        start), enabling starts freedrive: the arm holds itself up against gravity
        and can be moved by hand.

        Args:
            wait: Wait until the robot reports itself enabled, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report itself enabled within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            ENABLE_REJECTED: An error is latched (a pressed E-stop included) or a safety
                violation is active: the robot stays disabled. `clear_errors()` first.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `disable()`, `clear_errors()`, `home()`

        Examples:
            >>> robot.clear_errors(wait=True)
            >>> robot.enable(wait=True)
        """
        self._state_change("enable", wait, lambda s: s.get("state") == "ENABLED",
                           what="wait_enabled")

    def disable(self, wait: bool | None = None) -> None:
        """Disable the robot: it stops at once and holds every joint still.

        The robot accepts it in every mode, from every client, and never refuses
        it. It switches to IDLE mode at once and holds each joint at zero speed:
        the motors stay on and resist being moved. To take all torque off the arm,
        use `safety_stop()`. The gripper's jaws stop where they are.

        A program running in EXEC mode is cancelled without braking along its
        path: the drives stop the joints as fast as their limits allow. Its queued
        commands fail with `ABORTED`, and the robot refuses new moves with
        `MOTION_ABORTED` until `clear_errors()`. To stop along the path first, use
        `stop()` or `enter_idle_mode()`. Homing in progress is abandoned, and the
        arm is then not homed.

        Args:
            wait: Wait until the robot reports itself disabled, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: RTI refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report itself disabled within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            It also ends motors off and firmware flashing, whichever client sends
            it: the motors hold again, and during a flash the robot's CAN traffic
            resumes, which can make the flash fail.

        See also:
            `enable()`, `stop()`, `safety_stop()`

        Examples:
            >>> robot.disable(wait=True)
        """
        self._state_change("disable", wait, lambda s: s.get("state") == "DISABLED")

    def clear_errors(self, wait: bool | None = None) -> None:
        """Clear the latched errors, and the abort after a failed move.

        The robot sends a clear to every drive that reports a fault, waits about
        0.15 s, then empties its list of latched errors, warnings included. A fault
        that is still present latches again; when none is left, the robot leaves
        the error state for IDLE mode. It stays disabled: call `enable()` next.
        Homing that an error invalidated stays invalid: call `home()` again.

        It also lifts the motion abort at once: after a move failed or a program
        was interrupted, the robot refuses new moves with `MOTION_ABORTED` until
        this call.

        Args:
            wait: Wait until the robot reports no error and has left the error
                state, up to 10 s. Warnings do not count; `wait_error_reset()` waits
                for them too.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: RTI refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, an
                error was still latched after 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            A fault that is still present can be missing from the status for up to
            about 20 ms before it latches again, so a wait can, rarely, return
            early. Check `errors()` when it matters.

        See also:
            `enable()`, `errors()`, `wait_error_reset()`

        Examples:
            >>> robot.clear_errors(wait=True)
            >>> robot.enable(wait=True)
        """
        self._state_change("clear_errors", wait,
            lambda s: s.get("err", 0) == 0 and s.get("mode") != "ACTIVE_ERROR")

    def enter_exec_mode(self, wait: bool | None = None) -> None:
        """Switch to EXEC mode, where the robot runs queued moves.

        In EXEC mode the robot plays the moves a program queues, in the order they
        were sent. On entry it holds the arm where it is. Calling it in EXEC mode
        changes nothing.

        Args:
            wait: Wait until the robot reports EXEC mode, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot refused it: it is not in IDLE or EXEC mode, not
                enabled or not homed, or another client holds control. Or RTI refused it
                or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report EXEC mode within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            MODE_REJECTED: RTI refused EXEC: an error is latched, or the arm is not
                enabled or not homed. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `enter_idle_mode()`, `stop()`, `move_joints()`

        Examples:
            >>> robot.enter_exec_mode(wait=True)
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
        """
        self._state_change("enter_exec_mode", wait, lambda s: s.get("mode") == "EXEC",
                           what="wait_mode(EXEC)")

    # ------------------------------------------------------------------ #
    # RTI mode — the TCP half of the real-time interface.
    # The UDP streaming half lives in RTIClient (rti_client.py); these four just
    # open, arm, disarm and close the session. See TODO/RTI_mode_plan.md §3.
    # ------------------------------------------------------------------ #

    def remote_ip(self) -> str | None:
        """Return the robot's IP address on this connection.

        It reads the address from the client's own connection and asks the robot
        nothing. The real-time client, `RTIClient`, sends its packets to it.

        Returns:
            The robot's IP address, for example `"192.168.0.84"`; `None` before
            `connect()` and after `disconnect()`.

        See also:
            `local_ip()`, `rti_connect()`

        Examples:
            >>> robot.remote_ip()
            '192.168.0.84'
        """
        return self._conn.remote_ip()

    def local_ip(self) -> str | None:
        """Return this computer's IP address on the route to the robot.

        It is the address of the network interface this connection uses, read
        from the connection itself, so it is right on a computer with several
        networks. It asks the robot nothing. `rti_connect()` has the robot send
        its state packets there when it is given no address. Behind a router that
        translates addresses (NAT), the robot sees another address.

        Returns:
            This computer's IP address, for example `"192.168.0.20"`; `None`
            before `connect()` and after `disconnect()`.

        See also:
            `remote_ip()`, `local_label()`, `rti_connect()`

        Examples:
            >>> robot.local_ip()
            '192.168.0.20'
        """
        return self._conn.local_ip()

    def rti_connect(self, pc_ip: str | None = None, wait: bool | None = None, rate_hz: float | None = None, transport: str | None = None) -> None:
        """Pair this PC with the robot and switch to a real-time mode.

        The robot then sends its state to `pc_ip` every 4 ms tick and accepts
        real-time commands only from that address. It switches from IDLE mode to a
        real-time mode and holds the arm still where it is. Nothing moves, and no
        command is applied, until `rti_claim()`. `RTIClient.connect()` calls this
        for you and opens the link.

        `rate_hz` picks the contract. Without it the robot enters RTI mode, hard
        real time: your PC sends one target per tick over UDP, each with its send
        time, and the robot replays your position targets one tick behind; it
        stops when no command comes for 40 ms. With it the robot enters SOFT_RTI
        mode: you send joint position targets at your own rate, the robot fills in
        the ticks between them, and it stops when three of your intervals pass
        without one.

        Each call sets the session's contract again: the command timeout, the
        servo generator, the minimum duration and the dynamics feedforward, and in
        RTI mode the pace limit and the state rate too. It also switches the rate
        limiter, the start-pose check and real-time gravity compensation on, and
        sets the tool speed cap back to its start value: a session never inherits
        them from an earlier one. The other real-time settings keep their values.
        Without `pc_ip`, a client that is not connected raises `RuntimeError` and
        sends nothing.

        Args:
            pc_ip: The IPv4 address the robot sends its state to and accepts
                commands from. `None` uses this connection's own address on the
                route to the robot.
            wait: Wait until the robot reports the real-time mode, up to 10 s: RTI
                without `rate_hz`, SOFT_RTI with it.
                `None` follows the client's `sync` setting.
            rate_hz: Your command rate in Hz, for SOFT_RTI; `None` for RTI mode. It
                sets the command timeout to three of your intervals, at most 1 s.
                Prefer a rate that divides 250 Hz evenly: 125, 50, 25 or 10 Hz.
                Zero, a negative or a non-finite rate is refused.
            transport: `"udp"` or `"tcp"`; `None` means `"udp"`. With `"tcp"` the
                robot waits for your PC to dial its port 30003 from `pc_ip`, and one
                stream carries both directions (`RTIClient` dials it). SOFT_RTI only:
                without `rate_hz` the robot uses UDP.

        Raises:
            CommandRejected: The robot refused it: it is not in IDLE mode or in the
                real-time mode asked for, a PC drives the arm or its session did not
                stop within 2 s, a value is invalid, it is not enabled or not homed, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report the mode within 10 s.
            RobotConnectionError: The client is not connected.
            ValueError: `transport` is neither `"udp"` nor `"tcp"`, or `rate_hz` is
                not a number. Nothing is sent.
            TypeError: `rate_hz` is not a number. Nothing is sent.

        Errors:
            REJECTED: A PC drives the arm (a claim is active): `rti_release()`
                first; a session still stopping is waited for, up to 2 s. Or the
                robot is in the other real-time mode: `rti_disconnect()` first.
            BAD_ARGS: `pc_ip` is not an IPv4 address, `rate_hz` is not above 0,
                `transport` is not `"udp"` or `"tcp"`, or the message did not carry
                exactly three values.
            MODE_REJECTED: RTI refused RTI / SOFT_RTI mode: an error is latched, or the
                arm is not enabled or not homed. The message says why.
            RTI_REFUSED: The message reached RTI without your PC's address. The message
                says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `rti_claim()`, `rti_disconnect()`, `rti_set_command_timeout()`,
            `rti_set_servo_generator()`

        Examples:
            >>> robot.rti_connect(rate_hz=50, wait=True)
            >>> robot.latest_status()["mode"]
            'SOFT_RTI'
            >>> robot.rti_disconnect(wait=True)
        """
        if pc_ip is None:
            pc_ip = self.local_ip()
            if pc_ip is None:
                raise RuntimeError("not connected — call connect() first")
        if transport is None:
            transport = "udp"
        if transport not in ("udp", "tcp"):
            raise ValueError("transport must be 'udp' or 'tcp'")
        # ALL THREE fields always sent, "" meaning "no rate given" — the executor validates
        # on EXACT arity, so a variable-length call is rejected as BAD_ARGS before it
        # reaches RTI. That is not hypothetical: shipping this as a variable-length call is
        # exactly how SOFT_RTI was made unreachable the first time. Same convention as
        # rti_set_servo_generator.
        target = "RTI" if rate_hz is None else "SOFT_RTI"
        self._immediate("rti_connect",
                        [pc_ip, "" if rate_hz is None else float(rate_hz), transport])
        if self._should_wait(wait):
            self.wait(lambda s: s.get("mode") == target, what=f"wait_mode({target})")

    def rti_claim(self) -> None:
        """Claim real-time control: the robot starts applying your commands.

        From now on the robot applies the commands your PC streams, and its
        watchdog runs at once: the first command must arrive within the command
        timeout (40 ms in RTI mode, three of your intervals in SOFT_RTI), and so
        must every next one. Otherwise the arm brakes to rest within its limits,
        then the robot latches `RTI_LINK_LOST`, which disables it. The first
        command is checked against the arm (`rti_set_start_pose_check()`): one too
        far from it ends the stream the same way, with `RTI_START_POSE`. A stream
        the arm cannot follow ends the same way, with `RTI_POSITION_LIMIT`,
        `RTI_VELOCITY_LIMIT`, `RTI_ACCELERATION_LIMIT`, `RTI_JERK_LIMIT` or
        `RTI_NO_SEND_TIME` (see `RTIClient`).

        The robot answers at once and never reports whether the claim took. It
        ignores a claim while an error is latched, while the session is not paired
        and idle (already claimed, or still stopping after a release), or when the
        rate limiter is on and its planner is missing. The state packet's
        `rti_state` shows it: 2 means control is active. While a claim is active,
        control of the robot (`take_control()`) does not expire.

        Raises:
            CommandRejected: The robot refused it: it is not in a real-time mode (for
                example right after `rti_connect()` without `wait`), not enabled or not
                homed, or another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_REFUSED: No claim: an error is latched, the robot is not in an RTI mode,
                the PC is not paired or a claim is active already, or the rate limiter
                cannot start. The message says which.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            From the claim on, the arm follows your PC's commands. Keep the space
            around the arm clear and a hand near the E-stop.

        See also:
            `rti_connect()`, `rti_release()`, `rti_set_start_pose_check()`,
            `rti_set_command_timeout()`

        Examples:
            >>> import time
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.connect(rate_hz=50)
            >>> robot.wait_mode("SOFT_RTI")
            >>> q = rti.read_once()["j_p"]
            >>> robot.rti_claim()
            >>> for i in range(50):
            ...     rti.send_joint_position(q)
            ...     time.sleep(0.02)
            ...
            >>> robot.rti_release()
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        self._immediate("rti_claim")

    def rti_release(self) -> None:
        """End the real-time stream: brake to rest and hold, still paired.

        The arm brakes to rest from where it was commanded to be, within the
        joints' real-time acceleration and jerk limits, and holds there; no error
        is latched. After 0.5 s of holding, the session is paired and idle again:
        the robot stays in its real-time mode, keeps sending state and holds the
        arm still, and a new `rti_claim()` starts another stream. A claim sent
        while it brakes or during the 0.5 s is ignored.

        It works in both real-time modes, also when the stream itself is down.
        Send it within the command timeout after your last command, or the robot
        latches `RTI_LINK_LOST` first. Without an active claim the robot ignores
        it.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_REFUSED: No session to release: no claim is active. The message says
                why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `rti_claim()`, `rti_disconnect()`

        Examples:
            >>> import time
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.connect(rate_hz=50)
            >>> robot.wait_mode("SOFT_RTI")
            >>> q = rti.read_once()["j_p"]
            >>> rti.claim()
            >>> for i in range(50):
            ...     q[5] += 0.0005
            ...     rti.send_joint_position(q)
            ...     time.sleep(0.02)
            ...
            >>> robot.rti_release()
            >>> time.sleep(1)
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        self._immediate("rti_release")

    def rti_disconnect(self, wait: bool | None = None) -> None:
        """Unpair the PC and switch the robot to IDLE mode.

        The robot stops sending state, forgets the PC's address, closes a TCP
        stream, and switches to IDLE mode at once. It takes the call in IDLE mode,
        in a real-time mode and in the error state, and refuses it in every other
        mode. An enabled, homed arm with gravity compensation on is then in
        freedrive; otherwise every joint holds still. The real-time settings that
        last across sessions keep their values.

        End a stream first with `rti_release()` and let the arm come to rest.

        Args:
            wait: Wait until the robot reports IDLE mode, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot refused it: it is in another mode (EXEC, JOG,
                homing, motors off, flashing), or another client holds control. Or RTI
                refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report IDLE mode within 10 s, for example with an error latched.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            It never brakes. Sent while a stream moves the arm, the arm coasts in
            freedrive until friction stops it, or, with gravity compensation off,
            stops as hard as the drives can.

        See also:
            `rti_connect()`, `rti_release()`, `enter_idle_mode()`

        Examples:
            >>> robot.rti_connect(wait=True)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_disconnect")
        if self._should_wait(wait):
            self.wait(lambda s: s.get("mode") == "IDLE", what="wait_mode(IDLE)")

    def rti_set_rate_limiter(self, enabled: bool) -> None:
        """Switch the real-time rate limiter on or off.

        The rate limiter keeps every real-time command within the joints'
        real-time limits from the robot XML (`<rti_limits>`). Position targets go
        through the servo generator and, in RTI mode, the stream checks; velocity
        commands are limited in speed, acceleration and jerk; torque commands in
        their rate of change. It takes effect at once, also mid-stream.

        With it off, commands reach the motors unshaped. What still applies: the
        soft joint limits (position targets are clamped to them, a velocity brakes
        before them, torque pushing past them is cut), the tool speed cap on
        velocity commands, the SOFT_RTI position check, gravity compensation, the
        first-command check and the watchdog. No generator runs, the RTI-mode
        stream checks and the dynamics feedforward are skipped, and a claim no
        longer needs the generator's planner. It is meant for measuring latency,
        not for normal work.

        Args:
            enabled: `True` to limit every command, `False` to pass commands
                through.

        Defaults:
            On, set each time the robot software starts and at every
            `rti_connect()`: a session never inherits it switched off.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: The value is not true or false (1 / 0), or the message did
                not carry exactly one value; this client always sends one.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            Off, nothing limits how fast a command moves the arm: a position step
            jumps at the drives' full authority, and a velocity command has no
            speed limit. It stays off until it is switched on or the next
            `rti_connect()`.

        See also:
            `rti_set_start_pose_check()`, `rti_set_servo_generator()`,
            `rti_set_tcp_vel_limit()`

        Examples:
            >>> robot.rti_connect(wait=True)
            >>> robot.rti_set_rate_limiter(True)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_rate_limiter", [1 if enabled else 0])

    def rti_set_lowpass(self, cutoff_hz: float) -> None:
        """Filter incoming real-time commands with a low-pass filter.

        The robot passes the main value of every command, a position target, a
        velocity or a torque, through a second-order Butterworth low-pass filter
        at `cutoff_hz`, before the rate limiter and the stream checks; feedforward
        values are not filtered. It removes noise from a sensor-driven stream, at
        the cost of lag: about 1 ms at 100 Hz, 3 ms at 60 Hz, 9 ms at 25 Hz. In RTI
        mode noise reads as acceleration and can end the stream with
        `RTI_ACCELERATION_LIMIT`; 60 Hz is a good start there. Each claim starts
        the filter afresh.

        Do not use it in SOFT_RTI: the filter keeps changing the target between
        your packets, the generator then follows the filter instead of your
        motion, and the motion gets rougher (measured). Use
        `rti_set_min_duration()` there.

        Args:
            cutoff_hz: The cutoff in Hz. 0 switches the filter off, and so does
                125 Hz or above (half the loop rate). A negative or non-finite value
                is refused.

        Defaults:
            From `config/system.xml` `<rti_defaults><lowpass_cutoff_hz>`, read at
            start: 0 (off). It lasts until changed or the robot software restarts;
            `rti_connect()` does not reset it.

        Raises:
            CommandRejected: The robot refused it: `cutoff_hz` is negative or not
                finite, or another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `cutoff_hz` is not a number. Nothing is sent.
            TypeError: `cutoff_hz` is not a number. Nothing is sent.

        Errors:
            BAD_ARGS: `cutoff_hz` is negative or not a finite number, or the
                message did not carry exactly one value.
            RTI_REFUSED: RTI could not use the value. The message says why; the setting
                in force stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            It lasts across sessions, from any PC: switch it off (0) when your
            program is done.

        See also:
            `rti_set_min_duration()`, `rti_set_rate_limiter()`

        Examples:
            >>> robot.rti_connect(wait=True)
            >>> robot.rti_set_lowpass(60)
            >>> robot.rti_set_lowpass(0)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_lowpass", [float(cutoff_hz)])

    def rti_set_start_pose_check(self, enabled: bool) -> None:
        """Switch the check of the first command after a claim on or off.

        With the check on, the robot compares the first command of each claim with
        the arm. A position target must lie within 0.1 rad (about 5.7 deg) of every
        joint's measured position; a velocity within 10 % of the joint's real-time
        velocity limit (after `rti_set_vel_scale()`); a torque within what the
        joint's real-time torque-rate limit allows in four ticks (16 ms). A first
        command outside that is refused: the arm brakes to rest and the robot
        latches `RTI_START_POSE`. It catches a controller that starts from zeros or
        from an old pose.

        The setting is read when the first command of a claim arrives, so it
        applies to the next claim, or to the current one before its first command.

        Args:
            enabled: `True` to check the first command, `False` to let any first
                command through.

        Defaults:
            On, set each time the robot software starts and at every
            `rti_connect()`. The tolerance, 0.1 rad, is `config/system.xml`
            `<rti_defaults><start_pose_tol_rad>`, read at start.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: The value is not true or false (1 / 0), or the message did
                not carry exactly one value; this client always sends one.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            With the check off, a first command far from the arm, such as all
            zeros, reaches the motors: the arm moves there at its real-time limits.
            The check stays off until it is switched on or the next
            `rti_connect()`.

        See also:
            `rti_claim()`, `rti_set_rate_limiter()`

        Examples:
            >>> robot.rti_connect(wait=True)
            >>> robot.rti_set_start_pose_check(True)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_start_pose_check", [1 if enabled else 0])

    def set_gravity_comp(self, enabled: bool) -> Any:
        """Switch gravity compensation on or off for the whole robot.

        With it on, the robot adds the torque that holds the arm up against its own
        weight, from its model at the measured pose. It acts wherever the robot
        drives the joints: EXEC and JOG moves and holds, real-time streams, and the
        joints already homed while homing goes on. The error state always holds
        every joint still, with or without it.

        In IDLE mode it also decides how the arm is held. On an enabled, homed arm
        with it on, IDLE is freedrive: the arm floats, can be moved by hand, and
        feels its joint walls and the haptic zones. With it off, IDLE holds every
        joint still instead.

        The call returns as soon as the robot accepts it. `latest_status()` shows
        the switch in `gravity_comp` once it is in force, and the robot's activity
        log records each change.

        Args:
            enabled: `True` to add gravity compensation, `False` to stop adding it.

        Defaults:
            On, set each time the robot software starts.

        Returns:
            Always `None`: the robot's answer carries no data.

        Raises:
            CommandRejected: The robot refused it: a joint is moving, a PC drives the
                arm in a real-time session, or another client holds control. Or RTI
                refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            REJECTED: A joint is commanded to move faster than 0.5 deg/s
                (`config/system.xml` `<safety_monitor><hold_still_dps>`), or a PC
                drives the arm in a real-time session: switch it with the arm at
                rest, and in a session use `rti_set_gravity_comp()`.
            BAD_ARGS: The value was not true or false (1 / 0); this client always
                sends one of them.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            The switch is a step in the torque the joints get, which is why the
            robot takes it only with the arm at rest. On, IDLE mode makes an
            enabled, homed arm move when pushed.

        Note:
            A real-time stream gets gravity compensation only while this switch and
            `rti_set_gravity_comp()` are both on.

        See also:
            `rti_set_gravity_comp()`, `set_payload()`, `set_friction_comp()`,
            `enter_idle_mode()`

        Examples:
            >>> robot.enter_idle_mode(wait=True)
            >>> robot.set_gravity_comp(False)  # IDLE now holds every joint still
            >>> robot.set_gravity_comp(True)   # freedrive again
        """
        return self._immediate("set_gravity_comp", [1 if enabled else 0])

    def set_inertia_comp(self, enabled: bool) -> Any:
        """Switch the inertia feedforward on or off for the whole robot.

        With it on, the robot adds the torque each joint needs to accelerate as
        commanded: for the arm's links and for the motors' rotors behind the
        gearboxes. It is computed from the commanded motion of each control cycle,
        never from a measured one.

        It acts in EXEC and JOG motion and in real-time position streams. There it
        uses the stream's own acceleration, and only while `rti_set_dynamics_ff()`
        is `"stream"` (as at the start of every real-time session) and
        `rti_set_rate_limiter()` is on. Freedrive, holds, homing, and real-time
        velocity and torque streams get none.

        The call returns as soon as the robot accepts it. `latest_status()` shows
        the switch in `inertia_comp` once it is in force, and the robot's activity
        log records each change.

        Args:
            enabled: `True` to add the inertia torque, `False` to stop adding it.

        Defaults:
            From `config/system.xml` `<dynamics_feedforward><inertia>`, read when the
            robot software starts: on as shipped, and when the element is missing.

        Returns:
            Always `None`: the robot's answer carries no data.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: The value was not true or false (1 / 0); this client always
                sends one of them.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            Off, the drives' own position control makes the accelerating torque, so
            the joints follow fast moves less closely.

        See also:
            `set_coriolis_comp()`, `set_friction_ff()`, `rti_set_dynamics_ff()`,
            `set_payload()`

        Examples:
            >>> robot.set_inertia_comp(False)
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
            >>> robot.set_inertia_comp(True)
        """
        return self._immediate("set_inertia_comp", [1 if enabled else 0])

    def set_can_read_first(self, enabled: bool) -> Any:
        """Choose whether the robot reads CAN first in each control tick.

        On, the robot reads the drives' CAN replies at the start of each tick,
        computes that tick's commands from them and sends the commands at once:
        about 3.8 ms from a drive's measurement to the command computed from it,
        instead of about 7.8 ms in the older order, which reads after sending.
        Measured with the same rate of late replies and the same tracking. Homing
        always uses the older order. The change takes effect on the robot's next
        tick, in any mode, and lasts until the robot software restarts. The
        status field `can_read_first` shows the order in use; it reads `False`
        while homing.

        Args:
            enabled: `True` reads first and sends early; `False` uses the older
                order.

        Defaults:
            From `config/system.xml` `<can_read_first>`, read each time the robot
            software starts: on (1).

        Returns:
            `None`: the robot sends no data back.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: A raw client sent a value other than true / false or 1 / 0.
            RTI_REFUSED: The read-first socket did not open when the robot software
                started (RTI.log says why): the old tick order stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            When the robot cannot open the CAN socket it needs to read first, it
            keeps the older order and says why in its log; the call still returns
            normally. Check the status field.

        See also:
            `latest_status()`, `get_health()`

        Examples:
            >>> robot.set_can_read_first(False)
            >>> robot.set_can_read_first(True)
        """
        return self._immediate("set_can_read_first", [1 if enabled else 0])

    def joint_test(self, joint: int, amplitude_deg: float = 5.0, speed_dps: float = 5.0,
                   current_ma: float = 600.0) -> Any:
        """Move one joint a few degrees and back, as a powered test.

        The joint moves `amplitude_deg` from where it stands and back on a smooth
        ramp that peaks at `speed_dps`, and rests 0.5 s at each end: 5 degrees at
        5 deg/s takes about 4 s. The drive works in its own encoder counts, so the
        test needs no homing, and its current is limited to `current_ma`
        meanwhile. The robot stops the test at once when the joint falls more than
        2 degrees behind the commanded motion, when the robot is disabled or
        leaves IDLE mode, and on any mode change, an error or the E-stop. The joint
        then gets its normal current limit and command back, and stays where it
        stopped.

        The call returns at once. The result is in the status, `joint_test`:
        `joint`, `state` (`"running"`, `"done"`, `"fell_behind"` or `"stopped"`),
        `amplitude_deg`, `moved_deg` (the largest displacement the commanded way),
        `max_error_deg` (the largest tracking error), `peak_current_ma`,
        `end_error_deg` (where it ended against its start) and `seq`, which counts
        finished tests. The figures are written when the test ends.

        Args:
            joint: The joint to move, a whole number: 1 for J1; 0 stops a running
                test.
            amplitude_deg: How far to move, in degrees, up to 15; the sign gives
                the direction.
            speed_dps: The peak speed, in degrees per second, up to 10.
            current_ma: The drive's current limit during the test, in milliamperes,
                up to the joint's own current limit. PAR6's first-run check uses
                400 to 1500 mA, 1.5 times the joint's largest gravity torque.

        Returns:
            `None`: the robot sends no data back.

        Raises:
            CommandRejected: The robot refused it: it is not in IDLE mode or not
                enabled, a value is out of range, or another client holds control. Or
                RTI refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: A value is not a number, or `joint` is not a whole number.
                Nothing is sent.
            TypeError: A value is `None`. Nothing is sent.

        Errors:
            BAD_ARGS: `joint` is not 0 to the number of joints; or, for a test,
                `amplitude_deg` is 0, above 15 degrees or not finite, `speed_dps` is
                not above 0 and up to 10 deg/s, or `current_ma` is not above 0 and
                up to the joint's current limit in force (`set_pid_gains()` may
                have changed it).
            JOINT_TEST_REFUSED: The test cannot start: the robot software started less
                than 1.2 s ago, another test, a freed joint or the Hall sensor check
                runs, the joint has not reported its position yet, or, on a homed arm,
                the move would pass the joint's soft limits. The message says why; `seq`
                does not change.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            The joint moves by itself, with no check of the safety zones or
            self-collision. On a homed arm the robot refuses a test that would
            move the joint past its soft limits; unhomed, the angles are unknown
            and nothing is checked. Only the 2-degree tracking limit and the
            current limit stop a running test. The other joints hold still
            meanwhile. Keep the space clear and a hand on the E-stop.

        See also:
            `free_joint()`, `hall_check()`, `latest_status()`, `wait()`

        Examples:
            >>> seq = robot.latest_status()["joint_test"]["seq"]
            >>> robot.joint_test(6, 5, 5, 400)
            >>> robot.wait(lambda s: s["joint_test"]["seq"] > seq, 10)
            >>> robot.latest_status()["joint_test"]["state"]
            'done'
        """
        return self._immediate("joint_test", [_whole(joint, "joint"), float(amplitude_deg), float(speed_dps),
                                              float(current_ma)])

    def free_joint(self, joint: int) -> Any:
        """Free one joint so it can be turned by hand.

        The joint's drive gets a current limit of 0: the joint makes no torque and
        turns freely, while its encoder keeps counting, so `get_joints()` follows
        the hand. Every other joint keeps its IDLE behaviour: it holds still or,
        on a homed, enabled arm with gravity compensation on, floats in freedrive.
        It works enabled or disabled, homed or not. One joint is free at a time:
        freeing another first holds the one that was free.

        The robot holds the joint again by itself on any mode change (homing, a
        jog, an error, the E-stop), and 2 minutes after the last `free_joint()`
        for that joint; calling it again for the same joint starts the 2 minutes
        again. `free_joint(0)` holds it now. A joint held again stays where the
        hand left it. The status field `free_joint` names the free joint, 0 when
        none is.

        Args:
            joint: The joint to free, 1 for J1; 0 holds the free joint again.

        Returns:
            `None`: the robot sends no data back.

        Raises:
            CommandRejected: The robot refused it: it is not in IDLE mode, `joint` is
                out of range, or another client holds control. Or RTI refused it or did
                not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `joint` is not a whole number. Nothing is sent.
            TypeError: `joint` is `None`. Nothing is sent.

        Errors:
            BAD_ARGS: `joint` is not a whole number from 0 to the number of joints.
            FREE_JOINT_REFUSED: The joint cannot be freed: the robot software started
                less than 1.2 s ago, or the Hall sensor check or a test move runs. The
                message says why; the status field stays 0.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            A freed joint holds nothing. A joint that carries the arm's weight,
            such as J2 or J3, falls at once unless someone supports the arm.

        See also:
            `joint_test()`, `hall_check()`, `get_joints()`

        Examples:
            >>> robot.free_joint(6)
            >>> robot.get_joints()[5]
            184.2
            >>> robot.free_joint(0)
        """
        return self._immediate("free_joint", [_whole(joint, "joint")])

    def hall_check(self, on: bool = True) -> Any:
        """Start or stop the Hall sensor check: its joint goes limp.

        The joint that homes on a Hall sensor (J6 on PAR6) gets a current limit of
        0 and turns freely, and the robot reads the sensor's pin every control
        tick. Turn the joint by hand through its magnet and watch the status field
        `hall_check`: `joint` (the limp joint, 0 while the check is off), `pin`
        (the pin's level, 0 or 1, `None` until read), `changes` (the pin changes
        seen) and `edge_deg` (the joint's angle in degrees at the last change to 0
        and at the last change to 1, `None` until seen). It works enabled or
        disabled, homed or not; every other joint keeps its IDLE behaviour.

        The robot stops the check by itself on any mode change (homing, a jog, an
        error, the E-stop) and gives the joint its limits back, and 2 minutes
        after the last `hall_check(True)`: calling it again while it runs starts
        the 2 minutes again. Stop it sooner with `hall_check(False)`.

        Args:
            on: `True` starts the check, `False` stops it.

        Returns:
            `None`: the robot sends no data back.

        Raises:
            CommandRejected: The robot refused it: it is not in IDLE mode, or another
                client holds control. Or RTI refused it or did not answer within 1 s
                (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: A raw client sent a value other than true / false or 1 / 0.
            HALL_CHECK_REFUSED: The check cannot start: the arm has no Hall sensor
                joint, the robot software started less than 1.2 s ago, or a joint is
                free or a test move runs. The message says why; `joint` stays 0.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            The joint holds nothing while the check runs: a load off its axis turns
            it by itself. Support the tool first.

        See also:
            `free_joint()`, `joint_test()`, `latest_status()`

        Examples:
            >>> robot.hall_check(True)
            >>> robot.wait(lambda s: s["hall_check"]["changes"] >= 2, 60)
            >>> robot.latest_status()["hall_check"]["edge_deg"]
            [181.4, 183.9]
            >>> robot.hall_check(False)
        """
        return self._immediate("hall_check", [1 if on else 0])

    def set_coriolis_comp(self, enabled: bool) -> Any:
        """Switch the Coriolis feedforward on or off for the whole robot.

        With it on, the robot adds the torque the joints exert on each other when
        several move at once (the Coriolis and centripetal effects), computed from
        the commanded position and speed of each control cycle.

        It acts where the inertia feedforward does: EXEC and JOG motion, and
        real-time position streams while `rti_set_dynamics_ff()` is `"stream"` and
        `rti_set_rate_limiter()` is on. Without a commanded speed nothing is added.

        The call returns as soon as the robot accepts it. `latest_status()` shows
        the switch in `coriolis_comp` once it is in force, and the robot's activity
        log records each change.

        Args:
            enabled: `True` to add the Coriolis torque, `False` to stop adding it.

        Defaults:
            From `config/system.xml` `<dynamics_feedforward><coriolis>`, read when
            the robot software starts: on as shipped, and when the element is
            missing.

        Returns:
            Always `None`: the robot's answer carries no data.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: The value was not true or false (1 / 0); this client always
                sends one of them.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            The torque is small at normal speeds; it grows with the square of the
            joint speeds.

        See also:
            `set_inertia_comp()`, `set_friction_ff()`, `rti_set_dynamics_ff()`

        Examples:
            >>> robot.set_coriolis_comp(False)
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
            >>> robot.set_coriolis_comp(True)
        """
        return self._immediate("set_coriolis_comp", [1 if enabled else 0])

    def set_friction_ff(self, enabled: bool) -> Any:
        """Switch the friction feedforward on or off for the whole robot.

        With it on, the robot adds the torque each joint's friction takes at the
        commanded speed, from the friction model in the robot XML (per joint
        `<friction_coulomb_nm>`, `<friction_viscous_nm_s>` and
        `<friction_load_per_nm>`). `<dynamics_feedforward><friction_fraction>` in
        `config/system.xml` sets the share of the model it adds: all of it as
        shipped.

        It acts in EXEC and JOG motion and in real-time position and velocity
        streams. A joint commanded to stand still gets none, and neither do
        freedrive (see `set_friction_comp()`), homing and real-time torque streams.
        It uses no measurement, so it cannot run away: each drive's own control
        corrects what is left.

        The call returns as soon as the robot accepts it. `latest_status()` shows
        the switch in `friction_ff` once it is in force, and the robot's activity
        log records each change.

        Args:
            enabled: `True` to add the friction torque, `False` to stop adding it.

        Defaults:
            From `config/system.xml` `<dynamics_feedforward><friction>`, read when
            the robot software starts: off as shipped, and when the element is
            missing.

        Returns:
            Always `None`: the robot's answer carries no data.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: The value was not true or false (1 / 0); this client always
                sends one of them.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            It ships off: measured on the arm, it did not make the joints follow
            their path more closely. Switch it off while you measure friction, or
            it hides what you measure.

        See also:
            `set_friction_comp()`, `set_inertia_comp()`, `set_coriolis_comp()`

        Examples:
            >>> robot.set_friction_ff(True)
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
            >>> robot.set_friction_ff(False)
        """
        return self._immediate("set_friction_ff", [1 if enabled else 0])

    def set_friction_comp(self, enabled: bool) -> Any:
        """Switch friction compensation in freedrive on or off.

        With it on, in freedrive (IDLE mode, on an enabled, homed arm with gravity
        compensation on), the robot pushes each joint gently in the direction it is
        already moving, to make up part of its friction. The push is a share of the
        joint's friction model in the robot XML, less the joint's gravity-model
        error (`<gravity_error_nm>`): a joint whose gravity model is off by as much
        as that share gets none.

        It gives nothing at rest, so static friction still holds a released arm. It
        is full from 2 deg/s and fades out between 20 and 40 deg/s, as shipped. The
        shares and speeds are `config/system.xml` `<friction_comp>`: `fraction`,
        `model_share`, `sliding_ratio`, `band_dps`, `fade_start_dps` and
        `fade_end_dps`, read when the robot software starts. Outside freedrive it
        has no effect; for motion, see `set_friction_ff()`.

        The call returns as soon as the robot accepts it. `latest_status()` shows
        the switch in `friction_comp` once it is in force, and the robot's activity
        log records each change.

        Args:
            enabled: `True` to help joints moved by hand, `False` to stop helping.

        Defaults:
            Off, set each time the robot software starts.

        Returns:
            Always `None`: the robot's answer carries no data.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: The value was not true or false (1 / 0); this client always
                sends one of them.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `set_gravity_comp()`, `set_friction_ff()`, `get_joint_walls()`

        Examples:
            >>> robot.enter_idle_mode(wait=True)
            >>> robot.set_friction_comp(True)
        """
        return self._immediate("set_friction_comp", [1 if enabled else 0])

    def set_collision_detection(self, mode: str) -> Any:
        """Set how the robot reacts when the arm hits something.

        The robot estimates the external torque on each joint from the motor
        currents and its model. While it moves a joint in EXEC, JOG or a real-time
        stream, it compares that torque with the one the joint had at rest before
        the move. Above the joint's contact threshold (robot XML
        `<contact_torque_nm>`) it shows a live `J<n>:CONTACT_WARNING`; above its
        collision threshold (`<collision_torque_nm>`) it reacts as `mode` says. A
        threshold of 0 leaves it unchecked on that joint.

        The check runs on a homed arm, while a joint is commanded faster than
        0.5 deg/s and for 0.5 s after; an excursion must last 0.012 s
        (`config/system.xml` `<safety_monitor>`: `<hold_still_dps>`,
        `<collision_settle_s>`, `<collision_sustain_s>`). It never runs in
        freedrive, homing or the error state. A warning clears once the joint is
        back under half its contact threshold, or at rest.

        A change lasts while the client that made it holds control. When that
        client releases control, loses it or disconnects, the reaction returns
        to its start value, and the robot's log says so. `"warn"` and `"stop"`
        are refused when nothing can be checked: the safety monitor is off in
        `config/system.xml`, or the robot XML gives no joint a threshold.
        `latest_status()` shows the reaction in force in `collision_detection`
        (`"off"` while the safety monitor is off); the robot's log records a
        switch to a weaker one as a warning.

        Args:
            mode: `"off"`: no contact or collision check. `"warn"`: a live
                `J<n>:COLLISION_WARNING`, and the robot keeps moving. `"stop"`: the
                error `J<n>:COLLISION`: the robot stops and is disabled, holding
                every joint, until `clear_errors()` and `enable()`; a running
                program is cancelled. Letter case and spaces around the word do not
                matter.

        Defaults:
            From `config/system.xml` `<safety_monitor><collision>`, read when the
            robot software starts: `"warn"` as shipped, and when the element is
            missing.

        Returns:
            Always `None`: the robot's answer carries no data.

        Raises:
            CommandRejected: The robot refused it: `mode` is not one of the three words,
                nothing can be checked, or another client holds control. Or RTI refused
                it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            REJECTED: `"warn"` or `"stop"` while nothing can be checked: the safety
                monitor is off, or no joint has a threshold in the robot XML.
            BAD_ARGS: `mode` is not `"off"`, `"warn"` or `"stop"`.
            RTI_REFUSED: RTI could not read the reaction. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            With `"off"` or `"warn"` the arm keeps pushing after it hits something,
            with all the force its drives give. Turn the check down only for a move
            that presses on purpose, and back up right after it; the robot turns it
            back up itself only when your client leaves control.

        Note:
            A part the gripper holds reads as external torque until it is declared
            with `set_payload()`. A switch takes a new rest torque at once: switch
            while the arm stands still.

        See also:
            `set_payload()`, `clear_errors()`, `errors()`, `latest_status()`

        Examples:
            >>> robot.set_collision_detection("off")
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
            >>> robot.set_collision_detection("warn")
        """
        return self._immediate("set_collision_detection", [str(mode).strip().lower()])

    def set_payload(self, mass_kg: float, com_m: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
        """Declare the part the gripper holds: its mass and centre of mass.

        The robot adds the part to the gripper's own body in its model and rebuilds
        the model at once, in every mode: gravity compensation, the inertia and
        Coriolis feedforward, the external torque estimate behind collision
        detection, and the inertia the freedrive walls push against. A declared
        part then floats with the arm in freedrive and does not read as a contact.
        While the estimate starts again, collision detection pauses for 0.1 s.

        Call it each time the gripper picks up or lets go of a part;
        `set_payload(0)` brings back the gripper alone. `latest_status()` shows the
        payload in force in `payload` (`mass`, `com`) shortly after the call
        returns.

        Args:
            mass_kg: The part's mass, in kilograms: from 0 up to the robot XML's
                `<max_payload_kg>` (0.4 kg on PAR6).
            com_m: The part's centre of mass `(x, y, z)`, in metres, in the tool
                frame: its origin is the tool centre point, and +z points from the
                flange toward the jaws. At most 0.5 m from that origin. The default
                puts it at the tool centre point.

        Defaults:
            0 kg (the gripper alone), set each time the robot software starts.

        Returns:
            `{"mass": kg, "com": [x, y, z], "max_payload_kg": kg}`: the payload
            asked for, in kilograms and metres, and the robot's maximum.

        Raises:
            CommandRejected: The robot refused it, for example before homing, with a
                payload out of bounds (`PAYLOAD_LIMIT`), or because another client holds
                control. Or RTI refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `mass_kg` or a value in `com_m` is not a number. Nothing is
                sent.
            TypeError: `mass_kg` is `None`, or `com_m` is not a sequence. Nothing is
                sent.

        Errors:
            PAYLOAD_LIMIT: The mass is below 0 or above `<max_payload_kg>`, a value
                is not finite, `com_m` does not hold exactly three numbers, or the
                centre of mass is more than 0.5 m from the tool centre point.
            RTI_REFUSED: RTI refused the payload. The message says why; the payload in
                force stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            Declare what the gripper really holds, and call `set_payload(0)` once it
            lets go. In freedrive a declared mass the arm does not carry pushes it
            up, and an undeclared one lets it sink.

        See also:
            `set_collision_detection()`, `set_gravity_comp()`, `latest_status()`

        Examples:
            >>> robot.set_payload(0.2, (0.0, 0.0, 0.03))
            {'mass': 0.2, 'com': [0.0, 0.0, 0.03], 'max_payload_kg': 0.4}
            >>> robot.set_payload(0)
        """
        return self._immediate("set_payload", [float(mass_kg), [float(v) for v in com_m]])

    def rti_set_gravity_comp(self, enabled: bool) -> None:
        """Switch gravity compensation on or off for real-time control.

        With it on, the robot adds the torque that holds the arm up against gravity
        to every real-time command and to its holds and stops, as in the other
        modes. Turn it off only when your controller computes gravity itself: a
        torque controller that also gets the robot's would apply it twice. It takes
        effect at once.

        It needs the robot-wide switch on too (`set_gravity_comp()`), and it
        changes nothing outside the real-time modes.

        Args:
            enabled: `True` to add gravity compensation, `False` to leave gravity to
                your controller.

        Defaults:
            On, set each time the robot software starts and at every
            `rti_connect()`; `rti_disconnect()` keeps it.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: The value is not true or false (1 / 0), or the message did
                not carry exactly one value; this client always sends one.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            Off, a torque stream that adds no gravity of its own lets the arm fall,
            and position streams and holds carry the weight on the drives' own
            position loops alone. It stays off until it is switched on or the next
            `rti_connect()`.

        See also:
            `set_gravity_comp()`, `rti_set_dynamics_ff()`

        Examples:
            >>> robot.rti_connect(wait=True)
            >>> robot.rti_set_gravity_comp(True)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_gravity_comp", [1 if enabled else 0])

    def rti_set_dynamics_ff(self, source: str) -> None:
        """Choose the inertia and Coriolis feedforward of real-time streams.

        With `"stream"` the robot estimates your position stream's own
        acceleration, the change of its speed between your packets smoothed at
        about 8 Hz, and feeds the torque that motion needs forward to the drives:
        the joints' inertia, and their Coriolis effect on each other. It helps
        tracking from about 25 Hz up and comes too late below about 10 Hz; noise
        in your targets shows as torque ripple. The robot-wide `set_inertia_comp()`
        and `set_coriolis_comp()` must be on too. With `"off"` nothing is fed
        forward: choose it when your PC computes its own dynamics.

        It acts on position streams while the rate limiter is on; velocity and
        torque commands get none. `rti_connect()` sets it again: call this after
        it.

        Args:
            source: `"stream"` or `"off"`, in any case. The robot ignores any other
                value and keeps the current one.

        Defaults:
            `"stream"`, set at every `rti_connect()`.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: `source` is empty or holds a colon, or the message did not
                carry exactly one value.
            RTI_REFUSED: RTI could not use the value. The message says why; the setting
                in force stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `set_inertia_comp()`, `set_coriolis_comp()`, `rti_set_gravity_comp()`

        Examples:
            >>> robot.rti_connect(rate_hz=50, wait=True)
            >>> robot.rti_set_dynamics_ff("off")
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_dynamics_ff", [str(source).strip().lower()])

    def rti_set_tcp_vel_limit(self, mps: float) -> None:
        """Cap the tool's straight-line speed under real-time control.

        The robot keeps the tool centre point's translational speed at or below
        `mps`, computed at the arm's measured pose. Position streams are slowed
        along the direction they travel, through the generator's speed budget, so
        the arm trails a stream that asks for more. Velocity commands are scaled
        down as a whole, which keeps their direction. Torque commands are not
        capped. It takes effect at once.

        Joint speed limits do not bound the tool's speed: near a singularity a slow
        joint can move the tool fast, and this cap is the guard for that case. The
        arm then crawls; that is the cap working. With the rate limiter off it
        still caps velocity commands, but not position streams. Without a
        kinematic model on the robot it does nothing, and RTI.log says so.

        Args:
            mps: The cap in metres per second. 0 switches it off; a negative or
                non-finite value is refused.

        Defaults:
            From the robot XML `<safety><max_tcp_velocity>`, read at start: 0 (off)
            on PAR6. Every `rti_connect()` sets it back to that value.

        Raises:
            CommandRejected: The robot refused it: `mps` is negative or not finite, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `mps` is not a number. Nothing is sent.
            TypeError: `mps` is not a number. Nothing is sent.

        Errors:
            BAD_ARGS: `mps` is negative or not a finite number, or the message did
                not carry exactly one value.
            RTI_REFUSED: RTI could not use the value. The message says why; the setting
                in force stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            Switching it off removes the only guard on the tool's speed near a
            singularity, until it is set again or the next `rti_connect()`.

        See also:
            `rti_set_vel_scale()`, `rti_set_rate_limiter()`

        Examples:
            >>> robot.rti_connect(rate_hz=50, wait=True)
            >>> robot.rti_set_tcp_vel_limit(0.25)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_tcp_vel_limit", [float(mps)])

    def rti_set_command_timeout(self, seconds: float) -> None:
        """Set how long the robot waits for the next real-time command.

        While control is claimed, a gap longer than this between two commands ends
        the stream: the arm brakes to rest within its limits, then the robot
        latches `RTI_LINK_LOST`, which disables it. Until then the robot carries on
        with the last command: a position stream along your last targets, a
        velocity or a torque command unchanged. It takes effect at once, also
        mid-stream.

        In RTI mode it can only shorten the 40 ms: a stale velocity or torque must
        not drive on for longer. In SOFT_RTI, which takes position targets only, it
        can be widened up to the operator's ceiling, the robot XML
        `<safety><max_soft_rti_timeout>` (0.25 s on PAR6), or up to three of your
        intervals when that is longer, never above 1 s. Below 8 ms (two ticks)
        counts as 8 ms. `rti_connect()` sets it again: call this after it.

        Args:
            seconds: The longest gap allowed between two commands, in seconds,
                above 0 and limited as described.

        Defaults:
            Set at every `rti_connect()`: 0.04 s in RTI mode; in SOFT_RTI three of
            your intervals (3 / `rate_hz`), at least 8 ms and at most 1 s. Before
            the first one it is `config/system.xml`
            `<rti_defaults><command_timeout_s>`, 0.04 s, read at start.

        Raises:
            CommandRejected: The robot refused it: `seconds` is not a number above 0, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `seconds` is not a number. Nothing is sent.
            TypeError: `seconds` is not a number. Nothing is sent.

        Errors:
            BAD_ARGS: `seconds` is not a finite number above 0, or the message did
                not carry exactly one value.
            RTI_REFUSED: RTI could not use the value. The message says why; the setting
                in force stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `rti_connect()`, `rti_claim()`

        Examples:
            >>> robot.rti_connect(rate_hz=20, wait=True)
            >>> robot.rti_set_command_timeout(0.2)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_command_timeout", [float(seconds)])

    def rti_set_vel_scale(self, fraction: float) -> None:
        """Scale the real-time joint speed limits down.

        Every joint's real-time velocity limit becomes `fraction` of its value in
        the robot XML (`<rti_limits><velocity>`). It can only lower the limits, and
        each call scales from the XML values, so calls do not add up. It takes
        effect at once. It limits the servo generators, velocity commands while the
        rate limiter is on, and the first-command check; acceleration and jerk
        keep their limits, and the RTI-mode stream checks still use the full ones.

        In SOFT_RTI it smooths a slow stream: a generator capped near your travel
        speed glides between your targets instead of sprinting to each and
        waiting. Set it to about your travel speed divided by the joint's limit. It
        acts like a cliff: above your travel speed it changes nothing, and at it
        the arm has no speed in reserve to catch up after a delay.

        Args:
            fraction: The share of the robot XML limit, above 0 and at most 1: a
                fraction, not a percent. Below 0.01 counts as 0.01; zero, a
                negative, a non-finite value or one above 1 is refused.

        Defaults:
            1 (the robot XML limits), set each time the robot software starts. It
            lasts until changed or the robot software restarts; `rti_connect()`
            does not reset it.

        Raises:
            CommandRejected: The robot refused it: `fraction` is not above 0 and at most
                1, or another client holds control. Or RTI refused it or did not answer
                within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `fraction` is not a number. Nothing is sent.
            TypeError: `fraction` is not a number. Nothing is sent.

        Errors:
            BAD_ARGS: `fraction` is not a number above 0 and at most 1, or the
                message did not carry exactly one value.
            RTI_REFUSED: RTI could not use the value. The message says why; the setting
                in force stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            It lasts across sessions: set it back to 1 when your program is done,
            or the next session, from any PC, runs slower.

        See also:
            `rti_set_pace()`, `rti_set_min_duration()`, `rti_set_tcp_vel_limit()`

        Examples:
            >>> robot.rti_connect(rate_hz=20, wait=True)
            >>> robot.rti_set_vel_scale(0.1)
            >>> robot.rti_set_vel_scale(1)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_vel_scale", [float(fraction)])

    def rti_set_min_duration(self, multiplier: float) -> None:
        """Make each planned move last at least a multiple of your interval.

        The robot measures the interval between your position targets and plans
        each move toward one to last at least `multiplier` of those intervals.
        From about 1.5 up each plan outlives the next target, so the arm glides
        from target to target instead of braking at each; the price is lag, about
        one interval. At 1 the plan still ends as the next target lands, and the
        motion still pulses. It takes effect at once.

        It acts with `"ruckig"` and `"ruckig_advancing"` only, and makes the joints
        share one move duration. `rti_connect()` sets it again: call this after
        it.

        Args:
            multiplier: Command intervals, from 1 to 5 (clamped); 0 switches it
                off. A negative or non-finite value is refused.

        Defaults:
            Set at every `rti_connect()`: 1.5 in SOFT_RTI, 0 (off) in RTI mode.

        Raises:
            CommandRejected: The robot refused it: `multiplier` is negative or not
                finite, or another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `multiplier` is not a number. Nothing is sent.
            TypeError: `multiplier` is not a number. Nothing is sent.

        Errors:
            BAD_ARGS: `multiplier` is negative or not a finite number, or the
                message did not carry exactly one value.
            RTI_REFUSED: RTI could not use the value. The message says why; the setting
                in force stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `rti_set_servo_generator()`, `rti_set_pace()`, `rti_set_vel_scale()`

        Examples:
            >>> robot.rti_connect(rate_hz=50, wait=True)
            >>> robot.rti_set_min_duration(1.2)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_min_duration", [float(multiplier)])



    def take_control(self, force: bool = False, timeout: float | None = None) -> dict:
        """Take control of the robot for this client.

        While a client holds control, the robot refuses every other client the
        commands that take control, with `CONTROL_HELD`; the commands open to every
        client still work for all, stopping included. Usually no call is needed:
        the first command that takes control claims it when nobody holds it. Call
        it to reserve the robot before a program starts, or with `force=True` to
        take control from another client.

        Control is released by `release_control()`, when the client disconnects,
        or after 30 s without a command that takes control, with nothing queued
        and no real-time stream running. When control leaves a client this way
        or by a takeover, a collision-detection change it made returns to the
        start value (`set_collision_detection()`).

        With the takeover confirmation on (`set_force_confirm()`), a forced take
        from another client waits up to 20 s for someone at the control box:
        holding button 2 for 1 s allows it, button 1 refuses it.

        Args:
            force: Take control even when another client holds it.
            timeout: Seconds to wait for the robot's answer. `None` means 30 s with
                `force=True`, time for the button press, and 5 s otherwise.

        Returns:
            `{"holder": label}`, the client that now holds control: its address as
            the robot sees it, "ip:port", as `control_holder()` reports it.

        Raises:
            CommandRejected: The robot refused it; its error code says why.
            RobotTimeout: The robot did not answer within `timeout`.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: `force` was not true or false (a raw client's 1 / 0 also
                count).
            CONFIRM_BUSY: Another forced takeover is already waiting at the
                control box.
            CONTROL_HELD: Another client holds control, and `force` is false.
            CONFIRM_REQUIRED: The forced takeover was not confirmed: refused or not
                answered at the control box within 20 s, button 2 already held
                when it started, or this client disconnected meanwhile.
            INTERNAL: The confirmation failed inside the robot.

        See also:
            `release_control()`, `control_holder()`, `holds_control()`,
            `set_force_confirm()`

        Examples:
            >>> robot.take_control()
            {'holder': '192.168.1.20:51234'}
            >>> robot.release_control()
        """
        # A FORCED take can block on the robot for the whole physical-confirmation window
        # (~20 s: someone has to walk over and press the button). IMMEDIATE_TIMEOUT is 5 s,
        # so without a longer wait here the client gives up while the robot is still waiting
        # — and if the press then lands, control is granted to a caller that already
        # abandoned the request and does not know it holds the arm.
        if timeout is None:
            timeout = FORCE_TAKE_TIMEOUT if force else IMMEDIATE_TIMEOUT
        return self._immediate("take_control", [bool(force)], timeout=timeout)

    def set_force_confirm(self, enabled: bool) -> dict:
        """Require a button press at the robot for a forced takeover.

        With it on, `take_control(force=True)` from a client that does not hold
        control succeeds only when someone at the control box confirms it within
        20 s: holding button 2 for 1 s allows it, button 1 refuses it. With it
        off, any client on the network can take control from another. A claim on
        a robot that nobody controls never needs the button.

        Any client can switch it on. Only the client that holds control can
        switch it off, or any client while nobody holds control.

        Args:
            enabled: `True` to require the button press, `False` to stop requiring
                it.

        Defaults:
            Off, each time the robot software starts.

        Returns:
            `{"force_confirm": bool}`: the setting now in force.

        Raises:
            CommandRejected: The robot refused it; its error code says why.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: `enabled` was not true or false (a raw client's 1 / 0 also
                count).
            CONTROL_HELD: `enabled` is false and another client holds control.

        See also:
            `take_control()`

        Examples:
            >>> robot.set_force_confirm(True)
            {'force_confirm': True}
        """
        return self._immediate("set_force_confirm", [bool(enabled)])

    def release_control(self) -> Any:
        """Give up control of the robot.

        Another client can then take control. It is never refused: a client that
        does not hold control gets `released` false. Moves already queued keep
        running. A collision-detection change this client made returns to its
        start value.

        Returns:
            `{"released": bool, "holder": label or None}`: whether this client held
            control and gave it up, and who holds control now ("ip:port").

        Raises:
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        See also:
            `take_control()`, `control_holder()`

        Examples:
            >>> robot.take_control()
            >>> robot.release_control()
            {'released': True, 'holder': None}
        """
        return self._immediate("release_control")

    def control_holder(self) -> str | None:
        """Return the client that holds control, or `None`.

        It reads the latest status the robot sent, and asks the robot nothing.
        The robot names a client by its address as it sees it, "ip:port". Control
        that has expired reads as `None`, as it does before the first status
        arrives after connecting.

        Returns:
            The holder's "ip:port", or `None` when no client holds control.

        See also:
            `holds_control()`, `take_control()`

        Examples:
            >>> robot.control_holder()
            '192.168.1.20:51234'
        """
        return self.latest_status().get("control_holder")

    def local_label(self) -> str | None:
        """Return this connection's address as the robot names it.

        The robot names each client connection by its address and port,
        "ip:port", and reports the holder of control that way. Compare the two to
        see whether this connection holds control, as `holds_control()` does. It
        reads the client's own connection and asks the robot nothing. Behind a
        router that translates addresses (NAT), the robot sees another address
        and the two never match.

        Returns:
            This connection's "ip:port", for example `"192.168.0.20:51234"`;
            `None` before `connect()` and after `disconnect()`.

        See also:
            `control_holder()`, `holds_control()`, `local_ip()`

        Examples:
            >>> robot.take_control()
            >>> robot.control_holder() == robot.local_label()
            True
        """
        return self._conn.local_label()

    def holds_control(self) -> bool:
        """Return whether this client holds control.

        It compares `control_holder()` with this connection's own address, so it
        reads the latest status and asks the robot nothing. Behind a router that
        translates addresses (NAT) the robot sees another address, and it returns
        `False`. Right after a command claimed control on its own, the status can
        lag by about 20 ms.

        Returns:
            `True` when this connection holds control; `False` otherwise, also when
            the client is not connected.

        See also:
            `control_holder()`, `take_control()`

        Examples:
            >>> robot.take_control()
            >>> robot.holds_control()
            True
        """
        holder = self.control_holder()
        return holder is not None and holder == self.local_label()

    def reload_recipes(self) -> None:
        """Read the telemetry recipe files again and use them at once.

        The robot reads every `.xml` file in its `data_recipe/` folder and replaces
        the recipes it holds with the ones that parse: a new file becomes usable by
        `set_recipe()`, an edited one takes effect, a removed one is dropped. A
        file that does not parse is skipped, with a line in the robot's log; when
        none parses, the recipes in force stay. No restart and no homing are
        needed.

        Raises:
            CommandRejected: The robot refused it: the folder could not be read, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RECIPE_DIR: The recipe folder does not exist, or the robot could not
                read it or a file in it; the recipes in force stay.
            RTI_REFUSED: RTI could not use the recipes the robot read. The message says
                why; the recipes in force stay.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            When the file of the recipe in use is removed and the recipes are
            reloaded, the data streams carry only the joint positions and the tick,
            while the status field `recipe` still names the old recipe. Choose
            another one with `set_recipe()`.

        See also:
            `set_recipe()`, `reload_filters()`

        Examples:
            >>> robot.reload_recipes()
            >>> robot.set_recipe("diagnostics")
        """
        self._immediate("reload_recipes")

    def reload_filters(self) -> None:
        """Read the signal filter file again and apply it at once.

        The robot parses `config/filters.xml`, which sets the filter of each
        derived signal: the filtered joint speed, acceleration, jerk and torque,
        the tool centre point's acceleration and the gripper's speed. When the
        whole file is valid, the real-time loop rebuilds its filters from it on
        its next tick; no restart and no homing are needed. A signal the file
        leaves out gets its built-in default. An invalid file is refused, and the
        filters in force stay.

        Raises:
            CommandRejected: The robot refused it: the file is invalid, or another
                client holds control. Or RTI refused it or did not answer within 1 s
                (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            FILTER_CONFIG: The file is missing, does not parse, is written for a
                newer robot software, or sets an invalid filter, such as an unknown
                type, a median filter with an even window or an exponential filter
                without a time constant. The message says where.
            RTI_REFUSED: RTI could not use the filters the robot read. The message says
                why; the filters in force stay.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            Every filter starts again, so the filtered values, and what the robot
            computes from them (the tool's speed and acceleration, the external
            torque estimate, freedrive's friction compensation), settle again over
            about one filter time constant. Reload with the arm at rest.

        See also:
            `reload_recipes()`

        Examples:
            >>> robot.reload_filters()
        """
        self._immediate("reload_filters")

    def rti_set_state_rate(self, hz: float) -> None:
        """Set how often the robot sends its state packet in SOFT_RTI.

        The robot sends a state packet every 4 ms tick, 250 a second, however
        slowly you command. This lowers the rate: the robot then sends one every
        250 / `hz` ticks, rounded. It matters most over TCP, where state is many
        times your command traffic: a PC that reads too slowly fills the robot's
        send buffer, and the robot then closes the stream. `set_recipe()` makes
        each packet smaller.

        There is no off: the state stream is how `RTIClient` notices that the
        robot left real-time control. In RTI mode, where you close the loop every
        tick, the robot ignores this call; send it after a SOFT_RTI
        `rti_connect()`.

        Args:
            hz: Packets per second, from 10 to 250 (clamped); 0 sends every tick
                again. A negative or non-finite value is refused.

        Defaults:
            Every tick (250 per second), set each time the robot software starts
            and at every `rti_connect()` in RTI mode. A SOFT_RTI `rti_connect()`
            keeps the value it finds, so it lasts until changed.

        Raises:
            CommandRejected: The robot refused it: `hz` is negative or not finite, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `hz` is not a number. Nothing is sent.
            TypeError: `hz` is not a number. Nothing is sent.

        Errors:
            BAD_ARGS: `hz` is negative or not a finite number, or the message did
                not carry exactly one value.
            RTI_REFUSED: In RTI mode (hard RTI) the state stream follows your packets:
                give `rti_connect()` a `rate_hz` (SOFT_RTI) to throttle it. Or RTI could
                not use the value. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            `RTIClient.control()` waits 0.1 s for each packet by default, the very
            gap between packets at 10 Hz: at low rates give it a longer `timeout`.

        See also:
            `set_recipe()`, `rti_connect()`

        Examples:
            >>> robot.rti_connect(rate_hz=10, wait=True)
            >>> robot.rti_set_state_rate(50)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_state_rate", [float(hz)])

    def rti_set_pace(self, factor: float) -> None:
        """Cap the generator's speed at a multiple of your target's speed.

        The robot measures how fast your position targets move and caps the
        generator's speed, per joint, at `factor` times that, never below
        0.02 rad/s and never above the joint's limits. Like `rti_set_vel_scale()`
        it stops a slow stream from pulsing, but it follows you when you change
        speed or rate. At 1 the arm cannot catch up once it falls behind; above
        about 3 the cap no longer binds. It takes effect at once.

        It acts with `"ruckig"`, `"ruckig_advancing"` and `"passthrough"`, not with
        `"tracker"`.

        Args:
            factor: The headroom over your target's speed, up to 10 (clamped); 0
                switches it off. A negative or non-finite value is refused.

        Defaults:
            0 (off), set each time the robot software starts and at every
            `rti_connect()` in RTI mode. A SOFT_RTI `rti_connect()` keeps the value
            it finds, so it lasts until changed.

        Raises:
            CommandRejected: The robot refused it: `factor` is negative or not finite,
                or another client holds control. Or RTI refused it or did not answer
                within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `factor` is not a number. Nothing is sent.
            TypeError: `factor` is not a number. Nothing is sent.

        Errors:
            BAD_ARGS: `factor` is negative or not a finite number, or the message
                did not carry exactly one value.
            RTI_REFUSED: RTI could not use the value. The message says why; the setting
                in force stays.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `rti_set_vel_scale()`, `rti_set_min_duration()`,
            `rti_set_servo_generator()`

        Examples:
            >>> robot.rti_connect(rate_hz=20, wait=True)
            >>> robot.rti_set_pace(1.5)
            >>> robot.rti_set_pace(0)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_pace", [float(factor)])

    def rti_set_servo_generator(self, name: str, gain: float | None = None, lookahead: float | None = None, self_lookahead: float | None = None) -> None:
        """Choose how the robot fills in the ticks between position targets.

        The generator turns your position targets into one setpoint per tick,
        within the joints' real-time limits and soft limits. It acts on position
        streams while the rate limiter is on. A change takes effect at once: the
        new generator continues from the position and speed the old one reached.
        `rti_connect()` sets the generator again, so call this after it. A gain or
        a look-ahead that is not finite is refused, and nothing changes.

        Args:
            name: `"passthrough"`: RTI mode only; replays your stamped targets
                exactly, one tick behind, by their send times, and checks every
                sample against the limits. `"ruckig_advancing"`: plans toward where
                your target is moving and passes through it at its speed; the
                closest tracking of a sparse stream. `"ruckig"`: plans a
                jerk-limited move to each target and stops on it; exact and never
                overshoots, but on a sparse stream it sprints and waits.
                `"tracker"`: a servo law that never plans to stop; smooth on any
                stream, but a large step can overshoot. Any case. The robot ignores
                another name, and `"passthrough"` in SOFT_RTI.
            gain: The tracker's bandwidth in rad/s, from 1 to 150 (clamped); above
                about 20 a large step overshoots. `None` keeps the current value.
                Only the tracker uses it.
            lookahead: How far ahead the tracker extrapolates your target, in
                seconds, from 0 to 0.5 (clamped). `None` keeps the current value,
                or, with `gain` given, sets 2 / `gain`, which cancels the lag on a
                target moving at a steady speed.
            self_lookahead: The tracker's damping look-ahead, in seconds, from 0 to
                0.5 (clamped): its aim moves back by its own speed times this. `None`
                keeps the current value.

        Defaults:
            The generator is set at every `rti_connect()`: `"passthrough"` in RTI
            mode, `"ruckig_advancing"` in SOFT_RTI. The tracker's `gain` 15 rad/s,
            `lookahead` 2 / 15 s (about 0.133 s) and `self_lookahead` 0 are set each
            time the robot software starts and last until changed.

        Raises:
            CommandRejected: The robot refused it: a number is not finite, or another
                client holds control. Or RTI refused it or did not answer within 1 s
                (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `gain`, `lookahead` or `self_lookahead` is not a number.
                Nothing is sent.
            TypeError: `gain`, `lookahead` or `self_lookahead` is not a number.
                Nothing is sent.

        Errors:
            BAD_ARGS: `name` is empty or holds a colon, a number is not finite, or
                the message did not carry exactly four values.
            RTI_REFUSED: `"passthrough"` in SOFT_RTI (it needs a send time with every
                target), or RTI could not use a value. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            In RTI mode only `"passthrough"` checks the stream itself (the speed,
            acceleration, jerk and soft-limit errors); the others absorb a jump
            within the limits instead. Every one still needs each position
            target's send time in RTI mode.

        See also:
            `rti_set_min_duration()`, `rti_set_pace()`, `rti_set_vel_scale()`

        Examples:
            >>> robot.rti_connect(rate_hz=10, wait=True)
            >>> robot.rti_set_servo_generator("tracker", gain=15)
            >>> robot.rti_disconnect(wait=True)
        """
        self._immediate("rti_set_servo_generator",
                        [str(name),
                         "" if gain is None else float(gain),
                         "" if lookahead is None else float(lookahead),
                         "" if self_lookahead is None else float(self_lookahead)])

    def set_recipe(self, name: str) -> None:
        """Choose the telemetry recipe that the robot's data streams carry.

        A recipe is a file in the robot's `data_recipe/` folder that names the
        values sent every control tick, both in the telemetry the robot broadcasts
        on UDP port 30003 and in the state packets of a real-time stream. The
        robot switches on its next tick. The status field `recipe` names the
        recipe in use, and `recipes` lists the recipe files on the robot.

        Args:
            name: The recipe. The robot ships `"standard"` (joint positions and
                speeds, the drives' errors, the tool centre point's pose, the tick,
                the mode, the loop period and the real-time link), `"minimal"`
                (joint positions and the tick), `"commanded"` (measured, commanded
                and target positions), `"diagnostics"` (the real-time link's
                quality, the loop's timing, errors and external torques), `"full"`,
                and `"tracking"`, `"can_gaps"`, `"external_test"`, `"friction_id"`,
                `"haptic_wall"` and `"monitor_baseline"` for the robot's own tests.
                A recipe file added while the robot software runs needs
                `reload_recipes()` first.

        Defaults:
            `"standard"`, set each time the real-time loop starts. The robot's
            other processes keep the recipe in use when they restart.

        Raises:
            CommandRejected: The robot refused it: no recipe file has that name, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: No recipe file of that name is on the robot.
            RTI_REFUSED: RTI does not know the recipe. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            The real-time loop checks the name again, in lower case, against the
            recipes it has loaded. A file added but not reloaded, or a recipe whose
            name inside the file has capitals or differs from its file name, is
            refused there, in its log only; the call still returns normally. Check
            the status field `recipe`.

        See also:
            `reload_recipes()`, `rti_set_state_rate()`, `rti_connect()`

        Examples:
            >>> robot.set_recipe("diagnostics")
        """
        self._immediate("set_recipe", [str(name)])

    def stop(self, wait: bool | None = None) -> None:
        """Stop the robot's motion and cancel every queued move.

        In EXEC mode the arm brakes along its planned path at the joints'
        acceleration limits (at most about 0.15 s from full speed) and holds
        there, still in EXEC mode. Every client's queued moves are cancelled:
        those commands fail with `STOPPED`, and the gripper's jaws stop where
        they are. The robot accepts new moves at once,
        and the next one starts where the arm stopped. The robot answers once the
        arm is at rest, within 1.5 s.

        In any other mode the robot switches to IDLE mode at once, and answers at
        once; a homing in progress is abandoned, and the arm is then not homed.
        In the error state it has no effect.

        It also discards this client's failures of earlier moves that
        `wait_idle()` has not reported yet, and its last commands:
        `wait_motion_complete()` then returns at once.

        Args:
            wait: Wait until the arm has stopped: in EXEC mode until braking has
                ended, elsewhere until the robot reports IDLE mode; up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: RTI refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, the
                arm had not stopped within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_REFUSED: RTI could not read the stop it was sent: a fault, not something
                a program causes. RTI.log has the details.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            Every client can send it. Outside EXEC mode it does not brake: from JOG
            mode or a real-time stream at speed the arm coasts in freedrive until
            friction stops it. It also ends motors off and firmware flashing: the
            motors hold again, and during a flash the robot's CAN traffic resumes,
            which can make the flash fail.

        See also:
            `pause()`, `enter_idle_mode()`, `safety_stop()`, `wait_idle()`

        Examples:
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.stop(wait=True)
        """
        self._state_change("stop", wait, lambda s: s.get("mode") == "IDLE"
                           or (s.get("mode") == "EXEC" and not s.get("exec_stopping")), what="stop")
        # stop() cancels queued motion server-side, which replies "error"/STOPPED for
        # each one. Those are the expected consequence of stopping, not a motion
        # failure — drop them so they can't surface at a later wait_idle(). Safe to do
        # here: the executor writes those replies before stop's own "ok", so the
        # receiver thread has already recorded them by the time _state_change returns.
        self._conn.take_motion_errors()
        # ... and the last commands with them (2026-10-24): wait_motion_complete() raised their STOPPED on every call
        # until the next command.
        self._last_queued = self._last_gripper_queued = None

    def home(self, wait: bool | None = None) -> None:
        """Home the arm: find each joint's reference position.

        The arm moves each joint to its endstop, in the order of the robot's
        homing sequence, and sets the joint's reference there; the gripper is
        calibrated too. Then the robot returns to IDLE mode. It takes about half a
        minute. A homed arm can be homed again.

        A failed homing latches an error (`J<n>:HOMING_FAILED` or
        `GRIPPER_HOMING_FAILED`): the robot is then in the error state, disabled
        and not homed. An error during homing ends it the same way. Leaving it
        with `enter_idle_mode()`, `stop()` or `disable()` abandons it, and the arm
        is then not homed (`HOMING_ABORTED` in the activity log): home it again.

        Args:
            wait: Wait until homing has ended, up to 3 minutes.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot refused it: it is not in IDLE mode or not
                enabled, or another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            HomingFailed: With `wait`, homing failed. It names the robot's error
                keys.
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`,
                homing had not ended after 3 minutes.
            RobotConnectionError: The client is not connected.

        Errors:
            MODE_REJECTED: RTI refused HOMING, for example while an error is latched
                (`clear_errors()` first). The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            The arm moves by itself to every joint's endstop. Keep the space around
            it clear.

        See also:
            `enable()`, `clear_errors()`, `enter_exec_mode()`

        Examples:
            >>> robot.enable(wait=True)
            >>> robot.home(wait=True)
        """
        self._immediate("home")
        if self._should_wait(wait):
            # Homing starts a tick later; wait for it to begin, then to finish. The
            # homed flag can read stale, so completion is keyed off mode leaving HOMING.
            try:
                self.wait_mode("HOMING", timeout=3.0)
            except RobotTimeout:
                pass   # never entered HOMING (already homed / instant) — check below
            self._wait_homed(WAIT_HOMED_TIMEOUT, "home")

    def _wait_homed(self, timeout, what):
        """Until homed and out of HOMING — or out of HOMING with an error (a failed homing latches one, 2026-10-14):
        then HomingFailed at once, naming the robot's error keys."""
        self.wait(lambda s: s.get("mode") != "HOMING" and (s.get("homed") is True or s.get("err") == 1),
                  timeout, what)
        st = self.latest_status()
        if st.get("homed") is not True and st.get("err") == 1:
            raise HomingFailed(st.get("errors") or [])

    def pause(self, wait: bool | None = None) -> None:
        """Pause the running program: brake along its path and hold.

        The arm brakes along its planned path, at the same limits as `stop()`, and
        holds there in EXEC mode. Nothing is cancelled: moves queued later wait
        too, and `resume()` continues from exactly this point. Entering or leaving
        EXEC mode, or `stop()`, ends the pause. While a stop is braking, the robot
        ignores a pause.

        Every client can pause a program; only the client that holds control can
        resume it.

        Args:
            wait: Wait until the robot reports the program paused, up to 10 s: its
                planned motion has come to rest.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot is not in EXEC mode. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report the program paused within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `resume()`, `stop()`

        Examples:
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.pause(wait=True)
            >>> robot.resume()
            >>> robot.wait_idle()
        """
        self._state_change("pause", wait, lambda s: bool(s.get("exec_paused")))

    def resume(self, wait: bool | None = None) -> None:
        """Resume a paused program from where it stopped.

        The arm speeds back up along the same path, at the robot's EXEC
        acceleration limits, and the program continues. Without a pause it
        changes nothing. While a stop is braking, the robot ignores it.

        Args:
            wait: Wait until the robot no longer reports a pause, up to 10 s;
                without one it returns at once.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot refused it: it is not in EXEC mode, or another
                client holds control. Or RTI refused it or did not answer within 1 s
                (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`,
                still reported the pause after 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `pause()`, `stop()`

        Examples:
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.pause(wait=True)
            >>> robot.resume()
            >>> robot.wait_idle()
        """
        self._state_change("resume", wait, lambda s: not s.get("exec_paused"))

    def set_joint_speed(self, percent: float) -> Any:
        """Set the default speed of queued moves, as a share of the limits.

        Every queued move that does not pass its own `speed` runs at this share of
        each joint's speed limit. It slows the whole move down in time: at 50 % a
        move takes twice as long along the same path, with half the speed and a
        quarter of the acceleration. A `duration` given with a move overrides it.

        The setting is the robot's, not this client's: it applies to the moves every
        client sends after it, unless a move passes its own `speed`. Moves already
        sent keep the value they were sent with. It lasts until it is changed or the
        robot software restarts.

        Args:
            percent: The share of each joint's speed limit, in %: above 0 and at
                most 100. `1` means 1 %, and `0.5` half a percent.

        Defaults:
            100 %, set each time the robot software starts.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: The robot refused it: `percent` is not above 0 and at
                most 100, or another client holds control.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `percent` cannot be read as a number, for example `"fast"`.
                Nothing is sent.
            TypeError: `percent` is `None` or another type that is no number.
                Nothing is sent.

        Errors:
            BAD_ARGS: `percent` is not a number above 0 and at most 100.

        Note:
            Moves along a Cartesian path (straight lines, arcs, circles and splines)
            run at no more than 20 % of the speed limit, whatever this says. No
            status field shows the setting.

        See also:
            `set_profile()`, `set_blending()`, `move_joints()`

        Examples:
            >>> robot.set_joint_speed(50)
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
        """
        return self._immediate("set_joint_speed", [float(percent)])

    def set_blending(self, percent: float) -> Any:
        """Set the default blend: how queued moves join at their corners.

        Every queued move that does not pass its own `blend` takes this value. What
        the number means depends on the move's `blend_mode`, which each move passes
        for itself (`"exact_velocity"` when it passes none):

        - `"exact_velocity"`: the percent of the corner speed kept, from 0 to 100.
          It works between two `move_joints()` moves in a row with the `"ruckig"` or
          `"trapezoid"` profile, when the second one uses the same profile and is
          already queued while the first is planned (so never in sync mode). Every
          other move stops at its end.
        - `"zone"`: the radius of the rounded corner, in millimetres.
        - `"tolerance"`: the largest distance from the programmed path, in
          millimetres.
        - `"ruckig_corner"`: the size of the corner cut, in millimetres.

        `"zone"` and `"tolerance"` need the `"toppra"` profile, `"ruckig_corner"`
        the `"ruckig"` profile; see Per-move settings. The setting is the robot's:
        it applies to the moves every client sends after it, and moves already sent
        keep their value. It lasts until it is changed or the robot software
        restarts.

        Args:
            percent: The blend value: a percent with `"exact_velocity"` (a value
                above 100 counts as 100), millimetres with the other modes. 0 means
                no blending; a negative value is refused.

        Defaults:
            0 (no blending), set each time the robot software starts.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: The robot refused it: `percent` is negative or not a
                finite number, or another client holds control.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `percent` cannot be read as a number, for example `"some"`.
                Nothing is sent.
            TypeError: `percent` is `None` or another type that is no number.
                Nothing is sent.

        Errors:
            BAD_ARGS: `percent` is negative or not a finite number.

        Note:
            Arcs, circles and splines cannot blend yet: they ignore this value, and
            one that passes its own `blend` above 0 fails with
            `CURVE_BLEND_UNSUPPORTED`.

        See also:
            `set_profile()`, `set_joint_speed()`, `move_joints()`, `move_linear()`

        Examples:
            >>> robot.set_blending(50)
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.move_joints(95, -105, 165, 0, -30, 180)
            >>> robot.move_joints(95, -100, 160, 0, -30, 180)
            >>> robot.wait_idle()
            >>> robot.set_blending(0)
        """
        return self._immediate("set_blending", [float(percent)])

    def set_profile(self, profile: str) -> Any:
        """Set the default profile: how queued moves speed up and slow down.

        Every queued move that does not pass its own `profile` is timed with this
        one, joint moves and Cartesian moves alike. The robot knows these profiles,
        by these names and their other spellings:

        - `"ruckig"`: limits speed, acceleration and jerk. Smooth starts and stops,
          with no ringing after a fast stop.
        - `"trapezoid"` (also `"trapezoidal"`, `"trap"`): limits speed and
          acceleration. Shorter moves; the arm can ring briefly after a full-speed
          stop.
        - `"quintic"` (also `"poly5"`): a smooth polynomial shape that starts and
          ends at rest.
        - `"s_curve"` (also `"s-curve"`, `"scurve"`, `"smoothstep7"`): a smooth
          fixed shape, with the gentlest jerk of the fixed shapes.
        - `"toppra"`: the fastest timing along the path within the speed and
          acceleration limits. Needed for `"zone"` and `"tolerance"` blending.

        Case and spaces around the name do not matter. The setting is the robot's:
        it applies to the moves every client sends after it, and moves already sent
        keep their profile. It lasts until it is changed or the robot software
        restarts.

        Args:
            profile: The profile's name, one of those above.

        Defaults:
            `"ruckig"`, set each time the robot software starts.

        Returns:
            `{"profile": name}`: the profile now in force, by its main name, for
            example `"trapezoid"` for `"trap"`.

        Raises:
            CommandRejected: The robot refused it: the name is not a known profile,
                or another client holds control.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: No profile was given (only a raw client can send that).
            BAD_PROFILE: `profile` is not one of the names above.

        Note:
            No status field shows this default: the status's `current_profile` is
            the profile of the move that started last.

        See also:
            `set_joint_speed()`, `set_blending()`, `move_joints()`

        Examples:
            >>> robot.set_profile("trapezoid")
            {'profile': 'trapezoid'}
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
            >>> robot.set_profile("ruckig")
        """
        return self._immediate("set_profile", [str(profile).strip().lower()])

    set_default_profile = set_profile      # its other name (2026-10-23)

    def set_ik_solver(self, solver: str) -> Any:
        """Set the default IK solver, which turns poses into joint angles.

        Every queued move that does not pass its own `ik_solver` uses this solver to
        work out the joint angles for its poses: `move_pose()`, the straight lines,
        arcs, circles and splines, and their relative forms. Joint moves do not use
        it, and neither does `is_reachable()`.

        - `"pinocchio"` (also `"pin"`): damped least squares. Deterministic and
          fast.
        - `"roboticstoolbox"` (also `"robotics_toolbox"`, `"rtb"`): a
          Levenberg-Marquardt solve, with damped least squares as its fallback.

        Case and spaces around the name do not matter. The setting is the robot's:
        it applies to the moves every client sends after it, and moves already sent
        keep their solver. It lasts until it is changed or the robot software
        restarts. The status shows it as `default_ik_solver`.

        Args:
            solver: The solver's name, one of those above.

        Defaults:
            `"pinocchio"`, set each time the robot software starts.

        Returns:
            `{"ik_solver": name}`: the solver now in force, by its main name, for
            example `"roboticstoolbox"` for `"rtb"`.

        Raises:
            CommandRejected: The robot refused it: the name is not a known solver,
                or another client holds control.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: No solver was given (only a raw client can send that).
            BAD_IK_SOLVER: `solver` is not one of the names above.

        See also:
            `set_profile()`, `move_pose()`, `move_linear()`

        Examples:
            >>> robot.set_ik_solver("rtb")
            {'ik_solver': 'roboticstoolbox'}
            >>> x, y, z, rx, ry, rz = robot.get_pose()
            >>> robot.move_linear(x, y, z - 0.02, rx, ry, rz)
            >>> robot.wait_idle()
            >>> robot.set_ik_solver("pinocchio")
        """
        return self._immediate("set_ik_solver", [str(solver).strip().lower()])

    set_default_ik_solver = set_ik_solver      # its other name (2026-10-23)

    def set_completion_policy(self, policy: str) -> Any:
        """Set when the robot counts a queued move as finished.

        The policy decides when each move is reported finished, which is when
        `wait_idle()` returns, and when the next move starts:

        - `"commanded"`: once the robot has sent the move's last setpoint. The next
          move starts at once, while the arm may still be on its way.
        - `"settled"`: the robot holds the move's target until every joint is within
          0.01 rad (about 0.6 deg) of it, then reports the move finished and starts
          the next one. After 2 s it reports the move finished anyway.
        - `"strict"`: as `"settled"`, but after 2 s the move fails with
          `SETTLE_TIMEOUT` (`wait_idle()` raises `MotionFailed`). The moves after it
          still run.

        A move blended into the next one never waits. Case and spaces around the
        name do not matter.

        Unlike the other defaults, a move cannot carry its own policy: this one
        applies at once to every move, also those already sent, from every client.
        It lasts until it is changed or the robot software restarts. The status
        shows it as `exec_completion_policy`.

        Args:
            policy: `"commanded"`, `"settled"` or `"strict"`.

        Defaults:
            `"settled"`, set each time the robot software starts.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: The robot refused it: the name is not a policy, or another
                client holds control. Or RTI refused it or did not answer within 1 s
                (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: No policy was given (only a raw client can send that).
            BAD_POLICY: `policy` is not `"commanded"`, `"settled"` or `"strict"`.
            RTI_REFUSED: RTI could not use the policy. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            `completion_policy` given as a per-move setting is refused with
            `BAD_OPTIONS`.

        See also:
            `wait_idle()`, `set_blending()`, `move_joints()`

        Examples:
            >>> robot.set_completion_policy("strict")
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
        """
        return self._immediate("set_completion_policy", [str(policy).strip().lower()])

    # ------------------------------------------------------------------ #
    # Jog + live config (immediate; migrated from the GUI's old direct-to-RTI
    # push — see TODO/gui_executor_migration_plan.md). Jog forwards to RTI's
    # persistent JogHandler and returns on the executor ack, not on physical
    # completion, so a hold-to-jog caller drives its own cadence.
    # ------------------------------------------------------------------ #

    def enter_jog_mode(self, wait: bool | None = None) -> None:
        """Switch to JOG mode, where `jog()` moves one joint at a time.

        In JOG mode a joint moves while `jog()` asks for it, and `jog_stop()`
        stops it. Nothing moves on entry. Calling it in JOG mode changes nothing.

        Args:
            wait: Wait until the robot reports JOG mode, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot refused it: it is not in IDLE or JOG mode, not
                enabled or not homed, or another client holds control. Or RTI refused it
                or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report JOG mode within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            MODE_REJECTED: RTI refused JOG: an error is latched, or the arm is not
                enabled or not homed. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `jog()`, `jog_stop()`, `enter_idle_mode()`

        Examples:
            >>> import time
            >>> robot.enter_jog_mode(wait=True)
            >>> robot.jog(5, 1, 0.05)
            >>> time.sleep(0.5)
            >>> robot.jog_stop()
        """
        self._state_change("enter_jog_mode", wait, lambda s: s.get("mode") == "JOG",
                           what="wait_mode(JOG)")

    def enter_idle_mode(self, wait: bool | None = None) -> None:
        """Switch to IDLE mode.

        From EXEC mode the arm first brakes along its planned path to rest, at the
        joints' acceleration limits (at most about 0.15 s from full speed), then
        the robot switches. A program still queued is cancelled: its commands fail
        with `ABORTED`, and the robot refuses new moves with `MOTION_ABORTED` until
        `clear_errors()`. To cancel a program and stay in EXEC mode, use `stop()`.

        From JOG mode or a real-time stream the robot switches at once, without
        braking. Homing in progress is abandoned, and the arm is then not homed.
        It also ends motors off and
        firmware flashing, like their own `release_safety_stop()` and
        `exit_flashing_mode()`. In the error state it has no effect: the robot
        returns to it while an error is latched; call `clear_errors()`.

        In IDLE mode an enabled, homed arm with gravity compensation on is in
        freedrive: it holds itself up and can be moved by hand. Otherwise every
        joint holds still.

        Args:
            wait: Wait until the robot reports IDLE mode, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: Another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report IDLE mode within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            From JOG mode or a real-time stream at speed the arm does not brake: in
            freedrive it coasts until friction stops it. Stop the motion first,
            then switch.

        See also:
            `stop()`, `enter_exec_mode()`, `set_gravity_comp()`

        Examples:
            >>> robot.enter_idle_mode(wait=True)
        """
        self._state_change("enter_idle_mode", wait, lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")

    # ------------------------------------------------------------------ #
    # Firmware flashing — see docs/firmware_flashing.md.
    # FLASHING makes RTI go silent on the CAN bus so a separate flasher process
    # can own it, while the loop keeps running (E-stop, IO and telemetry stay
    # live). It does NOT flash anything itself; it grants the bus.
    # ------------------------------------------------------------------ #
    def safety_stop(self, wait: bool | None = None) -> None:
        """Switch the motors off: the arm goes limp and falls.

        Every joint's motor gets no position, no speed and zero current, so
        nothing holds the arm up. The robot accepts it in every mode and state,
        from every client, also with an error latched, and never refuses it. An
        error that latches while the motors are off does not switch them on
        again. The robot stays in this mode until `release_safety_stop()`.

        Unlike `disable()`, which holds every joint still, this takes all torque
        off the arm: to free something trapped by hand, or to hear whether a noise
        comes from a drive's own firmware. From EXEC mode it switches at once,
        without braking, and a queued program is cancelled as with `disable()`.
        The gripper's jaws stop where they are.

        Args:
            wait: Wait until the robot reports the motors off, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: RTI refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report the motors off within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            MODE_REJECTED: RTI refused SAFETY_STOP from the mode it is in. The message
                says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            The arm falls under its own weight. Support it, or park it, first.
            `stop()` and `disable()` from any client switch the motors on again.

        See also:
            `release_safety_stop()`, `disable()`, `stop()`

        Examples:
            >>> robot.safety_stop(wait=True)
            >>> robot.release_safety_stop(wait=True)
        """
        self._state_change("safety_stop", wait,
                           lambda s: s.get("mode") == "SAFETY_STOP",
                           what="wait_mode(SAFETY_STOP)")

    def release_safety_stop(self, wait: bool | None = None) -> None:
        """Switch the motors back on after a safety stop.

        The robot returns to IDLE mode. An enabled, homed arm with gravity
        compensation on is then in freedrive; otherwise every joint holds still
        where it is. With an error still latched, the robot goes to the error
        state instead, which holds every joint still.

        Args:
            wait: Wait until the robot reports IDLE mode, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot refused it: the motors are not off, or another
                client holds control. Or RTI refused it or did not answer within 1 s
                (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report IDLE mode within 10 s, for example with an error still
                latched.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `safety_stop()`, `clear_errors()`

        Examples:
            >>> robot.safety_stop(wait=True)
            >>> robot.release_safety_stop(wait=True)
        """
        self._state_change("release_safety_stop", wait,
                           lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")

    def enter_flashing_mode(self, parked_confirmed: bool = False, force: bool = False, wait: bool | None = None) -> None:
        """Hand the CAN bus to a firmware flasher; the motors let go.

        The robot stops sending on the CAN bus, so a separate flasher program can
        use it; everything else keeps running: the E-stop, the I/O, the status.
        About 0.1 s after the robot falls silent, each drive's watchdog lets go of
        its motor: the joints brake and sink slowly, the gripper coasts. So the
        arm must be parked or supported, which nothing on the robot can check:
        the call needs your claim, and the robot records which one you made.

        It works while disabled, not homed or with an error latched: that is how a
        drive left in its bootloader by a failed flash is recovered. End it with
        `exit_flashing_mode()`.

        Args:
            parked_confirmed: You parked the arm, at the robot's park pose, and
                confirm it.
            force: The arm is not parked but supported by other means, for example
                because a drive cannot move it.
            wait: Wait until the robot reports flashing mode, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot refused it: it is not in IDLE mode or the error
                state, or another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report flashing mode within 10 s.
            RobotConnectionError: The client is not connected.
            ValueError: Neither `parked_confirmed` nor `force` is true. Nothing is
                sent.

        Errors:
            MODE_REJECTED: RTI refused FLASHING: nobody claimed the arm can go limp
                safely, or the mode it is in does not allow it. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            The motors stop holding the arm. Park or support it first.

        Note:
            `stop()` and `disable()` from any client end flashing mode and resume
            the robot's CAN traffic, which can make a flash in progress fail.

        See also:
            `exit_flashing_mode()`, `safety_stop()`

        Examples:
            >>> robot.move_joints(0, -90, 180, 0, 0, 180)  # PAR6's park pose
            >>> robot.wait_idle()
            >>> robot.enter_idle_mode(wait=True)
            >>> robot.enter_flashing_mode(parked_confirmed=True, wait=True)
        """
        if not (parked_confirmed or force):
            raise ValueError(
                "enter_flashing_mode() needs an explicit claim about the arm: "
                "pass parked_confirmed=True after parking it, or force=True if "
                "it is physically supported but cannot be parked. Motors go "
                "limp in this mode."
            )
        self._state_change("enter_flashing_mode", wait,
                           lambda s: s.get("mode") == "FLASHING",
                           what="wait_mode(FLASHING)", args=[bool(force)])

    def exit_flashing_mode(self, wait: bool | None = None) -> None:
        """End firmware flashing and take the CAN bus back.

        The robot resumes its CAN traffic and returns to IDLE mode, where the
        motors hold the arm again. It gives every drive the normal time to answer
        again, so their silence during the flash latches no lost-connection
        error; a drive that was flashed is configured again once it answers. When
        the flasher reports that it flashed a drive, the robot marks the arm not
        homed: call `home()` before moving it. With an error still latched, the
        robot goes to the error state instead.

        Args:
            wait: Wait until the robot reports IDLE mode, up to 10 s.
                `None` follows the client's `sync` setting.

        Raises:
            CommandRejected: The robot refused it: it is not in flashing mode, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s, or, with `wait`, did
                not report IDLE mode within 10 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `enter_flashing_mode()`, `home()`

        Examples:
            >>> robot.exit_flashing_mode(wait=True)
            >>> robot.enable(wait=True)
            >>> robot.home(wait=True)
        """
        self._state_change("exit_flashing_mode", wait,
                           lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")

    def jog(self, joint: int, direction: int, speed: float) -> Any:
        """Start jogging one joint: it moves until `jog_stop()` or a limit.

        The joint speeds up to `speed` % of its speed limit and keeps moving until
        `jog_stop()`. It speeds up and brakes at the jog's acceleration and jerk
        (`set_jog_accel()`, `set_jog_profile()`, `set_jog_jerk_factor()`). Near the
        end of its range it brakes by itself and comes to rest exactly at its joint
        limit, at any speed; with self-collision checking on, it also comes to rest
        at the jog margin before two parts of the arm would touch. Asking for the
        same direction again there moves nothing.

        Calling `jog()` again for the same joint and direction changes the speed
        smoothly, as a joystick does. For another joint or direction, the moving
        joint first brakes to rest, then the new jog starts: one joint moves at a
        time. The call returns once the robot has accepted the jog; it does not
        wait for the motion.

        Args:
            joint: The joint to move, counted from 0: `0` is J1 and `5` is J6 on a
                six-joint arm, a whole number. A joint that does not exist is
                refused.
            direction: `1` for the joint's positive direction, `-1` for its negative
                one; a larger whole number counts by its sign. `0` stops the jog as
                `jog_stop()` does.
            speed: The speed in percent of the joint's speed limit, from 0 to 100:
                `10` is 10 %. 0 moves nothing.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: The robot refused it, for example outside JOG mode, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `joint`, `direction` or `speed` cannot be read as a number,
                or `joint` or `direction` is not a whole number. Nothing is sent.
            TypeError: An argument is `None` or another type that is no number.
                Nothing is sent.

        Errors:
            BAD_ARGS: The joint does not exist, the direction is not a whole
                number, the speed is not a number from 0 to 100, or the command did
                not carry exactly these three values.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            A jog has no time limit. The joint keeps moving until `jog_stop()`, a
            mode change, an error or the end of your program's connection, which
            stops it as `jog_stop()` does. A program that hangs while connected
            keeps it moving to its joint limit. It does not brake for
            safety zones: entering a stop zone stops the robot with an error. Keep a
            hand near the E-stop.

        Note:
            The braking is planned, so a stop takes time: with the start settings
            about 0.4 s from 20 % speed and 0.9 s from full speed, while the joint
            moves on at about half its speed.

        See also:
            `jog_stop()`, `enter_jog_mode()`, `set_jog_accel()`, `set_jog_profile()`

        Examples:
            >>> import time
            >>> robot.enter_jog_mode(wait=True)
            >>> robot.jog(0, 1, 5)
            >>> time.sleep(0.5)
            >>> robot.jog_stop()
        """
        return self._immediate("jog", [_whole(joint, "joint"), _whole(direction, "direction"), float(speed)])

    def jog_stop(self) -> Any:
        """Stop jogging: the joint brakes to rest along its path.

        The jogging joint brakes at the jog's acceleration and jerk, the same planned
        braking it uses at a limit, so it stops smoothly but not at once: with the
        start settings in about 0.4 s from 20 % speed and 0.9 s from full speed. The
        call returns before the joint is at rest. A later `jog()` starts a new jog.

        The robot accepts it from every client, in every mode, also while another
        client holds control, so a released jog button always stops the joint.
        Outside JOG mode it changes nothing.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: RTI refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `jog()`, `stop()`, `enter_idle_mode()`

        Examples:
            >>> import time
            >>> robot.jog(0, 1, 0.05)
            >>> time.sleep(0.5)
            >>> robot.jog_stop()
        """
        return self._immediate("jog_stop")

    def set_jog_accel(self, accel_time: float) -> Any:
        """Set the jog's ramp time, from rest to a joint's full speed.

        A jogging joint speeds up and brakes at its speed limit divided by this
        time, but never faster than its acceleration limit: a time shorter than that
        limit allows changes nothing more. With the `"scurve"` jog profile each ramp
        also rounds its start and end, which adds up to 1 / jerk factor seconds
        (`set_jog_jerk_factor()`). The braking at a limit and after `jog_stop()` is
        planned with the same acceleration.

        The setting is the robot's: it applies to every client's jogs and the
        dashboard's, and mode changes keep it. A jog in progress keeps its ramp
        until the joint comes to rest. It lasts until it is changed or the robot
        software restarts.

        Args:
            accel_time: Seconds from rest to the joint's full speed. It must be more
                than 0; a time under 0.05 s counts as 0.05 s.

        Defaults:
            From `config/system.xml` `<jog_defaults>`, read at start: 0.55 s.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: The robot refused it: `accel_time` is not more than 0 or
                not finite, or another client holds control. Or RTI refused it or did
                not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `accel_time` cannot be read as a number. Nothing is sent.
            TypeError: `accel_time` is `None` or another type that is no number.
                Nothing is sent.

        Errors:
            BAD_ARGS: `accel_time` is not a finite number above 0.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `set_jog_profile()`, `set_jog_jerk_factor()`, `jog()`

        Examples:
            >>> robot.set_jog_accel(1.0)
        """
        return self._immediate("set_jog_accel", [float(accel_time)])

    def set_jog_profile(self, profile: str) -> Any:
        """Set the jog's profile: with its jerk limited, or not.

        - `"scurve"`: limits the jerk too. The acceleration builds up and falls
          away gradually, so starts and stops are smooth. The jerk is the
          acceleration times the factor of `set_jog_jerk_factor()`.
        - `"trapezoidal"`: the acceleration switches on and off at once. Shorter
          ramps, harder starts and stops.

        Letter case does not matter, and `"s_curve"`, `"s-curve"`, `"trapezoid"`
        and `"trap"` are taken too. The setting is the robot's: it applies to every
        client's jogs and the dashboard's, and mode changes keep it. A jog in
        progress keeps its profile until the joint comes to rest. It lasts until it
        is changed or the robot software restarts.

        Args:
            profile: `"scurve"` or `"trapezoidal"`, or one of the other spellings
                above.

        Defaults:
            From `config/system.xml` `<jog_defaults>`, read at start: `"scurve"`.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: The robot refused it: the name is none of the above, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: `profile` is none of the names above.
            RTI_REFUSED: RTI could not use the name. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Note:
            The queued moves' profiles are another setting, `set_profile()`.

        See also:
            `set_jog_jerk_factor()`, `set_jog_accel()`, `set_profile()`

        Examples:
            >>> robot.set_jog_profile("trapezoidal")
            >>> robot.set_jog_profile("scurve")
        """
        return self._immediate("set_jog_profile", [str(profile).strip()])

    def set_jog_jerk_factor(self, factor: float) -> Any:
        """Set the jog's jerk, as a multiple of its acceleration.

        With the `"scurve"` jog profile the jerk is the jog's acceleration times
        this factor, per second: the acceleration takes up to 1 / factor seconds to
        build up, and again to fall away. A higher factor gives shorter, harder
        ramps; a lower one smoother, longer ones. With `"trapezoidal"` it has no
        effect.

        The setting is the robot's: it applies to every client's jogs and the
        dashboard's, and mode changes keep it. A jog in progress keeps its jerk
        until the joint comes to rest. It lasts until it is changed or the robot
        software restarts.

        Args:
            factor: The jerk as a multiple of the acceleration, per second. It must
                be more than 0; a factor under 0.5 counts as 0.5.

        Defaults:
            From `config/system.xml` `<jog_defaults>`, read at start: 3.0.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: The robot refused it: `factor` is not more than 0 or not
                finite, or another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `factor` cannot be read as a number. Nothing is sent.
            TypeError: `factor` is `None` or another type that is no number.
                Nothing is sent.

        Errors:
            BAD_ARGS: `factor` is not a finite number above 0.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `set_jog_profile()`, `set_jog_accel()`, `jog()`

        Examples:
            >>> robot.set_jog_jerk_factor(6.0)
        """
        return self._immediate("set_jog_jerk_factor", [float(factor)])

    def set_jog_control_mode(self, mode: str) -> Any:
        """Set how the drives follow a jog: PID or PD control.

        - `"pid"`: each drive runs its own position and speed loops, with an
          integral term, toward the jog's position and speed.
        - `"pd"`: each drive turns its position and speed errors into torque with
          its own two gains, without an integral term.

        The gains are the drives' own (`set_pid_gains()`). It applies in JOG mode
        only, from the next control tick, also to a jog in progress. Letter case
        does not matter. The setting is the robot's: it applies to every client's
        jogs and the dashboard's, and mode changes keep it. It lasts until it is
        changed or the robot software restarts.

        Args:
            mode: `"pid"` or `"pd"`.

        Defaults:
            From `config/system.xml` `<jog_defaults>`, read at start: `"pid"`.

        Returns:
            `None`: the robot sends no value back.

        Raises:
            CommandRejected: The robot refused it: the name is neither `"pid"` nor
                `"pd"`, or another client holds control. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: `mode` is neither `"pid"` nor `"pd"`.
            RTI_REFUSED: RTI could not use the name. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `set_pid_gains()`, `set_jog_profile()`, `jog()`

        Examples:
            >>> robot.set_jog_control_mode("pd")
            >>> robot.set_jog_control_mode("pid")
        """
        return self._immediate("set_jog_control_mode", [str(mode).strip()])


    def set_pid_gains(self, joint: int, **gains: float) -> Any:
        """Set the control gains and limits of one joint's drive.

        The robot stores the values given and at once sends that drive its whole
        configuration again: its watchdog, its current, velocity and voltage
        limits and every gain. Values not named keep theirs. Joints count from 0
        here, unlike in `free_joint()` and `joint_test()`. The values last until
        the robot software restarts; nothing is saved to the robot's
        configuration.

        Args:
            joint: The drive: 0 for J1 up to the number of joints minus 1 (5 for
                J6 on PAR6). The gripper's drive cannot be set this way.
            **gains: The values to set, in the drive's own units as in the robot's
                configuration: `KPP` (the position loop's proportional gain), `KPV`
                and `KIV` (the velocity loop's proportional and integral gains),
                `KPIQ` and `KIIQ` (the current loop's), `KP` and `KD` (the
                stiffness and damping of the drive's impedance controller), `Ilim`
                (the current limit, in milliamperes), `vel_lim` (the motor's
                velocity limit, in encoder ticks per second) and `vlim` (the voltage
                limit, in millivolts: 0 uses the supply voltage, and any other value
                below 6000 mV is raised to 6000 mV).

        Defaults:
            From the robot's configuration file (`robots/<robot>.xml`), read each
            time the robot software starts. PAR6's J1, for example: KPP 88, KPV
            0.078, KIV 0.00012, KPIQ
            1.5, KIIQ 0.1, KP 0.12, KD 0.002, Ilim 2400 mA, vel_lim 88000 ticks/s,
            vlim 6000 mV.

        Returns:
            `None`: the robot sends no data back.

        Raises:
            CommandRejected: The robot refused it: a value is wrong, homing or a test
                runs (see Errors), or another client holds control. Or RTI refused it or
                did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `joint` is not a whole number, or a gain is not a number.
                Nothing is sent.
            TypeError: `joint` or a gain is `None`. Nothing is sent.

        Errors:
            BAD_ARGS: `joint` is out of range, a gain's name is unknown, a value is
                not a finite number, a gain is negative, `Ilim` or `vel_lim` is
                not above 0 or is above that joint's limit in the robot's
                configuration, or no gain is given.
            REJECTED: The robot is homing, or the Hall sensor check, a freed joint
                or a joint test runs: each relies on the joint's current limit,
                which this command sets.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            A gain can make the joint oscillate or run away, and a low `Ilim` lets
            it sag or fall. Change gains with the arm at rest and a hand on the
            E-stop.

        See also:
            `joint_test()`, `set_jog_control_mode()`

        Examples:
            >>> robot.set_pid_gains(0, KPP=88, KPV=0.078)
        """
        payload = {"joint": _whole(joint, "joint")}
        payload.update({k: float(v) for k, v in gains.items()})
        return self._immediate("set_pid_gains", payload)

    def calibrate_gripper(self) -> Any:
        """Calibrate the gripper: find both ends of its jaws' travel.

        The gripper opens its jaws fully, then closes them fully, pressing against
        each end, and records the two ends as 0 and 255 of its position scale. It
        takes a few seconds: about 4.5 s from closed jaws. Until a gripper has
        calibrated, it moves to no target. Homing calibrates the gripper as one of
        its steps, so call this only to calibrate again without homing, for
        example after the gripper's drive lost power. No status field reports the
        calibration.

        The call returns at once. The robot then sends the gripper only status
        requests until the gripper reports itself calibrated, but not before the
        gripper XML's `<homing_calibration_wait_s>` has passed, and for at most
        30 s. The next gripper command ends a calibration still running, so wait
        that long before it. The calibration drops the gripper's target: the jaws
        stay where it leaves them until the next gripper command. While homing
        drives the gripper, and during firmware flashing, the calibration waits
        until they end.

        Returns:
            `None`. The robot answers before the calibration starts.

        Raises:
            CommandRejected: The robot refused it: the robot has no CAN gripper, or
                another client holds control. Or RTI refused it or did not answer within
                1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            NO_GRIPPER: The robot has no CAN gripper.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            The jaws open and close fully by themselves and let go of a part they
            hold. Keep hands away from the jaws.

        See also:
            `home()`, `set_gripper()`, `set_gripper_ctrl_mode()`, `open_gripper()`

        Examples:
            >>> import time
            >>> robot.calibrate_gripper()
            >>> time.sleep(6)
            >>> robot.open_gripper()
            >>> robot.wait_gripper_move_complete()
        """
        return self._immediate("calibrate_gripper")

    gripper_calibrate = calibrate_gripper      # its other name (2026-10-23)

    def set_gripper_ctrl_mode(self, mode: int) -> Any:
        """Switch the gripper between motor mode and firmware mode.

        In firmware mode the gripper's own firmware moves the jaws to the targets
        that `open_gripper()`, `close_gripper()`, `move_gripper()` and
        `set_gripper()` give it, and reports their position and whether something
        stopped them. In motor mode the robot gives the gripper's motor zero
        current: the jaws neither move nor hold, `set_gripper()` targets do not
        reach the gripper, and the queued gripper commands are refused with
        `GRIPPER_MOTOR_MODE`.

        Switching to motor mode also drops the gripper's target, so the jaws stay
        still when firmware mode returns. Homing drives the gripper itself in
        either mode. `latest_status()` reports the mode in `gripper`.

        Args:
            mode: `1` for motor mode or `2` for firmware mode.

        Defaults:
            From the robot's homing file, read each time the robot software
            starts: firmware mode when the first homing step that homes the
            gripper names firmware mode, as `config/PAR6_homing.xml` does, motor
            mode otherwise. It lasts until changed or until the robot software
            restarts.

        Returns:
            `None`. The robot answers before the gripper changes mode.

        Raises:
            CommandRejected: The robot refused it: `mode` is not 1 or 2, the robot has
                no CAN gripper, or another client holds control. Or RTI refused it or
                did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `mode` cannot be read as a whole number, for example the
                text "firmware". Nothing is sent.

        Errors:
            NO_GRIPPER: The robot has no CAN gripper.
            BAD_ARGS: `mode` is not 1 or 2.
            RTI_REFUSED: RTI could not read the mode. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            Motor mode takes the current off the gripper's motor: the jaws stop
            holding, and a part between them can slip out.

        See also:
            `set_gripper()`, `move_gripper()`, `calibrate_gripper()`

        Examples:
            >>> robot.set_gripper_ctrl_mode(2)
        """
        return self._immediate("set_gripper_ctrl_mode", [int(mode)])

    def set_gripper(self, position: int, speed: int = 100, current: int = 200, action: int = 1) -> Any:
        """Give the gripper a target at once, outside the queue.

        The robot sends the target to the gripper on its next cycle, whatever the
        arm is doing, and answers at once: the call does not wait for the jaws
        and reports nothing about them. For a program, use `move_gripper()`,
        which runs in order with the moves, waits for the jaws and reports the
        outcome.

        With `action` 1 the jaws move to `position` and stay there, pressing with
        up to `current` on whatever stops them. With `action` 0 the gripper is
        released: it has no target, and depending on its drive the jaws hold
        still or go limp. The target lasts until the next gripper command;
        `stop()`, `disable()` and an error replace it with where the jaws are.

        It takes effect only in firmware mode and on a calibrated gripper; in
        motor mode the robot keeps it and sends it when firmware mode returns. It
        ends a calibration in progress, and replaces the target of a queued
        gripper command still under way, which then fails with
        `GRIPPER_SETTLE_TIMEOUT` unless the jaws pass within 5 of its own target.

        Args:
            position: The target, a whole number from 0 (fully open) to 255
                (fully closed) on the gripper's scale; not millimetres.
            speed: How fast the jaws move, a whole number from 0 (slowest) to 255
                (fastest) on the gripper's scale.
            current: The gripper motor's current limit while the jaws move and
                hold, in mA: a whole number from 1 to the gripper XML's `<Ilim>`
                (1700 mA for the shipped grippers).
            action: `1` to move to `position` and stay there, or `0` to release
                the gripper.

        Defaults:
            No target, the gripper released, each time the robot software starts.
            Homing ends with the gripper released too, and so does
            `set_gripper_ctrl_mode(1)`.

        Returns:
            `None`. The robot answers before the target reaches the gripper.

        Raises:
            CommandRejected: The robot refused it: a value is outside its range, the
                robot has no CAN gripper, or another client holds control. Or RTI
                refused it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: A value cannot be read as a whole number, for example the
                text "open". Nothing is sent.

        Errors:
            NO_GRIPPER: The robot has no CAN gripper.
            BAD_ARGS: A value is outside its range above, or the robot did not get
                exactly four values (this method always sends four).
            RTI_REFUSED: RTI could not read the values. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            The jaws move at once, also while the robot is disabled, and keep
            pressing until the next gripper command, `stop()`, `disable()` or an
            error, which stop them where they are in firmware mode. Keep hands
            away from the jaws.

        See also:
            `move_gripper()`, `set_gripper_ctrl_mode()`, `calibrate_gripper()`

        Examples:
            >>> import time
            >>> robot.set_gripper(128, 100, 200, 1)
            >>> time.sleep(1)
            >>> robot.set_gripper(128, 100, 200, 0)
        """
        return self._immediate("set_gripper",
                               [int(position), int(speed), int(current), int(action)])

    # ------------------------------------------------------------------ #
    # Motion (queued)
    # ------------------------------------------------------------------ #

    def move_joints(self, *angles_deg: float, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """Move every joint to a target angle, in joint space.

        All joints start and arrive together, so the tool's path is not a straight
        line. Use `move_linear()` for a straight line.

        The robot needs no inverse kinematics along the way, so a joint move also
        passes through singular poses, where a straight-line move can fail.

        Args:
            *angles_deg: The target angle of each joint, J1 first, in degrees. Each
                must lie within its joint's limits, or the move is refused before
                anything moves. A single list or tuple of the angles works too.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. `ik_solver`, `singularity_strategy` and `samples`
                have no effect on a joint move. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.

        Errors:
            JOINT_LIMIT: A target angle is outside its joint's limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.

        See also:
            `move_pose()`, `set_joint_speed()`

        Examples:
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.move_joints([90, -105, 165, 0, -30, 180], speed=0.2)
            >>> robot.wait_idle()
        """
        if len(angles_deg) == 1 and isinstance(angles_deg[0], (list, tuple)):
            angles_deg = tuple(angles_deg[0])
        return self._queued("move_joints", [float(a) for a in angles_deg], timeout, options)

    def move_pose(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool to a pose, with the joints moving in joint space.

        The robot works out the joint angles for the pose, starting from where the
        arm is, then moves there like `move_joints()`: the tool's path is not a
        straight line. Use `move_linear()` for a straight line, and
        `is_reachable()` to check a pose first.

        Args:
            x: Target position along the base X axis, in metres.
            y: Target position along the base Y axis, in metres.
            z: Target position along the base Z axis, in metres.
            rx: Turn about the base X axis, in radians, or degrees with
                `angle_unit="deg"`. The orientation is R = Rx(rx) Ry(ry) Rz(rz).
            ry: Turn about the tool's new Y axis, in the same unit.
            rz: Turn about the tool's own Z axis, in the same unit.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `angle_unit` is neither `"rad"` nor `"deg"`.

        Errors:
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The solver returned an invalid joint solution.
            IK_MARGIN_VIOLATION: The joint solution is outside the joint limits.
            IK_POSE_ERROR: The joint solution misses the target pose by more than
                0.2 mm or 0.5 degrees.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        See also:
            `move_joints()`, `move_linear()`, `is_reachable()`

        Examples:
            >>> x, y, z, rx, ry, rz = robot.get_pose()
            >>> robot.move_pose(x, y, z + 0.05, rx, ry, rz)
            >>> robot.wait_idle()
        """
        return self._queued("move_pose", _pose_in((x, y, z, rx, ry, rz), angle_unit), timeout, options)

    def move_linear(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool in a straight line to a pose.

        The tool centre point travels on a straight line in the base frame while
        its orientation turns smoothly to the target.

        Args:
            x: Target position along the base X axis, in metres.
            y: Target position along the base Y axis, in metres.
            z: Target position along the base Z axis, in metres.
            rx: Turn about the base X axis, in radians, or degrees with
                `angle_unit="deg"`. The orientation is R = Rx(rx) Ry(ry) Rz(rz).
            ry: Turn about the tool's new Y axis, in the same unit.
            rz: Turn about the tool's own Z axis, in the same unit.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `angle_unit` is neither `"rad"` nor `"deg"`.

        Errors:
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the straight line,
                usually near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Moves along a Cartesian path run at no more than 20 % of the speed
            limit; a higher `speed` has no further effect.

        See also:
            `move_linear_rel_world()`, `move_linear_rel_tool()`, `move_pose()`

        Examples:
            >>> x, y, z, rx, ry, rz = robot.get_pose()
            >>> robot.move_linear(x, y, z - 0.05, rx, ry, rz)
            >>> robot.wait_idle()
        """
        return self._queued("move_linear", _pose_in((x, y, z, rx, ry, rz), angle_unit), timeout, options)

    def move_linear_rel_world(self, dx: float, dy: float, dz: float, drx: float = 0.0, dry: float = 0.0, drz: float = 0.0, timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool in a straight line by an offset along the base axes.

        The offset is measured from where the tool is when the move starts, along
        axes parallel to the base frame. A turn is
        about those axes, through the tool centre point.

        Args:
            dx: Offset along the base X axis, in metres.
            dy: Offset along the base Y axis, in metres.
            dz: Offset along the base Z axis, in metres.
            drx: Turn about the base X axis, in radians, or degrees with
                `angle_unit="deg"`.
            dry: Turn about the base Y axis, in the same unit.
            drz: Turn about the base Z axis, in the same unit.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `angle_unit` is neither `"rad"` nor `"deg"`.

        Errors:
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the straight line,
                usually near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Moves along a Cartesian path run at no more than 20 % of the speed
            limit; a higher `speed` has no further effect.

        See also:
            `move_linear_rel_tool()`, `move_linear()`

        Examples:
            >>> robot.move_linear_rel_world(0.05, 0, 0)
            >>> robot.move_linear_rel_world(0, 0, 0, drz=15, angle_unit="deg")
            >>> robot.wait_idle()
        """
        return self._queued(
            "move_linear_rel_world",
            _pose_in((dx, dy, dz, drx, dry, drz), angle_unit), timeout, options)

    def move_linear_rel_tool(self, dx: float, dy: float, dz: float, drx: float = 0.0, dry: float = 0.0, drz: float = 0.0, timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool in a straight line by an offset along its own axes.

        The offset is measured along the tool's axes as they are when the move
        starts. A turn is about those axes.

        Args:
            dx: Offset along the tool's X axis, in metres.
            dy: Offset along the tool's Y axis, in metres.
            dz: Offset along the tool's Z axis, in metres.
            drx: Turn about the tool's X axis, in radians, or degrees with
                `angle_unit="deg"`.
            dry: Turn about the tool's Y axis, in the same unit.
            drz: Turn about the tool's Z axis, in the same unit.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `angle_unit` is neither `"rad"` nor `"deg"`.

        Errors:
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the straight line,
                usually near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Moves along a Cartesian path run at no more than 20 % of the speed
            limit; a higher `speed` has no further effect.

        See also:
            `move_linear_rel_world()`, `move_linear()`

        Examples:
            >>> robot.move_linear_rel_tool(0, 0, 0.02)
            >>> robot.wait_idle()
        """
        return self._queued(
            "move_linear_rel_tool",
            _pose_in((dx, dy, dz, drx, dry, drz), angle_unit), timeout, options)


    @staticmethod
    def _curve_pose_values(pose, label, angle_unit="rad"):
        values = [float(v) for v in pose]
        if len(values) != 6:
            raise ValueError(f"{label} needs [x, y, z, rx, ry, rz]")
        return _pose_in(values, angle_unit)

    def move_arc(self, via: Sequence[float], end: Sequence[float], timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool along a circular arc through a via pose to an end pose.

        The arc runs from where the tool is, through the via pose's position, to the
        end pose, in the base frame. The orientation turns smoothly from the start
        to the end pose's orientation; the via pose's orientation is not used.

        Args:
            via: A pose `[x, y, z, rx, ry, rz]` the arc passes through: metres,
                and radians or degrees as `angle_unit` says.
            end: The pose the arc ends at, in the same form.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `via` or `end` does not have 6 values, or `angle_unit` is
                neither `"rad"` nor `"deg"`.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: The start, `via` and `end` do not define an
                arc: they lie on one line, or `via` or `end` is the start.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_arc_rel_world()`, `move_arc_rel_tool()`, `move_circle()`

        Examples:
            >>> x, y, z, rx, ry, rz = robot.get_pose()
            >>> robot.move_arc([x + 0.05, y + 0.05, z, rx, ry, rz],
            ...                [x + 0.10, y, z, rx, ry, rz])
            >>> robot.wait_idle()
        """
        args = self._curve_pose_values(via, "move_arc via", angle_unit)
        args.extend(self._curve_pose_values(end, "move_arc end", angle_unit))
        return self._queued("move_arc", args, timeout, options)

    def move_arc_rel_world(self, via: Sequence[float], end: Sequence[float], timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool along a circular arc given by offsets along the base axes.

        Like `move_arc()`, with `via` and `end` given as offsets from where the tool
        is when the move starts, along axes parallel to the base frame.

        Args:
            via: The offset `[dx, dy, dz, drx, dry, drz]` the arc passes through:
                metres, and radians or degrees as `angle_unit` says.
            end: The offset the arc ends at, in the same form.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `via` or `end` does not have 6 values, or `angle_unit` is
                neither `"rad"` nor `"deg"`.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: The start, `via` and `end` do not define an
                arc: they lie on one line, or `via` or `end` is the start.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_arc_rel_tool()`, `move_arc()`

        Examples:
            >>> robot.move_arc_rel_world([0.05, 0.05, 0, 0, 0, 0], [0.1, 0, 0, 0, 0, 0])
            >>> robot.wait_idle()
        """
        args = self._curve_pose_values(via, "move_arc_rel_world via", angle_unit)
        args.extend(self._curve_pose_values(end, "move_arc_rel_world end", angle_unit))
        return self._queued("move_arc_rel_world", args, timeout, options)

    def move_arc_rel_tool(self, via: Sequence[float], end: Sequence[float], timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool along a circular arc given by offsets along its own axes.

        Like `move_arc()`, with `via` and `end` given as offsets along the tool's
        axes as they are when the move starts.

        Args:
            via: The offset `[dx, dy, dz, drx, dry, drz]` the arc passes through:
                metres, and radians or degrees as `angle_unit` says.
            end: The offset the arc ends at, in the same form.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `via` or `end` does not have 6 values, or `angle_unit` is
                neither `"rad"` nor `"deg"`.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: The start, `via` and `end` do not define an
                arc: they lie on one line, or `via` or `end` is the start.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_arc_rel_world()`, `move_arc()`

        Examples:
            >>> robot.move_arc_rel_tool([0.03, 0.03, 0, 0, 0, 0], [0.06, 0, 0, 0, 0, 0])
            >>> robot.wait_idle()
        """
        args = self._curve_pose_values(via, "move_arc_rel_tool via", angle_unit)
        args.extend(self._curve_pose_values(end, "move_arc_rel_tool end", angle_unit))
        return self._queued("move_arc_rel_tool", args, timeout, options)

    def move_circle(self, center: Sequence[float], normal: Sequence[float], turns: float = 1.0, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """Move the tool around a circle about a centre point.

        The circle starts where the tool is and turns around `center` in the plane
        at right angles to `normal`. That plane passes through the tool's position,
        so `center` must lie in it, within 0.2 mm. With a whole number of turns
        the tool ends where it started. It keeps its orientation the whole way.

        Args:
            center: The circle's centre `[x, y, z]` in the base frame, in metres.
            normal: The direction `[x, y, z]` the circle's axis points in; any
                length. The tool goes around it counter-clockwise, seen from the
                tip of `normal`.
            turns: How many times to go around; a fraction goes part of the way, 0
                is refused. A negative number goes the other way.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `center` or `normal` does not have 3 values.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: `normal` is zero, `center` is not in the
                circle's plane (through the start, at right angles to `normal`),
                the start is the centre, or `turns` is 0 or not finite.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_circle_rel_world()`, `move_circle_rel_tool()`, `move_arc()`

        Examples:
            >>> x, y, z, rx, ry, rz = robot.get_pose()
            >>> robot.move_circle([x + 0.03, y, z], [0, 0, 1])
            >>> robot.wait_idle()
        """
        center_values = [float(v) for v in center]
        normal_values = [float(v) for v in normal]
        if len(center_values) != 3 or len(normal_values) != 3:
            raise ValueError("move_circle center and normal each need 3 values")
        return self._queued(
            "move_circle", center_values + normal_values + [float(turns)], timeout, options)

    def move_circle_rel_world(self, center: Sequence[float], normal: Sequence[float], turns: float = 1.0, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """Move the tool around a circle given relative to the base axes.

        Like `move_circle()`, with `center` given as an offset from where the tool
        is when the move starts, and `normal` along axes parallel to the base frame.

        Args:
            center: The centre's offset `[dx, dy, dz]` from the tool, in metres,
                at right angles to `normal`.
            normal: The direction `[x, y, z]` of the circle's axis, along the base
                axes; any length.
            turns: How many times to go around; a fraction goes part of the way, 0
                is refused. A negative number goes the other way.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `center` or `normal` does not have 3 values.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: `normal` is zero, `center` is not in the
                circle's plane (through the start, at right angles to `normal`),
                the start is the centre, or `turns` is 0 or not finite.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_circle_rel_tool()`, `move_circle()`

        Examples:
            >>> robot.move_circle_rel_world([0.03, 0, 0], [0, 0, 1])
            >>> robot.wait_idle()
        """
        center_values = [float(v) for v in center]
        normal_values = [float(v) for v in normal]
        if len(center_values) != 3 or len(normal_values) != 3:
            raise ValueError("move_circle_rel_world center and normal each need 3 values")
        return self._queued(
            "move_circle_rel_world", center_values + normal_values + [float(turns)], timeout, options)

    def move_circle_rel_tool(self, center: Sequence[float], normal: Sequence[float], turns: float = 1.0, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """Move the tool around a circle given relative to its own axes.

        Like `move_circle()`, with `center` and `normal` given along the tool's axes
        as they are when the move starts.

        Args:
            center: The centre's offset `[dx, dy, dz]` from the tool, along the
                tool's axes, in metres, at right angles to `normal`.
            normal: The direction `[x, y, z]` of the circle's axis, along the
                tool's axes; any length.
            turns: How many times to go around; a fraction goes part of the way, 0
                is refused. A negative number goes the other way.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `center` or `normal` does not have 3 values.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: `normal` is zero, `center` is not in the
                circle's plane (through the start, at right angles to `normal`),
                the start is the centre, or `turns` is 0 or not finite.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_circle_rel_world()`, `move_circle()`

        Examples:
            >>> robot.move_circle_rel_tool([0.02, 0, 0], [0, 0, 1])
            >>> robot.wait_idle()
        """
        center_values = [float(v) for v in center]
        normal_values = [float(v) for v in normal]
        if len(center_values) != 3 or len(normal_values) != 3:
            raise ValueError("move_circle_rel_tool center and normal each need 3 values")
        return self._queued(
            "move_circle_rel_tool", center_values + normal_values + [float(turns)], timeout, options)

    def move_spline(self, waypoints: Sequence[Sequence[float]], timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool along a smooth curve through a list of poses.

        The curve starts where the tool is and passes through every pose in order,
        in the base frame. The orientation turns smoothly from pose to pose.

        Args:
            waypoints: Two or more poses `[x, y, z, rx, ry, rz]`: metres, and
                radians or degrees as `angle_unit` says.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: Fewer than two waypoints, a waypoint without 6 values, or
                an `angle_unit` other than `"rad"` or `"deg"`.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: Fewer than two waypoints differ from each other
                and from the start.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_spline_rel_world()`, `move_spline_rel_tool()`, `move_linear()`

        Examples:
            >>> x, y, z, rx, ry, rz = robot.get_pose()
            >>> robot.move_spline([
            ...     [x + 0.03, y + 0.02, z, rx, ry, rz],
            ...     [x + 0.06, y - 0.02, z, rx, ry, rz],
            ...     [x + 0.09, y, z, rx, ry, rz],
            ... ])
            >>> robot.wait_idle()
        """
        poses = [self._curve_pose_values(pose, "move_spline waypoint", angle_unit) for pose in waypoints]
        if len(poses) < 2:
            raise ValueError("move_spline needs at least two waypoint poses")
        return self._queued("move_spline", [v for pose in poses for v in pose], timeout, options)

    def move_spline_rel_world(self, waypoints: Sequence[Sequence[float]], timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool along a smooth curve through offsets along the base axes.

        Like `move_spline()`, with each waypoint given as an offset from where the
        tool is when the move starts, along axes parallel to the base frame. Each
        offset is from that start point, not from the previous waypoint.

        Args:
            waypoints: Two or more offsets `[dx, dy, dz, drx, dry, drz]`: metres,
                and radians or degrees as `angle_unit` says.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: Fewer than two waypoints, a waypoint without 6 values, or
                an `angle_unit` other than `"rad"` or `"deg"`.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: Fewer than two waypoints differ from each other
                and from the start.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_spline_rel_tool()`, `move_spline()`

        Examples:
            >>> robot.move_spline_rel_world([
            ...     [0.03, 0.02, 0, 0, 0, 0],
            ...     [0.06, -0.02, 0, 0, 0, 0],
            ...     [0.09, 0, 0, 0, 0, 0],
            ... ])
            >>> robot.wait_idle()
        """
        poses = [self._curve_pose_values(pose, "move_spline_rel_world waypoint", angle_unit) for pose in waypoints]
        if len(poses) < 2:
            raise ValueError("move_spline_rel_world needs at least two waypoint poses")
        return self._queued("move_spline_rel_world", [v for pose in poses for v in pose], timeout, options)

    def move_spline_rel_tool(self, waypoints: Sequence[Sequence[float]], timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad", **options: Any) -> str:
        """Move the tool along a smooth curve through offsets along its own axes.

        Like `move_spline()`, with each waypoint given as an offset along the tool's
        axes as they are when the move starts. Each offset is from that start
        point, not from the previous waypoint.

        Args:
            waypoints: Two or more offsets `[dx, dy, dz, drx, dry, drz]`: metres,
                and radians or degrees as `angle_unit` says.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
            **options: Settings for this move only, overriding the defaults:
                `speed`, `accel` and `jerk` (percent of the limit, above 0 and at most
                100: 1 means 1 %), `duration` (seconds), `profile`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. A setting left out
                takes its default. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the move;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the move has finished.

        Raises:
            CommandRejected: The robot refused the move, for example outside EXEC
                mode, before homing or with an invalid argument. In sync mode also
                when the move fails after it was accepted.
            RobotTimeout: The robot did not accept the move within 5 s. In sync
                mode also when the move did not finish within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: Fewer than two waypoints, a waypoint without 6 values, or
                an `angle_unit` other than `"rad"` or `"deg"`.

        Errors:
            CURVE_BLEND_UNSUPPORTED: `blend` is above 0; curves do not blend yet.
            BAD_ARGUMENTS: A raw client sent the wrong number of values.
            CURVE_GEOMETRY_INVALID: Fewer than two waypoints differ from each other
                and from the start.
            IK_TARGET_UNREACHABLE: The arm cannot reach the target pose.
            IK_PARTIAL_PATH: The arm cannot reach a pose on the way to the target.
            IK_MARGIN_VIOLATION: A pose on the path needs a joint outside its
                limits.
            IK_BRANCH_JUMP: The arm would have to change its configuration on the
                way, for example flip its wrist.
            IK_POSE_ERROR: The joint solution misses a pose on the path.
            CARTESIAN_PATH_DEVIATION: The tool would stray from the curve, usually
                near a singularity.
            DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
            INVALID_DURATION: `duration` is not more than 0 seconds.
            TOPPRA_UNAVAILABLE: `profile="toppra"`, but that planner is missing.
            TOPPRA_INFEASIBLE: `profile="toppra"` found no timing within the
                limits.
            ACCEL_LIMIT_VIOLATION: The planned motion would exceed an
                acceleration limit.
            JOINT_LIMIT: The path would take a joint outside its limits.
            ZONE_LIMIT: The path would enter a safety zone.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.
            SELF_COLLISION: The path would bring two parts of the arm too close.

        Note:
            Blending does not apply: `blend` must be 0, and the move starts and ends
            at rest.
            A curve that would exceed an acceleration limit is slowed down, to
            25 % of `speed` at most, before it fails; with `duration` set it is
            not slowed.

        See also:
            `move_spline_rel_world()`, `move_spline()`

        Examples:
            >>> robot.move_spline_rel_tool([
            ...     [0, 0.02, 0.01, 0, 0, 0],
            ...     [0, 0.04, 0, 0, 0, 0],
            ... ])
            >>> robot.wait_idle()
        """
        poses = [self._curve_pose_values(pose, "move_spline_rel_tool waypoint", angle_unit) for pose in waypoints]
        if len(poses) < 2:
            raise ValueError("move_spline_rel_tool needs at least two waypoint poses")
        return self._queued("move_spline_rel_tool", [v for pose in poses for v in pose], timeout, options)

    # Other names (2026-10-23, the naming pass): the relative moves were documented as _rel_wrf / _rel_trf for a day;
    # every older program uses one of the two spellings, so both stay. The same functions, the robot accepts both
    # wire IDs (communication/protocol.COMMAND_ALIASES).
    move_linear_rel_wrf = move_linear_rel_world
    move_arc_rel_wrf = move_arc_rel_world
    move_circle_rel_wrf = move_circle_rel_world
    move_spline_rel_wrf = move_spline_rel_world
    move_linear_rel_trf = move_linear_rel_tool
    move_arc_rel_trf = move_arc_rel_tool
    move_circle_rel_trf = move_circle_rel_tool
    move_spline_rel_trf = move_spline_rel_tool

    def delay(self, seconds: float, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """Wait a given time between two queued moves.

        The pause is queued like a move: the arm stops and holds still for
        `seconds` after the moves queued before it, then the next move starts.

        Args:
            seconds: How long to wait, in seconds; more than 0.
            timeout: Seconds to wait for the move to finish, in sync mode only.
            **options: Settings for this pause only, the same as a move's: `speed`,
                `accel`, `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
                `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
                `singularity_strategy`, `samples` and `dry_run`. Only `duration`
                changes a pause: it replaces `seconds`. See Per-move settings.

        Returns:
            The command id. The call returns once the robot has accepted the pause;
            call `wait_idle()` to wait until it has finished. In sync mode the call
            returns when the pause has finished.

        Raises:
            CommandRejected: The robot refused the pause, for example outside EXEC
                mode or with an invalid argument. A pause needs neither an enabled
                nor a homed robot. In sync mode also when it fails after it was
                accepted.
            RobotTimeout: The robot did not accept the pause within 5 s. In sync
                mode also when the pause did not finish within `timeout`.
            RobotConnectionError: The client is not connected.

        Errors:
            INVALID_DURATION: The time is not more than 0 seconds, or not a
                finite number.
            ZONE_CHECK_UNAVAILABLE: Safety zones are active, but the robot model
                to check them is missing.

        See also:
            `wait_idle()`

        Examples:
            >>> robot.move_linear_rel_world(0, 0, -0.02)
            >>> robot.delay(0.5)
            >>> robot.move_linear_rel_world(0, 0, 0.02)
            >>> robot.wait_idle()
        """
        return self._queued("delay", [float(seconds)], timeout, options)

    # ------------------------------------------------------------------ #
    # Gripper (queued — the planner processes the queue strictly FIFO, so a gripper
    # command is never DISPATCHED to RTI ahead of a joint move queued before it).
    #
    # The checkpoint reflects REAL physical completion — the planner watches
    # gripper_object_detection (the gripper's status code) and only checkpoints
    # once the gripper has actually stopped (settled at target, or detected an
    # object), not just when the command was sent. So wait_idle()/
    # wait_motion_complete() genuinely wait for the gripper to finish, same as they
    # do for joint moves.
    #
    # Prefer the gripper-specific waits below, though: wait_motion_complete() waits
    # for the last QUEUED command of any kind, which is frequently a joint move
    # issued right after the gripper. wait_gripper_move_complete() waits for the
    # gripper itself, and wait_holding_part() / wait_released_part() wait for a
    # specific OUTCOME and raise GripFailed otherwise — so a missed grip cannot be
    # ignored the way an unchecked `if` can. gripper_result() exposes the raw code.
    #
    # What they do NOT do is wait for a PRECEDING joint move to physically finish
    # before the gripper command starts: dispatch is FIFO but non-blocking (a joint
    # move's trajectory batch is sent and the planner moves on immediately, without
    # waiting for RTI to finish executing it) — so move_joints(...); gripper_close()
    # will typically start closing the gripper WHILE the arm is still mid-motion.
    # For strict "move, then grip" sequencing, wait for the move first:
    #   robot.move_joints(...); robot.wait_motion_complete(); robot.gripper_close()
    #
    # Only one physical gripper target can be in flight at a time. Queuing a second
    # gripper command before the first settles supersedes it — the superseded
    # command's checkpoint resolves as an error (code SUPERSEDED, visible via
    # on_error() or if something specifically awaits it) rather than silently
    # claiming success for a target the gripper never actually reached.
    # ------------------------------------------------------------------ #

    def _queued_gripper(self, cmd, args, timeout, options):
        """_queued(), plus remember this as the last GRIPPER command.

        wait_gripper_move_complete() and the two outcome waits need the gripper's
        own pending entry: _last_queued is whatever was queued most recently,
        which is frequently a joint move issued right after.
        """
        return self._queued(cmd, args, timeout, options, gripper=True)

    def open_gripper(self, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """Open the gripper's jaws fully, as a queued command.

        The jaws move to position 1, next to the open end (0) of the gripper's 0
        to 255 scale, at speed 100 of 255 and with at most 200 mA. The command
        waits in the queue like a move, but the robot does not wait for a move to
        finish: it starts the jaws as soon as it has sent the moves queued before
        it. So the jaws can open while the arm still moves, and a move queued
        after it starts while the jaws still move.

        The command has settled when the jaws are within 5 of their target, or
        when the gripper steadily reports that they reached it or that something
        stopped them. `gripper_result()` then says which, and
        `wait_released_part()` raises `GripFailed` unless the jaws opened clear.

        Args:
            timeout: Seconds to wait for the jaws to settle, in sync mode only.
            **options: Accepted as on a move: `speed`, `accel`, `duration`,
                `profile`, `jerk`, `ramp_fraction`, `blend`, `blend_mode`,
                `blend_continuity`, `ik_solver`, `singularity_strategy`, `samples`
                and `dry_run`. Only `dry_run` changes a gripper command: the robot
                checks it and moves nothing. An unknown setting or a bad value is
                refused.

        Returns:
            The command id. The call returns once the robot has accepted the
            command; call `wait_gripper_move_complete()` to wait until the jaws
            have settled. In sync mode the call returns when they have settled.

        Raises:
            CommandRejected: The robot refused the command, for example outside
                EXEC mode, before homing, with an invalid setting, without a CAN
                gripper or in motor mode. In sync mode also when it fails after it
                was accepted.
            RobotTimeout: The robot did not accept the command within 5 s. In sync
                mode also when the jaws did not settle within `timeout`.
            RobotConnectionError: The client is not connected.

        Errors:
            GRIPPER_SETTLE_TIMEOUT: The jaws had neither reached the target nor
                reported an object 5 s after the command started. The commands
                queued after it still run.
            SUPERSEDED: Another gripper command started before this one settled
                and replaced its target. The commands queued after it still run.
            NO_GRIPPER: The robot has no CAN gripper; nothing is queued.
            GRIPPER_MOTOR_MODE: The gripper is in motor mode
                (`set_gripper_ctrl_mode()`); nothing is queued.

        Note:
            The gripper has one target at a time. A gripper command that starts
            before the previous one has settled replaces it, and the previous one
            fails with `SUPERSEDED`. Call `wait_gripper_move_complete()` between
            two gripper commands.

        Warning:
            The jaws move with force. Keep hands away from them.
            `stop()`, `disable()` and an error stop the jaws where they are.

        See also:
            `close_gripper()`, `move_gripper()`, `wait_released_part()`,
            `gripper_result()`

        Examples:
            >>> robot.open_gripper()
            >>> robot.wait_gripper_move_complete()
        """
        return self._queued_gripper("open_gripper", [], timeout, options)

    def close_gripper(self, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """Close the gripper's jaws fully, as a queued command.

        The jaws move to position 254, next to the closed end (255) of the
        gripper's 0 to 255 scale, at speed 100 of 255 and with at most 200 mA. A
        part between the jaws stops them, and they keep pressing on it with that
        current. The command waits in the queue like a move, but the robot does
        not wait for a move to finish: it starts the jaws as soon as it has sent
        the moves queued before it. So the jaws can close while the arm still
        moves, and a move queued after it starts while the jaws still move.

        The command has settled when the jaws are within 5 of their target, or
        when the gripper steadily reports that they reached it or that something
        stopped them. `gripper_result()` then says which, and
        `wait_holding_part()` raises `GripFailed` unless the jaws stopped on a
        part. Wait for one of them before the arm lifts the part.

        Args:
            timeout: Seconds to wait for the jaws to settle, in sync mode only.
            **options: Accepted as on a move: `speed`, `accel`, `duration`,
                `profile`, `jerk`, `ramp_fraction`, `blend`, `blend_mode`,
                `blend_continuity`, `ik_solver`, `singularity_strategy`, `samples`
                and `dry_run`. Only `dry_run` changes a gripper command: the robot
                checks it and moves nothing. An unknown setting or a bad value is
                refused.

        Returns:
            The command id. The call returns once the robot has accepted the
            command; call `wait_gripper_move_complete()` to wait until the jaws
            have settled. In sync mode the call returns when they have settled.

        Raises:
            CommandRejected: The robot refused the command, for example outside
                EXEC mode, before homing, with an invalid setting, without a CAN
                gripper or in motor mode. In sync mode also when it fails after it
                was accepted.
            RobotTimeout: The robot did not accept the command within 5 s. In sync
                mode also when the jaws did not settle within `timeout`.
            RobotConnectionError: The client is not connected.

        Errors:
            GRIPPER_SETTLE_TIMEOUT: The jaws had neither reached the target nor
                reported an object 5 s after the command started. The commands
                queued after it still run.
            SUPERSEDED: Another gripper command started before this one settled
                and replaced its target. The commands queued after it still run.
            NO_GRIPPER: The robot has no CAN gripper; nothing is queued.
            GRIPPER_MOTOR_MODE: The gripper is in motor mode
                (`set_gripper_ctrl_mode()`); nothing is queued.

        Note:
            The gripper has one target at a time. A gripper command that starts
            before the previous one has settled replaces it, and the previous one
            fails with `SUPERSEDED`. Call `wait_gripper_move_complete()` between
            two gripper commands.

        Warning:
            The jaws close with force and press on whatever is between them. Keep
            hands away from them.
            `stop()`, `disable()` and an error stop the jaws where they are.

        See also:
            `open_gripper()`, `move_gripper()`, `wait_holding_part()`,
            `gripper_result()`

        Examples:
            >>> robot.close_gripper()
            >>> robot.wait_gripper_move_complete()
            >>> robot.gripper_result().object_detected
            False
        """
        return self._queued_gripper("close_gripper", [], timeout, options)

    # Other names (2026-10-23, the naming pass): the names start with the verb, as every other command does.
    gripper_open = open_gripper
    gripper_close = close_gripper

    def gripper_result(self) -> GripperResult | None:
        """Return the outcome of this client's last settled gripper command.

        It reads what the robot reported when the command settled, and asks the
        robot nothing. In sync mode that is when the gripper call returned;
        otherwise after a wait such as `wait_gripper_move_complete()` or
        `wait_idle()`. A gripper command that failed leaves it unchanged: it then
        still returns the outcome of the one before. Gripper commands of other
        clients do not count.

        Returns:
            A `GripperResult` with the code and the command's id, or `None` when no
            gripper command of this client has settled yet.

        See also:
            `wait_holding_part()`, `wait_released_part()`,
            `wait_gripper_move_complete()`

        Examples:
            >>> robot.close_gripper()
            >>> robot.wait_gripper_move_complete()
            >>> robot.gripper_result()
            GripperResult(code=3, 'reached target, no object')
        """
        last = self._conn.last_gripper_result()
        if last is None:
            return None
        cid, code = last
        return GripperResult(code, command_id=cid)

    def move_gripper(self, position: int, speed: int | None = None, current: int | None = None, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """Move the gripper's jaws to a position, as a queued command.

        The position is on the gripper's own scale, not in millimetres: 0 is the
        fully open end and 255 the fully closed end, as the gripper found them
        when it was calibrated, and the scale runs evenly in between.
        `open_gripper()` and `close_gripper()` use 1 and 254. A part between the
        jaws stops them, and they keep pressing on it with `current`.

        Otherwise it behaves like them. The command waits in the queue like a
        move, but the robot starts the jaws as soon as it has sent the moves
        queued before it, so jaws and arm can move at the same time. It has
        settled when the jaws are within 5 of the target, or when the gripper
        steadily reports that they reached it or that something stopped them;
        `gripper_result()` then says which.

        Args:
            position: The target, from 0 (fully open) to 255 (fully closed). A
                fraction is dropped: 127.9 is sent as 127.
            speed: How fast the jaws move, from 0 (slowest) to 255 (fastest) on
                the gripper's scale. `None` means 100.
            current: The gripper motor's current limit while the jaws move and
                hold, in mA: they press with up to this current, and a stop
                against something is reported once they press with most of it.
                `None` means 200 mA. At most the current limit in the gripper's
                configuration.
            timeout: Seconds to wait for the jaws to settle, in sync mode only.
            **options: Accepted as on a move: `speed`, `accel`, `duration`,
                `profile`, `jerk`, `ramp_fraction`, `blend`, `blend_mode`,
                `blend_continuity`, `ik_solver`, `singularity_strategy`, `samples`
                and `dry_run`. Only `dry_run` changes a gripper command: the robot
                checks it and moves nothing. An unknown setting or a bad value is
                refused.

        Returns:
            The command id. The call returns once the robot has accepted the
            command; call `wait_gripper_move_complete()` to wait until the jaws
            have settled. In sync mode the call returns when they have settled.

        Raises:
            CommandRejected: The robot refused the command, for example outside
                EXEC mode, before homing, with an invalid setting, without a CAN
                gripper or in motor mode. In sync mode also when it fails after it
                was accepted.
            RobotTimeout: The robot did not accept the command within 5 s. In sync
                mode also when the jaws did not settle within `timeout`.
            RobotConnectionError: The client is not connected.
            ValueError: `position`, `speed` or `current` cannot be read as a whole
                number, for example the text "half". Nothing is sent.

        Errors:
            GRIPPER_SETTLE_TIMEOUT: The jaws had neither reached the target nor
                reported an object in time: 5 s at speed 100 or faster, and longer
                the lower `speed` is (about 30 s at speed 15). The commands queued
                after it still run.
            BAD_ARGUMENTS: `position` or `speed` is outside 0 to 255, or `current`
                is below 1 mA or above the current limit in the gripper's
                configuration.
            SUPERSEDED: Another gripper command started before this one settled
                and replaced its target. The commands queued after it still run.
            NO_GRIPPER: The robot has no CAN gripper; nothing is queued.
            GRIPPER_MOTOR_MODE: The gripper is in motor mode
                (`set_gripper_ctrl_mode()`); nothing is queued.

        Note:
            The gripper has one target at a time. A gripper command that starts
            before the previous one has settled replaces it, and the previous one
            fails with `SUPERSEDED`. Call `wait_gripper_move_complete()` between
            two gripper commands.

        Warning:
            The jaws move with force and press on whatever stops them. Keep hands
            away from them.
            `stop()`, `disable()` and an error stop the jaws where they are.

        See also:
            `open_gripper()`, `close_gripper()`, `set_gripper()`,
            `gripper_result()`

        Examples:
            >>> robot.move_gripper(128)
            >>> robot.wait_gripper_move_complete()
            >>> robot.move_gripper(200, speed=50, current=300)
            >>> robot.wait_gripper_move_complete()
        """
        args: list[int | None] = [int(position)]
        if speed is not None or current is not None:
            # A current without a speed keeps the speed's place empty: the robot takes its default speed there
            # (2026-10-24: the current was sent in the speed's place and read as the speed).
            args.append(None if speed is None else int(speed))
        if current is not None:
            args.append(int(current))
        return self._queued_gripper("move_gripper", args, timeout, options)

    # ------------------------------------------------------------------ #
    # Synchronization (status events + checkpoints)
    # ------------------------------------------------------------------ #

    def wait_idle(self, timeout: float | None = None, raise_on_error: bool = True) -> list[dict]:
        """Wait until every command this client queued has finished.

        It waits for this client's queued commands only: the moves, delays and
        gripper commands the robot accepted from it and has not yet answered as
        done. A command is done when it has finished, as `set_completion_policy()`
        defines it, or when it has failed. Another client's commands are not
        waited for.

        A queued command can fail after the robot accepted it: the robot cannot
        plan it (for example `JOINT_LIMIT`), `stop()` cancels it (`STOPPED`), or
        an earlier command failed or the robot left EXEC mode (`ABORTED`). The
        call reports every such failure since its last call, then forgets them,
        so a program that ran only part of its moves does not go on unnoticed.
        This client's `stop()` discards the failures not yet reported, and a
        failure a call already raised (a sync-mode command,
        `wait_motion_complete()`, `wait_gripper_move_complete()`) is not
        reported again. A command
        the robot refused when it was sent is not among them: its own call
        raised `CommandRejected`. When the connection is lost, the commands still
        running count as failed, with the code `CONNECTION_LOST`, or
        `DISCONNECTED` after `disconnect()`.

        Args:
            timeout: Seconds to wait at most. `None` waits as long as it takes;
                `0` checks once without waiting.
            raise_on_error: `True` raises `MotionFailed` when a command failed;
                `False` returns the failures instead.

        Returns:
            The failures since the last call, in the order they arrived: a dict
            each, with `id` (the id the queued call returned), `code` and
            `message`, and the error catalog's `number`, `title` and `fix` when
            the robot sent them. An empty list when every command finished, and
            always with `raise_on_error=True`.

        Raises:
            MotionFailed: With `raise_on_error=True`, a command failed. Its
                `failures` holds the list described under Returns.
            RobotTimeout: A command had not finished after `timeout`. The
                failures seen so far stay for the next call.

        See also:
            `wait_motion_complete()`, `on_motion_error()`, `stop()`,
            `set_completion_policy()`

        Examples:
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.move_joints(90, -100, 160, 0, -30, 180)
            >>> robot.wait_idle()
            []
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> failures = robot.wait_idle(timeout=60, raise_on_error=False)
        """
        if not self._conn.wait_idle(timeout=timeout):
            raise RobotTimeout("wait_idle timed out")
        failures = self._conn.take_motion_errors()
        if failures and raise_on_error:
            raise MotionFailed(failures)
        return failures

    def wait_motion_complete(self, timeout: float | None = None) -> None:
        """Wait until the last command this client queued has finished.

        It waits for one command: the last move, delay or gripper command the
        robot accepted from this client. A command the robot refused when it was
        sent does not count. It returns at once when this client has queued
        nothing yet or called `stop()` since, or when that command has already
        finished.

        It does not wait for the commands queued before it, nor report their
        failures: `wait_idle()` does both. Moves finish in the order they were
        queued, but a gripper command does not hold up the moves after it, so an
        earlier gripper command can still be moving.

        Args:
            timeout: Seconds to wait at most. `None` or `0` waits up to 120 s.

        Raises:
            CommandRejected: The command failed after the robot accepted it, for
                example with `JOINT_LIMIT`, `STOPPED` or `ABORTED`, or the
                connection was lost first (`CONNECTION_LOST` or `DISCONNECTED`).
                Its `code` names the failure. Every later call raises it again,
                until another command is queued or this client calls `stop()`.
            RobotTimeout: The command had not finished after `timeout`, or 120 s.

        See also:
            `wait_idle()`, `wait_gripper_move_complete()`, `on_motion_complete()`

        Examples:
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_motion_complete()
            >>> robot.move_joints(90, -100, 160, 0, -30, 180)
            >>> robot.wait_motion_complete(timeout=30)
        """
        if self._last_queued is None:
            return
        Connection.wait_done(self._last_queued, timeout or DEFAULT_MOVE_TIMEOUT)

    def wait_gripper_move_complete(self, timeout: float | None = None) -> None:
        """Wait until this client's last gripper command has settled.

        The jaws have settled when they reached the command's target, or when the
        gripper reported that something stopped them; this wait does not say
        which, `gripper_result()` does. It waits for the gripper command itself:
        `wait_motion_complete()` waits for the last queued command of any kind,
        which is often a move queued after the gripper. It returns at once when
        the command has already settled, and when this client has sent no
        gripper command.

        Args:
            timeout: The longest wait, in seconds. `None` or 0 mean 120 s.

        Raises:
            CommandRejected: The gripper command failed: its code says why, for
                example `GRIPPER_SETTLE_TIMEOUT`, `SUPERSEDED`, `STOPPED` or
                `ABORTED`, or the connection closed before it settled. Each call
                raises it again until the next gripper command, and `wait_idle()`
                reports the same failure once more, as `MotionFailed`.
            RobotTimeout: The command had not settled within `timeout`.

        See also:
            `gripper_result()`, `wait_holding_part()`, `wait_released_part()`,
            `wait_idle()`

        Examples:
            >>> robot.close_gripper()
            >>> robot.wait_gripper_move_complete()
            >>> robot.move_linear_rel_world(0, 0, 0.02)
            >>> robot.wait_idle()
        """
        if self._last_gripper_queued is None:
            return
        Connection.wait_done(self._last_gripper_queued,
                             timeout or DEFAULT_MOVE_TIMEOUT)

    def wait_holding_part(self, timeout: float | None = None) -> GripperResult:
        """Wait until the last gripper command has settled holding a part.

        It waits like `wait_gripper_move_complete()`, then checks why the jaws
        stopped: something must have stopped them before their target. Jaws that
        closed on nothing raise `GripFailed` as soon as they settle, so a missed
        pick cannot go unnoticed. Call it after `close_gripper()`, or a
        `move_gripper()` that closes, before the arm moves away.

        Args:
            timeout: The longest wait for the jaws to settle, in seconds. `None` or
                0 mean 120 s.

        Returns:
            The command's `GripperResult`: code 2 after a close. Code 1, an opening
            that something blocked, passes too.

        Raises:
            CommandRejected: The gripper command failed: its code says why, for
                example `GRIPPER_SETTLE_TIMEOUT` or `SUPERSEDED`, or the connection
                closed before it settled.
            GripFailed: The jaws settled without stopping on anything, or no
                gripper command of this client has settled yet. Its `result` holds
                the `GripperResult`, or `None`.
            RobotTimeout: The command had not settled within `timeout`.

        See also:
            `close_gripper()`, `wait_released_part()`, `gripper_result()`

        Examples:
            >>> from robotclient import GripFailed
            >>> robot.close_gripper()
            >>> try:
            ...     robot.wait_holding_part()
            ... except GripFailed as missed:
            ...     print(missed.result)
        """
        self.wait_gripper_move_complete(timeout)
        result = self.gripper_result()
        if result is None or not result.object_detected:
            raise GripFailed("holding a part", result)
        return result

    def wait_released_part(self, timeout: float | None = None) -> GripperResult:
        """Wait until the last gripper command has settled with the jaws clear.

        The mirror of `wait_holding_part()`: it waits like
        `wait_gripper_move_complete()`, then the jaws must have reached their
        target with nothing in the way. Jaws that something still blocked raise
        `GripFailed`: after an open, the part did not come free, and the arm must
        not carry it away. Call it after `open_gripper()`, or a `move_gripper()`
        that opens.

        Args:
            timeout: The longest wait for the jaws to settle, in seconds. `None` or
                0 mean 120 s.

        Returns:
            The command's `GripperResult`, with code 3.

        Raises:
            CommandRejected: The gripper command failed: its code says why, for
                example `GRIPPER_SETTLE_TIMEOUT` or `SUPERSEDED`, or the connection
                closed before it settled.
            GripFailed: The jaws did not report reaching their target with nothing
                in the way: something stopped them, or the gripper still reported
                motion when they came within 5 of the target (code 0), or no
                gripper command of this client has settled yet. Its `result` holds
                the `GripperResult`, or `None`.
            RobotTimeout: The command had not settled within `timeout`.

        See also:
            `open_gripper()`, `wait_holding_part()`, `gripper_result()`

        Examples:
            >>> robot.open_gripper()
            >>> robot.wait_released_part()
            GripperResult(code=3, 'reached target, no object')
        """
        self.wait_gripper_move_complete(timeout)
        result = self.gripper_result()
        if result is None or not result.reached_target:
            raise GripFailed("released and at target", result)
        return result

    def wait(self, predicate: Callable[[dict], bool], timeout: float | None = WAIT_STATE_TIMEOUT, what: str = 'wait') -> None:
        """Wait until the robot's status meets a condition of your own.

        `predicate` receives the latest status the robot sent: the dict
        `latest_status()` returns, with keys such as `mode`, `state`, `homed`,
        `err`, `errors`, `exec_paused` and `control_holder`. The call returns once
        `predicate` returns a true value. It checks it at once, then each time a
        new status arrives: the robot sends its status when it changes, checking
        50 times a second. It sends no command. `wait_mode()`, `wait_enabled()`,
        `wait_homed()` and `wait_error_reset()` are built on it.

        Args:
            predicate: A function that takes the status dict and returns a true
                value when the wait is over. Read it with `s.get(...)`: before the
                first status arrives the dict is empty.
            timeout: Seconds to wait at most. `None` waits as long as it takes.
            what: A name for the wait in the `RobotTimeout` message, which reads
                "<what> timed out".

        Raises:
            RobotTimeout: `predicate` was not true within `timeout`.
            RobotConnectionError: The client is not connected, or lost its
                connection while waiting.

        Note:
            `predicate` gets a copy of the status, and the client takes in no new
            status until it returns: keep it short, and do not send commands from
            it. An exception it raises ends the wait and reaches the caller.

        See also:
            `wait_mode()`, `latest_status()`, `on_status_change()`

        Examples:
            >>> robot.wait(lambda s: s.get("state") == "ENABLED" and s.get("homed"))
            >>> robot.wait(lambda s: not s.get("exec_paused"), timeout=30,
            ...            what="wait_resumed")
        """
        if self._conn.wait_status(predicate, timeout) is None:
            if not self._conn.connected:        # 2026-10-24: a lost connection read as "timed out", at once
                raise RobotConnectionError(f"{what}: not connected (the connection was lost or closed)")
            raise RobotTimeout(f"{what} timed out")

    def wait_enabled(self, timeout: float | None = WAIT_STATE_TIMEOUT) -> None:
        """Wait until the robot is enabled.

        It waits until the robot's status shows it enabled, and sends no command:
        enable it with `enable()`, or call `enable(wait=True)` to do both. It
        returns at once when the robot is already enabled.

        While an error is latched, the robot refuses to enable (`enable()` raises
        `CommandRejected`): this call then runs into its timeout. Call
        `clear_errors()` first.

        Args:
            timeout: Seconds to wait at most. `None` waits as long as it takes.

        Raises:
            RobotTimeout: The robot did not report itself enabled within
                `timeout`.
            RobotConnectionError: The client is not connected, or lost its
                connection while waiting.

        See also:
            `enable()`, `clear_errors()`, `wait()`

        Examples:
            >>> robot.enable()
            >>> robot.wait_enabled()
        """
        self.wait(lambda s: s.get("state") == "ENABLED", timeout, "wait_enabled")

    def wait_mode(self, mode: str, timeout: float | None = WAIT_STATE_TIMEOUT) -> None:
        """Wait until the robot is in a given mode.

        It waits until the robot's status shows `mode`, and sends no command:
        change the mode first, with `enter_exec_mode()`, `enter_idle_mode()` or
        the like, or call those with `wait=True` to do both. It returns at once
        when the latest status already shows `mode`. The robot sends its status
        when it changes, checking 50 times a second, so a mode that lasts less
        than about 20 ms can pass unseen.

        Args:
            mode: The mode, as the robot names it: `"IDLE"`, `"EXEC"`, `"JOG"`,
                `"HOMING"`, `"RTI"`, `"SOFT_RTI"`, `"SAFETY_STOP"`, `"FLASHING"`,
                `"ACTIVE_ERROR"` or `"BOOTING"`. Spelled any other way, for
                example in lower case, it never matches, and the wait runs into
                its timeout.
            timeout: Seconds to wait at most. `None` waits as long as it takes.

        Raises:
            RobotTimeout: The robot did not report `mode` within `timeout`.
            RobotConnectionError: The client is not connected, or lost its
                connection while waiting.

        See also:
            `wait()`, `latest_status()`, `enter_exec_mode()`, `enter_idle_mode()`

        Examples:
            >>> robot.enter_exec_mode()
            >>> robot.wait_mode("EXEC")
        """
        self.wait(lambda s: s.get("mode") == mode, timeout, f"wait_mode({mode})")

    def wait_homed(self, timeout: float | None = WAIT_HOMED_TIMEOUT) -> None:
        """Wait until the robot is homed and no homing is running.

        It waits until the robot's status shows the arm homed and the robot out of
        HOMING mode, so the homed flag of an earlier homing does not end the wait
        while a new one runs. It sends no command: start homing with `home()`, or
        call `home(wait=True)` to do both. It returns at once when the arm is
        homed and no homing runs.

        When homing ends in an error, it raises `HomingFailed` at once instead of
        waiting out the timeout. It also raises it when the arm is not homed and
        an error is latched, such as a pressed E-stop, even if no homing ran.
        Warnings do not count.

        Args:
            timeout: Seconds to wait at most. `None` waits as long as it takes.

        Raises:
            HomingFailed: Homing ended with an error, or an error is latched while
                the arm is not homed. Its `errors` lists the robot's error and
                warning keys, for example `J3:HOMING_FAILED`.
            RobotTimeout: The arm was not homed within `timeout`, for example
                because the robot refused or abandoned the homing.
            RobotConnectionError: The client is not connected, or lost its
                connection while waiting.

        Note:
            Right after `home()` on an arm that is already homed, the status can
            still show the earlier homing: this call then returns before the new
            homing starts. To home again and wait, call `home(wait=True)`.

        See also:
            `home()`, `clear_errors()`, `errors()`

        Examples:
            >>> robot.wait_homed()
            >>> robot.enter_exec_mode(wait=True)
        """
        self._wait_homed(timeout, "wait_homed")

    def wait_error_reset(self, timeout: float | None = WAIT_STATE_TIMEOUT) -> None:
        """Wait until the robot reports no error and no warning.

        It waits until the robot's status lists nothing but the warning
        `NOT_HOMED`, and sends no command. An
        error stays until `clear_errors()`, or the dashboard's Clear Errors
        button, clears it; a fault that is still present latches again. A warning
        lasts as long as its condition. It returns at once when nothing is
        listed.

        Unlike `clear_errors(wait=True)`, which ignores warnings, it also waits
        for every other warning to end.

        Args:
            timeout: Seconds to wait at most. `None` waits as long as it takes.

        Raises:
            RobotTimeout: An error or a warning was still listed after `timeout`.
            RobotConnectionError: The client is not connected, or lost its
                connection while waiting.

        Note:
            `NOT_HOMED`, listed while the arm is not homed, does not count: it
            describes a state, not a fault. To wait for the errors alone, use
            `clear_errors(wait=True)`.

        See also:
            `clear_errors()`, `errors()`, `wait()`

        Examples:
            >>> robot.clear_errors()
            >>> robot.wait_error_reset()
            >>> robot.enable(wait=True)
        """
        # NOT_HOMED is a state, not a fault: with it counted, this never passed on an unhomed arm (2026-10-24).
        self.wait(lambda s: not s.get("err") and not [k for k in (s.get("errors") or []) if k != "NOT_HOMED"],
                  timeout, "wait_error_reset")

    def errors(self) -> list[dict]:
        """Return the errors and warnings latched on the robot.

        It reads the latest status the robot sent, and asks the robot nothing; the
        list follows the robot within about 20 ms. An error stops the robot until
        `clear_errors()`. A warning is listed too, but it does not stop the robot.

        Returns:
            One dict per latched error or warning, oldest first, or `[]` when there
            is none: `key` (the error code, with `J<n>:` in front for a joint or
            `Z_<zone>:` for a safety zone, such as `"J3:WATCHDOG"`), `number` and
            `title` (from the error catalog; `None` and `""` for a code it lacks),
            `level` (`"error"` or `"warning"`) and `since` (when it latched, in
            seconds since 1970, UTC).

        See also:
            `clear_errors()`, `wait_error_reset()`, `latest_status()`

        Examples:
            >>> [e["key"] for e in robot.errors()]
            ['J3:WATCHDOG']
            >>> robot.clear_errors(wait=True)
        """
        return list(self.latest_status().get("error_details") or [])

    # ------------------------------------------------------------------ #
    # Data queries (immediate, never block)
    # ------------------------------------------------------------------ #

    def get_joints(self) -> list[float]:
        """Return the measured joint angles, in degrees.

        The robot reads every joint's encoder each control tick (4 ms on PAR6);
        this asks it for the latest reading. The angles are measured against the
        reference that homing sets, so read them on a homed arm. An angle is not
        wrapped into one turn: a joint whose range passes 180 degrees reports more,
        for example up to 376 degrees on PAR6's J3.

        Returns:
            One angle per joint, in degrees, J1 first.

        Raises:
            CommandRejected: The robot refused it: its control loop is not running,
                or it is in flashing mode.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            RTI_NOT_RUNNING: The control loop (RTI) has not run for over 0.5 s, or
                the robot is in flashing mode, where it reads no drive: the angles
                in its memory would be old.

        See also:
            `get_pose()`, `latest_status()`, `move_joints()`

        Examples:
            >>> robot.get_joints()
            [90.0, -105.0, 165.0, 0.0, -30.0, 180.0]
        """
        return self._immediate("get_joints")

    def get_pose(self, *, angle_unit: str = "rad") -> list[float]:
        """Return the measured pose of the tool centre point.

        The robot computes the pose every control tick from the measured joint
        angles, with the arm's model and the gripper in use. It is the tool centre
        point's position and orientation in the robot's base frame, the form
        `move_pose()` and `move_linear()` take.

        The orientation is three angles, R = Rx(rx) Ry(ry) Rz(rz): `rx` about the
        base X axis, then `ry` about the tool's new Y axis, then `rz` about the
        tool's own Z axis, its spin. `rx` and `rz` lie within -180 to 180 degrees,
        `ry` within -90 to 90 degrees.

        Args:
            angle_unit: `"rad"` or `"deg"`: the unit of the three angles returned.

        Returns:
            `[x, y, z, rx, ry, rz]`: the position in metres, then the angles in
            radians, or degrees with `angle_unit="deg"`.

        Raises:
            CommandRejected: The robot refused it: its control loop is not running,
                or it is in flashing mode.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `angle_unit` is neither `"rad"` nor `"deg"`.

        Errors:
            RTI_NOT_RUNNING: The control loop (RTI) has not run for over 0.5 s, or
                the robot is in flashing mode, where it reads no drive: the pose in
                its memory would be old.

        Note:
            A tool pointing straight down reads `rx` near 180 or -180 degrees, and
            can jump between the two on a tiny motion: the same orientation. With
            the tool's Z axis along base X or -X (`ry` at 90 or -90 degrees), `rx`
            and `rz` turn about the same axis: the robot then reports `rz` as 0 and
            the whole turn in `rx`.

        See also:
            `get_joints()`, `move_pose()`, `move_linear()`, `is_reachable()`

        Examples:
            >>> robot.get_pose(angle_unit="deg")
            [0.0, 0.2617, 0.2329, -120.0, 0.0, -90.0]
            >>> x, y, z, rx, ry, rz = robot.get_pose()
            >>> robot.move_linear(x, y, z + 0.02, rx, ry, rz)
            >>> robot.wait_idle()
        """
        return _pose_out(self._immediate("get_pose"), angle_unit)

    def get_health(self) -> dict:
        """Return the robot's health summary, one level per part.

        The robot judges each of its parts from its counters and readings, as the
        dashboard's Health card shows them, and returns one line per part. It first
        watches its counters advance for 50 ms, so the call takes a little longer
        than other reads. It moves nothing.

        A part's level is `"OK"`, `"WARN"`, `"ERROR"`, or `"STALE"` when the
        part's heartbeat has stopped and its data is old.

        Returns:
            A dict: `level` (the most serious part's level; `"STALE"` counts above
            `"ERROR"`), `time` (when it was judged, in seconds since 1970, UTC) and
            `components`, one dict per part: `name`, `level`, `summary` (one line
            saying why) and `values` (the numbers it was judged on; each part's are
            in the Status fields reference). The parts, in this order:
            `"control loop"`, `"memory lock"`, `"CAN"`, `"drives"`, `"gripper"`
            (with a CAN gripper only), `"errors"`, `"homing"`, `"executor"`,
            `"self-collision worker"`, `"recorder"`, `"processes"`, `"disk"`,
            `"memory"`, `"CPU temperature"` and `"clock"`.

        Raises:
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        See also:
            `get_hardware()`, `get_versions()`, `errors()`

        Examples:
            >>> health = robot.get_health()
            >>> health["level"]
            'OK'
            >>> [c["summary"] for c in health["components"] if c["level"] != "OK"]
            []
        """
        return self._immediate("get_health")

    def get_hardware(self, can: bool = True) -> dict:
        """Return what the drives, the CAN bus and the control box report.

        One read of everything the hardware reports, for checking a build: each
        drive's identity, supply and faults, the CAN interface's error counters,
        and the E-stop, the buttons and the isolated inputs and outputs. It moves
        nothing.

        Args:
            can: Also read the CAN interface's state and error counters, which
                takes the robot about 5 ms. `False` skips them, for polling.

        Returns:
            A dict:
            `drives`: one dict per joint, J1 first: `name` (`"J1"`), `can_id`,
                `answered_scan` (it answered the scan at start-up), `answering` (its
                data is fresh now), `lost` (its lost-connection error is latched),
                `firmware`, `hardware`, `serial`, `batch`, `voltage_v` (its supply,
                in volts), `temperature_c` (in degrees Celsius), `fault` (a fault is
                reported), `flags` (each fault flag: `temperature`, `encoder`,
                `supply_voltage`, `driver`, `velocity`, `current`, `estop` and
                `watchdog`), `calibrated`, `driver_type`, `firmware_supported`
                (`"ok"`, `"too_old"`, `"too_new"`, `"undeclared"`, or `None` before
                the drive reports) and `supported_range`.
            `gripper`: the same for a CAN gripper's drive, without `batch`, or
                `None`.
            `scan_unexpected_ids`: the CAN ids that answered the start-up scan
                although this robot does not use them.
            `can_link`: the CAN interface's `interface`, `state`, `restart_ms` and
                its counters `restarts`, `bus_errors`, `arbitration_lost`,
                `error_warning`, `error_passive` and `bus_off`; `None` with
                `can=False` or when the robot cannot read them.
            `io`: the control box: `estop_pressed`, per E-stop channel `estop_raw`
                (the level, 0 = pressed) and `estop_channel_pressed`,
                `estop_channels_used`, `software_estop`, `buttons_pressed` (`B1` and
                `B2`), `inputs` and `outputs` (the three isolated pins' levels) and
                `pcb_version`.
            `joint_num`, `has_gripper`, `tick` (the control tick of the read) and
                `time` (in seconds since 1970, UTC).

        Raises:
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        See also:
            `get_versions()`, `get_health()`

        Examples:
            >>> hardware = robot.get_hardware(can=False)
            >>> [d["voltage_v"] for d in hardware["drives"]]
            [23.6, 23.5, 23.4, 23.6, 23.5, 23.3]
        """
        return self._immediate("get_hardware", [1 if can else 0])

    def get_versions(self) -> dict:
        """Return the version of every part of the robot.

        The robot reports its software, the operating system, the description
        package, the control box and each drive, and the clients connected now. A
        value a part has not reported yet is `None`. The software's own version is
        the one the robot software started with. It moves nothing.

        Returns:
            A dict:
            `runtime`: the robot software's `version`, `branch` and `commit`.
            `python`: the Python version that runs it.
            `os`: `name`, `kernel` and `realtime` (whether the kernel is a
                real-time one).
            `description_package`: `version`, `path`, `min` (the oldest version the
                robot's configuration accepts) and `ok` (whether it is at least
                that).
            `protocols`: `commands`, `rti` and `telemetry`, the protocol versions.
            `hardware`: what the configuration declares: `robot`, `revision`,
                `driver_types` (per joint), `gripper`, `gripper_revision`,
                `gripper_driver_type` and `description_package_min`; and `error`
                when a configuration file could not be read.
            `drives`: one dict per joint: `joint` (1 for J1), `can_id`, `firmware`,
                `hardware`, `serial`, `batch`, `driver_type`, `supported` (as
                `firmware_supported` in `get_hardware()`) and `supported_range`.
            `gripper`: the same for a CAN gripper's drive, without `joint` and
                `batch`, or `None`.
            `control_box`: `pcb_configured` (the PCB revision the configuration
                names), and `hardware`, `software`, `serial` and `batch` as the
                board reports them.
            `config_formats`: `reads` (per kind of configuration file, the newest
                format this software reads) and `files` (each file read, with its
                `kind` and `schema`).
            `clients`: one dict per connected client: `client` (its name and
                address), and its client library's `version` and `protocol`.
            `time`: when it was read, in seconds since 1970, UTC.

        Raises:
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        See also:
            `get_hardware()`, `get_health()`

        Examples:
            >>> robot.get_versions()["runtime"]["version"]
            '0.10.0'
        """
        return self._immediate("get_versions")

    def set_log_level(self, name: str, level: str) -> Any:
        """Set the level of one of the robot's loggers, without a restart.

        The robot software writes its logs through named loggers, one per part.
        This sets one of them in the robot's two control processes, the command
        executor and the real-time loop: lines below the level are no longer
        written. A logger's level also applies to the loggers under it that have
        none of their own: `"rti"` covers `"rti.can"` until `"rti.can"` is set.
        The dashboard, the supervisor, the recorder and the self-collision worker
        keep their own levels; the dashboard's Logs tab sets the dashboard's. The
        level lasts until that process restarts.

        Args:
            name: The logger: `"rti"`, `"rti.startup"`, `"rti.loop"`, `"rti.mode"`,
                `"rti.flash"`, `"rti.errors"`, `"rti.can"`, `"rti.safety"`,
                `"rti.homing"`, `"rti.handler"`, `"rti.exec"`, `"rti.gripper"`,
                `"rti.dynamics"`, `"rti.selfcheck"`, `"rti.recipe"`,
                `"rti.incident"`, `"rti.net"`, `"exec"`, `"exec.control"`,
                `"exec.tcp"`, `"exec.forward"`, `"exec.status"`, `"exec.planner"`,
                `"exec.stop"`, `"config"`, `"filters"`, `"shm"`, `"kin"`, `"proc"`,
                `"io"`, or the libraries' `"toppra"` and `"can"`. The logging
                documentation says what each one writes. The other processes'
                loggers (`"sup"` and the ones under it, `"gui"`, `"sc"`, `"rec"`)
                are refused: this command does not reach those processes.
            level: `"DEBUG"`, `"INFO"`, `"WARNING"` or `"ERROR"`, in any case.

        Defaults:
            `"INFO"` for every logger, `"WARNING"` for `"toppra"` and `"can"`, set
            each time the command executor or the real-time loop starts.

        Returns:
            `{"name": name, "level": level}`: the logger and the level now set.

        Raises:
            CommandRejected: The robot refused it: the logger is unknown or belongs to
                another process, or the level is unknown. Or RTI refused it or did not
                answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            BAD_ARGS: `name` is not one of the loggers above, or `level` is not one
                of the four levels.
            RTI_REFUSED: RTI has no such logger or level. The message says why.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        See also:
            `capture_telemetry()`, `get_health()`

        Examples:
            >>> robot.set_log_level("exec.planner", "DEBUG")
            {'name': 'exec.planner', 'level': 'DEBUG'}
            >>> robot.set_log_level("exec.planner", "INFO")
        """
        return self._immediate("set_log_level", [str(name), str(level).upper()])

    def capture_telemetry(self, seconds: float = 10.0, name: str = "") -> Any:
        """Record the robot's per-tick data into a file on the robot.

        The robot's recorder saves every control tick's values from 5 s before
        the request to `seconds` after it: the joints' positions, speeds,
        accelerations, jerks and torques, raw and filtered; the commanded and
        target values; the tool centre point's pose, speed and acceleration; the
        external torques and forces; the feedforward torques; the drives' error
        flags; the loop's timing and the real-time link. The file is
        `temp_data/captures/<UTC time>_<name>.npz` on the robot; the dashboard's
        Recordings tab, Captures, plots it and downloads it. The real-time loop
        does no extra work for it.

        The call returns at once, when the request is placed. The robot's activity
        log says `CAPTURE_STARTED`, then `CAPTURE_SAVED` once the file is written.
        One capture runs at a time; `stop_capture()` ends it early.

        Args:
            seconds: How long to record after the request, in seconds, 1 to 60.
            name: A label for the file name. Letters, digits, `-` and `_` are kept,
                anything else becomes `-`, up to 32 characters; an empty one is
                `"capture"`.

        Returns:
            `{"seconds", "name", "folder"}`: the length, the label as the file name
            will carry it, and the captures' folder on the robot.

        Raises:
            CommandRejected: The robot refused it; the code says why (see Errors).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `seconds` is not a number. Nothing is sent.
            TypeError: `seconds` is `None`. Nothing is sent.

        Errors:
            BAD_ARGS: `seconds` is not from 1 to 60.
            CAPTURE_REFUSED: The recorder process is not running; nothing is
                recorded.

        Note:
            While another capture runs, the recorder refuses this one in its
            activity log only (`CAPTURE_REFUSED`); the call still returns normally.
            The robot keeps the 10 newest captures and deletes older ones. The 5 s
            before the request and the 10 kept are the Capture settings of the
            dashboard's Recordings tab (`config/system.xml` `<recorder>`).

        See also:
            `stop_capture()`, `set_log_level()`

        Examples:
            >>> robot.capture_telemetry(5, "start_pose")
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
        """
        return self._immediate("capture_telemetry", [float(seconds), str(name)])

    def stop_capture(self) -> Any:
        """End the running telemetry capture now and save what it has.

        The recorder writes the capture's file with the ticks recorded so far, and
        the robot's activity log says `CAPTURE_SAVED`, "stopped early". The call
        returns at once. Any client can stop a capture, also one that another
        client started.

        Returns:
            `None`: the robot sends no data back.

        Raises:
            CommandRejected: The recorder process is not running.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            CAPTURE_REFUSED: The recorder process is not running.

        Note:
            With no capture running, the recorder refuses the stop in its activity
            log only (`CAPTURE_REFUSED`); the call still returns normally.

        See also:
            `capture_telemetry()`

        Examples:
            >>> robot.capture_telemetry(60, "long_test")
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
            >>> robot.stop_capture()
        """
        return self._immediate("stop_capture")

    def get_status(self) -> dict:
        """Ask the robot for its state now, with the joint angles.

        The robot answers at once from its shared memory. The answer is not the
        status that `latest_status()` keeps: it adds the joint angles, the pose
        angle convention and the planner's state in one dict, and it names three
        flags differently. Most programs read `latest_status()`, which needs no
        round trip to the robot.

        Returns:
            A dict:
            `mode`, `state` and `homed`: as in `latest_status()`.
            `pose_angles`: `"xyz-mobile"`, the pose angles R = Rx Ry Rz that this
                robot reads and returns.
            `runtime_version`: the robot software's version, such as `"0.10.0"`.
            `protocols`: `commands`, `rti` and `telemetry`, the protocol versions.
            `error_active`: 1 while an error is latched (`err` in the status).
            `safety_violation`: 1 while a safety monitor error is latched
                (`safety` in the status).
            `software_estop`: 1 while the dashboard's software E-stop is on
                (`estop` in the status).
            `exec_paused`, `exec_settling`, `exec_stopping`: 1 while a program is
                paused, waits for the arm to settle at the end of a move, or brakes
                for a stop.
            `exec_completion_policy`: `"commanded"`, `"settled"` or `"strict"`.
            `planner`: the planner's state: `queue_depth`, `active_command`,
                `executing_seq`, `completed_seq`, `current_profile`, `last_error`,
                `last_diagnostic`, `motion_aborted`, `ik_config`,
                `default_ik_solver` and `default_fk_jacobian_backend` (see the
                Status fields reference).
            `joints`: the measured joint angles, in degrees, J1 first.

        Raises:
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        See also:
            `latest_status()`, `get_joints()`, `errors()`

        Examples:
            >>> status = robot.get_status()
            >>> status["mode"], status["planner"]["queue_depth"]
            ('EXEC', 0)
        """
        return self._immediate("get_status")

    def is_reachable(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, near: Sequence[float] | None = None,
                     *, angle_unit: str = "rad") -> dict:
        """Check whether the arm can reach a pose, without moving it.

        The robot searches for joint angles that put the tool centre point at the
        pose, within 0.2 mm and 0.5 degrees, with every joint inside its software
        limits. It starts from `near`, then from 128 fixed points spread over the
        joint range, so a pose always gets the same answer. Of the solutions it
        finds, it returns the one closest to `near` that keeps the arm's parts
        apart by the planner's self-collision margin. It also checks the pose
        against the enabled safety zones that stop the robot, margins included. A
        pose out of reach takes around 50 ms to refuse.

        It checks the end pose only. A `move_joints()` to the joints returned can
        still be refused for its path, which can cross a safety zone or bring two
        parts of the arm too close. `move_pose()` searches only near where the arm
        stands, so it can refuse a pose this call finds reachable.

        Args:
            x: Position along the base X axis, in metres.
            y: Position along the base Y axis, in metres.
            z: Position along the base Z axis, in metres.
            rx: Turn about the base X axis, in radians, or degrees with
                `angle_unit="deg"`. The orientation is R = Rx(rx) Ry(ry) Rz(rz).
            ry: Turn about the tool's new Y axis, in the same unit.
            rz: Turn about the tool's own Z axis, in the same unit.
            near: Joint angles in degrees, one per joint, J1 first: the solution
                closest to them is preferred. `None` means the arm's measured
                angles now.
            angle_unit: `"rad"` or `"deg"`: the unit of `rx`, `ry` and `rz`. `near`
                is in degrees either way.

        Returns:
            A dict:
            `reachable`: `True` when joint angles inside the limits reach the pose.
            `joints`: those joint angles in degrees, J1 first, or `None` when the
                pose is out of reach.
            `allowed`: `True` when the pose is reachable, outside every stopping
                safety zone and clear of self-collision.
            `zone`: the name of a stopping safety zone the pose is in, or `None`.
            `self_collision`: the two parts of the arm that come too close, such
                as `"upper arm and tool"`, when every solution found does so;
                `joints` is then the closest of those. Otherwise `None`.

        Raises:
            CommandRejected: The robot refused it; the code says why (see Errors).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `angle_unit` is neither `"rad"` nor `"deg"`, or a value is
                not a number. Nothing is sent.
            TypeError: A value is `None`, or `near` is not a sequence of numbers.
                Nothing is sent.

        Errors:
            BAD_ARGS: A value is not a finite number, or `near` does not have one
                angle per joint.
            NO_ROBOT_MODEL: The robot software could not load the arm's model, so
                it cannot search.
            SELF_COLLISION_MODEL: Self-collision checking is on, but its model
                could not be built from the geometry files.

        See also:
            `move_joints()`, `move_pose()`, `get_pose()`, `get_zones()`

        Examples:
            >>> x, y, z, rx, ry, rz = robot.get_pose()
            >>> answer = robot.is_reachable(x, y, z - 0.05, rx, ry, rz)
            >>> answer["allowed"]
            True
            >>> robot.move_joints(*answer["joints"])
            >>> robot.wait_idle()
        """
        args = _pose_in((x, y, z, rx, ry, rz), angle_unit)
        if near is not None:
            args += [float(a) for a in near]
        return self._immediate("is_reachable", args)

    def get_zones(self) -> dict:
        """Return the safety zones in force.

        A zone describes the cell around the robot, not the arm: an allowed zone is
        a space the robot must stay inside, a forbidden one a space it must stay
        out of. The robot checks the tool centre point and the gripper's tool
        spheres against every enabled zone: the planner before each EXEC move,
        keeping the zone's margin, and the safety monitor on the measured pose,
        once a sphere is more than 5 mm past the zone's surface
        (`<safety_monitor><zone_tolerance>` in `config/system.xml`). In freedrive a
        zone can push back as a wall you feel.

        It asks the robot for what is in force now and moves nothing.

        Returns:
            A dict with four keys:

            - `cell`: the active cell's name, from `config/system.xml`
              `<active_cell>`; `""` means no zones.
            - `zones`: one dict per zone of the cell file, disabled ones included:
              `name`; `shape` (`"box"`, `"prism"`, `"cylinder"`, `"sphere"` or
              `"plane"`); `kind` (`"allowed"` or `"forbidden"`); `reaction`
              (`"stop"`: the error `Z_<name>:ZONE_VIOLATION` stops the robot;
              `"warn"`: a live `Z_<name>:ZONE_WARNING`); `checks` (`"tool"`: the
              tool centre point and the tool spheres; `"tcp"`: the point only);
              `margin` (m, the planner's clearance); `enabled`; `haptic` (a wall in
              freedrive); `freedrive` (`"warn"` or `"stop"`: the reaction when
              pushed through by hand); `geometry`, in metres in the base frame
              (box: `min`, `max`; prism: `vertices`, `z_min`, `z_max`; cylinder:
              `p1`, `p2`, `radius`, `rounded`; sphere: `center`, `radius`; plane:
              `point`, `normal`).
            - `tool_spheres`: `[x, y, z, radius]` per tool sphere, in metres, in
              the tool frame.
            - `haptic_walls`: the `<haptic_walls>` policy of `config/system.xml`:
              `enabled`, `stiffness` (N/m), `damping` (N s/m), `max_force` (N),
              `breakthrough` (m), `velocity_tau` (s), `zone_mass_scaled`,
              `joint_walls`, `joint_max_torque` (Nm), `joint_breakthrough` (deg).

        Raises:
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        See also:
            `reload_zones()`, `is_reachable()`, `get_joint_walls()`

        Examples:
            >>> zones = robot.get_zones()
            >>> [z["name"] for z in zones["zones"] if z["enabled"]]
        """
        return self._immediate("get_zones")

    def reload_zones(self) -> dict:
        """Re-read the safety zones from their files and apply them.

        The robot reads `config/system.xml` again (the `<active_cell>` name and the
        `<haptic_walls>` policy), then the cell file `config/cells/<name>.xml` and
        the gripper XML's tool spheres, and checks every zone. Only when all of it
        is valid does it apply them, at once and without a restart or re-homing:
        to the planner, to `is_reachable()`, to the safety monitor and to the
        freedrive walls. An empty `<active_cell>` removes every zone.

        A cell file or tool sphere that is not valid is refused with
        `ZONE_CONFIG`, and the zones in force stay as they were. A `<haptic_walls>`
        value outside its range is not refused: it falls back to its default, with
        a warning in the robot's log. The gripper is the one the robot software
        started with; another `<active_gripper>` needs a restart.

        Moves planned before the call are not checked again; the safety monitor
        checks every move against the new zones. A wall never pushes on its own: a
        tool sphere already inside a zone when it is reloaded feels nothing until
        it has been outside once.

        Defaults:
            When the robot software starts it reads the same files, and a file that
            is not valid stops it. No zones as shipped: `<active_cell>` is empty.

        Returns:
            The zones now in force, as `get_zones()` returns them.

        Raises:
            CommandRejected: The robot refused it: the cell file or a tool sphere is not
                valid (`ZONE_CONFIG`), or another client holds control. Or RTI refused
                it or did not answer within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            ZONE_CONFIG: `config/system.xml` or the gripper XML cannot be read or
                is not valid XML; `<active_cell>`
                is not a plain name (letters, digits, `_`,
                `-`) or names no file in `config/cells/`; the cell file is not
                valid XML, its root is not `<cell>`, it holds other elements than
                `<zone>`, or it is written for a newer robot software; a zone in it
                breaks a rule of the cell file (its name, shape, kind, reaction,
                checks, margin, enabled, haptic or freedrive value, its geometry,
                an unknown attribute or element) or shares its name with another;
                or a tool sphere in the gripper XML is not four numbers with a
                radius of at least 0. The message names the file, the zone and the
                reason.
            RTI_REFUSED: RTI could not use the zones the robot read. The message says
                why; the zones in force stay.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            Zones are not a safety-rated function, and they check only the tool
            centre point and the gripper's tool spheres: the arm's links can still
            enter a zone.

        See also:
            `get_zones()`, `is_reachable()`, `get_joint_walls()`

        Examples:
            >>> zones = robot.reload_zones()
            >>> zones["cell"]
        """
        return self._immediate("reload_zones")

    def get_joint_walls(self) -> dict:
        """Return the freedrive joint walls in force, with their stable ranges.

        In freedrive (IDLE mode, on an enabled, homed arm with gravity compensation
        on) every joint has a wall near each end of its range, which pushes back
        like a spring with damping as you turn the joint into it. A wall is set by
        how it behaves, its natural frequency and damping ratio, the same on every
        joint; the robot turns that into stiffness and damping with the joint's
        inertia at each pose. A joint pushed through its wall gets its `freedrive`
        reaction.

        It asks the robot for the walls in force, the values they resolve to and
        the ranges that stay stable. It moves nothing.

        Returns:
            `{"walls": [...], "policy": {...}, "robot_xml": name}`. `walls` holds
            one dict per joint:

            - `joint` (1 to 6), `enabled` (whether the joint has a wall), `at`
              (`"soft"` or `"hard"`: at the soft or the hard limit), `lo_deg` and
              `hi_deg` (where its two walls are, deg).
            - In force: `frequency_hz`, `damping_ratio`, `filter_ms` (the speed
              filter behind the damping, ms), `scaling` (`"adaptive"` or
              `"fixed"`), `max_torque_nm`, `breakthrough_deg`, `freedrive`
              (`"warn"` or `"stop"`).
            - Stable ranges: `frequency_max_hz` (for this filter),
              `damping_ratio_window` (at this frequency and filter),
              `breakthrough_max_deg` (`None` when no endstop limits it).
            - At the joint's lightest pose: `stiffness_nm_per_deg`,
              `damping_nm_s_per_deg`, `stiffness_max_nm_per_deg`,
              `damping_window_nm_s_per_deg` and `inertia_kg_m2`;
              `stiffness_scale_max` (how many times stiffer at its heaviest pose),
              `scale_here` (at the arm's pose now), `drive_torque_nm` (the most
              the joint's drive can give, Nm).
            - `spec` (the settings as given, `None` meaning auto), `unsaved`
              (`True` when they differ from the robot XML), `note` (why a setting
              is not used, or `""`).

            `policy` is the part of `config/system.xml` `<haptic_walls>` the walls
            use: `enabled`, `joint_walls`, `stiffness` (N/m), `damping` (N s/m),
            `velocity_tau` (s), `joint_max_torque` (Nm), `joint_breakthrough`
            (deg). `robot_xml` names the robot XML file the walls are saved to.

        Raises:
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        See also:
            `set_joint_wall()`, `save_joint_walls()`, `get_zones()`

        Examples:
            >>> wall = robot.get_joint_walls()["walls"][4]  # J5
            >>> wall["frequency_hz"], wall["damping_ratio"]
        """
        return self._immediate("get_joint_walls")

    def set_joint_wall(self, joint: int, **fields: Any) -> Any:
        """Change one joint's freedrive wall, live.

        The robot merges the fields given into the joint's wall settings, checks
        the result and applies it at once; fields left out keep their value.
        Numbers are kept to 4 significant digits. Every wall re-arms: a joint that
        is inside its wall feels nothing until it has been back in its range once.
        Called with no fields, it sends every joint's live walls again.

        The change is not saved: `save_joint_walls()` writes it into the robot
        XML. Until then, the robot software starts again with the robot XML's
        walls.

        Args:
            joint: The joint, from 1 to 6.
            **fields: The settings to change. Each takes a value, or `"auto"` (or
                `None`) for its default; `freedrive` takes no `"auto"`.
                `at`: `"soft"` (the wall at the soft limit) or `"hard"` (at the
                hard limit; refused on a joint with an endstop, whose hard limit
                is the endstop).
                `frequency_hz`: the wall's natural frequency, in Hz, above 0 and up
                to the filter's stable maximum; higher feels stiffer.
                `damping_ratio`: above 0 and inside the stable window at that
                frequency; about 0.7 settles without bouncing, below 0.3 it rings.
                `filter_ms`: the speed filter behind the damping, 0 to 8 ms; less
                filtering allows a stiffer wall.
                `max_torque_nm`: the most the wall pushes, in Nm, above 0 and up to
                `<haptic_walls><joint_max_torque>` (20 Nm as shipped).
                `breakthrough_deg`: how far past the wall the joint counts as
                pushed through, 2 to 90 deg, never past an endstop.
                `freedrive`: `"warn"` (a live `J<n>:JOINT_WALL_WARNING`) or
                `"stop"` (the error `J<n>:JOINT_WALL`) when pushed through.
                `scaling`: `"adaptive"` (stiffness and damping follow the joint's
                inertia at each pose) or `"fixed"` (the lightest pose's at every
                pose: softer where the joint carries more).

        Defaults:
            From each joint's `<freedrive_wall>` in the robot XML, read when the
            robot software starts; `save_joint_walls()` writes it. Every field
            `auto` as shipped: 7.1 Hz, damping ratio 0.67 and a 4 ms filter (from
            `config/system.xml` `<haptic_walls>`), `"adaptive"`, `"warn"`; the
            wall at the soft limit, or at the hard limit on a joint without an
            endstop; a breakthrough of the soft-to-hard margin, 2 to 15 deg (15 deg
            at a hard limit); a max torque of the spring at the breakthrough at the
            heaviest pose, at most 5 Nm.

        Returns:
            The walls now in force, as `get_joint_walls()` returns them.

        Raises:
            CommandRejected: The robot refused it; the code says why (see Errors). Also
                when another client holds control. Or RTI refused it or did not answer
                within 1 s (`Errors:`).
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `joint` is not a whole number. Nothing is sent.
            TypeError: `joint` is not a number. Nothing is sent.

        Errors:
            BAD_ARGS: `joint` is not from 1 to 6; a field name is unknown; a value
                is not one of its words, not a number or outside its range; `at` is
                `"hard"` on a joint with an endstop; or `breakthrough_deg` reaches
                past the joint's endstop. Nothing changes.
            WALL_UNSTABLE: The frequency is above the stable maximum for the
                joint's filter, or the damping ratio is outside the stable window
                at that frequency and filter. The message gives the stable range;
                nothing changes.
            WALL_OFF: The joint has no wall: joint walls are off in
                `config/system.xml` `<haptic_walls>` (`<enabled>` or
                `<joint_walls>` is 0), or the robot has no inertia for the joint.
                Nothing changes.
            RTI_REFUSED: RTI could not use the wall settings. The message says why; the
                walls in force stay.
            RTI_NOT_RUNNING: RTI did not answer within 1 s: it is not running or not
                responding. The command may still have taken effect.

        Warning:
            A wall pushes with up to its max torque: let go of a joint deep inside
            its wall and it springs back. Try a new setting gently, with a hand on
            the arm.

        See also:
            `get_joint_walls()`, `save_joint_walls()`

        Examples:
            >>> robot.set_joint_wall(5, frequency_hz=5.0, damping_ratio=0.7)
            >>> robot.set_joint_wall(5, frequency_hz="auto", damping_ratio="auto")
        """
        return self._immediate("set_joint_wall", [_whole(joint, "joint"), dict(fields)])

    def save_joint_walls(self) -> Any:
        """Save the live freedrive walls into the robot XML.

        The robot writes every joint's live wall settings into its `<joint>` block
        of the robot XML as a `<freedrive_wall>` element, replacing the one there
        or adding it; the rest of the file, comments included, stays as it is.
        Settings at their default are written as `auto`. The walls in force do not
        change; from then on the robot software also starts with them.

        Returns:
            The walls in force, as `get_joint_walls()` returns them, now with
            `unsaved` false.

        Raises:
            CommandRejected: The robot refused it: the file could not be written
                (`SAVE_FAILED`), or another client holds control.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.

        Errors:
            SAVE_FAILED: The robot XML could not be read or written, has no
                `<joint>` block for one of the joints, or would no longer be valid
                XML after the change. The walls in force stay as they are.

        See also:
            `set_joint_wall()`, `get_joint_walls()`

        Examples:
            >>> robot.set_joint_wall(5, frequency_hz=6.0)
            >>> robot.save_joint_walls()
        """
        return self._immediate("save_joint_walls")

    # ------------------------------------------------------------------ #
    # Digital IO (immediate, isolated pins 1-3)
    # ------------------------------------------------------------------ #

    def set_digital_output(self, pin: int, value: bool | int) -> None:
        """Switch one of the control box's isolated outputs on or off.

        The robot takes the new level at once and switches the output within one
        control tick (4 ms). Outputs 1, 2 and 3 are on terminals 6, 7 and 8 of the
        control box; an isolated output switches an external voltage.
        `get_hardware()` reports every output's level, under `"io"`.

        It is not queued: the output switches when the command arrives, while moves
        sent before it may still be running. To switch it after a move, call
        `wait_idle()` first.

        Args:
            pin: The output: `1`, `2` or `3`. A number with a fraction is cut to a
                whole number first.
            value: `True` or `1` switches the output on, `False` or `0` off. Any
                other value is refused.

        Defaults:
            Every output is off each time the robot software starts, also after a
            restart.

        Raises:
            CommandRejected: The robot refused it: `pin` is not 1, 2 or 3, or
                another client holds control.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `pin` cannot be read as a whole number, for example
                `"one"`, or `value` is not `True`, `False`, 1 or 0. Nothing is
                sent.
            TypeError: `pin` is `None` or another type that is no number. Nothing
                is sent.

        Errors:
            BAD_ARGS: The pin or the value is missing, or the value is not true or
                false (1 / 0); only a raw client can send that.
            BAD_PIN: `pin` is not 1, 2 or 3.

        Warning:
            The robot never switches an output off by itself: not on an error, a
            pressed E-stop, `disable()` or `safety_stop()`, nor when your program
            ends or disconnects. Switch outputs off yourself when they must not
            stay on.

        See also:
            `get_digital_input()`, `wait_idle()`, `get_hardware()`

        Examples:
            >>> robot.move_joints(90, -105, 165, 0, -30, 180)
            >>> robot.wait_idle()
            >>> robot.set_digital_output(1, True)
            >>> robot.set_digital_output(1, False)
        """
        # Strict (2026-10-24): bool(value) made any non-empty string, even "0", switch the output on.
        if isinstance(value, bool) or (isinstance(value, (int, float)) and value in (0, 1)):
            return self._immediate("set_digital_output", [int(pin), bool(value)])
        raise ValueError(f"set_digital_output: value must be True / False or 1 / 0, got {value!r}")

    def get_digital_input(self, pin: int) -> bool:
        """Return whether one of the control box's isolated inputs is on.

        The robot reads every input once per control tick (4 ms) and takes a change
        only after five equal reads in a row (20 ms), so a short glitch is ignored.
        The call returns the level it read last. Inputs 1, 2 and 3 are on terminals
        11, 10 and 9 of the control box. An input with no signal on it reads
        `False`.

        Each call asks the robot and waits for its answer. To wait for an input,
        call it in a loop.

        Args:
            pin: The input: `1`, `2` or `3`. A number with a fraction is cut to a
                whole number first.

        Returns:
            `True` while the input is on, `False` while it is off.

        Raises:
            CommandRejected: The robot refused it: `pin` is not 1, 2 or 3, or its
                control loop is not running.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The client is not connected.
            ValueError: `pin` cannot be read as a whole number, for example
                `"one"`. Nothing is sent.
            TypeError: `pin` is `None` or another type that is no number. Nothing
                is sent.

        Errors:
            BAD_ARGS: No pin was given (only a raw client can send that).
            BAD_PIN: `pin` is not 1, 2 or 3.
            RTI_NOT_RUNNING: The control loop (RTI) has not run for over 0.5 s, or
                the robot is in flashing mode: the level would be old.

        See also:
            `set_digital_output()`, `get_hardware()`

        Examples:
            >>> robot.get_digital_input(1)
            False
            >>> import time
            >>> deadline = time.monotonic() + 10
            >>> while not robot.get_digital_input(2) and time.monotonic() < deadline:
            ...     time.sleep(0.05)
        """
        return self._immediate("get_digital_input", [int(pin)])

    # ------------------------------------------------------------------ #
    # Context manager
    # ------------------------------------------------------------------ #

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.disconnect()
        return False
