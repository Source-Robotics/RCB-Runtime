"""RTIClient — PC-side real-time streaming control (TODO/RTI_mode_plan.md §5).

Two ways to drive it: a blocking `control(callback)` loop paced by the robot's
state packets, plus a non-blocking send-when-ready mode.

    from robotclient import RobotClient, RTIClient
    from robotclient import JointVelocityCommand, MotionFinished

    robot = RobotClient()
    robot.connect("192.168.0.100")
    robot.enable(); robot.wait_enabled()

    rti = RTIClient(robot)
    rti.connect()                      # TCP handshake: pairs, enters RTI mode
    rti.claim()                        # start RT control

    def controller(state):
        if state.elapsed > 5.0:
            return MotionFinished()
        return JointVelocityCommand([0, 0, 0, 0.5, 0, 0])

    rti.control(controller)            # blocks until MotionFinished
    rti.disconnect()

This module deliberately does NOT import anything from the robot-side packages
(`communication/`, `data/`, `utility/`). robotclient is meant to be installable on an
operator PC on its own, so the 101-byte packet layout is duplicated here. It is frozen
wire format — if you change it, change communication/rti_protocol.py in the same commit.
"""
from __future__ import annotations
from typing import Callable, Sequence

import select
import socket
import struct
import time

from .version import check_protocols   # stdlib only (versioning V3): the RTI packet + telemetry format check

# --- Command types. MUST match communication/rti_protocol.py ---------------
CMD_JOINT_POSITION = 0x01
CMD_JOINT_VELOCITY = 0x02
CMD_JOINT_TORQUE = 0x03
CMD_CARTESIAN_POSE = 0x04
CMD_CARTESIAN_VELOCITY = 0x05
CMD_STOP = 0xFF

CMD_FMT = '<I B 12d'
CMD_SIZE = struct.calcsize(CMD_FMT)   # 101 bytes
CMD_TS_SIZE = CMD_SIZE + 8            # + the optional send time (communication/rti_protocol.py)

# Defaults from config/networking.py. Robot listens for commands on RTI_PORT_RECEIVE;
# the PC listens for state on RTI_PORT_SENDER.
RTI_PORT_RECEIVE = 30001
RTI_PORT_SENDER = 30002
RTI_PORT_STREAM_TCP = 30003

# Recipe tags whose values are per-joint vectors rather than scalars are not
# enumerated here — the parser infers arity from how many values follow a tag.


def pack_command_packet(seq: int, cmd_type: int, payload: Sequence[float], feedforward: Sequence[float],
                        timestamp: float | None = None) -> bytes:
    """Serialise a command. payload/feedforward are 6 floats each (zero-padded). With `timestamp` (the send time in
    seconds of a monotonic clock) the packet is 109 bytes — see communication/rti_protocol.py."""
    p = list(payload) + [0.0] * (6 - len(payload))
    f = list(feedforward) + [0.0] * (6 - len(feedforward))
    data = struct.pack(CMD_FMT, seq & 0xFFFFFFFF, cmd_type, *p[:6], *f[:6])
    if timestamp is not None:
        data += struct.pack('<d', float(timestamp))
    return data


def parse_state_packet(text: str) -> dict:
    """Parse the robot's recipe payload into {tag: value or [values]}.

    Wire format (utility/serialization.py) is a flat comma-separated list where only
    the FIRST value of a vector carries its tag:

        j_p:0.12,1.45,-0.78,0.0,1.23,-0.56,j_s:0.01,-0.02,...,tick:12345

    So a chunk containing ':' opens a new tag and every following bare chunk belongs
    to it. Splitting on ',' alone (and requiring ':' per chunk) silently drops every
    joint array — that was the 2026-08-01 draft's bug.
    """
    out = {}
    tag = None
    for chunk in text.split(','):
        if not chunk:
            continue
        if ':' in chunk:
            tag, _, first = chunk.partition(':')
            tag = tag.strip()
            out[tag] = [_num(first)]
        elif tag is not None:
            out[tag].append(_num(chunk))
    # Collapse single-element lists to scalars so state["tick"] is a number.
    return {k: (v[0] if len(v) == 1 else v) for k, v in out.items()}


def _num(s):
    s = s.strip()
    try:
        return float(s)
    except ValueError:
        return s


class RobotState(dict):
    """One state packet from the robot, as telemetry tag to value.

    Each key is a tag of the active telemetry recipe (`set_recipe()`). A tag
    with one value maps to a number, a per-joint or pose tag to a list, `mode`
    to its text; values are rounded to the recipe's precision. The standard
    recipe, the default, carries:

    - `j_p`: the joint positions, in rad.
    - `j_s`: the joint speeds, unfiltered, in rad/s.
    - `j_err`: each drive's error byte; 0 when the drive reports no fault.
    - `tick`: the robot's loop tick counter.
    - `TCP_p`: the tool pose [x, y, z, rx, ry, rz]: metres, and radians with
      R = Rx(rx) Ry(ry) Rz(rz).
    - `rti_seq`: the sequence number of the last command the robot accepted;
      0 before the first.
    - `rti_state`: 0 not paired, 1 paired without a claim, 2 control active,
      3 stopping after a release or a stop, 4 stopping after an error.
    - `mode`: the robot's mode, such as `"RTI"`.
    - `period_ms`: the measured length of the tick that sent the packet, in
      ms.

    The `diagnostics` recipe adds `rti_ok`, the share of the last 100 ticks
    that brought a command (0 to 1), and `rti_miss`, the ticks in a row
    without one. `latest_status()["recipes"]` lists the recipes on the robot.

    Attributes:
        elapsed: Seconds since `control()` began: robot time (its tick count
            times its tick length) when `robot_time` is true, else this PC's
            clock. 0 outside `control()`.
        robot_time: Whether `elapsed` is robot time: the recipe carries `tick`
            and the robot reported its tick length.

    Examples:
        >>> from robotclient import RTIClient
        >>> rti = RTIClient(robot)
        >>> rti.connect()
        >>> robot.wait_mode("RTI")
        >>> state = rti.read_once()
        >>> state["mode"], state["rti_state"]
        ('RTI', 1.0)
        >>> robot.rti_disconnect(wait=True)
        >>> rti.close()
    """

    elapsed = 0.0
    robot_time = False


class _Command:
    cmd_type = None

    def __init__(self, payload, feedforward=None):
        self.payload = list(payload)
        self.feedforward = list(feedforward) if feedforward is not None else [0.0] * 6


class JointPositionCommand(_Command):
    """Joint position targets for one real-time command.

    Return it from a `control()` callback. It works in both real-time modes.

    Args:
        payload: The six joint targets in radians. Fewer values are padded with
            0 rad, which commands those joints to zero; more are cut.
        feedforward: Each joint's speed at the target, in rad/s; `None` sends
            zeros.

    Attributes:
        payload: The targets, as given.
        feedforward: The speeds, as given, or six zeros.

    Examples:
        >>> from robotclient import JointPositionCommand
        >>> cmd = JointPositionCommand([1.571, -1.833, 2.88, 0.0, -0.524, 3.142])
    """
    cmd_type = CMD_JOINT_POSITION


class JointVelocityCommand(_Command):
    """Joint velocities for one real-time command, RTI mode only.

    Return it from a `control()` callback. SOFT_RTI refuses it. The robot
    applies it every tick until the next command, for at most the command
    timeout.

    Args:
        payload: The six joint velocities in rad/s. Fewer values are padded
            with 0 rad/s; more are cut.
        feedforward: Torque in Nm added to each joint's command; `None` sends
            zeros.

    Attributes:
        payload: The velocities, as given.
        feedforward: The torques, as given, or six zeros.

    Examples:
        >>> from robotclient import JointVelocityCommand
        >>> cmd = JointVelocityCommand([0, 0, 0, 0, 0, 0.02])
    """
    cmd_type = CMD_JOINT_VELOCITY


class JointTorqueCommand(_Command):
    """Joint torques for one real-time command, RTI mode only.

    Return it from a `control()` callback. SOFT_RTI refuses it. Leave gravity
    out: the robot adds it. It applies the torque every tick until the next
    command, for at most the command timeout.

    Args:
        torques: The six joint torques in Nm. Fewer values are padded with
            0 Nm; more are cut.

    Attributes:
        payload: The torques, as given.
        feedforward: Six zeros.

    Examples:
        >>> from robotclient import JointTorqueCommand
        >>> cmd = JointTorqueCommand([0, 0, 0, 0, 0, 0])
    """
    cmd_type = CMD_JOINT_TORQUE

    def __init__(self, torques):
        super().__init__(torques)


# rti_state codes, mirroring motion/rti_handler.py::SUBSTATE_CODE.
RTI_UNPAIRED        = 0
RTI_CONNECTED       = 1
RTI_CONTROL_ACTIVE  = 2
RTI_STOPPING_CLEAN  = 3
RTI_STOPPING_ERROR  = 4

_RTI_STATE_NAME = {0: "unpaired", 1: "connected (control NOT claimed)",
                   2: "control active", 3: "stopping (clean release)",
                   4: "stopping (error latched)"}


class ControlLost(RuntimeError):
    """Real-time control that the robot ended while this PC streamed.

    `control()` raises it when a state packet shows the claim revoked
    (`rti_state` neither 2 nor a clean release), and when the state stream
    stops after it had started: a latched error takes the robot out of its
    real-time mode, which ends the stream. Any read or send raises it when the
    robot closes the TCP stream. The message names the robot's errors and the
    link figures this client has; `robot.errors()` gives the robot's errors in
    full.

    Examples:
        >>> from robotclient import RTIClient, ControlLost, MotionFinished
        >>> rti = RTIClient(robot)
        >>> rti.connect()
        >>> robot.wait_mode("RTI")
        >>> rti.claim()
        >>> try:
        ...     rti.control(lambda state: MotionFinished())
        ... except ControlLost:
        ...     print(robot.errors())
        ...
        >>> robot.rti_disconnect(wait=True)
        >>> rti.close()
    """


class MotionFinished:
    """Signal from a `control()` callback that the motion is done.

    `control()` then sends the stop command and returns. The arm brakes to
    rest within its limits and holds, and after 0.5 s the session is paired
    and idle again, in RTI and SOFT_RTI mode alike.

    Args:
        cmd: Not used: `control()` sends the stop only.

    Attributes:
        cmd: As given.

    Examples:
        >>> from robotclient import MotionFinished
        >>> def callback(state):
        ...     return MotionFinished()
        ...
    """

    def __init__(self, cmd=None):
        self.cmd = cmd


class RTIClient:
    """Real-time streaming control of one robot, beside its `RobotClient`.

    It carries the real-time link: commands from your PC to the robot's UDP
    port 30001 and state packets from the robot to your PC's UDP port 30002,
    or both over one TCP stream to the robot's port 30003. The `RobotClient`
    it is given carries the handshake: pairing, claiming, releasing. Joint
    values are in radians here, unlike the rest of the client.

    In RTI mode (hard real time, `connect()` without a rate) your PC sends one
    target every 4 ms tick over UDP, each with its send time, and closes the
    loop itself: the robot replays your position targets one tick behind and
    refuses a stream the arm cannot follow. In SOFT_RTI (`connect()` with
    `rate_hz`) you send joint position targets at your own rate, over UDP or
    TCP, and the robot fills in the ticks between them.

    When commands stop coming or a stream breaks a limit, the arm brakes to
    rest within its limits, then the robot latches an error and disables:
    `RTI_LINK_LOST`, `RTI_START_POSE` (a first command too far from the arm),
    `RTI_NO_SEND_TIME`, or per joint `RTI_POSITION_LIMIT`,
    `RTI_VELOCITY_LIMIT`, `RTI_ACCELERATION_LIMIT` or `RTI_JERK_LIMIT`. Its
    state stream then ends, and `control()` raises `ControlLost`.

    Attributes:
        robot: The `RobotClient` that carries the handshake.
        clock: `"robot"` or `"pc"`, as given.
        transport: `"udp"` or `"tcp"`; `connect()` can change it.
        timestamps: Whether UDP commands carry their send time.
        seq: The sequence number of the last command sent; the first is 1.
        robot_ip: The robot's address, set by `connect()`; `None` before.
        tick_s: The robot's tick length in seconds, read at `connect()`;
            `None` when the robot does not report it.
        captured: What `capture()` recorded: (arrival time, packet) pairs.
        rtt_samples_ms: Every loop delay measured, in milliseconds.
        lost_seqs: Commands sent that the robot never echoed back.
        bytes_tx: Bytes of commands sent.
        bytes_rx: Bytes of state received.

    Examples:
        >>> import math
        >>> from robotclient import RTIClient
        >>> from robotclient import JointPositionCommand, MotionFinished
        >>> rti = RTIClient(robot)
        >>> rti.connect()
        >>> robot.wait_mode("RTI")
        >>> q0 = rti.read_once()["j_p"]
        >>> rti.claim()
        >>> def follow(state):
        ...     t = state.elapsed
        ...     if t > 2.0:
        ...         return MotionFinished()
        ...     q = list(q0)
        ...     q[5] += 0.0125 * (1 - math.cos(math.pi * t / 2.0))
        ...     return JointPositionCommand(q)
        ...
        >>> rti.control(follow)
        >>> robot.rti_disconnect(wait=True)
        >>> rti.close()
    """

    def __init__(self, robot, clock='robot', listen_port=RTI_PORT_SENDER, transport='udp', timestamps=True):
        """Create the real-time link for a connected `RobotClient`.

        It opens two UDP sockets and, for UDP, binds the state socket to
        `listen_port` on every interface at once. Nothing is sent until
        `connect()`. Call `close()` when done.

        Args:
            robot: A connected `RobotClient`; it carries the handshake.
            clock: `"robot"`: `control()` runs your callback on each state packet
                the robot sends. `"pc"`: you send when you like, with
                `send_joint_position()` and the other send calls; `control()` is not
                available. The robot behaves the same either way.
            listen_port: The UDP port state packets arrive on. The robot always
                sends to 30002, so keep the default. Not used with TCP.
            transport: `"udp"` or `"tcp"`. TCP carries both directions on one
                stream that this PC dials, in SOFT_RTI only: a lost packet then
                stalls everything behind it until it is sent again, while a closed
                stream is noticed at once.
            timestamps: Whether every UDP command carries its send time, so that the
                robot follows your real timing instead of the packets' arrival. RTI
                mode needs it; `False` is for SOFT_RTI only. TCP never carries it.

        Raises:
            ValueError: `clock` or `transport` is not one of its values.
            OSError: The UDP port `listen_port` is not free.

        Examples:
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.close()
        """
        if clock not in ('robot', 'pc'):
            raise ValueError("clock must be 'robot' or 'pc'")
        if transport not in ('udp', 'tcp'):
            raise ValueError("transport must be 'udp' or 'tcp'")
        self.robot = robot
        self.clock = clock
        self.transport = transport
        self.timestamps = bool(timestamps)
        self.seq = 0
        self.robot_ip = None
        self.tick_s = None            # the robot's tick length (s), from its status at connect()

        # Optional full capture. read_state() deliberately drains to NEWEST and discards the
        # rest, which is right for a controller and wrong for a measurement: off-robot there
        # is no /dev/shm, so the stream is the only source of truth and every packet counts.
        # Set capture(True) to have each drained packet appended here instead of dropped.
        self.captured = []
        self._capturing = False

        self._tcp = None              # the one stream socket, when transport='tcp'
        self._rxbuf = bytearray()     # partial state frame carried between reads
        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if transport == 'udp':
            self._recv_sock.bind(('0.0.0.0', listen_port))
        self._running = False

        # --- loop-delay measurement (needs the `rti_seq` tag in the active recipe) ---
        # Send time per outstanding sequence number. The robot echoes the last seq it
        # ACCEPTED, so matching the echo against these gives round-trip LOOP DELAY —
        # "how long until my command's effect is visible to me". That is the number that
        # governs loop stability; wire latency alone would not tell you.
        #
        # Bounded on purpose: seq is monotonic, so anything at or below an echoed value is
        # dead weight and gets pruned. At 250 Hz an unbounded dict grows by 250 entries a
        # second — harmless for a minute, a leak over an hour.
        self._t_send = {}
        self.rtt_samples_ms = []
        self.lost_seqs = 0            # sent, then skipped over by a later echo
        self._last_echo = None

        # --- link bandwidth, measured at THIS end ---
        # The robot publishes its own byte counters to shared memory, but a REMOTE PC
        # cannot read those, so count here too and a network run still gets numbers.
        self.bytes_tx = 0
        self.bytes_rx = 0

        # Robot-side link health, last seen. Only populated when the recipe carries
        # rti_ok / rti_miss (diagnostics does; standard does not) — used to enrich a
        # ControlLost message rather than to make decisions.
        self._last_ok = None
        self._last_miss = None
        self._had_control = False

    # ------------------------------------------------------------------ #
    # Lifecycle (TCP)
    # ------------------------------------------------------------------ #

    def connect(self, pc_ip: str | None = None, rate_hz: float | None = None, transport: str | None = None, *,
                allow_mismatch: bool = False) -> RTIClient:
        """Pair with the robot and switch it to a real-time mode.

        It checks that the robot speaks this client's real-time packet and
        telemetry formats, then calls `robot.rti_connect()` with this PC's address,
        and over TCP dials the robot's stream port. It returns once the robot has
        answered: unless the `RobotClient` runs in sync mode, the robot switches a
        few milliseconds later, so wait for the mode (`robot.wait_mode()`) before
        `claim()`.

        `rate_hz` picks the contract: none for RTI mode, your command rate for
        SOFT_RTI (see `RobotClient.rti_connect()`).

        Args:
            pc_ip: The address the robot sends state to and accepts commands from.
                `None` uses this PC's address on its connection to the robot.
            rate_hz: Your command rate in Hz for SOFT_RTI; `None` for RTI mode.
                Prefer one that divides 250 Hz evenly: 125, 50, 25 or 10 Hz.
            transport: `"udp"` or `"tcp"`, replacing the one given at creation;
                `None` keeps it. RTI mode is UDP only.
            allow_mismatch: Pair even when the robot reports another real-time
                packet or telemetry format version.

        Returns:
            This `RTIClient`, so that calls can be chained.

        Raises:
            CommandRejected: The robot refused the pairing: it is not enabled or
                not homed, or another client holds control.
            RobotTimeout: The robot did not answer within 5 s, or, in sync mode,
                did not report the mode within 10 s.
            RobotConnectionError: The robot connection was lost.
            VersionMismatch: The robot reports another real-time packet or
                telemetry format version, and `allow_mismatch` is false. Nothing is
                sent.
            ValueError: Without `rate_hz`, `transport` is `"tcp"` or the client
                sends no send times; or `transport` or `rate_hz` is not valid.
                Nothing is sent.
            TypeError: `rate_hz` is not a number. Nothing is sent.
            RuntimeError: `robot` is not connected. Nothing is sent.
            OSError: Over TCP, the robot's stream port did not accept the
                connection within 5 s.

        Examples:
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.connect(rate_hz=50)
            >>> robot.wait_mode("SOFT_RTI")
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        try:
            st = self.robot.latest_status() or {}
        except Exception:                    # noqa: BLE001 — a robot object without status(): nothing to check
            st = {}
        if isinstance(st.get("protocols"), dict):
            check_protocols(st, ("rti", "telemetry"), allow_mismatch)
        if rate_hz is None:
            if (transport or self.transport) == 'tcp':
                raise ValueError("hard RTI is UDP only (TCP stalls the stream behind any lost packet); for TCP give "
                                 "a rate_hz — SOFT_RTI")
            if not self.timestamps:
                raise ValueError("hard RTI needs the send time on every position target (the robot refuses one "
                                 "without it): RTIClient(timestamps=True), the default — or SOFT_RTI (rate_hz) "
                                 "for a sender without send times")
        self.robot_ip = self.robot.remote_ip()
        if self.robot_ip is None:
            raise RuntimeError("robot is not connected — call robot.connect() first")
        if pc_ip is None:
            pc_ip = self.robot.local_ip()
        if transport is not None:
            if transport not in ('udp', 'tcp'):
                raise ValueError("transport must be 'udp' or 'tcp'")
            self.transport = transport
        self.robot.rti_connect(pc_ip, rate_hz=rate_hz, transport=self.transport)
        # The robot's tick length, for control()'s robot time (None: an executor that does not publish it).
        try:
            ts_ = float(self.robot.latest_status().get("tick_s") or 0.0)
        except (TypeError, ValueError, AttributeError):
            ts_ = 0.0
        self.tick_s = ts_ if 0.0 < ts_ < 1.0 else None
        # Dial AFTER the robot has been told to expect us: it only accepts a stream
        # connection from the paired IP, and it only starts listening for one once
        # rti_connect has named that IP.
        if self.transport == 'tcp':
            self._tcp_connect()
        return self

    def claim(self) -> RTIClient:
        """Claim real-time control: the robot starts applying your commands.

        It calls `robot.rti_claim()`: see there. Send the first command at once,
        because the watchdog runs from the claim: 40 ms in RTI mode, three of your
        intervals in SOFT_RTI.

        Returns:
            This `RTIClient`, so that calls can be chained.

        Raises:
            CommandRejected: The robot refused it: it is not in a real-time mode
                (for example right after `connect()`, before the switch), not
                enabled or not homed, or another client holds control.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The robot connection is not open.

        Warning:
            From the claim on, the arm follows your PC's commands.

        Examples:
            >>> from robotclient import RTIClient, MotionFinished
            >>> rti = RTIClient(robot)
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> rti.claim()
            >>> rti.control(lambda state: MotionFinished())
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        self.robot.rti_claim()
        return self

    def release(self) -> None:
        """Release real-time control over the robot connection, staying paired.

        It calls `robot.rti_release()`: the arm brakes to rest within its limits
        and holds, and after 0.5 s the session is paired and idle again. It works
        in both real-time modes, also when the stream itself is down. Send it
        within the command timeout after your last command, or the robot latches
        `RTI_LINK_LOST` first.

        Raises:
            CommandRejected: Another client holds control.
            RobotTimeout: The robot did not answer within 5 s.
            RobotConnectionError: The robot connection is not open.

        Examples:
            >>> import time
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot, clock="pc")
            >>> rti.connect(rate_hz=50)
            >>> robot.wait_mode("SOFT_RTI")
            >>> q = rti.read_once()["j_p"]
            >>> rti.claim()
            >>> for i in range(50):
            ...     rti.send_joint_position(q)
            ...     time.sleep(0.02)
            ...
            >>> rti.release()
            >>> time.sleep(1)
            >>> rti.disconnect()
            >>> rti.close()
        """
        self.robot.rti_release()

    def disconnect(self) -> None:
        """Unpair from the robot and switch it to IDLE mode.

        It ends a running `control()` loop and calls `robot.rti_disconnect()`,
        which switches to IDLE mode at once, without braking: release first and
        let the arm come to rest. It waits for IDLE mode only when the
        `RobotClient` runs in sync mode. The sockets stay open: `close()` them, or
        `connect()` again.

        Raises:
            CommandRejected: Another client holds control.
            RobotTimeout: The robot did not answer within 5 s, or, in sync mode,
                did not report IDLE mode within 10 s.
            RobotConnectionError: The robot connection is not open.

        Warning:
            The arm does not brake: sent while it moves, it coasts in freedrive
            until friction stops it (see `robot.rti_disconnect()`).

        Examples:
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> rti.disconnect()
            >>> robot.wait_mode("IDLE")
            >>> rti.close()
        """
        self._running = False
        self.robot.rti_disconnect()

    def close(self) -> None:
        """Close the link's sockets.

        It closes the TCP stream and both UDP sockets; the robot is not told.
        Unpair first with `disconnect()`. The client cannot be used afterwards.

        Examples:
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.close()
        """
        if self._tcp is not None:
            try:
                self._tcp.close()
            except OSError:
                pass
            self._tcp = None
        self._send_sock.close()
        self._recv_sock.close()

    # ------------------------------------------------------------------ #
    # Control
    # ------------------------------------------------------------------ #

    def control(self, callback: Callable[[RobotState], _Command | MotionFinished | None], first_packet_timeout: float = 2.0, timeout: float = 0.1) -> None:
        """Run your control callback on every state packet the robot sends.

        It blocks. For each state packet it calls `callback(state)` with a
        `RobotState` and sends the command the callback returns at once, stamped
        with `state.elapsed` when that is robot time. Return `None` to send
        nothing that tick: the robot carries on alone (a position stream along
        the same curve, a velocity or a torque unchanged) until its command
        timeout. Return `MotionFinished` to send the stop and return.

        It needs an active claim (`claim()`): without one the robot ignores the
        commands, and nothing reports it. When the recipe carries `rti_state`, it
        returns quietly once the robot ends the claim cleanly, as `rti_release()`
        does.

        Args:
            callback: Called with each `RobotState`. It returns a
                `JointPositionCommand`, `JointVelocityCommand` or
                `JointTorqueCommand` to send, `MotionFinished` to end, or `None`.
            first_packet_timeout: Seconds to wait for the first state packet.
            timeout: Seconds to wait for each later packet. A longer silence means
                the robot has left real-time control.

        Raises:
            ControlLost: The robot revoked the claim (`rti_state` left 2 other than
                by a clean release), the state stream stopped after it had started,
                or the robot closed the TCP stream.
            RuntimeError: `clock` is not `"robot"`, or no state packet came within
                `first_packet_timeout`, for example before `connect()`.

        Note:
            Over UDP the packets that arrive while your callback runs queue up, and
            the next one handed to it is the oldest of them. Keep the callback
            shorter than a tick, or it falls further behind with every packet.

        Warning:
            Your callback moves the arm. Start from the measured pose
            (`state["j_p"]`) and keep the first motions small.

        Examples:
            >>> import math
            >>> from robotclient import RTIClient
            >>> from robotclient import JointPositionCommand, MotionFinished
            >>> rti = RTIClient(robot)
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> q0 = rti.read_once()["j_p"]
            >>> rti.claim()
            >>> def follow(state):
            ...     t = state.elapsed
            ...     if t > 2.0:
            ...         return MotionFinished()
            ...     q = list(q0)
            ...     q[5] += 0.0125 * (1 - math.cos(math.pi * t / 2.0))
            ...     return JointPositionCommand(q)
            ...
            >>> rti.control(follow)
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        if self.clock != 'robot':
            raise RuntimeError("control() is robot-clocked; use send_*() with clock='pc'")

        self._running = True
        _to = [first_packet_timeout]
        started = time.monotonic()
        tick0 = None
        received = False             # a packet seen: a stop after this is a LOST stream, not one that never started
        try:
            while self._running:
                try:
                    data = self._raw_recv(_to[0])
                except socket.timeout:
                    # Losing the state stream IS losing control, so raise ControlLost — not a
                    # generic error the caller has to guess at. This is the NORMAL path when
                    # the robot's own watchdog trips: latching a hard error forces
                    # ACTIVE_ERROR, which takes the robot out of RTI mode, which stops
                    # step() and therefore stops the stream. The PC never gets to read
                    # rti_state=4 because the packet carrying it is never sent. The reason
                    # lives on the robot at that point, so point the caller at it.
                    if getattr(self, "_had_control", False):
                        raise ControlLost(
                            "state stream stopped while we held control — the robot has left "
                            "RTI mode. A latched error forces ACTIVE_ERROR, which ends the "
                            "stream; check robot.latest_status() and the error list for the cause."
                            + self._link_summary())
                    if received:
                        # 2026-10-13: a recipe without rti_state (e.g. `tracking`) cannot tell us whether we held
                        # control — but packets came and then stopped, so the robot left RTI mode. Reporting "never
                        # started streaming" here sent the operator looking at the pairing instead.
                        raise ControlLost(
                            "state stream stopped after it had started — the robot has left RTI mode (a latched "
                            "error forces ACTIVE_ERROR, which ends the stream; the active recipe carries no "
                            "rti_state, so check robot.latest_status() and the error list for the cause)."
                            + self._link_summary())
                    raise RuntimeError("RTI timeout: no state packet from robot "
                                       "(never started streaming — is it paired?)")

                state = self._decode(data)
                received = True
                tick = state.get("tick")
                if self.tick_s is not None and isinstance(tick, (int, float)):
                    if tick0 is None:
                        tick0 = tick
                    state.elapsed = (tick - tick0) * self.tick_s
                    state.robot_time = True
                else:
                    state.elapsed = time.monotonic() - started

                # Did the robot revoke control? Needs `rti_state` in the active recipe;
                # absent, this is a no-op and the old blind behaviour applies.
                self._check_control_alive(state)

                result = callback(state)

                if isinstance(result, MotionFinished):
                    self._send(CMD_STOP, [0.0] * 6, [0.0] * 6, state.elapsed if state.robot_time else None)
                    break
                if result is not None:
                    # Stamped with the time the target was computed FOR (robot time when known): the
                    # robot then re-times it exactly, whenever it arrives.
                    self._send(result.cmd_type, result.payload, result.feedforward,
                               state.elapsed if state.robot_time else None)

                _to[0] = timeout        # first packet may be slow; the rest must not be
        finally:
            self._running = False

    def _check_control_alive(self, state):
        """Raise ControlLost if the robot is no longer applying our commands.

        The failure this exists to catch: a timeout or a rejected start pose drops the
        claim, the robot returns to `connected` and IGNORES every command from then on —
        while still streaming state packets that look completely normal. Without this the
        PC never finds out.
        """
        st = state.get("rti_state")
        if st is None:
            return                       # recipe carries no rti_state — cannot tell
        st = int(st)

        if st == RTI_CONTROL_ACTIVE:
            self._had_control = True
            return
        if not getattr(self, "_had_control", False):
            return                       # never had it; claim() may not have landed yet

        if st == RTI_STOPPING_CLEAN:
            self._running = False        # robot is releasing on purpose — end quietly
            return
        raise ControlLost(
            f"robot revoked control: rti_state={st} ({_RTI_STATE_NAME.get(st, '?')})"
            f"{self._link_summary()}")

    def _robot_error_summary(self):
        """The robot's own reason for stopping, over TCP.

        The UDP stream is gone by the time we need this — a latched error forces
        ACTIVE_ERROR, which ends RTI mode and therefore the stream. TCP survives that, so
        it is the only channel that can still say WHY. Without it every cause reads
        identically as "the robot stopped talking", and an E-stop is indistinguishable
        from a network fault.
        """
        try:
            st = self.robot.latest_status() or {}
        except Exception:
            return ""
        keys = st.get("errors") or []
        if keys:
            return f"robot errors: {', '.join(keys)}"
        if st.get("err"):
            return "robot reports an error but sent no keys (old command_executor?)"
        if st.get("mode") and st.get("mode") != "RTI":
            return f"robot left RTI mode (now {st['mode']}) with no error — released elsewhere?"
        return ""

    def _link_summary(self):
        """Metrics appended to a ControlLost message, so the cause is visible at once.

        Success rate and packets-lost-in-a-row are the two numbers that tell you WHY a
        link died. Robot-side figures need rti_ok / rti_miss in the recipe; the PC-side
        ones are always available.
        """
        bits = []
        cause = self._robot_error_summary()
        if cause:
            bits.append(cause)
        lat = self.latency_stats()
        if lat:
            bits.append(f"loop delay med {lat['med']:.1f} ms")
            bits.append(f"{lat['lost']} commands never echoed")
        if self._last_ok is not None:
            bits.append(f"robot success rate {self._last_ok * 100:.1f}%")
        if self._last_miss is not None:
            bits.append(f"{int(self._last_miss)} consecutive misses")
        return ("  [" + ", ".join(bits) + "]") if bits else ""

    def send_joint_position(self, positions: Sequence[float], feedforward_speed: Sequence[float] | None = None,
                            timestamp: float | None = None) -> None:
        """Send one joint position target to the robot.

        It sends the target at once and does not wait. The robot applies it only
        while control is claimed: in RTI mode send one every 4 ms tick, each with
        its send time; in SOFT_RTI at your own rate. A target beyond a joint's soft
        limit, or heading for it faster than the joint can still stop, ends the
        stream with `RTI_POSITION_LIMIT` after the arm has braked to rest. Over TCP
        it first reads the state that is waiting, so that the stream does not back
        up.

        Args:
            positions: The six joint targets in radians. Fewer values are padded
                with 0 rad, which commands those joints to zero; more are cut.
            feedforward_speed: Each joint's speed at the target, in rad/s; `None`
                sends zeros. `"ruckig"` and `"ruckig_advancing"` pass through the
                target at this speed, and the dynamics feedforward reads your
                stream's speed from it.
            timestamp: The time this target is for, in seconds of a monotonic
                clock, the same clock for every target; used over UDP with
                `timestamps` on only. `None` stamps the moment of sending. Better:
                read the clock once, compute the target from that reading and pass
                it, so that a pause between computing and sending does not look like
                a jump.

        Raises:
            ControlLost: Over TCP, the robot closed the stream.
            RuntimeError: `connect()` was not called.
            OSError: The packet could not be sent.

        Examples:
            >>> import time
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot, clock="pc")
            >>> rti.connect(rate_hz=50)
            >>> robot.wait_mode("SOFT_RTI")
            >>> q = rti.read_once()["j_p"]
            >>> rti.claim()
            >>> for i in range(50):
            ...     q[5] += 0.0005
            ...     rti.send_joint_position(q)
            ...     time.sleep(0.02)
            ...
            >>> rti.release()
            >>> time.sleep(1)
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        self._send(CMD_JOINT_POSITION, positions, feedforward_speed or [0.0] * 6, timestamp)

    def send_joint_velocity(self, velocities: Sequence[float], feedforward_torque: Sequence[float] | None = None) -> None:
        """Send one joint velocity command to the robot.

        RTI mode only: SOFT_RTI refuses velocity commands, and a session that
        sends only those ends with `RTI_LINK_LOST`. The robot applies the velocity
        every tick until the next command, for at most the command timeout
        (40 ms), so send one every tick. With the rate limiter on it is limited in
        speed, acceleration and jerk; the tool speed cap and the soft joint limits
        always apply (a joint brakes before its limit). The first one after a
        claim must be small (`rti_set_start_pose_check()`).

        Args:
            velocities: The six joint velocities in rad/s. Fewer values are padded
                with 0 rad/s; more are cut.
            feedforward_torque: Torque in Nm added to each joint's command, limited
                in its rate of change like a torque command; `None` sends zeros.

        Raises:
            ControlLost: Over TCP, the robot closed the stream.
            RuntimeError: `connect()` was not called.
            OSError: The packet could not be sent.

        Warning:
            A velocity command keeps the arm moving until the next command or the
            command timeout. Send zeros, or `stop()`, to stop.

        Examples:
            >>> import time
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot, clock="pc")
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> rti.claim()
            >>> for i in range(250):
            ...     rti.send_joint_velocity([0, 0, 0, 0, 0, 0.02])
            ...     time.sleep(0.004)
            ...
            >>> rti.stop()
            >>> time.sleep(1)
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        self._send(CMD_JOINT_VELOCITY, velocities, feedforward_torque or [0.0] * 6)

    def send_joint_torque(self, torques: Sequence[float]) -> None:
        """Send one joint torque command to the robot.

        RTI mode only, like velocity commands. The robot adds the torque that
        holds the arm up against gravity (with both gravity switches on), limits
        the rate of change while the rate limiter is on, and cuts torque that
        pushes a joint further past its soft limit; the drives limit the current.
        It applies the torque every tick until the next command, for at most the
        command timeout (40 ms). The first one after a claim must be small
        (`rti_set_start_pose_check()`).

        Args:
            torques: The six joint torques in Nm, without gravity. Fewer values are
                padded with 0 Nm; more are cut.

        Raises:
            ControlLost: Over TCP, the robot closed the stream.
            RuntimeError: `connect()` was not called.
            OSError: The packet could not be sent.

        Warning:
            Under torque control nothing holds a position: with zero torque the arm
            floats as in freedrive, and an error in the gravity model lets it
            drift. Keep a hand near the E-stop.

        Examples:
            >>> import time
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot, clock="pc")
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> rti.claim()
            >>> for i in range(50):
            ...     rti.send_joint_torque([0, 0, 0, 0, 0, 0])
            ...     time.sleep(0.004)
            ...
            >>> rti.stop()
            >>> time.sleep(1)
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        self._send(CMD_JOINT_TORQUE, torques, [0.0] * 6)

    def stop(self) -> None:
        """Ask the robot to end the stream: brake to rest and hold.

        It sends the stop command inside the stream and does not wait; it also
        ends a `control()` loop running in another thread. The robot then does
        what `rti_release()` does, in RTI and SOFT_RTI mode alike: the arm brakes
        to rest within its limits and holds, and after 0.5 s the session is paired
        and idle again.

        Raises:
            ControlLost: Over TCP, the robot closed the stream.
            RuntimeError: `connect()` was not called.
            OSError: The packet could not be sent.

        Examples:
            >>> import time
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot, clock="pc")
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> rti.claim()
            >>> for i in range(250):
            ...     rti.send_joint_velocity([0, 0, 0, 0, 0, 0.02])
            ...     time.sleep(0.004)
            ...
            >>> rti.stop()
            >>> time.sleep(1)
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        self._send(CMD_STOP, [0.0] * 6, [0.0] * 6)
        self._running = False

    # ------------------------------------------------------------------ #
    # State reads
    # ------------------------------------------------------------------ #

    def read_state(self) -> RobotState | None:
        """Return the newest state packet that has arrived, or `None`.

        It never blocks: it reads every packet waiting, keeps the newest and
        discards the others (`capture()` records them). It works while merely
        paired, without a claim.

        Returns:
            The newest `RobotState`, or `None` when no packet arrived since the
            last read. Its `elapsed` is 0: only `control()` sets it.

        Raises:
            ControlLost: Over TCP, the robot closed the stream.

        Examples:
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> state = rti.read_state()
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        latest = self._raw_drain()
        return self._decode(latest) if latest else None

    def read_once(self, timeout: float = 1.0) -> RobotState:
        """Return the current state packet, waiting for one if none is queued.

        It reads every packet waiting and returns the newest; only when none is
        waiting does it block, up to `timeout`. It works while merely paired,
        without a claim: read the arm's pose from it before streaming. The oldest
        packet would show the pose of up to the whole wait before.

        Args:
            timeout: Seconds to wait when no packet is waiting.

        Returns:
            The newest `RobotState`. Its `elapsed` is 0: only `control()` sets it.

        Raises:
            ControlLost: Over TCP, the robot closed the stream.
            RuntimeError: No packet arrived within `timeout`.

        Examples:
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.connect(rate_hz=50)
            >>> robot.wait_mode("SOFT_RTI")
            >>> rti.read_once()["j_p"]
            [1.571, -1.833, 2.88, 0.0, -0.524, 3.142]
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        latest = self._raw_drain()
        if latest is not None:
            return self._decode(latest)
        try:
            data = self._raw_recv(timeout)
        except socket.timeout:
            raise RuntimeError("RTI timeout: no state packet from robot")
        return self._decode(data)

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    # --- transport ------------------------------------------------------------------
    # Everything above this line talks in packets. These four are the only places that know
    # whether those packets ride datagrams or a byte stream. _raw_recv raises socket.timeout
    # on both paths so the existing ControlLost handling in control() works unchanged.

    def _tcp_connect(self, timeout=5.0):
        """Dial the robot's stream port. Called by connect() once the robot is expecting us."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((self.robot_ip, RTI_PORT_STREAM_TCP))
        # MANDATORY. Our command packet is 101 bytes; with Nagle enabled the kernel would
        # hold it back waiting for more to coalesce, adding tens of milliseconds to a
        # real-time stream. This single line is the difference between TCP being usable
        # here and not.
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._tcp = sock
        self._rxbuf = bytearray()

    def _tcp_pump(self):
        """Read whatever is waiting into _rxbuf and cap it. TCP only; never blocks.

        THE TRAP THIS EXISTS FOR. Over UDP a caller may ignore the robot's state stream
        entirely — unread datagrams are dropped by the kernel and nothing accumulates. Over
        TCP that same code KILLS THE LINK: unread bytes become backpressure, the robot's
        send buffer fills, its backlog passes the cap and it drops the session. A PC that
        only ever calls send_joint_position() in a loop is a perfectly reasonable open-loop
        controller and must not be punished for the transport it picked.

        So every send drains the socket first. The newest complete frame is left IN the
        buffer, so read_state() still finds it — this keeps the link alive without throwing
        away data the caller might still want.
        """
        if self.transport != 'tcp' or self._tcp is None:
            return
        reads = 0
        while reads < 64 and select.select([self._tcp], [], [], 0)[0]:
            chunk = self._tcp.recv(65536)
            if not chunk:
                raise ControlLost("robot closed the RTI TCP stream" + self._link_summary())
            self._rxbuf.extend(chunk)
            reads += 1
        # Keep only the newest complete frame, in place. Scanning from the front is the only
        # way to find frame boundaries in a length-prefixed stream.
        # Not while capturing: the trim below discards every frame but the newest, which is
        # exactly what a capture must not do. A capturing caller reads every iteration, so
        # the buffer stays small; the trim exists for callers that never read at all.
        if len(self._rxbuf) > 65536 and not getattr(self, '_capturing', False):
            keep, i, n = None, 0, len(self._rxbuf)
            while n - i >= 4:
                ln = int.from_bytes(self._rxbuf[i:i + 4], "big")
                if n - i - 4 < ln:
                    break
                keep = (i, i + 4 + ln)
                i += 4 + ln
            if keep is not None:
                tail = bytes(self._rxbuf[keep[1]:])
                frame = bytes(self._rxbuf[keep[0]:keep[1]])
                del self._rxbuf[:]
                self._rxbuf.extend(frame)
                self._rxbuf.extend(tail)

    def _tcp_take_frame(self):
        """Pop the NEWEST complete length-prefixed frame from _rxbuf, or None.

        Newest, not oldest, for the same reason read_once() drains: no caller ever wants a
        stale state packet. Whole frames ahead of it are discarded; the partial tail stays.
        """
        latest, i, n = None, 0, len(self._rxbuf)
        while n - i >= 4:
            ln = int.from_bytes(self._rxbuf[i:i + 4], "big")
            if n - i - 4 < ln:
                break                      # frame not fully arrived yet
            latest = bytes(self._rxbuf[i + 4:i + 4 + ln])
            i += 4 + ln
        if i:
            del self._rxbuf[:i]
        return latest

    def _tcp_take_frames(self):
        """Pop EVERY complete length-prefixed frame from _rxbuf, oldest first.

        _tcp_take_frame() deliberately keeps only the newest — right for control, where a
        stale state is worthless — and WRONG for capture, whose whole job is to record each
        packet. Capturing through _tcp_take_frame() silently kept about one packet in five
        (whatever arrived between two drains), and because the tick counter is what loss is
        measured from, the missing four looked like 80% packet loss on a perfect link.
        """
        frames, i, n = [], 0, len(self._rxbuf)
        while n - i >= 4:
            ln = int.from_bytes(self._rxbuf[i:i + 4], "big")
            if n - i - 4 < ln:
                break                      # frame not fully arrived yet
            frames.append(bytes(self._rxbuf[i + 4:i + 4 + ln]))
            i += 4 + ln
        if i:
            del self._rxbuf[:i]
        return frames

    def _raw_send(self, pkt):
        if self.transport == 'tcp':
            self._tcp_pump()          # see _tcp_pump: not optional on this transport
            self._tcp.sendall(pkt)
        else:
            self._send_sock.sendto(pkt, (self.robot_ip, RTI_PORT_RECEIVE))

    def capture(self, on: bool = True, limit: int = 200000) -> RTIClient:
        """Record every state packet that `read_state()` and `read_once()` read.

        Normally a read keeps only the newest packet. With capture on, each packet
        they read is appended to `captured` as an (arrival time, packet) pair: the
        time in seconds of this PC's `time.perf_counter()`, the packet as the raw
        bytes the robot sent. Packets that `control()` receives are not recorded.
        Switching it on empties `captured`; past `limit` entries the oldest
        quarter is dropped.

        Args:
            on: `True` to record, `False` to stop; what was recorded stays.
            limit: The most entries kept.

        Returns:
            This `RTIClient`, so that calls can be chained.

        Examples:
            >>> import time
            >>> from robotclient import RTIClient
            >>> rti = RTIClient(robot)
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> rti.capture(True)
            >>> time.sleep(0.1)
            >>> state = rti.read_state()
            >>> len(rti.captured) > 1
            True
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        self._capturing = bool(on)
        self._cap_limit = limit
        if on:
            self.captured = []
        return self

    def _cap(self, data):
        # (arrival time, payload). perf_counter, NOT monotonic: on Windows (Python <= 3.12)
        # monotonic() only advances every 15.6 ms, which put five packets on one stamp and
        # made the analysis blame the arm. Same clock as the suite's sender (_ext_common.now).
        # The timestamp is the PC's own clock on purpose:
        # the analytic path a test compares against is defined on THIS machine's clock, and
        # the robot's tick counter cannot be converted to it without clock sync. Network
        # latency therefore enters the measurement -- ~0.1-1 ms on a LAN, which at 15 deg/s
        # is under 0.02 deg and far below the effects being measured.
        if self._capturing and data is not None:
            self.captured.append((time.perf_counter(), data))
            if len(self.captured) > getattr(self, "_cap_limit", 200000):
                del self.captured[:len(self.captured) // 4]

    def _raw_drain(self):
        """Newest payload available right now, or None. Never blocks."""
        if self.transport == 'tcp':
            while select.select([self._tcp], [], [], 0)[0]:
                chunk = self._tcp.recv(65536)
                if not chunk:
                    raise ControlLost("robot closed the RTI TCP stream"
                                      + self._link_summary())
                self._rxbuf.extend(chunk)
            if self._capturing:
                # Record every frame, return the newest ONE PAYLOAD from THIS drain — the same
                # contract as the UDP path, where nothing new means None. (This used to return
                # captured[-1], which is a (time, payload) TUPLE: read_state() then crashed in
                # _decode, and when nothing new had arrived it re-served the previous packet.)
                newest = None
                for f in self._tcp_take_frames():
                    self._cap(f)
                    newest = f
                return newest
            return self._tcp_take_frame()
        self._recv_sock.setblocking(False)
        latest = None
        try:
            while True:
                try:
                    latest, _ = self._recv_sock.recvfrom(4096)
                except (BlockingIOError, socket.timeout):
                    break
                self._cap(latest)
        finally:
            self._recv_sock.setblocking(True)
        return latest

    def _raw_recv(self, timeout):
        """Block up to `timeout` for one payload. Raises socket.timeout, both transports."""
        if self.transport == 'tcp':
            end = time.monotonic() + timeout
            while True:
                got = self._tcp_take_frame()
                if got is not None:
                    return got
                left = end - time.monotonic()
                if left <= 0 or not select.select([self._tcp], [], [], left)[0]:
                    raise socket.timeout()
                chunk = self._tcp.recv(65536)
                if not chunk:
                    raise ControlLost("robot closed the RTI TCP stream"
                                      + self._link_summary())
                self._rxbuf.extend(chunk)
        self._recv_sock.settimeout(timeout)
        data, _ = self._recv_sock.recvfrom(4096)
        return data

    def _send(self, cmd_type, payload, feedforward, timestamp=None):
        if self.robot_ip is None:
            raise RuntimeError("not connected — call connect() first")
        self.seq += 1
        ts = None
        if self.timestamps and self.transport == 'udp':
            ts = time.perf_counter() if timestamp is None else float(timestamp)
        pkt = pack_command_packet(self.seq, cmd_type, payload, feedforward, ts)
        self._raw_send(pkt)
        self.bytes_tx += len(pkt)
        self._t_send[self.seq] = time.perf_counter()

    def _decode(self, data):
        self.bytes_rx += len(data)
        state = RobotState()
        state.update(parse_state_packet(data.decode('utf-8', errors='replace')))
        self._match_echo(state)
        if "rti_ok" in state:
            self._last_ok = state["rti_ok"]
        if "rti_miss" in state:
            self._last_miss = state["rti_miss"]
        return state

    def _match_echo(self, state):
        """Turn an `rti_seq` echo into a loop-delay sample. No-op if the tag is absent.

        The robot echoes the last seq it ACCEPTED, so an echo of N means every seq below N
        is now resolved: N itself gives a timing sample, and anything still outstanding
        below N was sent but never applied — i.e. lost.
        """
        echo = state.get("rti_seq")
        if echo is None:
            return                      # recipe does not carry the tag
        echo = int(echo)
        if echo == 0 or echo == self._last_echo:
            return                      # not streaming yet, or nothing new since last packet
        self._last_echo = echo

        t0 = self._t_send.pop(echo, None)
        if t0 is not None:
            self.rtt_samples_ms.append((time.perf_counter() - t0) * 1e3)

        # Prune everything the echo has moved past. Those commands are never coming back,
        # which is exactly what makes them countable as lost — the PC has no other way to
        # know a specific command was dropped.
        stale = [s for s in self._t_send if s <= echo]
        for s in stale:
            del self._t_send[s]
            self.lost_seqs += 1

    def latency_stats(self) -> dict | None:
        """Return the loop delay this client measured, in milliseconds.

        The robot echoes the sequence number of the last command it accepted
        (`rti_seq` in the state packet). The time from sending a command until a
        state packet echoes it is one loop delay: how long until your command's
        effect is visible to you, including up to one robot tick. It needs
        `rti_seq` in the active recipe; the standard recipe has it.

        Returns:
            `None` before any delay was measured. Otherwise `n`, the samples;
            `min`, `med` (the median), `p95`, `p99` and `max` in ms; and `lost`,
            the commands the robot never echoed.

        Examples:
            >>> from robotclient import RTIClient
            >>> from robotclient import JointPositionCommand, MotionFinished
            >>> rti = RTIClient(robot)
            >>> rti.connect()
            >>> robot.wait_mode("RTI")
            >>> q0 = rti.read_once()["j_p"]
            >>> rti.claim()
            >>> rti.control(lambda state: MotionFinished() if state.elapsed > 1
            ...             else JointPositionCommand(q0))
            >>> sorted(rti.latency_stats())
            ['lost', 'max', 'med', 'min', 'n', 'p95', 'p99']
            >>> robot.rti_disconnect(wait=True)
            >>> rti.close()
        """
        if not self.rtt_samples_ms:
            return None
        s = sorted(self.rtt_samples_ms)
        q = lambda f: s[min(len(s) - 1, int(len(s) * f))]
        return {"n": len(s), "min": s[0], "med": q(0.50), "p95": q(0.95),
                "p99": q(0.99), "max": s[-1], "lost": self.lost_seqs}
