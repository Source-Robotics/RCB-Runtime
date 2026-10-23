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
    """Parsed state packet. Dict of recipe tag → scalar or list.

    `elapsed` is seconds since control started, filled in by RTIClient.control() so
    callbacks can time themselves without closing over a start timestamp. It is the
    ROBOT's time (the packet's `tick` x the robot's tick length, 2026-10-11) when both are
    known: a target computed from it lands on the robot's tick grid exactly, however late
    the PC's callback runs — the PC's own clock put each target up to ±0.4 ms off that grid
    (measured), which a pass-through plays back as speed noise. `robot_time` says which
    (False: the PC's clock — a recipe without `tick`, or an executor too old to publish
    its tick length).
    """

    elapsed = 0.0
    robot_time = False


class _Command:
    cmd_type = None

    def __init__(self, payload, feedforward=None):
        self.payload = list(payload)
        self.feedforward = list(feedforward) if feedforward is not None else [0.0] * 6


class JointPositionCommand(_Command):
    """Target joint positions (rad); feedforward is joint speed (rad/s)."""
    cmd_type = CMD_JOINT_POSITION


class JointVelocityCommand(_Command):
    """Target joint velocities (rad/s); feedforward is joint torque (Nm)."""
    cmd_type = CMD_JOINT_VELOCITY


class JointTorqueCommand(_Command):
    """Target joint torques (Nm). Do NOT include gravity comp — the robot adds it."""
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
    """The robot revoked real-time control while control() was running.

    Raised rather than returned because silently continuing is the dangerous outcome:
    the robot ignores commands once the claim is dropped, so a loop that keeps running
    is an open loop that believes it is closed. The message carries the link metrics
    (success rate, consecutive losses) so the cause is visible immediately.
    """


class MotionFinished:
    """Return this from a control callback to end the loop cleanly."""

    def __init__(self, cmd=None):
        self.cmd = cmd


class RTIClient:
    """Real-time streaming control, with TCP for the handshake.

    Args:
        robot:       a connected RobotClient (used for the TCP control channel).
        clock:       'robot' (blocking, robot-paced — the default) or 'pc'.
        listen_port: UDP port to receive state on. Must match the robot's
                     RTI_PORT_SENDER. Unused when transport='tcp'.
        transport:   'udp' (default) or 'tcp'. UDP is the right transport for real-time
                     control and stays the default; TCP exists because industrial streaming
                     interfaces ship both (real-time over UDP, soft real-time over TCP) and because a PC already reaching
                     the robot through a firewall should not need a second hole opened.

                     What TCP costs is HEAD-OF-LINE BLOCKING: a lost packet stalls
                     everything behind it until retransmission, so UDP's "one dropped
                     setpoint" becomes TCP's "fifty frozen ticks". What it buys is a real
                     disconnect signal — a closed socket is unambiguous, where a silent UDP
                     peer is indistinguishable from a quiet one until the watchdog fires.

                     With 'tcp' ONE connection carries both directions and `listen_port` is
                     not used: the robot listens on RTI_PORT_STREAM_TCP and we dial in.
        timestamps:  True (default): every UDP command carries its send time (time.perf_counter(), or the
                     `timestamp` you pass to send_joint_position — e.g. when your device sampled the
                     target), so the robot follows your real timing instead of the arrival times. An
                     uneven sender (VR, Wi-Fi) left ~3x more chatter without it (simulated, 2026-10-11).
                     No clock synchronisation is needed; only differences are used. TCP never sends it.
                     REQUIRED for hard RTI since 2026-10-12 (the robot refuses a position target without
                     it); False is for SOFT_RTI only — connect() raises otherwise.
    """

    def __init__(self, robot, clock='robot', listen_port=RTI_PORT_SENDER, transport='udp', timestamps=True):
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
        """Pair with the robot and put it in an RTI mode.

        The robot cannot discover our IP before UDP starts, so it rides the handshake.
        Autodetected from the TCP socket's local address when not given.

        `rate_hz` picks the contract, mirroring the hard / soft real-time split of industrial streaming interfaces:

        - **omitted → RTI** (hard real-time). You close the loop: one setpoint every tick,
          40 ms watchdog. Over UDP with send times only (2026-10-12): the robot refuses a hard-RTI
          position target without one (RTI_NO_SEND_TIME), so this raises ValueError up front for
          `timestamps=False` or `transport='tcp'`.
        - **given → SOFT_RTI** (soft real-time). You send targets at *your* rate and the
          robot interpolates. The timeout is sized at three of your command intervals and
          the generator defaults to `ruckig_advancing`. Send times optional, UDP or TCP.

        Prefer a rate that divides evenly into the robot's loop rate — 50, 25 or 10 Hz on a
        250 Hz robot. A non-integer ratio makes commands land alternately on two tick
        counts, corrupting every velocity estimate derived from the stream.

        Before pairing it checks that the robot speaks this client's RTI packet and telemetry format (versioning V3):
        a different version raises VersionMismatch and pairs nothing (allow_mismatch=True skips the check). A robot
        that reports no versions was already warned about by RobotClient.connect.
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
        """Claim RT control. Timing rules start now; a 40 ms command gap stops the robot."""
        self.robot.rti_claim()
        return self

    def release(self) -> None:
        """Clean release over TCP (the UDP CMD_STOP path is preferred)."""
        self.robot.rti_release()

    def disconnect(self) -> None:
        """Unpair and return the robot to IDLE."""
        self._running = False
        self.robot.rti_disconnect()

    def close(self) -> None:
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
        """Blocking, robot-paced control loop.

        Blocks on the robot's state packet, calls `callback(state)`, sends the returned
        command straight back. Returning MotionFinished sends CMD_STOP and exits.

        Only the PC blocks — the robot's RT loop never does. If our reply misses a tick
        the robot reuses the last command, and after 100 ms of silence it stops.
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
        """`timestamp`: the time this target is FOR, in seconds of a monotonic clock — UDP with `timestamps` on
        only. Default: now, at the send. Better: read the clock once, compute the target from that reading and pass the
        same reading (`t = time.perf_counter(); send_joint_position(f(t), timestamp=t)`): a hiccup between computing
        and sending (a garbage-collection pause) otherwise looks like a jump, and in hard RTI the robot refuses a
        stream whose speed / acceleration jump beyond the joint limits (RTI_ACCELERATION_LIMIT)."""
        self._send(CMD_JOINT_POSITION, positions, feedforward_speed or [0.0] * 6, timestamp)

    def send_joint_velocity(self, velocities: Sequence[float], feedforward_torque: Sequence[float] | None = None) -> None:
        self._send(CMD_JOINT_VELOCITY, velocities, feedforward_torque or [0.0] * 6)

    def send_joint_torque(self, torques: Sequence[float]) -> None:
        self._send(CMD_JOINT_TORQUE, torques, [0.0] * 6)

    def stop(self) -> None:
        """Send CMD_STOP — the robot holds position for 0.5 s, then returns to paired."""
        self._send(CMD_STOP, [0.0] * 6, [0.0] * 6)
        self._running = False

    # ------------------------------------------------------------------ #
    # State reads
    # ------------------------------------------------------------------ #

    def read_state(self) -> RobotState | None:
        """Newest state packet, or None. Non-blocking; drains so we never read stale."""
        latest = self._raw_drain()
        return self._decode(latest) if latest else None

    def read_once(self, timeout: float = 1.0) -> RobotState:
        """Block for the CURRENT state packet. Works while merely paired (no claim).

        Drains first, exactly as read_state() does, then blocks only if nothing was queued.

        WHY THE DRAIN. A plain recvfrom() is FIFO: it returns the OLDEST datagram in the
        socket buffer, not the newest. The robot unicasts state at the full loop rate from
        the moment of connect(), so a caller that pairs, waits, then reads gets a packet as
        old as the wait -- 0.3 s of pairing queues ~75 packets on a 250 Hz robot and hands
        back the first of them.

        That is not theoretical. api_tests/27_soft_rti.py read its start pose this way and
        got a snapshot taken during the IDLE -> SOFT_RTI transition, while the arm was still
        settling. It reported 4.985 deg of joint-2 "drift" that had never happened: it was
        measuring a stale packet against fresh ones. Worse, it then COMMANDED that stale
        pose, which is a real step the arm really tracks -- and 4.9 deg sits just inside the
        0.1 rad start-pose tolerance, so the guard passed it.

        No caller ever wants the oldest packet. "Give me the state" means the current state.
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
        """Record EVERY state packet drained, not just the newest. Off-robot measurement."""
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
        """Loop-delay summary in ms, or None if the recipe carries no `rti_seq` tag.

        NOTE what this measures. The robot sends state at the TOP of each tick, before it
        reads commands, so a command applied during tick T is only echoed in tick T+1's
        packet. The figure therefore includes up to one robot tick — deliberately, because
        "when does my command become visible to me" is the quantity that governs loop
        stability. To recover wire RTT, subtract the loopback baseline (~1 tick).
        """
        if not self.rtt_samples_ms:
            return None
        s = sorted(self.rtt_samples_ms)
        q = lambda f: s[min(len(s) - 1, int(len(s) * f))]
        return {"n": len(s), "min": s[0], "med": q(0.50), "p95": q(0.95),
                "p99": q(0.99), "max": s[-1], "lost": self.lost_seqs}
