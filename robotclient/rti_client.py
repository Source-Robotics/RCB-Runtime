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

import socket
import struct
import time

# --- Command types. MUST match communication/rti_protocol.py ---------------
CMD_JOINT_POSITION = 0x01
CMD_JOINT_VELOCITY = 0x02
CMD_JOINT_TORQUE = 0x03
CMD_CARTESIAN_POSE = 0x04
CMD_CARTESIAN_VELOCITY = 0x05
CMD_STOP = 0xFF

CMD_FMT = '<I B 12d'
CMD_SIZE = struct.calcsize(CMD_FMT)   # 101 bytes

# Defaults from config/networking.py. Robot listens for commands on RTI_PORT_RECEIVE;
# the PC listens for state on RTI_PORT_SENDER.
RTI_PORT_RECEIVE = 30001
RTI_PORT_SENDER = 30002

# Recipe tags whose values are per-joint vectors rather than scalars are not
# enumerated here — the parser infers arity from how many values follow a tag.


def pack_command_packet(seq, cmd_type, payload, feedforward):
    """Serialise a command. payload/feedforward are 6 floats each (zero-padded)."""
    p = list(payload) + [0.0] * (6 - len(payload))
    f = list(feedforward) + [0.0] * (6 - len(feedforward))
    return struct.pack(CMD_FMT, seq & 0xFFFFFFFF, cmd_type, *p[:6], *f[:6])


def parse_state_packet(text):
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
    callbacks can time themselves without closing over a start timestamp.
    """

    elapsed = 0.0


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
    """Real-time streaming control over UDP, with TCP for the handshake.

    Args:
        robot:       a connected RobotClient (used for the TCP control channel).
        clock:       'robot' (blocking, robot-paced — the default) or 'pc'.
        listen_port: UDP port to receive state on. Must match the robot's
                     RTI_PORT_SENDER.
    """

    def __init__(self, robot, clock='robot', listen_port=RTI_PORT_SENDER):
        if clock not in ('robot', 'pc'):
            raise ValueError("clock must be 'robot' or 'pc'")
        self.robot = robot
        self.clock = clock
        self.seq = 0
        self.robot_ip = None

        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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

    def connect(self, pc_ip=None):
        """Pair with the robot and put it in RTI mode.

        The robot cannot discover our IP before UDP starts, so it rides the handshake.
        Autodetected from the TCP socket's local address when not given.
        """
        self.robot_ip = self.robot.remote_ip()
        if self.robot_ip is None:
            raise RuntimeError("robot is not connected — call robot.connect() first")
        if pc_ip is None:
            pc_ip = self.robot.local_ip()
        self.robot.rti_connect(pc_ip)
        return self

    def claim(self):
        """Claim RT control. Timing rules start now; a 40 ms command gap stops the robot."""
        self.robot.rti_claim()
        return self

    def release(self):
        """Clean release over TCP (the UDP CMD_STOP path is preferred)."""
        self.robot.rti_release()

    def disconnect(self):
        """Unpair and return the robot to IDLE."""
        self._running = False
        self.robot.rti_disconnect()

    def close(self):
        self._send_sock.close()
        self._recv_sock.close()

    # ------------------------------------------------------------------ #
    # Control
    # ------------------------------------------------------------------ #

    def control(self, callback, first_packet_timeout=2.0, timeout=0.1):
        """Blocking, robot-paced control loop.

        Blocks on the robot's state packet, calls `callback(state)`, sends the returned
        command straight back. Returning MotionFinished sends CMD_STOP and exits.

        Only the PC blocks — the robot's RT loop never does. If our reply misses a tick
        the robot reuses the last command, and after 100 ms of silence it stops.
        """
        if self.clock != 'robot':
            raise RuntimeError("control() is robot-clocked; use send_*() with clock='pc'")

        self._running = True
        self._recv_sock.settimeout(first_packet_timeout)
        started = time.monotonic()
        try:
            while self._running:
                try:
                    data, _ = self._recv_sock.recvfrom(4096)
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
                            "stream; check robot.status() and the error list for the cause."
                            + self._link_summary())
                    raise RuntimeError("RTI timeout: no state packet from robot "
                                       "(never started streaming — is it paired?)")

                state = self._decode(data)
                state.elapsed = time.monotonic() - started

                # Did the robot revoke control? Needs `rti_state` in the active recipe;
                # absent, this is a no-op and the old blind behaviour applies.
                self._check_control_alive(state)

                result = callback(state)

                if isinstance(result, MotionFinished):
                    self._send(CMD_STOP, [0.0] * 6, [0.0] * 6)
                    break
                if result is not None:
                    self._send(result.cmd_type, result.payload, result.feedforward)

                self._recv_sock.settimeout(timeout)
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
            st = self.robot.status() or {}
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

    def send_joint_position(self, positions, feedforward_speed=None):
        self._send(CMD_JOINT_POSITION, positions, feedforward_speed or [0.0] * 6)

    def send_joint_velocity(self, velocities, feedforward_torque=None):
        self._send(CMD_JOINT_VELOCITY, velocities, feedforward_torque or [0.0] * 6)

    def send_joint_torque(self, torques):
        self._send(CMD_JOINT_TORQUE, torques, [0.0] * 6)

    def stop(self):
        """Send CMD_STOP — the robot holds position for 0.5 s, then returns to paired."""
        self._send(CMD_STOP, [0.0] * 6, [0.0] * 6)
        self._running = False

    # ------------------------------------------------------------------ #
    # State reads
    # ------------------------------------------------------------------ #

    def read_state(self):
        """Newest state packet, or None. Non-blocking; drains so we never read stale."""
        self._recv_sock.setblocking(False)
        latest = None
        try:
            while True:
                try:
                    latest, _ = self._recv_sock.recvfrom(4096)
                except (BlockingIOError, socket.timeout):
                    break
        finally:
            self._recv_sock.setblocking(True)
        return self._decode(latest) if latest else None

    def read_once(self, timeout=1.0):
        """Block for a single state packet. Works while merely paired (no claim)."""
        self._recv_sock.settimeout(timeout)
        try:
            data, _ = self._recv_sock.recvfrom(4096)
        except socket.timeout:
            raise RuntimeError("RTI timeout: no state packet from robot")
        return self._decode(data)

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _send(self, cmd_type, payload, feedforward):
        if self.robot_ip is None:
            raise RuntimeError("not connected — call connect() first")
        self.seq += 1
        pkt = pack_command_packet(self.seq, cmd_type, payload, feedforward)
        self._send_sock.sendto(pkt, (self.robot_ip, RTI_PORT_RECEIVE))
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

    def latency_stats(self):
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
