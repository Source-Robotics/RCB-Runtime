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
from typing import Any, Callable, Sequence

from .connection import Connection
from .commands import generate_id
from .constants import (
    DEFAULT_PORT, IMMEDIATE_TIMEOUT, FORCE_TAKE_TIMEOUT, ACK_TIMEOUT, DEFAULT_MOVE_TIMEOUT,
    WAIT_STATE_TIMEOUT, WAIT_HOMED_TIMEOUT,
)
from .exceptions import RobotError, RobotTimeout, MotionFailed, GripFailed, HomingFailed, VersionMismatch
from .version import check_protocols, wire_names_for

# Pose angles (2026-10-21): radians unless a call says angle_unit="deg".
_RAD_PER = {"rad": 1.0, "deg": 3.141592653589793 / 180.0}


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
    """Why a gripper command finished — not just that it did.

    "complete" tells you the jaws stopped; it cannot tell you whether they
    stopped because they caught something or because they closed on empty air.
    That distinction is the whole of pick verification, so it gets a name rather
    than a bare integer nobody can be expected to remember.

    `object_detected` deliberately is not called "gripped": on gripper_open(),
    code 1 means something BLOCKED the opening, which is not a grip.

        res = robot.gripper_result()
        if res.object_detected:
            ...                      # something is between the jaws
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
        """True if something stopped the jaws (codes 1 and 2)."""
        return self.code in (1, 2)

    @property
    def reached_target(self) -> bool:
        """True if the gripper reached its commanded position with nothing in the way."""
        return self.code == 3

    @property
    def name(self) -> str:
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
        """Connect to the robot's executor and check that it speaks this client's command protocol (versioning V3):
        a different version raises VersionMismatch before anything is sent (allow_mismatch=True skips the check); a
        robot that reports none (a runtime from before 0.9.0) gives a VersionWarning. After it, runtime_version and
        protocols say what the robot reported (None when it did not)."""
        self._conn.connect(ip, port)
        # The executor pushes a status snapshot on connect; wait briefly for it so
        # status() is populated and the first wait_*() has a baseline.
        self._conn.wait_status(lambda s: bool(s), timeout=1.0)
        st = self._conn.latest_status()
        if not st:          # no snapshot within 1 s (a slow link): ask (get_status carries the same fields)
            try:
                st = self._immediate("get_status") or {}
            except RobotError:
                st = {}
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
        self._conn.disconnect()

    @property
    def connected(self) -> bool:
        return self._conn.connected

    def on_error(self, fn: Callable[[str], None]) -> None:
        self._conn.on_error = fn

    def on_status_change(self, fn: Callable[[dict], None]) -> None:
        self._conn.on_status_change = fn

    def on_motion_complete(self, fn: Callable[[str], None]) -> None:
        """fn(command_id) called whenever any queued motion command completes successfully."""
        self._conn.on_motion_complete = fn

    def on_motion_error(self, fn: Callable[[dict], None]) -> None:
        """fn({"id", "code", "message"}) called when a queued motion command fails.

        Fires as soon as the failure arrives. wait_idle() also raises MotionFailed for
        the same failures — use this callback to react immediately (e.g. abort a
        toolpath) rather than at the next synchronization point."""
        self._conn.on_motion_error = fn

    def latest_status(self) -> dict:
        """Latest status snapshot from the event stream: mode, state, homed, err, ..."""
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

    def _queued(self, cmd, args=None, timeout=DEFAULT_MOVE_TIMEOUT, options=None):
        cid = generate_id()
        p = self._conn.send_command(cid, cmd, args, options=options)
        Connection.wait_ack(p, ACK_TIMEOUT)          # raises if rejected at validation
        self._last_queued = p                        # for wait_motion_complete()
        if self._sync:
            Connection.wait_done(p, timeout)
        return cid

    # ------------------------------------------------------------------ #
    # State-changers (immediate ack; block until target state if sync/wait)
    # ------------------------------------------------------------------ #

    def enable(self, wait: bool | None = None) -> None:
        self._state_change("enable", wait, lambda s: s.get("state") == "ENABLED",
                           what="wait_enabled")

    def disable(self, wait: bool | None = None) -> None:
        self._state_change("disable", wait, lambda s: s.get("state") == "DISABLED")

    def clear_errors(self, wait: bool | None = None) -> None:
        self._state_change("clear_errors", wait,
            lambda s: s.get("err", 0) == 0 and s.get("mode") != "ACTIVE_ERROR")

    def enter_exec_mode(self, wait: bool | None = None) -> None:
        self._state_change("enter_exec_mode", wait, lambda s: s.get("mode") == "EXEC",
                           what="wait_mode(EXEC)")

    # ------------------------------------------------------------------ #
    # RTI mode — the TCP half of the real-time interface.
    # The UDP streaming half lives in RTIClient (rti_client.py); these four just
    # open, arm, disarm and close the session. See TODO/RTI_mode_plan.md §3.
    # ------------------------------------------------------------------ #

    def remote_ip(self) -> str | None:
        """Robot's IP as seen on this connection, or None."""
        return self._conn.remote_ip()

    def local_ip(self) -> str | None:
        """Our IP on the route to the robot, or None. Sent in rti_connect."""
        return self._conn.local_ip()

    def rti_connect(self, pc_ip: str | None = None, wait: bool | None = None, rate_hz: float | None = None, transport: str | None = None) -> None:
        """Pair for RTI and enter an RTI mode. Streams state; does NOT grant control.

        pc_ip is where the robot will send state packets. Defaults to this socket's
        local address, which is the interface the robot can actually reach.

        **`rate_hz` picks the contract**, mirroring the hard / soft real-time split of industrial streaming interfaces:

        - **omitted → RTI** (hard real-time). You close the loop: one setpoint every 4 ms,
          40 ms watchdog. Unchanged, and the mode that is hardware-verified.
        - **given → SOFT_RTI** (soft real-time). You send targets at *your* rate, 5–50 Hz,
          and the robot interpolates between them. The timeout is sized at three of your
          command intervals and the generator defaults to `ruckig_advancing`.

        The rate is the discriminator because it is the honest one: a PC that knows its own
        command rate is telling you it does not intend to close the loop every tick.

        **Prefer a rate that divides evenly into 250 Hz — 50, 25 or 10.** A non-integer
        ratio makes commands land alternately on two tick counts, corrupting every velocity
        estimate the robot derives from your stream.

        Both modes share one handler, so the soft joint limits, TCP speed cap, start-pose
        check and gravity compensation are identical. Only the contract differs.
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
        """Claim RT control. Timing rules start: a 100 ms command gap stops the robot.

        Rejected if an error is latched or the robot is not already paired.
        """
        self._immediate("rti_claim")

    def rti_release(self) -> None:
        """Release control cleanly, staying paired. Robot holds position for 0.5 s."""
        self._immediate("rti_release")

    def rti_disconnect(self, wait: bool | None = None) -> None:
        """Unpair and return to IDLE."""
        self._immediate("rti_disconnect")
        if self._should_wait(wait):
            self.wait(lambda s: s.get("mode") == "IDLE", what="wait_mode(IDLE)")

    def rti_set_rate_limiter(self, enabled: bool) -> None:
        """Enable/disable acceleration, jerk and torque-rate clamping on RTI commands.

        ON by default. Turning it off streams your commands to the motors untouched —
        appropriate for measuring true loop latency, not for normal operation.
        """
        self._immediate("rti_set_rate_limiter", [1 if enabled else 0])

    def rti_set_lowpass(self, cutoff_hz: float) -> None:
        """Set the command low-pass cutoff in Hz. 0 disables. **Default: OFF.**

        **DO NOT USE THIS IN SOFT_RTI.** Measured on hardware 2026-08-21: at 5 Hz it makes
        motion monotonically ROUGHER and LAGGIER, the opposite of its purpose --
        0.0440 Nm / 0.210 deg unfiltered, 0.0638 / 0.251 at a 2 Hz cutoff.

        The reason is a conflict with the generator. The filter runs at the loop rate on a
        signal that only updates at your command rate, so between commands it feeds the
        generator a still-settling value that keeps changing. `ruckig_advancing` estimates
        target velocity from successive targets, and it then measures FILTER TRANSIENTS
        rather than target motion; its extrapolation goes wrong and corrects. The filter
        fights the generator.

        It is meant for RTI, where you send every tick and the thing worth removing is
        genuine noise in your own command stream. For a sparse stream use
        `rti_set_min_duration` instead -- it was the entire Pareto frontier in that test.

        A 2nd-order Butterworth (since 2026-10-11; a one-pole filter before), defaulted
        OFF: a computed trajectory has no noise and pays nothing. In hard RTI your stream
        is checked for impossible motion, and noise reads as acceleration — about 0.003 deg
        of white noise per sample already risks RTI_ACCELERATION_LIMIT on J1. What a
        cutoff costs and buys there (simulated, J1): 100 Hz +1.0 ms, tolerates 0.005 deg;
        60 Hz +3.0 ms, 0.010 deg; 40 Hz +5.3 ms, ~0.017 deg; 25 Hz +8.8 ms, 0.030 deg. Enable it if your command stream is
        noisy; 60 Hz is a good start.
        """
        self._immediate("rti_set_lowpass", [float(cutoff_hz)])

    def rti_set_start_pose_check(self, enabled: bool) -> None:
        """Enable/disable validation of the first setpoint after rti_claim.

        ON by default. It is what stops a controller that initialises to zeros from
        commanding a step move across the workspace. Takes effect on the next claim.
        """
        self._immediate("rti_set_start_pose_check", [1 if enabled else 0])

    def set_gravity_comp(self, enabled: bool) -> Any:
        """Robot-WIDE gravity compensation. Affects every mode, not just RTI.

        Distinct from `rti_set_gravity_comp`, which is RTI-scoped and deliberately leaves
        this flag alone. This is the operator-level switch the dashboard drives.

        It also selects how IDLE holds the arm, which is not obvious and matters:

            ON   IDLE applies gravity-compensation TORQUE. The arm is weightless and
                 hand-guidable, and holds only as well as the gravity model is accurate.
            OFF  IDLE falls back to VELOCITY-hold at 0 rad/s. The arm is stiffer, and the
                 firmware velocity loop is now in circuit -- which is the loop that
                 limit-cycles on a joint whose gains are too stiff for its load.

        That fallback is the only way to enter velocity-hold deliberately, without waiting
        for an error, so it is also how api_tests/31_velhold_buzz.py reproduces the J3
        oscillation on demand.
        """
        return self._immediate("set_gravity_comp", [1 if enabled else 0])

    def set_inertia_comp(self, enabled: bool) -> Any:
        """Robot-wide INERTIA feedforward: the torque M(q)·qdd that the commanded acceleration needs (the links and the
        motor rotors behind the gearboxes), added to the current the drives get — in EXEC, JOG and RTI / SOFT_RTI
        position and velocity commands. Computed from the COMMANDED motion of the tick, never a measured one; not
        added in freedrive, holds, homing or RTI torque commands. Starts as system.xml <dynamics_feedforward> says
        (ON as shipped); latest_status() reports "inertia_comp". Separate from set_coriolis_comp and set_gravity_comp."""
        return self._immediate("set_inertia_comp", [1 if enabled else 0])

    def set_can_read_first(self, enabled: bool) -> Any:
        """The CAN tick order (option B, 2026-10-12; RTI starts in the order system.xml <can_read_first> names, ON by
        default): True = RTI reads the joints' CAN replies at the START of each tick, computes that tick's commands
        from them and sends them at once — 3.8 instead of 7.8 ms from a drive's measurement to the command computed
        from it (measured), with the same late-reply rate and tracking; False = the old order. Homing always uses the
        old order. RTI.log reports the waits when it is switched off."""
        return self._immediate("set_can_read_first", [1 if enabled else 0])

    def joint_test(self, joint: int, amplitude_deg: float = 5.0, speed_dps: float = 5.0,
                   current_ma: float = 600.0) -> Any:
        """A small powered test move of one joint and back, homed or not (2026-10-18; the first-run check's stage 3):
        joint 1..N moves `amplitude_deg` (signed, up to 15) from where it stands and back on a smooth ramp peaking at
        `speed_dps` (up to 10), its drive's current limited to `current_ma` meanwhile; every other joint keeps holding.
        joint_test(0) stops it. Only in IDLE with the arm ENABLED. The robot stops it itself when the joint falls 2 deg
        behind (blocked, an endstop, too little current), on a disable or any mode change. Result in
        status()["joint_test"]: state (running / done / fell_behind / stopped), moved_deg, max_error_deg,
        peak_current_ma, end_error_deg, seq (counts finished tests)."""
        return self._immediate("joint_test", [int(joint), float(amplitude_deg), float(speed_dps), float(current_ma)])

    def free_joint(self, joint: int) -> Any:
        """Free one joint so it can be turned by hand (2026-10-18; the first-run check): joint 1..N goes limp (its
        drive's current limit 0) while the others keep holding; 0 holds it again. Only in IDLE (enabled or not, homed
        or not). A joint that carries the arm's weight (J2, J3) FALLS when freed unless it is supported. The robot holds
        it again itself before any mode change (homing, a jog, an error, the E-stop) and 2 minutes after the last
        free_joint(n) (sending it again renews it). status()["free_joint"] = the freed joint, 0 = none."""
        return self._immediate("free_joint", [int(joint)])

    def hall_check(self, on: bool = True) -> Any:
        """The Hall sensor check (2026-10-13): True = the joint homed on a Hall sensor (J6) goes limp (its drive's
        current limit 0) and its Hall pin is read every tick — turn it by hand and watch the dashboard's I/O tab, or
        status()["hall_check"]. Only in IDLE (enabled or not, homed or not); the robot stops it itself before any mode
        change (homing, a jog, an error, the E-stop) and puts the joint's limits back. False = stop it."""
        return self._immediate("hall_check", [1 if on else 0])

    def set_coriolis_comp(self, enabled: bool) -> Any:
        """Robot-wide CORIOLIS / centripetal feedforward: the torque C(q, qd)·qd the joints exert on each other when
        several move at once, from the commanded motion — same modes as set_inertia_comp. Starts as system.xml
        <dynamics_feedforward> says (ON as shipped); latest_status() reports "coriolis_comp"."""
        return self._immediate("set_coriolis_comp", [1 if enabled else 0])

    def set_friction_ff(self, enabled: bool) -> Any:
        """Robot-wide FRICTION feedforward (2026-10-17): the torque each joint's friction takes at the COMMANDED speed
        (the robot XML's friction model — Coulomb + load x |gravity torque| rounded through zero speed + viscous x
        speed, measured by api_tests/77_friction_id.py), added to the current the drives get in EXEC, JOG and RTI /
        SOFT_RTI position and velocity commands. Nothing at a commanded speed of zero (a hold), in freedrive (see
        set_friction_comp), homing or RTI torque commands. Starts as system.xml <dynamics_feedforward><friction> says
        (OFF as shipped); latest_status() reports "friction_ff". Measuring friction (api_tests 54 / 77) needs it OFF."""
        return self._immediate("set_friction_ff", [1 if enabled else 0])

    def set_friction_comp(self, enabled: bool) -> Any:
        """Friction compensation in FREEDRIVE (IDLE with gravity comp ON): a push in the direction each joint is
        already moving — a fraction of its measured friction minus its measured gravity-model error (robot XML
        <friction_coulomb_nm> + <friction_load_per_nm> x |gravity torque| / <gravity_error_nm>, policy <friction_comp>
        in system.xml). Zero at rest, so a released arm is still held by its static friction. OFF at every RTI start.
        latest_status() reports "friction_comp"."""
        return self._immediate("set_friction_comp", [1 if enabled else 0])

    def set_collision_detection(self, mode: str) -> Any:
        """Collision detection's reaction: "off", "warn" or "stop" — until RTI restarts, then system.xml
        <safety_monitor><collision> again ("warn" as shipped). The safety monitor compares the external torque on each
        joint while the robot moves it (EXEC, JOG, RTI, SOFT_RTI) with the torque it had at rest before the move:
        above the robot XML's <contact_torque_nm> a live CONTACT_WARNING, above <collision_torque_nm> a
        COLLISION_WARNING ("warn") or the COLLISION error ("stop": the robot stops and holds until errors are
        cleared). A program that presses on things ON PURPOSE switches it to "off" (or "warn") first and back after.
        Declare a held part with set_payload — an undeclared one reads as external torque. latest_status() reports
        "collision_detection"."""
        return self._immediate("set_collision_detection", [str(mode).strip().lower()])

    def set_payload(self, mass_kg: float, com_m: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
        """Declare the part the gripper holds: mass [kg] and its centre of mass [m] in the TOOL frame (the
        gripper's link frame: +z out of the flange toward the jaws). The robot ADDS it to the gripper's own body and
        rebuilds its gravity feedforward and the freedrive walls' dynamics at once (~1 ms, every mode). `0` restores
        the gripper alone. Refused above the robot XML's `<max_payload_kg>` or with a COM more than 0.5 m out
        (`PAYLOAD_LIMIT`). Not persisted: every RTI start begins at 0. Needs control. `status()["payload"]` shows it.
        """
        return self._immediate("set_payload", [float(mass_kg), [float(v) for v in com_m]])

    def rti_set_gravity_comp(self, enabled: bool) -> None:
        """Enable/disable gravity-compensation feedforward on RTI commands.

        **ON by default** — the robot carries its own weight, as it does in IDLE, JOG and
        EXEC. Turn it off only if your controller computes gravity itself, since a torque
        controller that also receives ours would apply it twice.

        This is RTI-scoped. It does not change the robot-wide gravity flag, so IDLE and JOG
        are unaffected when you disconnect; that flag still overrides this one if the
        operator has switched compensation off globally.
        """
        self._immediate("rti_set_gravity_comp", [1 if enabled else 0])

    def rti_set_dynamics_ff(self, source: str) -> None:
        """Inertia / Coriolis feedforward source for RTI / SOFT_RTI POSITION streams: "off" or "stream".

        **"stream" at every RTI / SOFT_RTI entry** (since 2026-10-11) — set "off" after `rti_connect`, before claiming,
        if your PC computes its own dynamics (as with `rti_set_gravity_comp`). "stream" feeds forward
        M(q)·q̈ (+ C(q, q̇)·q̇) from the stream's OWN acceleration: the change of the target velocity between your
        packets over their interval, smoothed (~8 Hz). It helps from ~25 Hz up and is too late to help below ~10 Hz.
        The robot's generator acceleration is never fed forward (it switches between its limits as it chases each
        target). The robot-wide `set_inertia_comp` / `set_coriolis_comp` switches still apply on top. Velocity and
        torque commands get none.
        """
        self._immediate("rti_set_dynamics_ff", [str(source).strip().lower()])

    def rti_set_tcp_vel_limit(self, mps: float) -> None:
        """Cap the end effector's translational speed at `mps` metres/second. 0 disables.

        **OFF by default.** Joint velocity limits do not bound TCP speed — near a
        singularity a slow joint can produce a very fast end effector, and this is the
        only guard for that case. The cap scales the limiter's per-tick velocity budget
        (position commands) or the whole commanded vector, direction preserved (velocity
        commands). Like the soft joint limits, on the velocity path it applies even with
        the rate limiter disabled.

        Practical note: near a singularity the cap makes the arm CRAWL rather than let
        the TCP run away — that slowdown is the feature working, not a fault.
        """
        self._immediate("rti_set_tcp_vel_limit", [float(mps)])

    def rti_set_command_timeout(self, seconds: float) -> None:
        """How long RTI control may go without a command before the robot stops itself.

        **Hard RTI: fixed at 0.040 s, and this can only TIGHTEN it.** A request above 40 ms is
        clamped to 40 ms. Hard RTI carries velocity and torque commands, and while no new command
        arrives the robot keeps applying the LAST one every tick — a stale velocity or torque would
        keep driving for the whole window, so a client is never allowed to widen it.

        **SOFT_RTI: sized for your rate, widenable up to the operator's ceiling.** Connecting with
        `rti_connect(ip, rate_hz=...)` already sets it to 3 command intervals (`3.0 / rate_hz`),
        which tolerates two dropped packets without tolerating a dead PC. Call this AFTER the
        session is paired if you want more slack; the robot clamps the request to
        max(operator ceiling, your rate's own 3 intervals), capped at 1.0 s. The operator's
        ceiling is `<safety><max_soft_rti_timeout>` in the robot XML (0.25 s by default), so a
        remote PC can never exceed what the operator allowed. SOFT_RTI is POSITION-ONLY — velocity
        and torque packets are refused — which is what makes a longer window safe: while no
        command arrives the robot continues toward your LAST target and stops there.

        Below 2 ticks (8 ms) is clamped up: one jittered packet must not look like a dead link.
        Set it AFTER rti_connect: pairing re-applies the contract and resets the timeout.
        """
        self._immediate("rti_set_command_timeout", [float(seconds)])

    def rti_set_vel_scale(self, fraction: float) -> None:
        """Scale the RTI joint velocity limits to `fraction` of the robot XML values.

        **Defaults to 1.0** (the XML values). Can only lower the limit, never raise it.

        This is the smoothness knob for low-rate streaming. The robot's generator plans to
        arrive and stop at each target you send; if it is ten times faster than the speed
        you are actually travelling, it sprints to each target and waits, and the motion
        pulses. Capping it near your travel speed removes the room to sprint, so it glides
        instead — measured 59% velocity ripple at 20 Hz down to 0%.

        Set it to roughly `your_travel_speed / joint_velocity_limit`. It is a cliff rather
        than a slope: too high and it does nothing at all, and the cost of getting it right
        is that the arm has no speed in reserve to catch up after a disturbance.

        Velocity only — acceleration and jerk limits are unchanged.
        """
        self._immediate("rti_set_vel_scale", [float(fraction)])

    def rti_set_min_duration(self, multiplier: float) -> None:
        """Make each planned move last at least `multiplier` of your command intervals.

        **SOFT_RTI sets 1.5 at every entry (since 2026-10-11); hard RTI 0.** Call this after `rti_connect` to choose
        your own (0 = SOFT_RTI before 2026-10-11). Why the default: with the advancing generator and no pacing, every
        packet that differs from the robot's extrapolation was corrected with a kick at the full RTI limits — J1 visibly
        jerky at 50 / 10 Hz on the arm, and an operator's sudden stop overshot 2-7° (simulated); with 1.5, 4-13x
        smoother, 2-4x more accurate in shape, stop overshoot 0.4° at 50 Hz, none at <= 10 Hz, for more lag (+20 ms at
        50 Hz, +120 ms at 10 Hz). A minimum trajectory execution time. The robot
        measures your command rate itself, so you state a shape preference rather than a
        speed — no need to know your travel velocity in rad/s.

        **The multiplier must be above 1.** At exactly 1 the plan finishes just as your next
        command lands, so it still decelerates to a stop and you still get pulsing. From
        about **1.5** upward the plan always outlives your next command, the deceleration is
        never reached, and the motion becomes continuous — measured velocity swing at 5 Hz
        drops from 0.874 rad/s to 0.

        Like every pacing option it buys smoothness with lag. With the plain `ruckig` generator it was the least
        accurate of the smooth choices (2026-08); `rti_set_servo_generator("ruckig_advancing")` reaches 0.05°
        where this reaches 6.3°. Use this if you want a duration-floor contract; use the
        advancing generator if you want the best tracking.

        Ruckig generators only — the tracker has no planned duration to floor.
        """
        self._immediate("rti_set_min_duration", [float(multiplier)])



    def take_control(self, force: bool = False, timeout: float | None = None) -> dict:
        """Claim exclusive control of the robot. Returns the holder.

        Usually unnecessary — control attaches automatically to whoever issues the first
        command that moves or arms the robot. Call it to RESERVE control before starting a
        programme, or with `force=True` to take it from a holder that is wedged or absent.

        Exclusivity is how industrial arms commonly prevent two parties commanding one arm
        (they typically authenticate nobody, relying on network isolation). It is the half
        that actually matters.

        What is NEVER gated, no matter who holds control: `stop`, `pause`, `disable`,
        `safety_stop`, `clear_errors` and every read. If arbitration could lock someone out
        of stopping the robot, the safety feature would have become the hazard.
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
        """Require a press on the control box before a FORCED takeover succeeds.

        Without it, `take_control(force=True)` is software-only and any machine on the network
        can seize control from any other. Industrial arms commonly require a physical action at the
        robot for the same operation (a button on the arm, a pendant's enabling device). This is
        that gate on our hardware, and it is why the IP allowlist can safely be turned off.

        Only a takeover FROM A LIVE HOLDER is gated — a first claim on an idle robot is
        unaffected. Ships OFF.
        """
        return self._immediate("set_force_confirm", [bool(enabled)])

    def release_control(self) -> Any:
        """Give up control. Always allowed; never refused."""
        return self._immediate("release_control")

    def control_holder(self) -> str | None:
        """Who currently holds control ("ip:port"), or None. From the status stream."""
        return self.latest_status().get("control_holder")

    def local_label(self) -> str | None:
        """This connection's own "ip:port" as the executor labels it — compare with
        control_holder() to learn whether control is yours."""
        return self._conn.local_label()

    def holds_control(self) -> bool:
        """True if THIS connection is the current holder."""
        holder = self.control_holder()
        return holder is not None and holder == self.local_label()

    def reload_recipes(self) -> None:
        """Re-scan data_recipe/ on the robot. Needed after ADDING a recipe file.

        RecipeManager scans once at startup, so a recipe copied to the robot afterwards is
        invisible until this is called — and `set_recipe` for an unknown name is REFUSED
        robot-side with only a log line, which the caller never sees. Without this the only
        remedy was restarting RTI.py, which clears homing.
        """
        self._immediate("reload_recipes")

    def reload_filters(self) -> None:
        """Re-read config/filters.xml on the robot and apply it live — no restart, no re-home.

        The executor parses the file and hands RTI the spec table; RTI rebuilds its filter banks from it (each
        filter's state resets, so expect a brief transient of about one filter window). A malformed file is
        REFUSED (`FILTER_CONFIG`) and the filters in force stay. The dashboard's "Reload Filters" is this call.
        """
        self._immediate("reload_filters")

    def rti_set_state_rate(self, hz: float) -> None:
        """How often the robot sends its state packet back. 0 = every tick (the default).

        The state stream runs at the robot's FULL LOOP RATE no matter how slowly you
        command, so a 5 Hz SOFT_RTI session receives 250 packets a second — fifty per
        command, and ~136 KB/s on the `standard` recipe.

        Worth setting when:
          * you are on the TCP transport. State is 20-40x the traffic of your commands and
            is the direction that generates backpressure; this is the knob that prevents it.
          * you do not read every packet anyway. `read_state()` returns the NEWEST, so the
            rest were only ever going to be discarded.

        Composes with `set_recipe`: that cuts the payload (`minimal` is ~4.5x smaller), this
        cuts the rate. Clamped to [10 Hz, loop rate]. There is a FLOOR rather than an off
        switch because the state stream is the PC's only failure detector — RTIClient
        raises ControlLost when it stops, which is how a latched error on the robot becomes
        an exception on the PC instead of a silent hang.

        Ignored in hard RTI, where you close the loop every tick and therefore need state
        every tick.
        """
        self._immediate("rti_set_state_rate", [float(hz)])

    def rti_set_pace(self, factor: float) -> None:
        """Cap the robot's generator at `factor` × the speed your target is actually moving.

        **OFF by default.** The self-tuning alternative to `rti_set_vel_scale`: instead of
        you declaring a speed in rad/s, the robot measures how fast your commands are moving
        the target and derives the cap itself. It follows you if you change speed or command
        rate mid-session.

        `factor` is headroom. At 1.0 the robot may move exactly as fast as your target and
        therefore can never catch up once it falls behind — measured 9.3° of tracking error
        at 5 Hz against 5.0° for a well-chosen fixed cap. Above about 3 it stops binding.

        **Measured verdict: this is dominated.** It removes the pulsing, but so does the
        fixed cap, and both trail far behind `rti_set_servo_generator("tracker")` on
        accuracy (0.09°). Exposed for setups where a velocity bound is the
        expected contract; prefer the tracker if you just want good streaming.
        """
        self._immediate("rti_set_pace", [float(factor)])

    def rti_set_servo_generator(self, name: str, gain: float | None = None, lookahead: float | None = None, self_lookahead: float | None = None) -> None:
        """Choose how the robot fills in the ticks between your position commands.

        **"passthrough"** (hard RTI's default since 2026-10-11, hard RTI only) replays your
        targets exactly, one tick behind, using each packet's send time (RTIClient sends it
        by default; `control()` uses the robot's own time): no planning lag. Each joint
        still stays within its velocity / acceleration / jerk limits — a jump is caught up
        on, never overshot — and never passes a soft limit. A stream without send times
        gets "ruckig" below, exactly as before.

        **"ruckig"** plans a jerk-limited move to each target and *arrives* on it, stopped.
        Exact, no overshoot — right for point-to-point. On a stream it trails by about its
        stopping distance (15-20 ms on J1 at speed), and on a sparse stream it sprints to
        each target and idles until the next, which is what makes low-rate streaming pulse
        unless you cap the speed with `rti_set_vel_scale`.

        **"tracker"** is a servo-style tracking law (the common industrial streaming-servo command). It never plans to stop, so the
        pulsing has no mechanism to appear, and it needs **nothing** from you — no
        feedforward velocity, no travel speed to declare. On a constant-speed stream it
        tracks within 0.08° at any rate from 50 Hz down to 5 Hz, where the capped Ruckig
        path trails by up to 5°.

        The trade: it is a servo, not a planner. Given a large step it can overshoot, where
        Ruckig never does. Stream with "tracker", move point-to-point with "ruckig".

        `gain` is the tracking bandwidth in rad/s (default 15). Higher tracks a fast-moving
        target more tightly and passes more command noise through; above about 20 a large
        step starts to overshoot. `lookahead` is the extrapolation horizon in seconds; leave
        it out and it is re-derived as `2/gain`, the value that cancels the tracker's
        steady-state lag on a constant-speed target.

        Safe to switch mid-session — the new law continues from the position and velocity
        the old one had actually reached.
        """
        self._immediate("rti_set_servo_generator",
                        [str(name),
                         "" if gain is None else float(gain),
                         "" if lookahead is None else float(lookahead),
                         "" if self_lookahead is None else float(self_lookahead)])

    def rti_set_profile(self, name: str, rate_hz: float | None = None) -> None:
        """Pick the streaming contract: **"realtime"** (default) or **"servo"**.

        One call sets a coherent bundle instead of three loose settings. A servo-rate
        timeout with a point-to-point generator still pulses, and nothing otherwise stops
        you creating that combination.

        - **realtime** — you close the loop, one setpoint per 4 ms tick. 40 ms watchdog,
          Ruckig, no pacing. This is what the arm has been hardware-verified with.
        - **servo** — you send targets at your own rate and the robot fills in between
          them. Pass `rate_hz` and the timeout is sized at three of your command intervals.

        Prefer a rate that divides evenly into 250 Hz — **50, 25 or 10** — because a
        non-integer ratio makes command arrivals alternate between two tick counts, which
        corrupts every velocity estimate taken from the stream.

        Also makes the contract reportable, which matters for reading diagnostics:
        `control_success_rate` means "packet loss" only when you send once per tick. At
        20 Hz a perfectly healthy link scores about 8%.
        """
        self._immediate("rti_set_profile",
                        [str(name), "" if rate_hz is None else float(rate_hz)])

    def set_recipe(self, name: str) -> None:
        """Choose which telemetry recipe the robot's state stream carries.

        The recipe decides what every state packet contains. Until 2026-08-07 only the GUI
        could set it, which meant the controlling PC — the one process the recipe system
        exists to serve — could not ask for the data it needed.

        The one that usually matters remotely is **"diagnostics"**: it carries `rti_ok` and
        `rti_miss`, and those are the ONLY way to see robot-side link quality from another
        machine. Success rate and miss counters live in the robot's shared memory, which a
        remote host cannot read, so without this the PC is limited to its own timing.

        Shipped recipes: minimal, standard (default), commanded, diagnostics, full.
        `robot.status()["recipes"]` lists what is actually on disk.

        An unknown name is REFUSED robot-side and logged, leaving the current recipe in
        place — a typo must not silently strip your telemetry down to a fallback.
        """
        self._immediate("set_recipe", [str(name)])

    def stop(self, wait: bool | None = None) -> None:
        """Stop: the queued program is dropped. In EXEC the arm brakes along its planned path at the joints' hardware
        limits (at most ~0.15 s from full speed) and HOLDS there, still in EXEC — the next move starts from that point;
        elsewhere (JOG, RTI, ...) the robot goes to IDLE. Returns once stopped (the executor answers then)."""
        self._state_change("stop", wait, lambda s: s.get("mode") == "IDLE"
                           or (s.get("mode") == "EXEC" and not s.get("exec_stopping")), what="stop")
        # stop() cancels queued motion server-side, which replies "error"/STOPPED for
        # each one. Those are the expected consequence of stopping, not a motion
        # failure — drop them so they can't surface at a later wait_idle(). Safe to do
        # here: the executor writes those replies before stop's own "ok", so the
        # receiver thread has already recorded them by the time _state_change returns.
        self._conn.take_motion_errors()

    def home(self, wait: bool | None = None) -> None:
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
        """Brake along the planned path to rest and hold there (EXEC). With wait, returns at rest. resume()."""
        self._state_change("pause", wait, lambda s: bool(s.get("exec_paused")))

    def resume(self, wait: bool | None = None) -> None:
        """Speed back up along the same path and continue from exactly where pause() stopped."""
        self._state_change("resume", wait, lambda s: not s.get("exec_paused"))

    def set_joint_speed(self, percent: float) -> Any:
        return self._immediate("set_joint_speed", [float(percent)])

    def set_blending(self, percent: float) -> Any:
        return self._immediate("set_blending", [float(percent)])

    def set_profile(self, profile: str) -> Any:
        return self._immediate("set_profile", [str(profile).strip().lower()])

    set_default_profile = set_profile      # its other name (2026-10-23)

    def set_ik_solver(self, solver: str) -> Any:
        return self._immediate("set_ik_solver", [str(solver).strip().lower()])

    set_default_ik_solver = set_ik_solver      # its other name (2026-10-23)

    def set_completion_policy(self, policy: str) -> Any:
        return self._immediate("set_completion_policy", [str(policy).strip().lower()])

    # ------------------------------------------------------------------ #
    # Jog + live config (immediate; migrated from the GUI's old direct-to-RTI
    # push — see TODO/gui_executor_migration_plan.md). Jog forwards to RTI's
    # persistent JogHandler and returns on the executor ack, not on physical
    # completion, so a hold-to-jog caller drives its own cadence.
    # ------------------------------------------------------------------ #

    def enter_jog_mode(self, wait: bool | None = None) -> None:
        """Enter JOG mode. Requires the robot ENABLED and homed, like enter_exec_mode."""
        self._state_change("enter_jog_mode", wait, lambda s: s.get("mode") == "JOG",
                           what="wait_mode(JOG)")

    def enter_idle_mode(self, wait: bool | None = None) -> None:
        """Return to IDLE (always allowed).

        If queued motion is still pending, RTI discards it on leaving EXEC, so the program is
        ABORTED: pending commands fail with code ABORTED and new motion is refused
        (MOTION_ABORTED) until clear_errors(). To cancel a program deliberately, use stop(),
        which cancels it without that latch."""
        self._state_change("enter_idle_mode", wait, lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")

    # ------------------------------------------------------------------ #
    # Firmware flashing — see docs/firmware_flashing.md.
    # FLASHING makes RTI go silent on the CAN bus so a separate flasher process
    # can own it, while the loop keeps running (E-stop, IO and telemetry stay
    # live). It does NOT flash anything itself; it grants the bus.
    # ------------------------------------------------------------------ #
    def safety_stop(self, wait: bool | None = None) -> None:
        """Drive the motors with NOTHING — the arm goes limp and WILL FALL.

        Sends Position=None, Speed=None, Current=0 to every joint, so no loop is
        holding anything. This is not `disable()`: disable leaves a velocity-0
        hold that still resists motion. This removes all authority.

        Support the arm before calling this. It is here for diagnostics — with
        every motor commanded by nothing, anything still moving or buzzing is the
        driver's own firmware, not the runtime.

        Reachable from any mode and any state, by design.
        """
        self._state_change("safety_stop", wait,
                           lambda s: s.get("mode") == "SAFETY_STOP",
                           what="wait_mode(SAFETY_STOP)")

    def release_safety_stop(self, wait: bool | None = None) -> None:
        """Leave SAFETY_STOP for IDLE, restoring the normal hold."""
        self._state_change("release_safety_stop", wait,
                           lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")

    def enter_flashing_mode(self, parked_confirmed: bool = False, force: bool = False, wait: bool | None = None) -> None:
        """Hand the CAN bus to an external flasher.

        The motors LOSE TORQUE here (their watchdogs fire into shorted-phase
        brake), so the arm must be able to hold itself up. **Nothing on the
        robot can determine that**, so this call requires an explicit claim from
        you — calling it with no arguments is refused on purpose:

            robot.move_joints(*robot.park_pose_deg())   # or park it by hand
            robot.enter_flashing_mode(parked_confirmed=True)

        parked_confirmed=True — you have parked the arm and confirmed it.
        force=True            — the arm is NOT parked but IS physically
                                supported. Needed when a driver has lost its
                                firmware and cannot be commanded to park at all.

        Both open the gate; RTI logs which claim was made, so if an arm ever
        falls during maintenance there is a record of what was asserted.

        Unlike the other mode-entry calls this works while DISABLED, unhomed, or
        with errors active — that is the recovery path for a driver left in its
        bootloader by a failed flash.

        Note a flash INVALIDATES HOMING: the driver reboots and the encoders are
        on the motors, so re-home before moving the arm afterwards.
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
        """Return to IDLE and resume normal CAN traffic.

        RTI re-bases its CAN freshness clock on the way out, so nodes get the
        normal reconnect window instead of instantly latching a disconnect error
        after their silence during the flash. A driver that was reflashed comes
        back with its preloaded config wiped and is reconfigured automatically by
        the existing stale->fresh reconnect path.
        """
        self._state_change("exit_flashing_mode", wait,
                           lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")

    def jog(self, joint: int, direction: int, speed: float) -> Any:
        """Start jogging one joint. direction: +1 / -1. speed: fraction of the jog speed
        limit. Only valid in JOG mode; call jog_stop() to halt."""
        return self._immediate("jog", [int(joint), int(direction), float(speed)])

    def jog_stop(self) -> Any:
        """Halt jog motion. Accepted in any mode, so a released button always stops."""
        return self._immediate("jog_stop")

    def set_jog_accel(self, accel_time: float) -> Any:
        """Jog acceleration ramp time (s); persists on RTI's JogHandler."""
        return self._immediate("set_jog_accel", [float(accel_time)])

    def set_jog_profile(self, profile: str) -> Any:
        """Jog motion profile, e.g. 'scurve' / 'trapezoid'."""
        return self._immediate("set_jog_profile", [str(profile).strip()])

    def set_jog_jerk_factor(self, factor: float) -> Any:
        """Jog jerk factor (used by the s-curve profile)."""
        return self._immediate("set_jog_jerk_factor", [float(factor)])

    def set_jog_control_mode(self, mode: str) -> Any:
        """Jog control mode, e.g. 'pid'."""
        return self._immediate("set_jog_control_mode", [str(mode).strip()])


    def set_pid_gains(self, joint: int, **gains: float) -> Any:
        """Set control gains for one joint (or the gripper, joint=<joint_count>). Keys:
        KPP, KPV, KIV, KPIQ, KIIQ, KP, KD, Ilim, vel_lim — unknown keys are dropped by the
        executor. Forwarded as a dict; RTI reads the fields off the command dict."""
        payload = {"joint": int(joint)}
        payload.update({k: float(v) for k, v in gains.items()})
        return self._immediate("set_pid_gains", payload)

    def calibrate_gripper(self) -> Any:
        """Run the gripper calibration sweep (immediate; requires a CAN gripper)."""
        return self._immediate("calibrate_gripper")

    gripper_calibrate = calibrate_gripper      # its other name (2026-10-23)

    def set_gripper_ctrl_mode(self, mode: int) -> Any:
        """Gripper control mode: 1 = motor mode, 2 = firmware mode."""
        return self._immediate("set_gripper_ctrl_mode", [int(mode)])

    def set_gripper(self, position: int, speed: int = 100, current: int = 200, action: int = 1) -> Any:
        """Firmware/motor-mode direct gripper target (immediate; distinct from the queued
        open_gripper / close_gripper / move_gripper). position/speed 0-255 raw gripper scale,
        current in mA, action 0/1. Requires a CAN gripper (RTI ignores it otherwise)."""
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
                `speed` and `accel` (a fraction of the limit up to 1, or a percent
                above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
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
    # gripper_object_detection (a Robotiq-style status code) and only checkpoints
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
        cid = self._queued(cmd, args, timeout, options)
        self._last_gripper_queued = self._last_queued
        return cid

    def open_gripper(self, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        return self._queued_gripper("open_gripper", [], timeout, options)

    def close_gripper(self, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        return self._queued_gripper("close_gripper", [], timeout, options)

    # Other names (2026-10-23, the naming pass): the names start with the verb, as every other command does.
    gripper_open = open_gripper
    gripper_close = close_gripper

    def gripper_result(self) -> GripperResult | None:
        """Outcome of the most recently SETTLED gripper command, or None.

        Returns a GripperResult. Available as soon as the command's checkpoint
        arrives, which is:

            sync  — immediately after gripper_close() returns
            async — after wait_motion_complete() / wait_idle()

        The queued call itself still returns a command id, like every other
        queued command; the outcome is fetched rather than returned so gripper
        commands keep the same shape as joint moves.

            robot.gripper_close()
            robot.wait_motion_complete()
            if robot.gripper_result().object_detected:
                print("picked something")

        None means no gripper command has settled on this connection yet.
        """
        last = self._conn.last_gripper_result()
        if last is None:
            return None
        cid, code = last
        return GripperResult(code, command_id=cid)

    def move_gripper(self, position: int, speed: int | None = None, current: int | None = None, timeout: float = DEFAULT_MOVE_TIMEOUT, **options: Any) -> str:
        """position: 0-255 raw gripper scale, 1=fully open .. 254=fully closed —
        matches the GUI's gripper slider. NOT millimeters: firmware-mode grippers
        don't expose a verified mm conversion (see motion/trajectory_planner.py)."""
        args = [int(position)]
        if speed is not None:
            args.append(int(speed))
        if current is not None:
            args.append(int(current))
        return self._queued_gripper("move_gripper", args, timeout, options)

    # ------------------------------------------------------------------ #
    # Synchronization (status events + checkpoints)
    # ------------------------------------------------------------------ #

    def wait_idle(self, timeout: float | None = None, raise_on_error: bool = True) -> list[dict]:
        """Block until all queued motion has finished (checkpoints, must-not-lose).

        "Finished" includes *failed*: a command that errors is terminal, so it stops
        being outstanding. In async mode _queued() returned at the "queued" ack, so
        nothing was waiting on that command and its failure would otherwise vanish.
        Any such failure is raised here as MotionFailed — the robot may have executed
        only part of the motion you asked for.

        raise_on_error=False returns the failure list instead of raising (still
        clearing it), for callers that want to inspect partial execution themselves.
        """
        if not self._conn.wait_idle(timeout=timeout):
            raise RobotTimeout("wait_idle timed out")
        failures = self._conn.take_motion_errors()
        if failures and raise_on_error:
            raise MotionFailed(failures)
        return failures

    def wait_motion_complete(self, timeout: float | None = None) -> None:
        """Block until the most recently queued command completes — NOT the whole
        queue (see wait_idle for that). No-op if nothing has been queued yet."""
        if self._last_queued is None:
            return
        Connection.wait_done(self._last_queued, timeout or DEFAULT_MOVE_TIMEOUT)

    def wait_gripper_move_complete(self, timeout: float | None = None) -> None:
        """Block until the most recent GRIPPER command finishes moving.

        Distinct from wait_motion_complete(), which waits for the most recent
        QUEUED command of any kind — queue a gripper command then a joint move
        and that would wait for the move instead.

        "Finished" means the jaws stopped: reached the commanded position, or
        something stopped them. It does not say which. Use wait_holding_part() /
        wait_released_part() when the outcome is what you care about.
        """
        if self._last_gripper_queued is None:
            return
        Connection.wait_done(self._last_gripper_queued,
                             timeout or DEFAULT_MOVE_TIMEOUT)

    def wait_holding_part(self, timeout: float | None = None) -> GripperResult:
        """Block until the last gripper command confirms it is holding something.

        Raises GripFailed immediately if the jaws settled with nothing between
        them — an empty close is a definite answer, so there is nothing to gain
        by waiting out the timeout. Raises RobotTimeout if they never settle.

        This is the safe idiom for a pick, and it is deliberately shorter than
        the unsafe one:

            robot.gripper_close()
            robot.wait_holding_part()        # missed grips cannot be ignored
            robot.move_joints(lift)

        The alternative — wait, then check gripper_result() — works, but relies
        on remembering to write the check.
        """
        self.wait_gripper_move_complete(timeout)
        result = self.gripper_result()
        if result is None or not result.object_detected:
            raise GripFailed("holding a part", result)
        return result

    def wait_released_part(self, timeout: float | None = None) -> GripperResult:
        """Block until the last gripper command confirms the part is released.

        The mirror of wait_holding_part(): raises GripFailed if the jaws stopped
        because something was still between them, which on an open means the
        part did not come free — worth catching before the arm travels away with
        it still attached.
        """
        self.wait_gripper_move_complete(timeout)
        result = self.gripper_result()
        if result is None or not result.reached_target:
            raise GripFailed("released and at target", result)
        return result

    def wait(self, predicate: Callable[[dict], bool], timeout: float | None = WAIT_STATE_TIMEOUT, what: str = 'wait') -> None:
        """Block until predicate(status) is true, event-driven on the status stream.

        status keys: mode, state, homed, err, safety, estop.
        """
        if self._conn.wait_status(predicate, timeout) is None:
            raise RobotTimeout(f"{what} timed out")

    def wait_enabled(self, timeout: float | None = WAIT_STATE_TIMEOUT) -> None:
        self.wait(lambda s: s.get("state") == "ENABLED", timeout, "wait_enabled")

    def wait_mode(self, mode: str, timeout: float | None = WAIT_STATE_TIMEOUT) -> None:
        self.wait(lambda s: s.get("mode") == mode, timeout, f"wait_mode({mode})")

    def wait_homed(self, timeout: float | None = WAIT_HOMED_TIMEOUT) -> None:
        """Wait until homed and not currently homing (guards a stale HOMED flag). HomingFailed when the homing ends in
        an error instead."""
        self._wait_homed(timeout, "wait_homed")

    def wait_error_reset(self, timeout: float | None = WAIT_STATE_TIMEOUT) -> None:
        """Wait until no error is latched (after clear_errors(), or someone pressing Clear Errors). 2026-10-15."""
        self.wait(lambda s: not s.get("err") and not s.get("errors"), timeout, "wait_error_reset")

    def errors(self) -> list[dict]:
        """The latched errors from the latest status: [{"key", "number", "title", "level", "since"}] (2026-10-15;
        [] when clean). `key` is "J3:WATCHDOG" style; the number and title are the catalog's (docs/error_codes.md)."""
        return list(self.latest_status().get("error_details") or [])

    # ------------------------------------------------------------------ #
    # Data queries (immediate, never block)
    # ------------------------------------------------------------------ #

    def get_joints(self) -> list[float]:
        """Joint angles in degrees."""
        return self._immediate("get_joints")

    def get_pose(self, *, angle_unit: str = "rad") -> list[float]:
        """TCP pose [x, y, z, rx, ry, rz]: metres, the angles in radians (or degrees with angle_unit="deg"),
        R = Rx(rx) Ry(ry) Rz(rz)."""
        return _pose_out(self._immediate("get_pose"), angle_unit)

    def get_health(self) -> dict:
        """The robot's health summary (2026-10-15): {"level": "OK" | "WARN" | "ERROR" | "STALE", "components":
        [{"name", "level", "summary", "values"}]} — the control loop, CAN, drives, errors, homing, the executor, the
        self-collision worker, the processes, disk, memory, CPU temperature, the clock."""
        return self._immediate("get_health")

    def get_hardware(self, can: bool = True) -> dict:
        """What the robot's hardware reports (2026-10-18; the first-run check's data): {"drives": [{"name", "can_id",
        "answered_scan", "answering", "lost", "firmware", "hardware", "serial", "batch", "voltage_v", "temperature_c",
        "fault", "flags", "calibrated"}], "gripper" (the same, or None), "scan_unexpected_ids", "can_link" (the CAN
        interface's state and error counters, or None), "io": {"estop_pressed", "estop_raw", "buttons_pressed",
        "inputs", "outputs", ...}, "tick", "time"}. can=False skips the CAN counters (~5 ms; for polling). Moves
        nothing."""
        return self._immediate("get_hardware", [1 if can else 0])

    def get_versions(self) -> dict:
        """Every part's version (2026-10-21): {"runtime": {"version", "branch", "commit"}, "python", "os": {"name",
        "kernel", "realtime"}, "description_package": {"version", "path"}, "drives": [{"joint", "can_id", "firmware",
        "hardware", "serial", "batch"}], "gripper" (the same without "batch", or None), "control_box":
        {"pcb_configured", "hardware", "software", "serial", "batch"}, "time"}. A value a part has not reported yet is
        None. Moves nothing."""
        return self._immediate("get_versions")

    def set_log_level(self, name: str, level: str) -> Any:
        """Set one logger's level on the robot without a restart (2026-10-15): `name` from docs/logging.md "Logger
        names" ("rti.can", "exec.planner", ...), `level` DEBUG / INFO / WARNING / ERROR. Applied in RTI and the
        executor; not persisted (a restart returns to INFO)."""
        return self._immediate("set_log_level", [str(name), str(level).upper()])

    def capture_telemetry(self, seconds: float = 10.0, name: str = "") -> Any:
        """Record `seconds` (1-60) of the robot's per-tick channels into a file on the robot
        (temp_data/captures/<UTC time>_<name>.npz; the dashboard's Recordings tab > Captures shows and downloads it; the
        activity log says CAPTURE_SAVED when it is done). The loop does no extra work. 2026-10-15."""
        return self._immediate("capture_telemetry", [float(seconds), str(name)])

    def stop_capture(self) -> Any:
        """End the running capture now: the robot saves what it has recorded so far (the activity log says
        CAPTURE_SAVED, "stopped early"; CAPTURE_REFUSED when none runs). 2026-10-16."""
        return self._immediate("stop_capture")

    def get_status(self) -> dict:
        """One-shot status query over TCP: mode, state, homed, error_active, joints."""
        return self._immediate("get_status")

    def is_reachable(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, near: Sequence[float] | None = None,
                     *, angle_unit: str = "rad") -> dict:
        """Can the arm reach this TCP pose inside its joint limits? Moves nothing.

        Pose as move_pose / get_pose: [m, rad — or deg with angle_unit="deg"]; `near` is joint angles in deg either
        way. Returns
        {"reachable": bool, "joints": [deg] | None, "allowed": bool, "zone": name | None,
         "self_collision": "upper arm and tool" | None}:
        `joints` is the solution closest to `near` (joint angles in deg; default: where the arm is
        now) among those clear of self-collision (the planner's margin); `allowed` is reachable AND
        inside every stop-zone of the active cell (`zone` names the one it violates) AND clear of
        self-collision (`self_collision` names two links only when EVERY solution found brings them
        within the margin — `joints` is then the closest of those). A global search, like an industrial
        controller's "does an IK solution exist" query: "no" means no configuration inside the limits reaches the pose.
        move_pose only searches near its start, so it can still refuse a pose this calls reachable
        (IK_POSE_ERROR); move_joints(joints) will not, unless its PATH self-collides (the end pose
        is all this checks).
        """
        args = _pose_in((x, y, z, rx, ry, rz), angle_unit)
        if near is not None:
            args += [float(a) for a in near]
        return self._immediate("is_reachable", args)

    def get_zones(self) -> dict:
        """The safety zones in force: {"cell", "zones": [...], "tool_spheres": [...]}. A read."""
        return self._immediate("get_zones")

    def reload_zones(self) -> dict:
        """Re-read <active_cell> and its cell file and apply them (planner, is_reachable, RTI's
        monitor) without a restart. A bad file raises CommandRejected ZONE_CONFIG and the zones in
        force stay as they were. Returns the same summary as get_zones()."""
        return self._immediate("reload_zones")

    def get_joint_walls(self) -> dict:
        """The freedrive joint walls (docs/safety_zones.md §2b). A read. Returns {"walls": [...], "policy",
        "robot_xml"}; each wall (joint 1-based) carries what is in force — at, lo_deg/hi_deg, frequency_hz,
        damping_ratio, max_torque_nm, breakthrough_deg, freedrive — the stable ranges (frequency_max_hz,
        damping_ratio_window at the current frequency, the same for every joint), the stiffness/damping in Nm/deg that
        gives at the joint's lightest pose and the pose scale at the measured pose now (scale_here), the live `spec`
        (None = auto) and `unsaved` (live differs from the robot XML)."""
        return self._immediate("get_joint_walls")

    def set_joint_wall(self, joint: int, **fields: Any) -> Any:
        """Change one joint's freedrive wall LIVE (joint 1-based). Fields: at ("soft" | "hard"), frequency_hz,
        damping_ratio, filter_ms (0-8, the speed filter behind the damping), max_torque_nm, breakthrough_deg, freedrive
        ("warn" | "stop"), scaling ("adaptive" | "fixed": stiffness follows the pose's inertia, or stays at the lightest
        pose's); "auto" puts a field back to its default. Fields not given keep their value. A frequency /
        damping ratio outside the stable range for that filter raises CommandRejected WALL_UNSTABLE (the message
        gives the range) and nothing changes. Not saved until
        save_joint_walls(). Returns the same as get_joint_walls()."""
        return self._immediate("set_joint_wall", [int(joint), dict(fields)])

    def save_joint_walls(self) -> Any:
        """Write the live freedrive walls into the robot XML (<freedrive_wall> per joint). Returns get_joint_walls()."""
        return self._immediate("save_joint_walls")

    # ------------------------------------------------------------------ #
    # Digital IO (immediate, isolated pins 1-3)
    # ------------------------------------------------------------------ #

    def set_digital_output(self, pin: int, value: bool | int) -> None:
        return self._immediate("set_digital_output", [int(pin), bool(value)])

    def get_digital_input(self, pin: int) -> bool:
        return self._immediate("get_digital_input", [int(pin)])

    # ------------------------------------------------------------------ #
    # Context manager
    # ------------------------------------------------------------------ #

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.disconnect()
        return False
