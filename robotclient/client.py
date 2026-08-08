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

Angles are DEGREES at the API (converted to radians server-side).
"""

from .connection import Connection
from .commands import generate_id
from .constants import (
    DEFAULT_PORT, IMMEDIATE_TIMEOUT, ACK_TIMEOUT, DEFAULT_MOVE_TIMEOUT,
    WAIT_STATE_TIMEOUT, WAIT_HOMED_TIMEOUT,
)
from .exceptions import RobotTimeout, MotionFailed, GripFailed


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
    def object_detected(self):
        """True if something stopped the jaws (codes 1 and 2)."""
        return self.code in (1, 2)

    @property
    def reached_target(self):
        """True if the gripper reached its commanded position with nothing in the way."""
        return self.code == 3

    @property
    def name(self):
        return self._NAMES.get(self.code, f"unknown({self.code})")

    def __repr__(self):
        return f"GripperResult(code={self.code}, {self.name!r})"


class RobotClient:
    def __init__(self, sync=False):
        self._conn = Connection()
        self._sync = sync
        self._last_queued = None   # most recent queued command's _Pending, for wait_motion_complete()
        # Most recent GRIPPER command's _Pending, tracked separately: queue a
        # gripper command then a joint move and _last_queued is the MOVE, so
        # wait_motion_complete() would wait for the wrong thing.
        self._last_gripper_queued = None

    # ------------------------------------------------------------------ #
    # Connection
    # ------------------------------------------------------------------ #

    def connect(self, ip, port=DEFAULT_PORT):
        self._conn.connect(ip, port)
        # The executor pushes a status snapshot on connect; wait briefly for it so
        # status() is populated and the first wait_*() has a baseline.
        self._conn.wait_status(lambda s: bool(s), timeout=1.0)
        return self

    def disconnect(self):
        self._conn.disconnect()

    @property
    def connected(self):
        return self._conn.connected

    def on_error(self, fn):
        self._conn.on_error = fn

    def on_status_change(self, fn):
        self._conn.on_status_change = fn

    def on_motion_complete(self, fn):
        """fn(command_id) called whenever any queued motion command completes successfully."""
        self._conn.on_motion_complete = fn

    def on_motion_error(self, fn):
        """fn({"id", "code", "message"}) called when a queued motion command fails.

        Fires as soon as the failure arrives. wait_idle() also raises MotionFailed for
        the same failures — use this callback to react immediately (e.g. abort a
        toolpath) rather than at the next synchronization point."""
        self._conn.on_motion_error = fn

    def status(self):
        """Latest status snapshot from the event stream: mode, state, homed, err, ..."""
        return self._conn.latest_status()

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

    def enable(self, wait=None):
        self._state_change("enable", wait, lambda s: s.get("state") == "ENABLED",
                           what="wait_enabled")

    def disable(self, wait=None):
        self._state_change("disable", wait, lambda s: s.get("state") == "DISABLED")

    def clear_errors(self, wait=None):
        self._state_change("clear_errors", wait,
            lambda s: s.get("err", 0) == 0 and s.get("mode") != "ACTIVE_ERROR")

    def enter_exec_mode(self, wait=None):
        self._state_change("enter_exec_mode", wait, lambda s: s.get("mode") == "EXEC",
                           what="wait_mode(EXEC)")

    # ------------------------------------------------------------------ #
    # RTI mode — the TCP half of the real-time interface.
    # The UDP streaming half lives in RTIClient (rti_client.py); these four just
    # open, arm, disarm and close the session. See TODO/RTI_mode_plan.md §3.
    # ------------------------------------------------------------------ #

    def remote_ip(self):
        """Robot's IP as seen on this connection, or None."""
        return self._conn.remote_ip()

    def local_ip(self):
        """Our IP on the route to the robot, or None. Sent in rti_connect."""
        return self._conn.local_ip()

    def rti_connect(self, pc_ip=None, wait=None):
        """Pair for RTI and enter RTI mode. Streams state; does NOT grant control.

        pc_ip is where the robot will send state packets. Defaults to this socket's
        local address, which is the interface the robot can actually reach.
        """
        if pc_ip is None:
            pc_ip = self.local_ip()
            if pc_ip is None:
                raise RuntimeError("not connected — call connect() first")
        self._immediate("rti_connect", [pc_ip])
        if self._should_wait(wait):
            self.wait(lambda s: s.get("mode") == "RTI", what="wait_mode(RTI)")

    def rti_claim(self):
        """Claim RT control. Timing rules start: a 100 ms command gap stops the robot.

        Rejected if an error is latched or the robot is not already paired.
        """
        self._immediate("rti_claim")

    def rti_release(self):
        """Release control cleanly, staying paired. Robot holds position for 0.5 s."""
        self._immediate("rti_release")

    def rti_disconnect(self, wait=None):
        """Unpair and return to IDLE."""
        self._immediate("rti_disconnect")
        if self._should_wait(wait):
            self.wait(lambda s: s.get("mode") == "IDLE", what="wait_mode(IDLE)")

    def rti_set_rate_limiter(self, enabled):
        """Enable/disable acceleration, jerk and torque-rate clamping on RTI commands.

        ON by default. Turning it off streams your commands to the motors untouched —
        appropriate for measuring true loop latency, not for normal operation.
        """
        self._immediate("rti_set_rate_limiter", [1 if enabled else 0])

    def rti_set_lowpass(self, cutoff_hz):
        """Set the command low-pass cutoff in Hz. 0 disables. **Default: OFF.**

        A one-pole low-pass, defaulted OFF. Scaling the usual "cutoff at 10% of the
        control rate" convention to our loop puts it at 25 Hz, whose time constant
        (6.4 ms) is ~4x the absolute phase lag it would cost on a 1 kHz controller
        (1.6 ms) purely because our loop runs at 250 Hz — a real cost to hand an
        external control loop without being asked. Enable it if your command stream is
        noisy; 25 Hz is the rate-matched starting point.
        """
        self._immediate("rti_set_lowpass", [float(cutoff_hz)])

    def rti_set_start_pose_check(self, enabled):
        """Enable/disable validation of the first setpoint after rti_claim.

        ON by default. It is what stops a controller that initialises to zeros from
        commanding a step move across the workspace. Takes effect on the next claim.
        """
        self._immediate("rti_set_start_pose_check", [1 if enabled else 0])

    def rti_set_gravity_comp(self, enabled):
        """Enable/disable gravity-compensation feedforward on RTI commands.

        **ON by default** — the robot carries its own weight, as it does in IDLE, JOG and
        EXEC. Turn it off only if your controller computes gravity itself, since a torque
        controller that also receives ours would apply it twice.

        This is RTI-scoped. It does not change the robot-wide gravity flag, so IDLE and JOG
        are unaffected when you disconnect; that flag still overrides this one if the
        operator has switched compensation off globally.
        """
        self._immediate("rti_set_gravity_comp", [1 if enabled else 0])

    def set_recipe(self, name):
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

    def stop(self, wait=None):
        self._state_change("stop", wait, lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")
        # stop() cancels queued motion server-side, which replies "error"/STOPPED for
        # each one. Those are the expected consequence of stopping, not a motion
        # failure — drop them so they can't surface at a later wait_idle(). Safe to do
        # here: the executor writes those replies before stop's own "ok", so the
        # receiver thread has already recorded them by the time _state_change returns.
        self._conn.take_motion_errors()

    def home(self, wait=None):
        self._immediate("home")
        if self._should_wait(wait):
            # Homing starts a tick later; wait for it to begin, then to finish. The
            # homed flag can read stale, so completion is keyed off mode leaving HOMING.
            try:
                self.wait_mode("HOMING", timeout=3.0)
            except RobotTimeout:
                pass   # never entered HOMING (already homed / instant) — check below
            self.wait(lambda s: s.get("homed") is True and s.get("mode") != "HOMING",
                      WAIT_HOMED_TIMEOUT, "home")

    def pause(self, wait=None):
        """Freeze motion in place (only valid mid-EXEC). Resume with resume()."""
        self._state_change("pause", wait, lambda s: bool(s.get("exec_paused")))

    def resume(self, wait=None):
        """Continue motion frozen by pause(), exactly where it left off."""
        self._state_change("resume", wait, lambda s: not s.get("exec_paused"))

    def set_joint_speed(self, percent):
        return self._immediate("set_joint_speed", [float(percent)])

    def set_blending(self, percent):
        return self._immediate("set_blending", [float(percent)])

    def set_profile(self, profile):
        return self._immediate("set_profile", [str(profile).strip().lower()])

    def set_default_profile(self, profile):
        return self.set_profile(profile)

    def set_default_ik_solver(self, solver):
        return self._immediate("set_default_ik_solver", [str(solver).strip().lower()])

    def set_ik_solver(self, solver):
        return self.set_default_ik_solver(solver)

    def set_completion_policy(self, policy):
        return self._immediate("set_completion_policy", [str(policy).strip().lower()])

    # ------------------------------------------------------------------ #
    # Jog + live config (immediate; migrated from the GUI's old direct-to-RTI
    # push — see TODO/gui_executor_migration_plan.md). Jog forwards to RTI's
    # persistent JogHandler and returns on the executor ack, not on physical
    # completion, so a hold-to-jog caller drives its own cadence.
    # ------------------------------------------------------------------ #

    def enter_jog_mode(self, wait=None):
        """Enter JOG mode. Requires the robot ENABLED and homed, like enter_exec_mode."""
        self._state_change("enter_jog_mode", wait, lambda s: s.get("mode") == "JOG",
                           what="wait_mode(JOG)")

    def enter_idle_mode(self, wait=None):
        """Return to IDLE (always allowed). Unlike stop(), does NOT flush queued motion."""
        self._state_change("enter_idle_mode", wait, lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")

    # ------------------------------------------------------------------ #
    # Firmware flashing — see docs/firmware_flashing.md.
    # FLASHING makes RTI go silent on the CAN bus so a separate flasher process
    # can own it, while the loop keeps running (E-stop, IO and telemetry stay
    # live). It does NOT flash anything itself; it grants the bus.
    # ------------------------------------------------------------------ #
    def safety_stop(self, wait=None):
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

    def release_safety_stop(self, wait=None):
        """Leave SAFETY_STOP for IDLE, restoring the normal hold."""
        self._state_change("release_safety_stop", wait,
                           lambda s: s.get("mode") == "IDLE",
                           what="wait_mode(IDLE)")

    def enter_flashing_mode(self, parked_confirmed=False, force=False, wait=None):
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

    def exit_flashing_mode(self, wait=None):
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

    def jog(self, joint, direction, speed):
        """Start jogging one joint. direction: +1 / -1. speed: fraction of the jog speed
        limit. Only valid in JOG mode; call jog_stop() to halt."""
        return self._immediate("jog_set", [int(joint), int(direction), float(speed)])

    def jog_stop(self):
        """Halt jog motion. Accepted in any mode, so a released button always stops."""
        return self._immediate("jog_stop")

    def set_jog_accel(self, accel_time):
        """Jog acceleration ramp time (s); persists on RTI's JogHandler."""
        return self._immediate("jog_accel", [float(accel_time)])

    def set_jog_profile(self, profile):
        """Jog motion profile, e.g. 'scurve' / 'trapezoid'."""
        return self._immediate("jog_profile", [str(profile).strip()])

    def set_jog_jerk_factor(self, factor):
        """Jog jerk factor (used by the s-curve profile)."""
        return self._immediate("jog_jerk_factor", [float(factor)])

    def set_jog_control_mode(self, mode):
        """Jog control mode, e.g. 'pid'."""
        return self._immediate("jog_control_mode", [str(mode).strip()])

    def set_gravity_comp(self, enabled):
        """Enable/disable gravity-compensation torque. Sends an int 0/1 (never a bool):
        RTI decides ON by comparing the value to the string '1'."""
        return self._immediate("gravity_comp", [1 if enabled else 0])

    def set_pid_gains(self, joint, **gains):
        """Set control gains for one joint (or the gripper, joint=<joint_count>). Keys:
        KPP, KPV, KIV, KPIQ, KIIQ, KP, KD, Ilim, vel_lim — unknown keys are dropped by the
        executor. Forwarded as a dict; RTI reads the fields off the command dict."""
        payload = {"joint": int(joint)}
        payload.update({k: float(v) for k, v in gains.items()})
        return self._immediate("set_pid_gains", payload)

    def gripper_calibrate(self):
        """Run the gripper calibration sweep (immediate; requires a CAN gripper)."""
        return self._immediate("gripper_calibrate")

    def set_gripper_ctrl_mode(self, mode):
        """Gripper control mode: 1 = motor mode, 2 = firmware mode."""
        return self._immediate("gripper_ctrl_mode", [int(mode)])

    def set_gripper(self, position, speed=100, current=200, action=1):
        """Firmware/motor-mode direct gripper target (immediate; distinct from the queued
        gripper_open / gripper_close / move_gripper). position/speed 0-255 raw gripper scale,
        current in mA, action 0/1. Requires a CAN gripper (RTI ignores it otherwise)."""
        return self._immediate("gripper_set",
                               [int(position), int(speed), int(current), int(action)])

    # ------------------------------------------------------------------ #
    # Motion (queued)
    # ------------------------------------------------------------------ #

    def move_joints(self, *angles_deg, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        if len(angles_deg) == 1 and isinstance(angles_deg[0], (list, tuple)):
            angles_deg = tuple(angles_deg[0])
        return self._queued("move_joints", [float(a) for a in angles_deg], timeout, options)

    def move_pose(self, x, y, z, rx, ry, rz, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Endpoint IK move to TCP pose [m, rad]; timed as a joint-space move."""
        return self._queued("move_pose", [float(v) for v in (x, y, z, rx, ry, rz)], timeout, options)

    def move_linear(self, x, y, z, rx, ry, rz, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Cartesian line move; pass dry_run=True to validate the IK path without motion."""
        return self._queued("move_linear", [float(v) for v in (x, y, z, rx, ry, rz)], timeout, options)

    def move_linear_rel_world(self, dx, dy, dz, drx=0.0, dry=0.0, drz=0.0,
                              timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Cartesian line by world-frame TCP offset [m, rad]."""
        return self._queued(
            "move_linear_rel_wrf",
            [float(v) for v in (dx, dy, dz, drx, dry, drz)], timeout, options)

    def move_linear_rel_tool(self, dx, dy, dz, drx=0.0, dry=0.0, drz=0.0,
                             timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Cartesian line by tool-frame TCP offset [m, rad]."""
        return self._queued(
            "move_linear_rel_trf",
            [float(v) for v in (dx, dy, dz, drx, dry, drz)], timeout, options)

    def move_linear_rel_wrf(self, *args, **kwargs):
        return self.move_linear_rel_world(*args, **kwargs)

    def move_linear_rel_trf(self, *args, **kwargs):
        return self.move_linear_rel_tool(*args, **kwargs)


    @staticmethod
    def _curve_pose_values(pose, label):
        values = [float(v) for v in pose]
        if len(values) != 6:
            raise ValueError(f"{label} needs [x, y, z, rx, ry, rz]")
        return values

    def move_arc(self, via, end, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Absolute world-frame circular arc through via and end poses [m, rad]."""
        args = self._curve_pose_values(via, "move_arc via")
        args.extend(self._curve_pose_values(end, "move_arc end"))
        return self._queued("move_arc", args, timeout, options)

    def move_arc_rel_world(self, via, end, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Arc with via/end offsets in the world frame at command start [m, rad]."""
        args = self._curve_pose_values(via, "move_arc_rel_world via")
        args.extend(self._curve_pose_values(end, "move_arc_rel_world end"))
        return self._queued("move_arc_rel_wrf", args, timeout, options)

    def move_arc_rel_tool(self, via, end, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Arc with via/end offsets in the starting TCP frame [m, rad]."""
        args = self._curve_pose_values(via, "move_arc_rel_tool via")
        args.extend(self._curve_pose_values(end, "move_arc_rel_tool end"))
        return self._queued("move_arc_rel_trf", args, timeout, options)

    def move_circle(self, center, normal, turns=1.0, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Full circle around an absolute world-frame center and normal."""
        center_values = [float(v) for v in center]
        normal_values = [float(v) for v in normal]
        if len(center_values) != 3 or len(normal_values) != 3:
            raise ValueError("move_circle center and normal each need 3 values")
        return self._queued(
            "move_circle", center_values + normal_values + [float(turns)], timeout, options)

    def move_circle_rel_world(self, center, normal, turns=1.0,
                              timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Circle with center offset and normal expressed in the world frame."""
        center_values = [float(v) for v in center]
        normal_values = [float(v) for v in normal]
        if len(center_values) != 3 or len(normal_values) != 3:
            raise ValueError("move_circle_rel_world center and normal each need 3 values")
        return self._queued(
            "move_circle_rel_wrf", center_values + normal_values + [float(turns)], timeout, options)

    def move_circle_rel_tool(self, center, normal, turns=1.0,
                             timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Circle with center offset and normal expressed in the starting TCP frame."""
        center_values = [float(v) for v in center]
        normal_values = [float(v) for v in normal]
        if len(center_values) != 3 or len(normal_values) != 3:
            raise ValueError("move_circle_rel_tool center and normal each need 3 values")
        return self._queued(
            "move_circle_rel_trf", center_values + normal_values + [float(turns)], timeout, options)

    def move_spline(self, waypoints, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Absolute world-frame interpolating cubic spline through poses [m, rad]."""
        poses = [self._curve_pose_values(pose, "move_spline waypoint") for pose in waypoints]
        if len(poses) < 2:
            raise ValueError("move_spline needs at least two waypoint poses")
        return self._queued("move_spline", [v for pose in poses for v in pose], timeout, options)

    def move_spline_rel_world(self, waypoints, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Spline through world-frame offsets from the TCP at command start [m, rad]."""
        poses = [self._curve_pose_values(pose, "move_spline_rel_world waypoint") for pose in waypoints]
        if len(poses) < 2:
            raise ValueError("move_spline_rel_world needs at least two waypoint poses")
        return self._queued("move_spline_rel_wrf", [v for pose in poses for v in pose], timeout, options)

    def move_spline_rel_tool(self, waypoints, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        """Spline through starting-TCP-frame offsets; waypoints are not chained [m, rad]."""
        poses = [self._curve_pose_values(pose, "move_spline_rel_tool waypoint") for pose in waypoints]
        if len(poses) < 2:
            raise ValueError("move_spline_rel_tool needs at least two waypoint poses")
        return self._queued("move_spline_rel_trf", [v for pose in poses for v in pose], timeout, options)

    move_arc_rel_wrf = move_arc_rel_world
    move_arc_rel_trf = move_arc_rel_tool
    move_circle_rel_wrf = move_circle_rel_world
    move_circle_rel_trf = move_circle_rel_tool
    move_spline_rel_wrf = move_spline_rel_world
    move_spline_rel_trf = move_spline_rel_tool

    def delay(self, seconds, timeout=DEFAULT_MOVE_TIMEOUT, **options):
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

    def gripper_open(self, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        return self._queued_gripper("gripper_open", [], timeout, options)

    def gripper_close(self, timeout=DEFAULT_MOVE_TIMEOUT, **options):
        return self._queued_gripper("gripper_close", [], timeout, options)

    def gripper_result(self):
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

    def move_gripper(self, position, speed=None, current=None, timeout=DEFAULT_MOVE_TIMEOUT, **options):
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

    def wait_idle(self, timeout=None, raise_on_error=True):
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

    def wait_motion_complete(self, timeout=None):
        """Block until the most recently queued command completes — NOT the whole
        queue (see wait_idle for that). No-op if nothing has been queued yet."""
        if self._last_queued is None:
            return
        Connection.wait_done(self._last_queued, timeout or DEFAULT_MOVE_TIMEOUT)

    def wait_gripper_move_complete(self, timeout=None):
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

    def wait_holding_part(self, timeout=None):
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

    def wait_released_part(self, timeout=None):
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

    def wait(self, predicate, timeout=WAIT_STATE_TIMEOUT, what="wait"):
        """Block until predicate(status) is true, event-driven on the status stream.

        status keys: mode, state, homed, err, safety, estop.
        """
        if self._conn.wait_status(predicate, timeout) is None:
            raise RobotTimeout(f"{what} timed out")

    def wait_enabled(self, timeout=WAIT_STATE_TIMEOUT):
        self.wait(lambda s: s.get("state") == "ENABLED", timeout, "wait_enabled")

    def wait_mode(self, mode, timeout=WAIT_STATE_TIMEOUT):
        self.wait(lambda s: s.get("mode") == mode, timeout, f"wait_mode({mode})")

    def wait_homed(self, timeout=WAIT_HOMED_TIMEOUT):
        """Wait until homed and not currently homing (guards a stale HOMED flag)."""
        self.wait(lambda s: s.get("homed") is True and s.get("mode") != "HOMING",
                  timeout, "wait_homed")

    # ------------------------------------------------------------------ #
    # Data queries (immediate, never block)
    # ------------------------------------------------------------------ #

    def get_joints(self):
        """Joint angles in degrees."""
        return self._immediate("get_joints")

    def get_pose(self):
        """TCP pose [x, y, z, rx, ry, rz]."""
        return self._immediate("get_pose")

    def get_status(self):
        """One-shot status query over TCP: mode, state, homed, error_active, joints."""
        return self._immediate("get_status")

    # ------------------------------------------------------------------ #
    # Digital IO (immediate, isolated pins 1-3)
    # ------------------------------------------------------------------ #

    def set_digital_output(self, pin, value):
        return self._immediate("set_digital_output", [int(pin), bool(value)])

    def get_digital_input(self, pin):
        return self._immediate("get_digital_input", [int(pin)])

    # ------------------------------------------------------------------ #
    # Context manager
    # ------------------------------------------------------------------ #

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.disconnect()
        return False
