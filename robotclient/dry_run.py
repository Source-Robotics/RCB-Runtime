"""Offline EXEC planner client for previewing programs without RTI, ZMQ, or CAN."""
from __future__ import annotations
import math
from typing import Any, Callable, Sequence

from dataclasses import dataclass, replace
from queue import Empty, Queue

import numpy as np

import config.xml_parser as xml_parser
from communication.protocol import canonical_command
from motion.cartesian_paths import pose_from_xyz_rpy
from motion.exec_commands import (DEFAULT_PROFILE, MotionOptions, QueuedCommand, blend_problem,
                                  profile_option_problem)
from motion.reachability import find_reachable_clear, make_starts
from utility.error_catalog import entry as catalog_entry
from utility.zones import reach_reply, zones_summary
from motion.trajectory_planner import PlannerError, TrajectoryPlanner
from project_paths import CELLS_DIR, GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
from robotics.robot_kinematics import (
    calculate_forward_kinematics,
    calculate_jacobian,
    get_tcp_pose_rpy,
)
from utility.kinematics_backend import POSE_CONVENTION, set_kinematics_backend
from utility.pinocchio_attach import attach_pinocchio
from utility.robot_factory import create_robot

from .client import RobotClient, _pose_in, _pose_out
from .commands import generate_id
from .connection import Connection
from .exceptions import CommandRejected, MotionFailed, RobotConnectionError, RobotTimeout


# ── The executor's queue-time refusals (review P4, 2026-11-22) ─────────────────────────────────────────────────────
# command_executor.py cannot be imported offline (it starts its servers on import), so these are COPIES of its
# originals, each named where it stands. dummy_test_scripts/test_review_batch17_client.py lifts the originals from
# the executor's source and keeps the two equal on a table of cases: change both together.

# = command_executor._UNSUPPORTED_PER_COMMAND_OPTIONS: a session setting, refused per move (BAD_OPTIONS)
_UNSUPPORTED_PER_COMMAND_OPTIONS = {
    "completion_policy": "use set_completion_policy() instead of a per-command option",
}


def _reject_unsupported_per_command_options(option_overrides):
    """= command_executor._reject_unsupported_per_command_options: a ValueError (BAD_OPTIONS) naming each session
    setting given per move."""
    bad = sorted(set(option_overrides or {}) & set(_UNSUPPORTED_PER_COMMAND_OPTIONS))
    if not bad:
        return
    details = "; ".join(f"{key}: {_UNSUPPORTED_PER_COMMAND_OPTIONS[key]}" for key in bad)
    raise ValueError(details)


# = command_executor._POSE_ARG_COMMANDS
_POSE_ARG_COMMANDS = ("move_pose", "move_linear", "move_linear_rel_world", "move_linear_rel_tool")


def _queued_args_problem(cmd, args, joint_num):
    """= command_executor._queued_args_problem, the robot's joint count an argument here (the executor's JOINT_NUM):
    why a queued move's values cannot be planned (BAD_ARGS: too few, or one that is no finite number), or None."""
    if cmd == "move_joints":
        need, what = joint_num, f"{joint_num} joint angles in degrees"
    elif cmd in _POSE_ARG_COMMANDS:
        need, what = 6, "a pose: x, y, z in metres and rx, ry, rz"
    else:
        return None
    if not isinstance(args, (list, tuple)) or len(args) < need:
        got = f"{len(args)} value(s)" if isinstance(args, (list, tuple)) else repr(args)
        return f"{cmd} needs {what}; got {got}"
    for v in args[:need]:
        try:
            ok = not isinstance(v, bool) and math.isfinite(float(v))
        except (TypeError, ValueError):
            ok = False
        if not ok:
            return f"{cmd}: {v!r} is not a finite number ({what})"
    return None


_QUEUED_GRIPPER_COMMANDS = ("open_gripper", "close_gripper", "move_gripper")   # = command_executor's
# = command_executor._queued_gripper_problem's NO_GRIPPER text (the gripper XML's <CAN_gripper>, read as the executor)
_NO_GRIPPER = "this robot has no CAN gripper (the active gripper XML's <CAN_gripper> is not 1)"
# = the keys of command_executor._DIGITAL_OUTPUT_PINS / _DIGITAL_INPUT_PINS: the control box's isolated I/O (BAD_PIN)
_IO_PINS = (1, 2, 3)


def _motion_aborted_text(aborted):
    """= command_executor._submit_queued's MOTION_ABORTED message, from the planner's abort_info()."""
    why = (f"motion is aborted: command {aborted['cid']} failed ({aborted['code']})"
           if aborted.get("cid") else f"motion is aborted: {aborted['message']}")
    return why + " — clear errors (clear_errors / the Clear Errors button) to run motion again"


def _catalog_fields(code):
    """The error catalog's number, title and fix of a code, as the executor's reply() adds them to an error."""
    cat = catalog_entry(code) or {}
    return {k: cat[k] for k in ("number", "title", "fix") if cat.get(k) is not None}


def _rejected(code, message):
    """A refusal as the live client raises it: CommandRejected with the catalog's number, title and fix."""
    return CommandRejected(code, message, **_catalog_fields(code))


class _NoRobot(Connection):
    """The dry run's connection: no robot behind it (review P4, 2026-11-22). The client state the inherited methods
    read works — no address, no gripper result —; a command that needs the robot raises RobotConnectionError naming
    it (before, the dry-run client had no connection at all: an AttributeError)."""

    def send_command(self, cid, cmd, args=None, options=None):
        raise RobotConnectionError(f"{cmd}: a dry run has no robot to send it to (it plans moves offline)")


@dataclass(frozen=True)
class DryRunEvent:
    """A non-trajectory program event placed on the virtual timeline."""

    command_id: str
    sequence: int
    command: str
    time_s: float
    details: dict


@dataclass(frozen=True)
class DryRunError:
    """Planner failure retained in the preview instead of sent to an executor."""

    command_id: str
    sequence: int
    command: str
    code: str
    message: str

    def failure(self):
        """This error as the live client reports a failure (wait_idle(), MotionFailed, on_motion_error()): "id",
        "code", "message" and the error catalog's "number", "title" and "fix", as the robot sends them (the catalog's
        since review P4, 2026-11-22) — plus the preview's own fields. MotionFailed formats f["id"]: a bare __dict__
        (command_id, no id) raised a KeyError instead of MotionFailed (found 2026-10-05 by the block editor's Check);
        every failure record of the dry-run client is built here."""
        return {"id": self.command_id, **self.__dict__, **_catalog_fields(self.code)}


@dataclass(frozen=True)
class DryRunSegment:
    """One planned command's timestamped joint and TCP samples."""

    command_id: str
    sequence: int
    command: str
    profile: str
    start_time_s: float
    end_time_s: float
    time_s: np.ndarray
    joint_position_rad: np.ndarray
    joint_velocity_rad_s: np.ndarray
    joint_acceleration_rad_s2: np.ndarray
    joint_jerk_rad_s3: np.ndarray
    tcp_pose: np.ndarray
    tcp_velocity: np.ndarray
    tcp_acceleration: np.ndarray
    blend_zones: tuple[dict, ...]
    diagnostic: dict | None


@dataclass(frozen=True)
class DryRunPlan:
    """Complete offline program preview returned by DryRunRobotClient."""

    segments: tuple[DryRunSegment, ...]
    events: tuple[DryRunEvent, ...]
    errors: tuple[DryRunError, ...]
    duration_s: float
    final_joints_rad: np.ndarray

    @property
    def valid(self):
        return not self.errors

    def require_valid(self):
        if self.errors:
            raise MotionFailed([error.failure() for error in self.errors])
        return self


def _finite_difference(values, dt):
    values = np.asarray(values, dtype=np.float64)
    if values.shape[0] < 2:
        return np.zeros_like(values)
    return np.gradient(values, float(dt), axis=0, edge_order=1)


class _OfflineTrajectoryPlanner(TrajectoryPlanner):
    """TrajectoryPlanner with its live output boundary replaced by memory."""

    def __init__(self, initial_joints_rad, joint_num, max_vel, max_accel, dt, robot,
                 max_jerk=None, joint_limits=None, gripper_current_limit_ma=None):
        self._initial_joints = np.asarray(initial_joints_rad, dtype=np.float64).copy()
        self._commands = {}
        self._segments = []
        self._events = []
        self._errors = []
        self._timeline_s = 0.0
        super().__init__(
            Queue(), joint_num, max_vel, max_accel, dt,
            read_current_joints=lambda: self._initial_joints,
            robot=robot,
            read_exec_runway=None,
            offline_output=True,
            max_jerk=max_jerk,
            joint_limits=joint_limits,
            gripper_current_limit_ma=gripper_current_limit_ma,
        )

    @property
    def current_joints_rad(self):
        if self._last_endpoint is None:
            return self._initial_joints.copy()
        return self._last_endpoint.copy()

    def enqueue(self, command):
        self._commands[command.id] = command
        self._queue.put_nowait(command)

    def plan_pending(self):
        """Consume the virtual queue with the live planner's blend dispatch rules."""
        while True:
            try:
                command = self._queue.get_nowait()
            except Empty:
                break

            if self._is_blend_eligible(command):
                if (self._blend_buffer and
                        self._blend_lane(self._blend_buffer[0]) != self._blend_lane(command)):
                    self._flush_blend_buffer()
                self._blend_buffer.append(command)
                if len(self._blend_buffer) >= self.MAX_BLEND_LOOKAHEAD:
                    self._flush_blend_buffer()
            elif self._ends_chain(command):
                # the live planner's rule (decision 4a): a blend=0 move is the chain's last member
                self._blend_buffer.append(command)
                self._flush_blend_buffer()
            else:
                self._flush_blend_buffer()
                self._run_single(command)

        self._flush_blend_buffer()

    def reset(self, joints_rad):
        self.flush()
        self.clear_abort()
        self._initial_joints = np.asarray(joints_rad, dtype=np.float64).copy()
        self._segments.clear()
        self._events.clear()
        self._errors.clear()
        self._commands.clear()
        self._timeline_s = 0.0
        self._last_diagnostic = None
        self._last_error = None

    def stop_pending(self, message):
        """The live executor's stop for the commands not planned yet (review P4, 2026-11-22: they vanished, never
        completed nor failed): each fails STOPPED, and nothing aborts — the robot takes new moves at once. The next
        move starts where the planned program ends, as a live Stop holds the arm where it stopped (set_hold_start);
        flush() alone re-read the START joints. Returns the STOPPED errors, in queue order."""
        end = self.current_joints_rad
        pending = list(self._blend_buffer)
        try:
            while True:
                pending.append(self._queue.get_nowait())
        except Empty:
            pass
        self.flush()
        self.set_hold_start(end)
        stopped = [DryRunError(command_id=str(self._cmd_id(c)), sequence=self._cmd_seq(c), command=self._cmd_name(c),
                               code="STOPPED", message=str(message)) for c in pending]
        self._errors.extend(stopped)
        return stopped

    def summary(self):
        self.plan_pending()
        return DryRunPlan(
            segments=tuple(self._segments),
            events=tuple(self._events),
            errors=tuple(self._errors),
            duration_s=float(self._timeline_s),
            final_joints_rad=self.current_joints_rad,
        )

    def _run_single(self, command):
        if self._abort is not None:
            self._report_aborted(command)
            return
        try:
            self._mark_started(command)
            self._plan_and_send(command)
            self._mark_completed(command)
        except PlannerError as error:
            self._mark_error(command, error.code, error.message)
            self._report_error(command.id, error.code, error.message)
        except Exception as error:
            self._mark_error(command, "PLAN_FAIL", str(error))
            self._report_error(command.id, "PLAN_FAIL", str(error))
        finally:
            self._active_command = None

    def _mark_started(self, command):
        self._executing_seq = self._cmd_seq(command)
        self._active_command = self._cmd_name(command)
        self._last_diagnostic = None

    def _mark_completed(self, command):
        self._completed_seq = self._cmd_seq(command)
        self._last_error = None

    def _report_complete(self, command_id):
        return None

    def _report_error(self, command_id, code, message):
        command = self._commands.get(command_id)
        self._errors.append(DryRunError(
            command_id=str(command_id),
            sequence=self._cmd_seq(command) if command is not None else 0,
            command=self._cmd_name(command) if command is not None else "",
            code=str(code),
            message=str(message),
        ))
        # The preview must show what the robot will do: a failure aborts the rest.
        self._command_failed(command_id, code, message)

    def _send_range(self, pos, vel, torque_ff, range_start, range_end, command_id,
                    flags=0, terminal=True):
        if range_end <= range_start:
            return True
        command = self._commands[command_id]
        positions = np.asarray(pos[range_start:range_end], dtype=np.float64).copy()
        velocities = np.asarray(vel[range_start:range_end], dtype=np.float64).copy()
        accelerations = _finite_difference(velocities, self._dt)
        jerks = _finite_difference(accelerations, self._dt)
        tcp_pose, tcp_velocity = self._tcp_samples(positions, velocities)
        tcp_acceleration = _finite_difference(tcp_velocity, self._dt)
        time_s = self._timeline_s + np.arange(len(positions), dtype=np.float64) * self._dt
        duration_s = max(0.0, (len(positions) - 1) * self._dt)
        diagnostic = dict(self._last_diagnostic) if self._last_diagnostic else None
        blend_zones = []
        for zone in (diagnostic or {}).get("blend_zones", ()):
            zone_start = int(zone["start_index"])
            zone_end = int(zone["end_index"])
            start_index = max(int(range_start), zone_start)
            end_index = min(int(range_end) - 1, zone_end)
            if start_index <= end_index:
                blend_zones.append({
                    "corner_index": int(zone["corner_index"]),
                    "kind": str(zone.get("kind", "zone")),
                    "start_index": start_index - int(range_start),
                    "end_index": end_index - int(range_start),
                })
        self._segments.append(DryRunSegment(
            command_id=command_id,
            sequence=command.seq,
            profile=str(self._cmd_options(command).profile),
            command=command.cmd,
            start_time_s=float(self._timeline_s),
            end_time_s=float(self._timeline_s + duration_s),
            time_s=time_s,
            joint_position_rad=positions,
            joint_velocity_rad_s=velocities,
            joint_acceleration_rad_s2=accelerations,
            joint_jerk_rad_s3=jerks,
            tcp_pose=tcp_pose,
            tcp_velocity=tcp_velocity,
            tcp_acceleration=tcp_acceleration,
            blend_zones=tuple(blend_zones),
            diagnostic=diagnostic,
        ))
        self._timeline_s += duration_s
        return True

    def _send_gripper(self, command_type, values, command_id):
        # (position, speed, current), checked by the planner's _gripper_values first (the 0-255 scales, the gripper
        # XML's current limit) — not the command's arguments, as this override's signature once said.
        position = values[0]
        self._events.append(DryRunEvent(
            command_id=command_id,
            sequence=self._commands[command_id].seq,
            command=command_type,
            time_s=float(self._timeline_s),
            details={"position": int(position), "simulated": False},
        ))

    def _tcp_samples(self, positions, velocities):
        set_kinematics_backend(self._fk_jacobian_backend())
        tcp_pose = np.empty((len(positions), 6), dtype=np.float64)
        tcp_velocity = np.empty((len(positions), 6), dtype=np.float64)
        for index, (joint_position, joint_velocity) in enumerate(zip(positions, velocities)):
            transform = calculate_forward_kinematics(self._robot, joint_position)
            tcp_pose[index] = get_tcp_pose_rpy(self._robot, joint_position)
            tcp_velocity[index] = calculate_jacobian(
                self._robot, joint_position, T=transform) @ joint_velocity
        return tcp_pose, tcp_velocity


class DryRunRobotClient(RobotClient):
    """A robot client that plans a program offline, with no robot.

    It plans every move with the robot's own planner, limits, safety zones and
    self-collision check, in memory, and never opens a connection.
    `start_joints`, in degrees, J1 first, is where the virtual arm starts. Motion
    calls queue a virtual program; `plan_summary()` plans it and returns
    timestamped samples for plots and path previews. The waits plan what is queued
    too.

    It answers as the live client does: a command the robot refuses when it is
    sent raises `CommandRejected` with the robot's code (`BAD_ARGS`,
    `BAD_OPTIONS`, `MOTION_ABORTED`, `NO_GRIPPER`, `BAD_PIN`); a move that fails
    when it is planned is reported by `wait_idle()` (`MotionFailed`),
    `wait_motion_complete()` and `wait_gripper_move_complete()` (`CommandRejected`)
    and `on_motion_error()`, and the moves queued after it fail `ABORTED` until
    `clear_errors()`; `stop()` fails the moves not planned yet with `STOPPED`.

    It does not simulate the robot's modes and states: a move is planned whether
    or not `enable()`, `home()` and `enter_exec_mode()` were called, and nothing is
    refused for a mode. Commands that need the robot itself (jogging, real-time
    streaming, the other mode changes, control, drive parameters, diagnostics)
    raise `RobotConnectionError`.

    Examples:
        >>> from robotclient import DryRunRobotClient
        >>> robot = DryRunRobotClient([90, -105, 165, 0, -30, 180])
        >>> robot.move_joints(90, -100, 160, 0, -30, 180)
        >>> robot.wait_idle()
        []
        >>> robot.plan_summary().duration_s
    """

    def __init__(self, start_joints):
        initial_deg = np.asarray(start_joints, dtype=np.float64)
        if initial_deg.shape != (6,):
            raise ValueError("start_joints needs exactly six joint angles in degrees")
        # Every attribute the inherited methods read (review P4, 2026-11-22: without the base constructor,
        # close_gripper(); wait_gripper_move_complete(), set_payload(), ... raised AttributeError) — and a connection
        # with no robot behind it.
        super().__init__(name="dry run")
        self._conn = _NoRobot(name="dry run")

        system = xml_parser.load_system_config(SYSTEM_XML, ROBOTS_DIR)
        config = xml_parser.load_full_robot_config(
            str(system["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
            active_gripper=system["active_gripper"])
        joint_num = int(config["joint_num"])
        if initial_deg.shape != (joint_num,):
            raise ValueError(f"start_joints needs exactly {joint_num} joint angles in degrees")

        robot = create_robot(system["active_robot"], config.get("gripper_name", ""))
        attach_pinocchio(robot)
        # The SAME limits command_executor plans with, from the same robot XML fields — a preview
        # that plans with different numbers previews a different trajectory. This used to read the
        # velocity CEILING and derive acceleration as max_vel / 0.5 s (a limit computed from another
        # limit in consumer code, which CLAUDE.md forbids), and hard-coded a 4 ms tick.
        self._planner = _OfflineTrajectoryPlanner(
            np.deg2rad(initial_deg), joint_num,
            np.asarray(config["exec_velocity_limit"], dtype=np.float64)[:joint_num],
            np.asarray(config["exec_acceleration_limit"], dtype=np.float64)[:joint_num],
            float(config.get("delta_t", 0.004)), robot,
            max_jerk=np.asarray(config["exec_jerk_limit"], dtype=np.float64)[:joint_num],
            joint_limits=(config["software_joint_limit_negative"],
                          config["software_joint_limit_positive"]),
            # the gripper XML's <Ilim>, as command_executor passes it: a gripper current above it is BAD_ARGUMENTS
            gripper_current_limit_ma=config.get("gripper_Ilim"))
        can = config.get("gripper_CAN_gripper", 0)          # as command_executor reads it (NaN = none): NO_GRIPPER
        self._has_can_gripper = not (isinstance(can, float) and np.isnan(can)) and int(can) == 1
        # The same safety zones command_executor checks, so a preview refuses what the robot would.
        self._gripper_xml = GRIPPERS_DIR / f"{system['active_gripper']}.xml"
        self._zone_setup = xml_parser.load_zone_setup(SYSTEM_XML, CELLS_DIR, self._gripper_xml)
        self._planner.set_zones(self._zone_setup["zones"], self._zone_setup["tool_spheres"])
        # ...and the same self-collision check (the pieces of the arm + the active gripper), when enabled
        if system.get("self_collision", {}).get("enabled", 0):
            from utility.self_collision import SelfCollisionModel, load_geometry
            rg, tg = load_geometry(ROBOTS_DIR, GRIPPERS_DIR, config)
            self._planner.set_self_collision(SelfCollisionModel(robot, rg, tg, system["self_collision"],
                                                                 config["self_collision"]["pairs"] + config["gripper_collision"]["pairs"]))
        self._next_seq = 1
        self._mode = "IDLE"
        self._state = "DISABLED"
        self._homed = False
        self._digital_outputs = {}
        self._setup_events = []
        self._on_error = None
        self._on_status_change = None
        self._on_motion_complete = None
        self._on_motion_error = None
        self._reported_completion_ids = set()
        self._reported_error_ids = set()
        # Failures the program has had (review P4): reported by wait_idle(), raised by a wait_*_complete(), or
        # discarded by stop() — as the live client, wait_idle() reports each failure once.
        self._failures_taken = set()
        self._defaults = MotionOptions(
            profile=DEFAULT_PROFILE,
            speed=100.0,
            blend=0.0,
            blend_mode="exact_velocity",
            ik_solver="pinocchio",
            completion_policy="settled",
        )

    def connect(self, ip: str | None = None, port: int | None = None, *, allow_mismatch: bool = False) -> DryRunRobotClient:
        del allow_mismatch           # RobotClient.connect's version check (V3): nothing to check in a dry run
        return self

    def disconnect(self) -> None:
        return None

    @property
    def connected(self) -> bool:
        return True


    def on_error(self, fn: Callable[[dict], None]) -> None:
        # Kept, never called: the live client calls it for the robot's errors that answer no command, which a dry run
        # has none of. A planning failure goes to on_motion_error() (review P4, 2026-11-22).
        self._on_error = fn

    def on_status_change(self, fn: Callable[[dict], None]) -> None:
        self._on_status_change = fn

    def on_motion_complete(self, fn: Callable[[str], None]) -> None:
        self._on_motion_complete = fn

    def on_motion_error(self, fn: Callable[[dict], None]) -> None:
        self._on_motion_error = fn


    def _queued(self, command, args=None, timeout=None, options=None, gripper=False):
        del timeout                   # nothing waits here
        # The executor's queue-time refusals, in its order (command_executor._submit_queued, _build_queued_command;
        # review P4, 2026-11-22: a dry run accepted them all) — CommandRejected as the live client raises them.
        name = canonical_command(command)
        args = list(args or [])
        if name in _QUEUED_GRIPPER_COMMANDS and not self._has_can_gripper:
            raise _rejected("NO_GRIPPER", _NO_GRIPPER)
        problem = _queued_args_problem(name, args, self._planner.joint_num)
        if problem is not None:
            raise _rejected("BAD_ARGS", problem)
        aborted = self._planner.abort_info()           # known once the failure was planned (any wait plans)
        if aborted is not None:
            raise _rejected("MOTION_ABORTED", _motion_aborted_text(aborted))
        defaults = self._defaults
        if command.startswith(("move_arc", "move_circle", "move_spline")) and "blend" not in (options or {}):
            defaults = replace(defaults, blend=0.0)      # as the executor: a curve ignores the session blend (2026-10-24)
        try:
            _reject_unsupported_per_command_options(options)
            resolved = MotionOptions.from_dict(options, defaults)
            problem = blend_problem(name, resolved)
            if problem is None:
                problem = profile_option_problem(name, resolved)
            if problem is not None:
                raise ValueError(problem)
        except (TypeError, ValueError) as e:
            raise _rejected("BAD_OPTIONS", str(e)) from None
        command_id = generate_id()
        queued = QueuedCommand.build(command_id, self._next_seq, command, args, replace(resolved, dry_run=False))
        self._next_seq += 1
        self._planner.enqueue(queued)
        self._last_queued = command_id
        if gripper:                                    # the base client's _queued_gripper (2026-10-24)
            self._last_gripper_queued = command_id
        return command_id

    def _record_immediate_event(self, command, details):
        """Record a non-queued setup/I-O command at the start of a dry-run plan."""
        self._setup_events.append(DryRunEvent(
            command_id=generate_id(),
            sequence=0,
            command=command,
            time_s=0.0,
            details={'immediate': True, **details},
        ))

    def enable(self, wait: bool | None = None) -> None:
        del wait
        self._state = "ENABLED"

    def disable(self, wait: bool | None = None) -> None:
        del wait
        self._state = "DISABLED"

    def clear_errors(self, wait: bool | None = None) -> None:
        """Lift the motion abort after a failed move, as the robot does.

        It first plans what is queued, so the moves queued before the call meet
        the failure they were queued behind, as on the robot, whose planner runs
        ahead of the program. Then moves plan again: the next one starts where the
        planned program ends.

        Args:
            wait: Ignored: a dry run has nothing to wait for.

        See also:
            `wait_idle()`, `stop()`

        Examples:
            >>> robot.move_joints(90, -105, 165, 0, -30, 400)
            >>> robot.wait_idle(raise_on_error=False)
            >>> robot.clear_errors()
            >>> robot.move_joints(90, -100, 160, 0, -30, 180)
            >>> robot.wait_idle()
        """
        del wait
        # review P4 (2026-11-22): it did nothing, so every move after one failure stayed ABORTED for good
        self.plan_summary()
        self._planner.clear_abort()

    def home(self, wait: bool | None = None) -> None:
        del wait
        self._homed = True

    def enter_exec_mode(self, wait: bool | None = None) -> None:
        del wait
        self._mode = "EXEC"

    def stop(self, wait: bool | None = None) -> None:
        """Cancel every queued move not planned yet, as the robot's stop does.

        Each move queued since the program last planned (any wait, `get_joints()`,
        `plan_summary()`, ...) fails with `STOPPED`, and `on_motion_error()` gets
        it; nothing is aborted, and the next move starts where the planned
        program ends. As the live client, it discards the failures `wait_idle()`
        has not reported yet and forgets the last commands:
        `wait_motion_complete()` then returns at once. In EXEC mode the dry run
        stays in EXEC mode; in any other it goes to IDLE mode.

        Args:
            wait: Ignored: a dry run has nothing to wait for.

        See also:
            `clear_errors()`, `wait_idle()`

        Examples:
            >>> robot.move_joints(90, -100, 160, 0, -30, 180)
            >>> robot.stop()
            >>> robot.plan_summary().errors[-1].code
            'STOPPED'
        """
        del wait
        # review P4 (2026-11-22): a flush alone dropped those moves (never completed nor failed) and re-planned the
        # next move from the START joints
        stopped = self._planner.stop_pending("motion stopped")       # command_executor._stop_motion's message
        self._call_back_failures(stopped)
        self._failures_taken.update(error.command_id for error in self._planner._errors)
        self._last_queued = self._last_gripper_queued = None
        if self._mode != "EXEC":
            self._mode = "IDLE"

    def pause(self, wait: bool | None = None) -> None:
        del wait
        return None

    def resume(self, wait: bool | None = None) -> None:
        del wait
        return None

    def set_joint_speed(self, percent: float) -> Any:
        value = float(percent)
        try:
            self._defaults = MotionOptions.from_dict({"speed": value}, self._defaults)
        except ValueError as e:   # as the robot refuses it (2026-11-22: a bare ValueError here)
            raise _rejected("BAD_ARGS", f"set_joint_speed: {e}") from None
        self._record_immediate_event("set_joint_speed", {"percent": value, "kind": "setup"})

    def set_blending(self, percent: float) -> Any:
        value = float(percent)
        try:
            self._defaults = MotionOptions.from_dict({"blend": value}, self._defaults)
        except ValueError as e:   # as the robot refuses it (2026-11-22: a bare ValueError here)
            raise _rejected("BAD_ARGS", f"set_blending: {e}") from None
        self._record_immediate_event("set_blending", {"percent": value, "kind": "setup"})

    def set_profile(self, profile: str) -> Any:
        value = str(profile).strip().lower()
        try:
            self._defaults = MotionOptions.from_dict({"profile": value}, self._defaults)
        except ValueError as e:   # as the robot refuses it (2026-11-22: a bare ValueError here)
            raise _rejected("BAD_ARGS", f"set_profile: {e}") from None
        self._record_immediate_event("set_profile", {"profile": value, "kind": "setup"})

    set_default_profile = set_profile

    def set_ik_solver(self, solver: str) -> Any:
        value = str(solver).strip().lower()
        try:
            self._defaults = MotionOptions.from_dict({"ik_solver": value}, self._defaults)
        except ValueError as e:   # as the robot refuses it (2026-11-22: a bare ValueError here)
            raise _rejected("BAD_ARGS", f"set_ik_solver: {e}") from None
        self._record_immediate_event("set_ik_solver", {"solver": value, "kind": "setup"})

    set_default_ik_solver = set_ik_solver

    def set_completion_policy(self, policy: str) -> Any:
        value = str(policy).strip().lower()
        if value not in ("commanded", "settled", "strict"):     # = command_executor._COMPLETION_POLICY_NAMES
            raise _rejected("BAD_ARGS", f"set_completion_policy: {policy!r} is not one of commanded, settled, strict")
        try:
            self._defaults = MotionOptions.from_dict({"completion_policy": value}, self._defaults)
        except ValueError as e:   # as the robot refuses it (2026-11-22: a bare ValueError here)
            raise _rejected("BAD_ARGS", f"set_completion_policy: {e}") from None
        self._record_immediate_event("set_completion_policy", {"policy": value, "kind": "setup"})

    def plan_summary(self) -> DryRunPlan:
        """Plan everything queued and return the whole preview so far.

        It finishes the queued blends, plans every queued command and returns the
        program's timestamped samples, its events and every error. It calls
        `on_motion_complete()` once for each command that planned and
        `on_motion_error()` once for each that failed.

        Returns:
            The preview: `segments` (one per planned move, with joint and tool
            samples), `events` (settings, I/O and gripper commands on the
            timeline), `errors`, `duration_s` (seconds) and `final_joints_rad`.

        See also:
            `wait_idle()`, `stop()`

        Examples:
            >>> robot.move_joints(90, -100, 160, 0, -30, 180)
            >>> plan = robot.plan_summary()
            >>> plan.valid, round(plan.duration_s, 2)
        """
        summary = self._planner.summary()
        if self._setup_events:
            events = (*self._setup_events, *summary.events)
            summary = replace(summary, events=events)
        self._call_back_failures(summary.errors)
        completed_ids = [segment.command_id for segment in summary.segments]
        completed_ids.extend(
            event.command_id for event in summary.events if not event.details.get("immediate", False))
        for command_id in completed_ids:
            if command_id in self._reported_completion_ids:
                continue
            self._reported_completion_ids.add(command_id)
            if self._on_motion_complete is not None:
                self._on_motion_complete(command_id)
        return summary

    def _call_back_failures(self, errors):
        """on_motion_error() once per failure, with the live client's failure dict (review P4, 2026-11-22: it got
        command_id, not id). Not on_error(): the live client calls it only for the robot's errors that answer no
        command (FORBIDDEN, BAD_JSON), which a dry run never has — it got every planning failure a second time."""
        for error in errors:
            if error.command_id in self._reported_error_ids:
                continue
            self._reported_error_ids.add(error.command_id)
            if self._on_motion_error is not None:
                self._on_motion_error(error.failure())

    def wait_idle(self, timeout: float | None = None, raise_on_error: bool = True) -> list[dict]:
        """Plan everything queued and report the failures not reported yet.

        As the live client's: it reports every failure since its last call, then
        forgets them. A failure that `wait_motion_complete()` or
        `wait_gripper_move_complete()` already raised is not reported again, and
        `stop()` discards the failures not reported yet. A command refused when
        it was sent is not among them: its own call raised `CommandRejected`.

        Args:
            timeout: Ignored: planning offline takes no robot time.
            raise_on_error: `True` raises `MotionFailed` when a command failed;
                `False` returns the failures instead.

        Returns:
            The failures since the last call, in the order they arose: a dict
            each, with `id` (the id the queued call returned), `code`, `message`
            and the error catalog's `number`, `title` and `fix`, plus the
            preview's `command_id`, `sequence` and `command`. An empty list when
            every command planned, and always with `raise_on_error=True`.

        Raises:
            MotionFailed: With `raise_on_error=True`, a command failed. Its
                `failures` holds the list described under Returns.

        See also:
            `wait_motion_complete()`, `plan_summary()`, `clear_errors()`

        Examples:
            >>> robot.move_joints(90, -100, 160, 0, -30, 180)
            >>> robot.wait_idle()
            []
        """
        del timeout
        summary = self.plan_summary()
        # review P4 (2026-11-22): every call returned (or raised) every failure since the start, as `command_id`
        failures = [error.failure() for error in summary.errors if error.command_id not in self._failures_taken]
        self._failures_taken.update(failure["id"] for failure in failures)
        if failures and raise_on_error:
            raise MotionFailed(failures)
        return failures

    def _raise_if_failed(self, command_id):
        """Plan what is queued; the command's failure as the live client raises it from a wait_*_complete():
        CommandRejected with its code (review P4, 2026-11-22: it was MotionFailed), not reported again by
        wait_idle()."""
        summary = self.plan_summary()
        if command_id is None:
            return
        for error in summary.errors:
            if error.command_id == command_id:
                self._failures_taken.add(command_id)
                raise _rejected(error.code, error.message)

    def wait_motion_complete(self, timeout: float | None = None) -> None:
        """Plan everything queued and raise the last command's failure.

        As the live client's: it judges the last move, delay or gripper command
        queued, and returns at once when nothing was queued or `stop()` was called
        since. It does not report the failures of the commands before it:
        `wait_idle()` does.

        Args:
            timeout: Ignored: planning offline takes no robot time.

        Raises:
            CommandRejected: The command failed, for example with `JOINT_LIMIT`
                or `ABORTED`; its `code` names the failure. Every later call
                raises it again, until another command is queued or `stop()` is
                called.

        See also:
            `wait_idle()`, `wait_gripper_move_complete()`

        Examples:
            >>> robot.move_joints(90, -100, 160, 0, -30, 180)
            >>> robot.wait_motion_complete()
        """
        del timeout
        self._raise_if_failed(self._last_queued)

    def wait_gripper_move_complete(self, timeout: float | None = None) -> None:
        """Plan everything queued and raise the last gripper command's failure.

        As the live client's: it judges the last gripper command queued and
        returns at once when none was queued or `stop()` was called since. Offline
        the jaws report no outcome: `gripper_result()` stays `None`.

        Args:
            timeout: Ignored: planning offline takes no robot time.

        Raises:
            CommandRejected: The gripper command failed, for example with
                `BAD_ARGUMENTS` (a current above the gripper's limit) or
                `ABORTED`. Every later call raises it again, until the next
                gripper command or `stop()`.

        See also:
            `wait_motion_complete()`, `wait_idle()`

        Examples:
            >>> robot.close_gripper()
            >>> robot.wait_gripper_move_complete()
        """
        del timeout
        self._raise_if_failed(self._last_gripper_queued)

    def get_joints(self) -> list[float]:
        self.plan_summary()
        return np.rad2deg(self._planner.current_joints_rad).tolist()

    def get_pose(self, *, angle_unit: str = "rad") -> list[float]:
        self.plan_summary()
        return _pose_out(get_tcp_pose_rpy(self._planner._robot, self._planner.current_joints_rad).tolist(), angle_unit)

    def is_reachable(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, near: Sequence[float] | None = None,
                     *, angle_unit: str = "rad") -> dict:
        """Same answer as RobotClient.is_reachable; `near` defaults to where the preview has got to."""
        x, y, z, rx, ry, rz = _pose_in((x, y, z, rx, ry, rz), angle_unit)
        planner = self._planner
        if near is None:
            self.plan_summary()
            near_rad = planner.current_joints_rad
        else:
            near_rad = np.deg2rad(np.asarray(near, dtype=np.float64))
        if getattr(self, "_reach_starts", None) is None:
            self._reach_starts = make_starts(planner._joint_lo, planner._joint_hi)
        target = pose_from_xyz_rpy([x, y, z, rx, ry, rz]).A
        sc = getattr(planner, "_self_collision", None)          # the planner's own: the preview is one thread
        q, pair = find_reachable_clear(planner._robot, target, near_rad, planner._joint_lo, planner._joint_hi,
                                       self._reach_starts, sc)
        return reach_reply(None if q is None else np.rad2deg(q).tolist(), target.ravel().tolist(),
                           self._zone_setup, None if pair is None else sc.describe(*pair))

    def get_zones(self) -> dict:
        return zones_summary(self._zone_setup)

    def reload_zones(self) -> dict:
        self._zone_setup = xml_parser.load_zone_setup(SYSTEM_XML, CELLS_DIR, self._gripper_xml)
        self._planner.set_zones(self._zone_setup["zones"], self._zone_setup["tool_spheres"])
        return zones_summary(self._zone_setup)

    def get_status(self) -> dict:
        self.plan_summary()                # plans what is queued: motion_aborted is then current
        return {
            "mode": self._mode,
            "state": self._state,
            "homed": self._homed,
            "rti_alive": True,            # the robot's status field (review C16): offline there is no loop to stop
            "pose_angles": POSE_CONVENTION,
            "dry_run": True,
            "simulated": False,
            "virtual": True,
            "planner": self._planner.status_snapshot(),
            # As the live status (2026-11-22): `errors` are the robot's LATCHED error keys and a planning failure
            # latches none (it was the failures' dicts, so wait_error_reset() never passed after one); the failure
            # that aborted the program is `motion_aborted`, and every failure is in plan_summary() / wait_idle().
            "err": 0,
            "errors": [],
            "error_details": [],
            "motion_aborted": self._planner.abort_info(),
        }

    status = get_status
    latest_status = get_status     # offline, the last status and a fresh one are the same (2026-10-23)

    def set_digital_output(self, pin: int, value: bool | int) -> None:
        """Record an output switch, checked as the robot checks it.

        Nothing is switched: the preview shows the switch as an event at time 0.

        Args:
            pin: The output: `1`, `2` or `3`. A number with a fraction is cut to a
                whole number first.
            value: `True` or `1` for on, `False` or `0` for off. Any other value
                is refused.

        Raises:
            CommandRejected: `pin` is not 1, 2 or 3 (`BAD_PIN`), as the robot
                refuses it.
            ValueError: `pin` cannot be read as a whole number, or `value` is not
                `True`, `False`, 1 or 0.
            TypeError: `pin` is `None` or another type that is no number.

        See also:
            `get_digital_input()`, `plan_summary()`

        Examples:
            >>> robot.set_digital_output(1, True)
            >>> robot.plan_summary().events[0].details
            {'immediate': True, 'pin': 1, 'value': True, 'kind': 'io'}
        """
        # The live client's value check and the executor's pin check (_handle_set_digital_output; review P4,
        # 2026-11-22: any pin and any value were accepted)
        if not (isinstance(value, bool) or (isinstance(value, (int, float)) and value in (0, 1))):
            raise ValueError(f"set_digital_output: value must be True / False or 1 / 0, got {value!r}")
        pin = int(pin)
        if pin not in _IO_PINS:
            raise _rejected("BAD_PIN", f"output pin must be 1-3, got {pin}")
        value = bool(value)
        self._digital_outputs[pin] = value
        self._record_immediate_event(
            "set_digital_output", {"pin": pin, "value": value, "kind": "io"})

    def get_digital_input(self, pin: int) -> bool:
        """Return `False`, the level of an input with no signal, for a valid pin.

        Offline no input is read; a Check of a block program assumes the input
        it waits for arrives.

        Args:
            pin: The input: `1`, `2` or `3`. A number with a fraction is cut to a
                whole number first.

        Returns:
            `False`.

        Raises:
            CommandRejected: `pin` is not 1, 2 or 3 (`BAD_PIN`), as the robot
                refuses it.
            ValueError: `pin` cannot be read as a whole number.
            TypeError: `pin` is `None` or another type that is no number.

        See also:
            `set_digital_output()`

        Examples:
            >>> robot.get_digital_input(1)
            False
        """
        pin = int(pin)                         # the executor's pin check (_handle_get_digital_input; review P4)
        if pin not in _IO_PINS:
            raise _rejected("BAD_PIN", f"input pin must be 1-3, got {pin}")
        return False

    # Sound (2026-11-17): offline nothing plays and nothing is saved; a tone is checked the robot's way
    # (utility/sound.tune_events) so a Check refuses what the robot would refuse.
    def _sound_events(self, command, notes):
        from utility.sound import tune_events
        try:
            return tune_events(notes)
        except ValueError as e:
            raise CommandRejected("BAD_ARGS", f"{command}: {e}") from None

    def beep(self, freq_hz: float = 880.0, duration_s: float = 0.2) -> None:
        self._sound_events("beep", [(freq_hz, duration_s)])
        self._record_immediate_event("beep", {"freq_hz": freq_hz, "duration_s": duration_s, "kind": "sound"})

    def play_tune(self, notes: list) -> None:
        events = self._sound_events("play_tune", notes)
        self._record_immediate_event("play_tune", {"notes": len(events), "kind": "sound"})

    def identify_arm(self) -> None:
        self._record_immediate_event("identify_arm", {"kind": "sound"})

    def begin_gripper_change(self) -> None:
        self._record_immediate_event("begin_gripper_change", {"kind": "gripper"})

    def end_gripper_change(self, gripper: str | None = None, removed: bool = False) -> dict:
        self._record_immediate_event("end_gripper_change", {"gripper": gripper, "removed": bool(removed),
                                                            "kind": "gripper"})
        return {"gripper": gripper, "restart": False}

    def store_gripper_tool_id(self) -> dict:
        # offline nothing is stored in a drive
        self._record_immediate_event("store_gripper_tool_id", {"kind": "drive"})
        return {"tool_id": None, "tool": None}

    def play_sound(self, name: str) -> None:
        from utility.sound import TUNES
        if not isinstance(name, str) or name not in TUNES:
            raise CommandRejected("BAD_ARGS", f"play_sound takes one tune name: {', '.join(TUNES)}")
        self._record_immediate_event("play_sound", {"name": name, "kind": "sound"})

    def set_sound(self, **settings) -> None:
        if "node" in settings:                 # as the robot: 'node' is 'drive' since command protocol 4 (1 is J1)
            raise CommandRejected("BAD_ARGS", "set_sound: 'node' is 'drive' since command protocol 4 — 1 for J1")
        self._record_immediate_event("set_sound", {"settings": dict(settings), "kind": "sound"})

    def get_motion_settings(self) -> dict:
        """As the robot's (review P8, 2026-11-22): the session's speed, blend, profile, IK solver, completion policy."""
        d = self._defaults
        return {"speed": float(d.speed), "blend": float(d.blend), "profile": d.profile, "ik_solver": d.ik_solver,
                "completion_policy": d.completion_policy}

    def wait(self, predicate: Callable[[dict], bool], timeout: float | None = None, what: str = 'wait') -> None:
        del timeout
        if not predicate(self.get_status()):
            raise RobotTimeout(f"{what} cannot become true in dry-run mode")

    def wait_enabled(self, timeout: float | None = None) -> None:
        self.wait(lambda status: status["state"] == "ENABLED", timeout, "wait_enabled")

    def wait_mode(self, mode: str, timeout: float | None = None) -> None:
        self.wait(lambda status: status["mode"] == mode, timeout, f"wait_mode({mode})")

    def wait_homed(self, timeout: float | None = None) -> None:
        self.wait(lambda status: status["homed"], timeout, "wait_homed")
