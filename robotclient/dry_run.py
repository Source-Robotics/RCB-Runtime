"""Offline EXEC planner client for previewing programs without RTI, ZMQ, or CAN."""
from __future__ import annotations
from typing import Any, Callable, Sequence

from dataclasses import dataclass, replace
from queue import Empty, Queue

import numpy as np

import config.xml_parser as xml_parser
from motion.cartesian_paths import pose_from_xyz_rpy
from motion.exec_commands import DEFAULT_PROFILE, MotionOptions, QueuedCommand
from motion.reachability import find_reachable_clear, make_starts
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
from .constants import DEFAULT_MOVE_TIMEOUT
from .exceptions import MotionFailed, RobotTimeout


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
        """This error as a MotionFailed failure: the live client's {"id", "code", "message"} — MotionFailed formats
        f["id"], so a bare __dict__ (command_id, no id) raised a KeyError instead of MotionFailed; found 2026-10-05 by
        the block editor's Check — plus the preview's own fields."""
        return {"id": self.command_id, **self.__dict__}


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
                 max_jerk=None, joint_limits=None):
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

    def _send_gripper(self, command_type, args, command_id):
        if command_type == "open_gripper":
            position = self.GRIPPER_OPEN_POS
        elif command_type == "close_gripper":
            position = self.GRIPPER_CLOSE_POS
        else:
            if not args:
                raise ValueError("move_gripper needs [position]")
            position = int(args[0])
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
    """RobotClient-compatible offline EXEC program planner.

    `start_joints` is required and uses the normal public API unit: degrees. The
    object never creates a TCP connection or a live ZMQ output socket. Motion calls
    queue a virtual program; call `plan_summary()` to finalize buffered blends and
    receive timestamped samples suitable for NiceGUI plots or path previews.
    """

    def __init__(self, start_joints):
        initial_deg = np.asarray(start_joints, dtype=np.float64)
        if initial_deg.shape != (6,):
            raise ValueError("start_joints needs exactly six joint angles in degrees")

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
                          config["software_joint_limit_positive"]))
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
        self._sync = False
        self._last_queued = None
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


    def on_error(self, fn: Callable[[str], None]) -> None:
        self._on_error = fn

    def on_status_change(self, fn: Callable[[dict], None]) -> None:
        self._on_status_change = fn

    def on_motion_complete(self, fn: Callable[[str], None]) -> None:
        self._on_motion_complete = fn

    def on_motion_error(self, fn: Callable[[dict], None]) -> None:
        self._on_motion_error = fn


    def _queued(self, command, args=None, timeout=DEFAULT_MOVE_TIMEOUT, options=None):
        del timeout
        command_id = generate_id()
        resolved = replace(MotionOptions.from_dict(options, self._defaults), dry_run=False)
        queued = QueuedCommand.build(command_id, self._next_seq, command, args or [], resolved)
        self._next_seq += 1
        self._planner.enqueue(queued)
        self._last_queued = command_id
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
        del wait
        return None

    def home(self, wait: bool | None = None) -> None:
        del wait
        self._homed = True

    def enter_exec_mode(self, wait: bool | None = None) -> None:
        del wait
        self._mode = "EXEC"

    def stop(self, wait: bool | None = None) -> None:
        del wait
        self._planner.flush()
        self._mode = "IDLE"

    def pause(self, wait: bool | None = None) -> None:
        del wait
        return None

    def resume(self, wait: bool | None = None) -> None:
        del wait
        return None

    def set_joint_speed(self, percent: float) -> Any:
        value = float(percent)
        self._defaults = MotionOptions.from_dict({"speed": value}, self._defaults)
        self._record_immediate_event("set_joint_speed", {"percent": value, "kind": "setup"})

    def set_blending(self, percent: float) -> Any:
        value = float(percent)
        self._defaults = MotionOptions.from_dict({"blend": value}, self._defaults)
        self._record_immediate_event("set_blending", {"percent": value, "kind": "setup"})

    def set_profile(self, profile: str) -> Any:
        value = str(profile).strip().lower()
        self._defaults = MotionOptions.from_dict({"profile": value}, self._defaults)
        self._record_immediate_event("set_profile", {"profile": value, "kind": "setup"})

    set_default_profile = set_profile

    def set_ik_solver(self, solver: str) -> Any:
        value = str(solver).strip().lower()
        self._defaults = MotionOptions.from_dict({"ik_solver": value}, self._defaults)
        self._record_immediate_event("set_ik_solver", {"solver": value, "kind": "setup"})

    set_default_ik_solver = set_ik_solver

    def set_completion_policy(self, policy: str) -> Any:
        value = str(policy).strip().lower()
        self._defaults = MotionOptions.from_dict({"completion_policy": value}, self._defaults)
        self._record_immediate_event("set_completion_policy", {"policy": value, "kind": "setup"})

    def plan_summary(self) -> DryRunPlan:
        summary = self._planner.summary()
        if self._setup_events:
            events = (*self._setup_events, *summary.events)
            summary = replace(summary, events=events)
        for error in summary.errors:
            if error.command_id in self._reported_error_ids:
                continue
            self._reported_error_ids.add(error.command_id)
            payload = error.__dict__
            if self._on_error is not None:
                self._on_error(payload)
            if self._on_motion_error is not None:
                self._on_motion_error(payload)
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


    def wait_idle(self, timeout: float | None = None, raise_on_error: bool = True) -> list[dict]:
        del timeout
        summary = self.plan_summary()
        if summary.errors and raise_on_error:
            summary.require_valid()
        return [error.__dict__ for error in summary.errors]

    def wait_motion_complete(self, timeout: float | None = None) -> None:
        del timeout
        summary = self.plan_summary()
        if self._last_queued is not None:
            for error in summary.errors:
                if error.command_id == self._last_queued:
                    raise MotionFailed([error.failure()])

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
        summary = self.plan_summary()
        return {
            "mode": self._mode,
            "state": self._state,
            "homed": self._homed,
            "pose_angles": POSE_CONVENTION,
            "dry_run": True,
            "simulated": False,
            "virtual": True,
            "planner": self._planner.status_snapshot(),
            "errors": [error.__dict__ for error in summary.errors],
        }

    status = get_status
    latest_status = get_status     # offline, the last status and a fresh one are the same (2026-10-23)

    def set_digital_output(self, pin: int, value: bool | int) -> None:
        pin = int(pin)
        value = bool(value)
        self._digital_outputs[pin] = value
        self._record_immediate_event(
            "set_digital_output", {"pin": pin, "value": value, "kind": "io"})

    def get_digital_input(self, pin: int) -> bool:
        return False

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
