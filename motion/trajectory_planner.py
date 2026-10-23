"""Trajectory planner thread (Phase C) — runs inside the command_executor process.

Pops motion commands from an in-process queue, generates a full trajectory OUTSIDE the
RT loop, and PUSHes it as a ZMQ batch to RTI's ExecHandler. RTI just iterates the
waypoints one per tick. Planning errors (unreachable target, unknown command) are
reported back on the EXEC status socket keyed by the command id.

Current scope: joint moves and delays can send hardware trajectories; move_pose sends
endpoint-IK joint trajectories; move_linear validates a sampled IK path first, then can
send a conservative low-speed Cartesian hardware trajectory.

Gripper commands (open_gripper / close_gripper / move_gripper) are not trajectory batches —
gripper motion is driven by RTI's own firmware-mode CAN command (_gripper_fw_state,
RTI-process-local), so there's no batch/waypoint pipeline to hook a completion signal
into. Instead, the checkpoint tracks REAL physical completion via
gripper_object_detection, the same Robotiq-style status code the GUI already decodes
(nicegui_test.py): 0 = still moving, 1/2 = object detected while opening/closing,
3 = reached target with no object.

"Any nonzero value means settled" was the ORIGINAL rule and is no longer sufficient — see
_poll_gripper_completions, which decides on THREE criteria (position tolerance, a debounced
and direction-checked code, timeout). The settled code is also reported onward to the
client as `gripper_object` on the checkpoint, so a caller can tell a real grip (1/2) from a
close on empty air (3).

This mirrors how move_joints already works (send the batch, return immediately, let
something else — ExecHandler on RTI's side — report completion once physically true)
rather than blocking the planner thread: _send_gripper() sends gripper_set and returns
right away; run()'s loop polls _poll_gripper_completions() after every processed command
and on each idle timeout, checking the one in-flight gripper target against
gripper_object_detection AND its commanded position, firing the real checkpoint once it
settles (or an error after GRIPPER_SETTLE_TIMEOUT_S with no settle). Not blocking the planner means a queued joint
move after a gripper command is still sent to RTI immediately — dispatch stays FIFO, but
that's not the same as waiting for physical completion; see robotclient/client.py's
gripper section for the wait_motion_complete() pattern for strict sequencing.

Only ONE physical gripper target can be in flight (the firmware position is a single
value in _gripper_fw_state, overwritten by each new gripper_set) — so only one pending
checkpoint is tracked at a time. If a new gripper command arrives before the previous one
settles, the previous one is reported as SUPERSEDED (an error, not a false "complete") —
its target was overwritten before the gripper ever reached it, so it will never see its
own completion signal.

RESOLVED 2026-08-06 on hardware (api_tests/06_io.py --gripper-signal): the status DOES
reset to 0 when a new move starts. Measured 3 -> 0 (at t+20ms) -> 3 (at t+722ms) across an
open-then-close. It does not hold a stale value, so a settled code always belongs to the
command being tracked. The completion logic no longer depends on that being true either
way — see _poll_gripper_completions, which cross-checks position and travel direction.

Thread-safety: set_speed()/flush()/stop() are called from executor threads and only
assign plain attributes or drain the queue — no locking needed (single writer per field,
CPython attribute assignment is atomic).

Blending (velocity-continuous, TODO/executive_mode_plan.md §3.9 — joint-space only,
NOT the Cartesian path-rounding version the plan's diagram shows): when set_blend(pct)
is nonzero, a move_joints command peeks at the next queued command before planning. If
it's also move_joints, motion/trajectory_profiles.compute_corner_speed() decides a safe
non-zero speed to carry through the shared corner instead of stopping there, and both
segments are planned with that as a boundary velocity (this move's END, the next move's
START). No new transport is needed for this — ExecHandler's existing pre-fetch
double-buffer already swaps batches on the same tick with no gap (built for chunking,
reused here for free); each command still gets its own checkpoint exactly as before,
only the physical motion changes. Opportunistic only: if the next command isn't queued
yet at decision time, this move just plans normally (no blend_timeout wait). The
decided corner speed is carried forward as a scalar (self._last_exit_speed) and
re-expressed along each segment's own direction, since consecutive segments generally
point different ways — see compute_corner_speed()'s docstring for why a shared vector
doesn't work. gripper/delay commands always clear it (they aren't blend-compatible).
"""

import json
from dataclasses import replace
import logging
import time
from queue import Empty
from threading import RLock, Thread

import numpy as np
import zmq
from spatialmath import SE3

from communication.exec_channels import BATCH_FLAG_BLEND_CONTINUES, pack_batch, pack_fence
from communication.protocol import canonical_command
from config.networking import ZMQ_EXEC_TRAJECTORY_SOCKET, ZMQ_EXEC_STATUS_SOCKET, ZMQ_CMD_SOCKET
from motion.exec_commands import (
    DEFAULT_PROFILE, MotionOptions, QueuedCommand, normalize_profile_name, normalize_ik_solver_name,
    normalize_blend_mode_name,
)
from motion.cartesian_paths import (
    cartesian_line, cartesian_arc_3point, cartesian_circle, cartesian_spline,
    pose_from_xyz_rpy, build_composite_cartesian_path, blend_zone_poses,
    DEFAULT_ARCLEN_SPACING_M as CART_ARCLEN_SPACING_M,
    interp_pose, CartesianPath,
    ROTATION_RADIUS_M as CART_ROTATION_RADIUS_M,
)
from motion.ik_config import (
    DEFAULT_IK_CONFIG, DEFAULT_FK_JACOBIAN_BACKEND, json_safe_metric,
)
from motion.ik_path import SeededIKPathSolver
from motion.joint_paths import (
    build_composite_joint_path, arclen_spacing_for,
    blend_joint_zone,
)
from motion.planner_pipeline import (
    PathBuilder, make_time_parameterizer, ExecBatchSampler, TimedJointPath, JointPath,
    RuckigTimeParameterizer, RUCKIG_JERK_FACTOR,
    TOPPRAUnavailable, TOPPRAInfeasible, TOPPRATimeParameterizer,
    path_velocity_from_joint_velocity, build_geometric_path,
    blend_distance_for_tolerance,
)
from motion.ruckig_corner import plan_ruckig_corner_chain
from motion.trajectory_profiles import compute_corner_speed
from scipy.interpolate import CubicSpline
from robotics.robot_kinematics import (
    calculate_forward_kinematics, calculate_jacobian,
)
from utility.kinematics_backend import set_kinematics_backend
from utility.activity_log import get_exec_activity_logger, log_activity
from utility.zones import ZONE_WARNING, first_path_violation, robot_spheres

logger = logging.getLogger("exec.planner")  # docs/logging.md "Logger names"
_activity_logger = get_exec_activity_logger()


class PlannerError(Exception):
    """Planning failure with a client-visible error code."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)
        self.message = str(message)


class TrajectoryPlanner(Thread):
    # Must match ExecHandler.RECV_HWM. At 8 the send BLOCKED: a blended chain is one batch
    # per command, so a 79-command body could not be pushed until RTI had chewed through most
    # of it — measured 2.24s stuck inside _send_blended_chain on hardware. That wrecked the
    # deadline, because `sent_at` is stamped when the send COMPLETES: the wall-clock floor
    # then believed RTI had a full body of runway at a moment when RTI had already played
    # nearly all of it. Floor said 45 samples, RTI had 0, the arm stalled 0.20s mid-stream.
    SEND_HWM = 512
    SEND_TIMEOUT_MS = 5000  # backpressure timeout — a 2000-pt chunk is >=2s of
                            # playback, so RTI should drain well within this
    MAX_CHUNK_SIZE = 2000   # waypoints per ZMQ message (plan §3.4) — caps message
                            # size and lets RTI start moving before a long
                            # trajectory finishes sending
    MAX_BLEND_LOOKAHEAD = 100  # cap on buffered zone-blend commands before a forced flush

    # ---- Rolling buffer (zone blend phase 5, TODO/blending_design.md §6) --------
    #
    # A zone-blend chain is planned rest-to-rest, but only its BODY is sent; the
    # braking TAIL is held back so a command arriving later can still be spliced in
    # (see _hold_tail / _try_splice). If the tail is committed too late, RTI runs out
    # of samples and freezes on a waypoint that still carries nonzero velocity — an
    # abrupt stop with a velocity discontinuity on real hardware.
    #
    # The margin must cover everything between "the deadline passes" and "RTI holds
    # the batch". MEASURED on the Pi, 2026-07-11 (dummy_test_scripts/
    # measure_exec_ipc_latency.py, plus an offline bench of the continuation solve):
    #
    #     planner reaction (HOLD_POLL_S)                     5 ms
    #     plan the joint continuation (p95, max 15.4)     15.4 ms
    #     pack + send -> RTI holds it (max of 20 reps)      8.7 ms   (~2 RTI ticks)
    #     exec_samples_remaining is refreshed once per tick   4 ms
    #                                                     --------
    #                                                     ~33 ms worst case
    #
    # x3.6 -> 120 ms. The headroom is for this thread, not the numbers above: the
    # planner is a NORMAL-priority Python thread inside command_executor, so it can be
    # preempted or hit a GC pause at any moment — SCHED_FIFO protects RTI, not us.
    #
    # Committing EARLY is always safe (it only costs a splice opportunity); committing
    # LATE means RTI exhausts its buffer and freezes on a waypoint that still carries
    # nonzero velocity — the arm stops dead from speed. Risk is asymmetric, so the
    # margin errs high. But it cannot err TOO high: the margin sets SPLICE_MIN_BODY_S,
    # and a move whose body is shorter than that is never held at all. An earlier 250ms
    # margin (sized for Cartesian's 75ms IK+solve) disabled the feature for every move
    # under ~1.5s, which is most of them.
    #
    # Cartesian splicing is NOT built yet. When it is, this must be re-derived for it:
    # its continuation costs an IK solve (75ms p95), so its margin lands near 0.25s.
    # PER-FAMILY, and it has to be: the only real difference is that a Cartesian
    # continuation must run IK (75ms p95) where a joint one does not (15ms). A single
    # margin sized for Cartesian would give SPLICE_MIN_BODY_S = 0.5s, which stops the
    # JOINT lane splicing plenty of perfectly good moves — sizing for the slow lane
    # silently disables the fast one. (Measured: Cartesian moves are long anyway — the
    # 20% speed cap makes even a 50mm move 3.0s, with 1.3s of body — so 0.5s costs it
    # nothing.)
    #
    # RE-DERIVED after the first streaming run on hardware, where the deadline fired with
    # 13 samples of runway left instead of the 30-sample margin — ~68ms late.
    #
    # That is NOT OS jitter or a GC pause, which is what it looked like at first. The
    # planner is ONE thread: _service_held() (the deadline check) only runs on the
    # queue-poll timeout path in run(). While _splice_pending() is inside TOPPRA building
    # a continuation, the loop physically cannot look at the clock. A splice costs ~72ms;
    # the lateness measured 68ms. It is the planner being unable to check the deadline
    # while it is busy planning — self-inflicted, and BOUNDED by the splice cost.
    #
    # Bounded is the good news: it means the margin can simply cover it, and does not need
    # to cover an unbounded scheduling tail. So a splice must fit ENTIRELY inside it:
    #
    #                                joint      cartesian
    #   planner notices (poll)        5 ms         5 ms
    #   a splice already in flight   72 ms        72 ms   <- SPLICE_BRAKE_PROBES caps this
    #   plan the continuation      15.4 ms        23 ms   (was 75ms; path reuse killed it)
    #   pack + send -> RTI has it   8.7 ms       8.7 ms
    #   shared-memory refresh         4 ms         4 ms
    #   -------------------------------------------------
    #   worst case                  105 ms       113 ms
    #
    # 0.18s / 0.20s leaves ~1.7x on the WORST case, not the average. (The 3.6x I first
    # claimed was measured against the budget with the in-flight splice left out of it.)
    # This is why SPLICE_BRAKE_PROBES has a hard cap: it bounds the top line, and an
    # unbounded splice cost would make this margin unsizable.
    #
    # Note the Cartesian margin went DOWN (0.25 -> 0.20). Reusing the planned path instead
    # of re-IK-ing the remainder took its continuation from 75ms to 23ms, and the old
    # margin was sized for that 75ms.
    # SIZED BY THE PLANNER'S REAL COST, not by what we wish it were.
    #
    # Making TOPPRA actually respect its acceleration limit on the SAMPLED output (see
    # planner_pipeline._solve) costs a second solve. A 100-command splice measures ~176ms.
    # The margin must contain a whole in-flight splice, so it has to cover that.
    #
    # CORRECTNESS COMES FIRST. The alternative was to leave blended chains 2.5-6.2% over their
    # acceleration limit to keep the margin small — trading a real limit violation for a
    # streaming convenience. Not acceptable. If a command source cannot keep the buffer fed,
    # the arm stops between moves; it does NOT exceed its limits.
    #
    # min_body MUST STAY ABOVE THE MARGIN. Decoupling them (earlier today) removed the
    # DERIVATION, not the CONSTRAINT: if the body is shorter than the margin, the deadline
    # fires the instant the tail is held and there is no splice window at all. Raise them
    # together or not at all.
    # SIZED BY THE PLANNER'S REAL COST. CORRECTNESS FIRST, THEN THE TIMING BUDGET.
    #
    # TOPPRA now refines its grid until the SAMPLED acceleration is legal (planner_pipeline
    # ._solve) — no fudge factors, every invariant preserved. That costs a second, finer solve:
    # a 100-command splice measures ~318ms. The margin must contain a whole in-flight splice,
    # so it has to cover that.
    #
    # THE ALTERNATIVE WAS TO LEAVE BLENDED CHAINS 2.5-6.2% OVER THEIR ACCELERATION LIMIT to
    # keep the margin small. Not acceptable: an arm that stops between two SHORT moves when the
    # command source is slow is a nuisance; an arm that quietly exceeds its acceleration limit
    # is a defect.
    #
    # min_body MUST EXCEED THE MARGIN (else the deadline fires the instant a tail is held and
    # there is no splice window). It does NOT need to DOUBLE it — I made that mistake, set
    # min_body to 2.2x the margin, and concluded blending was impossible because even a
    # 130-degree move could not hold a tail. It only has to EXCEED it.
    #
    # Net effect: the shortest blendable move is ~1.10s (min_body for the body + min_body
    # reserved for the tail). A 25-degree hop (0.86s) no longer blends into the next command;
    # it runs to rest. That is the price of a correct acceleration limit, and it is the right
    # price.
    SPLICE_SAFETY_MARGIN_S = {"joint": 0.45, "cartesian": 0.50}

    # The shortest body we will hold a tail behind. NOT derived from the margin any more.
    #
    # It used to be SPLICE_MIN_BODY_FACTOR * margin, and raising the margin to cover thread
    # jitter dragged this to 0.36s with it — which broke a plain 0.86s move: its body is
    # 0.34s, so it refused to hold, and two such moves in a row would STOP between them
    # instead of blending. Padding the streaming worst case was taxing the idle best case.
    #
    # They guard different things, and the jitter is LOAD-DEPENDENT:
    #
    #   margin  — WHEN to commit the tail. Must survive the worst case, which happens under
    #             load (the ~68ms was measured mid-stream, planner busy in TOPPRA). Under
    #             load the bodies are long, so a fat margin is free.
    #   min_body — WHETHER to hold one at all. Only ever binds on SHORT, ISOLATED moves,
    #             and there the planner is idle: no TOPPRA running, nothing to preempt it,
    #             so jitter is ~0 and the 33ms budget is the whole story.
    #
    # So size this one on the idle reaction cost (33ms budget + ~20ms idle jitter ~= 55ms)
    # with the same x2 safety factor => ~0.11s. We keep the joint lane at its long-proven
    # 0.24s rather than cut it that far; there is no reason to spend the headroom.
    #
    # Cartesian drops 0.50s -> 0.28s. That is not a loosening, it is a stale number being
    # corrected: 0.50s was sized for a 75ms re-IK of the whole remainder, and path reuse
    # took that to 23ms. This is what lets short Cartesian moves stream at all.
    SPLICE_MIN_BODY_S = {"joint": 0.55, "cartesian": 0.60}
    HOLD_POLL_S = 0.005     # queue poll while holding a tail (vs 0.2s when idle)
    # How long after a send exec_samples_remaining may still predate it. Measured end-to-end
    # at 8.7ms (RTI reads the socket once per tick, publishes on the next); 4x for headroom.
    # Only inside this window is the wall-clock floor allowed to override the real reading.
    RUNWAY_STALE_S = 0.035

    # Backstop for a wedged RTI (never entered EXEC, died mid-batch, paused forever):
    # give up holding once the body should long since have played out. This is measured
    # from the END of the body, NOT from the send — an absolute cap would break a
    # legitimately long chain. A 5-command chain can easily be 20s+, and its tail is held
    # for nearly all of it; a flat 30s cap would commit that tail early while RTI still
    # had thousands of samples of runway, silently killing the splice on exactly the long
    # toolpaths this feature exists for.
    HOLD_GRACE_S = 30.0

    # All IK tuning (branch/start-jump thresholds, slimit/ilimit, tolerances, DLS
    # params, singularity-warn levels) lives in one place — motion/ik_config.py's
    # DEFAULT_IK_CONFIG — and is used by every IK call site here. See that module
    # for each value's rationale.

    # Gripper defaults — match the GUI's _gripper_cmd_state exactly (nicegui_test.py):
    # firmware-mode raw scale 0-255, 1=fully open, 254=fully closed. NOT millimeters —
    # firmware-mode grippers don't expose a verified mm conversion (see module docstring).
    GRIPPER_OPEN_POS = 1
    GRIPPER_CLOSE_POS = 254
    GRIPPER_DEFAULT_SPEED = 100
    GRIPPER_DEFAULT_CURRENT = 200
    GRIPPER_SETTLE_TIMEOUT_S = 5.0  # generous margin over a typical full-stroke time

    # Gripper completion is decided by THREE independent criteria, not one.
    # `gripper_object_detection != 0` alone was the original rule and it is the
    # weakest of the four implementations surveyed (2026-08-06): it trusts a
    # single frame of a signal whose reset behaviour on a new move has never been
    # verified on hardware. If it holds a stale nonzero from the previous
    # command, that rule completes the next command instantly — before the jaws
    # move — and a pick reports success it never had.
    #
    # The tolerance and debounce below are why: the jaws report a position that settles
    # slowly and a status code that flickers, so neither is trusted on a single sample.
    GRIPPER_POSITION_TOLERANCE = 5   # raw 0-255 units; "close enough" counts as arrived
    GRIPPER_DEBOUNCE_TICKS = 3       # consecutive nonzero polls before a code is believed
    CARTESIAN_MOVE_MAX_SPEED_SCALE = 0.20  # first hardware linear moves stay deliberately slow

    def __init__(self, motion_queue, joint_num, max_vel, max_accel, dt,
                 read_current_joints, speed_scale=1.0, read_gripper_status=None,
                 read_gripper_position=None,
                 robot=None, read_exec_runway=None, offline_output=False,
                 max_jerk=None, joint_limits=None, zones=(), tool_spheres=()):
        """
        Args:
            motion_queue:        queue.Queue of QueuedCommand records (legacy dicts
                                 are still accepted during the transition).
            joint_num:           number of arm joints.
            max_vel:             (J,) per-joint velocity limit (rad/s).
            max_accel:           (J,) per-joint acceleration limit (rad/s²).
            dt:                  RT sample period (s).
            read_current_joints: callable() -> (J,) current joint angles (rad),
                                 used as the start point when no prior endpoint.
            speed_scale:         initial speed fraction 0<scale<=1.
            read_gripper_status: callable() -> int, latest gripper_object_detection
            read_gripper_position: callable() -> int, latest gripper_position on the
                                 same raw 0-255 scale the commands use. Optional, but
                                 without it gripper completion falls back to trusting
                                 the status code alone — see _poll_gripper_completions.
                                 value (0=moving, 1/2/3=settled). None disables gripper
                                 completion tracking (gripper commands would then never
                                 checkpoint — only omit this if there's truly no gripper).
            robot:               robot model used for endpoint IK commands.
            read_exec_runway:    callable() -> float, rti['exec_samples_remaining'] —
                                 how many trajectory samples RTI still has buffered.
                                 Drives the rolling buffer's commit deadline
                                 (TODO/blending_design.md §6). None disables tail
                                 holding entirely: chains are then sent whole, exactly
                                 as they were before phase 5. A planner that cannot see
                                 RTI's runway must never hold a tail back, because it
                                 has no way to know when RTI is about to run dry.
            offline_output:      do not create output ZMQ sockets. Intended only for
                                 an offline planner subclass that overrides every send/
                                 checkpoint hook with an in-memory collector.
            joint_limits:        (lo, hi) software joint limits in rad, from the robot XML.
                                 Every planned trajectory is checked against them before it
                                 is sent (JOINT_LIMIT). None falls back to the robot model's
                                 qlim (itself read from the same XML); with neither, no check.
        """
        super().__init__(daemon=True, name="TrajectoryPlanner")
        self._queue = motion_queue
        self.joint_num = joint_num
        self._max_vel = np.asarray(max_vel, dtype=np.float64)
        self._max_accel = np.asarray(max_accel, dtype=np.float64)
        # From <exec_limits><jerk> in the robot XML. None means "not stated", and the ruckig
        # profile then falls back to its old `acceleration * RUCKIG_JERK_FACTOR` derivation —
        # so a robot XML predating the field behaves exactly as it did. Only profile='ruckig'
        # and ruckig_corner blending read this; no other profile bounds jerk at all.
        self._max_jerk = None if max_jerk is None else np.asarray(max_jerk, dtype=np.float64)
        self._dt = dt
        self._read_current_joints = read_current_joints
        self._read_gripper_status = read_gripper_status
        # Optional but strongly wanted: without it the direction cross-check and
        # the position criterion both fall back to trusting the status code alone.
        self._read_gripper_position = read_gripper_position
        self._read_exec_runway = read_exec_runway
        self._robot = robot
        self._offline_output = bool(offline_output)
        self._held = None            # in-flight chain whose braking tail is held back
                                      # (rolling buffer) — see _hold_tail()

        self._speed_scale = float(np.clip(speed_scale, 1e-3, 1.0))
        self._blend_pct = 0.0        # blend radius %, 0 = off (set_blend())
        self._last_endpoint = None   # end of last planned move (rad); None → read live
        self._last_exit_speed = None  # scalar rad-generalized/s carried across a
                                       # blended corner; None = next move starts from
                                       # rest. See module docstring "Blending".
        self._running = True
        # Software joint limits (rad). See _check_joint_limits().
        self._joint_lo = self._joint_hi = None
        if joint_limits is not None:
            self._joint_lo = np.asarray(joint_limits[0], dtype=np.float64)[:joint_num].copy()
            self._joint_hi = np.asarray(joint_limits[1], dtype=np.float64)[:joint_num].copy()
        elif robot is not None:
            qlim = np.asarray(getattr(robot, "qlim", []), dtype=np.float64)
            if qlim.shape == (2, joint_num):
                self._joint_lo, self._joint_hi = qlim[0].copy(), qlim[1].copy()
        # Safety zones of the active cell (utility/zones.py). One tuple, swapped whole by set_zones(),
        # so the planner thread never sees half of a reload. See _check_zones().
        self._zone_setup = ((), robot_spheres(()))
        self.set_zones(zones, tool_spheres)
        # ABORT LATCH. Once a queued command fails, the rest of the program must not run —
        # in async mode it is already queued behind the failure. Set by _latch_abort() (a
        # planner-side failure) or interrupt() (the robot stopped mid-program); while set,
        # every command that reaches the planner is answered ABORTED without being planned.
        # Cleared only by clear_abort() — command_executor calls it on clear_errors.
        self._abort = None
        # ABORT GATE (2026-10-13). What empties the queue — a failure (latch + drain), a Stop (flush), the robot
        # leaving EXEC (interrupt) — and command_executor's check -> register -> queue -> ack of a new command hold
        # this lock, so neither runs in the middle of the other. Without it a command checked just BEFORE such an
        # event was queued just AFTER it: behind a failure (its client heard ABORTED before "queued"), or after a Stop,
        # where nothing kept it from RUNNING (found by the simulated suite; dummy_test_scripts/test_exec_queue_race.py).
        # Re-entrant: interrupt() flushes under it.
        self.abort_gate = RLock()
        # The STOP EPOCH (2026-10-14): flush() (a Stop) counts it up and sends RTI a fence carrying it; every queued
        # command carries the epoch it was queued in (command_executor sets it under abort_gate), and a send of a
        # command from an older epoch is dropped (_send_range, under the gate). So nothing of a stopped program reaches
        # RTI after the fence — a command still being planned when the Stop came used to send its batches afterwards
        # (harmless while a Stop left EXEC; the robot now stays in EXEC, holding).
        self._epoch = 0
        self._cmd_epoch = 0          # the epoch of the command the planner thread is working on
        # Set when the arm may have moved by something other than EXEC (another mode, an
        # error, a hand). The next plan then starts from the MEASURED position instead of
        # the last planned endpoint. Consumed by _start_point().
        self._resync = False
        self._pending_gripper = None  # (cid, deadline) — at most one in-flight target
        self._path_builder = PathBuilder(joint_num)
        self._time_parameterizer = make_time_parameterizer(DEFAULT_PROFILE)
        self._sampler = ExecBatchSampler(joint_num)
        self._blend_buffer = []      # buffered move_joints commands awaiting a
                                      # zone-blend flush — TODO/blending_design.md §5.1
        self._toppra_path_parameterizer = TOPPRATimeParameterizer()
        # Composite paths are sampled at a uniform arc-length spacing (never by
        # sample index) — see motion/joint_paths.py's module docstring. Derived
        # from the execution limits so the geometry is never coarser than the
        # trajectory sampled off it.
        self._path_spacing = arclen_spacing_for(self._max_vel, self._dt)

        # Planner status surfaced by command_executor status events. These describe
        # planner dispatch state, not RTI physical completion.
        self._executing_seq = 0
        self._completed_seq = 0
        self._active_command = None
        self._current_profile = DEFAULT_PROFILE
        self._last_error = None
        self._last_diagnostic = None

        self._traj_sock = None
        self._status_sock = None
        self._cmd_sock = None
        if not self._offline_output:
            ctx = zmq.Context.instance()
            self._traj_sock = ctx.socket(zmq.PUSH)
            self._traj_sock.setsockopt(zmq.SNDHWM, self.SEND_HWM)
            self._traj_sock.setsockopt(zmq.SNDTIMEO, self.SEND_TIMEOUT_MS)
            self._traj_sock.setsockopt(zmq.LINGER, 0)
            self._traj_sock.connect(ZMQ_EXEC_TRAJECTORY_SOCKET)

            self._status_sock = ctx.socket(zmq.PUSH)
            self._status_sock.setsockopt(zmq.LINGER, 0)
            self._status_sock.connect(ZMQ_EXEC_STATUS_SOCKET)

            # For gripper commands only — RTI's general command socket. Multiple senders
            # connecting here is the existing, designed topology (GUI/command_executor
            # already do the same); this is just one more.
            self._cmd_sock = ctx.socket(zmq.PUSH)
            self._cmd_sock.setsockopt(zmq.LINGER, 0)
            self._cmd_sock.connect(ZMQ_CMD_SOCKET)

    # ------------------------------------------------------------------ #
    # Control surface (called from executor threads)
    # ------------------------------------------------------------------ #

    def set_speed(self, pct):
        """pct in 1..100 (percent of full speed)."""
        self._speed_scale = float(np.clip(pct / 100.0, 1e-3, 1.0))

    def set_blend(self, pct):
        """Blend radius as a percentage 0..100 — scales how much corner speed
        compute_corner_speed() is allowed to preserve between consecutive
        move_joints commands. 0 (default) = always stop between moves."""
        self._blend_pct = float(np.clip(pct, 0.0, 100.0))

    def flush(self):
        """Drop queued motion and forget the planned endpoint (re-sync to live); start a new stop epoch and send RTI
        its fence (nothing of the program before it is sent after it — see abort_gate's notes in __init__).

        Called on stop() (RTI brakes the arm along its path and drops what it holds up to the fence) and by
        interrupt(). A held tail is DISCARDED, not committed: the arm is stopping, so sending it the rest of a braking
        ramp it will never execute would only leave a stale batch in the socket for whatever runs next.
        """
        with self.abort_gate:
            try:
                while True:
                    self._queue.get_nowait()
            except Empty:
                pass
            self._blend_buffer = []
            self._held = None
            self._last_endpoint = None
            self._last_exit_speed = None
            self._epoch += 1
            if self._traj_sock is not None:
                try:
                    self._traj_sock.send(pack_fence(self._epoch), flags=zmq.NOBLOCK)
                except zmq.ZMQError as e:            # RTI not draining: its stop waits for the fence, then gives up
                    logger.error(f"stop fence {self._epoch} not sent: {e}")

    @property
    def epoch(self):
        """The current stop epoch (flush() counts it up)."""
        return self._epoch

    def set_hold_start(self, q):
        """The next move starts from `q` — the commanded point a Stop left the arm holding at (command_executor, once
        RTI confirmed the stop), not the measured position a flush would otherwise re-read."""
        with self.abort_gate:
            self._last_endpoint = np.array(q, dtype=np.float64)[:self.joint_num]
            self._last_exit_speed = None

    def abort_info(self):
        """The failure that aborted the program ({cid, code, message}), or None."""
        a = self._abort
        return dict(a) if a is not None else None

    def clear_abort(self):
        """Accept motion again after an abort. command_executor calls this on clear_errors."""
        with self.abort_gate:
            if self._abort is not None:
                logger.info(f"abort cleared (was: {self._abort['code']})")
            self._abort = None

    def interrupt(self, code, message):
        """The robot stopped mid-program — an error, E-stop or a mode change took it out of
        EXEC while commands were still pending. RTI already discarded what it had, so there is
        nothing to finish cleanly: latch the abort and drop everything queued, like flush().
        Called from command_executor's thread; the caller fails the pending clients (under abort_gate too)."""
        with self.abort_gate:
            if self._abort is None:
                self._abort = {"cid": None, "code": code, "message": message}
                logger.error(f"motion ABORTED: {message}")
            self.flush()
        self._resync = True

    def note_exec_exit(self):
        """RTI left EXEC. Whatever moves the arm next (JOG, IDLE by hand, RTI, homing, an
        error) is not EXEC, so the last planned endpoint can no longer be trusted as the start
        of the next move: re-read the measured position next time. Without this the next EXEC
        move planned from where the PREVIOUS program ended, not from where the arm was."""
        self._resync = True

    def status_snapshot(self):
        """Snapshot for command_executor status events."""
        return {
            "motion_aborted": self.abort_info(),
            "executing_seq": int(self._executing_seq),
            "completed_seq": int(self._completed_seq),
            "queue_depth": int(self._queue.qsize()),
            "active_command": self._active_command,
            "current_profile": self._current_profile,
            "last_error": self._last_error,
            "last_diagnostic": self._last_diagnostic,
            "ik_config": self._ik_config(),
        }

    def _ik_config(self):
        """Effective IK tuning, for the status block. last_diagnostic is null on a
        successful solve, so without this the active thresholds/limits would only be
        visible on failure. Single source of truth is DEFAULT_IK_CONFIG."""
        return DEFAULT_IK_CONFIG.status_dict()

    def _fk_jacobian_backend(self):
        """Concrete FK/Jacobian backend for IK planning — fast pinocchio, rtb fallback.
        INDEPENDENT of the IK engine (see motion/ik_config.py)."""
        if DEFAULT_FK_JACOBIAN_BACKEND == "pinocchio" and not hasattr(
                self._robot, "pinocchio_fkine_and_jacob0"):
            return "rtb"
        return DEFAULT_FK_JACOBIAN_BACKEND

    def stop(self):
        self._running = False

    # ------------------------------------------------------------------ #
    # Planner loop
    # ------------------------------------------------------------------ #

    def run(self):
        logger.info("started")
        while self._running:
            # While a tail is held, RTI is burning through the body we already sent and
            # we are on a clock — poll fast enough that noticing a command isn't itself
            # a meaningful part of the safety margin (0.2s would be).
            timeout = self.HOLD_POLL_S if self._held is not None else 0.2
            try:
                cmd = self._queue.get(timeout=timeout)
                ep = cmd.epoch if isinstance(cmd, QueuedCommand) else cmd.get("epoch")
                self._cmd_epoch = self._epoch if ep is None else ep
            except Empty:
                if self._held is not None:
                    # Nothing arrived. Commit the tail if RTI is about to run dry.
                    self._service_held()
                else:
                    # Queue-idle timeout — flush any pending zone-blend buffer rather
                    # than waiting indefinitely for a chain partner that may never
                    # arrive. Driven by the poll-with-timeout shape this loop already had,
                    # so it costs no extra wakeup.
                    self._flush_blend_buffer()
                self._poll_gripper_completions()
                continue

            if self._abort is not None:
                # The program already failed — nothing behind the failure may run.
                self._report_aborted(cmd)
                continue

            if self._held is not None:
                # A move is executing with its braking tail held back. Only a command on the
                # same blend lane can be spliced into it; anything else (gripper, delay,
                # a different family/mode/profile, blend=0, ...) means the move must finish
                # normally first, so commit the tail before handling it.
                if (self._is_blend_eligible(cmd)
                        and self._blend_lane(cmd) == self._held["blend_lane"]):
                    # ACCUMULATE — do not splice yet. The same rule as the initial flush:
                    # gather until the lookahead cap, or until forced. While a tail is held
                    # the forcing event is the commit DEADLINE rather than a queue-idle
                    # timeout (_service_held does it).
                    #
                    # Splicing one command at a time does not work, and cannot: every
                    # trajectory must end at REST, so the split has to leave braking room
                    # behind it. A single splice therefore ADDS one short leg while
                    # CONSUMING everything up to the split — the chain shrinks
                    # geometrically and dies (measured: 891 samples -> 719 -> 523 -> too
                    # short to hold -> sent whole -> ARM STOPS). Splicing N commands adds N
                    # legs for the same split, so the chain reaches a steady state and
                    # streams indefinitely.
                    self._blend_buffer.append(cmd)
                    if len(self._blend_buffer) >= self.MAX_BLEND_LOOKAHEAD:
                        self._splice_pending("lookahead cap reached")
                    else:
                        # THE DEADLINE MUST BE CHECKED HERE TOO, not just on the Empty
                        # branch. _service_held() used to be reachable ONLY from
                        # `except Empty:` — i.e. only when the inbox had been quiet for a
                        # whole poll interval. A back-to-back stream never goes quiet, so
                        # queue.get() always succeeded and the deadline was simply never
                        # looked at: measured 0 checks in 20 iterations under a trickle.
                        #
                        # On hardware this hid behind the lookahead cap (the buffer hit 100
                        # and spliced anyway, which happens to reset the hold). But a source
                        # that trickles just fast enough to keep the queue non-empty while
                        # never reaching the cap would sail straight past the deadline with
                        # nobody watching, and RTI would run dry mid-move. The deadline is
                        # the ONE thing standing between a held tail and a velocity
                        # discontinuity — it cannot be conditional on the inbox being idle.
                        self._service_held()
                    self._poll_gripper_completions()
                    continue
                self._commit_held_tail("a non-spliceable command arrived")

            if self._is_blend_eligible(cmd):
                # A chain is homogeneous by family and execution lane. In particular,
                # Ruckig's target-following corner transitions must never be folded into a
                # TOPPRA/Bezier chain (or vice versa).
                if self._blend_buffer and self._blend_lane(self._blend_buffer[0]) != self._blend_lane(cmd):
                    self._flush_blend_buffer()
                self._blend_buffer.append(cmd)
                if len(self._blend_buffer) >= self.MAX_BLEND_LOOKAHEAD:
                    self._flush_blend_buffer()
            else:
                # A non-eligible command (different type, blend=0, gripper/delay,
                # exact_velocity mode, ...) ends any in-flight zone-blend chain —
                # flush what's buffered first, then handle this command normally.
                self._flush_blend_buffer()
                self._run_single(cmd)

            # Check after every command too, not just when idle — a burst of queued
            # joint moves would otherwise starve this (queue.get() never times out).
            self._poll_gripper_completions()

    def _run_single(self, cmd):
        """Plan, send, and mark completion for exactly one command.

        The original unbuffered per-command flow — also reused as the
        individual-fallback path when a zone-blend chain can't be flushed as
        one composite trajectory (wrong profile, IK/TOPPRA failure, etc.).
        """
        if self._abort is not None:
            # Reached through a chain's individual fallback after one of its own commands
            # failed: the rest of the chain is behind the failure too.
            self._report_aborted(cmd)
            return
        try:
            self._mark_started(cmd)
            self._plan_and_send(cmd)
            self._mark_completed(cmd)
        except PlannerError as e:
            cid = self._cmd_id(cmd)
            logger.error(f"command {cid} failed: {e.code}: {e.message}")
            self._mark_error(cmd, e.code, e.message)
            self._report_error(cid, e.code, e.message)
            log_activity(_activity_logger, "EXEC", "error", e.code, id=cid, message=e.message,
                         command=self._fmt_command(cmd))
        except Exception as e:
            cid = self._cmd_id(cmd)
            logger.error(f"command {cid} failed: {e}")
            self._mark_error(cmd, "PLAN_FAIL", str(e))
            self._report_error(cid, "PLAN_FAIL", str(e))
            log_activity(_activity_logger, "EXEC", "error", "PLAN_FAIL", id=cid, message=str(e),
                         command=self._fmt_command(cmd))
        finally:
            self._active_command = None

    # ------------------------------------------------------------------ #
    # Zone blending — TODO/blending_design.md §5
    # ------------------------------------------------------------------ #

    _CARTESIAN_LINEAR_TYPES = ("move_linear", "move_linear_rel_world", "move_linear_rel_tool")

    def _chain_family(self, cmd):
        """'joint' / 'cartesian' / None (not chainable) — a chain only ever
        holds one family; move_linear/_rel_world/_rel_tool are interchangeable
        with each other within one chain, but never with move_joints."""
        ctype = self._cmd_name(cmd)
        if ctype == "move_joints":
            return "joint"
        if ctype in self._CARTESIAN_LINEAR_TYPES:
            return "cartesian"
        return None

    def _is_blend_eligible(self, cmd):
        """True if cmd should be buffered for a zone-blend chain rather than
        planned immediately."""
        if self._chain_family(cmd) is None:
            return False
        options = self._cmd_options(cmd)
        if bool(options.dry_run):
            # A dry run is never part of a chain: the chain is planned and SENT as one trajectory, and nothing in
            # that path looks at dry_run again (2026-10-23 review: a zone-blended dry-run move would have moved the
            # arm). It goes the single-command way, where _plan_and_send's dry-run gate holds; the real commands
            # around it end at rest, as around any non-chainable command.
            return False
        return (options.blend_mode in ("zone", "tolerance", "ruckig_corner")
                and float(options.blend or 0.0) > 0.0)

    def _blend_lane(self, cmd):
        """Return the planner lane that may safely share one blended chain."""
        options = self._cmd_options(cmd)
        geometry_lane = ("ruckig_corner" if options.blend_mode == "ruckig_corner"
                         else "geometric")
        return (self._chain_family(cmd), geometry_lane, options.profile)

    def _blend_radius_mm(self, cmd, p_prev, p, p_next):
        """Return the blend radius (mm) for the command owning this corner.

        `zone` takes the radius straight from the command. `tolerance` (`G64 P<tol>`)
        derives it from the turn angle so the path never strays more than the requested
        tolerance. `zone` and `tolerance` share the geometric TOPPRA lane; Ruckig uses a
        separate execution lane and is never combined with them.
        """
        options = self._cmd_options(cmd)
        blend = float(options.blend or 0.0)
        if options.blend_mode == "tolerance":
            return blend_distance_for_tolerance(p_prev, p, p_next, blend,
                                                options.blend_continuity)
        return blend

    # NOTE (2026-07-12): a _warn_if_blend_undersampled() used to live here, warning that a
    # blend zone smaller than the sample spacing "will NOT be rounded". THAT WAS FALSE and it
    # is deleted rather than fixed, so nobody re-adds it on the same bad reasoning.
    #
    # Both path builders floor the zone at THREE samples (`n = max(3, ...)` in
    # joint_paths/cartesian_paths), so a blend zone is ALWAYS represented no matter how small
    # it is, and the spline rounds the corner correctly. Measured: a 0.05mm tolerance lands a
    # 0.0131 deg corner deviation against a 0.0129 deg prediction.
    #
    # The real defect a tiny zone caused was different and is now FIXED: three samples packed
    # into a sliver of path, under an INDEX-based s, collapsed |dq/ds| there and made TOPPRA
    # brake for phantom curvature (§9.4's failure, through a new door) — 50% slower, not
    # unrounded. build_geometric_path now parameterizes by cumulative chord length, and
    # geometric_gridpoints keeps TOPPRA's constraint evaluation dense inside the zones.

    def _flush_blend_buffer(self):
        """Flush the zone-blend buffer: plan+send the buffered chain (all
        move_joints, or all move_linear-family) as one continuous
        Bezier-blended trajectory, or fall back to planning each command
        individually if blending isn't possible (wrong profile, or the
        composite build/IK/TOPPRA solve fails). Falling back to per-command planning is
        always safe: it changes the path shape at the corners, never the endpoints.
        """
        chain = self._blend_buffer
        self._blend_buffer = []
        if not chain:
            return
        if self._abort is not None:
            for cmd in chain:
                self._report_aborted(cmd)
            return

        family = self._chain_family(chain[0])

        # EXPERIMENTAL lane (TODO/blending_design.md §10.1). Kept entirely off the zone/TOPPRA
        # path below: different engine, different requirements, and deliberately NOT spliceable
        # (dispatched with geometry=None), so none of the rolling-buffer machinery is touched.
        if chain[0].options.blend_mode == "ruckig_corner":
            self._flush_ruckig_corner_chain(chain, family)
            return

        if chain[0].options.profile != "toppra":
            logger.warning(
                f"zone blend requires profile='toppra' (got "
                f"'{chain[0].options.profile}') — planning {len(chain)} "
                f"commands individually instead.")
            for cmd in chain:
                self._run_single(cmd)
            return

        plan_fn = self._plan_blended_chain if family == "joint" else self._plan_blended_cartesian_chain
        try:
            sample, timed_boundaries, waypoints, geometry, geom_boundaries = plan_fn(chain)
            # A blended corner can bulge past a limit that no individual endpoint touches.
            # Failing here falls back to planning each command alone, where the offending
            # one is rejected by name.
            self._check_joint_limits(sample.positions, sample.positions[0],
                                     f"blended chain of {len(chain)}")
            self._check_self_collision(sample.positions, f"blended chain of {len(chain)}")
            self._check_zones(sample.positions, sample.positions[0], f"blended chain of {len(chain)}")
        except Exception as e:
            logger.warning(
                f"zone blend failed for a {len(chain)}-command chain: "
                f"{e} — planning individually instead.")
            for cmd in chain:
                self._run_single(cmd)
            return

        # Preserve chain-level diagnostics across per-command state resets so dry-run
        # preview can export the exact timed blend zones for every command slice.
        chain_diag = self._last_diagnostic
        for cmd in chain:
            self._mark_started(cmd)
        self._last_diagnostic = chain_diag
        self._dispatch_chain(chain, sample, timed_boundaries,
                             self._splice_context(chain, waypoints, family, sample),
                             geometry=geometry, geom_boundaries=geom_boundaries)

    # ------------------------------------------------------------------ #
    # Rolling buffer — TODO/blending_design.md §6
    #
    # The rule (the user's, and it is the right one): a splice must reproduce the
    # batch blend EXACTLY, or be rejected. No degraded half-blends.
    #
    # How that is guaranteed, rather than hoped for:
    #
    #   1. Plan the chain rest-to-rest. Send only its BODY; hold the braking TAIL.
    #   2. The split is placed BEFORE the trajectory starts decelerating, so every
    #      sample we committed lies on TOPPRA's forward pass. The forward pass is
    #      causal — it depends only on the path travelled so far — and the blended
    #      path is geometrically identical to this one up to the corner entry. So the
    #      body we sent is bit-for-bit what the batch blend would have sent.
    #   3. When B arrives, plan the CONTINUATION from the exact state we committed
    #      (sd_start = the path velocity at the split), over the remaining geometry,
    #      with the corner placed at the SAME absolute point. Continuity is then exact
    #      by construction — measured seam error: 0.0 rad, 1e-16 rad/s.
    #   4. "Can we still make the corner from the speed we already committed?" is
    #      answered by TOPPRA itself: if that speed exceeds the backward pass from the
    #      corner, compute_trajectory returns None. That IS the reject condition —
    #      we do not have to invent a heuristic for it.
    #
    # Consequence: we never replan the consumed prefix, so streaming a long toolpath
    # stays O(n), not O(n^2).
    # ------------------------------------------------------------------ #

    # The seam — where the continuation picks up from the body already committed.
    #
    # POSITION must be exact on BOTH lanes, and is: 0.0 rad, measured. That is what the
    # arm's commanded stream actually follows, so a step here would be a real jump.
    SPLICE_SEAM_POS_TOL = 1e-4              # rad (0.006 deg) — regression guard, not slop
    #
    # VELOCITY is per-lane, and it has to be:
    #
    #   joint     — the committed body and the continuation are the SAME straight line,
    #               so sd_start reproduces q̇ to machine precision. Measured: 1e-16 rad/s.
    #               1e-3 is ~1e13x that; it exists to make a regression loud.
    #
    #   cartesian — cannot be exact, for a real reason. The continuation's joint path
    #               comes from a FRESH IK solve, which lands on a slightly different
    #               joint path than the original solve did (both are within the 0.2mm
    #               pose tolerance, but they are not the same path). Differentiating that
    #               difference amplifies it into the tangent DIRECTION, so the arm's
    #               committed velocity is a fraction of a degree off the continuation's
    #               tangent. Measured across geometries: 1.7e-4 .. 7.4e-3 rad/s — and the
    #               whole spliced Cartesian trajectory already differs from the batch by
    #               ~1.9e-2 rad/s for the same reason, so a tighter seam bound would be
    #               claiming a precision the lane does not have anywhere else.
    #               The POSITION seam stays exact, so nothing discontinuous reaches the
    #               arm; the residual is a feedforward term.
    #
    # The Cartesian bound is RELATIVE to the speed at the split, not absolute. The error
    # is an ANGLE between two directions, so the velocity step it produces scales with how
    # fast the arm is going — measured 3.2e-4 rad/s at the current 20% Cartesian speed cap,
    # 6.1e-3 at 50%. An absolute tolerance would therefore silently tighten as the cap is
    # raised (which is planned), and start refusing splices for no physical reason. "The
    # velocity feedforward step must stay under 2% of the speed we are actually travelling
    # at" is the bound that means something, and it holds at any speed.
    #
    # (sd_start itself is a PROJECTION of q̇ onto the tangent, not ‖q̇‖/‖dq/ds‖ — see
    # planner_pipeline.path_velocity_from_joint_velocity. The norm form fed the full
    # magnitude of q̇ into a slightly different direction and made this 4x worse.)
    SPLICE_SEAM_VEL_FLOOR = 1e-3                                 # rad/s, absolute
    SPLICE_SEAM_VEL_FRAC = {"joint": 0.0, "cartesian": 0.02}     # of the speed at the split

    # Slack on the CHEAP analytic braking filter in _choose_split (v^2/2a with the
    # tangential deceleration limit). That bound ignores the acceleration the arm spends
    # CORNERING while it brakes, so it is optimistic — hence a little slack. But only a
    # little: it is a FILTER, not the decision. TOPPRA still confirms every candidate
    # (_brakeable_split), so over-padding here does not buy safety, it just drags the split
    # earlier — and a split that is too early leaves a body under SPLICE_MIN_BODY_S, so no
    # tail is held and the NEXT command cannot splice at all. Measured: 1.6 already breaks
    # chained splicing; 2.0 (the first guess) broke it badly.
    SPLICE_BRAKE_MARGIN = 1.3

    def _seam_vel_tol(self, family, split_speed):
        return max(self.SPLICE_SEAM_VEL_FLOOR,
                   self.SPLICE_SEAM_VEL_FRAC[family] * float(split_speed))

    def _splice_context(self, chain, waypoints, family, sample):
        """What a future splice needs to rebuild the remaining geometry, or None if this
        chain can't be spliced into at all.

        The corner that a future command B would round sits at the chain's LAST waypoint,
        and its size is set by the LAST command's blend radius against the segment
        leading into it. Both are known NOW, before B exists — which is what lets us
        place that corner at the same absolute point later, from a split point part-way
        along the segment.

        Two distances come out of this, and they live in different spaces on purpose:

        * `dist_to_corner` — per-sample distance to the corner, in whatever metric the
          PATH BUILDER clamps on. That is joint radians for the joint lane, but
          millimetres for the Cartesian one (build_composite_cartesian_path clamps the
          blend radius against half the segment length in mm). Using the wrong space here
          would let the builder quietly relocate the corner, so the splice would blend a
          different shape than the batch would have.
        * `d_cut_joint` — the same cut, expressed as joint arc length. The braking bound
          in _choose_split compares SPEEDS, which are joint speeds, so it needs this one.
        """
        if self._read_exec_runway is None or self._robot is None:
            return None
        radius_mm = float(chain[-1].options.blend or 0.0)
        if radius_mm <= 0.0 or len(waypoints) < 2:
            return None
        pos = sample.positions
        n = pos.shape[0]
        if n < 2:
            return None

        ctx = {"family": family, "blend_mm": radius_mm,
               "last_target_q": np.asarray(pos[-1], dtype=np.float64),
               "speed_scale": (self._speed_scale_for(chain[-1].options) if family == "joint"
                               else self._cartesian_speed_scale_for(chain[-1].options))}

        if family == "joint":
            prev_q = np.asarray(waypoints[-2], dtype=np.float64)
            last_q = np.asarray(waypoints[-1], dtype=np.float64)
            tcp_prev = calculate_forward_kinematics(self._robot, prev_q).t * 1000.0
            tcp_last = calculate_forward_kinematics(self._robot, last_q).t * 1000.0
            seg_before_mm = float(np.linalg.norm(tcp_last - tcp_prev))
            if seg_before_mm <= 1e-6:
                return None
            # Exactly the conversion _plan_blended_chain uses, so the corner we place
            # later is the one the batch blend would have placed.
            frac_before = min(radius_mm / seg_before_mm, 0.5)
            d_cut = frac_before * float(np.linalg.norm(last_q - prev_q))
            if d_cut <= 1e-9:
                return None
            ctx["dist_to_corner"] = np.linalg.norm(pos - last_q[None, :], axis=1)
            ctx["cut"] = d_cut              # the metric the joint builder clamps on
            ctx["d_cut_joint"] = d_cut
            return ctx

        # --- Cartesian ---------------------------------------------------------
        prev_pose, last_pose = waypoints[-2], waypoints[-1]
        seg_before_mm = float(np.linalg.norm(last_pose.t - prev_pose.t)) * 1000.0
        if seg_before_mm <= 1e-6:
            return None
        # build_composite_cartesian_path clamps the radius to half the segment, so the
        # cut actually applied is this, not the raw request.
        cut_mm = min(radius_mm, seg_before_mm / 2.0)

        # Per-sample TCP distance to the corner. One FK per sample (~30us on pinocchio);
        # this runs once at plan time, never on the commit deadline.
        tcp = np.array([calculate_forward_kinematics(self._robot, q).t
                        for q in pos], dtype=np.float64) * 1000.0
        dist_mm = np.linalg.norm(tcp - tcp[-1][None, :], axis=1)

        # The same cut, as joint arc length: walk back from the end until the TCP is
        # cut_mm away, and measure how far the joints travelled getting there.
        arc_from_end = np.concatenate((
            [0.0],
            np.cumsum(np.linalg.norm(np.diff(pos[::-1], axis=0), axis=1))))[::-1]
        d_cut_joint = float(np.interp(cut_mm, dist_mm[::-1], arc_from_end[::-1]))
        if d_cut_joint <= 1e-9:
            return None

        ctx["dist_to_corner"] = dist_mm
        ctx["cut"] = cut_mm                 # the metric the Cartesian builder clamps on
        ctx["d_cut_joint"] = d_cut_joint
        ctx["last_target_pose"] = last_pose
        return ctx

    # ------------------------------------------------------------------ #
    # Continuation geometry — REUSE the planned path, don't rebuild it.
    #
    # The old approach handed three raw waypoints to the path builder
    # ([split, corner, new_target]) and let it CONSTRUCT the corner from scratch. To
    # place that Bezier where the batch would have put it, the builder needs a straight
    # run of at least 2x the cut leading into the corner.
    #
    # On a G-code path that run DOES NOT EXIST. Measured on 10.6mm chords with an 8mm
    # blend: every frac clamps at 0.50, so the blend zones meet in the middle of each
    # chord and the straight part has ZERO length — the whole path is Bezier, corner to
    # corner. So every chain-boundary splice was refused, and a 500-move toolpath stopped
    # dead every 100 moves.
    #
    # The fix is to stop rebuilding. We already planned this path, corners and all.
    # Everything from the split onward is exactly the path we still want; only the very
    # END changes (where the arm was going to stop, it now rounds a corner into B):
    #
    #     reuse planned[split : entry]  +  new Bezier round the corner  +  leg to B
    #     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #     copied verbatim — corners already in it stay exactly as planned
    #
    # The split may now land ANYWHERE before the Bezier entry, mid-corner included,
    # because no already-planned corner is being re-placed.
    #
    # Three things fall out of this, not one:
    #   * G-code boundary stops vanish (split anywhere).
    #   * The Cartesian seam becomes EXACT. The reused samples are the original IK
    #     solution, so the tangent at the split is the path the arm is actually on — not
    #     a fresh IK solve pointing a fraction of a degree elsewhere.
    #   * The Cartesian continuation only IK-solves the NEW corner and leg, not the whole
    #     remainder. That is what the 0.25s margin was paying for.
    # ------------------------------------------------------------------ #

    def _geom_index_for_split(self, geometry, timed_positions, split):
        """Which geometry sample the timed split sits at (matched by arc length — the one
        coordinate both samplings share)."""
        ga = self._cumulative_arclength(geometry)
        ta = self._cumulative_arclength(timed_positions)
        idx = int(np.searchsorted(ga, ta[split], side="left"))
        return int(np.clip(idx, 0, geometry.shape[0] - 1))

    def _reused_prefix(self, geometry, geom_split, split_q, d_cut_joint, spacing):
        """The already-planned path from the split up to the new Bezier's entry, resampled
        at uniform arc length.

        Returns (rows, entry_q), or (None, None) if the split is already at/past the entry.

        This does NOT stitch raw samples together. It resamples the SPLINE that TOPPRA
        itself fits through the planned geometry — which is the curve the arm is actually
        on, and which split_q (a TOPPRA sample) lies exactly on by construction.

        Stitching was tried and it fails, for the reason blending_design.md §9.4 already
        taught us: split_q falls BETWEEN two planned samples, so pinning it and then
        continuing from a neighbour leaves one interval of 0.5-1.5x the path spacing. That
        lone irregular step sits at s=0 — precisely where sd_start multiplies q''(s) — and
        the spline reads it as curvature. Measured: |q''| = 22.9 at s=0, giving a
        centripetal demand of 10.6 rad/s^2 against a 3.0 limit, so TOPPRA declared every
        splice infeasible. Resampling the spline puts every sample ON the true curve at a
        genuinely uniform spacing, and the problem disappears.
        """
        if geometry.shape[0] < 4:
            return None, None
        path = build_geometric_path(geometry)

        # Dense arc-length table along the spline (not the polyline).
        ss = np.linspace(0.0, 1.0, max(4 * geometry.shape[0], 400))
        pts = np.asarray(path(ss), dtype=np.float64)
        arc = np.concatenate((
            [0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))))
        total = float(arc[-1])
        if total <= 1e-9:
            return None, None

        # Where the split sits on that spline. split_q is a TOPPRA sample of this very
        # spline, so it lies on it exactly — find it by nearest point.
        i_split = int(np.argmin(np.linalg.norm(pts - split_q[None, :], axis=1)))
        arc_split = float(arc[i_split])
        arc_entry = total - float(d_cut_joint)
        if arc_entry <= arc_split + 0.5 * spacing:
            return None, None      # the split is already inside the new corner's zone

        # Resample in the path's OWN parameter, not in arc length. The two coincide on the
        # joint lane (its geometry IS uniform in joint arc), but NOT on the Cartesian one:
        # there the geometry is uniform in CARTESIAN space, so its joint spacing breathes
        # with the Jacobian. Forcing the prefix to uniform JOINT spacing made it meet the
        # freshly-built Bezier — which is Cartesian-uniform — with a |dq/ds| JUMP, and the
        # planner braked for it exactly as blending_design.md §9.4 describes (measured: the
        # spliced Cartesian move came out 73 samples slower than the batch). Stepping in s
        # preserves whatever spacing character the lane has, so the reused prefix and the
        # new corner join without a kink.
        s_split = float(ss[i_split])
        s_entry = float(np.interp(arc_entry, arc, ss))
        ds = 1.0 / max(geometry.shape[0] - 1, 1)
        n = max(2, int(round((s_entry - s_split) / ds)) + 1)
        s_at = np.linspace(s_split, s_entry, n)
        rows = np.asarray(path(s_at), dtype=np.float64).copy()
        rows[0] = split_q          # pin the seam exactly; the rest is the planned curve
        return rows, rows[-1].copy()

    def _margin_samples(self, family):
        return int(np.ceil(self.SPLICE_SAFETY_MARGIN_S[family] / self._dt))

    def _min_body_s(self, family):
        return self.SPLICE_MIN_BODY_S[family]

    def _exec_runway(self):
        """Samples RTI still has buffered, or None if we can't see it."""
        if self._read_exec_runway is None:
            return None
        try:
            return float(self._read_exec_runway())
        except Exception as e:
            logger.error(f"cannot read RTI runway ({e}) — will not hold a tail")
            return None

    def _choose_split(self, sample, boundaries, splice_ctx):
        """Where to cut the trajectory into body (send now) and tail (hold back).

        Returns a sample index, or None if this trajectory should just be sent whole.

        Three hard bounds, and they are what make the exact-or-reject guarantee both
        TRUE and USEFUL:

        * BEFORE THE FINAL BRAKING. This trajectory ends at REST; the batch blend it must
          reproduce does not (it rounds the corner into B). The two are identical right up
          until this one commits to stopping, so the split must land before that. NOT the
          global speed peak: on a long chain the arm accelerates and decelerates at every
          corner, and those decelerations are in the batch too. What is unique to this
          trajectory is its LAST descent to zero, so the bound is the last local maximum.
        * BEFORE THE NEW CORNER'S BLEND ZONE. We reuse the planned path from the split,
          so no already-planned corner is re-placed and no straight run-in is needed
          anywhere. The only thing we build fresh is the final corner, so the split must
          simply be before where that Bezier starts — one cut-length from the end. (The
          old bound demanded 2x the cut of STRAIGHT run-in, which a G-code path never has:
          with an 8mm blend on 10.6mm chords every frac clamps at 0.5, the blend zones
          meet in the middle, and the straight part has zero length. That is what made
          every chain-boundary splice fail.)
        * SLOW ENOUGH TO STILL MAKE ANY CORNER. The first two bounds alone put the
          split as LATE as possible, which maximises the deadline window — but leaves
          the arm at near-peak speed a hair from the corner, so TOPPRA then (correctly)
          refuses almost every continuation as too fast. Measured: chained splices were
          rejected 100% of the time. We do not know the corner's angle until B arrives,
          so bound the split by the WORST case: the arm must still be able to stop by
          the blend-zone entry, which is the tightest any corner can demand. This is
          read straight off this trajectory's OWN braking curve — it already encodes
          the acceleration limit along this exact path — so it costs no extra solve:
          the speed at arc s is admissible iff it does not exceed the speed this same
          trajectory carries at arc (s + d_cut), i.e. one blend-zone further along its
          own ramp down to rest.
        """
        if splice_ctx is None:
            return None
        n = sample.positions.shape[0]
        if n < 3:
            return None

        # The split may land anywhere in the trajectory now — it is no longer confined to
        # the last command's portion, because we reuse the planned path rather than
        # rebuilding it from waypoints. Whichever commands are not fully sent get held
        # back with the tail and are checkpointed by the continuation.
        lo = 1
        split = n - 1

        # There is NO separate "don't split inside the final braking ramp" bound. There
        # used to be, and every version of it was a heuristic on the speed profile that
        # got the wrong answer:
        #   * "last rising sample"  — a long chain cruises at a FLAT curvature-limited
        #     speed, and flat is not rising, so this pinned the split to the end of the
        #     initial acceleration (sample 108 of 891).
        #   * "start of the last monotonically-decreasing run" — on a path whose curvature
        #     slowly increases, the speed slowly DECLINES for most of the move. That is
        #     cornering, not braking, but the rule read it as the final ramp and pinned the
        #     split to 38% (339 of 891), holding back 65 of 100 commands.
        #
        # It is also unnecessary. Bound 3 (_brakeable_split) subsumes it exactly: if the
        # split were inside the ramp that brakes to a stop at the END, then stopping one
        # blend-zone EARLIER than that is by definition impossible, and TOPPRA says so.
        # One oracle beats three heuristics.

        cut = splice_ctx["cut"]                     # builder's metric — bound 2
        speed = np.linalg.norm(sample.velocities[lo:n], axis=1)

        # bound 2b: a CHEAP analytic pre-filter for braking, so the TOPPRA oracle
        # (_brakeable_split) only has to confirm one or two candidates instead of a dozen.
        #
        # This is the SAME v^2/(2a) bound that failed as a standalone rule — it is only a
        # NECESSARY condition, because it ignores that the arm spends acceleration budget
        # CORNERING while it brakes. But necessary is exactly what a filter needs: a split
        # that fails it is definitely infeasible, so there is no point asking TOPPRA.
        #
        # This is not an optimisation for its own sake. _brakeable_split ran up to 12 full
        # TOPPRA solves, which is nothing on a 2-command continuation and SECONDS on a
        # 155-command one. On hardware that took 2.3s inside _dispatch_chain — while RTI
        # had only 1.46s of body left. RTI ran dry ("0 samples left"), the arm coasted to a
        # near-standstill, and the safety check caused the very failure it exists to
        # prevent. A safety check that is too slow is not a safety check.
        arc_to_end = np.concatenate((
            [0.0],
            np.cumsum(np.linalg.norm(np.diff(sample.positions[lo:n][::-1], axis=0), axis=1))
        ))[::-1]
        dist_to_entry = np.maximum(arc_to_end - splice_ctx["d_cut_joint"], 0.0)
        tangent = np.abs(sample.velocities[lo:n]) / np.maximum(speed[:, None], 1e-9)
        a_along = np.min(self._max_accel[None, :] / np.maximum(tangent, 1e-6), axis=1)
        brake_dist = speed ** 2 / (2.0 * np.maximum(a_along, 1e-9))
        bound = self._last_ok(
            brake_dist * self.SPLICE_BRAKE_MARGIN <= dist_to_entry + 1e-9, lo)
        if bound is None:
            return None     # cannot stop by the blend entry from anywhere
        split = min(split, bound)

        # bound 2: before the new corner's Bezier entry (one cut-length from the end).
        dist_to_corner = splice_ctx["dist_to_corner"][lo:n]
        bound = self._last_ok(dist_to_corner >= cut, lo)
        if bound is None:
            return None     # already inside where the new corner would go
        split = min(split, bound)

        # bound 3: could we still stop by the blend-zone entry?
        # arc_to_end[i] is how far sample i still is from the target; the entry sits d_cut
        # nearer than that. The admissible speed at distance D is read off this
        # trajectory's OWN FINAL BRAKING RAMP — the run from `split` (bound 1) to rest,
        # which is precisely "the fastest you can be at distance D and still stop". No
        # extra solve: the ramp already encodes the accel limit along this exact path.
        #
        # bound 3 is NOT computed here — it cannot be, analytically. See
        # _brakeable_split(): the hardest thing any corner can demand is a full STOP at
        # the blend-zone entry, and whether that is reachable from a given split depends
        # on the curvature of the path in between (which limits how hard you can brake)
        # — so the honest way to answer it is to ask TOPPRA, which is the thing that
        # actually knows. That needs the geometry, so the caller does it.

        # bound 4: LEAVE ENOUGH BEHIND FOR THE NEXT SPLICE.
        # The split has to serve two masters. A LATE split buys the biggest deadline
        # window (more body for B to arrive in) — but the continuation it produces is
        # `[whatever is left after the split] + [B's leg]`, so a late split leaves a SHORT
        # continuation, which then cannot hold a tail of its own, gets sent whole, and ends
        # at REST. Optimising the deadline alone therefore kills streaming: measured on a
        # 500-chord G-code path, the chain spliced 4 commands deep and then stopped dead,
        # four times in 18.5s. Keep at least one minimum-body's worth of trajectory in the
        # tail so the next continuation is holdable too.
        keep = int(np.ceil(self._min_body_s(splice_ctx["family"]) / self._dt))
        split = min(split, n - 1 - keep)

        if split <= 0 or split >= n:
            return None
        if self._body_too_short(split, splice_ctx["family"]):
            return None
        return int(split)

    def _body_too_short(self, split, family):
        """Is the body we would send too short to react inside?

        We hold a tail so we can decide, mid-move, what happens next. That only works if
        RTI has enough runway left for us to notice, plan and send before it runs dry —
        which is what SPLICE_MIN_BODY_S encodes.

        The subtlety is WHOSE runway. The runway RTI will have once this body lands is the
        LEFTOVER it is still chewing on PLUS the body we are about to send — not the body
        alone. Ignoring the leftover made this needlessly strict: it refused to hold a tail
        that had ample runway, which on a stream means the arm stops for no reason. It is
        also what set the streaming threshold at ~7 commands when the physics says ~4.
        The deadline in _service_held() already reads the true runway from
        exec_samples_remaining; this now agrees with it instead of contradicting it.

        Called from BOTH min-body gates — _choose_split's, and the re-check after
        _brakeable_split() walks the split earlier. They MUST use the same rule: when they
        disagreed, _choose_split would approve a hold and the re-check would silently veto
        it, and the move went out whole with no explanation.
        """
        min_body = self._min_body_s(family)
        leftover_s = 0.0
        runway = self._exec_runway()
        if runway is not None:
            leftover_s = max(0.0, float(runway)) * self._dt
        if split * self._dt + leftover_s >= min_body:
            return False
        # Too little runway to react in — send it whole rather than hold a tail we could
        # not commit in time. Say so: this is the failure mode that shows up when the speed
        # limits are raised (a faster move is a SHORTER move, but the margin does not
        # shrink with it — it is dominated by the fixed cost of planning the continuation).
        # Without this line, splicing would simply stop happening one day and nothing would
        # explain why.
        logger.info(
            f"not holding a tail: body would be only {split * self._dt:.2f}s "
            f"(+{leftover_s:.2f}s already buffered in RTI), under the {family} lane's "
            f"{min_body:.2f}s minimum — this move is sent whole and will stop at its "
            f"target")
        return True

    @staticmethod
    def _last_ok(mask, offset):
        """Last index of the CONTIGUOUS leading run of True in `mask`, plus `offset`.
        None if the run is empty — i.e. the bound fails even at the first sample.

        Not the last True anywhere: these bounds all fail progressively along the
        trajectory, but then pass again near the very end where the arm is at rest and
        both sides of the comparison collapse to ~0. Taking the last True would sail
        straight past the failure and pick a split deep in the braking ramp.

        The None case is load-bearing. Returning `offset - 1` there (as this used to)
        hands back an index BEFORE the region we are allowed to split in, and the caller
        then held a tail it could not actually splice into: _choose_split said sample 845
        was fine and _try_splice rejected that very sample on geometry. Found on a
        500-chord G-code path, where the chain's last chord is so short that its blend
        zone consumes the whole segment and no corner run-in exists anywhere in it.
        """
        bad = np.nonzero(~np.asarray(mask))[0]
        if bad.size == 0:
            return offset + len(mask) - 1
        if bad[0] == 0:
            return None
        return offset + int(bad[0]) - 1

    def _brakeable_split(self, sample, split, splice_ctx, geometry, spacing):
        """Walk the split back until the arm could brake to a FULL STOP by the corner
        entry from there. Returns the usable split, or None.

        This is bound 3, and it is answered by TOPPRA rather than approximated.

        The reasoning: we do not know B yet, so we do not know the corner. But the
        HARDEST thing any corner can ask for is a complete stop at its entry (a full
        reversal). If we can still do that from the split, then every real corner — which
        needs less slowing — is reachable too. So: build the geometry from the split to the
        entry (the reused prefix, which we need anyway) and ask for a trajectory from
        sd_start down to zero. TOPPRA either finds one or it doesn't.

        Two analytic attempts at this failed and are worth remembering. Reading the limit
        off the trajectory's own speed profile is meaningless during a cruise, where the
        speed is flat at the CURVATURE limit rather than on a braking curve. And deriving
        it from the acceleration limits (v^2/2a with a = min_j a_max_j/|t^_j|) is only a
        NECESSARY condition: it ignores that the arm is also cornering while it brakes, and
        cornering spends acceleration budget. Both were wrong in the same direction —
        confidently, and by a lot. TOPPRA already models all of this correctly.
        """
        for candidate in self._split_candidates(split):
            split_q = sample.positions[candidate]
            split_qd = sample.velocities[candidate]
            geom_split = self._geom_index_for_split(geometry, sample.positions, candidate)
            prefix, _ = self._reused_prefix(geometry, geom_split, split_q,
                                            splice_ctx["d_cut_joint"], spacing)
            if prefix is None or prefix.shape[0] < 4:
                continue
            try:
                sd0 = path_velocity_from_joint_velocity(prefix, split_qd, s=0.0)
                self._toppra_path_parameterizer.parameterize_path(
                    prefix, self._max_vel, self._max_accel, self._dt,
                    speed_scale=splice_ctx["speed_scale"], sd_start=sd0, sd_end=0.0)
                return candidate
            except TOPPRAInfeasible:
                continue
            except Exception:
                continue
        return None

    # How many TOPPRA confirmations _brakeable_split may spend. This is a HARD BUDGET, not
    # a tuning knob: every solve here happens while RTI is draining the body we already
    # sent, so an expensive check starves the arm. It ran at 12 and took 2.3s on a
    # 155-command continuation, against 1.46s of runway — RTI hit "0 samples left" and the
    # arm coasted to a near-stop. The cheap analytic filter in _choose_split now narrows
    # the range first, so the first candidate is usually already feasible.
    SPLICE_BRAKE_PROBES = 6

    @classmethod
    def _split_candidates(cls, split):
        """Split indices to try, latest first — a late split buys the biggest deadline
        window, so take the latest that can still brake."""
        out, s = [], int(split)
        while s > 4 and len(out) < cls.SPLICE_BRAKE_PROBES:
            out.append(s)
            s = int(s * 0.75) - 1
        return out

    def _dispatch_chain(self, chain, sample, boundaries, splice_ctx, geometry=None,
                        geom_boundaries=None):
        """Send a planned zone-blend chain, holding its braking tail back if a command
        arriving later could still be spliced into it.

        The split may now fall in ANY command's portion, so more than one command can end
        up un-sent. Commands entirely inside the body are checkpointed here exactly as the
        batch path does; the one straddling the split gets a partial, non-terminal send
        (no checkpoint — RTI must play straight on); everything after it is held, and gets
        checkpointed by whatever we send next (a continuation, or the committed tail).
        """
        split = self._choose_split(sample, boundaries, splice_ctx)
        if split is not None and geometry is not None:
            spacing = (self._path_spacing if splice_ctx["family"] == "joint" else
                       float(np.median(np.linalg.norm(np.diff(geometry, axis=0), axis=1))))
            # bound 3: the latest split we could still brake to a full stop from.
            split = self._brakeable_split(sample, split, splice_ctx, geometry,
                                          spacing if spacing > 1e-9 else self._path_spacing)
            if split is not None and self._body_too_short(split, splice_ctx["family"]):
                split = None
        if split is None or geometry is None:
            self._send_blended_chain(chain, sample, boundaries)
            self._finish_chain(chain, sample)
            return

        n = sample.positions.shape[0]
        sent_cmds = sum(1 for b in boundaries if int(b) <= split)

        start_idx = 0
        for i in range(sent_cmds):
            end_idx = int(boundaries[i])
            if not self._send_range(sample.positions, sample.velocities, sample.torque_ff,
                                    start_idx, end_idx, self._cmd_id(chain[i]),
                                    flags=BATCH_FLAG_BLEND_CONTINUES):
                return
            self._mark_completed(chain[i])
            start_idx = end_idx

        # Partial body of the command straddling the split: no checkpoint, is_last_chunk
        # False, so RTI plays straight on into whatever we send next.
        if start_idx < split:
            if not self._send_range(sample.positions, sample.velocities, sample.torque_ff,
                                    start_idx, split, self._cmd_id(chain[sent_cmds]),
                                    terminal=False):
                return

        self._held = {
            "family": splice_ctx["family"],
            "chain": chain,
            "sample": sample,
            "boundaries": boundaries,
            "split": split,
            "blend_lane": self._blend_lane(chain[0]),
            "sent_cmds": sent_cmds,
            "splice": splice_ctx,
            "geometry": geometry,
            "geom_boundaries": geom_boundaries,
            "geom_split": self._geom_index_for_split(geometry, sample.positions, split),
            "sent_at": time.monotonic(),
        }
        # Where the arm ends up IF the tail is committed. A successful splice replaces
        # this. Either way the next command plans from a real endpoint, never from a
        # mid-move sample.
        self._last_endpoint = sample.positions[-1].copy()
        self._last_exit_speed = None
        self._active_command = None
        held_cmds = len(chain) - sent_cmds
        logger.info(f"holding tail: sent {split}/{n} samples "
                    f"({split * self._dt:.2f}s of {n * self._dt:.2f}s), "
                    f"{n - split} samples / {held_cmds} command(s) held for a splice")

    def _finish_chain(self, chain, sample):
        if sample.positions.shape[0]:
            self._last_endpoint = sample.positions[-1].copy()
        # Zone blending's own exit velocity doesn't map onto exact_velocity's
        # matched-corner-speed carry — clear it, same as gripper/delay/
        # move_pose/move_linear already do when they break a blend chain.
        self._last_exit_speed = None
        for cmd in chain:
            self._mark_completed(cmd)
        self._active_command = None

    def _service_held(self):
        """Commit the held tail if RTI is about to run dry.

        If we miss this deadline RTI exhausts its buffer and holds the last body
        waypoint — which still has nonzero velocity — so the arm stops dead from
        speed. Committing early only costs a splice opportunity; committing late is a
        real velocity discontinuity. The check is therefore deliberately trigger-happy.

        exec_samples_remaining is STALE for the first few ms after we send: RTI only
        looks at the socket once per tick, and publishes the count on the next one —
        measured at 8.7ms end-to-end. Trusting it naively means the very first poll
        after holding a tail (5ms later, HOLD_POLL_S) reads ZERO, concludes RTI is
        about to run dry, and commits the tail immediately. That is not hypothetical:
        it shipped, and on hardware it silently un-blended the FIRST corner of every
        stream while every later corner spliced fine (found 2026-07-11 because the arm
        visibly stopped at waypoint 1 — the log showed the tail committed 5ms after the
        body went out, on a reading of "0 samples left").

        The fix is a bound that is true regardless of what RTI has published yet: RTI
        cannot have consumed more of the body than wall-clock time allows, because it
        cannot start before we send. That floor can never exceed the truth (RTI would
        have to consume samples faster than real time), so taking the max with it only
        ever corrects a stale reading upward — it can never mask a real one.

        AND THE FLOOR IS NOT JUST A STALENESS PATCH — it is the primary signal on a
        stream. `exec_samples_remaining` is NOT "everything RTI still has to play". It is
        what RTI holds IN HAND: `_current` + one prefetched `_next` (see
        ExecHandler._mirror_exec_state). But _send_blended_chain sends ONE BATCH PER
        COMMAND, so a 96-command body arrives as ~96 messages — RTI takes two and the
        other ~94 sit unread in the ZMQ socket, counted by nobody.

        So on a G-code stream this field reads ~0 while RTI has seconds of work queued
        behind it, and it is the FLOOR that carries the deadline. Do not "fix" the floor
        because the log looks alarming, and do not treat a low reading as proof RTI is
        starving — measured on hardware: "RTI has 9 samples left" on a 200-chord stream
        that ran to completion with zero standstills. If you ever want the raw field to
        mean what its name suggests, RTI must count the socket backlog too; until then,
        `effective` is the honest number and the raw reading is a lower bound on one part
        of the truth.
        """
        held = self._held
        if held is None:
            return
        body_s = held["split"] * self._dt
        if time.monotonic() - held["sent_at"] > body_s + self.HOLD_GRACE_S:
            self._commit_held_tail(
                f"hold backstop: body ({body_s:.1f}s) should have played out "
                f"{self.HOLD_GRACE_S:.0f}s ago — RTI stalled or paused?")
            return
        runway = self._exec_runway()
        if runway is None:
            self._commit_held_tail("cannot read RTI runway")
            return

        # The floor is now ONLY a stale-read guard, not the primary signal.
        #
        # It exists because exec_samples_remaining is stale for ~8.7ms after a send (RTI reads
        # the socket once per tick and publishes on the next), so the first poll would see a
        # low number and commit instantly. That race is real and it shipped once.
        #
        # But it must NOT outlive that window. RTI now buffers by SAMPLES and publishes its
        # WHOLE runway (ExecHandler.TARGET_BUFFER_SAMPLES), so the reading is trustworthy —
        # whereas the floor assumes RTI held `split` samples at `sent_at`, which is false
        # whenever the send took real time. When the send blocked on a low SEND_HWM that
        # assumption was off by seconds, the floor masked a genuinely empty RTI, and the arm
        # stalled. Trust the measurement; use the floor only where the measurement cannot yet
        # be true.
        elapsed = time.monotonic() - held["sent_at"]
        if elapsed < self.RUNWAY_STALE_S:
            floor = held["split"] - elapsed / self._dt
            effective = max(runway, floor)
        else:
            floor = float("nan")          # not consulted; shown in the log for context
            effective = runway

        margin = self._margin_samples(held["family"])
        if effective <= margin:
            # Report `effective` — the number the decision was actually made on — and show
            # the in-hand reading next to it. Logging `runway` alone was actively
            # misleading: on a G-code stream it prints things like "RTI has 9 samples left"
            # while the true runway is seconds, because exec_samples_remaining counts only
            # what RTI holds IN HAND (_current + _next), and a blended chain is sent as ONE
            # BATCH PER COMMAND — so ~94 of a 96-command body sit unread in the ZMQ socket,
            # uncounted. That log line reads like RTI is one tick from starving. It is not,
            # and chasing it nearly got a correct safety mechanism "fixed" out of the code.
            why = (f"deadline — {effective:.0f} samples of runway left "
                   f"({effective * self._dt:.3f}s), margin is {margin} "
                   f"[RTI in-hand {runway:.0f}, wall-clock floor {floor:.0f}]")
            if self._blend_buffer:
                # Commands are waiting: splice them rather than stopping. This is the
                # deadline doing double duty — it is both "commit or lose the arm" and
                # "stop accumulating, we are out of time to wait for more".
                self._splice_pending(why)
                return
            self._commit_held_tail(why)

    def _commit_held_tail(self, reason):
        """Send the tail we held back. The move now finishes normally.

        The tail can span SEVERAL commands (the split may land anywhere), so each one
        still gets its own checkpoint at its own boundary — BLEND_CONTINUES for all but
        the chain's last, which terminates and settles as usual.
        """
        held = self._held
        self._held = None
        if held is None:
            return
        sample, split = held["sample"], held["split"]
        chain, boundaries = held["chain"], held["boundaries"]
        n = sample.positions.shape[0]
        remaining = chain[held["sent_cmds"]:]
        logger.info(f"committing held tail ({reason}) — {n - split} samples "
                    f"({(n - split) * self._dt:.2f}s), {len(remaining)} command(s)")

        start_idx = split
        for i, cmd in enumerate(remaining):
            k = held["sent_cmds"] + i
            end_idx = int(boundaries[k]) if k < len(boundaries) - 1 else n
            end_idx = max(end_idx, start_idx + 1)
            is_last = (k == len(chain) - 1)
            if not self._send_range(sample.positions, sample.velocities, sample.torque_ff,
                                    start_idx, end_idx, self._cmd_id(cmd),
                                    flags=0 if is_last else BATCH_FLAG_BLEND_CONTINUES):
                return
            self._mark_completed(cmd)
            start_idx = end_idx
        self._active_command = None

    def _try_splice(self, cmds):
        """Splice the WHOLE waiting batch into the move already executing, or reject and
        let it finish.

        Rejection is not a failure — it is the guarantee working. A rejected splice
        means we could not have reproduced the batch blend, so we don't pretend to:
        the in-flight move brakes to its target as planned, and the commands run next.
        """
        if not isinstance(cmds, (list, tuple)):
            cmds = [cmds]
        held = self._held
        sample, split, ctx = held["sample"], held["split"], held["splice"]
        split_q = sample.positions[split]
        split_qd = sample.velocities[split]
        cids = ",".join(self._cmd_id(c) for c in cmds)

        builder = (self._build_joint_continuation if ctx["family"] == "joint"
                   else self._build_cartesian_continuation)
        try:
            geom, handoff, waypoints, reject = builder(held, cmds, split_q)
        except TOPPRAInfeasible:
            geom = handoff = waypoints = None
            reject = "too fast to make the corner from the speed already committed"
        except Exception as e:
            geom = handoff = waypoints = None
            reject = f"continuation solve failed: {e}"

        timed = sd_start = None
        if reject is None:
            try:
                sd_start = path_velocity_from_joint_velocity(geom, split_qd, s=0.0)
                timed = self._toppra_path_parameterizer.parameterize_path(
                    geom, self._max_vel, self._max_accel, self._dt,
                    speed_scale=self._continuation_speed_scale(held),
                    sd_start=sd_start, sd_end=0.0)
            except TOPPRAInfeasible:
                # TOPPRA's own feasibility check IS the reject rule, and this is the
                # expected way to fail: the speed we already committed to the arm is
                # too high to still make this corner. Not an error — the guarantee.
                reject = "too fast to make the corner from the speed already committed"
            except Exception as e:
                reject = f"continuation solve failed: {e}"

        if reject is None:
            # The seam must be exact — the continuation has to pick up precisely where
            # the body left off, or the arm sees a step in position/velocity.
            seam_p = float(np.linalg.norm(timed.positions[0] - split_q))
            seam_v = float(np.linalg.norm(timed.velocities[0] - split_qd))
            vel_tol = self._seam_vel_tol(ctx["family"], np.linalg.norm(split_qd))
            if seam_p > self.SPLICE_SEAM_POS_TOL or seam_v > vel_tol:
                reject = (f"seam not exact (dpos={seam_p:.2e} rad, dvel={seam_v:.2e} "
                          f"rad/s, tol {self.SPLICE_SEAM_POS_TOL:.0e}/{vel_tol:.1e}) "
                          f"— refusing to inject a discontinuity")

        if reject is None:
            # A rejected splice just lets the in-flight move finish; the commands then run as a
            # fresh chain, where the one that leaves the limits is refused by name.
            try:
                self._check_joint_limits(timed.positions, split_q, "splice continuation")
                self._check_zones(timed.positions, split_q, "splice continuation")
                self._check_self_collision(timed.positions, "splice continuation")
            except PlannerError as e:
                reject = e.message

        if reject is not None:
            logger.info(f"splice REJECTED for {len(cmds)} command(s) "
                        f"[{cids}]: {reject} — the in-flight move will finish normally, "
                        f"then they run as a fresh chain")
            log_activity(_activity_logger, "EXEC", "info", "BLEND_SPLICE_REJECTED",
                         id=self._cmd_id(cmds[0]), command=self._fmt_command(cmds[0]),
                         message=reject)
            self._commit_held_tail("splice rejected")
            self._blend_buffer = list(cmds) + self._blend_buffer   # start a fresh chain
            return

        # Accepted. The continuation IS the batch blend's remainder.
        cont = self._sampler.sample(timed)

        # The continuation carries every command that was still held, plus the new one.
        # EVERY un-sent command carries over, not just the last one.
        #
        # The split may land anywhere, so several commands can still be held. Their path is
        # inside the reused prefix — so the arm executes them correctly — but if they are
        # dropped from the chain their CHECKPOINTS never fire and the client hangs on
        # wait_idle forever. That is exactly what happened on hardware: the 200-chord
        # stream traced the circle perfectly and then never finished. It slipped through
        # every earlier test because those only ever held ONE command.
        #
        # Boundaries for the held commands are mapped by ARC LENGTH — the coordinate the
        # original geometry and the resampled continuation share — same as
        # _map_boundaries_to_timed does for the timed/geometry pair.
        held_cmds = list(held["chain"][held["sent_cmds"]:])
        new_chain = held_cmds + list(cmds)

        geom_bounds = []
        if len(held_cmds) > 1 and held["geom_boundaries"] is not None:
            orig_arc = self._cumulative_arclength(held["geometry"])
            cont_arc = self._cumulative_arclength(geom)
            split_arc = float(orig_arc[min(held["geom_split"], len(orig_arc) - 1)])
            for k in range(held["sent_cmds"], len(held["chain"]) - 1):
                b = int(held["geom_boundaries"][k])
                target = float(orig_arc[min(b, len(orig_arc)) - 1]) - split_arc
                idx = int(np.searchsorted(cont_arc, target, side="left")) + 1
                lo_ok = (geom_bounds[-1] + 1) if geom_bounds else 1
                geom_bounds.append(int(np.clip(idx, lo_ok, int(handoff[0]) - 1)))

        geom_bounds += [int(b) for b in handoff]     # last held cmd, then one per new cmd
        geom_bounds[-1] = int(geom.shape[0])

        cont_boundaries = self._map_boundaries_to_timed(geom, geom_bounds, cont.positions)
        self._held = None
        for c in cmds:
            self._mark_started(c)
        logger.info(f"SPLICED {len(cmds)} command(s) ({ctx['family']}) into the "
                    f"in-flight move at sample {split} — continuation "
                    f"{cont.positions.shape[0]} samples, {len(new_chain)} command(s)")
        log_activity(_activity_logger, "EXEC", "info", "BLEND_SPLICED",
                     id=self._cmd_id(cmds[0]), command=self._fmt_command(cmds[0]),
                     message=f"spliced {len(cmds)} cmd(s) at sample {split}, "
                             f"sd_start={sd_start:.4f}")
        self._dispatch_chain(
            new_chain, cont, cont_boundaries,
            self._splice_context(new_chain, waypoints, ctx["family"], cont),
            geometry=geom, geom_boundaries=geom_bounds)

    def _splice_pending(self, why):
        """Splice everything currently buffered into the in-flight move."""
        if self._held is None or not self._blend_buffer:
            return
        batch = self._blend_buffer
        self._blend_buffer = []
        logger.info(f"splicing {len(batch)} buffered command(s) ({why})")
        self._try_splice(batch)

    def _continuation_speed_scale(self, held):
        options = held["chain"][-1].options
        return (self._speed_scale_for(options) if held["splice"]["family"] == "joint"
                else self._cartesian_speed_scale_for(options))

    def _build_joint_continuation(self, held, cmds, split_q):
        """Reuse the planned geometry from the split; build the new corner + the WHOLE
        waiting batch of commands behind it.

        Splicing the batch, not one command at a time, is what makes streaming sustain
        itself. Every trajectory we send must end at REST (we cannot know more commands
        are coming), so the split has to leave braking room behind it — and on a curved
        path at the curvature limit that room is large. A one-command splice therefore
        ADDS one short chord while CONSUMING everything up to the split, so the chain
        shrinks geometrically and dies:
            891 samples -> 719 -> 523 -> too short to hold -> sent whole -> ARM STOPS.
        Splicing N commands adds N chords for the same split, so the chain reaches a
        steady state (braking distance + batch arc) and never dies.

        Returns (geom, handoff_rows, waypoints, reject_reason) where handoff_rows is where
        the OLD chain's last command hands off — its checkpoint index.
        """
        ctx = held["splice"]
        last_q = ctx["last_target_q"]

        targets = []
        for c in cmds:
            args = self._cmd_args(c)
            if len(args) < self.joint_num:
                return None, None, None, "a queued command is malformed"
            targets.append(np.deg2rad(np.asarray(args[:self.joint_num], dtype=np.float64)))

        prefix, entry_q = self._reused_prefix(
            held["geometry"], held["geom_split"], split_q, ctx["d_cut_joint"],
            self._path_spacing)
        if prefix is None:
            return None, None, None, "split is already inside the new corner's blend zone"

        # Corner sizing, verbatim from _plan_blended_chain, so every corner we build lands
        # exactly where a batch of these same commands would have put it.
        waypoints = [last_q] + targets
        tcp = [calculate_forward_kinematics(self._robot, q).t * 1000.0 for q in waypoints]
        fracs = []
        for i in range(len(waypoints) - 2):
            w = i + 1
            sb = float(np.linalg.norm(tcp[w] - tcp[w - 1]))
            sa = float(np.linalg.norm(tcp[w + 1] - tcp[w]))
            r = float(cmds[i].options.blend or 0.0)
            fracs.append((min(r / sb, 0.5) if sb > 1e-6 else 0.0,
                          min(r / sa, 0.5) if sa > 1e-6 else 0.0))

        seg_after_mm = float(np.linalg.norm(tcp[1] - tcp[0]))
        frac_after = (min(ctx["blend_mm"] / seg_after_mm, 0.5)
                      if seg_after_mm > 1e-6 else 0.0)
        exit_q = last_q + frac_after * (targets[0] - last_q)

        spacing = self._path_spacing
        zone, _ = blend_joint_zone(entry_q, last_q, exit_q, spacing)
        # start_trim: the rest of the path starts at the Bezier's EXIT, part-way along the
        # first segment — not at last_q. Without it the builder would treat that exit as a
        # fresh waypoint and recompute every fraction against a shortened segment.
        rest, rest_bounds = build_composite_joint_path(
            waypoints, fracs, arclen_spacing=spacing, start_trim=frac_after)

        geom = np.vstack([prefix, zone[1:], rest[1:]])
        handoff = prefix.shape[0] + zone[1:].shape[0]      # end of the new Bezier
        # rest row j lands at geom row (handoff + j - 1), so a boundary COUNT b becomes:
        bounds = [handoff] + [handoff + int(b) - 1 for b in rest_bounds]
        return geom, bounds, [split_q] + waypoints, None

    def _build_cartesian_continuation(self, held, cmds, split_q):
        """Same as the joint continuation, in pose space — and the reuse is what makes it
        cheap: only the new corner and the new legs are IK-solved (~23ms vs 75ms for
        re-solving the whole remainder). The reused prefix IS the original IK solution, so
        the tangent at the split is the arm's actual direction of travel — which is why the
        Cartesian seam is exact rather than a fraction of a degree off.

        IK can fail for a purely kinematic reason (near-singular, branch jump). That is
        simply one more way to say no: reject, the move finishes, the commands run next.

        Returns (geom, boundary_rows, waypoints, reject_reason).
        """
        if not isinstance(cmds, (list, tuple)):
            cmds = [cmds]
        ctx = held["splice"]
        last_pose = ctx["last_target_pose"]

        cart_spacing = float(np.median(
            np.linalg.norm(np.diff(held["geometry"], axis=0), axis=1)))
        prefix, entry_q = self._reused_prefix(
            held["geometry"], held["geom_split"], split_q, ctx["d_cut_joint"],
            cart_spacing if cart_spacing > 1e-9 else self._path_spacing)
        if prefix is None:
            return None, None, None, "split is already inside the new corner's blend zone"

        backend = self._fk_jacobian_backend()
        set_kinematics_backend(backend)
        entry_pose = calculate_forward_kinematics(self._robot, entry_q)

        # Each command resolves against the PREVIOUS one's pose, so relative moves chain
        # exactly as _plan_blended_cartesian_chain does.
        targets, cur = [], last_pose
        for c in cmds:
            tp, _ = self._target_pose_for_linear(self._cmd_name(c), cur, self._cmd_args(c))
            targets.append(tp)
            cur = tp

        waypoints = [last_pose] + targets
        radii = [float(cmds[i].options.blend or 0.0) for i in range(len(cmds) - 1)]

        seg_after_mm = float(np.linalg.norm(targets[0].t - last_pose.t)) * 1000.0
        if seg_after_mm <= 1e-6:
            return None, None, None, "next command is a zero-length move"
        cut_after = min(ctx["blend_mm"], seg_after_mm / 2.0)
        frac_after = cut_after / seg_after_mm
        exit_pose = interp_pose(last_pose, targets[0], frac_after)

        spacing = CART_ARCLEN_SPACING_M
        zone_poses, _ = blend_zone_poses(entry_pose, last_pose, exit_pose, spacing)
        rest, rest_bounds = build_composite_cartesian_path(
            waypoints, radii, arclen_spacing=spacing, start_trim=frac_after)

        new_poses = list(zone_poses) + list(rest.poses)[1:]
        cart_path = CartesianPath(
            poses=tuple(new_poses), s=np.linspace(0.0, 1.0, len(new_poses)),
            translation_length=0.0, rotation_angle=0.0)

        solver = SeededIKPathSolver(
            self._robot, self.joint_num,
            solver=normalize_ik_solver_name(cmds[0].options.ik_solver or "pinocchio"),
            config=DEFAULT_IK_CONFIG, kinematics_backend=backend)
        result = solver.solve(cart_path, entry_q)   # seeded from the reused path's end
        if not result.success:
            return None, None, None, (
                f"continuation IK failed ({result.failure_code}): "
                f"{result.failure_message}")

        new_joints = result.positions
        geom = np.vstack([prefix, new_joints[1:]])
        nz = len(zone_poses)
        handoff = prefix.shape[0] + nz - 1                       # end of the new Bezier
        bounds = [handoff] + [prefix.shape[0] + nz + int(b) - 2 for b in rest_bounds]
        bounds[-1] = geom.shape[0]
        return geom, bounds, [split_q] + waypoints, None

    def _plan_blended_chain(self, chain):
        """Build + time-parameterize a joint-space zone-blended composite path
        for a buffered chain of move_joints commands. Raises on failure —
        caller falls back to planning each command individually.

        Returns (ExecTrajectorySample, timed_boundaries) — timed_boundaries[i]
        is the row index in the sample where chain[i]'s handoff lands, for
        _send_blended_chain() to checkpoint against.
        """
        if self._robot is None:
            raise NotImplementedError("zone blending needs a robot model for FK-based zone sizing")

        start = self._start_point()
        waypoints_rad = [start]
        for cmd in chain:
            args = self._cmd_args(cmd)
            if len(args) < self.joint_num:
                raise ValueError(f"move_joints needs {self.joint_num} angles, got {len(args)}")
            waypoints_rad.append(np.deg2rad(np.asarray(args[:self.joint_num], dtype=np.float64)))

        # FK-based mm -> segment-fraction conversion (TCP distance per segment): the blend
        # radius the user gives is in TCP millimetres, but the blend itself happens in joint
        # space, so each segment needs its own mm-per-fraction scale.
        tcp_mm = [calculate_forward_kinematics(self._robot, q).t * 1000.0 for q in waypoints_rad]
        blend_fracs = []
        for i in range(len(chain) - 1):
            wp_idx = i + 1
            seg_before = float(np.linalg.norm(tcp_mm[wp_idx] - tcp_mm[wp_idx - 1]))
            seg_after = float(np.linalg.norm(tcp_mm[wp_idx + 1] - tcp_mm[wp_idx]))
            r = self._blend_radius_mm(chain[i], tcp_mm[wp_idx - 1], tcp_mm[wp_idx],
                                      tcp_mm[wp_idx + 1])
            frac_before = min(r / seg_before, 0.5) if seg_before > 1e-6 else 0.0
            frac_after = min(r / seg_after, 0.5) if seg_after > 1e-6 else 0.0
            blend_fracs.append((frac_before, frac_after))

        geometry_positions, boundaries, geometry_blend_zones = build_composite_joint_path(
            waypoints_rad, blend_fracs, arclen_spacing=self._path_spacing,
            continuity=self._cmd_options(chain[0]).blend_continuity,
            return_blend_zones=True)

        speed_scale = self._speed_scale_for(chain[0].options)
        timed = self._toppra_path_parameterizer.parameterize_path(
            geometry_positions, self._max_vel,
            self._max_accel * self._accel_scale_for(chain[0].options),
            self._dt, speed_scale=speed_scale)

        timed_boundaries = self._map_boundaries_to_timed(geometry_positions, boundaries, timed.positions)
        sample = self._assert_commanded_accel(
            self._sampler.sample(timed),
            self._joint_accel_limit(self._cmd_options(chain[0])),
            "zone/tolerance joint chain")
        self._last_diagnostic = {"blend_zones": self._map_blend_zones_to_timed(
            geometry_positions, geometry_blend_zones, timed.positions)}
        return sample, timed_boundaries, waypoints_rad, geometry_positions, boundaries

    def _flush_ruckig_corner_chain(self, chain, family):
        """EXPERIMENTAL jerk-limited blending (TODO/blending_design.md §10.1).

        Two hard requirements, both REJECTED loudly rather than silently downgraded:

          joint-space only — the legs must be straight lines that Ruckig itself generates.
            A Cartesian move's legs are prescribed (the tool must track a line), and Ruckig
            is target-following: it would discard the line and carve an arc. Measured, with
            the rest of the reasoning, in dummy_test_scripts/probe_ruckig_corner_cartesian.py.

          profile="ruckig" — the whole chain is Ruckig-generated, legs and corners alike.
            Requiring the profile to say so keeps the command self-describing instead of
            silently overriding whatever the caller asked for.

        Falls back to individual moves on any failure, exactly like the zone lane.
        """
        if family != "joint":
            logger.warning(
                f"blend_mode='ruckig_corner' is joint-space only (got a "
                f"'{family}' chain) — planning {len(chain)} commands individually instead.")
            for cmd in chain:
                self._run_single(cmd)
            return

        if chain[0].options.profile != "ruckig":
            logger.warning(
                f"blend_mode='ruckig_corner' requires profile='ruckig' (got "
                f"'{chain[0].options.profile}') — planning {len(chain)} commands "
                f"individually instead.")
            for cmd in chain:
                self._run_single(cmd)
            return

        try:
            sample, boundaries = self._plan_ruckig_corner_chain(chain)
            self._check_joint_limits(sample.positions, sample.positions[0],
                                     f"ruckig_corner chain of {len(chain)}")
            self._check_self_collision(sample.positions, f"ruckig_corner chain of {len(chain)}")
            self._check_zones(sample.positions, sample.positions[0], f"ruckig_corner chain of {len(chain)}")
        except Exception as e:
            logger.warning(
                f"ruckig corner blend failed for a {len(chain)}-command chain: "
                f"{e} — planning individually instead.")
            for cmd in chain:
                self._run_single(cmd)
            return

        # _mark_started() CLEARS _last_diagnostic — it is a per-COMMAND channel, and a fresh
        # command must not inherit the previous one's diagnostics. But a chain diagnostic
        # describes the whole chain, and it was written during planning, above. Marking every
        # command started therefore wiped it and `last_diagnostic` came back null on hardware
        # while the motion itself was perfectly correct. Capture across, then re-publish.
        chain_diag = self._last_diagnostic
        for cmd in chain:
            self._mark_started(cmd)
        self._last_diagnostic = chain_diag
        # geometry=None and splice_ctx=None => _dispatch_chain sends the whole chain and
        # never enters the rolling-buffer path. That is deliberate: the splice machinery
        # re-plans continuations through TOPPRA's path parameterization, which this lane
        # does not have. A ruckig_corner stream therefore still stops between flushes.
        self._dispatch_chain(chain, sample, boundaries, None, geometry=None)

    def _plan_ruckig_corner_chain(self, chain):
        """Ruckig legs + full-state Ruckig corner transitions for a move_joints chain.

        Returns (ExecTrajectorySample, boundaries) — boundaries[i] is the row where
        chain[i] has been fully handed off, same contract as _plan_blended_chain's
        timed_boundaries, so _dispatch_chain checkpoints identically.
        """
        if self._robot is None:
            raise NotImplementedError(
                "ruckig corner blending needs a robot model for FK-based zone sizing")

        start = self._start_point()
        waypoints_rad = [start]
        for cmd in chain:
            args = self._cmd_args(cmd)
            if len(args) < self.joint_num:
                raise ValueError(f"move_joints needs {self.joint_num} angles, got {len(args)}")
            waypoints_rad.append(np.deg2rad(np.asarray(args[:self.joint_num], dtype=np.float64)))

        # `blend` stays in MILLIMETRES, exactly like `zone`, so one unit story holds across
        # every blend mode. Convert mm -> segment fraction via FK (TCP distance), then that
        # fraction -> a JOINT-space cut radius, which is the ball ruckig_corner cuts on.
        # This is the same mm->fraction->d_cut conversion _splice_context already performs.
        tcp_mm = [calculate_forward_kinematics(self._robot, q).t * 1000.0 for q in waypoints_rad]
        blend_dists = []
        for i in range(len(chain) - 1):
            wp = i + 1
            seg_before_mm = float(np.linalg.norm(tcp_mm[wp] - tcp_mm[wp - 1]))
            seg_after_mm = float(np.linalg.norm(tcp_mm[wp + 1] - tcp_mm[wp]))
            r = self._blend_radius_mm(chain[i], tcp_mm[wp - 1], tcp_mm[wp], tcp_mm[wp + 1])
            frac_before = min(r / seg_before_mm, 0.5) if seg_before_mm > 1e-6 else 0.0
            frac_after = min(r / seg_after_mm, 0.5) if seg_after_mm > 1e-6 else 0.0
            d_before = frac_before * float(np.linalg.norm(waypoints_rad[wp] - waypoints_rad[wp - 1]))
            d_after = frac_after * float(np.linalg.norm(waypoints_rad[wp + 1] - waypoints_rad[wp]))
            blend_dists.append((d_before, d_after))

        opts = self._cmd_options(chain[0])
        jerk_factor = RUCKIG_JERK_FACTOR * float(opts.jerk if opts.jerk is not None else 1.0)
        result = plan_ruckig_corner_chain(
            waypoints_rad, self._max_vel,
            self._max_accel * self._accel_scale_for(opts), self._dt, blend_dists,
            jerk_factor=jerk_factor, speed_scale=self._speed_scale_for(opts),
            # Same percentage semantics as jerk_factor above: `jerk` scales the budget,
            # whichever source states it. None here keeps the old derivation.
            max_jerk=(None if self._max_jerk is None
                      else self._max_jerk * float(opts.jerk if opts.jerk is not None else 1.0)))

        timed = TimedJointPath(positions=result.positions,
                               velocities=result.velocities,
                               num_points=result.num_points)
        self._last_diagnostic = dict(self._last_diagnostic or {})
        self._last_diagnostic["ruckig_corners"] = [
            {"waypoint": c.waypoint_index, "blended": c.blended,
             "entry_speed": round(c.entry_speed, 4), "exit_speed": round(c.exit_speed, 4),
             "cut_radius_rad": round(0.5 * (c.d_before + c.d_after), 4),
             "closest_approach_rad": round(c.min_corner_distance, 4),
             # Ruckig invents the corner path, so its deviation is MEASURED, never bounded a
             # priori. Reporting it is the only honest option — and it is what tells you the
             # corner actually stayed inside the blend radius you asked for.
             "deviation_rad": round(c.deviation, 4),
             "deviation_within_blend": bool(c.deviation <= min(c.d_before, c.d_after))}
            for c in result.corners]
        last_sample = timed.positions.shape[0] - 1
        self._last_diagnostic["blend_zones"] = [
            {"corner_index": int(window["corner_index"]),
             "start_index": int(np.clip(np.ceil(window["start_time_s"] / self._dt), 0, last_sample)),
             "end_index": int(np.clip(np.floor(window["end_time_s"] / self._dt + 1e-9), 0, last_sample)),
             "kind": "ruckig_corner"}
            for window in result.corner_time_ranges
        ]
        for zone in self._last_diagnostic["blend_zones"]:
            zone["end_index"] = max(zone["start_index"], zone["end_index"])
        return (self._assert_commanded_accel(
                    self._sampler.sample(timed), self._joint_accel_limit(opts),
                    "ruckig_corner chain"),
                result.command_end_rows())

    def _plan_blended_cartesian_chain(self, chain):
        """Build + IK-solve + time-parameterize a Cartesian zone-blended
        composite path for a buffered chain of move_linear-family commands.
        Raises on failure (IK or TOPPRA) — caller falls back to planning each
        command individually, same as _plan_blended_chain().

        Each command resolves its own target pose against the *previous*
        command's resolved pose in the chain (not always the chain's start),
        so move_linear_rel_world/_rel_tool chain correctly — a relative move's
        delta is relative to where the command before it in the chain ends up,
        exactly as if each had been sent and completed individually.

        Returns (ExecTrajectorySample, timed_boundaries) — see
        _plan_blended_chain() for the boundary contract; identical here since
        SeededIKPathSolver preserves one joint solution per Cartesian sample,
        so the Cartesian composite's boundaries index result.positions directly.
        """
        if self._robot is None:
            raise NotImplementedError("zone blending needs a robot model for IK")

        solver_name = normalize_ik_solver_name(chain[0].options.ik_solver or "pinocchio")
        # FK/Jacobian backend is independent of the IK engine (fast pinocchio, rtb
        # fallback) — set it for the pre-solve FK below and pass it to the solver.
        backend = self._fk_jacobian_backend()
        set_kinematics_backend(backend)

        start = self._start_point()
        start_pose = calculate_forward_kinematics(self._robot, start)
        waypoints_se3 = [start_pose]
        current_pose = start_pose
        for cmd in chain:
            ctype = self._cmd_name(cmd)
            args = self._cmd_args(cmd)
            target_pose, _ = self._target_pose_for_linear(ctype, current_pose, args)
            waypoints_se3.append(target_pose)
            current_pose = target_pose

        pts_mm = [np.asarray(w.t, dtype=np.float64) * 1000.0 for w in waypoints_se3]
        blend_radii_mm = [
            self._blend_radius_mm(chain[i], pts_mm[i], pts_mm[i + 1], pts_mm[i + 2])
            for i in range(len(chain) - 1)]
        cart_path, boundaries, geometry_blend_zones = build_composite_cartesian_path(
            waypoints_se3, blend_radii_mm,
            continuity=self._cmd_options(chain[0]).blend_continuity,
            return_blend_zones=True)

        solver = SeededIKPathSolver(
            self._robot, self.joint_num, solver=solver_name, config=DEFAULT_IK_CONFIG,
            kinematics_backend=backend)
        result = solver.solve(cart_path, start)

        if not result.success:
            raise PlannerError(
                result.failure_code or "IK_PARTIAL_PATH",
                f"{result.failure_message} at sample {result.first_failure_index}")

        diagnostic = result.diagnostic()
        singular = DEFAULT_IK_CONFIG.is_singularity_warning(
            diagnostic.get("max_jacobian_condition"), diagnostic.get("min_jacobian_sigma"))
        if singular:
            log_activity(
                _activity_logger, "EXEC", "warning", "IK_SINGULARITY_WARNING",
                id=self._cmd_id(chain[0]), command=f"zone_chain({len(chain)} commands)",
                message=(f"condition={diagnostic.get('max_jacobian_condition')}, "
                         f"sigma_min={diagnostic.get('min_jacobian_sigma')}"))

        speed_scale = self._cartesian_speed_scale_for(chain[0].options)
        timed = self._toppra_path_parameterizer.parameterize_path(
            result.positions, self._max_vel,
            self._max_accel * self._accel_scale_for(chain[0].options),
            self._dt, speed_scale=speed_scale)

        timed_boundaries = self._map_boundaries_to_timed(result.positions, boundaries, timed.positions)
        sample = self._assert_commanded_accel(
            self._sampler.sample(timed),
            self._accel_backstop_limit(chain[0].options),
            "zone/tolerance cartesian chain")
        diagnostic["blend_zones"] = self._map_blend_zones_to_timed(
            result.positions, geometry_blend_zones, timed.positions)
        self._last_diagnostic = diagnostic
        # The IK-solved joint path IS the geometry for this lane — a splice reuses it
        # verbatim from the split, so the continuation only IK-solves the new corner.
        return sample, timed_boundaries, waypoints_se3, result.positions, boundaries

    def _map_boundaries_to_timed(self, geometry_positions, geometry_boundaries, timed_positions):
        """Map each geometry-sample boundary (the end of a command's portion) to
        the corresponding row in the TOPPRA-timed trajectory. TOPPRA resamples
        the SAME geometric path at its own dt-spaced times, so geometry row
        indices don't line up 1:1 with the timed output.

        Mapped by CUMULATIVE ARC LENGTH, which increases monotonically along the
        path and is therefore a shared coordinate between the geometry and timed
        samplings, regardless of how either is sampled in time.

        (An earlier version matched by nearest joint position via argmin. That is
        WRONG for any path that revisits a position — e.g. a start->...->start
        loop — because the final command's target position also appears at the
        start of the trajectory, so argmin snaps its boundary to an early sample,
        collapses the boundary ordering, leaves that command's send range empty,
        and its checkpoint never fires (client hangs on wait_idle). Found on
        hardware 2026-07-10 with a start->p1->p2->start test. Arc length fixes it
        properly: the return-to-start sample has the LARGEST arc length, so it
        maps to the LAST timed index, not an early one.)
        """
        geom_cum = self._cumulative_arclength(geometry_positions)
        timed_cum = self._cumulative_arclength(timed_positions)
        n_timed = timed_positions.shape[0]

        timed_boundaries = []
        prev = 0
        for b in geometry_boundaries:
            target_arc = geom_cum[min(int(b), len(geom_cum)) - 1]
            idx = int(np.searchsorted(timed_cum, target_arc, side="left")) + 1
            # Guarantee strictly increasing, non-empty ranges — a collapsed
            # boundary would silently drop a command's checkpoint (see above).
            idx = max(prev + 1, min(idx, n_timed))
            timed_boundaries.append(idx)
            prev = idx
        timed_boundaries[-1] = n_timed  # last command always ends exactly at the end
        return timed_boundaries

    def _map_blend_zones_to_timed(self, geometry_positions, geometry_blend_zones,
                                  timed_positions):
        """Map exact composite-path blend-zone sample ranges onto timed samples."""
        if not geometry_blend_zones:
            return []
        geometry_arc = self._cumulative_arclength(geometry_positions)
        timed_arc = self._cumulative_arclength(timed_positions)
        last = timed_positions.shape[0] - 1
        timed_zones = []
        for zone in geometry_blend_zones:
            start_geometry = max(0, min(int(zone["start_index"]), len(geometry_arc) - 1))
            end_geometry = max(start_geometry, min(int(zone["end_index"]), len(geometry_arc) - 1))
            start_index = int(np.searchsorted(timed_arc, geometry_arc[start_geometry], side="left"))
            end_index = int(np.searchsorted(timed_arc, geometry_arc[end_geometry], side="left"))
            timed_zones.append({
                "corner_index": int(zone["corner_index"]),
                "start_index": max(0, min(start_index, last)),
                "end_index": max(0, min(max(start_index, end_index), last)),
            })
        return timed_zones

    @staticmethod
    def _cumulative_arclength(positions):
        """Cumulative joint-space path length at each sample: [0, d01, d01+d12, ...]."""
        if positions.shape[0] < 2:
            return np.zeros(positions.shape[0], dtype=np.float64)
        seg = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        return np.concatenate(([0.0], np.cumsum(seg)))

    def _send_blended_chain(self, chain, sample, timed_boundaries):
        """Send one composite blended trajectory, with each command in `chain`
        getting its own checkpoint at its timed_boundaries[i] sample index.
        BATCH_FLAG_BLEND_CONTINUES on every command but the last — the same
        mechanism (motion/exec_handler.py::_begin_completion) already used for
        the existing move_joints velocity-continuous blend, which fires a
        checkpoint's completion without holding/stopping when set.
        """
        start_idx = 0
        n = sample.positions.shape[0]
        for i, cmd in enumerate(chain):
            end_idx = timed_boundaries[i]
            # Defense in depth: an empty range would send no batch, so this
            # command's checkpoint would never fire and the client would hang on
            # wait_idle. _map_boundaries_to_timed already guarantees strictly
            # increasing boundaries; this guard makes a regression loud, not silent.
            if end_idx <= start_idx:
                logger.error(f"blended-chain boundary collapse at cmd {i} "
                             f"(start={start_idx}, end={end_idx}) — forcing a 1-sample "
                             f"range so the checkpoint still fires")
                end_idx = min(start_idx + 1, n)
            cid = self._cmd_id(cmd)
            is_last_cmd = (i == len(chain) - 1)
            flags = 0 if is_last_cmd else BATCH_FLAG_BLEND_CONTINUES
            sent = self._send_range(sample.positions, sample.velocities, sample.torque_ff,
                                     start_idx, end_idx, cid, flags=flags)
            if not sent:
                return False
            start_idx = end_idx
        return True

    def _start_point(self):
        if self._resync:
            # RTI left EXEC since the last move was planned (note_exec_exit / interrupt): the
            # arm may have been jogged, streamed, pushed or homed. Start from where it IS.
            self._resync = False
            self._last_endpoint = None
            self._last_exit_speed = None
        if self._last_endpoint is not None:
            return self._last_endpoint
        # A SNAPSHOT. read_current_joints returns a live view of shared memory and np.asarray of a float64 view
        # does not copy: `start` kept changing while the move was planned and checked, so a robot settling 0.4 mm
        # between the two made the zone check see the move's first sample as "deeper than the start" and refuse
        # the move back OUT of a zone (api_tests/53, 2026-10-02). The joint-limit recovery check had the same race.
        return np.array(self._read_current_joints(), dtype=np.float64)[:self.joint_num]

    def _cmd_id(self, cmd):
        return cmd.id if isinstance(cmd, QueuedCommand) else cmd.get("id", "")

    def _cmd_seq(self, cmd):
        return cmd.seq if isinstance(cmd, QueuedCommand) else int(cmd.get("seq", 0))

    def _cmd_name(self, cmd):
        # An old name of a command is that command (2026-10-23): the executor already maps them, this covers every
        # other way in (tests, tools, the offline planner fed directly).
        return canonical_command(cmd.cmd if isinstance(cmd, QueuedCommand) else cmd.get("cmd"))

    def _cmd_args(self, cmd):
        return list(cmd.args) if isinstance(cmd, QueuedCommand) else (cmd.get("args", []) or [])

    def _cmd_options(self, cmd):
        if isinstance(cmd, QueuedCommand):
            return cmd.options
        return MotionOptions.from_dict(cmd.get("options"))

    def _fmt_command(self, cmd):
        """Raw, untouched command call text for the activity log's `command` field."""
        args = self._cmd_args(cmd)
        name = self._cmd_name(cmd)
        return f"{name}({', '.join(map(str, args))})" if args else str(name)

    def _mark_started(self, cmd):
        seq = self._cmd_seq(cmd)
        self._executing_seq = seq
        self._active_command = self._cmd_name(cmd)
        options = self._cmd_options(cmd)
        self._current_profile = normalize_profile_name(options.profile or DEFAULT_PROFILE)
        self._last_diagnostic = None
        log_activity(_activity_logger, "EXEC", "info", "CMD_STARTED",
                     id=self._cmd_id(cmd), command=self._fmt_command(cmd),
                     context=self._command_context(options))

    def _command_context(self, options):
        """Effective per-command config for CMD_STARTED's `context` field.

        profile/ik_solver/speed/blend/completion_policy/singularity_strategy/
        blend_strategy are real, resolved MotionOptions that genuinely vary per
        command. control_mode/feedforward remain placeholders (the only value each
        can produce today) — kept as a reminder for when each becomes a real
        per-command choice (see TODO/activity_log_design.md §9): control_mode
        (PID vs PD selection), feedforward (add Coriolis/centripetal/friction).
        """
        return {
            "profile": self._current_profile,
            "ik_solver": normalize_ik_solver_name(options.ik_solver or "pinocchio"),
            "speed": self._speed_scale_for(options),
            "blend": self._blend_pct_for(options),
            "completion_policy": options.completion_policy or "settled",
            # Real now: reflect the command's actual blend mode + singularity policy
            # (blend_strategy previously always logged "exact_velocity" even for zone
            # blends — actively misleading during this session's debugging).
            "singularity_strategy": options.singularity_strategy or "stop",
            "blend_strategy": normalize_blend_mode_name(options.blend_mode or "exact_velocity"),
            "control_mode": "pid",
            "feedforward": ["position", "velocity", "torque_ff"],
        }

    def _mark_completed(self, cmd):
        self._completed_seq = self._cmd_seq(cmd)
        self._last_error = None
        log_activity(_activity_logger, "EXEC", "info", "CMD_COMPLETED",
                     id=self._cmd_id(cmd), command=self._fmt_command(cmd))

    def _mark_error(self, cmd, code, message):
        self._last_error = {
            "seq": self._cmd_seq(cmd),
            "cmd": self._cmd_name(cmd),
            "code": code,
            "message": message,
        }

    def _speed_scale_for(self, options):
        if options.speed is None:
            return self._speed_scale
        speed = float(options.speed)
        # Per-command speed accepts either API style: 0.4 = 40%, 40 = 40%.
        scale = speed if speed <= 1.0 else speed / 100.0
        return float(np.clip(scale, 1e-3, 1.0))

    def _accel_scale_for(self, options):
        """`accel` as a fraction of the acceleration limit — the `a` of MoveJoint(v, a, t).

        Was ACCEPTED AND SILENTLY IGNORED: `options.accel` was never read anywhere in this
        file, so move_joints(..., accel=25) planned exactly the same trajectory as without it.
        Same silent-drop class as blend_continuity.

        Scales the acceleration limit the way `speed` scales the velocity limit. Lower accel =
        gentler ramps = a longer move, which is the point of asking for it.
        """
        if options.accel is None:
            return 1.0
        a = float(options.accel)
        scale = a if a <= 1.0 else a / 100.0        # accepts 0.25 or 25
        return float(np.clip(scale, 1e-3, 1.0))

    def _scale_for_duration(self, options, natural_duration_s, speed_scale):
        """Re-scale so the move takes exactly `duration` seconds — the `t` of MoveJoint.

        Was also ACCEPTED AND SILENTLY IGNORED (options.duration is read exactly once in this
        file, and that is for the `delay` command, not for motion).

        Every profile here time-scales the same way: velocity by `scale`, acceleration by
        `scale^2`, jerk by `scale^3` — so the DURATION scales by 1/scale, exactly, for
        trapezoid, quintic, s_curve, TOPPRA and Ruckig alike. So: plan once, see how long the
        move naturally takes at `speed_scale`, and rescale to hit `t`.

        `t` OVERRIDES `speed`, matching the MoveJoint contract. But it cannot override PHYSICS:
        if `t` is shorter than the fastest legal move, we reject rather than silently violate a
        joint limit. Saying "I cannot do that in 0.2s" is the only honest answer.
        """
        if options.duration is None:
            return speed_scale
        t = float(options.duration)
        if t <= 0.0:
            raise PlannerError("INVALID_DURATION", f"duration must be > 0 (got {t})")
        fastest = natural_duration_s * speed_scale      # duration at scale = 1.0
        if t < fastest - 1e-9:
            raise PlannerError(
                "DURATION_INFEASIBLE",
                f"cannot complete this move in {t:.3f}s — the joint velocity/acceleration "
                f"limits make {fastest:.3f}s the fastest possible. Ask for {fastest:.3f}s or "
                f"longer, or raise the limits.")
        return float(np.clip(fastest / t, 1e-3, 1.0))

    def _blend_pct_for(self, options):
        if options.blend is None:
            return self._blend_pct
        return float(np.clip(options.blend, 0.0, 100.0))

    def _peek_next_command(self):
        """Look at (don't remove) the next queued command, for blend lookahead.
        Queue has no public peek(); only this thread ever calls .get() (removes
        from the left) while TCP handler threads only .put_nowait() (append to the
        right) — reading index 0 concurrently with an append on the other end is
        safe (queued dicts are never mutated in place after being put, either)."""
        try:
            return self._queue.queue[0]
        except IndexError:
            return None

    def _sample_joint_path(self, path, cid, options, start_vel=None, end_vel=None):
        profile = normalize_profile_name(options.profile or DEFAULT_PROFILE)
        time_parameterizer = make_time_parameterizer(profile, options.ramp_fraction, options.jerk,
                                                     max_jerk=self._max_jerk)
        max_accel = self._max_accel * self._accel_scale_for(options)
        speed_scale = self._speed_scale_for(options)
        try:
            if options.duration is not None:
                # Measure the move's natural length, then rescale to hit `t` exactly.
                probe = time_parameterizer.parameterize(
                    path, self._max_vel, max_accel, self._dt, speed_scale=speed_scale,
                    start_vel=start_vel, end_vel=end_vel)
                speed_scale = self._scale_for_duration(
                    options, probe.num_points * self._dt, speed_scale)
            timed = time_parameterizer.parameterize(
                path, self._max_vel, max_accel, self._dt,
                speed_scale=speed_scale, start_vel=start_vel, end_vel=end_vel)
        except TOPPRAUnavailable as e:
            raise PlannerError("TOPPRA_UNAVAILABLE", str(e)) from e
        except TOPPRAInfeasible as e:
            raise PlannerError("TOPPRA_INFEASIBLE", str(e)) from e
        return self._sampler.sample(timed)

    def _plan_move_joints(self, start, args, cid, options):
        """Plan a move_joints command, optionally blending its start into the
        previous move's exit speed and/or its end into the next queued move —
        see module docstring "Blending". Returns (pos, vel, n, v_corner) — v_corner
        is the exit speed this move ends at (0.0 if not blending into a next move);
        the caller applies it to self._last_exit_speed only after confirming the
        batch actually sent, same as self._last_endpoint."""
        profile = normalize_profile_name(options.profile or DEFAULT_PROFILE)
        time_parameterizer = make_time_parameterizer(profile, options.ramp_fraction, options.jerk,
                                                     max_jerk=self._max_jerk)
        path = self._path_builder.joint_line_from_degrees(start, args)
        target = path.target
        # Before any planning, so the error names the target rather than a sample of a path.
        self._check_joint_limits(target, start, f"move_joints id={cid} target")
        self._check_zones(target, start, f"move_joints id={cid} target")
        self._check_self_collision(np.vstack([np.asarray(start, dtype=np.float64)[None, :], np.asarray(target, dtype=np.float64)[None, :]]),
                                   f"move_joints id={cid} target")
        speed_scale = self._speed_scale_for(options)
        blend_pct = self._blend_pct_for(options)
        dist = path.delta

        start_vel = None
        if time_parameterizer.supports_boundary_velocity and self._last_exit_speed is not None:
            norm_dist = np.linalg.norm(dist)
            if norm_dist > 1e-9:
                start_vel = self._last_exit_speed * dist / norm_dist

        end_vel = None
        v_corner = 0.0
        if time_parameterizer.supports_boundary_velocity and blend_pct > 0.0:
            nxt = self._peek_next_command()
            # A dry-run successor is never sent, so there is nothing to carry a corner speed into: this move must
            # end at rest (2026-10-23 review: it was sent with BLEND_CONTINUES and a nonzero exit speed, RTI then
            # stopped it dead at the corner, and the move after the dry run started from that speed).
            if nxt is not None and self._cmd_name(nxt) == "move_joints" and not bool(self._cmd_options(nxt).dry_run):
                nxt_args = self._cmd_args(nxt)
                nxt_profile = normalize_profile_name(self._cmd_options(nxt).profile or DEFAULT_PROFILE)
                if nxt_profile == profile and len(nxt_args) >= self.joint_num:
                    nxt_target = np.deg2rad(np.asarray(nxt_args[:self.joint_num], dtype=np.float64))
                    dist_b = nxt_target - target
                    v_corner = compute_corner_speed(
                        dist, dist_b, self._max_vel, self._max_accel, self._dt,
                        blend_pct / 100.0, speed_scale=speed_scale,
                        profile=time_parameterizer.profile)
                    if v_corner > 1e-9:
                        end_vel = v_corner * dist / np.linalg.norm(dist)

        # `accel` scales the acceleration limit; `duration` then rescales the whole move to
        # land on exactly `t`. Both were accepted and SILENTLY IGNORED before today.
        max_accel = self._max_accel * self._accel_scale_for(options)
        if options.duration is not None:
            probe = time_parameterizer.parameterize(
                path, self._max_vel, max_accel, self._dt, speed_scale=speed_scale,
                start_vel=start_vel, end_vel=end_vel)
            speed_scale = self._scale_for_duration(
                options, probe.num_points * self._dt, speed_scale)

        try:
            timed = time_parameterizer.parameterize(
                path, self._max_vel, max_accel, self._dt,
                speed_scale=speed_scale, start_vel=start_vel, end_vel=end_vel)
        except TOPPRAUnavailable as e:
            raise PlannerError("TOPPRA_UNAVAILABLE", str(e)) from e
        except TOPPRAInfeasible as e:
            # Real TOPPRA failure, not a blend-boundary-velocity issue — must be
            # caught before the generic `except ValueError` below (TOPPRAInfeasible
            # is a ValueError subclass), otherwise it would be misread as "blend
            # infeasible" and silently retried without blend instead of surfacing
            # the actual TOPPRA error.
            raise PlannerError("TOPPRA_INFEASIBLE", str(e)) from e
        except ValueError as e:
            # compute_corner_speed said "safe" but the closed-form trapezoid solve
            # disagreed (should be rare — see trajectory_profiles.py's derivation
            # notes) — fall back to planning this move with no boundary velocity at
            # all rather than propagate a broken profile. Safe but suboptimal, same
            # philosophy as SEND_TIMEOUT/GRIPPER_SETTLE_TIMEOUT elsewhere in this file.
            logger.warning(f"blend infeasible for id={cid}, "
                           f"planning normally instead: {e}")
            timed = time_parameterizer.parameterize(
                path, self._max_vel, max_accel, self._dt,
                speed_scale=speed_scale)
            v_corner = 0.0

        sample = self._assert_commanded_accel(
            self._sampler.sample(timed), self._joint_accel_limit(options),
            "move_joints")
        return sample, v_corner

    def _pose_from_args(self, args):
        return pose_from_xyz_rpy(args, label="Cartesian pose")

    def _ik_diagnostics(self, target_pose, q):
        diagnostic = {}
        try:
            actual = calculate_forward_kinematics(self._robot, q)
            diagnostic["position_error_m"] = float(np.linalg.norm(actual.t - target_pose.t))
            diagnostic["rotation_error_fro"] = float(
                np.linalg.norm(actual.A[:3, :3] - target_pose.A[:3, :3]))
        except Exception as exc:
            diagnostic["fk_warning"] = str(exc)

        try:
            jacobian = calculate_jacobian(self._robot, q)
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            sigma_min = float(np.min(singular_values)) if singular_values.size else 0.0
            sigma_max = float(np.max(singular_values)) if singular_values.size else 0.0
            condition = float("inf") if sigma_min <= 1e-12 else sigma_max / sigma_min
            manipulability = float(np.prod(singular_values)) if singular_values.size else 0.0
            diagnostic.update({
                # JSON-safe: +inf at an exact singularity would emit a non-standard
                # `Infinity` token that strict parsers reject (see ik_config.py).
                "jacobian_condition": json_safe_metric(condition, is_condition=True),
                "jacobian_sigma_min": json_safe_metric(sigma_min),
                "manipulability": json_safe_metric(manipulability),
            })
            # The threshold check keeps the raw inf — only the wire representation is capped.
            if DEFAULT_IK_CONFIG.is_singularity_warning(condition, sigma_min):
                diagnostic["singularity_warning"] = True
                logger.warning(
                    f"IK target near singularity: condition={condition:.1f}, "
                    f"sigma_min={sigma_min:.3e}")
        except Exception as exc:
            diagnostic["singularity_warning"] = str(exc)
        return diagnostic

    def _pose_residual(self, target_pose, q):
        actual = calculate_forward_kinematics(self._robot, q)
        translation_error = float(np.linalg.norm(actual.t - target_pose.t))
        delta = actual.A[:3, :3].T @ target_pose.A[:3, :3]
        c = float((np.trace(delta) - 1.0) * 0.5)
        rotation_error = float(np.arccos(np.clip(c, -1.0, 1.0)))
        return translation_error, rotation_error

    def _solve_endpoint_pose(self, start, args, cid, options):
        if self._robot is None:
            raise NotImplementedError("move_pose needs a robot model for IK")

        solver = normalize_ik_solver_name(options.ik_solver or "pinocchio")

        target_pose = self._pose_from_args(args)
        start = np.asarray(start, dtype=np.float64)[:self.joint_num]
        # Solve through the SAME machinery as every path/blend IK — SeededIKPathSolver
        # with DEFAULT_IK_CONFIG — so move_pose shares one config AND gets the DLS
        # fallback (a bare ik_LM call misses many reachable targets; see solve_pose).
        # FK/Jacobian backend is independent of the engine (fast pinocchio, rtb fallback).
        # solve_pose returns a best-effort q; the checks below validate it and build
        # the granular diagnostic.
        q = SeededIKPathSolver(
            self._robot, self.joint_num, solver=solver, config=DEFAULT_IK_CONFIG,
            kinematics_backend=self._fk_jacobian_backend()
        ).solve_pose(target_pose, start)
        if q is None:
            raise PlannerError("IK_TARGET_UNREACHABLE", "IK failed for move_pose target")

        q = np.asarray(q, dtype=np.float64)[:self.joint_num]
        if q.shape[0] != self.joint_num or not np.all(np.isfinite(q)):
            raise PlannerError("IK_PARTIAL_PATH", "IK returned an invalid joint solution")

        qlim = np.asarray(getattr(self._robot, "qlim", []), dtype=np.float64)
        if qlim.shape == (2, self.joint_num):
            low, high = qlim
        elif qlim.shape == (self.joint_num, 2):
            low, high = qlim[:, 0], qlim[:, 1]
        else:
            low = high = None
        if low is not None and (np.any(q < low - 1e-6) or np.any(q > high + 1e-6)):
            raise PlannerError("IK_MARGIN_VIOLATION", "IK solution violates robot joint limits")

        delta = np.abs(q - start)
        start_delta_max = float(np.max(delta)) if delta.size else 0.0
        start_delta_norm = float(np.linalg.norm(delta))
        translation_error, rotation_error = self._pose_residual(target_pose, q)
        translation_tolerance = 2e-4
        rotation_tolerance = np.deg2rad(0.5)
        if translation_error > translation_tolerance or rotation_error > rotation_tolerance:
            raise PlannerError(
                "IK_POSE_ERROR",
                f"IK FK residual exceeds tolerance: translation={translation_error:.6f} m, "
                f"rotation={rotation_error:.6f} rad")

        self._last_diagnostic = {
            "command": "move_pose",
            "dry_run": bool(options.dry_run),
            "success": True,
            "hardware_enabled": not bool(options.dry_run) and not self._offline_output,
            "ik_solver": solver,
            "target_pose": [float(v) for v in args[:6]],
            "target_joints_deg": np.rad2deg(q).tolist(),
            "start_delta_max_rad": start_delta_max,
            "start_delta_norm_rad": start_delta_norm,
            "max_translation_error_m": translation_error,
            "max_rotation_error_rad": rotation_error,
            "pose_translation_tolerance_m": translation_tolerance,
            "pose_rotation_tolerance_rad": rotation_tolerance,
        }
        self._last_diagnostic.update(self._ik_diagnostics(target_pose, q))
        if self._last_diagnostic.get("singularity_warning") is True:
            log_activity(
                _activity_logger, "EXEC", "warning", "IK_SINGULARITY_WARNING", id=cid,
                command=f"move_pose({', '.join(map(str, args))})" if args else "move_pose",
                message=(f"condition={self._last_diagnostic.get('jacobian_condition'):.1f}, "
                         f"sigma_min={self._last_diagnostic.get('jacobian_sigma_min'):.3e}"))
        logger.info(f"move_pose id={cid} IK via {solver} -> "
                    f"{np.round(np.rad2deg(q), 2).tolist()} deg")
        return q

    def _plan_move_pose(self, start, args, cid, options):
        # A dry run is planned in full and stops at _plan_and_send's dry-run gate, after the joint-limit, zone and
        # self-collision checks (2026-10-23 review: it returned here, after the IK, so it skipped them).
        target = self._solve_endpoint_pose(start, args, cid, options)
        path = self._path_builder.joint_line_from_radians(start, target)
        # The read-only acceleration backstop every other move has (2026-10-23 review: move_pose had none).
        return self._assert_commanded_accel(
            self._sample_joint_path(path, cid, options), self._joint_accel_limit(options), "move_pose")

    def _move_linear_sample_count(self, options):
        if options.samples is None:
            return None
        return int(np.clip(int(options.samples), 2, 500))

    def _target_pose_for_linear(self, ctype, start_pose, args):
        delta_or_target = self._pose_from_args(args)
        if ctype == "move_linear":
            return delta_or_target, "absolute"
        if ctype == "move_linear_rel_world":
            target = np.asarray(start_pose.A, dtype=np.float64).copy()
            target[:3, :3] = delta_or_target.A[:3, :3] @ start_pose.A[:3, :3]
            target[:3, 3] = start_pose.t + delta_or_target.t
            return SE3(target, check=False), "wrf"
        if ctype == "move_linear_rel_tool":
            return start_pose * delta_or_target, "trf"
        raise ValueError(f"unknown Cartesian linear command '{ctype}'")

    def _solve_move_linear_path(self, ctype, start, args, cid, options, dry_run=False):
        if self._robot is None:
            raise NotImplementedError("move_linear needs a robot model for IK")

        solver_name = normalize_ik_solver_name(options.ik_solver or "pinocchio")
        # FK/Jacobian backend is independent of the IK engine (fast pinocchio, rtb
        # fallback) — set it for the pre-solve FK below and pass it to the solver.
        backend = self._fk_jacobian_backend()
        set_kinematics_backend(backend)

        start_pose = calculate_forward_kinematics(self._robot, start)
        target_pose, target_frame = self._target_pose_for_linear(ctype, start_pose, args)
        cart_path = cartesian_line(
            start_pose, target_pose, samples=self._move_linear_sample_count(options))

        solver = SeededIKPathSolver(
            self._robot, self.joint_num, solver=solver_name, config=DEFAULT_IK_CONFIG,
            kinematics_backend=backend)
        result = solver.solve(cart_path, start)
        # Kept so the retimer can re-solve IK at the EXACT poses the timing lands on. The
        # planner is single-threaded and consumes these immediately in _plan_move_linear.
        self._last_cart_path = cart_path
        self._last_ik_solver = solver

        diagnostic = {
            "command": ctype,
            "target_frame": target_frame,
            "dry_run": bool(dry_run),
            "hardware_enabled": not bool(dry_run) and not self._offline_output,
            "ik_solver": solver_name,
            "target_pose": [float(v) for v in args[:6]],
            "translation_length_m": cart_path.translation_length,
            "rotation_angle_rad": cart_path.rotation_angle,
        }
        diagnostic.update(result.diagnostic())
        if result.positions.size:
            diagnostic["target_joints_deg"] = np.rad2deg(result.positions[-1]).tolist()
        if DEFAULT_IK_CONFIG.is_singularity_warning(
                diagnostic.get("max_jacobian_condition"), diagnostic.get("min_jacobian_sigma")):
            diagnostic["singularity_warning"] = True
        self._last_diagnostic = diagnostic
        if diagnostic.get("singularity_warning") is True:
            log_activity(
                _activity_logger, "EXEC", "warning", "IK_SINGULARITY_WARNING", id=cid,
                command=f"{ctype}({', '.join(map(str, args))})" if args else ctype,
                message=(f"condition={diagnostic.get('max_jacobian_condition')}, "
                         f"sigma_min={diagnostic.get('min_jacobian_sigma')}"))

        if not result.success:
            raise PlannerError(
                result.failure_code or "IK_PARTIAL_PATH",
                f"{result.failure_message} at sample {result.first_failure_index}")

        logger.info(
            f"{ctype} id={cid} solved {cart_path.num_samples} "
            f"IK samples via {solver_name} (dry_run={bool(dry_run)})")
        return result

    def _cartesian_speed_scale_for(self, options):
        return min(self._speed_scale_for(options), self.CARTESIAN_MOVE_MAX_SPEED_SCALE)

    def _filter_duplicate_waypoints(self, waypoints):
        waypoints = np.asarray(waypoints, dtype=np.float64)
        if waypoints.ndim != 2 or waypoints.shape[1] != self.joint_num:
            raise PlannerError("IK_PARTIAL_PATH", f"invalid IK waypoint shape {waypoints.shape}")
        keep = [0]
        for idx in range(1, waypoints.shape[0]):
            if np.linalg.norm(waypoints[idx] - waypoints[keep[-1]]) > 1e-9:
                keep.append(idx)
        if keep[-1] != waypoints.shape[0] - 1:
            keep.append(waypoints.shape[0] - 1)
        return waypoints[keep]

    def _assert_sampled_joint_limits(self, positions):
        """Joint POSITION limits only. Acceleration is checked separately — see
        _assert_commanded_accel. (This function used to be described elsewhere in the file as
        the acceleration backstop. It never was. That comment is now corrected.)"""
        qlim = np.asarray(getattr(self._robot, "qlim", []), dtype=np.float64)
        if qlim.shape == (2, self.joint_num):
            low, high = qlim
        elif qlim.shape == (self.joint_num, 2):
            low, high = qlim[:, 0], qlim[:, 1]
        else:
            return
        if np.any(positions < low[None, :] - 1e-6) or np.any(positions > high[None, :] + 1e-6):
            raise PlannerError("IK_MARGIN_VIOLATION", "retimed Cartesian path violates joint limits")

    # ---------------------------------------------------------------------------------- #
    # Acceleration backstop — the guard that was missing (added 2026-07-13)
    #
    # WHAT IT IS: a READ-ONLY gate on a finished trajectory, immediately before it is sent.
    # It never modifies, clamps or rescales anything. Two outcomes only — the trajectory
    # passes and is sent BYTE-FOR-BYTE UNCHANGED, or it is rejected and NOTHING is sent.
    # Commanded/actual values are therefore identical to what they would be without it.
    #
    # WHY: the planner CAN emit a trajectory that violates the limits it was handed, and it
    # HAS. TOPPRA enforces its constraints only AT ITS GRIDPOINTS, and the spline it returns
    # BULGED between them — the samples actually sent to the motors ran at 8.6x the
    # acceleration limit. Nothing caught it; a human found it by measuring commanded
    # acceleration by hand. Before this, NOTHING anywhere checked acceleration: not the
    # planner (it only checked joint POSITION limits) and not RTI (which executes whatever
    # batch it is handed, with no clamp).
    #
    # THE TOLERANCE: we recover acceleration by differencing 4ms VELOCITY samples, and that
    # finite difference reads slightly HIGH against the continuous profile it came from. This
    # is a measurement artifact, not real over-acceleration. MEASURED worst case across every
    # lane and profile (2026-07-13): ruckig 100.0%, s_curve 100.2%, trapezoid/quintic 100.6%,
    # TOPPRA 101.9%, Cartesian 99.8% over 420 fuzzed moves. So 1.15 sits ~7x above the worst
    # real overshoot (no false rejections) while still catching a genuine blowout by a mile
    # (the TOPPRA bug was 860%). Do NOT tighten this toward 1.0 — you will start rejecting
    # legitimate TOPPRA moves. Do NOT loosen it past ~1.3 without re-measuring.
    ACCEL_BACKSTOP_TOLERANCE = 1.15

    # WHICH LIMIT THIS GUARDS — read before "improving" it.
    #
    # It checks against the HARDWARE acceleration limit (`self._max_accel`), NOT the scaled,
    # per-command limit implied by `speed`/`accel`. That is deliberate, and I got it wrong the
    # first time: a per-command check FALSE-REJECTED real moves, and the existing test suite
    # caught it. The cause, measured (2026-07-13):
    #
    #   `duration` OVERRIDES BOTH `speed` AND `accel`. The planner rescales the trajectory to
    #   land on `t`, and time-rescaling scales acceleration by 1/scale^2 — so a move that also
    #   carries `accel=30` can end up at 116% (trapezoid) / 126% (quintic) / 125% (ruckig) /
    #   164% (s_curve) of the acceleration that `accel=30` implies. The request is not violated
    #   so much as SUPERSEDED: you asked for an exact duration, and that is what you got. It
    #   stays inside the HARDWARE limit throughout (verified over 330 random plans).
    #
    # For the record, because a stale note in currently_doing.md claimed otherwise and I
    # repeated it: `accel` on its own is honoured EXACTLY — 100.0% +/- 0.2% of what you ask,
    # on all five profiles, at every value tried. There is no "ask for 25%, get 30%" effect.
    #
    # Re-deriving "the limit that was really in force" at each call site therefore means
    # reproducing the scaling logic of five profiles x three lanes x the duration override.
    # Any drift between that copy and the real thing REJECTS LEGITIMATE MOTION — the arm stops
    # for no reason. A guard that can false-reject is worse than no guard.
    #
    # The hardware limit has none of those problems: it is one number, it never moves, and every
    # lane's in-force limit is <= it by construction, so a legitimate trajectory can never trip
    # it. What it catches is the thing that actually hurts: a trajectory commanding more
    # acceleration than the joint can physically deliver. The TOPPRA gridpoint bug (17.3 rad/s^2
    # against joints whose hardware limits are 3.0-15.4) is caught by a mile.
    #
    # WHAT IT DOES NOT CATCH, stated plainly: "I asked for accel=25% and got 30%". That is a
    # fidelity issue, not a hazard, and it is already documented behaviour.
    def _accel_backstop_limit(self, _options=None):
        """The hardware acceleration limit — see the note above for why not the scaled one."""
        return self._max_accel

    # Kept as aliases so every call site reads as the lane it guards.
    _joint_accel_limit = _accel_backstop_limit
    _cartesian_accel_limit = _accel_backstop_limit

    def _assert_commanded_accel(self, sample, accel_limit, where):
        """Refuse to SEND a trajectory whose commanded acceleration exceeds what it was
        planned against. Read-only: returns `sample` untouched, or raises."""
        vel = np.asarray(sample.velocities, dtype=np.float64)
        if vel.shape[0] < 3:
            return sample
        acc = np.diff(vel, axis=0) / self._dt
        lim = np.asarray(accel_limit, dtype=np.float64) * self.ACCEL_BACKSTOP_TOLERANCE
        ratio = np.abs(acc) / np.maximum(lim[None, :], 1e-12)
        worst = float(ratio.max())
        if worst > 1.0:
            j = int(np.unravel_index(int(np.argmax(ratio)), ratio.shape)[1])
            peak = float(np.abs(acc[:, j]).max())
            raise PlannerError(
                "ACCEL_LIMIT_VIOLATION",
                f"{where}: commanded acceleration exceeds the limit it was planned against — "
                f"J{j} peaks at {peak:.3f} rad/s^2 against a {float(accel_limit[j]):.3f} rad/s^2 "
                f"limit ({peak / max(float(accel_limit[j]), 1e-12):.2f}x, backstop trips above "
                f"{self.ACCEL_BACKSTOP_TOLERANCE:.2f}x). Nothing was sent.")
        return sample

    def _utilization_metrics(self, vel, acc=None):
        """Per-joint velocity/acceleration utilization over a timed trajectory — the
        fraction of each joint's hardware limit actually used (1.0 = at the limit) —
        plus which joint is the bottleneck. Groundwork for the variable-velocity
        singularity policy (TODO/singularity.md Phase 2): near a wrist singularity a
        small TCP move demands large J4/J6 motion, so utilization spikes there."""
        out = {}
        if vel is not None and len(vel):
            vu = np.max(np.abs(vel) / (self._max_vel[None, :] + 1e-12), axis=0)
            out["velocity_utilization"] = float(np.max(vu))
            out["velocity_utilization_per_joint"] = [float(x) for x in vu]
            out["velocity_limiting_joint"] = int(np.argmax(vu))
        if acc is not None and len(acc):
            au = np.max(np.abs(acc) / (self._max_accel[None, :] + 1e-12), axis=0)
            out["acceleration_utilization"] = float(np.max(au))
            out["acceleration_utilization_per_joint"] = [float(x) for x in au]
            out["acceleration_limiting_joint"] = int(np.argmax(au))
        return out

    def _ik_every_sample(self, cart_path, s_t, total, pos_spline, vel_spline):
        """Solve IK at EVERY commanded sample instead of splining between coarse IK'd points.

        WHY. The spline is accurate (26 microns at production density) but only BECAUSE q(s) is
        smooth while the arm is well conditioned. Near a singularity q(s) swings too fast for a
        cubic to track, and the tool would leave the line while joint limits, the endpoint and
        finiteness ALL still passed. That was resting on the singularity guard rejecting such
        paths first — a fortunate interaction, not a design. Per-sample IK removes the
        interpolation entirely: the path is FOLLOWED, not fitted.

        AND IT IS AFFORDABLE — I first got this wrong. Seeded IK costs ~0.5ms/sample, so a 2s
        move needs ~312ms. But that is **16% of the motion's own duration**: IK runs 6x FASTER
        THAN REAL TIME. Calling it a blocking 710ms was the error. With chunked sending, the
        first chunk goes out and the rest is solved while RTI plays it.

        THE TRAP — NEVER FINITE-DIFFERENCE THE IK OUTPUT FOR VELOCITY. Each solve converges
        independently to ~1e-5, so consecutive solutions carry UNCORRELATED solver noise.
        Differentiating at 4ms amplifies it 250x for velocity, 62,500x for acceleration — the
        same effect that made the Cartesian splice seam oscillate. So:

            position  <- per-sample IK        (exact)
            velocity  <- J(q)^-1 . twist      (exact; the twist is from the ANALYTIC path)

        Falls back to the spline result on any failure: a slightly interpolated path beats no
        path, and _assert_cartesian_path_fidelity still guards it.
        """
        solver = getattr(self, "_last_ik_solver", None)
        if solver is None or total < 1e-9:
            return None
        poses, path_s = cart_path.poses, np.asarray(cart_path.s, dtype=np.float64)
        n = pos_spline.shape[0]
        if len(poses) < 2 or n < 3:
            return None

        # Our schedule is in TOOL arclength; the Cartesian path has its own parameter.
        u_t = path_s[0] + np.clip(s_t / total, 0.0, 1.0) * (path_s[-1] - path_s[0])

        # Re-solve the whole trajectory as a Cartesian path of the EXACT poses the timing
        # landed on. Reusing solver.solve() rather than poking at internals means this
        # inherits the seeding, branch-continuity, joint-limit and singularity checks.
        # Interpolate WITHIN the pose list: find the bracketing segment for each u and
        # lerp/slerp inside it. (interp_pose takes TWO poses and a fraction, not a list.)
        idx = np.clip(np.searchsorted(path_s, u_t, side="right") - 1, 0, len(poses) - 2)
        span = np.where(path_s[idx + 1] - path_s[idx] > 1e-12,
                        path_s[idx + 1] - path_s[idx], 1.0)
        frac = np.clip((u_t - path_s[idx]) / span, 0.0, 1.0)
        sampled_poses = tuple(
            interp_pose(poses[int(i)], poses[int(i) + 1], float(f))
            for i, f in zip(idx, frac))
        dense = CartesianPath(poses=sampled_poses, s=u_t,
                              translation_length=cart_path.translation_length,
                              rotation_angle=cart_path.rotation_angle)
        try:
            result = solver.solve(dense, pos_spline[0])
        except Exception as exc:
            logger.warning(f"per-sample IK failed ({exc}); keeping the splined path")
            return None
        pos = np.asarray(result.positions, dtype=np.float64)
        if pos.shape != pos_spline.shape:
            return None

        # Velocity from the Jacobian and the path's own analytic twist — never by
        # differentiating the IK output.
        vel = np.zeros_like(vel_spline)
        for i in range(1, n - 1):
            p0 = sampled_poses[i - 1]
            p1 = sampled_poses[i + 1]
            two_dt = 2.0 * self._dt
            v_lin = (p1.t - p0.t) / two_dt
            rel = p0.R.T @ p1.R
            ang = 0.5 * np.array([rel[2, 1] - rel[1, 2],
                                  rel[0, 2] - rel[2, 0],
                                  rel[1, 0] - rel[0, 1]])
            w = (p0.R @ ang) / two_dt
            J = calculate_jacobian(self._robot, pos[i])
            vel[i] = np.linalg.lstsq(J, np.concatenate([v_lin, w]), rcond=None)[0]
        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(vel)):
            return None
        self._last_diagnostic["ik_per_sample"] = int(n)
        return pos, vel

    def _assert_cartesian_path_fidelity(self, sampled, cart_path):
        """Every commanded sample must lie ON the programmed Cartesian path.

        WHY. We do not interpolate blindly: with `cart_path` in hand, _ik_every_sample solves
        IK at every commanded sample, so the path is FOLLOWED rather than fitted. But that pass
        falls back to the spline if any solve fails — and the spline only tracks q(s) while the
        arm is well conditioned. Near a singularity q(s) swings too fast for a cubic, and the
        tool leaves the line while joint limits, the endpoint and finiteness ALL still pass.
        That used to rest on the singularity guard rejecting such paths first — a fortunate
        interaction, not a design. This makes it a check.

        ONLY MEANINGFUL WHEN WE KNOW THE PATH. Two earlier versions of this guard got that
        wrong and fired on healthy motion:
          - v1 compared against a straight line through the endpoints. True for move_linear,
            FALSE in general — this function also takes arbitrary joint waypoint sets, and it
            rejected a path that was never meant to be straight (5.9mm "error").
          - v2 compared against the polyline through the IK'd waypoints. Also wrong: a smooth
            interpolation legitimately bows off the CHORDS between sparse waypoints (1.3mm).
        There is no ground truth to check against unless the caller hands us the actual
        Cartesian path. So when `cart_path` is None we do not guess — we skip.
        """
        if cart_path is None or sampled.shape[0] < 3:
            return
        poses = cart_path.poses
        if len(poses) < 2:
            return
        ref = np.array([P.t for P in poses], dtype=np.float64)
        xyz = np.array([calculate_forward_kinematics(self._robot, q).t for q in sampled],
                       dtype=np.float64)
        a = ref[:-1]
        seg = ref[1:] - a
        seg_len2 = np.einsum("ij,ij->i", seg, seg)
        seg_len2 = np.where(seg_len2 > 1e-18, seg_len2, 1.0)
        rel = xyz[:, None, :] - a[None, :, :]
        t = np.clip(np.einsum("ijk,jk->ij", rel, seg) / seg_len2[None, :], 0.0, 1.0)
        closest = a[None, :, :] + t[:, :, None] * seg[None, :, :]
        dev = np.linalg.norm(xyz[:, None, :] - closest, axis=2).min(axis=1)
        worst = float(dev.max())
        tol = float(DEFAULT_IK_CONFIG.pose_translation_tolerance_m)
        self._last_diagnostic["cartesian_path_deviation_m"] = worst
        if worst > tol:
            raise PlannerError(
                "CARTESIAN_PATH_DEVIATION",
                f"the commanded tool path strays {worst * 1000:.3f}mm from the programmed "
                f"Cartesian path (tolerance {tol * 1000:.3f}mm). Per-sample IK must have "
                f"fallen back to interpolation, and the interpolation is not tracking q(s) — "
                f"usually a near-singular stretch. Raise `samples`, or route around it.")

    def _retime_cartesian_on_arclength(self, waypoints, options, profile, speed_scale,
                                       cart_s=None, cart_path=None):
        """Run a scalar profile (quintic / s_curve / ruckig) along the path's ARC LENGTH.

        The path geometry is preserved EXACTLY — we only choose when we are where. q(s) is a
        spline through the IK'd joint waypoints; the profile times a 1-DOF move from s=0 to
        s=L; then q(t) = q(s(t)).

        The sdot/sddot caps are DELIBERATELY CONSERVATIVE and known to be incomplete: they
        come from the worst |dq/ds| anywhere on the path, and they IGNORE the centripetal
        term (qdd = q'*sddot + q''*sdot^2). That is exactly the 64,000x-varying constraint
        that makes a scalar profile the wrong tool for a BLENDED path — but a Cartesian LINE
        has no corners, so |dq/ds| is well behaved and the approximation holds. The stretch
        loop below is what actually enforces the joint limits, and _assert_sampled_joint_limits
        is the backstop. If a future caller sends this a path WITH corners, the stretch loop
        will simply slow it to a crawl rather than violate anything — loudly slow, not unsafe.
        """
        # Parameterize by the distance the TOOL travels, not the distance the JOINTS travel.
        # Timing against joint arc length lets the Jacobian distort the tool's speed profile:
        # a "trapezoid" measured a 29.5% ripple across its cruise phase instead of holding
        # flat. Fall back to joint arc length only if the caller could not supply a Cartesian
        # one (then the profile is still valid, just not a TOOL-space profile).
        if cart_s is not None and len(cart_s) == waypoints.shape[0] and cart_s[-1] > 1e-9:
            ss = np.asarray(cart_s, dtype=np.float64)
        else:
            seg = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
            ss = np.concatenate(([0.0], np.cumsum(seg)))
        # Strictly increasing knots, or CubicSpline raises.
        ss = np.maximum.accumulate(ss)
        keep = np.concatenate(([True], np.diff(ss) > 1e-12))
        ss, waypoints = ss[keep], waypoints[keep]
        total = float(ss[-1])
        if total < 1e-9:
            pos = np.repeat(waypoints[:1], 2, axis=0)
            return self._sampler.sample(TimedJointPath(pos, np.zeros_like(pos), 2))
        q_of_s = CubicSpline(ss, waypoints, axis=0)

        # sdot/sddot caps from the steepest |dq/ds| anywhere on the path.
        s_probe = np.linspace(0.0, total, max(64, waypoints.shape[0]))
        dq_ds = np.abs(np.asarray(q_of_s(s_probe, 1), dtype=np.float64))
        worst = np.maximum(dq_ds.max(axis=0), 1e-9)
        # speed_scale governs SPEED. Acceleration keeps the envelope the shipping Cartesian
        # retimer has always used — the RAW limit, enforced by the stretch loop below and
        # backstopped by _assert_commanded_accel (the acceleration gate; _assert_sampled_joint_limits
        # only ever checked joint POSITION limits, despite what this comment used to claim). Scaling it by speed_scale^2 (the
        # convention TOPPRA uses in JOINT space) would cut the Cartesian accel budget 25x and
        # make every move crawl: measured trapezoid peak accel 5.02 rad/s^2 under the shipping
        # rule vs 0.14 under the scaled one. Same envelope as today, just parameterized right.
        vlim = self._max_vel * speed_scale
        alim = self._max_accel * self._accel_scale_for(options)
        sdot_max = float(np.min(vlim / worst))
        sddot_max = float(np.min(alim / worst))

        parameterizer = make_time_parameterizer(profile, options.ramp_fraction,
                                                options.jerk, max_jerk=self._max_jerk)
        if options.duration is not None:
            probe = parameterizer.parameterize(
                JointPath(start=np.zeros(1), target=np.array([total])),
                np.array([sdot_max]), np.array([sddot_max]), self._dt, speed_scale=1.0)
            d_scale = self._scale_for_duration(options, probe.num_points * self._dt, 1.0)
            # Ruckig derives its jerk limit from the acceleration limit we hand it, but jerk
            # scales with scale^3 while acceleration scales with scale^2 — so scaling only
            # sddot leaves the jerk budget one factor of `scale` too generous and the move
            # finishes EARLY (measured 5.54s against a requested 6.00s). Pass the parameterizer
            # a pre-scaled jerk factor so all three scale consistently.
            sdot_max *= d_scale
            sddot_max *= d_scale * d_scale
            if isinstance(parameterizer, RuckigTimeParameterizer):
                parameterizer = RuckigTimeParameterizer(
                    jerk_factor=parameterizer.jerk_factor * d_scale)
        stretch = 1.0
        pos = vel = acc = None
        for _ in range(8):
            timed = parameterizer.parameterize(
                JointPath(start=np.zeros(1), target=np.array([total])),
                np.array([sdot_max / stretch]), np.array([sddot_max / (stretch ** 2)]),
                self._dt, speed_scale=1.0)
            s_t = np.clip(timed.positions[:, 0], 0.0, total)
            sdot_t = timed.velocities[:, 0]
            pos = np.asarray(q_of_s(s_t), dtype=np.float64)
            vel = np.asarray(q_of_s(s_t, 1), dtype=np.float64) * sdot_t[:, None]
            acc = np.diff(vel, axis=0, prepend=vel[:1]) / self._dt

            vel_ratio = float(np.max(np.abs(vel) / (self._max_vel[None, :] + 1e-12)))
            acc_ratio = float(np.max(np.abs(acc) / (self._max_accel[None, :] + 1e-12)))
            ratio = max(vel_ratio, np.sqrt(acc_ratio))
            if ratio <= 1.0:
                break
            stretch *= ratio * 1.10

        # Land the endpoint exactly — s(t) is clipped to L, but float error in the spline
        # evaluation would otherwise leave a small standing offset at the target.
        pos[-1] = waypoints[-1]
        vel[-1] = 0.0

        # A dry run refines too: it must check the trajectory the real run would send (2026-10-23 review).
        if cart_path is not None:
            refined = self._ik_every_sample(cart_path, s_t, total, pos, vel)
            if refined is not None:
                pos, vel = refined
                acc = np.diff(vel, axis=0, prepend=vel[:1]) / self._dt

        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(vel)):
            raise PlannerError("IK_PARTIAL_PATH", "retimed Cartesian trajectory is not finite")
        self._assert_sampled_joint_limits(pos)
        self._assert_cartesian_path_fidelity(pos, cart_path)
        self._last_diagnostic.update({
            "cartesian_speed_scale": float(speed_scale),
            "cartesian_profile": profile,
            "trajectory_points": int(pos.shape[0]),
            "trajectory_duration_s": float(pos.shape[0] * self._dt),
            "retime_stretch": float(stretch),
            **self._utilization_metrics(vel, acc),
        })
        return self._assert_commanded_accel(
            self._sampler.sample(TimedJointPath(pos, vel, pos.shape[0])),
            self._cartesian_accel_limit(options), "move_linear (arclength)")

    def _retime_cartesian_joint_path(self, waypoints, options, cart_s=None, cart_path=None):
        """Time an IK-solved Cartesian joint path, honouring `profile`.

        THIS USED TO IGNORE `profile` ENTIRELY. Every move_linear ran one hardcoded retimer
        (velocity-proportional knot times + a CubicSpline + a stretch loop), so
        `move_linear(..., profile="ruckig")` accepted the option and quietly did something
        else. Same silent-drop family as blend_continuity: the API lied.

        THE SUBTLETY — why we cannot just call parameterize() like move_joints does. A
        Cartesian move is a PATH of many IK'd joint waypoints. trapezoid/quintic/s_curve/
        ruckig are POINT-TO-POINT generators: they read only start and target and would drive
        straight from one to the other IN JOINT SPACE, throwing the path away — and the tool
        would not travel in a straight line at all. They are not path followers.

        So the scalar profiles are applied to the PATH PARAMETER instead of to the endpoints:

            s(t) <- profile timed along the path's ARC LENGTH (a 1-DOF problem)
            q(t)  = spline(s(t))            <- the geometry is preserved EXACTLY
            qd(t) = spline'(s(t)) * sdot(t)

        The path is untouched; only the schedule along it changes. TOPPRA needs none of this
        — it is a genuine path follower, so it gets the joint waypoints directly and produces
        the time-optimal timing.

        The stretch loop stays in both branches as the safety net: the scalar sdot/sddot caps
        below ignore the CENTRIPETAL term (q'' * sdot^2), so they are necessary but not
        sufficient, and the loop is what actually enforces the joint limits.
        """
        waypoints = self._filter_duplicate_waypoints(waypoints)
        if waypoints.shape[0] < 2:
            pos = np.repeat(waypoints[:1], 2, axis=0)
            return self._sampler.sample(TimedJointPath(pos, np.zeros_like(pos), 2))

        profile = normalize_profile_name(options.profile or DEFAULT_PROFILE)
        speed_scale = self._cartesian_speed_scale_for(options)

        if profile == "toppra":
            # A real path follower. It sees q'(s) and q''(s), so it accounts for centripetal
            # acceleration properly instead of being clamped by a single conservative scalar.
            # `accel` and `duration` were ignored on THIS branch even after they were wired
            # into the scalar one — the branch returns early, so it needs them explicitly.
            max_accel = self._max_accel * self._accel_scale_for(options)
            # The same codes as the joint moves for the same failures (2026-10-23 review: they escaped as PLAN_FAIL).
            try:
                if options.duration is not None:
                    probe = self._toppra_path_parameterizer.parameterize_path(
                        waypoints, self._max_vel, max_accel, self._dt, speed_scale=speed_scale)
                    speed_scale = self._scale_for_duration(
                        options, probe.num_points * self._dt, speed_scale)
                timed = self._toppra_path_parameterizer.parameterize_path(
                    waypoints, self._max_vel, max_accel, self._dt, speed_scale=speed_scale)
            except TOPPRAUnavailable as e:
                raise PlannerError("TOPPRA_UNAVAILABLE", str(e)) from e
            except TOPPRAInfeasible as e:
                raise PlannerError("TOPPRA_INFEASIBLE", str(e)) from e
            pos, vel = timed.positions, timed.velocities

            # Refine to per-sample IK, same as the scalar lanes. TOPPRA hands back q(t)
            # directly rather than an s(t) schedule, so recover the schedule from the output
            # itself: FK each sample, measure how far along the TOOL path it is, and re-solve
            # at the exact pose that arc length corresponds to.
            if cart_path is not None:          # a dry run refines too (2026-10-23 review)
                s_of_t = self._cartesian_arclength(pos)
                refined = self._ik_every_sample(cart_path, s_of_t, float(s_of_t[-1]), pos, vel)
                if refined is not None:
                    pos, vel = refined

            acc = np.diff(vel, axis=0, prepend=vel[:1]) / self._dt
            self._assert_sampled_joint_limits(pos)
            self._assert_cartesian_path_fidelity(pos, cart_path)
            self._last_diagnostic.update({
                "cartesian_speed_scale": float(speed_scale),
                "cartesian_profile": profile,
                "trajectory_points": int(pos.shape[0]),
                "trajectory_duration_s": float(pos.shape[0] * self._dt),
                "retime_stretch": 1.0,
                **self._utilization_metrics(vel, acc),
            })
            # Sample the REFINED arrays, not the original `timed` — refining pos/vel and then
            # returning the un-refined object throws the whole per-sample IK pass away, and it
            # does so silently (the diagnostic still says it ran).
            return self._assert_commanded_accel(
                self._sampler.sample(TimedJointPath(pos, vel, pos.shape[0])),
                self._cartesian_accel_limit(options), "move_linear")

        # EVERY scalar profile — trapezoid included — is timed against the TOOL's distance.
        #
        # trapezoid used to run a separate retimer that spaced knots by JOINT displacement and
        # splined through them. That is not a Cartesian trapezoid: the tool's cruise speed
        # rippled 29.5% instead of holding flat, and the spline through 60 velocity-weighted
        # knots WOBBLED, producing peak accelerations of 5.02 rad/s^2 where a clean profile
        # needs a fraction of that. It was the DEFAULT, so it was the one everybody got.
        #
        # A Cartesian move exists to control the TOOL (welding, dispensing, cutting, printing
        # all care about tool speed, not joint speed), so the tool's distance is the only
        # parameter worth timing against.
        return self._retime_cartesian_on_arclength(waypoints, options, profile, speed_scale,
                                                   cart_s=cart_s, cart_path=cart_path)

    def _plan_move_linear(self, ctype, start, args, cid, options):
        # A dry run is planned in full (timing too) and stops at _plan_and_send's dry-run gate, after the joint-limit,
        # zone and self-collision checks (2026-10-23 review: it returned here, after the IK, so it skipped them).
        result = self._solve_move_linear_path(ctype, start, args, cid, options, dry_run=bool(options.dry_run))
        # The profile must be timed against CARTESIAN distance, not joint distance — see
        # _cartesian_arclength.
        cart_s = self._cartesian_arclength(result.positions)
        return self._retime_cartesian_joint_path(
            result.positions, options, cart_s=cart_s,
            cart_path=getattr(self, "_last_cart_path", None))
    def _curve_sample_count(self, options):
        if options.samples is None:
            return None
        return int(np.clip(int(options.samples), 2, 3000))

    def _curve_command_frame(self, ctype):
        """Return the base curve command and its argument frame."""
        if "_rel_" not in ctype:
            return ctype, "absolute"
        base, suffix = ctype.rsplit("_rel_", 1)
        # The commands end in _rel_world / _rel_tool (2026-10-23); "wrf" / "trf" stay the planner's own frame tokens.
        frame = {"world": "wrf", "tool": "trf"}.get(suffix)
        if frame is None:
            raise PlannerError("BAD_ARGUMENTS", f"unknown curve frame in '{ctype}'")
        return base, frame

    def _curve_pose_args(self, args, offset, label):
        values = args[offset:offset + 6]
        if len(values) != 6:
            raise PlannerError("BAD_ARGUMENTS", f"{label} needs 6 pose values [x, y, z, rx, ry, rz]")
        return pose_from_xyz_rpy(values, label)

    def _resolve_curve_pose(self, start_pose, pose, frame):
        """Resolve an absolute, WRF-relative, or fixed-start-TRF-relative pose."""
        if frame == "absolute":
            return pose
        if frame == "trf":
            return start_pose * pose
        target = np.asarray(start_pose.A, dtype=np.float64).copy()
        target[:3, :3] = pose.A[:3, :3] @ start_pose.A[:3, :3]
        target[:3, 3] = start_pose.t + pose.t
        return SE3(target, check=False)

    def _build_curve_path(self, ctype, start_pose, args, options):
        samples = self._curve_sample_count(options)
        base, frame = self._curve_command_frame(ctype)
        input_frame = "world" if frame == "absolute" else frame
        if base == "move_arc":
            if len(args) != 12:
                raise PlannerError("BAD_ARGUMENTS", "move_arc needs via and end poses (12 values)")
            via = self._resolve_curve_pose(
                start_pose, self._curve_pose_args(args, 0, f"{ctype} via"), frame)
            end = self._resolve_curve_pose(
                start_pose, self._curve_pose_args(args, 6, f"{ctype} end"), frame)
            return cartesian_arc_3point(start_pose, via, end, samples=samples), {
                "input_frame": input_frame,
                "via_pose": [float(v) for v in args[:6]],
                "end_pose": [float(v) for v in args[6:12]],
            }
        if base == "move_circle":
            if len(args) not in (6, 7):
                raise PlannerError("BAD_ARGUMENTS", "move_circle needs center, normal, and optional turns")
            center = np.asarray(args[:3], dtype=np.float64)
            normal = np.asarray(args[3:6], dtype=np.float64)
            if frame == "wrf":
                center = start_pose.t + center
            elif frame == "trf":
                center = start_pose.t + start_pose.R @ center
                normal = start_pose.R @ normal
            turns = float(args[6]) if len(args) == 7 else 1.0
            return cartesian_circle(start_pose, center, normal, turns=turns, samples=samples), {
                "input_frame": input_frame,
                "center": [float(v) for v in args[:3]],
                "normal": [float(v) for v in args[3:6]],
                "turns": turns,
            }
        if base == "move_spline":
            if len(args) < 12 or len(args) % 6:
                raise PlannerError(
                    "BAD_ARGUMENTS",
                    "move_spline needs at least two waypoint poses (a multiple of 6 values)")
            waypoints = [
                self._resolve_curve_pose(
                    start_pose, self._curve_pose_args(args, i, f"{ctype} waypoint {i // 6}"), frame)
                for i in range(0, len(args), 6)
            ]
            return cartesian_spline(start_pose, waypoints, samples=samples), {
                "input_frame": input_frame,
                "waypoint_count": len(waypoints),
                "end_pose": [float(v) for v in args[-6:]],
            }
        raise PlannerError("BAD_ARGUMENTS", f"unknown Cartesian curve command '{ctype}'")

    def _solve_curve_path(self, ctype, start, args, cid, options, dry_run=False):
        if self._robot is None:
            raise NotImplementedError(f"{ctype} needs a robot model for IK")
        if float(options.blend or 0.0) > 0.0:
            raise PlannerError(
                "CURVE_BLEND_UNSUPPORTED",
                f"{ctype} does not support blend yet; use blend=0 until tangent-aware curve blending exists")

        solver_name = normalize_ik_solver_name(options.ik_solver or "pinocchio")
        backend = self._fk_jacobian_backend()
        set_kinematics_backend(backend)
        start_pose = calculate_forward_kinematics(self._robot, start)
        try:
            cart_path, curve_details = self._build_curve_path(ctype, start_pose, args, options)
        except (TypeError, ValueError) as exc:
            raise PlannerError("CURVE_GEOMETRY_INVALID", str(exc)) from exc

        solver = SeededIKPathSolver(
            self._robot, self.joint_num, solver=solver_name, config=DEFAULT_IK_CONFIG,
            kinematics_backend=backend)
        result = solver.solve(cart_path, start)
        self._last_cart_path = cart_path
        self._last_ik_solver = solver

        diagnostic = {
            "command": ctype,
            "target_frame": curve_details["input_frame"],
            "dry_run": bool(dry_run),
            "hardware_enabled": not bool(dry_run) and not self._offline_output,
            "ik_solver": solver_name,
            "translation_length_m": cart_path.translation_length,
            "rotation_angle_rad": cart_path.rotation_angle,
            "curve": curve_details,
        }
        diagnostic.update(result.diagnostic())
        if result.positions.size:
            diagnostic["target_joints_deg"] = np.rad2deg(result.positions[-1]).tolist()
        if DEFAULT_IK_CONFIG.is_singularity_warning(
                diagnostic.get("max_jacobian_condition"), diagnostic.get("min_jacobian_sigma")):
            diagnostic["singularity_warning"] = True
        self._last_diagnostic = diagnostic
        if diagnostic.get("singularity_warning") is True:
            log_activity(
                _activity_logger, "EXEC", "warning", "IK_SINGULARITY_WARNING", id=cid,
                command=ctype,
                message=(f"condition={diagnostic.get('max_jacobian_condition')}, "
                         f"sigma_min={diagnostic.get('min_jacobian_sigma')}"))
        if not result.success:
            raise PlannerError(
                result.failure_code or "IK_PARTIAL_PATH",
                f"{result.failure_message} at sample {result.first_failure_index}")
        logger.info(
            f"{ctype} id={cid} solved {cart_path.num_samples} IK samples via "
            f"{solver_name} (dry_run={bool(dry_run)})")
        return result

    def _curve_retime_options(self, options, scale):
        """Scale a curve's timing only after its acceleration backstop rejects it.

        Curvature adds a q'' * s_dot^2 term that the scalar path profiles do not model.
        Keeping this retry local to new curve commands preserves every existing motion lane.
        """
        return replace(
            options,
            speed=self._cartesian_speed_scale_for(options) * scale * 100.0,
            accel=self._accel_scale_for(options) * scale * scale * 100.0,
        )

    def _retime_curve_path(self, result, options):
        cart_s = self._cartesian_arclength(result.positions)
        scales = (1.0,) if options.duration is not None else (1.0, 0.7, 0.5, 0.35, 0.25)
        last_error = None
        for attempt, scale in enumerate(scales, start=1):
            timing_options = options if scale == 1.0 else self._curve_retime_options(options, scale)
            try:
                sample = self._retime_cartesian_joint_path(
                    result.positions, timing_options, cart_s=cart_s,
                    cart_path=getattr(self, "_last_cart_path", None))
            except PlannerError as error:
                if error.code != "ACCEL_LIMIT_VIOLATION":
                    raise
                last_error = error
                continue
            self._last_diagnostic.update({
                "curve_retime_scale": float(scale),
                "curve_retime_attempts": int(attempt),
            })
            if scale < 1.0:
                logger.warning(
                    "curve timing slowed to %.0f%% after curvature acceleration check",
                    scale * 100.0)
            return sample
        raise last_error

    def _plan_move_curve(self, ctype, start, args, cid, options):
        # A dry run is planned in full, as for move_linear (2026-10-23 review).
        result = self._solve_curve_path(ctype, start, args, cid, options, dry_run=bool(options.dry_run))
        return self._retime_curve_path(result, options)

    def _cartesian_arclength(self, waypoints):
        """Cumulative distance TRAVELLED BY THE TOOL along an IK'd joint path, in metres.

        This is the parameter a Cartesian profile must be timed against. Timing it against
        JOINT arc length instead — which is what the first version of the profile support did
        — lets the Jacobian distort the tool's speed: equal joint motion is NOT equal tool
        distance, so a "trapezoid" came out with a 29.5% ripple across its cruise phase
        instead of holding flat. The whole reason move_linear exists is controlled TOOL
        motion (welding, dispensing, cutting, printing all care about tool speed, not joint
        speed), so the tool's distance is the only parameter worth timing against.

        Rotation is folded in as an equivalent arc at ROTATION_RADIUS_M, the same convention
        motion/cartesian_paths.py already uses to make a pure-rotation move commensurable
        with a pure-translation one.
        """
        poses = [calculate_forward_kinematics(self._robot, q) for q in waypoints]
        xyz = np.array([T.t for T in poses], dtype=np.float64)
        d_trans = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
        d_rot = np.zeros_like(d_trans)
        for i in range(len(poses) - 1):
            rel = poses[i].R.T @ poses[i + 1].R
            cos = float(np.clip((np.trace(rel) - 1.0) / 2.0, -1.0, 1.0))
            d_rot[i] = np.arccos(cos) * CART_ROTATION_RADIUS_M
        step = np.hypot(d_trans, d_rot)
        return np.concatenate(([0.0], np.cumsum(step)))

    def _report_complete(self, cid):
        try:
            self._status_sock.send_json(
                {"checkpoint_id": cid or "", "status": "complete"}, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.error(f"could not report completion for {cid}")

    def _plan_and_send(self, cmd):
        ctype = self._cmd_name(cmd)
        cid = self._cmd_id(cmd)
        args = self._cmd_args(cmd)
        options = self._cmd_options(cmd)

        if ctype in ("open_gripper", "close_gripper", "move_gripper"):
            self._last_exit_speed = None   # gripper breaks any in-flight blend chain
            self._send_gripper(ctype, args, cid)
            return

        start = self._start_point()
        v_corner = 0.0   # non-move_joints commands never leave a nonzero exit speed

        if ctype == "move_joints":
            sample, v_corner = self._plan_move_joints(start, args, cid, options)

        elif ctype == "move_pose":
            self._last_exit_speed = None
            sample = self._plan_move_pose(start, args, cid, options)
            if sample is None:
                self._report_complete(cid)
                return

        elif ctype == "delay":
            secs = float(options.duration) if options.duration is not None else (float(args[0]) if args else 0.0)
            # The moves' rule for a time (2026-10-23 review: 0 or less, or NaN, became a silent ~8 ms hold; infinity
            # failed as PLAN_FAIL).
            if not (np.isfinite(secs) and secs > 0.0):
                raise PlannerError("INVALID_DURATION", f"a pause needs a time above 0 seconds (got {secs})")
            n = max(2, int(round(secs / self._dt)))
            pos = np.repeat(start[None, :], n, axis=0)
            vel = np.zeros((n, self.joint_num))
            sample = self._sampler.sample(TimedJointPath(pos, vel, n))

        elif ctype in ("move_linear", "move_linear_rel_world", "move_linear_rel_tool"):
            self._last_exit_speed = None
            sample = self._plan_move_linear(ctype, start, args, cid, options)
            if sample is None:
                self._report_complete(cid)
                return

        elif ctype in (
                "move_arc", "move_arc_rel_world", "move_arc_rel_tool",
                "move_circle", "move_circle_rel_world", "move_circle_rel_tool",
                "move_spline", "move_spline_rel_world", "move_spline_rel_tool"):
            self._last_exit_speed = None
            sample = self._plan_move_curve(ctype, start, args, cid, options)
            if sample is None:
                self._report_complete(cid)
                return

        else:
            raise ValueError(f"unknown motion command '{ctype}'")

        # The last gate before RTI, for EVERY command type. move_joints was the one command
        # that could drive a joint into its endstop: nothing checked it anywhere downstream
        # (not ExecHandler, not dispatch, not the drive firmware).
        self._check_joint_limits(sample.positions, start, f"{ctype} id={cid}")
        self._check_zones(sample.positions, start, f"{ctype} id={cid}")
        self._check_self_collision(sample.positions, f"{ctype} id={cid}")
        if bool(options.dry_run):
            # Every move's dry run ends here, planned in full and past every check above; nothing sent, the planner's
            # endpoint untouched (2026-10-23: move_joints ignored dry_run and MOVED the arm; the review the same day
            # found the Cartesian moves returning after their IK, before the checks). The arm is at rest: the move
            # before a dry run is planned to end at rest (the lookahead above skips a dry-run successor), so the next
            # real move starts from rest too.
            logger.info(f"{ctype} dry-run id={cid}; no trajectory sent")
            self._last_exit_speed = None
            self._report_complete(cid)
            return
        flags = BATCH_FLAG_BLEND_CONTINUES if v_corner > 1e-9 else 0
        sent = self._send_chunks(sample.positions, sample.velocities, cid, sample.torque_ff, flags=flags)
        if sent:
            self._last_endpoint = sample.positions[-1].copy()
            # Gated on sent, same as _last_endpoint above: a failed send means we
            # don't actually know the arm's exit state, so don't let the next
            # command inherit a blend assumption that might not hold. delay always
            # ends at rest (v_corner=0 from the default above), which correctly
            # breaks any in-flight blend chain once it takes effect.
            self._last_exit_speed = v_corner if v_corner > 1e-9 else None
            logger.info(f"{ctype} id={cid} → {sample.num_points} waypoints "
                        f"({(sample.num_points - 1) * self._dt:.2f}s)")
        # else: _send_chunks already reported the error; leave _last_endpoint and
        # _last_exit_speed alone so the next command starts from the last state we
        # know RTI actually has.

    def _send_chunks(self, pos, vel, cid, torque_ff=None, flags=0):
        """Send a trajectory in <=MAX_CHUNK_SIZE pieces. Only the final chunk carries
        the checkpoint id — RTI (and thus the client) sees exactly one completion,
        on that last piece, regardless of how many chunks the move took.

        Chunking isn't about compute time here (joint-space math is fast — the
        plan's rationale for Cartesian chunking doesn't apply) — it's about message
        size and time-to-first-motion: a 10-minute move at 100 Hz is 60,000 points
        (~7 MB) as one message; chunked, RTI starts moving on the first ~2000-point
        piece immediately instead of waiting for the whole thing to pack and send.

        Returns True if all chunks sent, False if a send timed out partway (already
        reported via _report_error — whatever chunks DID send are still valid and
        will play out on RTI; the client sees an error instead of a hung wait_idle).
        """
        n = pos.shape[0]
        if torque_ff is None:
            torque_ff = np.zeros((n, self.joint_num), dtype=np.float32)
        return self._send_range(pos, vel, torque_ff, 0, n, cid, flags=flags)

    def _send_range(self, pos, vel, torque_ff, range_start, range_end, cid, flags=0,
                    terminal=True):
        """Send samples [range_start:range_end) of shared pos/vel/torque_ff arrays,
        in <=MAX_CHUNK_SIZE wire pieces. Only the final wire piece (at range_end)
        carries the checkpoint id/is_last_chunk/flags.

        Used both for a single command's own full array (via _send_chunks, the
        whole array is [0:n)) and for one command's slice of a larger blended
        composite array (via _send_blended_chain, TODO/blending_design.md §5.4),
        where flags is typically BATCH_FLAG_BLEND_CONTINUES for every command
        but the chain's last.

        terminal=False sends the range as a NON-final piece of the command: no
        checkpoint id, is_last_chunk=False on every wire piece. That is what the
        rolling buffer sends for a command's body — RTI must play straight on into the
        continuation (or the tail) we send next, rather than completing the command and
        settling. The command's checkpoint fires later, on whichever range does end it.
        """
        for start_idx in range(range_start, range_end, self.MAX_CHUNK_SIZE):
            end_idx = min(start_idx + self.MAX_CHUNK_SIZE, range_end)
            is_last = terminal and (end_idx == range_end)
            chunk_flags = flags if is_last else 0
            batch = pack_batch(pos[start_idx:end_idx], vel[start_idx:end_idx],
                               torque_ff[start_idx:end_idx],
                               checkpoint_id=cid if is_last else "", is_last_chunk=is_last,
                               flags=chunk_flags)
            # Each try under abort_gate, so a Stop's flush (which counts the epoch up and sends the fence under the
            # same lock) can never fall between the epoch check and the send; non-blocking, so the lock is never held
            # while RTI is not draining (a Stop / disable must not wait on it) — retried outside it until the timeout.
            deadline = time.monotonic() + self.SEND_TIMEOUT_MS / 1000.0
            while True:
                with self.abort_gate:
                    if self._cmd_epoch != self._epoch:
                        logger.info(f"id={cid}: not sent — the program was stopped (epoch "
                                    f"{self._cmd_epoch} -> {self._epoch})")
                        return True
                    try:
                        self._traj_sock.send(batch, flags=zmq.NOBLOCK)
                        break
                    except zmq.Again:
                        pass
                if time.monotonic() >= deadline:
                    logger.error(f"send timed out (RTI not draining) — "
                                f"{end_idx}/{range_end} waypoints sent for id={cid}")
                    self._report_error(cid, "SEND_TIMEOUT",
                                       "RTI not consuming trajectory batches")
                    return False
                time.sleep(0.002)
        return True

    def _send_gripper(self, ctype, args, cid):
        """Send gripper_set:<pos>:<speed>:<current>:1 to RTI on the existing command
        socket (RTI already handles this string identically regardless of sender — the
        GUI uses the exact same format). Does NOT checkpoint here — see module
        docstring: completion is tracked asynchronously via gripper_object_detection
        by _poll_gripper_completions(), the same way move_joints's real completion is
        signaled by ExecHandler rather than by the planner blocking."""
        if ctype == "open_gripper":
            position = self.GRIPPER_OPEN_POS
        elif ctype == "close_gripper":
            position = self.GRIPPER_CLOSE_POS
        else:  # move_gripper
            if not args:
                raise ValueError("move_gripper needs [position] (0-255 raw scale, "
                                "1=open .. 254=closed — matches the GUI slider)")
            position = int(args[0])
        speed = int(args[1]) if len(args) > 1 else self.GRIPPER_DEFAULT_SPEED
        current = int(args[2]) if len(args) > 2 else self.GRIPPER_DEFAULT_CURRENT

        # Only one physical target can be in flight (_gripper_fw_state is a single
        # value on RTI, overwritten by each new gripper_set). If a previous command
        # is still pending, its target was just superseded before the gripper ever
        # reached it — report that honestly rather than leave it hanging or falsely
        # claim it completed.
        if self._pending_gripper is not None:
            prev_cid = self._pending_gripper[0]
            self._report_error(prev_cid, "SUPERSEDED",
                               "gripper command superseded by a newer command before it settled")
            self._pending_gripper = None

        rti_msg = f"gripper_set:{position}:{speed}:{current}:1"
        self._cmd_sock.send_string(json.dumps({"msg": rti_msg, "id": cid, "source": "planner"}))
        logger.info(f"{ctype} id={cid} → {rti_msg}")

        if self._read_gripper_status is None:
            # No gripper status feed wired up — can't track real completion, so at
            # least don't hang the caller forever. Fire-and-forget as a fallback.
            logger.warning(f"no read_gripper_status configured — "
                           f"id={cid} checkpoints immediately, not on real completion")
            try:
                self._status_sock.send_json(
                    {"checkpoint_id": cid, "status": "complete"}, flags=zmq.NOBLOCK)
            except zmq.Again:
                logger.error(f"could not report gripper checkpoint for {cid}")
            return

        # (cid, deadline, commanded target, consecutive-nonzero count). The target
        # is kept because two of the three completion criteria need it: distance
        # to it, and which DIRECTION we are travelling.
        self._pending_gripper = [cid, time.monotonic() + self.GRIPPER_SETTLE_TIMEOUT_S,
                                 int(position), 0]

    def _poll_gripper_completions(self):
        """Decide whether the one in-flight gripper command has finished.

        THREE independent criteria, any of which completes it. One is not enough:
        `gripper_object_detection` is a Robotiq-style code (0 moving, 1/2 object
        while opening/closing, 3 at target) whose behaviour on a NEW move — reset
        to 0, or hold the previous value — has never been verified on hardware.
        A stale nonzero would complete every command on its first poll.

          1. POSITION. Within GRIPPER_POSITION_TOLERANCE of the commanded target.
             Independent of the status code entirely, so it works even if the
             code is useless.
          2. DEBOUNCED + DIRECTION-CHECKED code. Nonzero for GRIPPER_DEBOUNCE_TICKS
             consecutive polls (one glitched frame cannot complete a command), AND
             consistent with the direction of travel: on this gripper 1 = fully
             open and 254 = fully closed, so "object while opening" (1) is only
             believable when the target is MORE OPEN than where we are. A stale
             code from the opposite direction is rejected — which is exactly the
             failure the unverified reset behaviour would produce.
          3. TIMEOUT. GRIPPER_SETTLE_TIMEOUT_S with none of the above -> error,
             rather than hanging the queue.

        Called from run() after every processed command and on each idle timeout.
        """
        if self._pending_gripper is None or self._read_gripper_status is None:
            return
        cid, deadline, target, streak = self._pending_gripper
        obj = self._read_gripper_status()
        position = self._read_gripper_position() if self._read_gripper_position else None

        # --- 1. position reached ------------------------------------------------
        if position is not None and abs(position - target) <= self.GRIPPER_POSITION_TOLERANCE:
            # Report the code ONLY if it is consistent with how we got here.
            # Completing on position while the code says something contradictory
            # means the code is stale, and forwarding it would have the client's
            # gripper_result() claim "object detected" on a move that simply
            # travelled to its target. Reaching the target with no believable
            # object is exactly what code 3 means, so say that instead.
            outcome = obj if self._code_matches_direction(obj, target, position) else 3
            self._finish_gripper(cid, outcome,
                                 f"position {position} within "
                                 f"{self.GRIPPER_POSITION_TOLERANCE} of {target}"
                                 + ("" if outcome == obj
                                    else f" (ignored inconsistent object_detection={obj})"))
            return

        # --- 2. debounced, direction-checked status code ------------------------
        streak = streak + 1 if obj != 0 else 0
        self._pending_gripper[3] = streak
        if streak >= self.GRIPPER_DEBOUNCE_TICKS:
            if self._code_matches_direction(obj, target, position):
                self._finish_gripper(cid, obj, f"object_detection={obj} "
                                               f"stable for {streak} polls")
                return
            # Believed-but-inconsistent: almost certainly a stale value from the
            # previous command. Say so once rather than silently completing.
            if streak == self.GRIPPER_DEBOUNCE_TICKS:
                logger.warning(
                    f"gripper id={cid} ignoring object_detection={obj}: "
                    f"inconsistent with travel from {position} to {target} "
                    f"(likely stale from the previous command)")

        # --- 3. timeout ---------------------------------------------------------
        if time.monotonic() >= deadline:
            self._pending_gripper = None
            logger.error(f"gripper id={cid} timed out waiting to settle "
                         f"(object_detection={obj}, position={position}, target={target})")
            self._report_error(cid, "GRIPPER_SETTLE_TIMEOUT",
                               "gripper did not report a settled position in time")

    def _code_matches_direction(self, obj, target, position):
        """Is this object-detection code consistent with the way we are travelling?

        Scale: 1 = fully open, 254 = fully closed, so OPENING decreases position.
          1 = object while opening -> only believable if the target is more open
          2 = object while closing -> only believable if the target is more closed
          3 = reached target, no object -> direction carries no information
        With no position feed there is nothing to cross-check against, so the code
        is taken at face value (the old behaviour).
        """
        if obj == 3:
            return True
        if position is None:
            return True
        if obj == 1:
            return target < position
        if obj == 2:
            return target > position
        return True

    def _finish_gripper(self, cid, obj, reason):
        """Report a completed gripper command, carrying WHY it settled."""
        self._pending_gripper = None
        try:
            # Carry WHAT happened, not just THAT it happened. `obj` is known here
            # and used to be logged and discarded, so a client could tell a
            # gripper had settled but not whether it had actually caught anything
            # (1/2) or closed on nothing (3) — the single thing a pick needs to
            # verify. It rides the checkpoint that already flows.
            self._status_sock.send_json(
                {"checkpoint_id": cid, "status": "complete",
                 "gripper_object": int(obj)}, flags=zmq.NOBLOCK)
            logger.info(f"gripper id={cid} settled — {reason}")
        except zmq.Again:
            logger.error(f"could not report gripper checkpoint for {cid}")

    def _report_error(self, cid, code, message):
        # The abort's STATE CHANGES — the latch, and dropping everything queued behind the
        # failure — happen BEFORE this error leaves. A client that sees the failure can react at
        # once (a script catching MotionFailed, calling clear_errors and queueing its next
        # program). If either were still pending then, it would land on the NEXT program: a late
        # latch re-aborts it, a late drain swallows its first moves. Measured in the test, both.
        # Only the ABORTED *reports* for the dropped commands go out after this error, so a
        # client reads the cause first (on the arm they arrived cause-last, which reads backwards).
        with self.abort_gate:                    # a submission never lands between the latch and the drain
            aborting = self._latch_abort(cid, code, message)
            dropped = self._finish_abort(aborting) if aborting else ()
        try:
            self._status_sock.send_json(
                {"checkpoint_id": cid or "", "status": "error",
                 "code": code, "message": message}, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.error(f"could not report error for {cid} (status backpressure)")
        for cmd in dropped:
            self._report_aborted(cmd, aborting)

    # Failures that do NOT abort the program. Gripper outcomes and the strict-completion settle
    # timeout stay per-command for now (user decision 2026-09-30); ABORTED is the abort itself.
    # SETTLE_TIMEOUT is reported by RTI, never by the planner — listed so the rule is in one place.
    NON_ABORTING_CODES = frozenset({"ABORTED", "GRIPPER_SETTLE_TIMEOUT", "SUPERSEDED", "SETTLE_TIMEOUT"})

    # Joint-limit comparison slack (rad): absorbs float noise, far below anything physical.
    JOINT_LIMIT_TOL = 1e-6

    def _command_failed(self, cid, code, message):
        """For a subclass that reports failures its own way (the dry run): abort unless exempt."""
        with self.abort_gate:
            aborting = self._latch_abort(cid, code, message)
            dropped = self._finish_abort(aborting) if aborting else ()
        for cmd in dropped:
            self._report_aborted(cmd, aborting)

    def _latch_abort(self, cid, code, message):
        """Set the abort latch for a failure that aborts. Returns the latch dict, or None when
        the code is exempt or an abort is already latched (the FIRST failure is the cause)."""
        if code in self.NON_ABORTING_CODES or self._abort is not None:
            return None
        self._abort = {"cid": cid, "code": code, "message": message}
        return self._abort

    def _finish_abort(self, abort):
        """A queued command failed: the program stops here.

        Everything ALREADY sent to RTI is before the failure and finishes normally — a held
        braking tail is committed, not dropped, because RTI is mid-motion and without it would
        run dry at speed. Everything queued BEHIND the failure is answered ABORTED unplanned,
        and new commands are refused until clear_errors (clear_abort()). Industrial arms commonly
        do the same: an error clears the motion queue and commands are ignored until a reset.
        Returns the dropped commands; the CALLER reports them ABORTED, after the failure itself.
        """
        cid, code, message = abort["cid"], abort["code"], abort["message"]
        logger.error(f"motion ABORTED: command {cid} failed ({code}: {message}) — "
                     f"the rest of the program is cancelled until clear_errors")
        log_activity(_activity_logger, "EXEC", "error", "MOTION_ABORTED", id=cid,
                     message=f"{code}: {message}")
        self._last_exit_speed = None
        if code == "SEND_TIMEOUT":
            # RTI is not consuming: part of this move may have reached it, so where the arm
            # stops is unknown, and sending it a held tail would only block again.
            self._held = None
            self._last_endpoint = None
        elif self._held is not None:
            self._commit_held_tail("aborting after a failed command")
        dropped = self._blend_buffer
        self._blend_buffer = []
        try:
            while True:
                dropped.append(self._queue.get_nowait())
        except Empty:
            pass
        return dropped

    def _report_aborted(self, cmd, abort=None):
        a = abort or self._abort or {}
        if a.get("cid"):
            why = f"not executed: command {a['cid']} failed earlier ({a.get('code')})"
        else:
            why = f"not executed: {a.get('message', 'motion was aborted')}"
        self._report_error(self._cmd_id(cmd), "ABORTED", why + " — clear errors to run motion again")

    def set_self_collision(self, model):
        """Install the self-collision model (utility/self_collision.SelfCollisionModel) — or None for no check."""
        self._self_collision = model

    def _check_self_collision(self, positions, label):
        """Raise PlannerError('SELF_COLLISION') if any sample of the path brings two links within the PLANNER margin
        (TODO/self_collision_plan.md). Same gate as _check_joint_limits: the whole path, before anything moves.
        A path that STARTS within the margin may move the parts apart, never closer (the escape rule, 2026-10-07)."""
        model = getattr(self, "_self_collision", None)
        if model is None:
            return
        hit = model.check_path(positions, "planner")
        if hit is None:
            return
        k, a, b, gap, why = hit
        n = int(np.atleast_2d(np.asarray(positions)).shape[0])
        where = f"at sample {k + 1} of {n} ({100.0 * k / max(n - 1, 1):.0f} % of the way)"
        if why == "within":
            # THIS pair's planner margin (per-pair overrides differ; the smallest of all pairs — what this said until
            # 2026-10-13 — made a correct refusal read "4.8 mm apart; the margin is 3 mm")
            own = [float(m) for pf, m in zip(model.pair_frames, model.margins["planner"]) if set(pf) == {a, b}]
            margin = own[0] if own else float(model.margins["planner"].min())
            text = (f"the arm would run into itself — {model.describe(a, b)} would be {1e3 * gap:.1f} mm apart {where}; "
                    f"their margin is {1e3 * margin:.0f} mm")
        elif why == "closer":
            text = (f"the arm starts with {model.describe(a, b)} within the self-collision margin and this move brings "
                    f"them CLOSER ({1e3 * gap:.1f} mm {where}) — from there only a move that takes them apart is allowed")
        else:
            text = (f"the arm starts within the self-collision margin and this move brings ANOTHER pair, "
                    f"{model.describe(a, b)}, within it ({1e3 * gap:.1f} mm {where}) — from there only a move that "
                    f"takes the parts apart is allowed")
        raise PlannerError("SELF_COLLISION", f"{label}: {text}")

    def _check_joint_limits(self, positions, start, label):
        """Raise PlannerError('JOINT_LIMIT') if any sample leaves the software joint limits.

        RECOVERY: a joint that STARTS outside its range (pushed there by hand, or a limit
        tightened in the XML) may stay as far out as it already is, never farther — so a move
        back into range is accepted and nothing can drive it further out. Same rule as RTI's
        _brake_at_limits and JOG.
        """
        if self._joint_lo is None:
            return
        pos = np.atleast_2d(np.asarray(positions, dtype=np.float64))[:, :self.joint_num]
        st = np.asarray(start, dtype=np.float64)[:self.joint_num]
        lo = np.minimum(self._joint_lo, st) - self.JOINT_LIMIT_TOL
        hi = np.maximum(self._joint_hi, st) + self.JOINT_LIMIT_TOL
        over = pos > hi[None, :]
        under = pos < lo[None, :]
        bad = np.flatnonzero(np.any(over | under, axis=0))
        if bad.size == 0:
            return
        j = int(bad[0])
        worst = float(pos[:, j].max() if np.any(over[:, j]) else pos[:, j].min())
        raise PlannerError(
            "JOINT_LIMIT",
            f"{label}: J{j + 1} would reach {np.degrees(worst):.2f} deg, outside its software "
            f"limits [{np.degrees(self._joint_lo[j]):.2f}, {np.degrees(self._joint_hi[j]):.2f}] deg")

    def set_zones(self, zones, tool_spheres):
        """Install the active cell's zones (startup, and command_executor's reload_zones). Validates the
        tool spheres; the zones arrive validated from config/xml_parser.parse_cell_zones()."""
        active = tuple(z for z in zones if z["enabled"])
        self._zone_setup = (active, robot_spheres(tool_spheres))

    def _tcp_poses(self, positions):
        """Flat row-major TCP poses for joint samples — from the planner's OWN model (pinocchio keeps
        one scratch buffer per model; this thread is its only user)."""
        robot = self._robot
        fk = getattr(robot, "pinocchio_fkine", None)
        if fk is not None:
            return [fk(q).ravel().tolist() for q in positions]
        return [np.asarray(robot.fkine(q).A, dtype=np.float64).ravel().tolist() for q in positions]

    def _check_zones(self, positions, start, label):
        """Raise PlannerError('ZONE_LIMIT') if any sample takes a robot sphere into a stop-zone of the
        active cell (with the zone's margin). Same RECOVERY rule as the joint limits: a robot that
        starts in violation may stay as deep as it is, never deeper. Warn-zones are logged, not refused."""
        zones, spheres = self._zone_setup
        if not zones:
            return
        if self._robot is None:
            raise PlannerError("ZONE_CHECK_UNAVAILABLE",
                               f"{label}: safety zones are active but there is no robot model to check them")
        pos = np.atleast_2d(np.asarray(positions, dtype=np.float64))[:, :self.joint_num]
        poses = self._tcp_poses(pos)
        start_pose = self._tcp_poses(np.atleast_2d(np.asarray(start, dtype=np.float64))[:, :self.joint_num])[0]
        hit = first_path_violation(zones, poses, spheres, start_pose, reaction="stop")
        if hit is not None:
            zone, i, pen, k = hit
            what = "the TCP" if k == 0 else f"tool sphere {k}"
            side = "leave allowed" if zone["kind"] == "allowed" else "enter forbidden"
            where = f"{100.0 * i / max(len(poses) - 1, 1):.0f}% of the move" if len(poses) > 1 else "the target"
            raise PlannerError(
                "ZONE_LIMIT",
                f"{label}: {what} would {side} zone '{zone['name']}' by {1000.0 * pen:.1f} mm "
                f"(margin {1000.0 * zone['margin']:.0f} mm included) at {where}")
        warn = first_path_violation(zones, poses, spheres, start_pose, reaction="warn")
        if warn is not None:
            zone = warn[0]
            logger.warning(f"{label}: passes through warn-zone '{zone['name']}' "
                           f"({1000.0 * warn[2]:.1f} mm) — allowed, reaction=warn")
            log_activity(_activity_logger, "EXEC", "warning", ZONE_WARNING,
                         message=f"{label} passes through zone '{zone['name']}'")
