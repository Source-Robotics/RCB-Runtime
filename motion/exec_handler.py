"""EXEC mode handler (RTI side) — consumes trajectory batches, feeds the controller.

Runs inside the RTI 1 kHz loop via the mode-dispatch pipeline. It does NOT generate
trajectories (that's the planner thread in command_executor) and it does NOT touch CAN.
Like every handler, ``step()`` returns ``(TickResult, ControlOutput)`` and
``utility/mode_dispatch.py::dispatch_step`` owns commit → FK → motor-space → CAN send.
JogHandler (robotics/jog.py) is the reference pattern.

Data flow:
  planner (command_executor)  --ZMQ PUSH-->  ZMQ_EXEC_TRAJECTORY_SOCKET (we PULL/bind)
  us, on last-chunk done      --ZMQ PUSH-->  ZMQ_EXEC_STATUS_SOCKET (executor PULL/binds)

Per tick:
  1. Flush any pending checkpoint statuses (non-blocking).
  2. If paused: hold in place (batch/index untouched) — see pause()/resume() below.
  3. Pre-fetch the next batch before the current one runs out (double-buffer, no gap).
  4. Swap to the pre-fetched batch when the current is exhausted; publish checkpoint.
  5. Read this tick's waypoint → ControlOutput(pos, vel, torque_ff + gravity).
  6. If no batch available: hold last position (Speed=0) with gravity feedforward.

pause()/resume() (called from RTI.py's command dispatch on "exec_pause"/"exec_resume",
same single-threaded pattern as JogHandler.set_jog/stop_jog — no locking needed): pause()
freezes consumption without touching _current/_index/_next, so the in-flight batch and
position within it are preserved; resume() continues advancing from exactly there. This
differs from having no batch available (which has nothing to resume into) — pause holds a
*specific* waypoint, ready to continue, not just "idle until data arrives."

RT rules (see CLAUDE.md): all numpy buffers and the ControlOutput are pre-allocated in
__init__; step() only writes into them. ZMQ receives/sends are non-blocking — the RT
tick never blocks on IPC.
"""

import logging

from collections import deque

import numpy as np
import zmq

from utility.tick_types import TickResult, ControlOutput
from communication.exec_channels import BATCH_FLAG_BLEND_CONTINUES, unpack_batch
from config.networking import ZMQ_EXEC_TRAJECTORY_SOCKET, ZMQ_EXEC_STATUS_SOCKET

logger = logging.getLogger(__name__)

COMPLETION_COMMANDED = 0
COMPLETION_SETTLED = 1
COMPLETION_STRICT = 2
COMPLETION_POLICIES = {
    "commanded": COMPLETION_COMMANDED,
    "settled": COMPLETION_SETTLED,
    "strict": COMPLETION_STRICT,
}
COMPLETION_POLICY_NAMES = ("commanded", "settled", "strict")


class ExecHandler:
    """Consumes ZMQ trajectory batches and produces per-tick joint setpoints."""

    # BUFFER BY SAMPLES, NOT BY BATCHES. This used to hold exactly ONE prefetched batch
    # (`_next`) plus whatever ZMQ had queued, so the runway was measured in BATCHES — and a
    # blended chain is sent as ONE BATCH PER COMMAND. On a G-code stream of short commands
    # (~7 samples each, measured) that made RTI's whole buffer ~10 batches = ~280ms, no
    # matter how much trajectory the planner had actually sent.
    #
    # It stayed hidden while blended motion was slow. The 2026-07-12 arc-length fix made it
    # ~5x faster, which cut the samples-per-command ~5x, which cut RTI's runway ~5x — and the
    # arm stalled mid-stream for 0.20s (hardware, 200-chord run). The speedup did not cause
    # that; it uncovered it.
    #
    # Now we drain the socket until we hold TARGET_BUFFER_SAMPLES of actual motion, so the
    # runway is a TIME, independent of how the planner chose to chunk it. This also makes
    # `exec_samples_remaining` mean what its name says (see _mirror_exec_state) — the planner
    # had been flying on a wall-clock estimate because the published number was structurally
    # tiny for short commands.
    TARGET_BUFFER_SAMPLES = 750        # 3.0s at 4ms — comfortably over the planner's margin
    MAX_PULLS_PER_TICK = 8             # bound the per-tick unpack cost; this is the RT loop
    # Bound how many batches ZMQ buffers ahead of us (each <=256 KB, but a per-command batch
    # on a dense toolpath is only a few hundred bytes). Must be deep enough to hold
    # TARGET_BUFFER_SAMPLES worth of SHORT batches, or we re-create the bug above one layer
    # down — and it must not be so deep that the planner can run far ahead of a stop().
    RECV_HWM = 512
    # Kept for compatibility with anything that reads it; the sample-based target above is
    # what actually drives prefetching now.
    PREFETCH_THRESHOLD = 100
    # Completion policy defaults. At a 1 kHz RTI tick, 500 ticks is 0.5 s.
    DEFAULT_SETTLE_TOLERANCE_RAD = 0.01
    DEFAULT_SETTLE_MAX_TICKS = 500

    def __init__(
            self,
            joint_num,
            shared_data=None,
            completion_policy="settled",
            settle_tolerance_rad=DEFAULT_SETTLE_TOLERANCE_RAD,
            settle_max_ticks=DEFAULT_SETTLE_MAX_TICKS):
        self.joint_num = joint_num
        self._shared_data = shared_data

        # Pre-allocated buffers — never reallocated inside step().
        self._target_pos     = np.zeros(joint_num, dtype=np.float64)  # rad
        self._target_vel     = np.zeros(joint_num, dtype=np.float64)  # rad/s
        self._target_acc     = np.zeros(joint_num, dtype=np.float64)  # rad/s^2, derived from velocity setpoint
        self._target_trq_ff  = np.zeros(joint_num, dtype=np.float64)  # Nm (planner feedforward)
        self._trq            = np.zeros(joint_num, dtype=np.float64)  # Nm (ff + gravity, commanded)
        self._vel_zero       = np.zeros(joint_num, dtype=np.float64)  # rad/s, hold velocity
        self._acc_zero       = np.zeros(joint_num, dtype=np.float64)  # rad/s^2, hold acceleration
        self._last_target_pos = np.zeros(joint_num, dtype=np.float64)  # rad, hold position
        self._last_target_vel = np.zeros(joint_num, dtype=np.float64)  # rad/s, for finite-diff acceleration
        self._settle_error   = np.zeros(joint_num, dtype=np.float64)  # rad

        # Reused ControlOutput — fields overwritten each tick (no per-tick alloc).
        self._out = ControlOutput(mode="pid")

        # Batch state (double-buffer for seamless swaps).
        self._current = None   # dict from unpack_batch (holds arrays that keep raw alive)
        self._index = 0
        # A QUEUE of pending batches plus a RUNNING SAMPLE COUNT. The count is maintained
        # incrementally (never re-summed) so step() stays O(1) and allocation-free.
        self._pending = deque()
        self._pending_samples = 0

        # Checkpoint statuses awaiting a successful non-blocking send.
        self._pending_status = []

        # Pause state — set via pause()/resume(), called from RTI's command dispatch.
        self._paused = False

        # Completion state. "commanded" completes after the last commanded sample.
        # "settled" and "strict" hold the final target until measured joints are
        # within tolerance or the settle window expires.
        self._completion_policy = COMPLETION_SETTLED
        self._settle_tolerance_rad = float(settle_tolerance_rad)
        self._settle_max_ticks = int(settle_max_ticks)
        self._settling = False
        self._settling_checkpoint_id = ""
        self._settling_policy = COMPLETION_SETTLED
        self._settling_ticks = 0
        self.set_completion_policy(completion_policy)

        # ZMQ sockets — bound/connected once for the process lifetime. enter()/exit()
        # only drain and reset local state; they do not open/close sockets (avoids
        # IPC rebind races on every mode transition).
        ctx = zmq.Context.instance()
        self._traj_sock = ctx.socket(zmq.PULL)
        self._traj_sock.setsockopt(zmq.RCVHWM, self.RECV_HWM)
        self._traj_sock.bind(ZMQ_EXEC_TRAJECTORY_SOCKET)

        self._status_sock = ctx.socket(zmq.PUSH)
        self._status_sock.setsockopt(zmq.SNDHWM, 64)
        self._status_sock.setsockopt(zmq.LINGER, 0)
        self._status_sock.connect(ZMQ_EXEC_STATUS_SOCKET)

    # ------------------------------------------------------------------ #
    # Mode handler interface (called by mode_dispatch / RTI.py)
    # ------------------------------------------------------------------ #

    def enter(self, ctx, **kwargs):
        """Reset batch state; drain any stale batches; capture hold position."""
        self._drain_traj_socket()
        self._current = None
        self._pending.clear()
        self._pending_samples = 0
        self._index = 0
        self._pending_status.clear()
        self._paused = False
        self._clear_settling()
        self._mirror_exec_state(ctx)
        self._last_target_pos[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
        # Filtered, not raw: this seeds trajectory continuity, and a raw (encoder-dither) speed
        # here would put noise into the next move's start velocity. Before the 2026-07-17
        # raw/filtered split joint_speed WAS filtered, so this restores that behaviour.
        self._last_target_vel[:] = ctx.shared_data.joint['joint_speed_filtered'][:self.joint_num]

    def pause(self):
        """Freeze consumption in place. Batch/index untouched — resume() continues from here."""
        self._paused = True

    def resume(self):
        """Continue consuming from exactly where pause() froze."""
        self._paused = False

    def set_completion_policy(self, policy):
        """Set final-checkpoint completion behavior: commanded, settled, or strict."""
        if isinstance(policy, str):
            key = policy.strip().lower()
            if key not in COMPLETION_POLICIES:
                raise ValueError(
                    f"unknown EXEC completion policy '{policy}' "
                    f"(expected one of {', '.join(COMPLETION_POLICIES)})")
            self._completion_policy = COMPLETION_POLICIES[key]
            self._mirror_completion_policy()
            return
        policy_idx = int(policy)
        if policy_idx < 0 or policy_idx >= len(COMPLETION_POLICY_NAMES):
            raise ValueError(f"unknown EXEC completion policy index {policy_idx}")
        self._completion_policy = policy_idx
        self._mirror_completion_policy()

    def step(self, ctx):
        # 0. Flush any checkpoint statuses that couldn't send on earlier ticks.
        self._flush_status()

        # Mirror pause state into shared memory every tick (cheap; keeps
        # command_executor's status stream correct even if pause()/resume() raced
        # with a status read). Cheaper than diffing — it's one float64 write.
        self._mirror_exec_state(ctx)

        # 1. PAUSED: hold exactly in place. Batch/index untouched so resume() continues
        #    seamlessly. Distinct from "no batch" — this freezes a specific waypoint.
        if self._paused:
            return TickResult(), self._hold(ctx)

        if self._settling:
            return TickResult(), self._tick_settling(ctx)

        # 2. PRE-FETCH until we hold TARGET_BUFFER_SAMPLES of motion — a TIME-based runway,
        #    not a batch-count one. Bounded pulls per tick so a burst cannot blow the budget.
        remaining = (self._current['num_points'] - self._index) if self._current else 0
        pulls = 0
        while (remaining + self._pending_samples < self.TARGET_BUFFER_SAMPLES
               and pulls < self.MAX_PULLS_PER_TICK):
            raw = self._recv_nonblocking()
            if raw is None:
                break
            batch = self._safe_unpack(raw)
            if batch is None:
                break
            self._pending.append(batch)
            self._pending_samples += batch['num_points']
            pulls += 1

        # 3. SWAP if the current batch is exhausted (or absent).
        if self._current is None or self._index >= self._current['num_points']:
            if self._current is not None and self._current['is_last_chunk']:
                if self._begin_completion(self._current['checkpoint_id'], self._current.get('flags', 0)):
                    self._current = None
                    return TickResult(), self._hold(ctx)
            # Mark the finished batch consumed so its checkpoint can't re-fire on
            # subsequent hold ticks (when no next batch is available).
            self._current = None

            if self._pending:
                self._current = self._pending.popleft()
                self._pending_samples -= self._current['num_points']
                self._index = 0
            else:
                # Nothing pre-fetched — one more non-blocking try, else hold.
                raw = self._recv_nonblocking()
                batch = self._safe_unpack(raw) if raw is not None else None
                if batch is not None:
                    self._current = batch
                    self._index = 0
                else:
                    return TickResult(), self._hold(ctx)

        # 4. READ this tick's waypoint into pre-allocated buffers.
        i = self._index
        self._target_pos[:]    = self._current['joint_positions'][i]
        self._target_vel[:]    = self._current['joint_velocities'][i]
        self._target_trq_ff[:] = self._current['torque_ff'][i]
        np.subtract(self._target_vel, self._last_target_vel, out=self._target_acc)
        self._target_acc *= (1.0 / ctx.delta_t) if ctx.delta_t > 0 else 0.0
        self._index += 1
        self._last_target_pos[:] = self._target_pos
        self._last_target_vel[:] = self._target_vel

        # 4. Commanded torque = planner feedforward + gravity comp (when enabled).
        if bool(ctx.shared_data.rti['gravity_comp_enabled'][0]):
            np.add(self._target_trq_ff,
                   ctx.shared_data.joint['joint_gravity_torque'][:self.joint_num],
                   out=self._trq)
        else:
            self._trq[:] = self._target_trq_ff

        out = self._out
        out.pos_rad = self._target_pos
        out.vel_rad = self._target_vel
        out.acc_rad = self._target_acc
        out.trq_nm  = self._trq
        # Target = raw planner intent (before gravity comp), commanded = what we send.
        out.target_pos_rad = self._target_pos
        out.target_vel_rad = self._target_vel
        out.target_acc_rad = self._target_acc
        out.target_trq_nm  = self._target_trq_ff
        return TickResult(), out

    def exit(self, ctx):
        """Flush remaining batches so a re-entry starts clean. Sockets stay open."""
        self._drain_traj_socket()
        self._current = None
        self._pending.clear()
        self._pending_samples = 0
        self._index = 0
        self._paused = False
        self._clear_settling()
        self._mirror_exec_state(ctx)

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _hold(self, ctx):
        """Hold last target position with Speed=0 and gravity feedforward."""
        if bool(ctx.shared_data.rti['gravity_comp_enabled'][0]):
            self._trq[:] = ctx.shared_data.joint['joint_gravity_torque'][:self.joint_num]
        else:
            self._trq[:] = 0.0
        out = self._out
        out.pos_rad = self._last_target_pos
        out.vel_rad = self._vel_zero
        out.acc_rad = self._acc_zero
        out.trq_nm  = self._trq
        out.target_pos_rad = self._last_target_pos
        out.target_vel_rad = self._vel_zero
        out.target_acc_rad = self._acc_zero
        out.target_trq_nm  = None
        return out

    def _mirror_exec_state(self, ctx):
        if ctx.shared_data is not None:
            ctx.shared_data.rti['exec_paused'][0] = 1.0 if self._paused else 0.0
            ctx.shared_data.rti['exec_settling'][0] = 1.0 if self._settling else 0.0
            ctx.shared_data.rti['exec_completion_policy_idx'][0] = float(self._completion_policy)
            # Buffered runway (samples not yet consumed in current + prefetched next).
            # The planner reads this for the rolling-buffer deadline cross-check
            # (TODO/blending_design.md §6): when this drops below the planner's
            # safety margin and it's still holding a command's tail back, it must
            # commit the tail now or RTI will run dry and hold (freeze) at the
            # boundary. Written every tick from the RT loop — one float64 store.
            remaining = 0
            if self._current is not None:
                remaining += self._current['num_points'] - self._index
            remaining += self._pending_samples
            ctx.shared_data.rti['exec_samples_remaining'][0] = float(remaining)

    def _mirror_completion_policy(self):
        if self._shared_data is not None:
            self._shared_data.rti['exec_completion_policy_idx'][0] = float(self._completion_policy)

    def _recv_nonblocking(self):
        try:
            return self._traj_sock.recv(flags=zmq.NOBLOCK, copy=True)
        except zmq.Again:
            return None

    def _drain_traj_socket(self):
        while self._recv_nonblocking() is not None:
            pass

    def _safe_unpack(self, raw):
        try:
            return unpack_batch(raw)
        except Exception as e:  # malformed batch — drop it, don't crash the RT loop
            logger.error(f"[EXEC] Dropped malformed trajectory batch: {e}")
            return None

    def _begin_completion(self, checkpoint_id, flags=0):
        """Return True if playback should pause for settle checking."""
        if not checkpoint_id:
            return False
        if flags & BATCH_FLAG_BLEND_CONTINUES:
            # A blended successor is already waiting; do not force a settled hold at
            # the corner or the velocity-continuous seam turns into a stop/start.
            self._complete_checkpoint(checkpoint_id)
            return False
        if self._completion_policy == COMPLETION_COMMANDED:
            self._complete_checkpoint(checkpoint_id)
            return False
        self._settling = True
        self._settling_checkpoint_id = checkpoint_id
        self._settling_policy = self._completion_policy
        self._settling_ticks = 0
        return True

    def _tick_settling(self, ctx):
        if self._is_settled(ctx):
            self._complete_checkpoint(self._settling_checkpoint_id)
            self._clear_settling()
            self._mirror_exec_state(ctx)
            return self._hold(ctx)

        self._settling_ticks += 1
        if self._settling_ticks >= self._settle_max_ticks:
            if self._settling_policy == COMPLETION_STRICT:
                self._error_checkpoint(
                    self._settling_checkpoint_id,
                    "SETTLE_TIMEOUT",
                    "EXEC final target did not settle within tolerance")
            else:
                self._complete_checkpoint(self._settling_checkpoint_id)
            self._clear_settling()
            self._mirror_exec_state(ctx)
        return self._hold(ctx)

    def _is_settled(self, ctx):
        actual = ctx.shared_data.joint['joint_position'][:self.joint_num]
        np.subtract(actual, self._last_target_pos, out=self._settle_error)
        np.abs(self._settle_error, out=self._settle_error)
        return bool(np.all(self._settle_error <= self._settle_tolerance_rad))

    def _clear_settling(self):
        self._settling = False
        self._settling_checkpoint_id = ""
        self._settling_ticks = 0

    def _complete_checkpoint(self, checkpoint_id):
        if checkpoint_id:
            self._pending_status.append({"checkpoint_id": checkpoint_id, "status": "complete"})
        self._flush_status()

    def _error_checkpoint(self, checkpoint_id, code, message):
        if checkpoint_id:
            self._pending_status.append({
                "checkpoint_id": checkpoint_id,
                "status": "error",
                "code": code,
                "message": message,
            })
        self._flush_status()

    def _flush_status(self):
        """Non-blocking send of pending checkpoint completions. Retries next tick."""
        while self._pending_status:
            status = self._pending_status[0]
            try:
                self._status_sock.send_json(status, flags=zmq.NOBLOCK)
            except zmq.Again:
                break  # executor not draining yet — retry next tick
            self._pending_status.pop(0)
