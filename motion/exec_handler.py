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
  2. Paused (or stopped) at rest: hold in place — see "Controlled pause / stop" below.
  3. Pre-fetch the next batch before the current one runs out (double-buffer, no gap).
  4. Swap to the pre-fetched batch when the current is exhausted; publish checkpoint.
  5. Read this tick's waypoint → ControlOutput(pos, vel, torque_ff + gravity).
  6. If no batch available: hold last position (Speed=0) with gravity feedforward.

Controlled pause / stop (2026-10-13, user's choice: as industrial arms commonly do it). pause() / resume() /
begin_stop() are called from RTI.py's command dispatch ("exec_pause" / "exec_resume"; a set_mode:IDLE while EXEC plays —
the client's stop() and enter_idle_mode() — calls begin_stop()), the same single-threaded pattern as
JogHandler.set_jog — no locking.
  - pause() brakes the arm ALONG THE PLANNED PATH at the joints' hardware ceilings — robot XML
    <joint_acceleration_limit>, the limits hard RTI's controlled stops use — and holds it there; exec_paused once it
    is at rest. Never above <joint_jerk_limit> * dt / 2 either: then stepping in one tick from EXEC's acceleration
    (when it is no higher) to full braking, and back, keeps within the jerk ceiling. PAR6: J * dt / 2 = A at 250 Hz,
    so it brakes at the full ceiling; a robot XML on the parser's 50 x fallback would brake at a tenth of it.
  - resume() speeds it back up along the same path at EXEC's own acceleration (<exec_limits>) and continues from
    exactly where it stopped.
  - A STOP (exec_stop:<epoch> — the client's stop(), 2026-10-14, user: "stays in EXEC, holding") brakes the same way,
    then drops the program — what it holds and everything on the socket up to the planner's FENCE of that epoch
    (communication/exec_channels.pack_fence; batches carry the epoch they arrived in) — and holds there, still in
    EXEC, exec_stop_seq = the epoch. Braking never runs into a batch from after the fence (a new program).
  - set_mode:IDLE while EXEC moves (enter_idle_mode()) brakes the same way, waits for the measured arm to come to rest
    (IDLE is freedrive — a moving arm would coast), then asks for IDLE.
  - Errors and the E-stop stay instant (ACTIVE_ERROR); an error during a pause drops the rest by leaving EXEC.
Until 2026-10-13 a pause FROZE the setpoint (the drives stopped the arm as hard as they could) and a stop switched
to IDLE at speed (freedrive: the arm coasted).

How: playback has a RATE r (1 = as planned, 0 = stopped) and a fractional PLAYHEAD between two samples. Each tick
the playhead advances by the new rate r'; the setpoint is the planned path interpolated there, its velocity the
planned velocity w (taken where the old rate would have put the playhead) times r'. r changes as fast as the limits
allow: on every joint the commanded velocity may change by at most A * dt per tick, |w * r' - v_last| <= A * dt —
linear in r', so one bound per joint, solved exactly. At r = 1 on a whole sample the tick is the plain sample read,
exactly as before.

RT rules (see CLAUDE.md): all numpy buffers and the ControlOutput are pre-allocated in
__init__; step() only writes into them. ZMQ receives/sends are non-blocking — the RT
tick never blocks on IPC.
"""

import logging
import math

from collections import deque

import numpy as np
import zmq

from utility.state_machine import RobotMode
from utility.tick_types import TickResult, ControlOutput
from communication.exec_channels import BATCH_FLAG_BLEND_CONTINUES, fence_epoch, unpack_batch
from config.networking import ZMQ_EXEC_TRAJECTORY_SOCKET, ZMQ_EXEC_STATUS_SOCKET, background_zmq_context

logger = logging.getLogger("rti.exec")  # docs/logging.md "Logger names"

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
    # Controlled pause / stop (module docstring). A stop asks for IDLE once the MEASURED arm is at rest — every
    # joint's filtered speed below this — or after
    # STOP_REST_MAX_S at the latest: IDLE is freedrive, where a still-moving arm would coast.
    STOP_REST_SPEED_RAD_S = 0.02
    STOP_REST_MAX_S = 0.5
    _VEL_EPS = 1e-9            # rad/s: a joint the path does not move at the new playhead sets no bound on the rate

    def __init__(
            self,
            joint_num,
            shared_data=None,
            completion_policy="settled",
            settle_tolerance_rad=DEFAULT_SETTLE_TOLERANCE_RAD,
            settle_max_ticks=DEFAULT_SETTLE_MAX_TICKS,
            *,
            stop_accel,
            stop_jerk,
            resume_accel):
        """stop_accel / stop_jerk: the robot XML's hardware ceilings (rad/s^2, rad/s^3 per joint) a pause or a stop
        brakes at; resume_accel: EXEC's own acceleration (<exec_limits>), at which a resume speeds back up."""
        self.joint_num = joint_num
        self._shared_data = shared_data

        # Controlled pause / stop limits — copies, so nothing outside can change them under the RT loop.
        self._stop_accel = np.array(stop_accel, dtype=np.float64)
        self._stop_jerk = np.array(stop_jerk, dtype=np.float64)
        self._resume_accel = np.array(resume_accel, dtype=np.float64)
        for name, lim in (("stop_accel", self._stop_accel), ("stop_jerk", self._stop_jerk),
                          ("resume_accel", self._resume_accel)):
            if lim.shape != (joint_num,) or not np.all(np.isfinite(lim)) or np.any(lim <= 0.0):
                raise ValueError(f"ExecHandler {name} must be {joint_num} finite values > 0, got {lim!r}")

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

        # Playback rate and playhead (module docstring). The last OUTPUT sat at playhead (_index - 1) + _frac of
        # the current batch (-1 = the tail below); at r = 1 the fraction stays 0 and this is the plain sample read.
        self._rate = 1.0
        self._rate_target = 1.0            # 0 = pause / stop asked for, 1 = play (resume)
        self._frac = 0.0
        self._stopping = False             # begin_stop(): brake, then hold (a stop) or IDLE once the arm is at rest
        self._stop_epoch = None            # a STOP's epoch (hold in EXEC); None = the stop ends in IDLE
        self._stop_done = 0                # the epoch of the last completed stop (mirrored: exec_stop_seq)
        self._rx_epoch = 0                 # the last fence received: every batch is tagged with it on arrival
        self._stop_ticks = 0               # ticks since begin_stop() (the log line)
        self._rest_wait = 0                # ticks at r = 0 waiting for the measured arm to come to rest
        self._rest_wait_max = 0            # STOP_REST_MAX_S in ticks (set with the bands, from dt)
        self._idle_asked = False
        self._band_dt = 0.0                # the dt the per-tick velocity bands below were computed for
        self._stop_band = np.zeros(joint_num, dtype=np.float64)     # rad/s per tick, braking
        self._resume_band = np.zeros(joint_num, dtype=np.float64)   # rad/s per tick, speeding up
        # The sample before the current batch's first: the previous batch's last (or the hold point) — a playhead
        # between the two interpolates across the seam. Shaped like a batch so _locate() treats it as one.
        self._tail = {'num_points': 1, 'is_last_chunk': False, 'checkpoint_id': '', 'flags': 0,
                      'joint_positions': np.zeros((1, joint_num), dtype=np.float64),
                      'joint_velocities': np.zeros((1, joint_num), dtype=np.float64),
                      'torque_ff': np.zeros((1, joint_num), dtype=np.float64)}
        # Rate solver buffers.
        self._ramp_w = np.zeros(joint_num, dtype=np.float64)        # path velocity at the candidate playhead
        self._ramp_lo = np.zeros(joint_num, dtype=np.float64)
        self._ramp_hi = np.zeros(joint_num, dtype=np.float64)
        self._ramp_t1 = np.zeros(joint_num, dtype=np.float64)
        self._ramp_t2 = np.zeros(joint_num, dtype=np.float64)
        self._ramp_abs = np.zeros(joint_num, dtype=np.float64)
        self._ramp_moving = np.zeros(joint_num, dtype=bool)
        self._tick_ok = TickResult()
        self._tick_to_idle = TickResult(pending_mode=RobotMode.IDLE)

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
        # Its I/O threads at normal priority off the RT core (config/networking.background_zmq_context).
        ctx = background_zmq_context(zmq.Context.instance())
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
        self._reset_playback()
        self._clear_settling()
        self._mirror_exec_state(ctx)
        self._last_target_pos[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
        # Filtered, not raw: this seeds trajectory continuity, and a raw (encoder-dither) speed
        # here would put noise into the next move's start velocity. Before the 2026-07-17
        # raw/filtered split joint_speed WAS filtered, so this restores that behaviour.
        self._last_target_vel[:] = ctx.shared_data.joint['joint_speed_filtered'][:self.joint_num]

    def pause(self):
        """Brake along the planned path to rest and hold there (exec_paused once at rest); resume() continues."""
        if not self._stopping:
            self._rate_target = 0.0

    def resume(self):
        """Speed back up along the same path, from exactly where pause() stopped it. Ignored while stopping."""
        if not self._stopping:
            self._rate_target = 1.0

    def begin_stop(self, epoch=None):
        """Brake along the planned path. epoch (a Stop): then drop the program up to that epoch's fence and hold, in
        EXEC. None (set_mode:IDLE): ask for IDLE once the measured arm is at rest (step() returns it) — also when a
        stop is already braking or waiting for its fence (the executor's fallback)."""
        if self._stopping:
            if epoch is None:
                self._stop_epoch = None
            elif self._stop_epoch is not None and epoch > self._stop_epoch:
                # A second Stop while one runs (two clients, 2026-10-14: the Blocks server and its runner, 0.5 ms
                # apart): it takes over — its fence comes after the first's, so it covers everything, and the
                # executor waits for ITS epoch (ignored, it fell back to IDLE after 1.5 s).
                self._stop_epoch = int(epoch)
            return
        if epoch is not None and self._current is not None and self._current.get('epoch', 0) >= epoch:
            self._stop_done = max(self._stop_done, int(epoch))    # its program ended already: this one came after it
            return
        self._stopping = True
        self._stop_epoch = None if epoch is None else int(epoch)
        self._rate_target = 0.0
        self._stop_ticks = 0
        self._rest_wait = 0
        self._idle_asked = False

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

        # 1. A controlled pause / stop / resume runs (rate or target below 1), or the playhead sits between two
        #    samples (a resume ended there): the ramped read below. Paused or stopped AT REST: hold right there —
        #    batch and playhead untouched, so resume() continues seamlessly.
        if self._stopping:
            self._stop_ticks += 1
        ramp = self._rate != 1.0 or self._rate_target != 1.0 or self._frac != 0.0
        if ramp and self._rate == 0.0 and self._rate_target == 0.0:
            return self._tick_at_rest(ctx)

        if self._settling:
            self._rate = self._rate_target     # nothing moves while settling: no ramp to run
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
            if self._note_fence(raw):
                pulls += 1
                continue
            batch = self._safe_unpack(raw)
            if batch is None:
                break
            batch['epoch'] = self._rx_epoch
            self._pending.append(batch)
            self._pending_samples += batch['num_points']
            pulls += 1

        # 3. SWAP if the current batch is exhausted (or absent). Not while the playhead sits between its last sample
        #    and the next batch's first (a fraction left): the ramped read crosses that seam itself.
        if self._frac == 0.0 and (self._current is None or self._index >= self._current['num_points']):
            self._keep_tail()
            if self._current is not None and self._current['is_last_chunk']:
                if self._begin_completion(self._current['checkpoint_id'], self._current.get('flags', 0)):
                    self._current = None
                    self._rate = self._rate_target     # holding: no ramp to run
                    return TickResult(), self._hold(ctx)
            # Mark the finished batch consumed so its checkpoint can't re-fire on
            # subsequent hold ticks (when no next batch is available).
            self._current = None

            if self._pending and not self._after_stop(self._pending[0]):
                self._current = self._pending.popleft()
                self._pending_samples -= self._current['num_points']
                self._index = 0
            else:
                # Nothing pre-fetched (or only a program from after a stop's fence) — one more try, else hold.
                batch = None if self._pending else self._recv_one()
                if batch is not None and not self._after_stop(batch):
                    self._current = batch
                    self._index = 0
                else:
                    if batch is not None:                # from after the stop's fence: it plays once the stop is done
                        self._pending.append(batch)
                        self._pending_samples += batch['num_points']
                    self._rate = self._rate_target     # holding: no ramp to run
                    return TickResult(), self._hold(ctx)

        if ramp:
            self._read_ramped(ctx)
            return TickResult(), self._output(ctx)

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
        return TickResult(), self._output(ctx)

    def _output(self, ctx):
        """The tick's ControlOutput from the _target_* buffers just read."""
        # Commanded torque = planner feedforward + gravity comp (when enabled).
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
        return out

    def exit(self, ctx):
        """Flush remaining batches so a re-entry starts clean. Sockets stay open."""
        self._drain_traj_socket()
        self._current = None
        self._pending.clear()
        self._pending_samples = 0
        self._index = 0
        self._reset_playback()
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

    # ---- controlled pause / stop (module docstring) ------------------------------------------------------------

    def _reset_playback(self):
        self._rate = 1.0
        self._rate_target = 1.0
        self._frac = 0.0
        self._stopping = False
        self._stop_epoch = None
        self._stop_ticks = 0
        self._rest_wait = 0
        self._idle_asked = False

    def _tick_at_rest(self, ctx):
        """Paused or stopped at rest: hold. A stop then drops its program and holds (a Stop), or asks for IDLE once
        the measured arm is at rest too (set_mode:IDLE)."""
        if self._stopping and self._stop_epoch is not None:
            return self._finish_stop(ctx)
        if self._stopping:
            if self._band_dt != ctx.delta_t:
                self._set_bands(ctx.delta_t)
            self._rest_wait += 1
            if self._arm_at_rest(ctx) or self._rest_wait >= self._rest_wait_max:
                if not self._idle_asked:
                    self._idle_asked = True
                    logger.info("stop: braked along the path, at rest %.0f ms after the stop (%s) -> IDLE",
                                self._stop_ticks * ctx.delta_t * 1000.0,
                                "measured still" if self._rest_wait < self._rest_wait_max else
                                "still moving after %.1f s — IDLE anyway" % self.STOP_REST_MAX_S)
                return self._tick_to_idle, self._hold(ctx)
        return self._tick_ok, self._hold(ctx)

    def _arm_at_rest(self, ctx):
        np.abs(ctx.shared_data.joint['joint_speed_filtered'][:self.joint_num], out=self._ramp_abs)
        return float(self._ramp_abs.max()) < self.STOP_REST_SPEED_RAD_S

    def _note_fence(self, raw):
        """True when `raw` was a stop's fence (its epoch noted: every batch after it is tagged with it)."""
        e = fence_epoch(raw)
        if e is None:
            return False
        if e > self._rx_epoch:
            self._rx_epoch = e
        return True

    def _recv_one(self):
        """One more non-blocking try for a batch — a fence noted on the way —, tagged with its epoch; else None."""
        raw = self._recv_nonblocking()
        if raw is None or self._note_fence(raw):
            return None
        batch = self._safe_unpack(raw)
        if batch is not None:
            batch['epoch'] = self._rx_epoch
        return batch

    def _after_stop(self, batch):
        """A batch from after the fence of the stop in progress — a new program, never played while stopping."""
        return self._stop_epoch is not None and batch['epoch'] >= self._stop_epoch

    def _finish_stop(self, ctx):
        """A Stop at rest: drop the stopped program (the batch played, everything queued before the fence), wait for
        the fence if it is not in yet (what arrives before it is the stopped program too), then hold here in EXEC —
        what came after the fence plays next."""
        n = self._stop_epoch
        self._current = None
        self._index = 0
        self._frac = 0.0
        self._clear_settling()
        while self._pending and self._pending[0]['epoch'] < n:
            self._pending_samples -= self._pending.popleft()['num_points']
        pulls = 0
        while self._rx_epoch < n and pulls < self.MAX_PULLS_PER_TICK:
            raw = self._recv_nonblocking()
            if raw is None:
                break
            pulls += 1
            self._note_fence(raw)              # anything else before the fence: the stopped program, dropped
        if self._rx_epoch < n:
            return self._tick_ok, self._hold(ctx)
        self._stop_done = n
        logger.info("stop: braked along the path, at rest %.0f ms after the stop; the program dropped "
                    "(epoch %d), holding here in EXEC", self._stop_ticks * ctx.delta_t * 1000.0, n)
        self._reset_playback()
        self._mirror_exec_state(ctx)
        return self._tick_ok, self._hold(ctx)

    def _set_bands(self, dt):
        """The per-tick velocity change allowed: braking at the ceiling, but never above jerk * dt / 2 (module
        docstring: the one-tick step to full braking then keeps within the jerk ceiling); speeding up at EXEC's."""
        np.multiply(self._stop_jerk, 0.5 * dt, out=self._stop_band)
        np.minimum(self._stop_band, self._stop_accel, out=self._stop_band)
        self._stop_band *= dt
        np.multiply(self._resume_accel, dt, out=self._resume_band)
        self._rest_wait_max = max(1, int(round(self.STOP_REST_MAX_S / dt))) if dt > 0 else 1
        self._band_dt = dt

    def _keep_tail(self):
        """The sample before the next batch's first: this batch's last, or (no batch) the hold point at rest."""
        tail = self._tail
        cur = self._current
        if cur is not None:
            n = cur['num_points'] - 1
            tail['joint_positions'][0] = cur['joint_positions'][n]
            tail['joint_velocities'][0] = cur['joint_velocities'][n]
            tail['torque_ff'][0] = cur['torque_ff'][n]
        else:
            tail['joint_positions'][0] = self._last_target_pos
            tail['joint_velocities'][0] = 0.0
            tail['torque_ff'][0] = 0.0

    def _continues(self, batch):
        """Playback runs straight on from `batch` into the next one — exactly when _begin_completion would not hold."""
        if not batch['is_last_chunk'] or not batch['checkpoint_id']:
            return True
        if batch.get('flags', 0) & BATCH_FLAG_BLEND_CONTINUES:
            return True
        return self._completion_policy == COMPLETION_COMMANDED

    def _locate(self, k):
        """The batch and row of playhead sample k: -1 is the tail, past the current batch the pending ones in order
        as far as playback runs on into them. (None, 0) beyond the end of the playable path."""
        if k < 0:
            return self._tail, 0
        b = self._current
        j = 0
        while k >= b['num_points']:
            if j >= len(self._pending) or not self._continues(b) or self._after_stop(self._pending[j]):
                return None, 0
            k -= b['num_points']
            b = self._pending[j]
            j += 1
        return b, k

    def _path_point(self, p, key, out):
        """out = the planned `key` interpolated at playhead p. Returns the playhead used: p, or the last sample of
        the playable path when p lies beyond it (p is never more than one sample past the last output)."""
        k = math.floor(p)
        b, i = self._locate(k)
        if b is None:                      # past the end of the playable path: stay on its last sample
            k -= 1
            b, i = self._locate(k)
            out[:] = b[key][i]
            return float(k)
        g = p - k
        if g == 0.0:
            out[:] = b[key][i]
            return p
        b2, i2 = self._locate(k + 1)
        if b2 is None:
            out[:] = b[key][i]
            return float(k)
        np.subtract(b2[key][i2], b[key][i], out=out)
        out *= g
        out += b[key][i]
        return p

    def _rate_bound(self, band, slowing, rate):
        """The new rate from the path velocity at the candidate playhead (_ramp_w): on every joint
        |w * r' - v_last| <= band, i.e. r' in [(v_last - band) / w, (v_last + band) / w] — the lowest (slowing) or
        highest (speeding up) rate every joint allows, never past the target nor back past the current rate. A joint
        the path does not move there sets no bound. Should the joints' ranges not overlap, the rate stays."""
        w = self._ramp_w
        np.subtract(self._last_target_vel, band, out=self._ramp_lo)
        np.add(self._last_target_vel, band, out=self._ramp_hi)
        np.abs(w, out=self._ramp_abs)
        np.greater(self._ramp_abs, self._VEL_EPS, out=self._ramp_moving)
        self._ramp_t1.fill(-np.inf)
        self._ramp_t2.fill(np.inf)
        np.divide(self._ramp_lo, w, out=self._ramp_t1, where=self._ramp_moving)
        np.divide(self._ramp_hi, w, out=self._ramp_t2, where=self._ramp_moving)
        np.minimum(self._ramp_t1, self._ramp_t2, out=self._ramp_lo)
        np.maximum(self._ramp_t1, self._ramp_t2, out=self._ramp_hi)
        lo = float(self._ramp_lo.max())
        hi = float(self._ramp_hi.min())
        if slowing:
            new = lo if lo > self._rate_target else self._rate_target
            return rate if new > hi or new > rate else new
        new = hi if hi < self._rate_target else self._rate_target
        return rate if new < lo or new < rate else new

    def _read_ramped(self, ctx):
        """This tick's setpoint at the new rate: the playhead advanced by it, the path interpolated there."""
        dt = ctx.delta_t
        if self._band_dt != dt:
            self._set_bands(dt)
        last = (self._index - 1) + self._frac
        rate = self._rate
        # The path velocity where the CURRENT rate would put the playhead; the new rate scales it. That makes the
        # bound exact (the velocity sent is w * r', the very expression bounded) — the path velocity at the new
        # playhead differs from w only by the plan's acceleration over (rate - r') of a sample (< 0.002 rad/s).
        self._path_point(last + rate, 'joint_velocities', self._ramp_w)
        new = rate
        if self._rate_target != rate:
            slowing = self._rate_target < rate
            new = self._rate_bound(self._stop_band if slowing else self._resume_band, slowing, rate)
        p = self._path_point(last + new, 'joint_positions', self._target_pos)
        self._path_point(p, 'torque_ff', self._target_trq_ff)
        np.multiply(self._ramp_w, new, out=self._target_vel)
        np.subtract(self._target_vel, self._last_target_vel, out=self._target_acc)
        self._target_acc *= (1.0 / dt) if dt > 0 else 0.0
        self._last_target_pos[:] = self._target_pos
        self._last_target_vel[:] = self._target_vel
        self._rate = new
        # The playhead into the next batch when it crossed over (_locate only lets it into one that continues).
        n = self._current['num_points']
        while p >= n:
            self._keep_tail()
            cur = self._current
            if cur['is_last_chunk']:
                self._begin_completion(cur['checkpoint_id'], cur.get('flags', 0))   # continues: reported, no hold
            self._current = self._pending.popleft()
            self._pending_samples -= self._current['num_points']
            p -= n
            n = self._current['num_points']
        k = math.floor(p)
        self._index = k + 1
        self._frac = p - k

    def _mirror_exec_state(self, ctx):
        if ctx.shared_data is not None:
            paused = self._rate == 0.0 and self._rate_target == 0.0 and not self._stopping
            ctx.shared_data.rti['exec_paused'][0] = 1.0 if paused else 0.0
            ctx.shared_data.rti['exec_stopping'][0] = 1.0 if self._stopping else 0.0
            ctx.shared_data.rti['exec_stop_seq'][0] = float(self._stop_done)
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
        while True:
            raw = self._recv_nonblocking()
            if raw is None:
                return
            self._note_fence(raw)              # a stop's fence still counts (its stop may be waiting for it)

    def _safe_unpack(self, raw):
        try:
            return unpack_batch(raw)
        except Exception as e:  # malformed batch — drop it, don't crash the RT loop
            logger.error(f"Dropped malformed trajectory batch: {e}")
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
