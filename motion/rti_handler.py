"""RTIModeHandler — Layer 2 motion generator for RTI (real-time interface) mode.

An external PC streams setpoints over UDP at the loop rate; this handler turns the
newest packet into a joint-space ControlOutput. See TODO/RTI_mode_plan.md.

Two lifecycles, deliberately kept separate — conflating them was the 2026-08-01 bug:

  enter(ctx) / exit(ctx)
      The MODE-dispatch hooks. RTI.py calls these on every RobotMode transition,
      with the TickContext, exactly as it does for ExecHandler. They must accept ctx.

  connect_pc() / claim_control() / request_release() / disconnect()
      The RTI PAIRING lifecycle, driven by TCP commands from the PC. These carry the
      PC address and control claim, and are called from RTI.py's ZMQ command handlers,
      never by dispatch.

Sub-states (handler-local, not in the global state machine — see plan §3):

    disconnected    no PC paired. Holds position.
    connected       paired; state streams to the PC; NO timing rules, so an idle PC
                    is not an error. Holds position.
    control_active  PC claimed control; timeout + success-rate tracking enforced.
    stopping_clean  PC ended motion (UDP CMD_STOP or TCP release). Holds 0.5 s, then
                    → connected. No error latched.
    stopping_error  timeout / packet loss. Holds 0.5 s, then → connected, error latched.

Hot-path discipline (CLAUDE.md): step() allocates nothing. Every buffer is
preallocated in __init__ and written with `dest[:] = src`; one ControlOutput
instance is reused and its channel fields are re-pointed per tick.
"""

import logging

import numpy as np

from communication.rti_protocol import (
    CMD_JOINT_POSITION, CMD_JOINT_VELOCITY, CMD_JOINT_TORQUE,
    CMD_CARTESIAN_POSE, CMD_CARTESIAN_VELOCITY, CMD_STOP,
    CLOCK_ROBOT, CLOCK_PC, unpack_command_packet,
)
from data.data_recipe import build_payload
from utility.signal_filters import create_filter_bank, apply_filter_bank
from utility.tick_types import TickResult, ControlOutput

# Ruckig does the position rate limiting (plan §11b). Imported defensively because the
# handler must still load — and still clamp to the soft joint limits — on a machine where
# it is missing; see _limit_position for what happens then.
try:
    from ruckig import (Ruckig, InputParameter, OutputParameter,
                        ControlInterface, Synchronization)
    _RUCKIG_AVAILABLE = True
except ImportError:                                  # pragma: no cover - env dependent
    _RUCKIG_AVAILABLE = False

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------- #
# COMMAND LATENCY — why it is one tick, and what the clock mode buys you.
# Full derivation in TODO/RTI_mode_plan.md §1.1-1.2; the short version, because it
# determines how the ordering in step() must not be changed:
#
# RTI.py drains UDP near the top of the tick, then calls dispatch. Inside step() the
# state packet goes out BEFORE commands are read. So in robot-clocked mode the PC is
# replying to a packet sent after this tick's drain — its reply cannot be read until the
# NEXT tick's drain. One tick of command latency is therefore structural (~4 ms at 250 Hz),
# not a tuning problem, and no amount of re-ordering inside this file removes it.
#
# The seq ECHO costs a second tick (~8 ms round trip) because _send_state() runs before
# _receive_command(). That is deliberate: sending state as early in the tick as possible
# gives the PC the largest window to reply before the next drain. Flipping the order would
# surface the echo a tick sooner while SHRINKING that window — trading real control latency
# for prettier diagnostics. Do not flip it.
#
# CLOCK_PC would actually have LOWER average command latency (~2 ms): the PC sends at
# arbitrary phase rather than always landing just after a drain, so it waits half a tick on
# average instead of a full one. The cost is synchronisation — the robot may see two
# commands in one tick (drain_rti_udp keeps only the newest) or none, which is also why the
# success-rate warnings below are gated to CLOCK_ROBOT. RTI.py currently hardcodes
# CLOCK_ROBOT; this is the lever to reach for if command latency ever binds.
# --------------------------------------------------------------------------- #

# rti['rti_substate'] codes — mirrored to shared memory every tick for the GUI.
SUBSTATE_CODE = {
    None:             0.0,
    'connected':      1.0,
    'control_active': 2.0,
    'stopping_clean': 3.0,
    'stopping_error': 4.0,
}

# Success-rate warning thresholds (plan §3). Logged at most once per crossing.
#
# The rate is over a MOVING WINDOW of the last SUCCESS_WINDOW_TICKS ticks, not a
# running total since the claim. A cumulative average is dominated by ancient history:
# a link that dropped 40% for the first second and has been perfect for a minute since
# still reads as degraded, and one that has just fallen over still reads as fine.
#
# 100 samples is chosen for RESOLUTION, not duration: it gives the ratio 1% granularity,
# which is what makes the warning thresholds below meaningful. At PAR6's ~250 Hz that
# window spans ~400 ms.
SUCCESS_WINDOW_TICKS = 100
SUCCESS_WARN = 0.95
SUCCESS_BAD = 0.90

# IMPORTANT — the ratio is commands-per-TICK, so it only means "packet loss" when the PC
# is expected to send once per robot tick. That is true in ROBOT-clocked mode and false in
# PC-clocked mode, where a healthy 100 Hz PC against our 250 Hz loop legitimately scores
# 0.4. The ratio warnings are therefore gated on CLOCK_ROBOT. The metric that means the
# same thing in BOTH modes is consecutive misses, below — that is what actually indicates
# a broken link, and it is the right thing to build a hard limit on.
# How long control may go without a command before the robot stops itself.
#
# 40 ms (2026-08-01, tightened from the plan §3 value of 100 ms). 100 ms is a long time
# for an arm to keep applying a stale setpoint while an external PC is closing the loop.
#
# Why 40 ms and not less: at 250 Hz a tick is 4 ms, so a 20 ms timeout is only 5 ticks.
# The PC running the control loop is typically a non-RT Linux box where a single scheduling
# hiccup costs several milliseconds, and 5 ticks of tolerance would make a routine hiccup
# indistinguishable from a dead link. 40 ms = 10 ticks keeps the wall-clock reaction tight
# while leaving room for the early warning below to actually be early.
COMMAND_TIMEOUT_S = 0.040
STOPPING_HOLD_S = 0.500     # plan §3: hold position this long after any stop

# Command low-pass (plan §11a). Standard one-pole IIR:
# gain = dt/(dt + 1/(2*pi*fc)); out = gain*y + (1-gain)*y_last — i.e. this codebase's
# "ema" with tau = 1/(2*pi*fc).
#
# CUTOFF: 25 Hz at our ~250 Hz loop — 10% of the control rate, and 20% of Nyquist.
# (§11a's suggested 30 Hz assumed a 100 Hz loop.)
#
# DEFAULT: **OFF.** This is a deliberate departure from the usual convention of enabling
# such a filter by default.
#
# Why: the filter's phase lag is set by tau = 1/(2*pi*fc), so a cutoff at 10% of the loop
# rate costs tau 6.4 ms here, against ~1.6 ms on a 1 kHz controller. That is inherent to
# running 4x slower, not a tuning choice. In RTI mode an external PC is closing the loop,
# and 6.4 ms of added lag is a real cost to hand it by
# default. So: opt-in, with the two features that actually prevent damage (rate limiter,
# start-pose check) left ON. Revisit if a real command stream turns out to need it.
DEFAULT_LOWPASS_CUTOFF_HZ = 25.0

# Bandwidth is accumulated per tick and converted to a rate once per window. One second
# matches the GUI's refresh, so the displayed number is a real second of traffic rather
# than a single tick scaled up (which would swing wildly with packet-size jitter).
BANDWIDTH_WINDOW_S = 1.0

# Half the uint32 sequence-number range. A backwards jump larger than this is a wrap, not
# a stale packet — see the wrap guard in _receive_command(). RFC 1982 serial arithmetic.
SEQ_WRAP_HALF = 0x80000000

# Start-pose check (plan §16). How far the PC's FIRST position command may sit from where
# the arm actually is before control is refused. 0.1 rad ~= 5.7 deg: comfortably more than
# any tracking error, far less than the "controller initialised to zeros" case this exists
# to catch. Velocity/torque commands get the analogous "must start near rest" check.
DEFAULT_START_POSE_TOL_RAD = 0.1
START_VEL_TOL_FRACTION = 0.10    # first velocity command <= 10% of the joint velocity limit
START_TRQ_TOL_FRACTION = 0.25    # first torque command <= 25% of one tick of torque-rate budget


class RTIModeHandler:
    """RTI mode handler. One instance, created once before the RT loop."""

    def __init__(self, joint_num, clock_mode=CLOCK_ROBOT, delta_t=0.01,
                 vel_limit=None, accel_limit=None, jerk_limit=None, torque_rate_limit=None,
                 limit_neg=None, limit_pos=None,
                 start_pose_tol_rad=DEFAULT_START_POSE_TOL_RAD,
                 command_timeout_s=COMMAND_TIMEOUT_S,
                 stopping_hold_s=STOPPING_HOLD_S,
                 lowpass_cutoff_hz=0.0,
                 success_window_ticks=SUCCESS_WINDOW_TICKS,
                 success_warn=SUCCESS_WARN,
                 success_bad=SUCCESS_BAD):
        self.joint_num = joint_num
        self.clock_mode = clock_mode
        self.delta_t = delta_t

        self.sub_state = None          # see module docstring
        self.pc_addr = None            # (ip, port) — set by connect_pc() from the TCP handshake

        # --- timing: ticks derived from the REAL loop period, never a hardcoded rate ---
        # Policy values come from <rti_defaults> in config/system.xml via RTI.py; the
        # module constants above are the fallback when that block is absent, so there is
        # still exactly one definition of each default.
        self.timeout_ticks = max(1, int(round(command_timeout_s / delta_t)))
        self.stopping_ticks = max(1, int(round(stopping_hold_s / delta_t)))
        self.success_window_ticks = int(success_window_ticks)
        self.success_warn = float(success_warn)
        self.success_bad = float(success_bad)
        self.stopping_counter = 0
        # Warn at half the budget so the warning is genuinely early. Derived rather than
        # fixed: a constant would silently become "warn exactly when we stop" if the
        # timeout were ever shortened.
        self.miss_warn_ticks = max(2, self.timeout_ticks // 2)

        # --- command state ---
        self.last_cmd_tick = 0
        self.last_cmd_type = None
        self.seq_rx_last = 0
        self.seq_rx_gaps = 0           # count of non-consecutive sequence numbers

        # --- success rate: moving window over the last SUCCESS_WINDOW_TICKS ---
        self.control_ticks = 0         # ticks scored since the claim (window fill gauge)
        self.commands_received = 0     # total accepted since the claim (diagnostics only)
        self.success_rate = 1.0
        self._warned_at = 0            # 0 = none, 1 = warned <0.95, 2 = warned <0.90
        self._sr_window = np.zeros(self.success_window_ticks, dtype=np.float64)
        self._sr_idx = 0
        self._sr_filled = 0
        self._sr_sum = 0.0
        self.consecutive_misses = 0
        self.worst_consecutive_misses = 0
        self._miss_warned = False
        # Raised for one tick when the command watchdog trips. RTI.py forwards it to
        # process_errors_tick(), which is the ONLY thing that may set rti['error_active'] —
        # it recomputes that flag from its own error set every tick, so writing the flag
        # here directly did nothing at all (found on hardware 2026-08-02).
        self.link_lost_event = False

        # --- preallocated buffers (hot path allocates nothing) ---
        self._zeros = np.zeros(joint_num, dtype=np.float64)
        self._payload = np.zeros(joint_num, dtype=np.float64)
        self._feedforward = np.zeros(joint_num, dtype=np.float64)
        self._hold_pos = np.zeros(joint_num, dtype=np.float64)
        self._out = ControlOutput()

        # Scratch for the 6-wide wire packet before it is narrowed to joint_num.
        self._rx_payload = np.zeros(6, dtype=np.float64)
        self._rx_feedforward = np.zeros(6, dtype=np.float64)

        # ------------------------------------------------------------------ #
        # Phase 2 safety layer (plan §11b / §16). Both features are independently
        # switchable at runtime and default ON — a hardware bring-up turns them off
        # deliberately, never by forgetting to turn them on.
        # ------------------------------------------------------------------ #
        self.rate_limit_enabled = True
        self.start_pose_check_enabled = True
        self.start_pose_tol_rad = float(start_pose_tol_rad)

        # Gravity compensation. RTI was the ONLY mode that never applied it — IDLE
        # (mode_dispatch.py), JOG (jog.py) and EXEC (exec_handler.py) all read the same
        # robot-wide rti['gravity_comp_enabled'] flag, and RTI simply ignored it. Worse, the
        # comment in _command_output claimed dispatch added it downstream, which it does not:
        # dispatch only converts trq_nm, so trq_nm=None became Current=0 on the wire and the
        # arm carried its own weight on firmware PID error alone.
        #
        # TWO switches, both must be on. The robot-wide flag is the operator's (GUI, and it
        # governs every mode); this one is RTI-scoped, so a PC computing its own gravity in a
        # torque controller can turn it off for the duration of its session without changing
        # what IDLE or JOG do when it lets go.
        self.gravity_comp_enabled = True
        self._trq = np.zeros(joint_num, dtype=np.float64)   # command + gravity, preallocated

        def _limit(arr, fallback):
            """Per-joint limit array; +inf where unknown, so a missing limit is a
            no-op comparison rather than a NaN that silently disables clamping."""
            if arr is None:
                return np.full(joint_num, fallback, dtype=np.float64)
            out = np.asarray(arr, dtype=np.float64)[:joint_num].copy()
            return np.where(np.isnan(out), fallback, out)

        self.vel_limit = _limit(vel_limit, np.inf)
        self.accel_limit = _limit(accel_limit, np.inf)
        self.jerk_limit = _limit(jerk_limit, np.inf)
        self.torque_rate_limit = _limit(torque_rate_limit, np.inf)
        # SOFTWARE joint limits — the unconditional backstop. Independent of the rate
        # limiter toggle on purpose: turning the limiter off must never make it possible
        # to drive past a soft limit. +/-inf when unknown, so a robot XML without them
        # behaves exactly as before rather than clamping to zero.
        self.limit_neg = _limit(limit_neg, -np.inf)
        self.limit_pos = _limit(limit_pos, np.inf)

        # Rate-limiter state: the last position/velocity/acceleration/torque the limiter
        # actually emitted. Seeded from the measured arm on claim_control() so the first
        # limited command is relative to where the arm really is, not to a stale value.
        self._rl_q = np.zeros(joint_num, dtype=np.float64)
        self._rl_v = np.zeros(joint_num, dtype=np.float64)
        self._rl_a = np.zeros(joint_num, dtype=np.float64)
        self._rl_t = np.zeros(joint_num, dtype=np.float64)
        # Limiter scratch (hot path allocates nothing).
        self._rl_tmp_v = np.zeros(joint_num, dtype=np.float64)
        self._rl_tmp_a = np.zeros(joint_num, dtype=np.float64)
        self._rl_tmp_j = np.zeros(joint_num, dtype=np.float64)
        self._rl_out = np.zeros(joint_num, dtype=np.float64)
        self._rl_tgt = np.zeros(joint_num, dtype=np.float64)   # limit-clamped target
        self._rl_dist = np.zeros(joint_num, dtype=np.float64)  # distance to target
        self._rl_vstop = np.zeros(joint_num, dtype=np.float64) # max stoppable speed
        self._start_pose_checked = False

        # ------------------------------------------------------------------ #
        # Braking bound for VELOCITY and TORQUE commands (open defect A).
        #
        # Position commands were clamped to the soft joint limits from the start; velocity
        # and torque commands were not bounded by position AT ALL. `_limit_velocity` clipped
        # v/a/j and never looked at where the joint was, so a sustained velocity command drove
        # straight through a soft limit — measured 189 deg past it. Nothing downstream caught
        # it either: `run_safety_checks()` is never called and there is no JOINT_LIMIT error.
        #
        # WHY sqrt-style braking IS correct here, having failed for position targets. The
        # position limiter limit-cycled because it re-planned to stop ON a target that MOVED
        # every tick. A soft joint limit is a STATIONARY barrier, and the requirement is only
        # "do not cross it" — undershooting is free, so there is no cycle to enter. Same
        # formula, genuinely different problem.
        #
        # Jerk-aware form (derived and validated offline 2026-08-02):
        #     v_stop(d) = a * (sqrt((a/j)^2 + 2d/a) - a/j)
        # The (a/j) terms account for deceleration not being available instantly, which is
        # exactly what plain sqrt(2ad) ignored.
        #
        # Precomputed with SAFE substitutions so no inf*0 can appear — that is the arithmetic
        # behind open defect C, and it must not be reintroduced here.
        # Acceleration alone, for the LEGACY cascade's stopping-speed term. Kept separate
        # from _brake_ok below, which additionally needs finite jerk and joint limits: a
        # robot with a real acceleration limit but no declared joint limits should still get
        # a correct vstop, and folding the two masks together would silently compute it from
        # the 1.0 placeholder instead.
        self._accel_finite = np.isfinite(self.accel_limit) & (self.accel_limit > 0)
        self._accel_safe = np.where(self._accel_finite, self.accel_limit, 1.0)

        _brake_ok = (np.isfinite(self.accel_limit) & np.isfinite(self.jerk_limit)
                     & np.isfinite(self.limit_neg) & np.isfinite(self.limit_pos)
                     & (self.accel_limit > 0) & (self.jerk_limit > 0))
        self._brake_ok = _brake_ok
        # Substitute BEFORE the arithmetic, not after. `np.where(ok, a/j, 0)` still evaluates
        # a/j for every joint, so an absent limit computes inf/inf = NaN and only then throws
        # it away — the right answer via a NaN the interpreter warns about, which is precisely
        # the shape of the bug being fixed a few lines below. Feed the operators finite
        # numbers and there is nothing to discard.
        self._brake_a = np.where(_brake_ok, self.accel_limit, 1.0)
        self._brake_r = (np.where(_brake_ok, self.accel_limit, 0.0)
                         / np.where(_brake_ok, self.jerk_limit, 1.0))
        self._brake_r2 = self._brake_r * self._brake_r
        self._bl_dup = np.zeros(joint_num, dtype=np.float64)   # distance to upper limit
        self._bl_ddn = np.zeros(joint_num, dtype=np.float64)   # distance to lower limit
        self._bl_vup = np.zeros(joint_num, dtype=np.float64)   # max speed toward upper
        self._bl_vdn = np.zeros(joint_num, dtype=np.float64)   # max speed toward lower
        self._bl_mask = np.zeros(joint_num, dtype=bool)        # scratch for torque blocking

        # ------------------------------------------------------------------ #
        # Ruckig — the POSITION rate limiter (plan §11b).
        #
        # It replaced a hand-rolled cascade that capped velocity at sqrt(2*a*|d|).
        # That cap never converged: as d -> 0 it forces v -> 0, the jerk limit stops the
        # acceleration reversing fast enough, the joint overshoots, d changes sign, and the
        # cycle repeats indefinitely. Measured on hardware 2026-08-02 and reproduced
        # offline: a SUSTAINED +/-0.010 rad (0.6 deg) oscillation on a CONSTANT target,
        # still going after 10 s, for any non-zero step however small. It reached the
        # motors — motor_position_sent oscillated with it.
        #
        # Ruckig converges exactly (0.000000 deg steady-state band, every joint, every step
        # size) because it solves the jerk-limited profile instead of clamping derivatives
        # and hoping. Measured ~3.5 us median / ~32 us p99 for 6 DoF on this Pi.
        #
        # Synchronization.No is load-bearing: a rate limiter must treat joints
        # independently. Phase-synchronised output would slow every joint to the slowest,
        # which is right for a coordinated move and wrong for a safety filter.
        #
        # Ruckig's own min_position/max_position are deliberately NOT used. The soft-limit
        # clamp in _limit_position is unconditional — it must hold when the rate limiter is
        # switched OFF too — so it cannot live inside the limiter, and having it in both
        # places would add a hot-path error return (ErrorPositionalLimits) for a bound that
        # is already enforced.
        self._rk = None
        self._rk_errors = 0
        self._rk_tv = np.zeros(joint_num, dtype=np.float64)   # clamped feedforward
        if not _RUCKIG_AVAILABLE:
            logger.error("[RTI] ruckig is not installed — the position rate limiter falls "
                         "back to the legacy cascade, which does NOT converge on a "
                         "constant target. Soft joint limits are still enforced.")
        elif not (np.all(np.isfinite(self.vel_limit)) and
                  np.all(np.isfinite(self.accel_limit)) and
                  np.all(np.isfinite(self.jerk_limit))):
            # Ruckig rejects a non-finite limit. Rather than substitute an invented
            # ceiling, say so and let the cascade handle it — an XML missing a limit is a
            # config problem to fix, not one to paper over with a made-up number.
            logger.error("[RTI] velocity/acceleration/jerk limits are not all finite in "
                         "the robot XML — Ruckig disabled, falling back to the legacy "
                         "cascade. Fix the XML rather than relying on this path.")
        else:
            self._rk = Ruckig(joint_num, delta_t)
            self._rk_in = InputParameter(joint_num)
            self._rk_out = OutputParameter(joint_num)
            self._rk_in.control_interface = ControlInterface.Position
            self._rk_in.synchronization = Synchronization.No
            self._rk_in.max_velocity = self.vel_limit
            self._rk_in.max_acceleration = self.accel_limit
            self._rk_in.max_jerk = self.jerk_limit
            self._rk_in.target_acceleration = self._zeros

        # Command low-pass (§11a). None = disabled. Built from the shared filter engine
        # (utility/signal_filters.py) rather than a hand-rolled IIR, but deliberately NOT
        # wired into config/filters.xml: that file configures MEASUREMENT signals and is
        # hot-reloaded from disk, whereas this is a command-path setting the PC sets per
        # session over TCP (§11c). Two sources of truth for one cutoff is a bug waiting to
        # happen, so this bank is owned here.
        self._lp_bank = None
        self._lp_cutoff_hz = 0.0
        self._lp_out = np.zeros(joint_num, dtype=np.float64)
        if lowpass_cutoff_hz:
            self.set_lowpass_filter(lowpass_cutoff_hz)

        # --- link bandwidth, both directions ---
        # NOTE ON RX ACCURACY: this counts the bytes the handler actually CONSUMES, which
        # is at most one command packet per tick — drain_rti_udp keeps only the newest and
        # silently discards the rest. In robot-clocked mode that is 1:1 with the wire and
        # the figure is exact. In PC-clocked mode a PC sending faster than the tick rate
        # will put more on the wire than this reports. TX is always exact.
        self._bw_window_ticks = max(1, int(round(BANDWIDTH_WINDOW_S / delta_t)))
        self._bw_ticks = 0
        self._tx_acc = 0
        self._rx_acc = 0
        self.tx_bytes_per_s = 0.0
        self.rx_bytes_per_s = 0.0
        # Over-rate accounting. Only the newest command per tick can be applied, so a PC
        # sending faster than the loop ticks has the remainder thrown away — and nothing
        # else shows it: success rate reads 100% (a command DID arrive) and misses read 0.
        # Without this a PC-clocked controller can lose half its output silently.
        self._rx_pkts = 0          # packets pulled off the socket this window
        self._rx_applied = 0       # of those, ones that became a setpoint
        self.discard_pct = 0.0     # % of received command packets thrown away

    # ------------------------------------------------------------------ #
    # Mode-dispatch hooks — called by RTI.py on RobotMode transitions.
    # Signatures must match ExecHandler: enter(ctx, **kwargs) / exit(ctx).
    # ------------------------------------------------------------------ #

    def enter(self, ctx, **kwargs):
        """Entering RobotMode.RTI.

        Does NOT pair with a PC — pairing is connect_pc(), driven by TCP. Normally
        connect_pc() has already run (rti_connect is what requests this mode), so this
        only seeds the hold position and publishes state.
        """
        self._hold_pos[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
        self.last_cmd_tick = ctx.current_tick
        if self.sub_state is None:
            # Mode entered without a TCP handshake (e.g. operator set_mode:RTI).
            # Legal: we stream nothing and hold until a PC connects.
            logger.info("[RTI] Mode entered with no PC paired — holding until rti_connect")
        self._publish(ctx)

    def exit(self, ctx):
        """Leaving RobotMode.RTI. Drops the claim but keeps the PC pairing.

        Control must never survive a mode change: coming back into RTI has to go
        through rti_claim again, so a stale claim cannot silently re-arm timing rules.
        """
        if self.sub_state in ('control_active', 'stopping_clean', 'stopping_error'):
            logger.info("[RTI] Left RTI mode while control was claimed — claim dropped")
            self.sub_state = 'connected'
        self.stopping_counter = 0
        self._publish(ctx)

    # ------------------------------------------------------------------ #
    # RTI pairing lifecycle — called from RTI.py's TCP/ZMQ command handlers.
    # ------------------------------------------------------------------ #

    def connect_pc(self, pc_addr, current_tick):
        """TCP rti_connect: pair with a PC and start unicast state streaming."""
        self.sub_state = 'connected'
        self.pc_addr = pc_addr
        self.seq_rx_last = 0
        self.seq_rx_gaps = 0
        self.last_cmd_tick = current_tick
        self._reset_bandwidth()   # a new session's traffic, not the previous one's
        logger.info("[RTI] Paired with %s:%s — streaming state", pc_addr[0], pc_addr[1])

    def take_link_lost(self):
        """One-shot read of the watchdog flag, for RTI.py to feed process_errors_tick()."""
        ev = self.link_lost_event
        self.link_lost_event = False
        return ev

    def claim_control(self, current_tick, measured_pos=None):
        """TCP rti_claim: connected → control_active. Timing rules start now.

        measured_pos seeds the rate limiter so the first limited command is relative to
        where the arm actually is. Without it the limiter would ramp from 0 rad and yank
        the arm toward the origin — the exact failure the limiter exists to prevent.
        """
        self.sub_state = 'control_active'
        self.last_cmd_tick = current_tick
        self.last_cmd_type = None
        self._payload[:] = 0.0
        self._feedforward[:] = 0.0
        self.control_ticks = 0
        self.commands_received = 0
        self.success_rate = 1.0
        self._warned_at = 0
        self._sr_window[:] = 0.0
        self._sr_idx = 0
        self._sr_filled = 0
        self._sr_sum = 0.0
        self.consecutive_misses = 0
        self.worst_consecutive_misses = 0
        self._miss_warned = False

        if measured_pos is not None:
            self._rl_q[:] = np.asarray(measured_pos, dtype=np.float64)[:self.joint_num]
        self._rl_v[:] = 0.0
        self._rl_a[:] = 0.0
        self._rl_t[:] = 0.0
        self._start_pose_checked = False

        # Rebuild the low-pass so it seeds from THIS session's first command. The engine
        # seeds an ema from its first sample rather than ramping from zero, but that only
        # helps if the state is actually fresh — carrying the previous session's last
        # output into a new claim would drag the first setpoint toward a stale pose.
        if self._lp_bank is not None:
            self.set_lowpass_filter(self._lp_cutoff_hz)

        logger.info("[RTI] Control claimed — timeout %d ticks, rate limiter %s, start-pose check %s",
                    self.timeout_ticks,
                    "ON" if self.rate_limit_enabled else "OFF",
                    "ON" if self.start_pose_check_enabled else "OFF")

    def request_release(self):
        """TCP rti_release: begin a CLEAN stop. Returns True if it was accepted."""
        if self.sub_state != 'control_active':
            return False
        self._begin_stop('stopping_clean')
        logger.info("[RTI] Clean release requested over TCP")
        return True

    def disconnect(self):
        """TCP rti_disconnect: drop the pairing entirely."""
        self.sub_state = None
        self.pc_addr = None
        self.stopping_counter = 0
        logger.info("[RTI] Unpaired")

    # ------------------------------------------------------------------ #
    # Per-tick step
    # ------------------------------------------------------------------ #

    def step(self, ctx):
        """One RT tick. Returns (TickResult, ControlOutput) for dispatch_step."""
        # Count every received byte here, not inside _receive_command, so the figure stays
        # honest in EVERY sub-state. A packet that is malformed, stale, or arriving while
        # we are already stopping still consumed link bandwidth — and those are exactly the
        # situations where you most want to see traffic. Counting it deeper would make a
        # PC flooding garbage appear to go silent the moment its garbage tripped the timeout.
        n = getattr(ctx, "rti_udp_count", 0) or 0
        if ctx.rti_udp_data is not None:
            # Every command packet is the same 101 bytes, so counting the survivor's length
            # n times is exact — and it closes the RX-bandwidth undercount that existed
            # while the drain discarded packets without telling anyone.
            self._rx_acc += len(ctx.rti_udp_data) * max(1, n)
            self._rx_pkts += max(1, n)
        elif n:
            self._rx_pkts += n     # all rejected (wrong sender / truncated) but still traffic

        # Stream state to the paired PC in every sub-state except unpaired.
        if self.pc_addr is not None:
            self._send_state(ctx)

        if self.sub_state == 'control_active':
            out = self._step_active(ctx)
        elif self.sub_state in ('stopping_clean', 'stopping_error'):
            out = self._step_stopping(ctx)
        else:
            # None (unpaired) or 'connected' — no timing rules, just hold.
            out = self._hold(ctx)

        # Publish AFTER the transitions above, so shared memory shows this tick's
        # outcome. Publishing first made rti_substate lag a tick, which hides a
        # single-tick stop from anything sampling it.
        self._update_bandwidth()
        self._publish(ctx)
        return TickResult(), out

    def _step_active(self, ctx):
        got = self._receive_command(ctx)

        if got:
            if self.last_cmd_type == CMD_STOP:
                self._begin_stop('stopping_clean')
                logger.info("[RTI] Motion finished by PC")
                return self._hold(ctx)

            # Validate the FIRST command of this claim before it can reach a motor.
            if not self._start_pose_checked:
                self._start_pose_checked = True
                if self.start_pose_check_enabled and not self._start_pose_ok(ctx):
                    self._begin_stop('stopping_error')
                    self.link_lost_event = True
                    return self._hold(ctx)

        # Only score ticks from the first accepted command onward. Counting the
        # gap between claim_control() and the PC's first packet would report 0%
        # and fire the "control unreliable" warning before control has started.
        if self.last_cmd_type is not None:
            self.control_ticks += 1
            if got:
                self.commands_received += 1
            self._update_success_rate(got)

        if not got and (ctx.current_tick - self.last_cmd_tick) > self.timeout_ticks:
            self._begin_stop('stopping_error')
            logger.warning(
                "[RTI] No command for %d ticks (limit %d) — stopping, latching RTI_LINK_LOST",
                ctx.current_tick - self.last_cmd_tick, self.timeout_ticks)
            self.link_lost_event = True
            return self._hold(ctx)

        if self.last_cmd_type is None:
            # Claimed but the first packet has not arrived yet — hold, don't time out
            # until the window elapses (handled above).
            return self._hold(ctx)

        return self._command_output(ctx)

    def _step_stopping(self, ctx):
        self.stopping_counter -= 1
        if self.stopping_counter <= 0:
            was = self.sub_state
            self.sub_state = 'connected'
            if was == 'stopping_clean':
                logger.info("[RTI] Clean stop complete — back to connected")
            else:
                logger.warning("[RTI] Error stop complete — back to connected, error latched")
        return self._hold(ctx)

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _begin_stop(self, which):
        self.sub_state = which
        self.stopping_counter = self.stopping_ticks

    def _hold(self, ctx):
        """Hold the current measured position with zero velocity.

        Position hold, not zero torque — plan §8.1: dropping to zero torque would let the arm
        fall. It holds because the firmware PID tracks the position setpoint, so gravity
        feedforward is not what keeps it up; adding it just means the PID starts from the
        right torque instead of having to build it out of tracking error. Same as EXEC's hold.

        (Until 2026-08-06 this said gravity was "added downstream by dispatch". It was not —
        dispatch only converts trq_nm, so None became Current=0 on the wire.)
        """
        self._hold_pos[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
        out = self._out
        out.pos_rad = self._hold_pos
        out.vel_rad = self._zeros
        out.acc_rad = None
        out.trq_nm = self._add_gravity(ctx, None)
        out.mode = "pid"
        out.target_pos_rad = None
        out.target_vel_rad = None
        out.target_acc_rad = None
        out.target_trq_nm = None
        return out

    def _command_output(self, ctx):
        """Map the latest command onto ControlOutput (plan §7 motor-command mapping).

        gravity comp is NOT added here — dispatch/_to_motor_space owns that, so the
        PC never sends it (plan §2.3).
        """
        out = self._out
        out.mode = "pid"
        out.acc_rad = None
        out.target_acc_rad = None
        ct = self.last_cmd_type

        # Low-pass first, then rate-limit: the filter shapes the request, the limiter
        # enforces what the arm can physically do. Doing it the other way round would let
        # the filter smear a limiter clamp back out past the limit. Only the primary
        # payload is filtered — feedforward terms are already the PC's own derivatives and
        # filtering them would add lag to the term whose whole job is to remove lag.
        # NOTE: reads self._payload, never writes it, so _target below stays the raw request.
        payload = self._apply_lowpass(self._payload)

        # MEASURED position, for the soft-limit guard on the velocity and torque paths. It has
        # to be the encoder: those modes never command a position, so the limiter has no
        # integrated position of its own that means anything. A view, not a copy — read-only.
        q_meas = ctx.shared_data.joint['joint_position'][:self.joint_num]

        # Gravity is added to whatever torque the command carries — none at all for a position
        # command — so `commanded = target + gravity_comp` holds here the same way it does in
        # EXEC (CLAUDE.md §17.1). target_* below stays the PC's raw request, pre-gravity, which
        # is what makes the compensation visible in telemetry rather than hidden inside it.
        if ct == CMD_JOINT_POSITION:
            out.pos_rad = self._limit_position(payload)
            out.vel_rad = self._feedforward
            out.trq_nm = self._add_gravity(ctx, None)
        elif ct == CMD_JOINT_VELOCITY:
            out.pos_rad = None
            out.vel_rad = self._limit_velocity(payload, q_meas)
            out.trq_nm = self._add_gravity(ctx, self._limit_torque(self._feedforward, q_meas))
        elif ct == CMD_JOINT_TORQUE:
            out.pos_rad = None
            out.vel_rad = None
            out.trq_nm = self._add_gravity(ctx, self._limit_torque(payload, q_meas))
        else:
            # CARTESIAN_* need IK/Jacobian in the RT loop — plan §9 defers them.
            # Refuse rather than approximate: hold position.
            return self._hold(ctx)

        # _target is the PC's RAW request; _commanded above is what survived the rate
        # limiter. Keeping them distinct is the whole point of the naming convention
        # (docs/commanded_target_actual.md) and it is what makes limiter activity
        # visible in telemetry: target != commanded means the limiter is working.
        if ct == CMD_JOINT_POSITION:
            out.target_pos_rad = self._payload
            out.target_vel_rad = self._feedforward
            out.target_trq_nm = None
        elif ct == CMD_JOINT_VELOCITY:
            out.target_pos_rad = None
            out.target_vel_rad = self._payload
            out.target_trq_nm = self._feedforward
        else:   # CMD_JOINT_TORQUE
            out.target_pos_rad = None
            out.target_vel_rad = None
            out.target_trq_nm = self._payload
        return out

    # ------------------------------------------------------------------ #
    # Phase 2: rate limiting (plan §11b) and start-pose validation (§16)
    # ------------------------------------------------------------------ #

    def _limit_position(self, q_des):
        """Clamp a position command so velocity, acceleration and jerk stay in budget.

        Ruckig solves the jerk-limited profile to the target each tick (plan §11b). The
        legacy cascade below it is the fallback for a machine without Ruckig — see the
        note in __init__ for why it is a fallback and not the primary path.

        TWO SAFETY LAYERS, both unconditional:

        1. SOFTWARE JOINT LIMITS. The target is clamped on the way in and the output on the
           way out. Applied even when the rate limiter is switched off — disabling a
           smoothing feature must never unlock the workspace. Clamping the TARGET (not just
           the output) is what stops the integrator winding up against an unreachable goal:
           clamp only the output and velocity stays high while position is pinned, so the
           moment the target came back in range it would jump.

        2. VELOCITY / ACCELERATION / JERK. Ruckig plans a profile that respects all three
           and ARRIVES — which is the part the old cascade could not do. It capped velocity
           at sqrt(2*a*|d|), the speed you can stop from at constant deceleration; the jerk
           limit means deceleration is not available instantly, so it consistently arrived
           too fast, overshot, and limit-cycled forever at +/-0.6 deg.

           That cap was once documented as a standard rate-limiting approach. It is not:
           a rate limiter tapers acceleration toward the VELOCITY limit and has no
           target-distance term at all. Conflating a rate limiter with a target-seeker is
           what produced the limit cycle. Corrected 2026-08-02.

        FEEDFORWARD. The PC's feedforward velocity becomes Ruckig's target_velocity, so the
        planner aims to PASS THROUGH the streamed setpoint at the right speed rather than
        come to rest on it. A follower that plans to stop on every setpoint trails a moving
        command by its whole stopping distance; measured on a 0.5 rad/s ramp, feeding
        feedforward cut tracking error from 1.41 deg to 0.48 deg. A PC that sends zeros
        (the common case, and every hold-position command) is unaffected.

        Rate limiting a discontinuous signal can itself destabilise a PC-side loop — this
        is a safety net, not a licence to stream garbage.
        """
        # Layer 1a: a target outside the soft limits is not a target.
        tgt = self._rl_tgt
        np.clip(q_des, self.limit_neg, self.limit_pos, out=tgt)

        if not self.rate_limit_enabled:
            # Still honour the joint limits — see the docstring. Keep the limiter state in
            # step so toggling it back on mid-run does not produce a step. Velocity and
            # acceleration are zeroed rather than left stale: re-enabling mid-stream then
            # plans from "at rest", which ramps, where a stale non-zero velocity would let
            # the first limited tick jump.
            self._rl_q[:] = tgt
            self._rl_v[:] = 0.0
            self._rl_a[:] = 0.0
            return tgt

        if self._rk is not None:
            return self._limit_position_ruckig(tgt)

        dt = self.delta_t
        v, a, j, out = self._rl_tmp_v, self._rl_tmp_a, self._rl_tmp_j, self._rl_out
        d, vstop = self._rl_dist, self._rl_vstop

        np.subtract(tgt, self._rl_q, out=d)
        np.divide(d, dt, out=v)
        np.clip(v, -self.vel_limit, self.vel_limit, out=v)

        # Layer 2: never travel faster than we can still stop from.
        #
        # _accel_safe, not accel_limit: with an absent (infinite) acceleration limit this
        # computed 2*|d| * inf, which is NaN for any joint sitting exactly ON its target —
        # i.e. HOLD POSITION, the commonest RTI operation there is. The NaN propagated
        # through the clips into the emitted setpoint and on to int() in the motor-space
        # conversion. The two conditions coincide by construction, because a missing limit is
        # exactly what disables Ruckig and drops control onto this fallback in the first
        # place: the safety net failed unsafely, and only in the case it was needed.
        # Joints with no acceleration limit get vstop = inf, which is the honest answer —
        # no limit means no stopping-distance constraint to enforce.
        np.abs(d, out=vstop)
        np.multiply(vstop, 2.0, out=vstop)
        np.multiply(vstop, self._accel_safe, out=vstop)
        np.sqrt(vstop, out=vstop)
        np.copyto(vstop, np.inf, where=~self._accel_finite)
        np.clip(v, -vstop, vstop, out=v)

        np.subtract(v, self._rl_v, out=a)
        np.divide(a, dt, out=a)

        np.subtract(a, self._rl_a, out=j)
        np.divide(j, dt, out=j)
        np.clip(j, -self.jerk_limit, self.jerk_limit, out=j)

        np.multiply(j, dt, out=a)
        np.add(self._rl_a, a, out=a)
        np.clip(a, -self.accel_limit, self.accel_limit, out=a)

        np.multiply(a, dt, out=v)
        np.add(self._rl_v, v, out=v)
        np.clip(v, -self.vel_limit, self.vel_limit, out=v)

        np.multiply(v, dt, out=out)
        np.add(self._rl_q, out, out=out)
        # Layer 1b: the backstop. Whatever the cascade produced, it does not leave the
        # workspace. Writing the clamped value back into _rl_q keeps the integrator honest.
        np.clip(out, self.limit_neg, self.limit_pos, out=out)

        self._rl_q[:] = out
        self._rl_v[:] = v
        self._rl_a[:] = a
        return out

    def _limit_position_ruckig(self, tgt):
        """One Ruckig tick. `tgt` is already clamped to the soft joint limits.

        State lives in _rl_q / _rl_v / _rl_a rather than in Ruckig (via pass_to_input) on
        purpose: those three are what claim_control() seeds from the measured arm, what
        _limit_velocity shares when the PC switches command type mid-session, and what the
        rate-limiter-off branch resets. Feeding them in each tick keeps ONE copy of the
        limiter state instead of two that can silently diverge.

        ALLOCATION NOTE. new_position / new_velocity / new_acceleration each return a fresh
        Python list — three small allocations per tick, which is a deliberate exception to
        the hot-path rule in CLAUDE.md. The binding exposes no zero-copy view, and the
        alternative was keeping a limiter that oscillated all the way to the motors.
        Measured cost is in the noise next to the ~3.5 us solve.
        """
        inp = self._rk_in
        inp.current_position = self._rl_q
        inp.current_velocity = self._rl_v
        inp.current_acceleration = self._rl_a
        inp.target_position = tgt
        # Feedforward is the PC's own derivative and it is not validated anywhere upstream,
        # so it is clamped before it becomes a planner input — Ruckig rejects a target
        # velocity above max_velocity outright, and that would be a hot-path exception.
        np.clip(self._feedforward, -self.vel_limit, self.vel_limit, out=self._rk_tv)
        inp.target_velocity = self._rk_tv

        try:
            self._rk.update(inp, self._rk_out)
        except Exception:
            # Ruckig raises on a non-finite input, which is exactly what a corrupt command
            # packet looks like. Holding the last limited position is the safe answer: the
            # old cascade would have propagated the NaN to the motors instead. Cold path —
            # the f-string and the log call are fine here.
            self._rk_errors += 1
            if self._rk_errors == 1:
                logger.error("[RTI] Ruckig rejected a setpoint (non-finite command?) — "
                             "holding position. Further occurrences are counted, not logged.")
            return self._rl_q

        out = self._rl_out
        out[:] = self._rk_out.new_position
        self._rl_v[:] = self._rk_out.new_velocity
        self._rl_a[:] = self._rk_out.new_acceleration
        # Layer 1b: the backstop, same as the cascade path. Ruckig is not given the
        # positional limits (see __init__), so this is the only thing enforcing them here.
        np.clip(out, self.limit_neg, self.limit_pos, out=out)
        self._rl_q[:] = out
        return out

    def _vstop_into(self, d, out):
        """out[:] = fastest speed from which each joint can still stop within distance d.

        d must already be non-negative. Joints whose limits are not finite get +inf, i.e. no
        constraint — the alternative is inventing a braking distance from a limit the XML
        never stated, and a made-up bound is worse than an absent one because it looks real.
        """
        np.divide(d, self._brake_a, out=out)
        np.multiply(out, 2.0, out=out)
        np.add(out, self._brake_r2, out=out)
        np.sqrt(out, out=out)
        np.subtract(out, self._brake_r, out=out)
        np.multiply(out, self._brake_a, out=out)
        np.copyto(out, np.inf, where=~self._brake_ok)
        return out

    def _brake_at_limits(self, v, q):
        """Cap velocity `v` in place so no joint can overrun its soft limit from position `q`.

        DIRECTION-AWARE, and motion AWAY from a limit is always allowed. A joint already
        outside its range (a limit tightened in the XML, or a push while disabled) must still
        be able to come back — clamping symmetrically would strand it there permanently, with
        no command able to rescue it.
        """
        np.subtract(self.limit_pos, q, out=self._bl_dup)
        np.maximum(self._bl_dup, 0.0, out=self._bl_dup)
        self._vstop_into(self._bl_dup, self._bl_vup)

        np.subtract(q, self.limit_neg, out=self._bl_ddn)
        np.maximum(self._bl_ddn, 0.0, out=self._bl_ddn)
        self._vstop_into(self._bl_ddn, self._bl_vdn)

        # +v moves toward limit_pos, so it is bounded by the upper headroom, and vice versa.
        np.clip(v, -self._bl_vdn, self._bl_vup, out=v)
        return v

    def _limit_velocity(self, v_des, q_meas=None):
        """Clamp a velocity command to the velocity, acceleration and jerk budgets.

        `q_meas` is the MEASURED joint position. Velocity mode does not command position, so
        the limiter has no integrated position of its own to trust — the only honest source
        for "how far is this joint from its limit" is the encoder.
        """
        if not self.rate_limit_enabled:
            # The soft limits are NOT part of the rate limiter and must hold with it off,
            # exactly as they do on the position path. Only the v/a/j smoothing is optional.
            self._rl_v[:] = v_des
            if q_meas is not None:
                self._rl_v[:] = self._brake_at_limits(self._rl_v, q_meas)
            return self._rl_v

        dt = self.delta_t
        v, a, j = self._rl_out, self._rl_tmp_a, self._rl_tmp_j

        np.clip(v_des, -self.vel_limit, self.vel_limit, out=v)

        np.subtract(v, self._rl_v, out=a)
        np.divide(a, dt, out=a)

        np.subtract(a, self._rl_a, out=j)
        np.divide(j, dt, out=j)
        np.clip(j, -self.jerk_limit, self.jerk_limit, out=j)

        np.multiply(j, dt, out=a)
        np.add(self._rl_a, a, out=a)
        np.clip(a, -self.accel_limit, self.accel_limit, out=a)

        np.multiply(a, dt, out=v)
        np.add(self._rl_v, v, out=v)
        np.clip(v, -self.vel_limit, self.vel_limit, out=v)

        # Brake for the soft limits LAST, after the v/a/j cascade. Doing it earlier would let
        # the cascade's own acceleration clamp re-inflate a velocity the barrier had just cut
        # back, which is the whole point of the guard.
        if q_meas is not None:
            self._brake_at_limits(v, q_meas)

        self._rl_v[:] = v
        self._rl_a[:] = a
        # Integrate position too, so a later switch to position commands starts continuous.
        np.multiply(v, dt, out=self._rl_tmp_v)
        np.add(self._rl_q, self._rl_tmp_v, out=self._rl_q)
        return v

    def _gravity_on(self, ctx):
        """Both switches must agree: the robot-wide operator flag AND RTI's own."""
        return (self.gravity_comp_enabled
                and bool(ctx.shared_data.rti['gravity_comp_enabled'][0]))

    def _add_gravity(self, ctx, base):
        """Add gravity feedforward to a commanded torque. `base` may be None.

        Added AFTER _limit_torque, deliberately. The torque rate limiter exists to stop the
        PC slewing its request too fast; gravity is the robot's own compensation for its own
        weight, and throttling it would make the arm sag exactly when the pose is changing
        quickly — the moment it is needed most. It also means toggling this on or off does
        not slew: the command path is untouched either way.

        Returns `base` unchanged when compensation is off, so an off state costs nothing and
        keeps the None ("channel not commanded") semantics intact.
        """
        if not self._gravity_on(ctx):
            return base
        g = ctx.shared_data.joint['joint_gravity_torque'][:self.joint_num]
        if base is None:
            self._trq[:] = g          # position commands carry no torque request of their own
        else:
            np.add(base, g, out=self._trq)
        return self._trq

    def _block_torque_past_limits(self, t, q):
        """Zero any torque pushing a joint further beyond its soft limit.

        COARSE ON PURPOSE, and weaker than the velocity guard. Braking smoothly in torque
        space needs deceleration = torque / inertia, and effective joint inertia is
        pose-dependent and not modelled in the RT loop (that is §22 `robot.load_model()`
        territory). Without it there is no honest distance-based bound, so this is a hard
        backstop at the boundary rather than a graded approach.

        It only removes the OUTWARD component: torque pulling back into range passes
        untouched, so a joint outside its range can always recover. Gravity compensation is
        added AFTER this, so zeroing the PC's contribution holds the arm rather than dropping
        it — which is why blocking is safe here at all.
        """
        np.greater(q, self.limit_pos, out=self._bl_mask)
        np.copyto(t, 0.0, where=self._bl_mask & (t > 0.0))
        np.less(q, self.limit_neg, out=self._bl_mask)
        np.copyto(t, 0.0, where=self._bl_mask & (t < 0.0))
        return t

    def _limit_torque(self, t_des, q_meas=None):
        """Clamp commanded torque slew to torque_rate_limit (Nm/s).

        Torque rate only — magnitude is bounded downstream by the per-joint current
        limit in motor space, so clamping it here would duplicate that badly (in Nm,
        before the gear ratio is applied).

        `q_meas` enables the soft-limit backstop; see _block_torque_past_limits for why it is
        a boundary block rather than the graded brake the velocity path gets.
        """
        if not self.rate_limit_enabled:
            # Soft limits hold with the rate limiter off, same rule as every other path.
            self._rl_t[:] = t_des
            if q_meas is not None:
                self._block_torque_past_limits(self._rl_t, q_meas)
            return self._rl_t

        dt = self.delta_t
        d = self._rl_tmp_j
        np.subtract(t_des, self._rl_t, out=d)
        np.divide(d, dt, out=d)
        np.clip(d, -self.torque_rate_limit, self.torque_rate_limit, out=d)
        np.multiply(d, dt, out=d)
        np.add(self._rl_t, d, out=self._rl_t)
        if q_meas is not None:
            self._block_torque_past_limits(self._rl_t, q_meas)
        return self._rl_t

    def _start_pose_ok(self, ctx):
        """Validate the first command of a claim. False = refuse to start (plan §16).

        Refuse to begin control when the first setpoint is far from the current state.
        The hazard is cheap to catch and expensive to miss: a PC-side controller that
        initialises its target to zeros, or replays a pose from a previous session, sends
        one step command that the firmware PID chases at full authority.
        """
        ct = self.last_cmd_type
        measured = ctx.shared_data.joint['joint_position'][:self.joint_num]

        if ct == CMD_JOINT_POSITION:
            np.subtract(self._payload, measured, out=self._rl_tmp_v)
            np.abs(self._rl_tmp_v, out=self._rl_tmp_v)
            worst = int(np.argmax(self._rl_tmp_v))
            if self._rl_tmp_v[worst] > self.start_pose_tol_rad:
                logger.error(
                    "[RTI] START POSE REJECTED — joint %d commanded %.4f rad but arm is at "
                    "%.4f rad (%.4f off, tolerance %.4f). Refusing control; send the CURRENT "
                    "pose as your first setpoint.",
                    worst, self._payload[worst], measured[worst],
                    self._rl_tmp_v[worst], self.start_pose_tol_rad)
                return False
            return True

        if ct == CMD_JOINT_VELOCITY:
            budget = self.vel_limit * START_VEL_TOL_FRACTION
            bad = np.abs(self._payload) > budget
            if bad.any():
                worst = int(np.argmax(np.abs(self._payload)))
                logger.error(
                    "[RTI] START VELOCITY REJECTED — joint %d commanded %.4f rad/s as its "
                    "FIRST setpoint (limit %.4f). The arm is at rest; ramp up from zero.",
                    worst, self._payload[worst], budget[worst])
                return False
            return True

        if ct == CMD_JOINT_TORQUE:
            budget = self.torque_rate_limit * self.delta_t / START_TRQ_TOL_FRACTION
            bad = np.abs(self._payload) > budget
            if bad.any():
                worst = int(np.argmax(np.abs(self._payload)))
                logger.error(
                    "[RTI] START TORQUE REJECTED — joint %d commanded %.4f Nm as its FIRST "
                    "setpoint (limit %.4f). Ramp up from zero.",
                    worst, self._payload[worst], budget[worst])
                return False
            return True

        return True

    def set_rate_limiter(self, enabled):
        """Runtime toggle (plan §11c). OFF streams the PC's commands through untouched."""
        self.rate_limit_enabled = bool(enabled)
        logger.warning("[RTI] Rate limiter %s", "ENABLED" if enabled else "DISABLED")

    def set_start_pose_check(self, enabled):
        """Runtime toggle. OFF means a bad first setpoint reaches the motors."""
        self.start_pose_check_enabled = bool(enabled)
        logger.warning("[RTI] Start-pose check %s", "ENABLED" if enabled else "DISABLED")

    def set_gravity_comp(self, enabled):
        """RTI-scoped gravity-compensation toggle (default ON).

        Turn this off when the PC computes its own gravity term — a torque controller that
        also receives ours would apply it twice. It does NOT touch the robot-wide
        rti['gravity_comp_enabled'] flag, so IDLE and JOG are unaffected when the PC
        disconnects; the robot-wide flag still wins over this one if the operator has it off.
        """
        self.gravity_comp_enabled = bool(enabled)
        logger.warning("[RTI] Gravity compensation %s", "ENABLED" if enabled else "DISABLED")

    def set_lowpass_filter(self, cutoff_hz):
        """Set the command low-pass cutoff in Hz (plan §11c). 0 disables it.

        Cold path — rebuilding the bank allocates, so never call this per tick. Clamped to
        below Nyquist, above which a discrete filter is meaningless.
        """
        cutoff = float(cutoff_hz)
        nyquist = 0.5 / self.delta_t
        if cutoff <= 0.0:
            self._lp_bank = None
            self._lp_cutoff_hz = 0.0
            logger.warning("[RTI] Command low-pass DISABLED")
            return
        if cutoff >= nyquist:
            logger.warning("[RTI] Low-pass cutoff %.1f Hz is at/above Nyquist (%.1f Hz) — "
                           "disabling instead of pretending to filter", cutoff, nyquist)
            self._lp_bank = None
            self._lp_cutoff_hz = 0.0
            return

        # First-order IIR == the engine's "ema", parameterised by time constant.
        # The cutoff form, gain = dt/(dt + 1/(2*pi*fc)), is the same filter written as a
        # per-tick alpha; tau = 1/(2*pi*fc) is that same pole as a time constant, which is
        # what this codebase's filter config uses so a cutoff means the same thing at any
        # loop rate.
        tau = 1.0 / (2.0 * np.pi * cutoff)
        self._lp_bank = create_filter_bank({"kind": "ema", "tau": tau}, self.joint_num,
                                           self.delta_t)
        self._lp_cutoff_hz = cutoff
        logger.warning("[RTI] Command low-pass ENABLED at %.1f Hz (tau %.4f s)", cutoff, tau)

    def _apply_lowpass(self, x):
        """Filter an incoming command vector in place-ish. Returns the array to use."""
        if self._lp_bank is None:
            return x
        apply_filter_bank(self._lp_bank, x, self._lp_out)
        return self._lp_out

    def _receive_command(self, ctx):
        """Parse this tick's UDP packet, if any. Returns True if one was accepted."""
        data = ctx.rti_udp_data
        if data is None:
            return False

        result = unpack_command_packet(data)
        if result is None:
            return False

        seq, cmd_type, payload, feedforward = result

        # Sequence numbers only detect loss/reorder; they are not a filter. A
        # stale (older) packet is dropped, since drain_rti_udp already keeps only
        # the newest and an out-of-order arrival is by definition superseded.
        #
        # WRAP GUARD: seq is a uint32, which at 250 Hz wraps after ~199 days of unbroken
        # streaming. A naive `seq <= seq_rx_last` would then see 0 <= 4294967295 and reject
        # every packet FOREVER — the link would go dead with no error and no obvious cause.
        # Treat a huge backwards jump as a wrap rather than a stale packet. The threshold is
        # half the range, the standard serial-number-arithmetic convention (RFC 1982): any
        # real reordering is a handful of packets, never two billion.
        if self.seq_rx_last:
            delta = (seq - self.seq_rx_last) & 0xFFFFFFFF
            if delta == 0 or delta > SEQ_WRAP_HALF:
                return False                      # stale or duplicate
            if delta > 1:
                self.seq_rx_gaps += 1             # delta-1 commands never arrived

        self._rx_applied += 1
        self.seq_rx_last = seq
        self.last_cmd_tick = ctx.current_tick
        self.last_cmd_type = cmd_type
        self._payload[:] = payload[:self.joint_num]
        self._feedforward[:] = feedforward[:self.joint_num]
        return True

    def _update_success_rate(self, got):
        """Roll the moving window forward one tick and track consecutive misses.

        O(1): the window is a preallocated ring plus a running sum, so cost does not
        grow with SUCCESS_WINDOW_TICKS and nothing is allocated.
        """
        slot = self._sr_idx
        self._sr_sum += (1.0 if got else 0.0) - self._sr_window[slot]
        self._sr_window[slot] = 1.0 if got else 0.0
        self._sr_idx = (slot + 1) % self.success_window_ticks
        if self._sr_filled < self.success_window_ticks:
            self._sr_filled += 1
        self.success_rate = self._sr_sum / self._sr_filled

        # Consecutive misses — the one metric that means the same thing in both clock
        # modes, and the only honest "the link is broken" signal in PC-clocked mode.
        if got:
            self.consecutive_misses = 0
        else:
            self.consecutive_misses += 1
            if self.consecutive_misses > self.worst_consecutive_misses:
                self.worst_consecutive_misses = self.consecutive_misses
            if self.consecutive_misses >= self.miss_warn_ticks and not self._miss_warned:
                logger.warning(
                    "[RTI] %d consecutive ticks with no command (%.0f ms) — link stalling; "
                    "%d more and control stops",
                    self.consecutive_misses, self.consecutive_misses * self.delta_t * 1e3,
                    self.timeout_ticks - self.consecutive_misses)
                self._miss_warned = True

        # The RATIO only means packet loss when one command per tick is expected, which
        # is exactly the robot-clocked contract. In PC-clocked mode it is a duty cycle
        # and warning on it would be pure noise, so don't.
        if self.clock_mode != CLOCK_ROBOT:
            return

        # Don't judge the link until the window has actually filled.
        if self._sr_filled < self.success_window_ticks:
            return

        # Latch the warning level so a bad link logs twice, not 250x/second.
        if self.success_rate < self.success_bad and self._warned_at < 2:
            logger.warning("[RTI] Poor packet success rate: %.1f%% over the last %d ticks "
                           "— control unreliable",
                           self.success_rate * 100.0, self.success_window_ticks)
            self._warned_at = 2
        elif self.success_rate < self.success_warn and self._warned_at < 1:
            logger.warning("[RTI] Some packets lost: %.1f%% over the last %d ticks "
                           "— check network",
                           self.success_rate * 100.0, self.success_window_ticks)
            self._warned_at = 1

    def _update_bandwidth(self):
        """Convert the accumulated byte counts into a rate once per window.

        Integer adds every tick, one division per window — cheap enough for the hot path,
        and nothing is allocated.
        """
        self._bw_ticks += 1
        if self._bw_ticks < self._bw_window_ticks:
            return
        window_s = self._bw_ticks * self.delta_t
        self.tx_bytes_per_s = self._tx_acc / window_s
        self.rx_bytes_per_s = self._rx_acc / window_s
        self.discard_pct = (100.0 * (self._rx_pkts - self._rx_applied) / self._rx_pkts
                            if self._rx_pkts else 0.0)
        self._tx_acc = 0
        self._rx_acc = 0
        self._rx_pkts = 0
        self._rx_applied = 0
        self._bw_ticks = 0

    def _reset_bandwidth(self):
        self._bw_ticks = 0
        self._tx_acc = 0
        self._rx_acc = 0
        self.tx_bytes_per_s = 0.0
        self.rx_bytes_per_s = 0.0
        # Over-rate accounting. Only the newest command per tick can be applied, so a PC
        # sending faster than the loop ticks has the remainder thrown away — and nothing
        # else shows it: success rate reads 100% (a command DID arrive) and misses read 0.
        # Without this a PC-clocked controller can lose half its output silently.
        self._rx_pkts = 0          # packets pulled off the socket this window
        self._rx_applied = 0       # of those, ones that became a setpoint
        self.discard_pct = 0.0     # % of received command packets thrown away

    def _publish(self, ctx):
        rti = ctx.shared_data.rti
        rti['control_success_rate'][0] = self.success_rate
        rti['rti_substate'][0] = SUBSTATE_CODE.get(self.sub_state, 0.0)
        # The instantaneous half of the picture. success_rate is a damped 400 ms gauge by
        # design; these react next-tick and are what the stop keys off, so anything asking
        # "is the link healthy RIGHT NOW" should read these instead.
        rti['rti_consecutive_misses'][0] = self.consecutive_misses
        rti['rti_worst_consecutive_misses'][0] = self.worst_consecutive_misses
        rti['rti_tx_bytes_per_s'][0] = self.tx_bytes_per_s
        rti['rti_rx_bytes_per_s'][0] = self.rx_bytes_per_s
        # Echoed to the PC via the `rti_seq` recipe tag — the PC matches it against its own
        # send timestamps to measure loop delay, which is otherwise unmeasurable without
        # cross-machine clock sync.
        rti['rti_seq_rx_last'][0] = self.seq_rx_last
        rti['rti_discard_pct'][0] = self.discard_pct

    def _send_state(self, ctx):
        """Unicast the active recipe payload to the paired PC.

        Same build_payload() the telemetry broadcast uses (plan §2.2) — one
        serialisation per tick, just routed to the PC instead of broadcast.
        """
        try:
            payload = build_payload(ctx.recipe, ctx.Joint_data, ctx.Gripper_data,
                                    ctx.motor_buffers, self.joint_num)
            encoded = payload.encode()
            ctx.socket_sender.sendto(encoded, self.pc_addr)
            self._tx_acc += len(encoded)
        except OSError as exc:
            # Transient send failure (unreachable host, buffer full). Never kill the
            # RT loop over it; the PC-side timeout is the real backstop.
            logger.warning("[RTI] State send failed: %s", exc)
