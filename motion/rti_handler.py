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
    stopping_clean  PC ended motion (UDP CMD_STOP or TCP release). Brakes to rest within the
                    limits (2026-10-11), holds 0.5 s, then → connected. No error latched.
    stopping_error  timeout / packet loss / a refused command (the strict checks). Brakes to rest
                    within the limits, THEN the error is latched (→ ACTIVE_ERROR).

Hot-path discipline (CLAUDE.md): step() allocates nothing. Every buffer is
preallocated in __init__ and written with `dest[:] = src`; one ControlOutput
instance is reused and its channel fields are re-pointed per tick.
"""

import logging

import numpy as np

from communication.rti_protocol import (
    CMD_JOINT_POSITION, CMD_JOINT_VELOCITY, CMD_JOINT_TORQUE,
    CMD_CARTESIAN_POSE, CMD_CARTESIAN_VELOCITY, CMD_STOP,
    CLOCK_ROBOT, CLOCK_PC, unpack_command_packet, unpack_command_timestamp,
)
from data.data_recipe import build_payload
from utility.kinematics_backend import should_use_pinocchio
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

# Ceiling for the RUNTIME timeout (rti_set_command_timeout). 40 ms above is right for a PC
# streaming at the loop rate and stays the default; it is wrong for servo mode, where the
# PC deliberately sends at 5-50 Hz and a 50 ms gap between two HEALTHY 20 Hz commands is
# longer than the whole dead-link budget.
#
# Why a ceiling exists at all: raising the timeout does not let the arm run away — between
# commands the generator continues toward the LAST target and stops there, so the motion is
# bounded by what the PC already asked for. What it delays is the moment a genuinely dead
# link is DECLARED, and that must stay bounded. 1.0 s tolerates four missed commands even at
# 5 Hz, the slowest rate servo mode is meant to serve; past a second of silence, latching
# RTI_LINK_LOST is the correct answer regardless of the configured rate.
MAX_COMMAND_TIMEOUT_S = 1.0

# WHO MAY WIDEN THE WATCHDOG, and how far (2026-09-30). Until then ONE ceiling — the 1.0 s above —
# applied to both contracts and any client could raise it in any mode. That is safe for a
# POSITION stream and not for the other two command types: while no new command arrives the
# handler re-applies the LAST one every tick (`_step_active` -> `_command_output`). A stale
# position converges to its target and holds — bounded. A stale VELOCITY or TORQUE keeps being
# applied for the whole window. Hard RTI carries all three, so a client could leave a stale torque
# running for up to a second.
#
#   RTI       the watchdog is COMMAND_TIMEOUT_S (40 ms) and can only be TIGHTENED by a client.
#   SOFT_RTI  position targets ONLY (velocity/torque are refused in _receive_command). A client
#             may widen the watchdog up to the OPERATOR's ceiling, <safety><max_soft_rti_timeout>
#             in the robot XML — a remote PC can never exceed what the operator allowed. 250 ms
#             by default: ~12 missed commands at 50 Hz, longer than the ~100 ms outages that
#             stopped UDP in the loss tests (servo_mode_options.md 17.14-17.17), still well under
#             a second. The declared rate always keeps its own 3 intervals (a 5 Hz stream needs
#             600 ms just to be healthy), so the ceiling limits EXTRA slack, never the contract.
DEFAULT_SOFT_TIMEOUT_CEILING_S = 0.25

# Servo generators (TODO/servo_mode_options.md §3). The names are the wire values of
# rti_set_servo_generator, so they are constants rather than bare strings in three files.
SERVO_RUCKIG = "ruckig"       # point-to-point planner, default, hardware-verified
SERVO_TRACKER = "tracker"     # servo-style tracking law (the common streaming-servo command), for low-rate streams
SERVO_ADVANCING = "ruckig_advancing"   # same planner, aimed at where the target IS
SERVO_PASSTHROUGH = "passthrough"      # hard RTI: the PC's stamped samples replayed exactly (2026-10-11)
SERVO_GENERATORS = (SERVO_RUCKIG, SERVO_TRACKER, SERVO_ADVANCING, SERVO_PASSTHROUGH)

# PASS-THROUGH (hard RTI's generator since 2026-10-11; TODO/servo_mode_options.md §17.21). Hard RTI trailed the PC's
# targets by 26-41 ms on the arm: plain ruckig plans to STOP at every target (~19 ms on J1) and the drive got no speed
# (~11 ms) — where a hard real-time interface should add a tick. Industrial 1 kHz interfaces pass the PC's position
# straight through and derive its speed from the stream. Two findings shaped how we do it (simulated with this handler,
# PAR6 limits, J1 sine 63 deg/s): (1) a PC that times its targets with its own clock samples them up to ±0.4 ms off the
# robot's tick grid (measured on the arm: step sizes ±10 %); played as if evenly spaced that is 8-11 deg/s of speed
# noise — so the samples are REPLAYED by their send times (communication/rti_protocol, 109-byte packets), a small,
# slowly adapting delay behind the slowest recent delivery; (2) position, speed and acceleration must come from ONE
# curve (a quadratic through the last three samples) — a straight-line position with a separately estimated speed
# disagreed by about what a one-tick jerk-limited landing can absorb, and every disagreement cascaded. Per tick Ruckig
# (time-synchronised, a minimum duration of exactly one tick) lands each joint on its aim when that joint's own fastest
# landing fits in the tick — the pass-through — and otherwise catches up within the limits (a step, a jump, a dead
# stop): no joint is slowed by another, nothing coasts past. Simulated: planner lag 18.8 -> 4.0 ms, path error
# 0.13 -> 0.000 deg (robot-clocked), 4.3-5.8 ms / <= 0.004 deg with ±0.4-2 ms PC jitter, 6 ms / 0.008 deg for an
# uneven 200-250 Hz sender, drops / late / mixed packets clean, steps never overshoot, soft limits within a / j. The
# costs: a PC that stops dead at speed overshoots by the braking distance (J1 at 60 deg/s ~1 deg; the old planner
# trailed by that much all the time), and target noise passes through more (3.5 vs 2.9 deg/s above 10 Hz).
# A 101-byte packet carries no time, and arrival ticks are no substitute (simulated pass-through on arrival ticks with
# a jittery PC: 13 / 39 deg/s of wobble, 2 deg peaks): hard RTI refuses such a position target (2026-10-12,
# _refuse_unstamped); until then it silently got the old planner.
PT_RING = 8                       # samples kept
PT_DELAY_DECAY_TICKS = 0.002      # per packet: the slowest recent delivery is forgotten slowly
PT_DELAY_RATE_TICKS = 0.002       # per tick: the replay delay drifts at most this much (a 0.2 % time warp, in the speed)
# Past the newest sample (dropped / late packets) the replay continues on the same curve — constant acceleration, as
# industrial 1 kHz interfaces extrapolate a lost motion packet — until the 40 ms watchdog, whose stop is a controlled
# one (_begin_stop). Until 2026-10-11 it coasted 3 ticks and braked: the stop that followed froze the command, and
# braking first kept that freeze gentle (user: as industrial 1 kHz interfaces do, keeping our 40 ms, with the
# controlled stop).
PT_GATE = 1.5                     # a curvature above 1.5x the acceleration limit is a jump: plan to stop on the aim
PT_MAX_GAP_TICKS = 25.0           # samples further apart (100 ms) or out of order: a new start

# STRICT COMMAND CHECKS (2026-10-11, user: as industrial 1 kHz interfaces do). A hard-RTI stream with send times must be motion the arm
# can do: a sample that moves a joint past its soft limit (or faster than it can still stop before it), faster than its
# velocity limit, with an acceleration or a jerk above its limit (what 1 kHz interfaces call a velocity / acceleration
# DISCONTINUITY) is an ERROR — a controlled stop, then J{i}:RTI_POSITION_LIMIT / RTI_VELOCITY_LIMIT /
# RTI_ACCELERATION_LIMIT / RTI_JERK_LIMIT latched — instead of being absorbed (a step, a dead stop, a wild packet:
# the pass-through caught up within the limits and carried on). Checked on the PC's samples over their TRUE intervals
# (send times), so a stream without them (101 bytes: the robot cannot tell jitter from motion) keeps the forgiving
# planner. SOFT_RTI: only the position check (the robot fills in between sparse targets, so the stream's speed is not
# the arm's). Noise in the targets reads as acceleration (0.02 deg per sample ~ 44 rad/s^2): filter on the PC or
# rti_set_lowpass. The limits are the RTI ceilings (<rti_limits>), not a vel_scale preference (that only limits).
CMD_CHECK_TOL = 0.10              # 10 % over a limit before a sample is refused (a jerk-limited plan at the limit passes)
CMD_CHECK_POS_EPS = 1e-9          # rad: a target ON the limit is fine
CMD_CHECK_BRAKE_TOL = 0.02        # rad/s over the braking curve toward a limit: refused (absolute — a % of the speed limit
                                  # was 0.48 rad/s on J1: refused 0.4 deg from the limit, too late to brake within it)

# Streaming contracts (rti_set_profile). Some industrial controllers split these into two interfaces (a hard
# real-time one and a soft one); we keep one code path and name the contract, because the difference genuinely
# is settings — but an UNNAMED difference makes the diagnostics lie (success rate means
# packet loss at 250 Hz and nothing at 20 Hz).
# Superseded 2026-08-20 by the RobotMode.RTI / RobotMode.SOFT_RTI split. The contract is
# now carried by the MODE, which is visible in shared memory, the GUI, the activity log and
# status() — where a profile flag was invisible to all of them. Two mechanisms for one
# contract is the confusion the split exists to end, so rti_set_profile was removed rather
# than kept as a second route.
PROFILE_REALTIME = "realtime"
PROFILE_SERVO = "servo"
SERVO_PROFILES = (PROFILE_REALTIME, PROFILE_SERVO)

# Tracker bandwidth wn, rad/s. MEASURED, not reasoned: the first value here was 25, picked
# from bandwidth argument alone, and the sweep in servo_mode_options.md §5.4 beat it on every
# axis. 15 rad/s (~2.4 Hz) is the knee —
#   * constant ramp: 0.08 deg tracking error at every rate from 50 Hz down to 5 Hz
#   * 0.5 Hz sinusoid: better than Ruckig at every rate (0.72 deg vs 2.05 at 50 Hz)
#   * 0.5 rad step: ZERO overshoot, and the fastest settle of the gains that do not overshoot
# Above ~20 the step response starts to overshoot, because wn^2 * step_size exceeds the
# acceleration limit and the law saturates. That threshold is a property of THIS arm's
# <joint_acceleration_limit> and the step size, so it moves with the robot XML — treat 20 as
# a measured landmark on PAR6, not a universal constant.
# How many recent command gaps the arrival-interval estimate is taken over. 8 is long
# How many recent command gaps the arrival-interval estimate averages over.
#
# 12 is CHOSEN, not arbitrary, and it is the single most sensitive constant here. Arrivals
# are whole ticks, so for a command rate of 250/n the pattern of gaps repeats with period n,
# and an average is exact only when the window is a whole number of those periods. 12 covers
# periods 1, 2, 3, 4, 6 and 12 -- which is every rate anyone actually asks for between 20 and
# 62.5 Hz. Measured ripple at 30 Hz and 60 Hz: 24-29% at window 8, 0.004% at window 12.
# Rates whose period does not divide 12 (55, 65, 70, 90 Hz -> 11, 13, 7, 9) are NOT fixed by
# any window. LONGER windows were measured (24/36/48/72/144) and are strictly worse -- they
# average across more partial periods and start degrading the rates that 12 gets exactly
# right, so 12 is a maximum and not a floor. See docs for what to do at those rates instead.
GAP_WINDOW = 12

# How much longer than the current estimate a gap must be before it is read as a DROPPED
# command rather than a normal one. Plain rounding (i.e. a margin of 1.5) is not safe here and
# self-destructs at high rates: at 100 Hz the true interval is 2.5 ticks and arrivals alternate
# 2 and 3, so if the estimate is momentarily 2.0 then round(3/2.0) = 2 reads a PERFECTLY NORMAL
# gap as two intervals, stores 1.5, and drags the estimate to 1.75 -- where it locks, because
# every 3-tick gap keeps confirming it. Measured before this margin existed: estimate 1.75
# against a true 2.5, target velocity alternating 0.286/0.571 against a true 0.400.
# 1.75 keeps real drops detected (a dropped command at any rate at least doubles the gap)
# while leaving normal jitter alone.
SPAN_MARGIN = 1.75

# Inertia / Coriolis FEEDFORWARD in RTI / SOFT_RTI (2026-10-11, TODO/dynamics_feedforward_plan.md F3). The generator's
# own acceleration (_rl_a) is never fed forward: it switches between its limits as the planner chases each new target
# (simulated on PAR6: ±1.8 Nm on J1). "stream" feeds forward the STREAM's acceleration instead — the change of the
# target-velocity estimate between arriving packets, over their interval, smoothed by a critically damped 2nd-order
# low-pass. Simulated against the torque the commanded motion needs (1.0 = no better than no feedforward): hard RTI
# 250 Hz 0.12, SOFT_RTI 50 Hz 0.27, wandering 50-60 Hz 0.33, 25 Hz 0.38, 10 Hz 0.69, 5 Hz ~1 (too late to help),
# ripple <= 0.03 Nm. Set at every RTI / SOFT_RTI entry to DYNAMICS_FF_DEFAULT; a PC may choose with rti_set_dynamics_ff.
DYNAMICS_FF_SOURCES = ("off", "stream")
# "stream" since 2026-10-11 (user: "small gains and no harm - why not on by default"): on the arm (api_tests/68, J1,
# hard RTI 250 Hz + SOFT_RTI 50 / wandering / 10 Hz) it never raised the buzz or the jerkiness, and cut the tracking
# error 2-10 % and the drive's own extra torque 4-9 %. Like gravity compensation: on by default, "off" for a PC that
# computes its own dynamics.
DYNAMICS_FF_DEFAULT = "stream"

# SOFT_RTI's duration pacing, set at EVERY SOFT_RTI entry (2026-10-11, user; TODO/dynamics_feedforward_plan.md F3). Without
# it the advancing generator re-plans to each extrapolated target as fast as the RTI limits allow (J1: 32 rad/s^2,
# 16000 rad/s^3), so every packet that differs from the extrapolation is corrected with a kick: on the arm
# (api_tests/68, J1 sine 63 deg/s) J1's speed strayed 9.6-13.6 deg/s from its smooth path at 1.5-10 Hz at 50 / 10 Hz
# (2.1 in hard RTI) — "really jerky", the user — and, simulated, an operator's sudden stop at 60 deg/s overshot
# 2.0 deg (50 Hz) / 6.8 deg (10 Hz). With 1.5 command intervals (simulated, the real handler, reproducing the arm's
# commanded wobble to 0.01 deg/s): 4-13x smoother, 2-4x MORE accurate in shape, stop overshoot 0.44 deg (50 Hz) / 0
# (<= 10 Hz), for more lag (+20 ms at 50 Hz, +50 at 25 Hz, +120 at 10 Hz). A PC may still set its own
# (rti_set_min_duration; 0 = the old behaviour) after rti_connect. Hard RTI keeps 0 (one setpoint per tick).
SOFT_MIN_DURATION_DEFAULT = 1.5

# Packet SEND TIMES (2026-10-11, communication/rti_protocol.py; TODO/servo_mode_options.md §17.20). A sender whose rate
# wanders (VR, Wi-Fi) made every target look early or late against the arrival ticks — the remaining fast chatter after
# the duration pacing (on the arm, J1's generator acceleration above ~10 Hz 0.6 -> 20.6 rad/s^2 from a steady to a
# wandering 50-60 Hz sender). With the PC's own send times the velocity is taken over the TRUE interval and the
# "when was this target valid" clock follows the PC: simulated 3x less chatter, a 3x more faithful path, less lag.
TS_MAX_INTERVAL_S = 1.0           # a longer gap between two timestamped packets: today's estimate for that one
TS_MAX_SPANS = 50                 # sequence numbers further apart than this are not "dropped packets" but a new stream
TS_OFFSET_LEAK_TICKS = 0.002      # per packet: the fastest delivery is forgotten slowly (clock drift, a slower route)
TS_RESYNC_TICKS = 25.0            # a delivery 100 ms slower than the fastest: the PC's clock jumped -> re-anchor
_NAN = float("nan")
STREAM_ACC_LP_HZ = 8.0            # the low-pass: unsmoothed, a wandering sender's estimate was worse than none (1.5)
STREAM_ACC_HOLD_INTERVALS = 2.0   # no packet for this many intervals: the held estimate drops to 0 (then decays)

TRACKER_GAIN_DEFAULT = 15.0
TRACKER_GAIN_OVERSHOOT_WARN = 20.0
TRACKER_GAIN_MIN = 1.0        # below this the tracker is slower than any useful motion
TRACKER_GAIN_MAX = 150.0      # 150 rad/s ~= 24 Hz, still an order below Nyquist at 250 Hz
TRACKER_LOOKAHEAD_MAX = 0.5   # beyond half a second the extrapolation dominates the target

# Auto velocity pacing (rti_set_pace). The budget never drops below this, so a target that
# is momentarily stationary — a turning point, or the PC simply holding still — cannot pin
# the generator at zero and strand it when the target moves again.
# Duration pacing (rti_set_min_duration), as a multiple of the measured command interval.
# Below ~1.2 the plan ends before the next command and its deceleration still runs, so the
# velocity returns to zero and nothing is gained; the useful range starts just above 1.
# Floor for rti_set_state_rate. NOT zero, deliberately: the state stream is the PC's only
# failure detector — RTIClient.control() raises ControlLost when it stops, which is how a
# latched error on the robot becomes an exception on the PC rather than a silent hang. An
# operator who could switch it off entirely would be deleting that signal without knowing.
# 10 Hz also happens to be what control()'s default 0.1 s timeout can tolerate.
STATE_RATE_MIN_HZ = 10.0

MIN_DURATION_MULT_MIN = 1.0
MIN_DURATION_MULT_MAX = 5.0
# 2.0, not 1.5 — measured on hardware 2026-08-21 (servo_mode_options.md 14.9). At 5 Hz on
# three wrist joints the whole Pareto frontier is min_duration, and 2.0 is its most accurate
# point (0.036 deg worst lag, 0.0275 Nm roughness, 0.004 deg coordination spread). 1.5 sits
# just OFF the frontier: 2.0 beats it on both axes.
MIN_DURATION_MULT_SUGGESTED = 2.0

PACE_FLOOR_RAD_S = 0.02
PACE_FACTOR_MAX = 10.0     # beyond this the cap is looser than any real motion needs

# Floor for the runtime velocity scale (rti_set_vel_scale). 1% of the XML limit is already
# far slower than any useful motion; below it the arm is effectively stopped, which is what
# a stop command is for. Guards against a caller passing 0 and silently freezing the arm
# rather than getting an error.
MIN_VEL_SCALE = 0.01

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
DEFAULT_LOWPASS_CUTOFF_HZ = 25.0   # (informational; the filter is a 2nd-order Butterworth since 2026-10-11 —
                                  #  for hard RTI's strict checks 60 Hz (+3 ms) is the better start, §17.22)

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
                 success_bad=SUCCESS_BAD,
                 tcp_vel_limit=0.0,
                 soft_timeout_ceiling=DEFAULT_SOFT_TIMEOUT_CEILING_S):
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
        self._last_motion_type = None
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
        # A hold captures the measured pose ONCE, when it begins, and holds it stiffly until a command is
        # accepted again (2026-10-04, review S6 / §7.4: what industrial arms commonly do after a stop, and what a
        # PC-side client expects after rti_release — "it stays where I left it"). Until then _hold re-sampled
        # the pose every tick: a damped float that a hand could push and that drifted under gravity.
        self._hold_frozen = False
        # acc_feedforward=False: the acceleration below is published, never fed forward (see ControlOutput).
        self._out = ControlOutput(acc_feedforward=False)

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
        # The XML ceiling, kept untouched so set_vel_scale() always scales from the source
        # rather than compounding on whatever the last call left behind. Scaling in place
        # from self.vel_limit would make scale(0.5) twice mean 0.25 — a preference that
        # ratchets is a preference nobody can reason about.
        self._vel_limit_base = self.vel_limit.copy()
        self.vel_scale = 1.0
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
        # Pass-through (SERVO_PASSTHROUGH): the ring of samples (send time in ticks, the PC's position) and scratch.
        self._pt_s = np.zeros(PT_RING, dtype=np.float64)
        self._pt_q = np.zeros((PT_RING, joint_num), dtype=np.float64)
        self._pt_k = 0                    # samples taken since the last start (ring index = k % PT_RING)
        self._pt_last_tick = -1           # the packet the ring last saw
        self._pt_shift = 0.0              # ticks taken off the send times after a jump of the PC's clock
        self._pt_arrival = 0              # the arrival tick of the newest sample
        self._pt_L = 0.0                  # replay delay: robot tick - send time (ticks), slewed
        self._pt_Lt = 0.0                 # ... its target: the slowest recent delivery
        self._pt_rk_cfg = False           # Ruckig's input set up for the pass-through (sync / duration / target acc)
        self._pt_fbc = np.zeros(joint_num, dtype=np.float64)
        self._pt_fab = np.zeros(joint_num, dtype=np.float64)
        self._pt_fabc = np.zeros(joint_num, dtype=np.float64)
        self._pt_aim_raw = np.zeros(joint_num, dtype=np.float64)
        self._pt_aim = np.zeros(joint_num, dtype=np.float64)
        self._pt_v = np.zeros(joint_num, dtype=np.float64)
        self._pt_a = np.zeros(joint_num, dtype=np.float64)
        self._pt_vb = np.zeros(joint_num, dtype=np.float64)
        self._pt_gate = np.zeros(joint_num, dtype=bool)
        self._pt_mask = np.zeros(joint_num, dtype=bool)
        self._pt_ok = np.zeros(joint_num, dtype=bool)
        self._pt_imd = np.zeros(joint_num, dtype=np.float64)
        self._pt_skip_checks = 0          # samples whose spacing was guessed (a clock jump): not checked
        self._chk_v = np.zeros(joint_num, dtype=np.float64)       # strict checks' scratch
        self._chk_a = np.zeros(joint_num, dtype=np.float64)
        self._chk_j = np.zeros(joint_num, dtype=np.float64)
        self._chk_b = np.zeros(joint_num, dtype=np.float64)
        self._chk_m = np.zeros(joint_num, dtype=bool)
        self._chk_tick = -1               # the packet the SOFT_RTI position check last saw
        self._chk_q_prev = np.zeros(joint_num, dtype=np.float64)   # its previous target (raw) and when it was valid
        self._chk_t_prev = _NAN
        self._chk_ts_prev = _NAN
        self._chk_vlim = self._vel_limit_base * (1.0 + CMD_CHECK_TOL)
        self._chk_alim = self.accel_limit * (1.0 + CMD_CHECK_TOL)
        self._chk_jlim = self.jerk_limit * (1.0 + CMD_CHECK_TOL)
        # Controlled stop (_begin_stop): brake within the limits, THEN hold — and only then raise the error.
        self._ramp_on = False
        self._ramp_seed_pending = False
        self._ramp_errors = ()            # error keys raised once the arm is at rest
        self._ramp_link_lost = False
        self._cmd_errors = ()             # handed to the error pass once (take_command_errors)
        # This tick's position bounds: the soft limits widened to wherever the limiter IS, per
        # joint. See _effective_limits().
        self._lim_lo_eff = np.zeros(joint_num, dtype=np.float64)
        self._lim_hi_eff = np.zeros(joint_num, dtype=np.float64)
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
        # it either: `run_safety_checks()` was never called and there was no joint-limit error.
        # (Since 2026-09-30 utility/safety_monitor.py latches OUT_OF_RANGE on the MEASURED
        # position — the detector behind this preventer.)
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
        # TCP (Cartesian) velocity cap — open defect D. Joint velocity limits do NOT bound
        # end-effector speed: near a singularity a slow joint produces a very fast TCP, which
        # is exactly where an arm hurts someone. This cap closes that in the one mode where
        # an external, untrusted stream drives the arm.
        #
        # Mechanism (local tangent method): per tick, take the joint-space direction to the
        # target u = d/|d|; the translational Jacobian gives the TCP metres per radian of
        # progress along it, tcp_rate = |J_pos @ u|. Path speed s must satisfy
        # tcp_rate * s <= limit, and joint i moves at |u_i| * s — so Ruckig's per-joint
        # max_velocity is set to min(vel_limit_i, |u_i| * limit / tcp_rate) for this tick.
        # Velocity commands are simpler: scale the whole vector when |J_pos @ v| exceeds
        # the limit, which preserves the commanded direction exactly.
        #
        # 0 = disabled (the shipped default until a value is verified on hardware). The
        # number comes from <safety><max_tcp_velocity> in the robot XML — the same key the
        # safety monitor's TCP_OVERSPEED check reads (utility/safety_monitor.py), so the
        # DETECTOR and this PREVENTER can never disagree about the limit.
        # ------------------------------------------------------------------ #
        # Servo generator selection (TODO/servo_mode_options.md §3). Two laws, one job:
        # turn sparse targets into a per-tick setpoint. "ruckig" is the default and the only
        # one hardware-verified; "tracker" is a servo-style tracking law for low-rate streams.
        # Both share every safety layer — soft limits, TCP cap, limit braking.
        # ------------------------------------------------------------------ #
        self.servo_generator = SERVO_RUCKIG
        # Which contract this session is running under. Set by apply_mode_contract() from
        # the RobotMode being entered; reported in telemetry so a reader knows whether the
        # success-rate metric means anything (it does not below ~125 Hz).
        self.soft_mode = False
        # Operator's ceiling on how far a client may widen the SOFT_RTI watchdog (seconds). Kept
        # inside [the hard-RTI timeout, the absolute maximum]: below the hard value it would make
        # SOFT_RTI stricter than RTI, above the maximum it would exceed the safety bound.
        try:
            _ceil = float(soft_timeout_ceiling)
        except (TypeError, ValueError):
            _ceil = DEFAULT_SOFT_TIMEOUT_CEILING_S
        if not np.isfinite(_ceil):
            _ceil = DEFAULT_SOFT_TIMEOUT_CEILING_S
        self.soft_timeout_ceiling = min(max(_ceil, COMMAND_TIMEOUT_S), MAX_COMMAND_TIMEOUT_S)
        # One line at start-up so the policy in force is visible in RTI.log (and so a restart onto
        # the new code can be CONFIRMED with a grep instead of by running the arm).
        logger.info("[RTI] Watchdog policy: RTI %.0f ms, tighten-only; SOFT_RTI position-only, widenable "
                    "up to %.0f ms (operator ceiling <safety><max_soft_rti_timeout>)",
                    COMMAND_TIMEOUT_S * 1e3, self.soft_timeout_ceiling * 1e3)
        self._soft_rate_hz = None         # the PC's declared command rate, SOFT_RTI only
        self._soft_refused = 0            # non-position packets refused this session
        self.tracker_gain = TRACKER_GAIN_DEFAULT
        self.tracker_lookahead = 2.0 / TRACKER_GAIN_DEFAULT   # cancels ramp lag; see the law
        self.tracker_self_lookahead = 0.0                     # damping-style lookahead; off
        self._trk_prev_tgt = np.zeros(joint_num, dtype=np.float64)
        self._trk_vtgt = np.zeros(joint_num, dtype=np.float64)
        self._trk_ahead = np.zeros(joint_num, dtype=np.float64)
        self._trk_a = np.zeros(joint_num, dtype=np.float64)
        self._trk_v = np.zeros(joint_num, dtype=np.float64)
        self._trk_da = np.zeros(joint_num, dtype=np.float64)
        self._trk_cmd_tick = -1
        self._trk_gap = 1.0
        self._trk_gaps = []
        self._state_every = 1
        self._trk_ideal = 0.0
        self._trk_ideal_seeded = False
        self._trk_vtgt_from_pc = False
        self._trk_seeded = False
        self._trk_interval_s = delta_t   # the time the last velocity estimate was taken over (s): _stream_acceleration
        self._cmd_ts = _NAN               # this packet's send time (s, the PC's clock); nan without one
        self._ts_prev = _NAN              # the previous packet's
        self._ts_prev_seq = 0
        self._ts_off = _NAN               # arrival tick minus send time (ticks): the fastest delivery, forgotten slowly
        self._trk_nvcap = np.zeros(joint_num, dtype=np.float64)
        # Stream-acceleration feedforward (see DYNAMICS_FF_SOURCES). All preallocated: _stream_acceleration runs every
        # tick of a position stream.
        self.dynamics_ff_source = DYNAMICS_FF_DEFAULT
        self._sacc = np.zeros(joint_num, dtype=np.float64)         # the smoothed estimate — what is fed forward
        self._sacc_lp1 = np.zeros(joint_num, dtype=np.float64)     # first low-pass stage
        self._sacc_hold = np.zeros(joint_num, dtype=np.float64)    # the newest packet's raw estimate, held
        self._sacc_vprev = np.zeros(joint_num, dtype=np.float64)   # the previous packet's velocity
        self._sacc_tmp = np.zeros(joint_num, dtype=np.float64)
        self._sacc_cmd_tick = -1
        self._sacc_n = 0                  # packets taken since the claim: a velocity from the 2nd, an acceleration from the 3rd
        _tau = 1.0 / (2.0 * np.pi * STREAM_ACC_LP_HZ)
        self._sacc_alpha = float(delta_t / (_tau + delta_t))

        # Auto velocity pacing — the self-tuning form of the velocity cap. Off by default;
        # rti_set_vel_scale stays the shipped behaviour until this is verified on the arm.
        self.pace_factor = 0.0
        self._pace_v = np.zeros(joint_num, dtype=np.float64)
        # Duration pacing, as a multiple of the measured command interval. Off by default.
        self.min_duration_mult = 0.0
        self._min_dur_applied = False
        # Precomputed bounds — see _limit_position_tracker for why these are not built
        # inline. _refresh_tracker_constants() rebuilds them whenever a gain or a limit
        # changes, so there is exactly one place that knows how they are derived.
        self._trk_wn2 = np.zeros(joint_num, dtype=np.float64)
        self._trk_2wn = np.zeros(joint_num, dtype=np.float64)
        self._trk_jstep = np.zeros(joint_num, dtype=np.float64)
        self._trk_njstep = np.zeros(joint_num, dtype=np.float64)
        self._trk_naccel = np.zeros(joint_num, dtype=np.float64)
        self._refresh_tracker_constants()

        self.tcp_vel_limit = max(0.0, float(tcp_vel_limit))
        self._tcp_u = np.zeros(joint_num, dtype=np.float64)          # direction to target
        self._tcp_scaled_vmax = np.zeros(joint_num, dtype=np.float64)
        self._tcp_cap_applied = False    # Ruckig max_velocity currently holds a scaled set
        self._tcp_warned = False         # one-shot: no Jacobian source available

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
        self._rk_unavailable = ""     # why, for the rti_claim refusal message
        self._rk_errors = 0
        self._rk_tv = np.zeros(joint_num, dtype=np.float64)   # clamped feedforward
        if not _RUCKIG_AVAILABLE:
            self._rk_unavailable = ("the ruckig package is not importable (it is a pinned "
                                    "requirement: `pip install -r requirements.txt`)")
            logger.error("[RTI] %s — RTI mode cannot claim control with the rate limiter "
                         "enabled.", self._rk_unavailable)
        elif not (np.all(np.isfinite(self.vel_limit)) and
                  np.all(np.isfinite(self.accel_limit)) and
                  np.all(np.isfinite(self.jerk_limit))):
            # Ruckig rejects a non-finite limit. Rather than substitute an invented
            # ceiling, say so — an XML missing a limit is a config problem to fix, not one
            # to paper over with a made-up number.
            self._rk_unavailable = ("velocity/acceleration/jerk limits are not all finite "
                                    "in the robot XML")
            logger.error("[RTI] %s — RTI mode cannot claim control with the rate limiter "
                         "enabled. Fix the XML.", self._rk_unavailable)
        else:
            self._rk = Ruckig(joint_num, delta_t)
            self._rk_in = InputParameter(joint_num)
            self._rk_out = OutputParameter(joint_num)
            self._rk_in.control_interface = ControlInterface.Position
            # Per-joint independence: each joint moves at its own optimal pace rather than
            # waiting for the slowest. Flipped to Synchronization.Time by set_min_duration,
            # because Ruckig IGNORES minimum_duration under Synchronization.No — a duration
            # floor is a property of one shared trajectory, and with No sync there is no
            # shared trajectory to floor. Verified: identical first-tick velocity with the
            # floor at 0.0 and 0.2 under No, honoured under Time.
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
        self._hold_frozen = True
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

    def can_claim(self):
        """(ok, reason) — may control be claimed right now?

        Refuses when the rate limiter is ON but has no working planner. There used to be a
        fallback here (a hand-rolled derivative cascade) and it was removed 2026-08-08,
        because it was a trap rather than a safety net:

          - It converges ONLY when the jerk limit is loose enough to be inert. At PAR6's
            pre-liberal jerk (310 rad/s^3, 50x accel) it oscillates +/-0.58 deg on a
            CONSTANT target, forever, straight to the motors. So it was correct today and
            would silently become wrong the moment jerk is tightened — a config change in
            an unrelated XML field, with no connection to this code path.
          - It could not be exercised, so it rotted. A NaN bug sat in it undetected until
            someone went looking (see defect C).

        Failing loudly at claim time is strictly safer than moving badly: an operator gets a
        named error before anything moves, instead of a wrist that will not settle.
        """
        if self.rate_limit_enabled and self._rk is None:
            return False, (self._rk_unavailable or "rate limiter has no planner")
        return True, ""

    def claim_control(self, current_tick, measured_pos=None):
        """TCP rti_claim: connected → control_active. Timing rules start now.

        Returns True if control was claimed, False if refused — see can_claim().

        measured_pos seeds the rate limiter so the first limited command is relative to
        where the arm actually is. Without it the limiter would ramp from 0 rad and yank
        the arm toward the origin — the exact failure the limiter exists to prevent.
        """
        ok, why = self.can_claim()
        if not ok:
            logger.error("[RTI] rti_claim REFUSED — %s. Fix the install or the robot XML, "
                         "or stream with the rate limiter explicitly disabled "
                         "(rti_set_rate_limiter(False)) and accept unfiltered commands.", why)
            return False

        self.sub_state = 'control_active'
        self.last_cmd_tick = current_tick
        self.last_cmd_type = None
        self._last_motion_type = None
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

        # Tracker state, cleared for the same reason the limiter state is: a target-velocity
        # estimate carried over from the previous session would extrapolate this session's
        # first setpoint toward wherever the last PC was going.
        self._trk_prev_tgt[:] = self._rl_q
        self._trk_vtgt[:] = 0.0
        self._trk_cmd_tick = current_tick
        self._trk_gap = 1.0
        self._trk_gaps = []
        self._trk_ideal = 0.0
        self._trk_ideal_seeded = False
        self._trk_vtgt_from_pc = False
        self._trk_seeded = False
        self._trk_interval_s = self.delta_t
        self._cmd_ts = self._ts_prev = self._ts_off = _NAN
        self._ts_prev_seq = 0
        self._pt_reset(current_tick)
        self._pt_skip_checks = 0
        self._chk_tick = current_tick                   # the strict checks start fresh with this session
        self._chk_t_prev = self._chk_ts_prev = _NAN
        self._ramp_on = False
        self._ramp_errors = ()
        self._ramp_link_lost = False
        self._reset_stream_acceleration(current_tick)

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
        return True

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

        # Stream state to the paired PC in every sub-state except unpaired. Every tick
        # unless the PC asked for less (set_state_rate) — see there for why that matters
        # most on the TCP transport.
        if self.pc_addr is not None and (
                self._state_every == 1
                or ctx.current_tick % self._state_every == 0):
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
        if self.sub_state != 'control_active':      # the packet was refused (no send time in hard RTI): the stop
            return self._step_stopping(ctx)

        if got:
            if self.last_cmd_type == CMD_STOP:
                self._begin_stop('stopping_clean')
                logger.info("[RTI] Motion finished by PC")
                return self._step_stopping(ctx)

            # Validate the FIRST command of this claim before it can reach a motor.
            if not self._start_pose_checked:
                self._start_pose_checked = True
                if self.start_pose_check_enabled and not self._start_pose_ok(ctx):
                    self._begin_stop('stopping_error', link_lost=True)
                    return self._step_stopping(ctx)

        # Only score ticks from the first accepted command onward. Counting the
        # gap between claim_control() and the PC's first packet would report 0%
        # and fire the "control unreliable" warning before control has started.
        if self.last_cmd_type is not None:
            self.control_ticks += 1
            if got:
                self.commands_received += 1
            self._update_success_rate(got)

        if not got and (ctx.current_tick - self.last_cmd_tick) > self.timeout_ticks:
            self._begin_stop('stopping_error', link_lost=True)
            logger.warning(
                "[RTI] No command for %d ticks (limit %d) — braking to rest, then latching RTI_LINK_LOST",
                ctx.current_tick - self.last_cmd_tick, self.timeout_ticks)
            return self._step_stopping(ctx)

        if self.last_cmd_type is None:
            # Claimed but the first packet has not arrived yet — hold, don't time out
            # until the window elapses (handled above).
            return self._hold(ctx)

        self._hold_frozen = False          # streaming again: the next hold starts from where the stream ends
        out = self._command_output(ctx)
        if self.sub_state != 'control_active':      # a strict command check refused this packet: a controlled stop
            return self._step_stopping(ctx)
        return out

    def _step_stopping(self, ctx):
        if self._ramp_on:
            return self._ramp_output(ctx)        # braking; the 0.5 s hold below counts from the rest
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

    def _begin_stop(self, which, errors=(), link_lost=False):
        """A CONTROLLED stop (2026-10-11, review §7.1 for RTI; user: RTI's stops brake, safety trips stay instant):
        every joint brakes to rest within its acceleration / jerk limits from where it is commanded to be, then
        holds there; the error (`link_lost` -> RTI_LINK_LOST, `errors` -> those keys) is raised only once the arm is at
        rest — raising it at once switched the mode to ACTIVE_ERROR, whose zero-velocity hold stopped the arm as hard
        as the drives could. Without Ruckig: the old instant hold, the error at once."""
        self.sub_state = which
        self.stopping_counter = self.stopping_ticks
        self._hold_frozen = False          # the hold that follows captures where the arm is NOW, once
        self._ramp_errors = errors
        self._ramp_link_lost = link_lost
        self._ramp_on = self._rk is not None
        self._ramp_seed_pending = True
        if not self._ramp_on:
            self._raise_stop_errors()

    def _raise_stop_errors(self):
        if self._ramp_link_lost:
            self.link_lost_event = True
            self._ramp_link_lost = False
        if self._ramp_errors:
            self._cmd_errors = self._ramp_errors
            self._ramp_errors = ()

    def take_command_errors(self):
        """One-shot read of the strict command checks' error keys (a tuple, () when none), for RTI.py to feed
        process_errors_tick(rti_errors=)."""
        ev = self._cmd_errors
        if ev:
            self._cmd_errors = ()
        return ev

    def _ramp_output(self, ctx):
        """One tick of the controlled stop: Ruckig's velocity interface to rest, each joint on its own (none slowed by
        another), within the RTI acceleration / jerk limits. Starts from the commanded state where there is one (a
        position stream: the generator's; a velocity stream: its speed from the measured position), else from the
        measured position and filtered speed (torque, or the rate limiter off)."""
        if self._ramp_seed_pending:
            self._ramp_seed_pending = False
            ct = self._last_motion_type
            if ct == CMD_JOINT_POSITION and self.rate_limit_enabled:
                pass                                      # (_rl_q, _rl_v, _rl_a): what the drives were commanded
            elif ct == CMD_JOINT_VELOCITY and self.rate_limit_enabled:
                self._rl_q[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
            else:
                self._rl_q[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
                self._rl_v[:] = ctx.shared_data.joint['joint_speed_filtered'][:self.joint_num]
                self._rl_a[:] = 0.0
        q = self._brake_tick()
        np.abs(self._rl_v, out=self._chk_b)
        moving = self._chk_b.max() > 1e-9
        np.abs(self._rl_a, out=self._chk_b)
        if not moving and self._chk_b.max() <= 1e-9:
            self._ramp_on = False                         # at rest: hold HERE, then the error
            self._hold_pos[:] = q
            self._hold_frozen = True
            self._raise_stop_errors()
        out = self._out
        out.mode = "pid"
        out.pos_rad = q
        out.vel_rad = self._rl_v
        out.acc_rad = self._rl_a                          # published, not fed forward
        out.acc_feedforward = False
        out.trq_nm = self._add_gravity(ctx, None)
        out.target_pos_rad = None
        out.target_vel_rad = None
        out.target_acc_rad = None
        out.target_trq_nm = None
        return out

    def _hold(self, ctx):
        """Hold the current measured position with zero velocity.

        Position hold, not zero torque — plan §8.1: dropping to zero torque would let the arm
        fall. It holds because the firmware PID tracks the position setpoint, so gravity
        feedforward is not what keeps it up; adding it just means the PID starts from the
        right torque instead of having to build it out of tracking error. Same as EXEC's hold.

        (Until 2026-08-06 this said gravity was "added downstream by dispatch". It was not —
        dispatch only converts trq_nm, so None became Current=0 on the wire.)

        The pose is captured ONCE, on the first hold tick after a stream / a stop / mode entry, and held
        stiffly (2026-10-04). Re-sampling every tick made the hold a damped float (see __init__).
        """
        if not self._hold_frozen:
            self._hold_pos[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
            self._hold_frozen = True
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
            out.pos_rad = self._limit_position(payload, ctx)
            # The drive's speed feedforward. Hard RTI: the GENERATOR's own velocity — what the commanded positions
            # really move at (2026-10-11). It was the PC's feedforward, which most PCs send as zeros: the drive then
            # made all the motion from position error alone and trailed its command by 17.5 ms on the arm, against
            # 6-7 ms in EXEC / SOFT_RTI, which hand it their trajectory's speed. SOFT_RTI keeps the PC's / the
            # advancing estimate (verified on the arm; ruckig_advancing writes its estimate into _feedforward).
            out.vel_rad = self._rl_v if (not self.soft_mode and self.rate_limit_enabled) else self._feedforward
            out.trq_nm = self._add_gravity(ctx, None)
            # Source "stream" (opt-in): the STREAM's acceleration is published AND fed forward. Otherwise the
            # generator's (ruckig / ruckig_advancing / tracker) commanded acceleration is PUBLISHED only
            # (joint_acceleration_commanded, telemetry j_ac) — never fed forward: it switches between its limits as
            # the planner chases each new target (2026-10-11). With the rate limiter OFF (no target-velocity
            # estimate, no bounded acceleration) — none either way.
            if self.dynamics_ff_source == "stream" and self.rate_limit_enabled:
                # The stream's estimate for every generator, the pass-through included. Tried and measured on the arm
                # (2026-10-12, J1): the pass-through's own in-phase acceleration was no better than none (0.98-1.07x
                # tracking) where this 8 Hz-smoothed, ~44 ms-late estimate gave 0.91x — its lag likely adds a
                # speed-proportional part that offsets some of J1's friction.
                out.acc_rad = self._stream_acceleration(ctx)
                out.acc_feedforward = True
            else:
                out.acc_rad = self._rl_a if self.rate_limit_enabled else None
                out.acc_feedforward = False
        elif ct == CMD_JOINT_VELOCITY:
            out.pos_rad = None
            out.vel_rad = self._limit_velocity(payload, q_meas, ctx)
            out.trq_nm = self._add_gravity(ctx, self._limit_torque(self._feedforward, q_meas))
            out.acc_rad = self._rl_a if self.rate_limit_enabled else None      # the limiter's (see above)
            out.acc_feedforward = False       # published only — the stream source is for position streams
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

    def _limit_position(self, q_des, ctx=None):
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
           RECOVERY: the bounds are the limits WIDENED to where the limiter is this tick
           (_effective_limits), so a joint already outside its range may stay as far out as it
           is or come back, never go further — the rule the velocity path, JOG and EXEC already
           follow. Clamping symmetrically yanked such a joint onto the limit in ONE tick: a
           joint resting 1.5 deg past it got a 1.5 deg step command (measured 2026-10-01).

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
        # Layer 1a: a target outside the soft limits is not a target — except that a joint
        # already outside may stay where it is (see RECOVERY above).
        self._effective_limits()
        tgt = self._rl_tgt
        np.clip(q_des, self._lim_lo_eff, self._lim_hi_eff, out=tgt)

        # SOFT_RTI's strict check (2026-10-11, user): a target past a soft limit — or heading for it faster than the
        # joint could still stop before it (the braking curve, on the targets' own speed) — is an ERROR: a controlled
        # stop, then J{i}:RTI_POSITION_LIMIT; not a silent clamp. Checking only "past" was too late: the generator
        # was at the limit at speed by then, and the clamp stopped it at 171 rad/s^2 on J1 (simulated; the same
        # before the check existed). The arm runs about one interval behind the targets, so refused at the curve it
        # still has the room to brake. (Hard RTI with send times: _pt_check_sample.)
        if self.soft_mode and self.last_cmd_tick != self._chk_tick:
            refused = self._soft_check_position(q_des, tgt)
            self._chk_tick = self.last_cmd_tick
            if refused:
                return self._rl_q

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

        if self.servo_generator == SERVO_TRACKER:
            # The tracker needs no external planner, so it is reachable even where Ruckig is
            # not. It still enforces every limit the Ruckig path does.
            return self._limit_position_tracker(tgt, ctx)

        # No fallback. claim_control() refuses when the limiter is enabled without a
        # working planner, so reaching here with _rk unset means something bypassed that
        # check — hold the last limited position rather than emit an unlimited setpoint.
        if self._rk is None:
            return self._rl_q
        if self.servo_generator == SERVO_PASSTHROUGH:
            return self._limit_position_passthrough(q_des, tgt, ctx)
        return self._limit_position_ruckig(tgt, ctx)

    def _effective_limits(self):
        """This tick's position bounds: the soft limits widened, per joint, to where the limiter
        IS (_rl_q). Inside the range they ARE the soft limits; outside, the joint's own position
        becomes its bound, so nothing can move it further out and nothing yanks it back in.
        The bound tightens as the joint comes back: it can never return to where it was.
        Computed once per tick, before the generator moves _rl_q, and used by EVERY position
        clamp of that tick (target, tracker look-ahead, advancing aim, output)."""
        np.minimum(self.limit_neg, self._rl_q, out=self._lim_lo_eff)
        np.maximum(self.limit_pos, self._rl_q, out=self._lim_hi_eff)

    def _refresh_tracker_constants(self):
        """Rebuild the tracker's precomputed per-tick bounds. Cold path only.

        `wn^2`, `2*wn`, `jerk*dt` and the negated accel bound are constant between
        configuration changes, and building them inline would allocate four small arrays on
        every tick — the exact hot-path allocation this module's rules forbid. Called from
        __init__ and from set_servo_generator; a limit that changed without coming through
        one of those would leave these stale, which is why nothing else may write them.
        """
        wn = self.tracker_gain
        self._trk_wn2[:] = wn * wn
        self._trk_2wn[:] = 2.0 * wn
        np.multiply(self.jerk_limit, self.delta_t, out=self._trk_jstep)
        np.negative(self._trk_jstep, out=self._trk_njstep)
        np.negative(self.accel_limit, out=self._trk_naccel)

    def _tcp_velocity_budget(self, tgt, ctx):
        """This tick's per-joint velocity budget, shrunk if the TCP cap binds (defect D).

        Returns `self.vel_limit` itself when the cap is inactive — callers rely on that
        IDENTITY (`is not self.vel_limit`) to tell "capped" from "uncapped" without a
        comparison. Otherwise returns the scaled buffer.

        Shrinks the budget so the end effector cannot exceed tcp_vel_limit along the
        direction the generator is about to travel (local tangent method; see __init__ for
        the maths). Falls through to the plain joint limits when disabled, when there is
        nothing to move toward, when the motion is pure orientation (tcp_rate ~ 0 — no
        translational consequence to bound), or when no Jacobian is available (warned once).

        Shared by both generators, so the cap cannot silently apply to one and not the other.
        """
        if self.tcp_vel_limit <= 0.0 or ctx is None:
            return self.vel_limit
        np.subtract(tgt, self._rl_q, out=self._tcp_u)
        dist = float(np.sqrt(np.dot(self._tcp_u, self._tcp_u)))
        if dist <= 1e-9:
            return self.vel_limit
        jpos = self._tcp_jacobian_pos(ctx)
        if jpos is None:
            return self.vel_limit
        self._tcp_u /= dist
        vt = jpos @ self._tcp_u              # (3,) — small alloc, cap-enabled only
        tcp_rate = float(np.sqrt(np.dot(vt, vt)))
        if tcp_rate <= 1e-9:
            return self.vel_limit
        s = self.tcp_vel_limit / tcp_rate    # allowed path speed, rad/s
        np.abs(self._tcp_u, out=self._tcp_scaled_vmax)
        self._tcp_scaled_vmax *= s
        np.minimum(self._tcp_scaled_vmax, self.vel_limit, out=self._tcp_scaled_vmax)
        # Ruckig rejects a non-positive max_velocity; 1 mrad/s is far below anything the arm
        # can resolve, so the floor never binds physically.
        np.maximum(self._tcp_scaled_vmax, 1e-3, out=self._tcp_scaled_vmax)
        return self._tcp_scaled_vmax

    def _update_target_velocity(self, tgt):
        """Estimate how fast the PC's target is moving, and how often it arrives.

        Shared by every generator, because two of them need it and a third (duration
        pacing in AUTO) needs only the interval. Keeping one copy means the two consumers
        cannot disagree about what "the command rate" is.

        THE POINT, and the easy thing to get wrong: the derivative is taken over the REAL
        arrival interval, never over a tick. Targets 12.5 ticks apart carry 50 ms of travel;
        dividing that by the 4 ms tick overestimates the speed 12.5x.

        Idempotent within a tick — it only does work when a NEW command has arrived, so
        calling it from more than one generator path is safe.
        """
        if self.last_cmd_tick == self._trk_cmd_tick:
            return
        gap = self.last_cmd_tick - self._trk_cmd_tick
        dts = self._cmd_ts - self._ts_prev        # nan unless this AND the previous packet carried a send time
        if self._trk_seeded and gap > 0 and 0.0 < dts <= TS_MAX_INTERVAL_S:
            # TIMESTAMPED: the PC said when each target was taken, so the velocity is taken over the TRUE interval
            # and the interval estimate (duration pacing, the aim point's cap) is the PC's real rate — not the
            # arrival ticks an uneven sender scatters. Dropped packets are counted from the sequence numbers.
            spans = (self.seq_rx_last - self._ts_prev_seq) & 0xFFFFFFFF
            if spans < 1 or spans > TS_MAX_SPANS:
                spans = 1
            self._trk_gaps.append(dts / (spans * self.delta_t))
            if len(self._trk_gaps) > GAP_WINDOW:
                del self._trk_gaps[0]
            self._trk_gap = sum(self._trk_gaps) / len(self._trk_gaps)
            np.subtract(tgt, self._trk_prev_tgt, out=self._trk_vtgt)
            self._trk_vtgt *= 1.0 / dts
            np.clip(self._trk_vtgt, -self.vel_limit, self.vel_limit, out=self._trk_vtgt)
            self._trk_interval_s = dts
        elif self._trk_seeded and gap > 0:
            # MEDIAN of recent gaps, not this one. The instantaneous gap is an integer
            # number of ticks, so a PC rate that is not an exact divisor of the loop rate
            # cannot be represented by it: 20 Hz is due every 12.5 ticks, so arrivals
            # alternate 12, 13, 12, 13. Dividing by that alternates the velocity estimate
            # by +/-4% and alternates the extrapolation horizon with it, which lands
            # straight in ruckig_advancing's aim point as a stutter. The median of
            # [12,13,12,13] is exactly 12.5.
            #
            # Median rather than mean because a DROPPED command doubles one gap, and a mean
            # would carry that outlier into the estimate for the whole window.
            #
            # This also removes the need for the PC to declare its rate. A declared rate
            # would fix the integer-ratio case but not the two cases that actually matter:
            # a sender that does not know its own rate, and one whose rate is not stable.
            # Measuring and smoothing handles all three.
            # How many command intervals this gap spans: 1 normally, 2+ when a command was
            # DROPPED. Both the estimator and the velocity divide by it, so a drop reads as
            # one long interval rather than as a doubled target velocity.
            # The FIRST interval has nothing to compare against, so it must be taken at face
            # value and seed the estimator. Deriving spans from the initial _trk_gap instead
            # self-locks: seeded at 1.0, a 12-tick gap reads as 12 spans of 1 tick, stores
            # 1.0, and the estimate never leaves 1.0 -- making spans*est reproduce the raw
            # gap and silently disabling all of this.
            if not self._trk_gaps:
                spans = 1
            elif gap > SPAN_MARGIN * self._trk_gap:
                spans = max(2, int(round(gap / self._trk_gap)))
            else:
                spans = 1
            self._trk_gaps.append(gap / spans)
            if len(self._trk_gaps) > GAP_WINDOW:
                del self._trk_gaps[0]
            # MEAN of recent gaps, not a median and not a long-baseline slope. Both were
            # tried and both measured WORSE:
            #   * a MEDIAN is right only for half-integer ratios -- at 12.5 ticks the gaps
            #     split 12/13 evenly and it lands on 12.5, but at 8.33 they split two-thirds
            #     8 to one-third 9 and it returns 8, a 4% overestimate.
            #   * a SLOPE over a long baseline should in theory beat both, and does not: it
            #     settles too slowly and degraded even the exact rates (0.004% -> 0.017%).
            # The window length is load-bearing -- see GAP_WINDOW.
            self._trk_gap = sum(self._trk_gaps) / len(self._trk_gaps)
            np.subtract(tgt, self._trk_prev_tgt, out=self._trk_vtgt)
            self._trk_vtgt *= 1.0 / (spans * self._trk_gap * self.delta_t)
            # A stale target the PC never updated must not become a velocity estimate that
            # keeps extrapolating forever.
            np.clip(self._trk_vtgt, -self.vel_limit, self.vel_limit, out=self._trk_vtgt)
            self._trk_interval_s = spans * self._trk_gap * self.delta_t
        else:
            spans = 1
            self._trk_vtgt[:] = 0.0
            self._trk_seeded = True
        self._trk_prev_tgt[:] = tgt
        self._trk_cmd_tick = self.last_cmd_tick
        if self._cmd_ts == self._cmd_ts:
            # A send time: the target was valid at the PC's send time plus the FASTEST delivery seen (the offset
            # between the two clocks; later packets were merely delayed, and the aim point then extrapolates over
            # the delay). No clock synchronisation: only this offset is estimated, forgotten slowly for drift, and
            # re-anchored when the PC's clock jumps.
            sample = self.last_cmd_tick - self._cmd_ts / self.delta_t
            if not (self._ts_off == self._ts_off) or sample > self._ts_off + TS_RESYNC_TICKS:
                self._ts_off = sample
            else:
                self._ts_off += TS_OFFSET_LEAK_TICKS     # forget first, so a delivery as fast as ever re-anchors
                if sample < self._ts_off:                # exactly on itself (a steady sender: the arrival tick)
                    self._ts_off = sample
            self._trk_ideal = self._cmd_ts / self.delta_t + self._ts_off
            self._trk_ideal_seeded = True
        else:

            # The IDEAL fractional tick this command was meant for, which is NOT the tick it
            # arrived on. A 60 Hz command is due every 4.1667 ticks but can only ARRIVE on a
            # whole tick, so it is already 0-1 ticks stale by an amount that cycles. Ageing the
            # aim point from the arrival tick throws that away and re-quantises the extrapolation
            # every command -- the residual roughness that survived fixing the velocity estimate.
            # Advancing a fractional clock instead keeps the aim point continuous.
            if self._trk_ideal_seeded:
                self._trk_ideal += spans * self._trk_gap
                # Re-sync if the PC's true rate differs from our estimate, or its clock drifts:
                # never let the ideal time wander more than one interval from reality.
                drift = self.last_cmd_tick - self._trk_ideal
                if drift > self._trk_gap or drift < -self._trk_gap:
                    self._trk_ideal = float(self.last_cmd_tick)
            else:
                self._trk_ideal = float(self.last_cmd_tick)
                self._trk_ideal_seeded = True
        self._ts_prev = self._cmd_ts
        self._ts_prev_seq = self.seq_rx_last

        # The PC's OWN velocity, when it sent one, beats anything derived here. A VR headset
        # or haptic controller measures hand velocity directly; our estimate is a two-point
        # difference over its position samples -- noisier, and lagged by half an interval.
        # Used by the advancing generator; zero feedforward means "no claim", so we fall
        # back to the derived estimate.
        self._trk_vtgt_from_pc = bool(np.any(self._feedforward))

    def _auto_pace_budget(self, vcap):
        """Shrink this tick's velocity budget to a multiple of the TARGET's own speed.

        The self-tuning form of the velocity cap, and the answer to the one real objection
        to `set_vel_scale`: that the operator has to know their own travel speed in rad/s.
        They do not — the robot already measures how fast the target is moving
        (`_update_target_velocity`), so it can derive the cap itself and follow a PC that
        changes speed or rate mid-session.

        WHY A VELOCITY CAP AND NOT A DURATION FLOOR. The obvious alternative was a
        minimum trajectory execution time (soft real-time servo interfaces offer one) -> Ruckig's `minimum_duration`.
        Measured: it is a NO-OP for us. Ruckig satisfies a duration floor by padding the END
        of the trajectory — the first tick's velocity is bit-identical with the floor at
        0.0, 0.2 and 0.5 s. Since we re-plan every tick and consume only the first tick, the
        floor can never reach our output. It works for interfaces that plan
        once and execute to completion (plan + sample); it cannot work in a per-tick
        re-planning architecture. Capping velocity is the mechanism that survives.

        `pace_factor` is the headroom multiplier. 1.0 would let the generator move exactly as
        fast as the target and never catch up after a disturbance, so the default leaves
        margin. Below the floor the cap is ignored: a target that is momentarily stationary
        (a turning point, or the PC holding still) must not pin the budget at zero.
        """
        if self.pace_factor <= 0.0 or not self._trk_seeded:
            return vcap
        np.abs(self._trk_vtgt, out=self._pace_v)
        self._pace_v *= self.pace_factor
        # Floor, then never above whatever budget the caller already computed — so the TCP
        # cap and the joint limits still win. min(), never max().
        np.maximum(self._pace_v, PACE_FLOOR_RAD_S, out=self._pace_v)
        np.minimum(self._pace_v, vcap, out=self._pace_v)
        return self._pace_v

    def _limit_position_tracker(self, tgt, ctx=None):
        """Servo generator B: a servo-style tracking law (plan / servo_mode_options §3b).

        The difference from Ruckig in one sentence: **this never plans to stop.** Ruckig is a
        point-to-point planner, so on a sparse stream it sprints to each target and idles
        until the next one, and that stop/sprint cycle is the ripple measured in
        TODO/servo_mode_options.md §5.1. A tracker has no arrival phase to express, so the
        mechanism does not exist.

        The law is a critically-damped second-order tracker on an extrapolated target:

            q_ahead = tgt + v_tgt * lookahead_time
            a       = wn^2 * (q_ahead - q) - 2*wn*v          wn = tracker_gain
            v      += a*dt ;  q += v*dt

        Both knobs are real and independent, which is why they are kept separate rather than
        collapsed into one:

          * `tracker_gain` (wn, rad/s) is the tracking BANDWIDTH — how hard it pulls. Raising
            it tightens tracking and passes more of the command's own noise through.
          * `tracker_lookahead` (s) cancels the steady-state ramp lag. A critically damped
            second-order tracker following a ramp at speed v trails by exactly 2v/wn, so
            extrapolating the target forward by 2/wn cancels that lag analytically rather
            than by tuning. That relationship is why servo interfaces commonly couple the two knobs, and it is the
            default this handler picks (see set_servo_generator).

        `v_tgt` is estimated across the REAL arrival interval, never across a tick — targets
        12.5 ticks apart carry 50 ms of travel, and dividing that by 4 ms overestimates the
        speed 12.5x. The estimate is deliberately tolerant here in a way it could not be for
        Ruckig: it feeds an extrapolation that the second-order filter then smooths, rather
        than a hard waypoint velocity the planner must satisfy exactly (§5.3).

        WHAT THIS LAW IS NOT GOOD AT, stated plainly: it is a servo, not a planner. Given a
        large step it saturates on acceleration and can overshoot, where Ruckig arrives
        exactly and never overshoots. Use Ruckig for point-to-point moves and this for
        streams — that is the whole reason both are selectable rather than one replacing the
        other. The soft joint limits are clamped unconditionally either way.
        """
        dt = self.delta_t

        self._update_target_velocity(tgt)

        # --- extrapolate, then pull ---
        # The aim point ADVANCES between commands. This is the difference between tracking a
        # moving target and tracking a sequence of stationary ones: with a fixed horizon the
        # aim point is constant for the whole 200 ms between two 5 Hz commands, so the
        # tracker converges to it and decelerates — reproducing Ruckig's arrive-and-stop in
        # miniature. Measured: 148.6% ripple at 5 Hz with a fixed horizon, 3.7% once the
        # horizon grows with the age of the last command.
        #
        # BOUNDED at one command interval. In steady streaming that exactly covers the gap;
        # if the PC goes quiet the extrapolation stops growing after one interval and the
        # tracker coasts to the projected point and holds, instead of running away on a
        # velocity estimate nobody has confirmed.
        age = ctx.current_tick - self._trk_ideal if ctx is not None else 0.0
        if age > self._trk_gap:
            age = self._trk_gap
        elif age < 0:
            age = 0.0        # never extrapolate BACKWARDS, whatever the clocks say
        horizon = self.tracker_lookahead + age * dt

        ahead = self._trk_ahead
        np.multiply(self._trk_vtgt, horizon, out=ahead)
        np.add(ahead, tgt, out=ahead)
        # The other common meaning of "lookahead", which is NOT the same mechanism as ours despite the
        # shared name: project the robot's CURRENT POSITION forward in time with its current velocity (a
        # high value prevents overshoot) — the robot's own state, so it adds damping. Ours projects the TARGET, which cancels lag and CAUSES overshoot on a
        # stop. Measured frontier in servo_mode_options.md §5.5; default 0 because at our
        # bandwidth it costs more tracking than the overshoot it buys back.
        if self.tracker_self_lookahead > 0.0:
            np.multiply(self._rl_v, self.tracker_self_lookahead, out=self._trk_da)
            np.subtract(ahead, self._trk_da, out=ahead)
        # Extrapolation must not aim outside the workspace, or the tracker would drive at a
        # target the output clamp then refuses — the classic wind-up against a pinned output.
        np.clip(ahead, self._lim_lo_eff, self._lim_hi_eff, out=ahead)

        # a = wn^2 * (ahead - q) - 2*wn*v, written as two fused ops rather than four so the
        # per-call NumPy overhead on 6-element arrays does not dominate the arithmetic.
        a = self._trk_a
        np.subtract(ahead, self._rl_q, out=a)
        np.multiply(a, self._trk_wn2, out=a)
        np.multiply(self._rl_v, self._trk_2wn, out=self._trk_da)
        np.subtract(a, self._trk_da, out=a)

        # Jerk, then acceleration. Jerk first so the acceleration clamp is the final word.
        # Both bounds are PRECOMPUTED: `-self.accel_limit` and `self.jerk_limit * dt` each
        # build a fresh array, and three of those per tick is exactly the hot-path
        # allocation this file's rules forbid.
        np.subtract(a, self._rl_a, out=self._trk_da)
        np.clip(self._trk_da, self._trk_njstep, self._trk_jstep, out=self._trk_da)
        np.add(self._rl_a, self._trk_da, out=a)
        np.clip(a, self._trk_naccel, self.accel_limit, out=a)

        # --- integrate, bounded at every step ---
        v = self._trk_v
        np.multiply(a, dt, out=v)
        np.add(v, self._rl_v, out=v)
        vcap = self._tcp_velocity_budget(tgt, ctx)
        np.negative(vcap, out=self._trk_nvcap)
        np.clip(v, self._trk_nvcap, vcap, out=v)
        # The soft-limit brake, same bound the velocity command path uses. Unconditional:
        # a tracker with no arrival phase has nothing else stopping it at a joint limit.
        self._brake_at_limits(v, self._rl_q)

        out = self._rl_out
        np.multiply(v, dt, out=out)
        np.add(out, self._rl_q, out=out)
        np.clip(out, self._lim_lo_eff, self._lim_hi_eff, out=out)

        # Re-derive the state actually achieved after clamping, so the next tick integrates
        # from the truth rather than from what was requested. Skipping this is how a limiter
        # winds up against its own clamp.
        np.subtract(out, self._rl_q, out=self._trk_da)
        self._trk_da /= dt
        np.subtract(self._trk_da, self._rl_v, out=self._rl_a)
        self._rl_a /= dt
        self._rl_v[:] = self._trk_da
        self._rl_q[:] = out
        return out

    def _limit_position_ruckig(self, tgt, ctx=None):
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
        self._update_target_velocity(tgt)   # pacing and the advancing aim both need this

        inp = self._rk_in
        inp.current_position = self._rl_q
        inp.current_velocity = self._rl_v
        inp.current_acceleration = self._rl_a

        if self.servo_generator == SERVO_ADVANCING:
            # Aim at where the target IS, and tell the planner to PASS THROUGH at the
            # target's own speed instead of stopping on it. Same planner, same limits —
            # only the boundary condition changes, from an intercept-and-stop to a
            # rendezvous. That is the whole difference, and it is worth ~4.3 deg at 5 Hz.
            #
            # Ruckig is a Type V generator (arbitrary target states); the shipped
            # `ruckig` mode uses it as a Type IV one by always asking for target_velocity
            # = 0. This mode simply uses the capability we already have.
            age = ctx.current_tick - self._trk_ideal if ctx is not None else 0.0
            if age > self._trk_gap:
                age = self._trk_gap
            elif age < 0:
                age = 0.0
            adv = self._trk_ahead
            np.multiply(self._trk_vtgt, age * self.delta_t, out=adv)
            np.add(adv, tgt, out=adv)
            np.clip(adv, self._lim_lo_eff, self._lim_hi_eff, out=adv)
            inp.target_position = adv
            aim = adv
            # Feedforward: the PC's own velocity WINS when it sent one. An earlier version
            # overwrote it unconditionally, reasoning that a measured estimate beats "what
            # the PC claims" — backwards. A VR headset or haptic controller MEASURES hand
            # velocity; our number is a two-point difference over its position samples, so
            # it is noisier and lagged by half an interval. Derive only when the PC is
            # silent on the subject.
            if not self._trk_vtgt_from_pc:
                self._feedforward[:] = self._trk_vtgt
        else:
            inp.target_position = tgt
            aim = tgt

        # Duration pacing (a minimum trajectory execution time). Force the
        # planned move to last at least `min_duration_mult` command intervals.
        #
        # WHY A MULTIPLE AND NOT A TIME, and why > 1 is the whole point: at exactly 1x the
        # trajectory ends as the next command lands, so its deceleration phase DOES execute
        # and the velocity still returns to zero — a stretched bump. Above ~1.5x the plan
        # always outlives the next command, so the deceleration is perpetually deferred and
        # never runs. That is what turns the bump into a glide (measured: velocity swing
        # 0.874 -> 0.000 rad/s at 5 Hz). Expressed as a multiple because the useful value is
        # relative to the PC's rate, which the robot already measures.
        if self.min_duration_mult > 0.0 and self._trk_seeded:
            inp.minimum_duration = (self.min_duration_mult * self._trk_gap * self.delta_t)
            self._min_dur_applied = True
        elif self._min_dur_applied:
            inp.minimum_duration = 0.0        # release once, not every tick
            self._min_dur_applied = False

        vcap = self._tcp_velocity_budget(tgt, ctx)
        vcap = self._auto_pace_budget(vcap)
        if vcap is not self.vel_limit:
            inp.max_velocity = vcap
            self._tcp_cap_applied = True
        elif self._tcp_cap_applied:
            # Cap released (disabled, at target, or Jacobian lost): restore the base budget
            # once rather than re-sending it every tick.
            inp.max_velocity = self.vel_limit
            self._tcp_cap_applied = False

        # Feedforward is the PC's own derivative and it is not validated anywhere upstream,
        # so it is clamped before it becomes a planner input — Ruckig rejects a target
        # velocity above max_velocity outright, and that would be a hot-path exception.
        # Clamped against the ACTIVE budget: when the TCP cap has shrunk max_velocity, a
        # feedforward clipped only to the base limits would still be rejected.
        np.clip(self._feedforward, -vcap, vcap, out=self._rk_tv)
        # Never ask to ARRIVE at a soft limit at speed (2026-10-11): the target speed toward a limit at most what the
        # joint can still stop from at the target (the braking curve: 0 on the limit). Without it a stream into a
        # limit made the planner reach the clamped target at speed and the clamp stopped it — SOFT_RTI, J1 at 60 deg/s,
        # 50 Hz, pacing off: 167 rad/s^2 (simulated; the same before the strict checks). Far from a limit: unchanged.
        self._brake_at_limits(self._rk_tv, aim)
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
        np.clip(out, self._lim_lo_eff, self._lim_hi_eff, out=out)
        self._rl_q[:] = out
        return out

    def _pt_reset(self, current_tick):
        """Empty the pass-through ring (a claim, a generator switch): the next three stamped packets start it again."""
        self._pt_k = 0
        self._pt_shift = 0.0
        self._pt_last_tick = current_tick
        self._pt_restore_rk()

    def _pt_restore_rk(self):
        """Give Ruckig's input back to the planner path (its own synchronisation / duration logic, no target
        acceleration) — once, when the pass-through hands over."""
        if not self._pt_rk_cfg or self._rk is None:
            return
        inp = self._rk_in
        inp.synchronization = Synchronization.Time if self.min_duration_mult > 0.0 else Synchronization.No
        inp.minimum_duration = 0.0
        self._min_dur_applied = False          # the planner re-applies its pacing itself, when there is one
        inp.target_acceleration = self._zeros
        self._pt_rk_cfg = False

    def _brake_tick(self):
        """One tick of braking straight to rest from (_rl_q, _rl_v, _rl_a), time-optimally within each joint's limits
        (Ruckig's velocity interface, joints independent). The controlled stop's step. The speed never exceeded what
        each joint can stop from before its soft limit (the braking curve, in every position path), so this stop is
        clear of the limits; the clamp is the backstop."""
        self._effective_limits()
        inp = self._rk_in
        inp.control_interface = ControlInterface.Velocity
        inp.synchronization = Synchronization.No
        inp.minimum_duration = 0.0
        inp.current_position = self._rl_q
        inp.current_velocity = self._rl_v
        inp.current_acceleration = self._rl_a
        inp.target_velocity = self._zeros
        inp.target_acceleration = self._zeros
        try:
            self._rk.update(inp, self._rk_out)
            failed = False
        except Exception:
            failed = True
        inp.control_interface = ControlInterface.Position
        self._pt_rk_cfg = True                         # the next planner / pass-through tick resets its own settings
        self._pt_restore_rk()
        if failed:
            self._rk_errors += 1
            self._rl_v[:] = 0.0
            self._rl_a[:] = 0.0
            return self._rl_q
        out = self._rl_out
        out[:] = self._rk_out.new_position
        self._rl_v[:] = self._rk_out.new_velocity
        self._rl_a[:] = self._rk_out.new_acceleration
        np.clip(out, self._lim_lo_eff, self._lim_hi_eff, out=out)
        self._rl_q[:] = out
        return out

    def _pt_check_sample(self, k):
        """The strict checks on the newest stamped sample (k samples in the ring). True: refused — a controlled stop
        has begun with the joints' error keys. Divided differences over the TRUE send-time intervals: speed (2
        samples), acceleration (3), jerk (4). Hot path when clean: preallocated arrays only."""
        dt = self.delta_t
        i3 = (k - 1) % PT_RING
        q3, s3 = self._pt_q[i3], self._pt_s[i3]
        m = self._chk_m
        # position: outside the (recovery-widened) soft limits
        v, a, jj, b = self._chk_v, self._chk_a, self._chk_j, self._chk_b
        np.subtract(self._lim_lo_eff, CMD_CHECK_POS_EPS, out=b)
        np.less(q3, b, out=m)
        bad_pos = bool(m.any())
        if not bad_pos:
            np.add(self._lim_hi_eff, CMD_CHECK_POS_EPS, out=b)
            np.greater(q3, b, out=m)
            bad_pos = bool(m.any())
        bad_v = bad_a = bad_j = False
        if k >= 2:
            i2 = (k - 2) % PT_RING
            s2 = self._pt_s[i2]
            np.subtract(q3, self._pt_q[i2], out=v)
            v *= 1.0 / ((s3 - s2) * dt)
            np.abs(v, out=b)
            np.greater(b, self._chk_vlim, out=m)
            bad_v = bool(m.any())
            # ... and toward a limit no faster than the joint can still stop before it (the braking curve)
            b[:] = v
            self._brake_at_limits(b, q3)
            np.subtract(v, b, out=b)
            np.abs(b, out=b)
            np.greater(b, CMD_CHECK_BRAKE_TOL, out=m)
            if m.any():
                bad_pos = True
            if k >= 3:
                i1 = (k - 3) % PT_RING
                s1 = self._pt_s[i1]
                np.subtract(self._pt_q[i2], self._pt_q[i1], out=jj)
                jj *= 1.0 / ((s2 - s1) * dt)                     # the previous segment's speed
                np.subtract(v, jj, out=a)
                a *= 2.0 / ((s3 - s1) * dt)                      # acceleration (the second divided difference x 2)
                np.abs(a, out=b)
                np.greater(b, self._chk_alim, out=m)
                bad_a = bool(m.any())
                if k >= 4 and not bad_a:
                    i0 = (k - 4) % PT_RING
                    s0 = self._pt_s[i0]
                    np.subtract(self._pt_q[i1], self._pt_q[i0], out=b)
                    b *= 1.0 / ((s1 - s0) * dt)                  # the speed before that
                    np.subtract(jj, b, out=b)
                    b *= 2.0 / ((s2 - s0) * dt)                  # the previous acceleration
                    np.subtract(a, b, out=jj)
                    jj *= 3.0 / ((s3 - s0) * dt)                 # jerk (the third divided difference x 6)
                    np.abs(jj, out=b)
                    np.greater(b, self._chk_jlim, out=m)
                    bad_j = bool(m.any())
        if not (bad_pos or bad_v or bad_a or bad_j):
            return False
        return self._refuse_command(q3, v, a, jj, bad_pos, bad_v, bad_a, bad_j)

    def _refuse_unstamped(self):
        """Cold path: a hard-RTI position target without the PC's send time — a controlled stop, then
        RTI_NO_SEND_TIME (once: the stop has begun, later packets are not read)."""
        if self.sub_state != 'control_active':
            return
        logger.error("[RTI] Hard-RTI position command without the PC's send time (a 101-byte packet, or one not finite) — refused, "
                     "braking to rest. Hard RTI follows the PC's own timing: keep send times on (RTIClient's "
                     "default), or use SOFT_RTI (rti_connect with a rate) for a sender without them.")
        self._begin_stop('stopping_error', errors=("RTI_NO_SEND_TIME",))

    def _refuse_command(self, q, v, a, jerk, bad_pos, bad_v, bad_a, bad_j):
        """Cold path: name each offending joint, log it, begin the controlled stop with the error keys."""
        keys, why = [], []
        lim = (self._vel_limit_base * (1.0 + CMD_CHECK_TOL), self.accel_limit * (1.0 + CMD_CHECK_TOL),
               self.jerk_limit * (1.0 + CMD_CHECK_TOL))
        for j in range(self.joint_num):
            if bad_pos and (q[j] < self._lim_lo_eff[j] - CMD_CHECK_POS_EPS or q[j] > self._lim_hi_eff[j]
                            + CMD_CHECK_POS_EPS or self._too_fast_for_limit(j, q[j], v[j])):
                keys.append(f"J{j + 1}:RTI_POSITION_LIMIT")
                why.append(f"J{j + 1} target {np.degrees(q[j]):.2f} deg at {np.degrees(v[j]):.1f} deg/s: past "
                           f"[{np.degrees(self.limit_neg[j]):.1f}, {np.degrees(self.limit_pos[j]):.1f}] deg or "
                           f"unable to stop before it")
            if bad_v and abs(v[j]) > lim[0][j]:
                keys.append(f"J{j + 1}:RTI_VELOCITY_LIMIT")
                why.append(f"J{j + 1} speed {v[j]:.2f} rad/s > {self._vel_limit_base[j]:.2f}")
            if bad_a and abs(a[j]) > lim[1][j]:
                keys.append(f"J{j + 1}:RTI_ACCELERATION_LIMIT")
                why.append(f"J{j + 1} acceleration {a[j]:.1f} rad/s^2 > {self.accel_limit[j]:.1f}")
            if bad_j and abs(jerk[j]) > lim[2][j]:
                keys.append(f"J{j + 1}:RTI_JERK_LIMIT")
                why.append(f"J{j + 1} jerk {jerk[j]:.0f} rad/s^3 > {self.jerk_limit[j]:.0f}")
        if not keys:                                       # the masks said so; name the first joint generically
            keys.append("J1:RTI_POSITION_LIMIT" if bad_pos else "J1:RTI_VELOCITY_LIMIT")
        logger.error("[RTI] Command refused (%s) — braking to rest: %s", ", ".join(keys), "; ".join(why))
        self._begin_stop('stopping_error', errors=tuple(keys))
        return True

    def _soft_check_position(self, q_des, tgt):
        """SOFT_RTI: the new target past a soft limit, or its speed (from the previous target, over the send-time
        interval when both carry one, else the arrival interval) above the braking curve at it. True: refused."""
        m, v, b = self._chk_m, self._chk_v, self._chk_b
        np.subtract(q_des, tgt, out=b)
        np.abs(b, out=b)
        np.greater(b, CMD_CHECK_POS_EPS, out=m)
        bad_pos = bool(m.any())
        dts = self._cmd_ts - self._chk_ts_prev
        if 0.0 < dts <= TS_MAX_INTERVAL_S:
            interval = dts
        else:
            interval = (self.last_cmd_tick - self._chk_tick) * self.delta_t if self._chk_t_prev == self._chk_t_prev \
                else 0.0
        v[:] = 0.0
        if not bad_pos and interval > 0.0:
            np.subtract(q_des, self._chk_q_prev, out=v)
            v *= 1.0 / interval
            b[:] = v
            self._brake_at_limits(b, q_des)
            np.subtract(v, b, out=b)
            np.abs(b, out=b)
            np.greater(b, CMD_CHECK_BRAKE_TOL, out=m)
            bad_pos = bool(m.any())
        if not bad_pos:
            # ... and the generator itself (ruckig_advancing aims AHEAD of the newest target by up to an interval)
            b[:] = self._rl_v
            self._brake_at_limits(b, self._rl_q)
            np.subtract(self._rl_v, b, out=b)
            np.abs(b, out=b)
            np.greater(b, CMD_CHECK_BRAKE_TOL, out=m)
            bad_pos = bool(m.any())
        self._chk_q_prev[:] = q_des
        self._chk_t_prev = 0.0
        self._chk_ts_prev = self._cmd_ts
        if not bad_pos:
            return False
        self._chk_a[:] = 0.0
        return self._refuse_command(q_des, v, self._chk_a, self._chk_a, True, False, False, False)

    def _too_fast_for_limit(self, j, q, v):
        """Cold path: is joint j's speed v at q faster (with the tolerance) than it can still stop from before its
        soft limit? (the braking curve, one joint)"""
        self._chk_b[:] = 0.0
        self._chk_b[j] = v
        self._brake_at_limits(self._chk_b, np.full(self.joint_num, q))
        return abs(v - self._chk_b[j]) > CMD_CHECK_BRAKE_TOL

    def _limit_position_passthrough(self, q_des, tgt, ctx):
        """Hard RTI's generator (see SERVO_PASSTHROUGH / PT_*): the PC's stamped samples replayed a small adaptive
        delay behind their send times, on a quadratic through the last three, each joint landing exactly on it when
        its own one-tick landing is within the limits, else catching up. `q_des` the PC's target (unclamped: the
        curve is the PC's), `tgt` the same clamped. Until three samples are in, the plain ruckig tick (hard RTI refuses a
        position without a send time since 2026-10-12 — the old planner is no longer a fallback for one)."""
        self._update_target_velocity(tgt)                  # the stream's velocity / interval: the stream feedforward
        dt = self.delta_t
        if self.last_cmd_tick != self._pt_last_tick:       # a new packet
            self._pt_last_tick = self.last_cmd_tick
            if self._cmd_ts == self._cmd_ts:              # always, in hard RTI (an unstamped position is refused)
                s = self._cmd_ts / dt - self._pt_shift
                k = self._pt_k
                if k > 0:
                    s_prev = self._pt_s[(k - 1) % PT_RING]
                    ds = s - s_prev
                    if not (0.0 < ds <= PT_MAX_GAP_TICKS):
                        arr_gap = self.last_cmd_tick - self._pt_arrival
                        if 0 < arr_gap <= PT_MAX_GAP_TICKS:
                            # The PC's CLOCK jumped (the packets kept coming): carry its timeline on at the arrival
                            # spacing. A restart would hand two ticks to the planner, which brakes — ~1 deg lost at
                            # 60 deg/s (simulated). That spacing is a guess: the strict checks skip it.
                            self._pt_shift += ds - arr_gap
                            s = s_prev + arr_gap
                            self._pt_skip_checks = 4
                        else:
                            k = 0                          # a silence: start again from this sample
                self._pt_arrival = self.last_cmd_tick
                i = k % PT_RING
                self._pt_s[i] = s
                self._pt_q[i] = q_des
                self._pt_k = k + 1
                delay = self.last_cmd_tick - s
                if k == 0:
                    self._pt_L = self._pt_Lt = delay
                elif delay > self._pt_Lt - PT_DELAY_DECAY_TICKS:
                    self._pt_Lt = delay
                else:
                    self._pt_Lt -= PT_DELAY_DECAY_TICKS
                if self._pt_skip_checks > 0:
                    self._pt_skip_checks -= 1
                elif self._pt_check_sample(k + 1):
                    return self._rl_q                      # refused: a controlled stop has begun (see _step_active)
        k = self._pt_k
        if k < 3:
            self._pt_restore_rk()
            return self._limit_position_ruckig(tgt, ctx)

        # The replay clock: L moves toward the slowest recent delivery at most PT_DELAY_RATE_TICKS per tick; the
        # time warp it makes is in the target speed (warp) so position and speed stay one curve.
        dl = self._pt_Lt - self._pt_L
        if dl > PT_DELAY_RATE_TICKS:
            dl = PT_DELAY_RATE_TICKS
        elif dl < -PT_DELAY_RATE_TICKS:
            dl = -PT_DELAY_RATE_TICKS
        self._pt_L += dl
        warp = 1.0 - dl
        r = ctx.current_tick - self._pt_L

        # The segment [B, C] holding r (newest first; past the newest -> the newest segment), A the sample before B.
        n = k if k < PT_RING else PT_RING
        seg = n - 2
        for j in range(n - 1):
            if self._pt_s[(k - 2 - j) % PT_RING] <= r:
                seg = j
                break
        i_c = (k - 1 - seg) % PT_RING
        i_b = (k - 2 - seg) % PT_RING
        t_b = self._pt_s[i_b]
        t_c = self._pt_s[i_c]
        rr = t_b if r < t_b else r                         # past the newest: the same curve, until the watchdog
        fbc, fab, fabc = self._pt_fbc, self._pt_fab, self._pt_fabc
        np.subtract(self._pt_q[i_c], self._pt_q[i_b], out=fbc)
        fbc *= 1.0 / (t_c - t_b)                            # rad per tick
        gate = self._pt_gate
        if seg + 2 < n:
            i_a = (k - 3 - seg) % PT_RING
            t_a = self._pt_s[i_a]
            np.subtract(self._pt_q[i_b], self._pt_q[i_a], out=fab)
            fab *= 1.0 / (t_b - t_a)
            np.subtract(fbc, fab, out=fabc)
            fabc *= 1.0 / (t_c - t_a)                       # half the curvature, rad per tick^2
            np.abs(fabc, out=self._pt_vb)
            self._pt_vb *= 2.0 / (dt * dt)
            np.less_equal(self._pt_vb, PT_GATE * self.accel_limit, out=gate)
        else:
            gate[:] = False
        np.multiply(fabc, gate, out=fabc)                  # a jump: the straight segment, stop on it

        aim_raw, aim, v, a = self._pt_aim_raw, self._pt_aim, self._pt_v, self._pt_a
        # p(rr) = q_B + f_BC (rr - t_B) + f_ABC (rr - t_B)(rr - t_C);  p' and p'' with the warp
        np.multiply(fabc, (rr - t_b) * (rr - t_c), out=aim_raw)
        np.multiply(fbc, rr - t_b, out=v)                  # v as scratch
        aim_raw += v
        aim_raw += self._pt_q[i_b]
        np.multiply(fabc, 2.0 * rr - t_b - t_c, out=v)
        v += fbc
        v *= warp / dt
        np.multiply(fabc, 2.0 * warp * warp / (dt * dt), out=a)
        np.multiply(v, gate, out=v)
        # Never past a soft limit: the aim clamped (and its speed 0 there), and the speed never more than the joint
        # can still stop from before its limit.
        np.clip(aim_raw, self._lim_lo_eff, self._lim_hi_eff, out=aim)
        np.not_equal(aim, aim_raw, out=self._pt_mask)
        np.copyto(v, 0.0, where=self._pt_mask)
        np.copyto(a, 0.0, where=self._pt_mask)
        self._pt_vb[:] = v
        self._brake_at_limits(self._pt_vb, aim)
        np.not_equal(self._pt_vb, v, out=self._pt_mask)
        np.copyto(a, 0.0, where=self._pt_mask)
        v[:] = self._pt_vb
        np.clip(a, -self.accel_limit, self.accel_limit, out=a)

        inp = self._rk_in
        vcap = self._tcp_velocity_budget(aim, ctx)
        vcap = self._auto_pace_budget(vcap)
        if vcap is not self.vel_limit:
            inp.max_velocity = vcap
            self._tcp_cap_applied = True
        elif self._tcp_cap_applied:
            inp.max_velocity = self.vel_limit
            self._tcp_cap_applied = False
        np.clip(v, -vcap, vcap, out=v)
        # Every tick, not once: a setter (rti_set_min_duration, sent by every servo test's config) may have changed
        # them since — and under Synchronization.No the one-tick minimum duration does not hold (each joint would land
        # early and coast past its aim).
        inp.synchronization = Synchronization.Time
        inp.minimum_duration = dt
        self._pt_rk_cfg = True
        inp.current_position = self._rl_q
        inp.current_velocity = self._rl_v
        inp.current_acceleration = self._rl_a
        inp.target_position = aim
        inp.target_velocity = v
        inp.target_acceleration = a
        try:
            self._rk.update(inp, self._rk_out)
        except Exception:
            self._rk_errors += 1
            if self._rk_errors == 1:
                logger.error("[RTI] Ruckig rejected a setpoint (non-finite command?) — "
                             "holding position. Further occurrences are counted, not logged.")
            return self._rl_q
        out = self._rl_out
        out[:] = self._rk_out.new_position
        self._rl_v[:] = self._rk_out.new_velocity
        self._rl_a[:] = self._rk_out.new_acceleration
        if self._rk_out.trajectory.duration > dt * (1.0 + 1e-9):
            # Some joint cannot land in one tick (a jump, a dead stop, a speed cap): the shared plan is that joint's
            # catch-up — every joint whose OWN landing fits still lands exactly on its aim (no joint slows another).
            self._pt_imd[:] = self._rk_out.trajectory.independent_min_durations
            np.less_equal(self._pt_imd, dt * (1.0 + 1e-9), out=self._pt_ok)
            ok = self._pt_ok
            np.copyto(out, aim, where=ok)
            np.copyto(self._rl_v, v, where=ok)
            np.copyto(self._rl_a, a, where=ok)
        np.clip(out, self._lim_lo_eff, self._lim_hi_eff, out=out)
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

    def _tcp_scale_velocity(self, v, ctx):
        """Scale the whole velocity vector so the TCP stays under tcp_vel_limit.

        Whole-vector, not per-joint: scaling every component by the same factor preserves
        the commanded DIRECTION exactly, which is what a velocity-streaming controller is
        entitled to expect. A per-joint clip would bend the motion.

        A LIMIT, not smoothing — called on both the limited and the limiter-off paths,
        same rule as the soft joint limits.
        """
        if self.tcp_vel_limit <= 0.0 or ctx is None:
            return v
        jpos = self._tcp_jacobian_pos(ctx)
        if jpos is None:
            return v
        vt = jpos @ v                              # (3,) — small alloc, cap-enabled only
        speed = float(np.sqrt(np.dot(vt, vt)))
        if speed > self.tcp_vel_limit:
            v *= self.tcp_vel_limit / speed
        return v

    def _limit_velocity(self, v_des, q_meas=None, ctx=None):
        """Clamp a velocity command to the velocity, acceleration and jerk budgets.

        `q_meas` is the MEASURED joint position. Velocity mode does not command position, so
        the limiter has no integrated position of its own to trust — the only honest source
        for "how far is this joint from its limit" is the encoder.
        """
        if not self.rate_limit_enabled:
            # The soft limits are NOT part of the rate limiter and must hold with it off,
            # exactly as they do on the position path. Only the v/a/j smoothing is optional.
            # The TCP cap is a limit too, so it applies here as well — before the brake,
            # which always has the last word.
            self._rl_v[:] = v_des
            self._tcp_scale_velocity(self._rl_v, ctx)
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

        # TCP cap after the cascade (so the accel clamp cannot re-inflate what it cut),
        # then the soft-limit brake LAST — the hard workspace guard always has the final
        # word, and it can only lower further, never raise.
        self._tcp_scale_velocity(v, ctx)
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

    def set_dynamics_ff_source(self, name):
        """Inertia / Coriolis feedforward source for RTI / SOFT_RTI position streams: "stream" (the stream's own
        acceleration; DYNAMICS_FF_DEFAULT, set at every RTI / SOFT_RTI entry) or "off". The robot-wide
        inertia / Coriolis switches still apply on top. An unknown name is refused and leaves the source unchanged."""
        key = str(name).strip().lower()
        if key not in DYNAMICS_FF_SOURCES:
            logger.warning("[RTI] rti_set_dynamics_ff refused: %r — known: %s; source stays %r",
                           name, ", ".join(DYNAMICS_FF_SOURCES), self.dynamics_ff_source)
            return False
        self.dynamics_ff_source = key
        self._reset_stream_acceleration(self._trk_cmd_tick)
        logger.warning("[RTI] Dynamics feedforward source: %s", key.upper())
        return True

    def _reset_stream_acceleration(self, current_tick):
        self._sacc[:] = 0.0
        self._sacc_lp1[:] = 0.0
        self._sacc_hold[:] = 0.0
        self._sacc_vprev[:] = 0.0
        self._sacc_cmd_tick = current_tick
        self._sacc_n = 0

    def _stream_acceleration(self, ctx):
        """This tick's stream acceleration (rad/s^2) for the feedforward: on a NEW packet (the target-velocity estimator
        has just run) the velocity's change since the previous packet over their interval, held until the next;
        dropped to 0 after STREAM_ACC_HOLD_INTERVALS without one; then the 2nd-order low-pass, every tick. The
        velocity is the PC's own when it sent one, else the estimate. No acceleration before the 3rd packet: the
        estimator's first velocity is a seeded 0, and a stream that starts moving would read it as a jump."""
        if self._trk_cmd_tick != self._sacc_cmd_tick:
            v = self._feedforward if self._trk_vtgt_from_pc else self._trk_vtgt
            self._sacc_n += 1
            if self._sacc_n >= 3:
                # the SAME interval the velocity estimate was taken over (its drop rule, or the send times) — a rule
                # of its own here disagreed for gaps of 1.5-1.75 intervals and halved that packet's estimate
                np.subtract(v, self._sacc_vprev, out=self._sacc_hold)
                self._sacc_hold *= 1.0 / self._trk_interval_s
            else:
                self._sacc_hold[:] = 0.0
            self._sacc_vprev[:] = v
            self._sacc_cmd_tick = self._trk_cmd_tick
        elif ctx.current_tick - self._sacc_cmd_tick > STREAM_ACC_HOLD_INTERVALS * self._trk_gap:
            self._sacc_hold[:] = 0.0
        a = self._sacc_alpha
        np.subtract(self._sacc_hold, self._sacc_lp1, out=self._sacc_tmp)
        self._sacc_tmp *= a
        self._sacc_lp1 += self._sacc_tmp
        np.subtract(self._sacc_lp1, self._sacc, out=self._sacc_tmp)
        self._sacc_tmp *= a
        self._sacc += self._sacc_tmp
        return self._sacc

    def apply_mode_contract(self, soft, rate_hz=None):
        """Configure the handler for the mode being entered. Called by dispatch on enter().

        The two RTI modes share this one handler, so every safety layer is shared by
        construction — soft joint limits, brake-at-limits, the TCP velocity cap, gravity
        compensation and start-pose validation are the same code in both. What differs is
        only the streaming contract, and it is all set here:

            RTI       the PC closes the loop, one setpoint per tick. 40 ms watchdog, the
                      pass-through (stamped targets replayed one tick behind; a stream without
                      send times gets the point-to-point Ruckig planner, as before 2026-10-11).
            SOFT_RTI  the PC sends targets at its own rate and the robot fills in between
                      them. Timeout sized from the rate, ruckig_advancing (best measured
                      accuracy, §13.6), pacing left to the operator.

        `rate_hz` is the PC's intended command rate. Given it, the timeout is sized at three
        command intervals — enough to ride out two dropped packets, not enough to keep
        driving at a dead PC. Omitted, the timeout is left as-is.

        Never lowers safety: the timeout is still clamped, the velocity scale still cannot
        exceed the robot XML, and the soft joint limits are untouched either way.
        """
        if not soft:
            # Flags first: the timeout ceiling depends on which contract this is.
            self.soft_mode = False
            self._soft_rate_hz = None
            self._soft_refused = 0
            self.dynamics_ff_source = DYNAMICS_FF_DEFAULT   # every entry (a PC may choose: rti_set_dynamics_ff)
            self.set_command_timeout(COMMAND_TIMEOUT_S)
            self.set_servo_generator(SERVO_PASSTHROUGH)     # the old planner for 101-byte streams (2026-10-11)
            self.set_pace(0.0)
            self.set_min_duration(0.0)
            self._state_every = 1     # hard RTI closes the loop per tick; it needs every one
            logger.warning("[RTI] Contract: HARD real-time. One setpoint per %.0f ms tick; "
                           "ripple is 0%% here and always was.", self.delta_t * 1e3)
            return

        if rate_hz is not None:
            try:
                hz = float(rate_hz)
            except (TypeError, ValueError):
                logger.warning("[RTI] SOFT_RTI: rate is not a number: %r — timeout left "
                               "unchanged", rate_hz)
                hz = None
            else:
                if not np.isfinite(hz) or hz <= 0.0:
                    logger.warning("[RTI] SOFT_RTI: rate %r is not a positive finite Hz — "
                                   "timeout left unchanged", rate_hz)
                    hz = None
        else:
            hz = None

        # Flags first: timeout_ceiling() keeps the declared rate's own 3 intervals even when the
        # operator's ceiling is lower (a 5 Hz stream needs 600 ms just to be healthy).
        self.soft_mode = True
        self._soft_rate_hz = hz
        self._soft_refused = 0
        self.dynamics_ff_source = DYNAMICS_FF_DEFAULT       # every entry (a PC may choose: rti_set_dynamics_ff)
        self.set_min_duration(SOFT_MIN_DURATION_DEFAULT)   # the glide, not the kicks (see the constant)
        if hz is not None:
            self.set_command_timeout(3.0 / hz)
        self.set_servo_generator(SERVO_ADVANCING)
        logger.warning("[RTI] Contract: SOFT real-time. The ROBOT fills in between your "
                       "targets. %s Prefer a rate that divides into %.0f Hz (50, 25, 10) — "
                       "a non-integer ratio corrupts every velocity estimate taken from "
                       "the stream.",
                       f"Timeout sized for {hz:.0f} Hz." if hz is not None
                       else "Timeout unchanged — pass a rate to size it.",
                       1.0 / self.delta_t)

    def _set_ruckig_sync(self, mode):
        """Switch Ruckig's synchronization mode. Cold path, and it is a REAL behaviour change.

        Under `Synchronization.No` each joint moves at its own optimal pace; under `Time`
        they share one duration, so a fast joint waits for the slowest. `Time` is the
        coordinated motion most operators expect, but `No` is what this handler has been
        hardware-verified with, so the switch is coupled to the one feature that requires it
        (duration pacing — Ruckig ignores `minimum_duration` entirely under `No`) rather than
        made the default. Logged whenever it changes, because a silent change of motion
        character is exactly the kind of thing that gets diagnosed as a hardware fault.
        """
        if self._rk is None or self._rk_in.synchronization == mode:
            return
        self._rk_in.synchronization = mode
        logger.warning("[RTI] Ruckig synchronization -> %s. %s",
                       "TIME" if mode == Synchronization.Time else "NONE",
                       "Joints now share one trajectory duration (coordinated motion)."
                       if mode == Synchronization.Time else
                       "Joints move independently again (the hardware-verified default).")

    def set_min_duration(self, multiplier):
        """Duration pacing: each planned move must last at least N command intervals.

        A minimum trajectory execution time, and the third way to stop the
        planner sprinting between sparse targets. 0 disables (the default).

        Expressed as a MULTIPLE of the measured command interval rather than as a time,
        because the useful value is relative to the PC's rate — and the robot already
        measures that rate (`_update_target_velocity`), so the operator states a shape
        preference rather than a number of seconds or a speed in rad/s.

        **The multiple must exceed 1, and that is the entire mechanism.** At exactly 1x the
        plan ends as the next command lands, so its deceleration phase still executes and
        the velocity still returns to zero — a stretched bump, not a glide. Above ~1.5x the
        plan always outlives the next command, the deceleration is perpetually deferred, and
        the motion becomes continuous. Measured at 5 Hz: velocity swing 0.874 -> 0.000 rad/s,
        jerk 2546 -> 0.

        Like every pacing mechanism it buys smoothness with lag (4.32 -> 6.29 deg at 5 Hz),
        and it is the least accurate of the smooth options — `ruckig_advancing` reaches
        0.05 deg. Exposed because it is self-tuning and because it is what a user of soft real-time
        servo interfaces will look for.

        Ruckig generators only. The tracker has no planned duration to floor.

        CORRECTION, 2026-08-20: this was first reported as a structural no-op, on the
        strength of a first-tick test taken FROM REST — where the jerk-limited ramp is
        identical regardless of total duration, so the floor genuinely cannot show. From a
        MOVING state it changes the first tick immediately (0.4233 -> 0.3767 rad/s). Testing
        the one initial condition that cannot exhibit the effect is how a real feature got
        called impossible twice.
        """
        try:
            requested = float(multiplier)
        except (TypeError, ValueError):
            logger.warning("[RTI] Minimum duration refused: not a number: %r", multiplier)
            return
        if not np.isfinite(requested) or requested < 0.0:
            logger.warning("[RTI] Minimum duration refused: %r is not a non-negative finite "
                           "multiple", multiplier)
            return
        if requested == 0.0:
            self.min_duration_mult = 0.0
            self._set_ruckig_sync(Synchronization.No)
            logger.warning("[RTI] Duration pacing DISABLED (per-joint independence restored)")
            return

        value = min(max(requested, MIN_DURATION_MULT_MIN), MIN_DURATION_MULT_MAX)
        if value != requested:
            logger.warning("[RTI] Duration multiple %.2f outside [%.1f, %.1f] — clamped to "
                           "%.2f", requested, MIN_DURATION_MULT_MIN, MIN_DURATION_MULT_MAX,
                           value)
        self.min_duration_mult = value
        self._set_ruckig_sync(Synchronization.Time)
        if value < MIN_DURATION_MULT_SUGGESTED:
            logger.warning("[RTI] Duration pacing %.2fx — at or near 1x the plan ends as the "
                           "next command lands, so its deceleration still runs and the "
                           "velocity still returns to zero. %.1fx or above is where the "
                           "motion actually becomes continuous.",
                           value, MIN_DURATION_MULT_SUGGESTED)
        else:
            logger.warning("[RTI] Duration pacing %.2fx the measured command interval", value)

    def set_state_rate(self, hz):
        """How often to unicast the recipe payload to the PC. 0 = every tick (the default).

        The state stream runs at the FULL LOOP RATE regardless of how slowly the PC commands,
        so a 5 Hz SOFT_RTI session still receives 250 packets a second — fifty per command,
        and 136 KB/s on the `standard` recipe. This lets the PC ask for less.

        Two reasons it matters more than it looks:

          * **On the TCP transport it is the direction that generates backpressure.** Commands
            are 101 bytes at the PC's rate; state is the whole recipe at 250 Hz, twenty to
            forty times the traffic. A PC that reads slowly fills the robot's send buffer, and
            this is the knob that stops that happening in the first place.
          * It composes with `set_recipe`. `minimal` cuts the payload ~4.5x; this cuts the
            rate. At 5 Hz commands, 250 -> 10 Hz state is a 25x reduction on its own.

        Clamped to [STATE_RATE_MIN_HZ, loop rate] — see that constant for why there is a
        floor rather than an off switch. Ignored in hard RTI, where the PC closes the loop
        every tick and therefore needs state every tick; apply_mode_contract() clears it.
        """
        # REFUSED in hard RTI, not merely reset on mode entry. apply_mode_contract() clears
        # this when the mode is entered, but nothing stopped a PC lowering it AFTERWARDS —
        # and in hard RTI the PC closes the loop every tick, so a reduced state rate silently
        # breaks the one guarantee that contract makes. Same reasoning as refusing TCP there.
        if not self.soft_mode:
            logger.warning("[RTI] rti_set_state_rate is refused in hard RTI: you close the "
                           "loop every tick, so you need state every tick. Reconnect with "
                           "a rate_hz for SOFT_RTI if you want to throttle it.")
            return
        try:
            v = float(hz)
        except (TypeError, ValueError):
            logger.warning("[RTI] state rate: not a number: %r", hz)
            return
        if not np.isfinite(v) or v <= 0.0:
            self._state_every = 1
            logger.warning("[RTI] State rate: every tick (%.0f Hz)", 1.0 / self.delta_t)
            return
        loop_hz = 1.0 / self.delta_t
        clamped = min(max(v, STATE_RATE_MIN_HZ), loop_hz)
        if clamped != v:
            logger.warning("[RTI] State rate %.1f Hz outside [%.0f, %.0f] — clamped to %.1f. "
                           "The floor exists because the state stream is the PC's only way "
                           "to notice the robot has left RTI mode.",
                           v, STATE_RATE_MIN_HZ, loop_hz, clamped)
        # A whole number of ticks, so the spacing is exact rather than drifting — the same
        # reason the command rate wants an integer ratio (servo_mode_options.md 15.1).
        self._state_every = max(1, int(round(loop_hz / clamped)))
        logger.warning("[RTI] State rate -> every %d ticks (%.1f Hz), was every tick",
                       self._state_every, loop_hz / self._state_every)

    def set_pace(self, factor):
        """Auto velocity pacing: cap the generator at `factor` x the TARGET's own speed.

        0 disables (the default). This is the self-tuning alternative to `set_vel_scale`,
        and it exists because the one real objection to the fixed cap is that the operator
        must know their travel speed in rad/s. Here they do not — the robot measures how
        fast the target is actually moving and derives the cap from that, so it follows a PC
        that changes speed or command rate mid-session.

        `factor` is headroom. 1.0 lets the generator move exactly as fast as the target and
        so never catch up after a disturbance; 1.5-2.0 leaves margin to recover while still
        removing the room to sprint. Above about 3 it stops binding and does nothing, the
        same cliff the fixed cap has.

        Composes with everything: it can only ever LOWER the budget the TCP cap and the
        joint limits already set, never raise it.

        NEGATIVE RESULT WORTH KEEPING. The first attempt at this was a minimum
        trajectory execution time, via Ruckig's `minimum_duration`. It is a NO-OP
        in our architecture — Ruckig meets a duration floor by padding the END of the
        trajectory, so the first tick is bit-identical with the floor at 0.0, 0.2 or 0.5 s,
        and the first tick is all a per-tick re-planner ever uses. See _auto_pace_budget.
        """
        try:
            requested = float(factor)
        except (TypeError, ValueError):
            logger.warning("[RTI] Velocity pacing refused: not a number: %r", factor)
            return
        if not np.isfinite(requested) or requested < 0.0:
            logger.warning("[RTI] Velocity pacing refused: %r is not a non-negative finite "
                           "factor", factor)
            return
        if requested == 0.0:
            self.pace_factor = 0.0
            logger.warning("[RTI] Auto velocity pacing DISABLED")
            return
        value = min(requested, PACE_FACTOR_MAX)
        if value != requested:
            logger.warning("[RTI] Pacing factor %.2f above the %.1f ceiling — clamped. A "
                           "factor that large never binds anyway.", requested,
                           PACE_FACTOR_MAX)
        self.pace_factor = value
        logger.warning("[RTI] Auto velocity pacing at %.2fx the measured target speed "
                       "(floor %.3f rad/s)", value, PACE_FLOOR_RAD_S)

    def set_servo_generator(self, name, gain=None, lookahead=None, self_lookahead=None):
        """Select the position generator: "ruckig" (default) or "tracker".

        The two are not better/worse, they are for different jobs, and the choice is exposed
        rather than decided here because only hardware can settle it:

          * **ruckig** — a point-to-point planner. Arrives exactly, never overshoots, bounds
            jerk. On a SPARSE stream it sprints to each target and idles, which is the
            ripple in TODO/servo_mode_options.md §5.1. Right for steps; wrong for streams
            unless the velocity budget is capped near the travel speed (rti_set_vel_scale).
          * **tracker** — a servo-style critically-damped second-order tracking law.
            Never plans to stop, so it has no ripple mechanism, and needs nothing from the
            PC — no feedforward, no operator-supplied travel speed. It is a servo, not a
            planner: on a large step it saturates and can overshoot where Ruckig would not.

        `gain` is the tracking bandwidth wn in rad/s; `lookahead` is the extrapolation
        horizon in seconds. Passing gain WITHOUT lookahead re-derives lookahead as 2/wn, the
        value that analytically cancels the ramp lag — so tuning one knob does not silently
        leave the other at a value that no longer pairs with it. Pass both to override.

        Switching mid-session is safe: both generators keep their state in the same
        _rl_q/_rl_v/_rl_a buffers, so the new law continues from the position, velocity and
        acceleration the old one actually achieved rather than from a jump.
        """
        key = str(name).strip().lower()
        if key not in SERVO_GENERATORS:
            logger.warning("[RTI] Servo generator refused: %r is not one of %s",
                           name, ", ".join(SERVO_GENERATORS))
            return
        if key == SERVO_PASSTHROUGH and self.soft_mode:
            logger.warning("[RTI] Servo generator 'passthrough' refused in SOFT_RTI: it replays a target EVERY tick "
                           "(hard RTI); sparse targets need a generator that fills in between them.")
            return
        if key == SERVO_TRACKER and self.rate_limit_enabled is False:
            logger.warning("[RTI] Servo generator 'tracker' selected while the rate limiter "
                           "is OFF — the limiter-off path streams commands through "
                           "unshaped, so NO generator runs at all. Re-enable the rate "
                           "limiter for the tracker to have any effect.")

        if gain is not None:
            try:
                g = float(gain)
            except (TypeError, ValueError):
                logger.warning("[RTI] Servo generator: gain is not a number: %r", gain)
                return
            if not np.isfinite(g):
                logger.warning("[RTI] Servo generator: gain %r is not finite", gain)
                return
            clamped = min(max(g, TRACKER_GAIN_MIN), TRACKER_GAIN_MAX)
            if clamped != g:
                logger.warning("[RTI] Tracker gain %.2f outside [%.1f, %.1f] — clamped to "
                               "%.2f", g, TRACKER_GAIN_MIN, TRACKER_GAIN_MAX, clamped)
            if clamped > TRACKER_GAIN_OVERSHOOT_WARN:
                logger.warning("[RTI] Tracker gain %.1f is above %.0f — on a large STEP the "
                               "law saturates on acceleration and overshoots (measured 22%% "
                               "at gain 25, 46%% at gain 40 for a 0.5 rad step). Fine for a "
                               "smooth stream; use 'ruckig' for point-to-point.",
                               clamped, TRACKER_GAIN_OVERSHOOT_WARN)
            self.tracker_gain = clamped
            if lookahead is None:
                # Re-pair it. See the docstring: 2/wn is derived, not tuned.
                self.tracker_lookahead = 2.0 / clamped

        if lookahead is not None:
            try:
                la = float(lookahead)
            except (TypeError, ValueError):
                logger.warning("[RTI] Servo generator: lookahead is not a number: %r",
                               lookahead)
                return
            if not np.isfinite(la):
                logger.warning("[RTI] Servo generator: lookahead %r is not finite", lookahead)
                return
            self.tracker_lookahead = min(max(la, 0.0), TRACKER_LOOKAHEAD_MAX)

        if self_lookahead is not None:
            try:
                sl = float(self_lookahead)
            except (TypeError, ValueError):
                logger.warning("[RTI] Servo generator: self_lookahead is not a number: %r",
                               self_lookahead)
                return
            if not np.isfinite(sl):
                logger.warning("[RTI] Servo generator: self_lookahead %r is not finite",
                               self_lookahead)
                return
            self.tracker_self_lookahead = min(max(sl, 0.0), TRACKER_LOOKAHEAD_MAX)

        self._refresh_tracker_constants()
        self.servo_generator = key
        self._pt_reset(self.last_cmd_tick)
        if key == SERVO_TRACKER:
            logger.warning("[RTI] Servo generator -> TRACKER (gain %.2f rad/s, lookahead "
                           "%.4f s). A servo law, not a planner: prefer 'ruckig' for "
                           "point-to-point moves.",
                           self.tracker_gain, self.tracker_lookahead)
        elif key == SERVO_ADVANCING:
            logger.warning("[RTI] Servo generator -> RUCKIG_ADVANCING (planner aimed at the "
                           "moving target; exact arrival kept, ripple removed)")
        elif key == SERVO_PASSTHROUGH:
            logger.warning("[RTI] Servo generator -> PASSTHROUGH (hard RTI's default: stamped targets replayed "
                           "exactly, ~1 tick; a position without a send time is refused)")
        else:
            logger.warning("[RTI] Servo generator -> RUCKIG (plans to stop at every target)")

    def timeout_ceiling(self):
        """The most the watchdog may be set to RIGHT NOW, in seconds.

        RTI:       COMMAND_TIMEOUT_S. A client can only tighten it — see the note above
                   DEFAULT_SOFT_TIMEOUT_CEILING_S for why a stale velocity/torque makes any
                   widening unsafe there.
        SOFT_RTI:  the operator's ceiling, but never less than the declared rate's own 3
                   intervals (a 5 Hz stream is unhealthy below 600 ms), capped at the absolute
                   maximum. The ceiling limits EXTRA slack; it must not break the rate contract.
        """
        if not self.soft_mode:
            return COMMAND_TIMEOUT_S
        own = 3.0 / self._soft_rate_hz if self._soft_rate_hz else 0.0
        return min(max(self.soft_timeout_ceiling, own), MAX_COMMAND_TIMEOUT_S)

    def set_command_timeout(self, seconds):
        """Runtime command watchdog, in seconds. Returns the value actually applied (or None).

        Sized for the PC's RATE, not for a dead link. Servo mode — a PC streaming targets at
        5-50 Hz and letting the robot interpolate — is impossible under the shipped 40 ms
        default, and not for a subtle reason: at 20 Hz the interval between two perfectly
        healthy commands is 50 ms, so the gap between them IS the timeout. Control stops between
        every pair of commands (TODO/servo_mode_options.md §7).

        Raising it does NOT let a POSITION stream run away: while no command arrives the
        generator continues toward the LAST target and stops there, so the motion a long timeout
        permits is bounded by the last thing the PC asked for. What it delays is the moment a
        genuinely dead link is declared — hence a ceiling. It is NOT true of velocity or torque,
        where the last command keeps being applied every tick: that is why the ceiling depends
        on the contract (timeout_ceiling()) and why SOFT_RTI refuses those command types.

        Clamped to [2 ticks, timeout_ceiling()]. A value below two ticks would make a single
        jittered packet indistinguishable from a dead link. In hard RTI the ceiling is the
        40 ms default, so the call can only TIGHTEN it. Takes effect immediately, including
        mid-control: the timeout is only ever compared, never latched, so a PC that widens it
        before slowing down is never caught by the old value.

        Cold path — logs, and does not run per tick.
        """
        try:
            requested = float(seconds)
        except (TypeError, ValueError):
            logger.warning("[RTI] Command timeout refused: not a number: %r", seconds)
            return None
        if not np.isfinite(requested):
            logger.warning("[RTI] Command timeout refused: %r is not finite", seconds)
            return None

        floor = 2.0 * self.delta_t
        ceiling = self.timeout_ceiling()
        value = min(max(requested, floor), ceiling)
        if value != requested:
            if requested > ceiling:
                logger.warning(
                    "[RTI] Command timeout %.4f s exceeds the %s limit of %.3f s — clamped to %.4f s. %s",
                    requested, "SOFT_RTI operator" if self.soft_mode else "RTI",
                    ceiling,  value,
                    "(Widen <safety><max_soft_rti_timeout> in the robot XML to allow more.)"
                    if self.soft_mode else
                    "(Hard RTI's watchdog can only be tightened: a stale velocity/torque command "
                    "keeps driving for the whole window.)")
            else:
                logger.warning("[RTI] Command timeout %.4f s is below the floor of %.4f s — "
                               "clamped to %.4f s", requested, floor, value)

        self.timeout_ticks = max(1, int(round(value / self.delta_t)))
        # Kept derived, as in __init__: a fixed warn threshold would silently become
        # "warn exactly when we stop" the moment the timeout is shortened here.
        self.miss_warn_ticks = max(2, self.timeout_ticks // 2)
        logger.warning("[RTI] Command timeout %.4f s (%d ticks) — supports a PC rate down "
                       "to ~%.1f Hz", value, self.timeout_ticks, 1.0 / value)
        return value

    def set_vel_scale(self, fraction):
        """Scale the RTI joint VELOCITY limits to `fraction` of the robot XML values.

        A PREFERENCE in the three-tier sense (CLAUDE.md): it can only lower the per-mode
        limit, never raise it above the XML ceiling. 1.0 restores the XML values exactly.
        Always applied to the ORIGINAL limits, so repeated calls do not compound.

        This is the knob that makes low-rate streaming smooth, and it works by REMOVING the
        generator's room to sprint. Ruckig plans to arrive and stop at each target; given
        50 ms of silence and 10x more speed than the PC is actually travelling, it covers
        the gap in 5 ms and idles for 45 — that stop/sprint cycle IS the ripple. Capped near
        the travel speed it cannot sprint, so it moves continuously instead: measured 59%
        velocity ripple -> 0.0% at 20 Hz (TODO/servo_mode_options.md §5.2).

        Two honest caveats:
          * It is a CLIFF, not a slope. It does nothing at all until it binds. A common
            25% default changed NOTHING in our test, because the test travelled at 10% of
            the limit and 25% still left room to sprint.
          * The cost is lag, and it is real: a generator capped AT the travel speed has no
            reserve to catch up after a disturbance (0.82 deg -> 1.05 deg trailing at 20 Hz).

        Velocity only. Acceleration and jerk are deliberately untouched — that matches what
        was measured, and what asynchronous position interfaces commonly expose
        (a maximum joint velocity, and nothing else).

        Note the same asymmetry the TCP cap has: on the position path this acts through the
        limiter's per-tick velocity budget, so with the rate limiter OFF a raw position
        stream passes unshaped. On the velocity path the clamp applies either way.
        """
        try:
            requested = float(fraction)
        except (TypeError, ValueError):
            logger.warning("[RTI] Velocity scale refused: not a number: %r", fraction)
            return
        if not np.isfinite(requested) or requested <= 0.0:
            logger.warning("[RTI] Velocity scale refused: %r is not a positive finite "
                           "fraction", fraction)
            return

        if requested > 1.0:
            logger.warning("[RTI] Velocity scale %.3f would RAISE the RTI velocity limit "
                           "above the robot XML value — clamped to 1.0. Raise "
                           "<rti_limits><velocity> in the robot XML if you need more.",
                           requested)
            value = 1.0
        elif requested < MIN_VEL_SCALE:
            logger.warning("[RTI] Velocity scale %.4f is below the %.2f floor — clamped. "
                           "Use a stop, not a scale, to hold the arm still.",
                           requested, MIN_VEL_SCALE)
            value = MIN_VEL_SCALE
        else:
            value = requested

        self.vel_scale = value
        # In place, so self.vel_limit keeps its identity — _limit_position_ruckig decides
        # whether the TCP cap is engaged with `vcap is not self.vel_limit`, and rebinding
        # the attribute here would quietly break that test.
        np.multiply(self._vel_limit_base, value, out=self.vel_limit)

        if self._rk is not None:
            # Ruckig's binding COPIES on assignment, so mutating self.vel_limit in place
            # above does not reach the planner. It has to be pushed.
            self._rk_in.max_velocity = self.vel_limit
            # The TCP cap owns max_velocity while engaged and restores the base budget on
            # release. Clearing the flag makes the next tick recompute against the NEW base
            # rather than restoring the stale one. No tick runs unbounded in between: the
            # cap is recomputed before _rk.update() on every tick it applies to.
            self._tcp_cap_applied = False

        logger.warning("[RTI] RTI velocity scale %.3f — max %.3f rad/s (XML %.3f rad/s)",
                       value, float(np.max(self.vel_limit)),
                       float(np.max(self._vel_limit_base)))

    def set_tcp_velocity_limit(self, mps):
        """Runtime TCP translational-speed cap in m/s. 0 disables (the default).

        A LIMIT, not a smoothing preference: on the velocity path it applies even with the
        rate limiter switched off, same rule as the soft joint limits. On the position path
        it acts through Ruckig's per-tick velocity budget, so with the rate limiter off a
        raw position stream passes unshaped — there is no velocity being generated for the
        cap to scale. That asymmetry is inherent, not an oversight.
        """
        self.tcp_vel_limit = max(0.0, float(mps))
        logger.warning("[RTI] TCP velocity limit %s",
                       f"{self.tcp_vel_limit:.3f} m/s" if self.tcp_vel_limit > 0
                       else "DISABLED")

    def _tcp_jacobian_pos(self, ctx):
        """(3, joint_num) translational Jacobian at the MEASURED pose, or None.

        Measured, not commanded: this cap exists for the physical world, so it must reason
        from where the arm actually is. Costs one kinematics call (~30 us via pinocchio on
        the Pi) per tick, only while the cap is enabled — the same cost class
        _fk_commanded() already pays every tick, and like it, the call allocates its result
        (an accepted exception to the hot-path rule, matching the Ruckig list returns).

        None (no model, or the call failed) leaves the cap inert rather than guessing: an
        invented Jacobian would silently bound the wrong thing. Warned once, not per tick.
        """
        robot = getattr(ctx, 'robot', None) if ctx is not None else None
        if robot is None:
            if not self._tcp_warned:
                self._tcp_warned = True
                logger.error("[RTI] TCP velocity limit is set but no robot model is "
                             "available — the cap is INERT. ctx.robot is None.")
            return None
        try:
            q = ctx.shared_data.joint['joint_position'][:self.joint_num]
            if should_use_pinocchio(robot):
                _, J = robot.pinocchio_fkine_and_jacob0(q)
            else:
                J = robot.jacob0(q=q)
            return J[:3, :self.joint_num]
        except Exception:
            if not self._tcp_warned:
                self._tcp_warned = True
                logger.exception("[RTI] TCP velocity limit: Jacobian computation failed — "
                                 "the cap is INERT until this is fixed.")
            return None

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

        # A 2nd-order Butterworth (the engine's "butter2"; tau = 1/(2*pi*fc), what this codebase's filter config
        # uses so a cutoff means the same thing at any loop rate). Was the first-order "ema" until 2026-10-11: with the
        # strict checks (target noise reads as acceleration) the Butterworth buys the most noise tolerance per ms of
        # lag — J1, simulated: 60 Hz +3.0 ms -> 3x the tolerated noise (0.010 deg), 40 Hz +5.25 ms -> ~6x; the
        # first-order 25 Hz cost +4.5 ms for 3x (TODO/servo_mode_options.md §17.22, user). Default stays OFF.
        tau = 1.0 / (2.0 * np.pi * cutoff)
        self._lp_bank = create_filter_bank({"kind": "butter2", "tau": tau}, self.joint_num,
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

        # SOFT_RTI IS POSITION-ONLY. Its watchdog may be widened (timeout_ceiling), and that is
        # only safe because a stale POSITION target converges and holds. A stale velocity or
        # torque command is re-applied every tick for the whole window, so those types are
        # refused here and stay in hard RTI, whose 40 ms watchdog a client cannot raise. A
        # refused packet is not "received": it does not feed the watchdog, so a PC that only
        # ever sends velocity trips LINK_LOST rather than moving the arm.
        if self.soft_mode and cmd_type != CMD_JOINT_POSITION:
            self._soft_refused += 1
            if self._soft_refused == 1:               # once per session — never per tick
                logger.warning(
                    "[RTI] SOFT_RTI accepts position targets only; refusing a %s packet. "
                    "Velocity/torque streaming needs hard RTI (rti_connect without a rate), "
                    "whose 40 ms watchdog is what makes a stale command safe.",
                    "velocity" if cmd_type == CMD_JOINT_VELOCITY else
                    "torque" if cmd_type == CMD_JOINT_TORQUE else f"type-{cmd_type}")
            return False

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

        _ts = unpack_command_timestamp(data)
        # HARD RTI NEEDS THE SEND TIME (user, 2026-10-12). Its pass-through follows the PC's own timing; a position
        # target without it (a 101-byte packet) used to fall back to the old planner — ~20 ms behind the PC instead of
        # ~6, silently. Refused like the strict checks: a controlled stop, then RTI_NO_SEND_TIME. SOFT_RTI, and
        # velocity / torque / stop packets, take a packet either way. A stamp that is not finite reads as None.
        if not self.soft_mode and cmd_type == CMD_JOINT_POSITION and _ts is None:
            self._refuse_unstamped()
            return False
        self._rx_applied += 1
        self.seq_rx_last = seq
        self.last_cmd_tick = ctx.current_tick
        self._cmd_ts = _NAN if _ts is None else _ts
        self.last_cmd_type = cmd_type
        if cmd_type != CMD_STOP:
            self._last_motion_type = cmd_type      # what the arm was DOING when a stop arrives (the stop ramp's seed)
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
        # Nor in SOFT_RTI: the PC sends at ITS rate (50 Hz reads as 20 %, 10 Hz as 4 %), and its watchdog is sized from
        # that rate — the warning fired at every SOFT_RTI claim on the arm (2026-10-11) and meant nothing.
        if self.soft_mode:
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
