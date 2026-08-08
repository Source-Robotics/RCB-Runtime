"""Ruckig corner blending — ADDITIVE PROTOTYPE, wired into nothing.

Status (2026-07-12): this module is NOT reachable from the planner. No MotionOptions
value selects it, trajectory_planner.py does not import it, and deleting this file
breaks nothing. It exists to answer, with measured numbers, whether trajectory-splicing
corner blending is worth productizing as the jerk-limited blend lane. Bench + comparison
against the shipped zone blend: dummy_test_scripts/test_ruckig_corner_blend.py.

The shape of the method: a blend ball around the connection point in joint-space radians,
points inside replaced by a Ruckig segment, original
trajectories kept outside, radius 0 = plain concatenation.

ONE DELIBERATE DESIGN CHOICE: the obvious version of this keeps the pre-planned input
trajectories and rewrites only the corner. We REGENERATE the legs with Ruckig as well. That
is what makes this lane jerk-bounded END TO END instead of only at the corners — and it is
not a shortcut taken by accident. The corners-only version was built and measured:
11.81s / peak jerk 216 against zone's 7.78s / 226 — SLOWER AND NOT SMOOTHER, strictly
dominated by the zone lane we already ship. The jerk lives in the LEGS (TOPPRA is bang-bang
in acceleration), so rewriting corners alone buys nothing; and a corners-only blend only
deletes the STOP at a connection, while our zone lane never creates one (it solves the whole
chain in a single TOPPRA pass). Do not "fix" this module to rewrite corners only.

The method
----------
The shipped zone blend prescribes corner GEOMETRY (a Bezier) and hands the whole
composite path to TOPPRA. blending_design.md §10 is right that Ruckig can never
follow that Bezier — it is target-following. This method sidesteps the objection by
not prescribing corner geometry at all:

    1. Plan each leg waypoint->waypoint REST-TO-REST with Ruckig
       (phase-synchronized, so legs are straight lines in joint space).
    2. Around each interior waypoint, cut the incoming leg where it enters a
       joint-space ball of radius d_before and the outgoing leg where it leaves a
       ball of radius d_after (last-outside / first-outside-radius scan, refined
       by bisection on the continuous trajectory).
    3. Replace everything inside the ball with a Ruckig state-to-state transition
       matching position, velocity AND acceleration at both cut states.

Because the transition matches full states, every seam is C2 by construction, and
the corner is jerk-limited because Ruckig is. The trades, stated up front:

  - The corner path is whatever Ruckig invents — there is NO a-priori deviation
    bound, so this can never back the `tolerance`/G64 contract. Deviation is
    measured after the fact (see the bench) the same way
    _assert_cartesian_path_fidelity treats spline-IK deviation.
  - Joint space only. A Cartesian lane would inherit the same "invented path"
    caveat through IK and is out of scope for the prototype.
  - Legs are rest-to-rest Ruckig, so corner entry speed is bounded by the leg's
    own jerk-limited braking ramp — the blend then skips the actual stop. Legs
    ending at rest is the precondition the splice relies on.

An infeasible transition (Ruckig rejects the boundary states) degrades that one
corner to the unblended rest-to-rest join — same exact-or-reject philosophy as the
rolling buffer, never a half-blend.

Planner-side code (command_executor process), NOT the RT hot path — normal
allocation rules apply, same as trajectory_planner.py.
"""

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

# Read-only reuse of the shipped jerk convention (jerk = accel * factor) so this
# lane cannot disagree with profile="ruckig" about the arm's jerk budget.
from motion.planner_pipeline import RUCKIG_JERK_FACTOR

# A blend ball may consume at most this fraction of each adjacent leg, leaving a
# stub between the exit of one corner and the entry of the next on a shared leg.
# Same intent as build_composite_joint_path's 0.5 clamp + overlap rescale.
MAX_LEG_FRACTION = 0.45

# Bisection refinement of the ball-crossing time, seconds. One sample step is
# already a fine bracket; this just sharpens the cut state.
_CUT_TIME_TOL = 1e-5

# Samples used to measure how far a corner strayed from the programmed path. Reported, not
# enforced — Ruckig invents the corner, so deviation is MEASURED, never guaranteed a priori.
_DEVIATION_SAMPLES = 32

# ======================================================================================
# DEAD END — DO NOT RE-ATTEMPT: "carry speed through the corners" (tried 2026-07-12)
# ======================================================================================
# The legs here are planned REST-TO-REST and then cut at the blend ball. That LOOKS wasteful
# — the arm appears to ramp up and straight back down every leg — and Ruckig accepts nonzero
# boundary velocities, so carrying a corner speed through the legs looks like free time.
# It is not. It was built, measured, and REVERTED. True chain duration, carry OFF -> ON:
#
#     sharp zigzag (6 reversals)      12.91s -> 14.23s   (+10%)
#     shallow bends, long legs         8.83s -> 12.46s   (+41%)
#     single 90-deg corner, 40deg legs 6.51s ->  7.44s   (+14%)
#     single 90-deg corner, big blend  6.15s ->  7.42s   (+21%)
#
# WORSE ON EVERY GEOMETRY TRIED. Two reasons, and the first is the one that matters:
#
#  1. THE BRAKING TAIL IS NEVER EXECUTED. The leg is CUT at the blend ball, so the
#     rest-to-rest deceleration you think you are paying for is thrown away before it ever
#     runs. There is no wasted braking to reclaim. (Measured: leg time barely moves,
#     4.96s -> 4.88s.)
#  2. A faster corner is a HARDER corner. The transition has to reverse more momentum, so
#     it takes LONGER and swings WIDER: transition time 7.95s -> 9.35s, deviation 0.005 ->
#     0.061 rad — 13x, straight through the blend radius the caller asked for.
#
# THE MEASUREMENT THAT FOOLED ME: I first summed the FULL leg durations (17.63s -> 9.93s,
# "-44%") and called it a saving. That sum is not what the chain executes — the chain cuts
# the legs. I measured a cost that was never being paid. If you are tempted by this again,
# measure `chain.duration`, not the sum of `leg.duration`.
# ======================================================================================


class RuckigCornerInfeasible(RuntimeError):
    """A leg (not a corner) could not be timed — limits are broken, not tight."""


@dataclass
class CornerDiag:
    """Per-corner record of what the blend actually did."""

    waypoint_index: int
    d_before: float                 # post-clamp cut radius, rad (incoming leg)
    d_after: float                  # post-clamp cut radius, rad (outgoing leg)
    blended: bool                   # False = transition infeasible, corner kept as stop
    entry_speed: float = 0.0        # |qd| at the incoming cut state, rad/s
    exit_speed: float = 0.0         # |qd| at the outgoing cut state, rad/s
    transition_duration: float = 0.0
    min_corner_distance: float = 0.0  # closest approach to the waypoint, rad
    # How far the INVENTED corner strays from the programmed polyline (rad). Ruckig picks
    # the corner path, so this is MEASURED, never guaranteed a priori.
    deviation: float = 0.0


@dataclass
class RuckigCornerChain:
    """Sampled result of plan_ruckig_corner_chain, dt-uniform."""

    positions: np.ndarray           # (n, dof)
    velocities: np.ndarray          # (n, dof)
    accelerations: np.ndarray       # (n, dof)
    dt: float
    duration: float
    corners: List[CornerDiag] = field(default_factory=list)
    segment_times: List[float] = field(default_factory=list)  # cumulative seam times
    # One entry per LEG (i.e. per queued command): the time at which that command has been
    # fully handed off — the end of the transition that rounds its target waypoint, or the
    # waypoint itself if that corner was not blended. The planner maps these to row indices
    # to fire per-command checkpoints, same role as build_composite_joint_path's boundaries.
    command_end_times: List[float] = field(default_factory=list)
    # Exact planned time windows of accepted Ruckig corner transitions. These are
    # emitted for diagnostics and visualisation; they are not geometric Bezier zones.
    corner_time_ranges: List[dict] = field(default_factory=list)

    @property
    def num_points(self):
        return int(self.positions.shape[0])

    def command_end_rows(self):
        """command_end_times as row indices into positions; last is always num_points."""
        n = self.num_points
        rows = [min(n, max(1, int(round(t / self.dt)) + 1)) for t in self.command_end_times]
        if rows:
            rows[-1] = n
        return rows


def _require_ruckig():
    try:
        from ruckig import Ruckig, InputParameter, Trajectory, Result, Synchronization
    except Exception as exc:  # pragma: no cover - env-dependent
        raise RuckigCornerInfeasible(
            "ruckig is not importable — it is pinned in requirements.txt; "
            "check the .venv, not ~/.local") from exc
    return Ruckig, InputParameter, Trajectory, Result, Synchronization


def _calculate(inp, dof):
    Ruckig, _, Trajectory, Result, _ = _require_ruckig()
    traj = Trajectory(dof)
    try:
        result = Ruckig(dof).calculate(inp, traj)
    except Exception as exc:
        # Ruckig RAISES (RuckigError) on an out-of-bounds boundary state rather than
        # returning a Result. Letting that escape would blow up the whole chain over one
        # bad corner, when the lane's whole contract is exact-or-reject PER CORNER. Convert
        # it into the same "infeasible" signal every other rejection uses.
        return None, exc
    ok = result in (Result.Working, Result.Finished)
    return traj if ok else None, result


# Ruckig SATURATES velocity and acceleration exactly at their limits. Read a state back off
# its own trajectory and floating-point can hand you a value a hair OVER — measured
# |v| = 1.0000000001 * vmax, and accel 0.864001 against a 0.864000 limit. Feed that back in
# as a boundary state and Ruckig raises "target velocity ... undercuts its minimum velocity
# limit". A fuzz over 400 random chains hit it on 4 of them (~1%), and because the planner
# falls back to individual moves on any exception, the symptom was an arm that quietly
# STOPPED at every corner on ~1% of chains, for no visible reason.
#
# The state came from a trajectory that respects the limits by construction, so the excess is
# pure representation error and clamping to the limit is EXACT recovery, not a fudge factor.
_LIMIT_EPS = 1.0 - 1e-9


def _clamp_state(state, vmax, amax):
    """Pull a cut state back inside the limits (see _LIMIT_EPS) before handing it to Ruckig."""
    q, qd, qdd = state
    return (q,
            np.clip(qd, -vmax * _LIMIT_EPS, vmax * _LIMIT_EPS),
            np.clip(qdd, -amax * _LIMIT_EPS, amax * _LIMIT_EPS))


def _leg_trajectory(start, end, vmax, amax, jmax):
    """Rest-to-rest, phase-synchronized leg: a straight joint-space line.

    Rest-to-rest is NOT the waste it looks like: the leg is cut at the blend ball, so the
    braking tail is discarded before it ever runs. Carrying a nonzero boundary velocity was
    built, measured and reverted — see the DEAD END block at the top of this module.
    """
    _, InputParameter, _, _, Synchronization = _require_ruckig()
    dof = start.shape[0]
    inp = InputParameter(dof)
    inp.current_position = start.tolist()
    inp.target_position = end.tolist()
    inp.max_velocity = vmax.tolist()
    inp.max_acceleration = amax.tolist()
    inp.max_jerk = jmax.tolist()
    # Phase sync keeps the leg on the straight line between waypoints (verified
    # ~4e-16 off-line in the bench). Ruckig silently falls back to time sync when
    # phase sync is impossible; the bench measures leg straightness so a fallback
    # shows up as a number, not a surprise.
    inp.synchronization = Synchronization.Phase
    traj, result = _calculate(inp, dof)
    if traj is None:
        raise RuckigCornerInfeasible(
            f"ruckig could not time a plain rest-to-rest leg (result={result}) — "
            f"velocity/acceleration/jerk limits must all be positive")
    return traj


def _polyline_distance(points, waypoints):
    """Max distance from the joint-space waypoint polyline (rad).

    For a move_joints chain the polyline IS the programmed path — the legs are straight
    lines through the waypoints — so this is exactly "how far did the invented corner
    stray from what was asked for".
    """
    pts = np.asarray(points, dtype=np.float64)
    best = np.full(pts.shape[0], np.inf)
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        d = b - a
        denom = float(np.dot(d, d))
        if denom < 1e-18:
            continue
        u = np.clip((pts - a) @ d / denom, 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(pts - (a + u[:, None] * d[None, :]), axis=1))
    return float(best.max())


def _transition(state_a, state_b, vmax, amax, jmax):
    """Full-state corner transition; None if Ruckig rejects the boundary states."""
    _, InputParameter, _, _, _ = _require_ruckig()
    qa, qda, qdda = state_a
    qb, qdb, qddb = state_b
    dof = len(qa)
    inp = InputParameter(dof)
    inp.current_position = list(qa)
    inp.current_velocity = list(qda)
    inp.current_acceleration = list(qdda)
    inp.target_position = list(qb)
    inp.target_velocity = list(qdb)
    inp.target_acceleration = list(qddb)
    inp.max_velocity = vmax.tolist()
    inp.max_acceleration = amax.tolist()
    inp.max_jerk = jmax.tolist()
    traj, _ = _calculate(inp, dof)
    return traj


def _distance_to(traj, t, point):
    q, _, _ = traj.at_time(float(t))
    return float(np.linalg.norm(np.asarray(q) - point))


def _entry_cut_time(leg, corner_q, radius, dt):
    """Last time the incoming leg is OUTSIDE the corner ball (then stays inside).

    Sample-scan from the end backwards to find the last sample outside the radius,
    then bisect the bracketing interval on the continuous trajectory.
    """
    duration = float(leg.duration)
    n = max(2, int(np.ceil(duration / dt)) + 1)
    times = np.minimum(np.arange(n) * dt, duration)
    outside = None
    for i in range(n - 1, -1, -1):
        if _distance_to(leg, times[i], corner_q) > radius:
            outside = i
            break
    if outside is None or outside == n - 1:
        return None  # never outside (degenerate leg) — caller skips the blend
    lo, hi = times[outside], times[outside + 1]
    while hi - lo > _CUT_TIME_TOL:
        mid = 0.5 * (lo + hi)
        if _distance_to(leg, mid, corner_q) > radius:
            lo = mid
        else:
            hi = mid
    return lo


def _exit_cut_time(leg, corner_q, radius, dt):
    """First time the outgoing leg is OUTSIDE the corner ball."""
    duration = float(leg.duration)
    n = max(2, int(np.ceil(duration / dt)) + 1)
    times = np.minimum(np.arange(n) * dt, duration)
    outside = None
    for i in range(n):
        if _distance_to(leg, times[i], corner_q) > radius:
            outside = i
            break
    if outside is None or outside == 0:
        return None
    lo, hi = times[outside - 1], times[outside]
    while hi - lo > _CUT_TIME_TOL:
        mid = 0.5 * (lo + hi)
        if _distance_to(leg, mid, corner_q) > radius:
            hi = mid
        else:
            lo = mid
    return hi


def _state_at(traj, t):
    q, qd, qdd = traj.at_time(float(t))
    return (np.asarray(q, dtype=np.float64),
            np.asarray(qd, dtype=np.float64),
            np.asarray(qdd, dtype=np.float64))


def plan_ruckig_corner_chain(waypoints, max_vel, max_accel, dt, blend_dists,
                             jerk_factor=RUCKIG_JERK_FACTOR, speed_scale=1.0,
                             max_jerk=None):
    """Plan a jerk-limited joint-space chain with Ruckig corner transitions.

    Args:
        waypoints: sequence of (dof,) joint arrays, length >= 2, radians.
        max_vel / max_accel: per-joint limits (rad/s, rad/s^2).
        dt: output sample period, seconds.
        blend_dists: for each of the N-2 interior waypoints, either a scalar
            joint-space cut radius (rad) used on both sides, or a
            (d_before, d_after) pair. 0 disables the blend at that corner.
        jerk_factor: FALLBACK only — jerk limit = max_accel * jerk_factor, the convention
            used before the robot XML stated a jerk budget. Applies when max_jerk is None.
        max_jerk: per-joint jerk limit (rad/s^3) from <exec_limits><jerk>, already scaled by
            the command's `jerk` option. Preferred over jerk_factor when given, so the budget
            comes from config rather than being computed from the acceleration limit.
        speed_scale: same convention as every other lane — velocity x scale,
            acceleration x scale^2, jerk x scale^3, so `speed` keeps one meaning.

    Returns:
        RuckigCornerChain sampled uniformly at dt (final row exactly at the last
        waypoint, at rest).
    """
    waypoints = [np.asarray(w, dtype=np.float64) for w in waypoints]
    n_wp = len(waypoints)
    if n_wp < 2:
        raise ValueError("Need at least 2 waypoints")
    n_corners = n_wp - 2
    if len(blend_dists) != n_corners:
        raise ValueError(f"Expected {n_corners} blend_dists, got {len(blend_dists)}")

    scale = float(np.clip(speed_scale, 1e-3, 1.0))
    vmax = np.asarray(max_vel, dtype=np.float64) * scale
    amax = np.asarray(max_accel, dtype=np.float64) * scale * scale
    jmax = ((np.asarray(max_jerk, dtype=np.float64) if max_jerk is not None
             else np.asarray(max_accel, dtype=np.float64) * float(jerk_factor))
            * scale ** 3)

    dt = float(dt)
    legs = [_leg_trajectory(waypoints[i], waypoints[i + 1], vmax, amax, jmax)
            for i in range(n_wp - 1)]
    leg_lengths = [float(np.linalg.norm(waypoints[i + 1] - waypoints[i]))
                   for i in range(n_wp - 1)]

    # Window of each leg that survives the cutting: [t_from, t_to].
    leg_from = [0.0] * (n_wp - 1)
    leg_to = [float(leg.duration) for leg in legs]

    corners: List[CornerDiag] = []
    transitions: List[Optional[object]] = []

    for c in range(n_corners):
        corner_q = waypoints[c + 1]
        d = blend_dists[c]
        d_before, d_after = (float(d[0]), float(d[1])) if np.ndim(d) else (float(d), float(d))
        d_before = min(max(d_before, 0.0), MAX_LEG_FRACTION * leg_lengths[c])
        d_after = min(max(d_after, 0.0), MAX_LEG_FRACTION * leg_lengths[c + 1])
        diag = CornerDiag(waypoint_index=c + 1, d_before=d_before, d_after=d_after,
                          blended=False)

        transition = None
        if d_before > 0.0 and d_after > 0.0:
            t_a = _entry_cut_time(legs[c], corner_q, d_before, dt)
            t_b = _exit_cut_time(legs[c + 1], corner_q, d_after, dt)
            # The entry cut must land inside the surviving window — after a
            # previous corner's exit cut on a shared short leg it might not.
            if (t_a is not None and t_b is not None
                    and t_a > leg_from[c] and t_b < leg_to[c + 1]):
                # Clamp before handing the cut states to Ruckig — see _clamp_state. Without
                # this, a state read back off a limit-saturated leg can sit a hair OUTSIDE
                # the limit and Ruckig raises instead of solving.
                state_a = _clamp_state(_state_at(legs[c], t_a), vmax, amax)
                state_b = _clamp_state(_state_at(legs[c + 1], t_b), vmax, amax)
                transition = _transition(state_a, state_b, vmax, amax, jmax)
                if transition is not None:
                    leg_to[c] = t_a
                    leg_from[c + 1] = t_b
                    diag.blended = True
                    diag.entry_speed = float(np.linalg.norm(state_a[1]))
                    diag.exit_speed = float(np.linalg.norm(state_b[1]))
                    diag.transition_duration = float(transition.duration)
                    # Closest approach to the waypoint = how much corner was cut.
                    ts = np.linspace(0.0, float(transition.duration),
                                     max(2, int(np.ceil(transition.duration / dt)) + 1))
                    diag.min_corner_distance = min(
                        _distance_to(transition, t, corner_q) for t in ts)
                    # How far the INVENTED corner strays from the programmed polyline.
                    # Reported, never guaranteed — this lane picks its own corner path, so
                    # the only honest thing to do is measure it and say so.
                    dev_ts = np.linspace(0.0, float(transition.duration), _DEVIATION_SAMPLES)
                    diag.deviation = _polyline_distance(
                        [transition.at_time(float(t))[0] for t in dev_ts], waypoints)
        transitions.append(transition)
        corners.append(diag)

    # Assemble (duration, sampler) segments: leg windows interleaved with transitions.
    # Track when each COMMAND (= each leg) has been fully handed off, so the planner can
    # checkpoint per command: that is the end of the transition rounding its target, or the
    # waypoint itself when that corner was not blended.
    segments = []
    command_end_times = []
    corner_time_ranges = []
    cursor = 0.0
    for i in range(n_wp - 1):
        window = leg_to[i] - leg_from[i]
        if window > 1e-12:
            segments.append((window, legs[i], leg_from[i]))
            cursor += window
        if i < n_corners and transitions[i] is not None:
            transition_start = cursor
            transition_duration = float(transitions[i].duration)
            segments.append((transition_duration, transitions[i], 0.0))
            cursor += transition_duration
            corner_time_ranges.append({
                "corner_index": int(corners[i].waypoint_index),
                "start_time_s": transition_start,
                "end_time_s": cursor,
            })
        command_end_times.append(cursor)

    seg_durations = np.array([s[0] for s in segments], dtype=np.float64)
    seg_starts = np.concatenate(([0.0], np.cumsum(seg_durations)))
    total = float(seg_starts[-1])

    n = max(2, int(np.ceil(total / dt)) + 1)
    dof = waypoints[0].shape[0]
    pos = np.zeros((n, dof), dtype=np.float64)
    vel = np.zeros((n, dof), dtype=np.float64)
    acc = np.zeros((n, dof), dtype=np.float64)
    for k in range(n):
        t = min(k * dt, total)
        # Last segment owns its end point; searchsorted-right minus one, clamped.
        idx = min(int(np.searchsorted(seg_starts, t, side="right")) - 1,
                  len(segments) - 1)
        _, traj, offset = segments[idx]
        q, qd, qdd = traj.at_time(offset + (t - seg_starts[idx]))
        pos[k] = q
        vel[k] = qd
        acc[k] = qdd
    # Land the endpoint exactly at rest, same rule as RuckigTimeParameterizer.
    pos[-1] = waypoints[-1]
    vel[-1] = 0.0
    acc[-1] = 0.0

    # The last command always ends at the end of the trajectory.
    if command_end_times:
        command_end_times[-1] = total

    return RuckigCornerChain(positions=pos, velocities=vel, accelerations=acc,
                             dt=dt, duration=total, corners=corners,
                             segment_times=list(seg_starts[1:]),
                             command_end_times=command_end_times,
                             corner_time_ranges=corner_time_ranges)
