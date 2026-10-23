"""Typed EXEC planner command records and per-command motion options.

The TCP protocol still accepts simple JSON, but command_executor normalizes queued
motion into these records before handing work to TrajectoryPlanner. This gives later
PathBuilder / TimeParameterizer / IK plumbing one stable internal shape while keeping
legacy dict commands working in tests during the transition.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping, Optional, Sequence, Tuple


_PROFILE_ALIASES = {
    "trapezoid": "trapezoid",
    "trapezoidal": "trapezoid",
    "trap": "trapezoid",
    "quintic": "quintic",
    "poly5": "quintic",
    "s_curve": "s_curve",
    "s-curve": "s_curve",
    "scurve": "s_curve",
    "smoothstep7": "s_curve",
    "toppra": "toppra",
    # Ruckig — jerk-limited (3rd-order) POINT-TO-POINT timing, the DEFAULT since 2026-10-11.
    # Cannot zone-blend (target-following, not path-following) — that is TOPPRA's
    # lane and always will be. See blending_design.md 10.
    "ruckig": "ruckig",
}
SUPPORTED_PROFILES = tuple(dict.fromkeys(_PROFILE_ALIASES.values()))

# The profile of every EXEC move that names none — ONE value, read by command_executor's defaults, the dry run
# (Plan Preview, the Blocks tab's Check), MotionOptions and the planner's fallbacks, so a preview always plans what the
# robot runs. "ruckig" since 2026-10-11 (user): on the arm (api_tests/66, J1 at 100 %) the trapezoid's abrupt end of
# braking left J1 ringing ~0.2 s (overshoot 0.076 deg); ruckig settled at once (0.014 deg) for ~1.6x the move time.
# `robot.set_profile("trapezoid")` (until the executor restarts) or a per-move profile="trapezoid" brings the old
# behaviour back. TODO/dynamics_feedforward_plan.md F4.
DEFAULT_PROFILE = "ruckig"

# The IK "solver" selects the IK ENGINE (and, with it, the FK/Jacobian backend the
# solve runs on):
#   "pinocchio"       -> damped-least-squares on Pinocchio's C++ Jacobian (default;
#                        deterministic, fast, no random restarts)
#   "roboticstoolbox" -> roboticstoolbox ik_LM primary + a DLS fallback, on rtb kinematics
_IK_SOLVER_ALIASES = {
    "pinocchio": "pinocchio",
    "pin": "pinocchio",
    "roboticstoolbox": "roboticstoolbox",
    "robotics_toolbox": "roboticstoolbox",
    "rtb": "roboticstoolbox",
}
SUPPORTED_IK_SOLVERS = tuple(dict.fromkeys(_IK_SOLVER_ALIASES.values()))

# `blend`'s UNIT depends on the mode — check which one you are in before reading a number:
#   exact_velocity -> blend is a PERCENT (pass through the waypoint carrying velocity)
#   zone           -> blend is a DISTANCE in mm (cut this much off each leg)
#   tolerance      -> blend is a MAX PATH DEVIATION in mm (`G64 P<tol>`); the blend distance
#                     is then derived PER CORNER from the turn angle, so sharp corners round
#                     tightly and shallow ones round freely. This is the knob G-code implies:
#                     a slicer emits no blend info at all, and a fixed mm distance does not
#                     compose across segments of varying length.
#   ruckig_corner   — EXPERIMENTAL, joint-space only, requires profile="ruckig". Does NOT
#                     prescribe corner geometry at all: legs are straight Ruckig lines, and
#                     the corner is a full-state (q, qd, qdd) Ruckig transition between the
#                     points where the legs cross a ball around the waypoint. Seams are C2 by
#                     construction and the WHOLE move is jerk-limited — the only lane that
#                     bounds jerk through a blend (TOPPRA is structurally jerk-blind). Measured
#                     on hardware vs `zone`: peak jerk 141x lower on a zigzag (225.6 -> 1.6
#                     rad/s^3) and 112x lower on a square (280 -> 2.5), for +46% to +59%
#                     duration. That time is the price of the jerk bound, not a defect: the
#                     legs are rest-to-rest and only the corner neighbourhood is cut out, so
#                     average speed stays below `zone`, which solves the whole chain in one
#                     TOPPRA pass. The corner path is INVENTED by
#                     Ruckig, so its deviation is measured, never guaranteed — this mode can
#                     therefore never back the `tolerance`/G64 contract. Not spliceable: a
#                     ruckig_corner chain is sent whole, so it does not participate in the
#                     rolling buffer (a stream still stops between flushes).
#                     See TODO/blending_design.md §10.1 and motion/ruckig_corner.py.
_BLEND_MODE_ALIASES = {
    "exact_velocity": "exact_velocity",
    "velocity": "exact_velocity",
    "zone": "zone",
    "tolerance": "tolerance",
    "g64": "tolerance",
    "ruckig_corner": "ruckig_corner",
    "ruckig": "ruckig_corner",
}
SUPPORTED_BLEND_MODES = tuple(dict.fromkeys(_BLEND_MODE_ALIASES.values()))

# Per-command singularity policy (TODO/singularity.md §6). The vocabulary is defined
# now so the wire/API surface is stable, but only "stop" is IMPLEMENTED in Phase 1:
#   stop              -> validate the whole path before motion; reject/decelerate on a
#                        singular/failed sample (today's effective behavior, made explicit)
#   variable_velocity -> slow through the singular region (Phase 2/3, not built)
#   auto_avoid        -> deform the path around it (Phase 5, not built)
# The two unbuilt strategies are recognized but REJECTED at validation, so a client
# can't silently select a no-op.
_SINGULARITY_STRATEGY_ALIASES = {
    "stop": "stop",
    "variable_velocity": "variable_velocity",
    "var_vel": "variable_velocity",
    "auto_avoid": "auto_avoid",
    "avoid": "auto_avoid",
}
SUPPORTED_SINGULARITY_STRATEGIES = tuple(dict.fromkeys(_SINGULARITY_STRATEGY_ALIASES.values()))
IMPLEMENTED_SINGULARITY_STRATEGIES = ("stop",)


def normalize_profile_name(profile: object) -> str:
    key = str(profile).strip().lower()
    try:
        return _PROFILE_ALIASES[key]
    except KeyError:
        raise ValueError(
            f"unknown motion profile '{profile}' "
            f"(expected one of {', '.join(SUPPORTED_PROFILES)})")


def normalize_singularity_strategy(strategy: object) -> str:
    key = str(strategy).strip().lower()
    try:
        canonical = _SINGULARITY_STRATEGY_ALIASES[key]
    except KeyError:
        raise ValueError(
            f"unknown singularity strategy '{strategy}' "
            f"(expected one of {', '.join(SUPPORTED_SINGULARITY_STRATEGIES)})")
    if canonical not in IMPLEMENTED_SINGULARITY_STRATEGIES:
        raise ValueError(
            f"singularity strategy '{canonical}' is not implemented yet "
            f"(currently only: {', '.join(IMPLEMENTED_SINGULARITY_STRATEGIES)})")
    return canonical


def normalize_ik_solver_name(solver: object) -> str:
    key = str(solver).strip().lower()
    try:
        return _IK_SOLVER_ALIASES[key]
    except KeyError:
        raise ValueError(
            f"unknown IK solver '{solver}' "
            f"(expected one of {', '.join(SUPPORTED_IK_SOLVERS)})")


def normalize_blend_mode_name(mode: object) -> str:
    key = str(mode).strip().lower()
    try:
        return _BLEND_MODE_ALIASES[key]
    except KeyError:
        raise ValueError(
            f"unknown blend mode '{mode}' "
            f"(expected one of {', '.join(SUPPORTED_BLEND_MODES)})")


SUPPORTED_BLEND_CONTINUITY = ("c1", "c2")


def normalize_blend_continuity(value):
    key = str(value).strip().lower()
    # Accept the geometric names too — that is how people will think about it.
    aliases = {"c1": "c1", "quadratic": "c1", "quad": "c1",
               "c2": "c2", "quintic": "c2"}
    try:
        return aliases[key]
    except KeyError:
        raise ValueError(
            f"unknown blend continuity '{value}' "
            f"(expected one of {', '.join(SUPPORTED_BLEND_CONTINUITY)})")


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    key = str(value).strip().lower()
    if key in ("1", "true", "yes", "on"):
        return True
    if key in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"expected boolean option, got {value!r}")


_ALLOWED_OPTION_KEYS = {
    "profile",
    "speed",
    "accel",
    "duration",
    "blend",
    "blend_mode",
    "blend_continuity",
    "ramp_fraction",
    "jerk",
    "ik_solver",
    "samples",
    "completion_policy",
    "singularity_strategy",
    "dry_run",
}


@dataclass(frozen=True)
class MotionOptions:
    """Planner options carried by every queued EXEC command.

    Units are client/API-facing unless noted otherwise:
      speed: 0..1 fraction or 1..100 percent
      blend: unit depends on blend_mode — percent (0..100) under "exact_velocity"
             (scales compute_corner_speed's preserved corner speed), millimeters
             under "zone" (physical Bezier blend-zone radius, the usual
             industrial convention — see TODO/blending_design.md §3/§4). Same field, two
             meanings; read blend_mode before interpreting it.
      blend_mode: "exact_velocity" (default, passes through the waypoint exactly),
             "zone" (Bezier corner-cutting at a fixed radius), "tolerance"/"g64"
             (corner-cutting sized PER CORNER so the path never strays more than
             `blend` mm from the programmed one — the knob G-code implies), or
             "ruckig_corner" (EXPERIMENTAL: jerk-limited corner invented by Ruckig
             instead of a prescribed Bezier; joint-space only, requires profile="ruckig",
             not spliceable, deviation measured rather than guaranteed).
             Zone/tolerance need the TOPPRA profile — see TODO/blending_design.md.
      jerk: RUCKIG ONLY. Percentage of the default jerk budget, which is 3x the
             ACCELERATION limit (RUCKIG_JERK_FACTOR). jerk=100 is today's behaviour;
             jerk=25 is four times gentler. Ruckig is the only profile that BOUNDS jerk —
             trapezoid and toppra STEP acceleration (their jerk is limited only by the 4ms
             tick), and quintic/s_curve are fixed shapes whose jerk falls out of the
             duration. Raises on any other profile rather than pretending.
             Note `accel` also scales the jerk budget, since it is defined relative to the
             acceleration limit — so accel=25 already quarters it.
      ramp_fraction: TRAPEZOID ONLY. The LSPB blend — the fraction of the move spent
             ACCELERATING (and again decelerating). 1/3 gives the classic
             1/3-accel, 1/3-cruise, 1/3-decel trapezoid (peak vel 1.5x average, the
             textbook "1/3 rule", the usual LSPB default). Our default is 0.25
             (1/4, 1/2, 1/4 — peak vel 1.333x, sharper ramps). 0.5 = triangular.
             Raises on any other profile: a quintic has no cruise phase to size.
      blend_continuity: the CORNER CURVE. "c1" (default, what has always shipped) is a
             quadratic Bezier: tangent-continuous, but curvature STEPS where the curve
             meets the straight legs, and curvature drives centripetal acceleration — so
             that step is an instantaneous acceleration change, i.e. infinite jerk, twice
             per corner. It is in the GEOMETRY: no profile, TOPPRA included, can time it
             away. "c2" is a quintic Bezier with zero second derivative at both ends, so
             curvature starts and ends at zero and matches the legs — no step, no spike.
             The trade is real and measured: the C2 curve is tighter at the apex (peak
             curvature 4.34 vs 1.41), so TOPPRA brakes harder there. C2 buys smoothness
             and pays corner speed, which is why you choose rather than being given one.
      accel: fraction of the ACCELERATION limit, 0..1 or 0..100 (the `a` of a
             (speed, accel, duration) joint move). Lower = gentler ramps = a longer move.
      duration: force the move to take exactly this many seconds (the `t`). OVERRIDES
             `speed`. Cannot override physics: too short => DURATION_INFEASIBLE.
      samples: explicit sample count override, reserved for dry-run/path tools
      dry_run: validate/diagnose a queued motion without sending hardware trajectory
    """

    profile: str = DEFAULT_PROFILE
    speed: Optional[float] = None
    accel: Optional[float] = None
    duration: Optional[float] = None
    blend: Optional[float] = None
    blend_mode: str = "exact_velocity"
    blend_continuity: str = "c1"
    ramp_fraction: Optional[float] = None
    jerk: Optional[float] = None
    ik_solver: Optional[str] = None
    samples: Optional[int] = None
    completion_policy: Optional[str] = None
    singularity_strategy: str = "stop"
    dry_run: Optional[bool] = None

    @classmethod
    def from_dict(cls, data: Optional[Mapping[str, Any]], base: Optional["MotionOptions"] = None):
        """Merge JSON options onto base defaults, validating known option names."""
        opts = base or cls()
        if not data:
            return opts
        unknown = sorted(set(data) - _ALLOWED_OPTION_KEYS)
        if unknown:
            raise ValueError(f"unknown motion option(s): {', '.join(unknown)}")

        updates = {}
        for key, value in data.items():
            if value is None:
                updates[key] = None
            elif key == "profile":
                updates[key] = normalize_profile_name(value)
            elif key == "ik_solver":
                updates[key] = normalize_ik_solver_name(value)
            elif key == "blend_mode":
                updates[key] = normalize_blend_mode_name(value)
            elif key == "singularity_strategy":
                updates[key] = normalize_singularity_strategy(value)
            elif key == "completion_policy":
                updates[key] = str(value).strip().lower()
            elif key in ("speed", "accel", "duration", "blend"):
                updates[key] = float(value)
            elif key == "samples":
                updates[key] = int(value)
            elif key == "dry_run":
                updates[key] = _parse_bool(value)
            elif key == "blend_continuity":
                updates[key] = normalize_blend_continuity(value)
            elif key == "jerk":
                j = float(value)
                scale = j if j <= 1.0 else j / 100.0        # accepts 0.25 or 25
                if not (0.0 < scale <= 1.0):
                    raise ValueError(
                        f"jerk must be in (0, 100] percent (or (0, 1] as a fraction) — it is a "
                        f"percentage of the default jerk budget (3x the acceleration limit). "
                        f"Got {value!r}")
                updates[key] = scale
            elif key == "ramp_fraction":
                r = float(value)
                if not (0.0 < r <= 0.5):
                    raise ValueError(
                        f"ramp_fraction must be in (0, 0.5] — it is the fraction of the move "
                        f"spent accelerating. 1/3 = the classic 1/3-1/3-1/3 trapezoid; 0.5 = "
                        f"triangular (no cruise). Got {value!r}")
                updates[key] = r
            else:
                # A key passed the _ALLOWED_OPTION_KEYS check but has no handler here, so it
                # would be SILENTLY DROPPED — the client sets it, the planner never sees it,
                # and nothing says a word. That is exactly what happened to blend_continuity:
                # it was added to the allow-list, the elif chain was not, and c2 was
                # unreachable from any client while the offline tests (which set the dataclass
                # field directly) all passed. Fail loudly instead.
                raise ValueError(
                    f"motion option '{key}' is allowed but has no parser in "
                    f"MotionOptions.from_dict — it would be silently ignored")
        return replace(opts, **updates)

    def to_dict(self):
        return {
            "profile": self.profile,
            "speed": self.speed,
            "accel": self.accel,
            "duration": self.duration,
            "blend": self.blend,
            "blend_mode": self.blend_mode,
            "ik_solver": self.ik_solver,
            "samples": self.samples,
            "completion_policy": self.completion_policy,
            "singularity_strategy": self.singularity_strategy,
            "dry_run": self.dry_run,
        }


@dataclass(frozen=True)
class QueuedCommand:
    """Internal queued EXEC command record."""

    id: str
    seq: int
    cmd: str
    args: Tuple[Any, ...]
    options: MotionOptions
    epoch: int = 0          # the planner's stop epoch when it was queued (TrajectoryPlanner.flush; command_executor)

    @classmethod
    def from_legacy(cls, data: Mapping[str, Any], default_options: Optional[MotionOptions] = None):
        return cls(
            id=str(data.get("id", "")),
            seq=int(data.get("seq", 0)),
            cmd=str(data.get("cmd", "")),
            args=tuple(data.get("args", ()) or ()),
            options=MotionOptions.from_dict(data.get("options"), default_options),
        )

    @classmethod
    def build(cls, cid: str, seq: int, cmd: str, args: Sequence[Any], options: MotionOptions):
        return cls(id=str(cid), seq=int(seq), cmd=str(cmd), args=tuple(args or ()), options=options)
