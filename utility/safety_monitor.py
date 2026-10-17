"""Safety monitor — the independent check on what the arm ACTUALLY does, every RT tick, in every mode.

Every serious arm splits safety into two layers (TODO/safety_stack_research.md §2-§3):

  1. PREVENTION, owned by whatever generates motion, and allowed to differ per mode — EXEC refuses a
     move that would leave the soft limits, JOG stops at them, RTI clamps/brakes to them.
  2. An independent MONITOR on the MEASURED state, the same in every mode, with a tolerance beyond
     the prevention limit, that STOPS when prevention failed or something outside the software moved
     the arm (a hand, a collision, a slipping reference).

This is layer 2. It replaces utility/safety_checks.run_safety_checks(), which was never called, cost
~1.1 ms of the 4 ms tick, and measured the wrong things (it pooled all motors into one array).

It is NOT a safety-rated function. Certified industrial equivalents run on redundant certified hardware
(PL d / Cat 3); this is single-channel Python on one core — the level of the software checks of an
external-control interface, which are not safety-rated either. The physical E-stop stays the
safety function; this catches failures and surprises, and stops.

WHAT IT CHECKS (per-joint keys are "J1:KEY".."J6:KEY", the joint's 1-based number, like every other joint error —
0-based until 2026-10-09):

  OUT_OF_RANGE    measured position past a SOFT limit + tolerance, in an ACTIVE mode (the robot is
                  moving itself: JOG, EXEC, RTI, ...). EDGE-TRIGGERED: fires when a joint goes from
                  inside to outside, never merely for BEING outside — an arm that rests or homes just
                  outside a limit must not be unable to start, and after a clear the joint can be jogged
                  back without re-tripping. Re-arms once it is back inside. A joint that is outside
                  WITHOUT having tripped (dragged there in freedrive) fires if it is then driven more
                  than the tolerance FURTHER out — the same "deeper" rule as the zones.
                  PASSIVE modes (IDLE — gravity-comp freedrive or velocity hold — and ACTIVE_ERROR) do
                  not trip it (user decision 2026-10-01): the robot commands no motion there, so the
                  only thing that moves the arm is a hand, and stopping the hand is wrong. Instead:
  SOFT_LIMIT_WARNING  a live WARNING ("J{i}:SOFT_LIMIT_WARNING") in every checked mode while a joint is
                  past its soft limit: JOG, EXEC and RTI will only move it back in. Clears itself.
  JOINT_WALL_WARNING / JOINT_WALL   PASSIVE modes only, per joint with a freedrive wall
                  (utility/haptic_walls.py — you FEEL it, at the soft limit by default, J6 at its hard limit):
                  pushed THROUGH it = more than the joint's breakthrough past the wall. The joint's `freedrive`
                  reaction decides: a live "J{i}:JOINT_WALL_WARNING" (default; clears once back inside the wall)
                  or the error "J{i}:JOINT_WALL" (stops). Set per joint in the robot XML / dashboard Walls tab.
  HARD_LIMIT      PASSIVE modes, a joint WITHOUT an endstop (J6, homed on a Hall sensor) and WITHOUT a wall:
                  measured past its hard limit + tolerance. There the hard limit is the cable wind-up boundary
                  and nothing physical stops a hand. (In active modes OUT_OF_RANGE trips long before.)
  BEYOND_ENDSTOP  measured position past a HARD limit + the ENDSTOP tolerance (robot XML
                  <endstop_tolerance_deg>, 10 deg on PAR6 — generous on purpose: homing differs between
                  robots and an endstop hit hard reads a few degrees past it), on joints that HAVE an
                  endstop. A joint cannot really get that far past its endstop, so this means the position
                  REFERENCE is wrong — error_checks invalidates homing. Joints homed on a Hall sensor (J6)
                  have no endstop; their hard limit is a wind-up boundary and OUT_OF_RANGE covers them.
  OVERSPEED       FILTERED speed above the joint's ceiling x margin. Never the raw speed: J6's raw
                  speed reads 1.2x its ceiling in normal motion (api_tests/46: a sampling pattern, not
                  motion). Every checked mode, freedrive included (user decision 2026-10-01: over the
                  ceiling in freedrive is more likely the arm FALLING than a hand, and the stop holds it).
  TRACKING        |commanded - measured| above a per-joint limit, in EXEC (where the commanded
                  position IS a position target). Catches a bump, a collision or a stall during
                  motion (a "position deviates from path" stop).
  TCP_OVERSPEED   filtered TCP translational speed above <safety><max_tcp_velocity> x margin. Inactive
                  while that limit is 0 (it ships 0) and until the check is switched on.
  These three have a MODE in <safety_monitor> (2026-10-01): "off", "warn" or "stop" (legacy 0/1 = off/stop).
  "warn" is for switching a check on before its thresholds are measured (user decision, shipped for
  overspeed and tracking): a trip is a live "<key>_WARNING" — J{i}:OVERSPEED_WARNING, J{i}:TRACKING_WARNING,
  TCP_OVERSPEED_WARNING — logged with the value, and when it clears the PEAK of the excursion is logged;
  nothing stops. Once normal use shows no false warnings, switch the check to "stop".
  HOLD_OSCILLATION_WARNING  (2026-10-06) a joint commanded to stand still whose MOTOR CURRENT keeps swinging
                  fast: the drive fighting itself — the audible whine of a joint with too-high PID gains (J3 on
                  PAR6, measured with api_tests/59_hold_noise.py: ~230 mA of ripple while buzzing, <= 25 mA in a
                  quiet hold once settled), or a loose encoder magnet (J5, 2026-08). The current each drive reports
                  is its instantaneous FOC current, so a whine of a few hundred Hz is SAMPLED at 250/s (folded
                  below 125 Hz), not averaged away. Per tick: a first-order high-pass at <hold_oscillation_hz>
                  (removes the slow settling and a hand's push), squared and averaged over HOLD_OSC_EMA_S; above
                  the joint's robot-XML <hold_oscillation_ma> for <hold_oscillation_sustain_s> while the joint's
                  COMMANDED speed is under <hold_still_dps> -> a live warning (never a stop). The sustain matters:
                  a hold often KICKS for 0.25-0.5 s as it engages (up to ~290 mA measured) and then settles, and a
                  buzz can die out by itself (one did after ~0.8 s) — 1.5 s shipped.
                  Clears once the level is under half the threshold or the joint moves; RTI.log gets the peak.
  SELF_COLLISION  (2026-10-06) two links within the self-collision MONITOR margin on the measured pose — not computed
                  here: the collision WORKER process (self_collision_worker.py, utility/self_collision.py: the convex
                  pieces through coal, ~100 Hz on another core) writes rti['sc_hit'] / the pair / the gap, and this
                  monitor reads them (a folded pose costs the check 1-2.5 ms, too much for the tick). Active modes
                  only (freedrive: nothing, user decision D5); 2 ticks of sustain; edge-triggered, re-arms when the
                  hit clears. <self_collision><reaction> off | warn | stop. SELF_COLLISION_STALE is a live warning
                  when the worker's heartbeat (rti['sc_heartbeat']) has not moved for 0.5 s: nothing is watching.
                  ESCAPE rule (2026-10-07, user decision — the joint limits' and zones' rule): a pair already within
                  the margin on the first fresh tick of an active period (the arm pushed there in freedrive) does
                  not trip; only that pair getting SC_ESCAPE_TOL_M (2 mm) closer than then, or ANOTHER pair coming
                  within the margin, does — read per pair from rti['sc_pair_gaps']. Every passive period re-takes it.
  CONTACT_WARNING / COLLISION  (2026-10-16) the external joint torque the momentum observer estimates
                  (tcp['torque_external'], utility/momentum_observer.py) minus a BASELINE taken at rest before each move
                  — the model's own error at that pose and the stiction the joint settled with — above the joint's
                  robot-XML <contact_torque_nm> (a live J{i}:CONTACT_WARNING, always a warning) or <collision_torque_nm>
                  (<safety_monitor><collision>: warn -> a live J{i}:COLLISION_WARNING; stop -> the error J{i}:COLLISION,
                  the normal safety stop). Only in EXEC, JOG, RTI and SOFT_RTI (user decision; never in freedrive,
                  hand guiding or homing), only while the arm is commanded to move and for <collision_settle_s> after
                  (a joint commanded faster than <hold_still_dps>): at rest the baseline follows the estimate (an EMA
                  of COLLISION_BASELINE_TAU_S), so whatever the arm rests with is not a contact. Sustain
                  <collision_sustain_s>; nothing for OBSERVER_SETTLE_S after the observer (re)starts. A warning clears
                  once the joint is under half its contact threshold, or at rest; RTI.log gets the peak. A PC program
                  that pushes on purpose switches it with the collision_detection command (off | warn | stop, until
                  RTI restarts). The thresholds are the measured model error (api_tests/76_external_torque.py): an
                  undeclared payload (set_payload) reads as external torque too.
  ZONE_VIOLATION  (key "Z_<zone>:ZONE_VIOLATION") the measured TCP or a tool sphere past a STOP-zone's
                  surface + <zone_tolerance> (utility/zones.py; the zone's margin is the planner's, not
                  counted here). Edge-triggered like OUT_OF_RANGE, but it fires AGAIN if the robot is
                  driven deeper by more than the tolerance — JOG does not stop at zone walls yet and RTI
                  is monitor-only in v1, so after a clear nothing else would stop it going further in.
  ZONE_WARNING    ("Z_<zone>:ZONE_WARNING") the same for a WARN-zone, but a live WARNING, not an error:
                  held in `warnings` (one set, mutated in place, shared with SOFT_LIMIT_WARNING) while
                  the robot is in the zone.
                  In PASSIVE modes (freedrive) a zone uses its `freedrive` reaction instead (warn by default,
                  or stop), and a HAPTIC zone (utility/haptic_walls.py — you feel it as a wall) only counts as
                  passed at <haptic_walls><breakthrough> past its surface, not at the tolerance.

Every check needs its condition for a sustain window (seconds, converted to ticks at start-up) so a
single impossible sample cannot trip it. Nothing is checked while the robot is not homed (positions
mean nothing), or in HOMING (drives into the endstops on purpose), FLASHING and BOOTING (no valid
state), or SAFETY_STOP (deliberately limp — error_checks would re-energise it into a velocity hold).

HOT PATH. step() runs every tick: it allocates nothing on a quiet tick (preallocated buffers, `out=`
everywhere, reads fixed views of the shared-memory arrays) and returns the shared empty tuple. Strings
and a tuple are built only on the tick something fires.
"""
from __future__ import annotations

import logging
logger = logging.getLogger("rti.safety")  # docs/logging.md "Logger names"
import math

import numpy as np

from utility.state_machine import RobotMode
from utility.zones import ZONE_VIOLATION, ZONE_WARNING, penetration_at, robot_spheres, sphere_centres, zone_key

OUT_OF_RANGE = "OUT_OF_RANGE"
BEYOND_ENDSTOP = "BEYOND_ENDSTOP"
OVERSPEED = "OVERSPEED"
TRACKING = "TRACKING"
TCP_OVERSPEED = "TCP_OVERSPEED"
SOFT_LIMIT_WARNING = "SOFT_LIMIT_WARNING"
HARD_LIMIT = "HARD_LIMIT"
OVERSPEED_WARNING = "OVERSPEED_WARNING"
TRACKING_WARNING = "TRACKING_WARNING"
TCP_OVERSPEED_WARNING = "TCP_OVERSPEED_WARNING"
JOINT_WALL = "JOINT_WALL"
JOINT_WALL_WARNING = "JOINT_WALL_WARNING"
HOLD_OSCILLATION_WARNING = "HOLD_OSCILLATION_WARNING"
SELF_COLLISION = "SELF_COLLISION"
SELF_COLLISION_WARNING = "SELF_COLLISION_WARNING"
SELF_COLLISION_STALE = "SELF_COLLISION_STALE"
SC_SUSTAIN_S = 0.008            # two ticks of the worker saying "hit" before it counts
SC_STALE_S = 0.5                # the worker's heartbeat unchanged this long -> SELF_COLLISION_STALE
SC_ESCAPE_TOL_M = 0.002         # escape rule: a pair in contact at the start of an active period may get this much closer
SC_PAIRS = 49                   # rti['sc_pair_gaps'] = 7 x 7 frames, index a * 7 + b
HOLD_OSC_EMA_S = 0.1            # s: the averaging of the high-passed current's square (the level's time constant)
CONTACT_WARNING = "CONTACT_WARNING"
COLLISION = "COLLISION"
COLLISION_WARNING = "COLLISION_WARNING"
COLLISION_BASELINE_TAU_S = 0.1  # s: at rest the baseline follows the external torque estimate with this time constant
OBSERVER_SETTLE_S = 0.1         # s: no collision check this long after the observer (re)starts (5 of its time constants)
MONITOR_KEYS = frozenset({OUT_OF_RANGE, BEYOND_ENDSTOP, OVERSPEED, TRACKING, TCP_OVERSPEED, ZONE_VIOLATION,
                          HARD_LIMIT, JOINT_WALL, SELF_COLLISION, COLLISION})

EXEMPT_MODES = frozenset({RobotMode.HOMING, RobotMode.FLASHING, RobotMode.BOOTING, RobotMode.SAFETY_STOP})
TRACKING_MODES = frozenset({RobotMode.EXEC})
# Collision detection runs where the ROBOT drives the arm (user decision 2026-10-16) — not in freedrive / hand guiding /
# impedance, where a hand or a contact is the point.
COLLISION_MODES = frozenset({RobotMode.EXEC, RobotMode.JOG, RobotMode.RTI, RobotMode.SOFT_RTI})
# The robot commands NO motion in these: IDLE holds gravity torque (freedrive) or zero velocity, ACTIVE_ERROR
# holds zero velocity. A soft limit passed here was passed by a hand — warn, do not stop (2026-10-01).
PASSIVE_MODES = frozenset({RobotMode.IDLE, RobotMode.ACTIVE_ERROR})
SOFT_WARN_HYSTERESIS_RAD = np.radians(0.1)     # the warning clears 0.1 deg back inside, so it cannot flap

_NONE = ()


def check_mode(value: object) -> str:
    """A check's policy value -> "off" | "warn" | "stop". Accepts the words and the legacy 0/1 (False/True)."""
    if isinstance(value, str):
        v = value.strip().lower()
        return v if v in ("off", "warn", "stop") else ("stop" if v == "1" else "off")
    return "stop" if value else "off"


def joint_key(i: int, key: str) -> str:
    """Same format as error_checks._joint_key — index i (0-based) -> 'J{i+1}:KEY', the joint's 1-based number (2026-10-09)
    (checked by the test; importing it would be circular)."""
    return f"J{i + 1}:{key}"


def sustain_ticks(seconds: float, dt: float) -> int:
    """A window in seconds -> whole ticks, at least 1 (so it means the same on a 100 Hz robot)."""
    return max(1, int(np.ceil(float(seconds) / float(dt) - 1e-9)))


class SafetyMonitor:
    """See the module docstring. Construct once before the RT loop; call step(mode) every tick."""

    def __init__(self, joint_num, dt, *, q_meas, q_cmd, v_filt, tcp_v_filt, homing_status,
                 soft_lo, soft_hi, hard_lo, hard_hi, has_endstop, vel_ceiling,
                 tolerance_rad, endstop_tolerance_rad, tracking_limit_rad, tcp_limit, policy,
                 tcp_pose=None, zones=(), tool_spheres=(), haptic=None, joint_walls=None,
                 i_meas=None, v_cmd=None, hold_osc_ma=None, sc=None, sc_policy=None,
                 t_ext=None, contact_nm=None, collision_nm=None, ext_observer=None):
        n = int(joint_num)
        self.n = n
        # Fixed VIEWS of the live shared-memory arrays: reading them allocates nothing.
        self._q = q_meas
        self._qc = q_cmd
        self._v = v_filt
        self._tcp = tcp_v_filt
        self._homing = homing_status

        f = lambda a: np.asarray(a, dtype=np.float64)[:n].copy()
        self.soft_lo, self.soft_hi = f(soft_lo), f(soft_hi)
        tol = float(tolerance_rad)
        self.tol = tol
        # NaN limits (a robot XML without them) must never trip anything: make them infinite.
        self._lim_lo = np.where(np.isfinite(self.soft_lo), self.soft_lo - tol, -np.inf)
        self._lim_hi = np.where(np.isfinite(self.soft_hi), self.soft_hi + tol, np.inf)
        self._in_lo = np.where(np.isfinite(self.soft_lo), self.soft_lo, -np.inf)
        self._in_hi = np.where(np.isfinite(self.soft_hi), self.soft_hi, np.inf)
        hard_lo, hard_hi = f(hard_lo), f(hard_hi)
        endstop = np.asarray(has_endstop, dtype=bool)[:n]
        ok = endstop & np.isfinite(hard_lo) & np.isfinite(hard_hi)
        self.end_tol = float(endstop_tolerance_rad)
        self._end_lo = np.where(ok, hard_lo - self.end_tol, -np.inf)
        self._end_hi = np.where(ok, hard_hi + self.end_tol, np.inf)
        self._end_in_lo = np.where(ok, hard_lo, -np.inf)
        self._end_in_hi = np.where(ok, hard_hi, np.inf)
        # PASSIVE-mode bound per joint (JOINT_WALL / HARD_LIMIT): trip points, the inside it re-arms at, and the
        # reaction. set_joint_walls() fills them from the resolved freedrive walls.
        self._endstop, self._hard_lo, self._hard_hi = endstop, hard_lo, hard_hi
        self._pb_lo, self._pb_hi = np.full(n, -np.inf), np.full(n, np.inf)
        self._pb_in_lo, self._pb_in_hi = np.full(n, -np.inf), np.full(n, np.inf)
        self._pb_wall = np.zeros(n, dtype=bool)                  # a freedrive wall (else: wind-up hard limit)
        self._pb_warn = np.zeros(n, dtype=bool)                  # pushed through -> warn instead of stop
        self._pb_warned = False                                  # some JOINT_WALL_WARNING is live
        ceiling = f(vel_ceiling)
        self._v_ceiling = np.where(np.isfinite(ceiling) & (ceiling > 0), ceiling, np.inf)
        self._v_trip = self._v_ceiling * float(policy["overspeed_margin"])
        trk = f(tracking_limit_rad)
        self._trk = np.where(np.isfinite(trk) & (trk > 0), trk, np.inf)   # 0 / NaN = that joint off
        self._trk_rearm = self._trk * 0.5
        self._tcp_limit = float(tcp_limit) if tcp_limit and tcp_limit > 0 else 0.0
        self._tcp_trip = self._tcp_limit * float(policy["tcp_speed_margin"])

        self.enabled = bool(policy["enabled"])
        self.check_range = bool(policy["joint_limit"])
        self.check_endstop = bool(policy["endstop"]) and bool(ok.any())
        self._check_pb_policy = bool(policy["joint_limit"])
        self.check_passive_bound = False                         # set by set_joint_walls()
        self.speed_mode = check_mode(policy["overspeed"])
        self.tracking_mode = check_mode(policy["tracking"])
        self.tcp_mode = check_mode(policy["tcp_speed"])
        self.check_speed = self.speed_mode != "off"
        self.check_tracking = self.tracking_mode != "off" and bool(np.isfinite(self._trk).any())
        self.check_tcp = self.tcp_mode != "off" and self._tcp_trip > 0.0
        self._n_pos = sustain_ticks(policy["position_sustain_s"], dt)
        self._n_spd = sustain_ticks(policy["overspeed_sustain_s"], dt)
        self._n_trk = sustain_ticks(policy["tracking_sustain_s"], dt)
        self._n_tcp = sustain_ticks(policy["tcp_speed_sustain_s"], dt)

        # Per-check state: a counter of consecutive bad ticks and an ARMED flag (fire at most once
        # per excursion; re-armed only once the joint is properly back in range).
        z = lambda dtype: np.zeros(n, dtype=dtype)
        self._c_rng, self._a_rng = z(np.int64), z(bool)
        self._c_end, self._a_end = z(np.int64), z(bool)
        self._c_spd, self._a_spd = z(np.int64), z(bool)
        self._c_trk, self._a_trk = z(np.int64), z(bool)
        self._c_pb, self._a_pb = z(np.int64), z(bool)
        self._c_tcp = 0
        self._a_tcp = False
        # Soft-limit warning state and the "deeper" check for joints outside without a trip.
        self._w_soft = z(bool)                     # joint currently warned
        self._soft_warned = False                  # any joint warned (skips work on a quiet tick)
        self._deep_dirty = False                   # the "deeper" state needs clearing in a passive mode
        self._in_lo_h = self._in_lo + SOFT_WARN_HYSTERESIS_RAD
        self._in_hi_h = self._in_hi - SOFT_WARN_HYSTERESIS_RAD
        self._c_deep = z(np.int64)
        self._r_ref = np.full(n, np.nan)           # how far past the soft limit it was (NaN = unknown)
        # Scratch.
        self._b1, self._b2, self._b3 = z(bool), z(bool), z(bool)
        self._b4, self._b5 = z(bool), z(bool)
        self._f1, self._f2, self._f3 = z(np.float64), z(np.float64), z(np.float64)
        # homing_status must be the JOINTS-ONLY view (no gripper slot): slicing it here every tick
        # would allocate a new view object per tick.
        self._bh = np.zeros(np.asarray(homing_status).shape, dtype=bool)
        # Keys prebuilt, so firing formats nothing.
        self._k_rng = [joint_key(i, OUT_OF_RANGE) for i in range(n)]
        self._k_end = [joint_key(i, BEYOND_ENDSTOP) for i in range(n)]
        self._k_spd = [joint_key(i, OVERSPEED) for i in range(n)]
        self._k_trk = [joint_key(i, TRACKING) for i in range(n)]
        self._k_spdw = [joint_key(i, OVERSPEED_WARNING) for i in range(n)]
        self._k_trkw = [joint_key(i, TRACKING_WARNING) for i in range(n)]
        # Warn mode: the peak of each live excursion (logged when its warning clears).
        self._pk_spd, self._pk_trk = z(np.float64), z(np.float64)
        self._spd_warned = self._trk_warned = False
        self._pk_tcp = 0.0
        self._k_soft = [joint_key(i, SOFT_LIMIT_WARNING) for i in range(n)]
        self._k_hard = [joint_key(i, HARD_LIMIT) for i in range(n)]
        self._k_wall = [joint_key(i, JOINT_WALL) for i in range(n)]
        self._k_wallw = [joint_key(i, JOINT_WALL_WARNING) for i in range(n)]
        self._k_pb = list(self._k_hard)                          # per joint: the passive-bound error key
        # HOLD_OSCILLATION_WARNING: fixed views of the motor currents (mA, joints only) and the commanded joint speeds;
        # the per-joint threshold (robot XML, mA; 0 / NaN = that joint unchecked).
        self._i, self._vc = i_meas, v_cmd
        thr = f(hold_osc_ma) if hold_osc_ma is not None else np.zeros(n)
        thr = np.where(np.isfinite(thr) & (thr > 0), thr, np.inf)
        self._ho_thr = thr
        self._ho_thr2, self._ho_clear2 = thr * thr, 0.25 * thr * thr          # squared: no sqrt per tick
        self.hold_osc_mode = check_mode(policy.get("hold_oscillation", "off"))
        if self.hold_osc_mode == "stop":                                       # warn-only by design (the user's ask)
            logger.warning("<hold_oscillation> supports off | warn — using warn")
            self.hold_osc_mode = "warn"
        self.check_hold_osc = (self.hold_osc_mode != "off" and i_meas is not None and v_cmd is not None
                               and bool(np.isfinite(thr).any()))
        self._ho_hz = float(policy.get("hold_oscillation_hz", 25.0))
        tau = 1.0 / (2.0 * np.pi * self._ho_hz)
        self._ho_a = tau / (tau + float(dt))                                   # first-order high-pass
        self._ho_b = float(dt) / HOLD_OSC_EMA_S                                # EMA of the square
        self._ho_still = np.radians(float(policy.get("hold_still_dps", 0.5)))
        self._ho_n = sustain_ticks(policy.get("hold_oscillation_sustain_s", 1.5), dt)
        self._ho_sustain_s = self._ho_n * float(dt)
        self._ho_prev, self._ho_hp, self._ho_ms, self._ho_f = z(np.float64), z(np.float64), z(np.float64), z(np.float64)
        self._ho_pk = z(np.float64)                                            # peak level (mA^2) of a live warning
        self._ho_c = z(np.int64)
        self._ho_b1, self._ho_b2, self._ho_b3 = z(bool), z(bool), z(bool)
        self._ho_live = z(bool)
        self._ho_any = False                                                   # some HOLD_OSCILLATION_WARNING live
        self._ho_counting = False                                              # some counter may be non-zero
        self._ho_fresh = True                                                  # restart the filter on the next tick
        self._k_how = [joint_key(i, HOLD_OSCILLATION_WARNING) for i in range(n)]
        # CONTACT_WARNING / COLLISION(_WARNING): a fixed view of the external torque estimate (tcp['torque_external'],
        # joints only), the per-joint thresholds (robot XML, Nm; 0 / NaN = that threshold unchecked) and the observer
        # (its `running` / `resets`: no check while it starts again). The commanded speeds (v_cmd) say "moving".
        self._te, self._obs = t_ext, ext_observer
        def _threshold(a):
            a = f(a) if a is not None else np.zeros(n)
            return np.where(np.isfinite(a) & (a > 0), a, np.inf)
        self._cn_thr, self._co_thr = _threshold(contact_nm), _threshold(collision_nm)
        self._cl_lo = np.minimum(self._cn_thr, self._co_thr)                   # the lower of the two: the early out
        self._cl_clear = 0.5 * self._cl_lo                                      # a warning clears under this
        self._cl_possible = t_ext is not None and v_cmd is not None and bool(np.isfinite(self._cl_lo).any())
        self.collision_mode = check_mode(policy.get("collision", "off"))
        self.check_collision = self._cl_possible and self.collision_mode != "off"
        self._cl_n = sustain_ticks(policy.get("collision_sustain_s", 0.012), dt)
        self._cl_settle_n = sustain_ticks(policy.get("collision_settle_s", 0.5), dt)
        self._cl_hold_n = sustain_ticks(OBSERVER_SETTLE_S, dt)
        self._cl_k = min(1.0, float(dt) / COLLISION_BASELINE_TAU_S)              # the baseline's EMA at rest
        self._cl_base, self._cl_d = z(np.float64), z(np.float64)
        self._cl_pk = z(np.float64)                                             # a live excursion's peak, Nm
        self._cl_cc, self._cl_oc = z(np.int64), z(np.int64)                     # ticks over contact / collision
        self._cl_b1, self._cl_b2, self._cl_b3 = z(bool), z(bool), z(bool)
        self._cl_live_c, self._cl_live_w = z(bool), z(bool)                     # CONTACT_ / COLLISION_WARNING live
        self._cl_armed = np.ones(n, dtype=bool)                                 # COLLISION fires once per excursion
        self._cl_any = self._cl_counting = False
        self._cl_fresh = True                                                   # take the baseline on the next tick
        self._cl_since = self._cl_settle_n                                      # ticks since the arm was commanded to move
        self._cl_hold = 0                                                       # ticks left without a check
        self._cl_resets = ext_observer.resets if ext_observer is not None else 0
        self._k_ct = [joint_key(i, CONTACT_WARNING) for i in range(n)]
        self._k_cl = [joint_key(i, COLLISION) for i in range(n)]
        self._k_clw = [joint_key(i, COLLISION_WARNING) for i in range(n)]
        # SELF_COLLISION from the worker's shared-memory fields: sc = {"hit", "heartbeat", "pair_a", "pair_b", "gap"} views.
        self._sc = sc
        self.sc_mode = check_mode((sc_policy or {}).get("reaction", "off")) if (sc_policy or {}).get("enabled", 0) else "off"
        self.check_self_collision = sc is not None and self.sc_mode != "off"
        self._sc_n = sustain_ticks(SC_SUSTAIN_S, dt)
        self._sc_stale_n = sustain_ticks(SC_STALE_S, dt)
        self._sc_c, self._sc_armed, self._sc_live = 0, True, False
        self._sc_last_hb, self._sc_hb_ticks, self._sc_stale = -1.0, 0, False
        # the escape rule: per pair, the gap below which it trips — +inf (any gap = within the margin = trip) for a pair
        # that was clear when the active period began, (its gap then - SC_ESCAPE_TOL_M) for one already within it
        self._sc_gaps = sc.get("pair_gaps") if sc is not None else None
        self._sc_thr = np.full(SC_PAIRS, np.inf)
        self._sc_viol = np.zeros(SC_PAIRS, dtype=bool)
        self._sc_rebase = True
        if self.check_self_collision and self._sc_gaps is None:
            raise ValueError("SafetyMonitor: self-collision checking needs sc['pair_gaps'] (rti['sc_pair_gaps'])")
        try:
            from utility.self_collision import PAR6_NAMES as _sc_names        # names only; no Pinocchio at import
            self._sc_names = _sc_names
        except Exception:                                                     # pragma: no cover
            self._sc_names = tuple(f"frame {i}" for i in range(7))
        # Safety zones. _pose is a fixed VIEW of the measured flat 4x4 TCP pose (tcp['robot_pose_matrix']).
        self._pose = tcp_pose
        self._z_tol = float(policy.get("zone_tolerance", 0.005))
        self._zones_on = bool(policy.get("zones", 1)) and tcp_pose is not None
        self.warnings = set()               # live warning keys (zones, soft limits); error_checks reads it
        self.details = {}                   # key -> the measured value behind it, written when it fires (2026-10-15):
                                            # the error list's and the activity log's message (error_checks detail_of)
        self.set_zones(zones, tool_spheres, log=False, haptic=haptic)
        self.set_joint_walls(joint_walls, log=False)
        self._warn_config()

    def set_zones(self, zones, tool_spheres, log=True, haptic=None):
        """Install the active cell's zones (startup, and RTI's reload_zones command). Cold path: builds
        the per-zone state lists that step() then only writes into."""
        self._zones = tuple(z for z in zones if z["enabled"])
        self._spheres = robot_spheres(tool_spheres)
        nz = len(self._zones)
        self._z_count = [0] * nz
        self._z_armed = [False] * nz                     # armed only once seen clear of the zone
        self._z_ref = [math.nan] * nz                    # depth at the last fire (NaN = not known)
        # Reaction and trip depth per zone, ACTIVE modes vs PASSIVE (freedrive) modes.
        haptic = haptic or {}
        walls_on = bool(haptic.get("enabled", 1)) and float(haptic.get("max_force", 1.0)) > 0.0
        breakthrough = float(haptic.get("breakthrough", 0.03))
        self._z_warn = [z["reaction"] == "warn" for z in self._zones]
        self._z_pas_warn = [z["reaction"] == "warn" or z.get("freedrive", "warn") == "warn" for z in self._zones]
        self._z_act_trip = [self._z_tol] * nz
        self._z_pas_trip = [breakthrough if (walls_on and z.get("haptic", True)) else self._z_tol
                            for z in self._zones]
        self._z_wkeys = [zone_key(z, ZONE_WARNING) for z in self._zones]
        self._z_ekeys = [zone_key(z, ZONE_VIOLATION) for z in self._zones]
        ns = len(self._spheres)
        self._z_n = [1 if z["checks"] == "tcp" else ns for z in self._zones]
        self._cx, self._cy, self._cz = [0.0] * ns, [0.0] * ns, [0.0] * ns   # sphere centres, per tick
        for k in [k for k in self.warnings if k.startswith("Z_")]:
            self.warnings.discard(k)                     # the soft-limit warnings stay
        self.check_zones = self._zones_on and nz > 0
        if log:
            logger.info(f"zones reloaded: {nz} enabled, {len(self._spheres)} robot sphere(s), "
                         f"zone checks {'ON' if self.check_zones else 'OFF'}")

    def set_joint_walls(self, walls, log=True):
        """Install the resolved freedrive joint walls (utility/haptic_walls.resolve_joint_walls) — startup, live
        tuning (set_joint_walls command), reload_zones. Cold path. A joint with an enabled wall is pushed through at
        wall +/- its breakthrough (its own warn/stop); a joint without an endstop and without a wall stops at its
        hard limit + tolerance; any other joint has no passive bound (BEYOND_ENDSTOP covers it)."""
        n, tol = self.n, self.tol
        byj = {w["joint"]: w for w in (walls or ()) if w.get("enabled") and 0 <= w["joint"] < n}
        for i in range(n):
            w = byj.get(i)
            if w is not None:
                bt = float(w["bt"])
                self._pb_in_lo[i], self._pb_in_hi[i] = float(w["lo"]), float(w["hi"])
                self._pb_lo[i], self._pb_hi[i] = float(w["lo"]) - bt, float(w["hi"]) + bt
                self._pb_wall[i], self._pb_warn[i] = True, w["freedrive"] == "warn"
                self._k_pb[i] = self._k_wall[i]
            elif not self._endstop[i] and np.isfinite(self._hard_lo[i]) and np.isfinite(self._hard_hi[i]):
                self._pb_in_lo[i], self._pb_in_hi[i] = self._hard_lo[i], self._hard_hi[i]
                self._pb_lo[i], self._pb_hi[i] = self._hard_lo[i] - tol, self._hard_hi[i] + tol
                self._pb_wall[i], self._pb_warn[i] = False, False
                self._k_pb[i] = self._k_hard[i]
            else:
                self._pb_in_lo[i], self._pb_in_hi[i] = -np.inf, np.inf
                self._pb_lo[i], self._pb_hi[i] = -np.inf, np.inf
                self._pb_wall[i], self._pb_warn[i] = False, False
                self._k_pb[i] = self._k_hard[i]
        self._a_pb[:] = False                    # re-arm only once seen inside the new bound
        self._c_pb[:] = 0
        self._pb_warned = any(k in self.warnings for k in self._k_wallw)   # live ones clear once back inside
        self.check_passive_bound = self._check_pb_policy and bool(np.isfinite(self._pb_lo).any()
                                                                  or np.isfinite(self._pb_hi).any())
        if log:
            logger.info(f"joint walls set: pushed through at {self._pb_text()}")

    def _pb_text(self):
        parts = []
        for i in range(self.n):
            if self._pb_wall[i]:
                parts.append(f"J{i + 1} wall +/-{np.degrees(self._pb_in_lo[i] - self._pb_lo[i]):.1f} deg "
                             f"({'warn' if self._pb_warn[i] else 'stop'})")
            elif np.isfinite(self._pb_lo[i]):
                parts.append(f"J{i + 1} hard limit (stop)")
        return ", ".join(parts) or "none"

    def _collision_text(self):
        if not self._cl_possible:
            return "off (no thresholds in the robot XML)" if self._te is not None else "off (no external torque)"
        if not self.check_collision:
            return "off"
        lim = ", ".join(f"J{i + 1} " + "/".join(f"{v:.2f}" if np.isfinite(v) else "-" for v in (c, o))
                        for i, (c, o) in enumerate(zip(self._cn_thr, self._co_thr)))
        return (f"{self.collision_mode} (contact/collision {lim} Nm over the baseline taken at rest; EXEC/JOG/RTI/"
                f"SOFT_RTI while commanded to move + {self._cl_settle_n} ticks, sustain {self._cl_n} ticks)")

    def _hold_osc_text(self):
        if not self.check_hold_osc:
            return "off"
        lim = "/".join(f"{v:g}" if np.isfinite(v) else "-" for v in self._ho_thr)
        return (f"{self.hold_osc_mode} ({lim} mA J1-J6, current above {self._ho_hz:g} Hz, {self._ho_n} ticks, "
                f"commanded under {np.degrees(self._ho_still):g} deg/s)")

    # ------------------------------------------------------------------ setup checks (cold)
    def _warn_config(self):
        margin = np.minimum(self._in_lo - self._end_in_lo, self._end_in_hi - self._in_hi)
        tight = np.flatnonzero(np.isfinite(margin) & (margin + self.end_tol <= self.tol))
        for i in tight:
            logger.warning(f"J{i + 1}: soft-to-hard margin {np.degrees(margin[i]):.2f} deg + the "
                            f"{np.degrees(self.end_tol):.2f} deg endstop tolerance is not larger than the "
                            f"{np.degrees(self.tol):.2f} deg tolerance — OUT_OF_RANGE and BEYOND_ENDSTOP would fire together")
        logger.info(
            f"monitor {'ON' if self.enabled else 'OFF'}: out-of-range {self.check_range} (warning only in "
            f"IDLE/ACTIVE_ERROR), freedrive bounds {self._pb_text() if self.check_passive_bound else 'off'}, "
            f"beyond-endstop {self.check_endstop}, overspeed {self.speed_mode if self.check_speed else 'off'}, "
            f"tracking {self.tracking_mode if self.check_tracking else 'off'}, "
            f"TCP speed {self.tcp_mode if self.check_tcp else 'off'}, hold oscillation {self._hold_osc_text()}, "
            f"self-collision {self.sc_mode if self.check_self_collision else 'off'} (from the worker), "
            f"collision {self._collision_text()}, "
            f"zones {self.check_zones} ({len(self._zones)} enabled, "
            f"{len(self._spheres)} robot sphere(s), tolerance {1000 * self._z_tol:.1f} mm); "
            f"tolerance {np.degrees(self.tol):.2f} deg (endstops {np.degrees(self.end_tol):.2f} deg), position sustain "
            f"{self._n_pos} ticks")

    # ------------------------------------------------------------------ per tick
    def _disarm(self):
        self._a_rng[:] = False
        self._a_end[:] = False
        self._a_spd[:] = False
        self._a_trk[:] = False
        self._c_rng[:] = 0
        self._c_end[:] = 0
        self._c_spd[:] = 0
        self._c_trk[:] = 0
        self._c_tcp = 0
        self._a_tcp = False
        self._spd_warned = self._trk_warned = False
        self._pk_tcp = 0.0
        for i in range(len(self._zones)):
            self._z_count[i] = 0
            self._z_armed[i] = False
            self._z_ref[i] = math.nan
        self._a_pb[:] = False
        self._c_pb[:] = 0
        self._c_deep[:] = 0
        self._r_ref.fill(np.nan)
        self._w_soft[:] = False
        self._soft_warned = False
        self._deep_dirty = False
        self._pb_warned = False
        self._ho_c[:] = 0
        self._ho_live[:] = False
        self._ho_any = False
        self._ho_fresh = True
        self._sc_c, self._sc_armed, self._sc_live, self._sc_stale = 0, True, False, False
        self._sc_rebase = True
        self._collision_reset()
        if self.warnings:
            self.warnings.clear()

    def _band(self, x, lo, hi, in_lo, in_hi, count, armed, n_trip):
        """Shared out-of-band machinery. Returns a bool array of joints that fire THIS tick, or None.
        bad = outside [lo, hi]; re-arm = inside [in_lo, in_hi]; fire = armed and bad for n_trip ticks."""
        b1, b2, b3 = self._b1, self._b2, self._b3
        np.less(x, lo, out=b1)
        np.greater(x, hi, out=b2)
        np.logical_or(b1, b2, out=b3)                          # b3 = bad now
        if not b3.any():
            # The common tick: nothing out of band. Reset the counters, and only do the re-arm
            # work if some joint is still disarmed (after an excursion or an exempt period).
            count.fill(0)
            if not armed.all():
                np.greater_equal(x, in_lo, out=b1)
                np.less_equal(x, in_hi, out=b2)
                np.logical_and(b1, b2, out=b1)
                np.logical_or(armed, b1, out=armed)
            return None
        count += b3
        count *= b3                                            # consecutive bad ticks
        np.greater_equal(x, in_lo, out=b1)
        np.less_equal(x, in_hi, out=b2)
        np.logical_and(b1, b2, out=b1)                         # b1 = properly inside
        np.logical_or(armed, b1, out=armed)
        np.equal(count, n_trip, out=b2)
        np.logical_and(b2, armed, out=b2)                      # b2 = fire
        return b2 if b2.any() else None

    def step(self, mode):
        """One tick. Returns a tuple of NEW error keys (usually the shared empty tuple)."""
        if not self.enabled:
            return _NONE
        np.equal(self._homing, 2.0, out=self._bh)
        if mode in EXEMPT_MODES or not self._bh.all():
            self._disarm()
            return _NONE
        fired = None
        q = self._q

        passive = mode in PASSIVE_MODES
        if self.check_range:
            past = self._soft_warn(q)
            if passive:
                # Freedrive / holds: no trip. Armed = inside, so a later crossing in an active mode still
                # fires; a joint outside takes its "deeper" reference at the first active tick.
                np.logical_not(past, out=self._a_rng)
                self._c_rng.fill(0)
                if self._deep_dirty:
                    self._c_deep.fill(0)
                    self._r_ref.fill(np.nan)
                    self._deep_dirty = False
            else:
                hit = self._band(q, self._lim_lo, self._lim_hi, self._in_lo, self._in_hi,
                                 self._c_rng, self._a_rng, self._n_pos)
                if hit is not None:
                    fired = self._fire(fired, hit, self._a_rng, self._k_rng, OUT_OF_RANGE, q,
                                       "position", self.soft_lo, self.soft_hi, "soft limits")
                if not self._a_rng.all():
                    self._deep_dirty = True
                    fired = self._range_deeper(q, fired)
        # Freedrive walls / the wind-up boundary matter only in passive modes: in an active mode OUT_OF_RANGE
        # (and its "deeper" rule) trips long before.
        if self.check_passive_bound and passive:
            hit = self._band(q, self._pb_lo, self._pb_hi, self._pb_in_lo, self._pb_in_hi,
                             self._c_pb, self._a_pb, self._n_pos)
            if hit is not None:
                fired = self._fire_passive_bound(fired, hit, q)
        if self._pb_warned:
            self._clear_wall_warnings(q)
        if self.check_endstop:
            hit = self._band(q, self._end_lo, self._end_hi, self._end_in_lo, self._end_in_hi,
                             self._c_end, self._a_end, self._n_pos)
            if hit is not None:
                fired = self._fire(fired, hit, self._a_end, self._k_end, BEYOND_ENDSTOP, q,
                                   "position", self._end_in_lo, self._end_in_hi, "endstops", tol=self.end_tol)
        if self.check_speed:
            np.abs(self._v, out=self._f1)
            hit = self._band(self._f1, -np.inf, self._v_trip, 0.0, self._v_ceiling,
                             self._c_spd, self._a_spd, self._n_spd)
            if hit is not None and self.speed_mode == "warn":
                self._spd_warned = self._warn(hit, self._a_spd, self._k_spdw, self._pk_spd, self._f1,
                                              OVERSPEED, "filtered speed", self._v_trip, per_s=True)
            elif hit is not None:
                fired = self._fire(fired, hit, self._a_spd, self._k_spd, OVERSPEED, self._f1,
                                   "filtered speed", None, self._v_trip, "trip speed", per_s=True)
            if self._spd_warned:
                self._spd_warned = self._warn_track(self._a_spd, self._k_spdw, self._pk_spd, self._f1,
                                                    "speed", self._v_ceiling, per_s=True)
        if self.check_tracking:
            if mode in TRACKING_MODES:
                np.subtract(self._qc, q, out=self._f1)
                np.abs(self._f1, out=self._f1)
                hit = self._band(self._f1, -np.inf, self._trk, 0.0, self._trk_rearm,
                                 self._c_trk, self._a_trk, self._n_trk)
                if hit is not None and self.tracking_mode == "warn":
                    self._trk_warned = self._warn(hit, self._a_trk, self._k_trkw, self._pk_trk, self._f1,
                                                  TRACKING, "tracking error", self._trk)
                elif hit is not None:
                    fired = self._fire(fired, hit, self._a_trk, self._k_trk, TRACKING, self._f1,
                                       "tracking error", None, self._trk, "limit")
                if self._trk_warned:
                    self._trk_warned = self._warn_track(self._a_trk, self._k_trkw, self._pk_trk, self._f1,
                                                        "tracking error", self._trk)
            else:
                self._c_trk[:] = 0
                if self._trk_warned:                         # left EXEC: the excursion is over
                    self._a_trk[:] = True
                    np.copyto(self._f1, 0.0)
                    self._trk_warned = self._warn_track(self._a_trk, self._k_trkw, self._pk_trk, self._f1,
                                                        "tracking error", self._trk)
        if self.check_hold_osc:
            self._hold_osc_step()
        if self.check_self_collision:
            if passive:
                self._sc_rebase = True                       # the next active period takes its escape baseline
            else:
                fired = self._self_collision_step(fired)
        if self.check_collision:
            fired = self._collision_step(fired, mode)
        if self.check_tcp:
            t = self._tcp
            s = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]) ** 0.5
            if s > self._tcp_trip:
                self._c_tcp += 1
            else:
                self._c_tcp = 0
                if s <= self._tcp_limit:                     # re-arm once back under the LIMIT itself
                    self._a_tcp = True
            if self._a_tcp and self._c_tcp == self._n_tcp:
                self._a_tcp = False
                msg = f"TCP speed {s:.3f} m/s above the {self._tcp_trip:.3f} m/s trip for {self._n_tcp} ticks"
                if self.tcp_mode == "warn":
                    self.warnings.add(TCP_OVERSPEED_WARNING)
                    self._pk_tcp = s
                    self.details[TCP_OVERSPEED_WARNING] = msg
                    logger.warning(msg + " — reaction: warn")
                else:
                    self.details[TCP_OVERSPEED] = msg
                    logger.error(msg)
                    fired = (fired or []) + [TCP_OVERSPEED]
            if self._pk_tcp > 0.0:                           # a live TCP warning: track its peak, clear it
                if s > self._pk_tcp:
                    self._pk_tcp = s
                if self._a_tcp:
                    self.warnings.discard(TCP_OVERSPEED_WARNING)
                    logger.info(f"TCP back under {self._tcp_limit:.3f} m/s; the excursion peaked at "
                                 f"{self._pk_tcp:.3f} m/s")
                    self._pk_tcp = 0.0
        if self.check_zones:
            fired = self._step_zones(fired, mode in PASSIVE_MODES)
        return tuple(fired) if fired else _NONE

    def _hold_osc_step(self):
        """HOLD_OSCILLATION_WARNING, one tick, allocation-free: the motor current high-passed (first order), its
        square averaged (EMA); a joint commanded still whose level stays above its threshold for the sustain warns."""
        x, prev, hp, ms, f = self._i, self._ho_prev, self._ho_hp, self._ho_ms, self._ho_f
        if self._ho_fresh:                                   # first tick after an exempt / unhomed period
            np.copyto(prev, x)
            hp.fill(0.0)
            ms.fill(0.0)
            self._ho_fresh = False
        np.subtract(x, prev, out=f)
        hp += f
        hp *= self._ho_a                                     # hp = a * (hp + x - x_prev)
        np.copyto(prev, x)
        np.multiply(hp, hp, out=f)
        np.subtract(f, ms, out=f)
        f *= self._ho_b
        ms += f                                              # ms = EMA of hp^2, mA^2
        still, over, fire = self._ho_b1, self._ho_b2, self._ho_b3
        np.greater(ms, self._ho_thr2, out=over)
        if not over.any() and not self._ho_any:              # the common tick: every joint quiet, nothing live
            if self._ho_counting:
                self._ho_c.fill(0)
                self._ho_counting = False
            return
        np.abs(self._vc, out=f)
        np.less(f, self._ho_still, out=still)
        np.logical_and(over, still, out=over)
        c = self._ho_c
        c += over
        c *= over                                            # consecutive ticks over, still
        self._ho_counting = True
        np.equal(c, self._ho_n, out=fire)
        if fire.any():
            self._hold_osc_fire(fire)
        if self._ho_any:
            self._hold_osc_track(still)

    def _sc_take_baseline(self):
        """The escape rule's baseline, once per active period (cold): every pair already within the monitor margin
        may stay there or move apart, never get SC_ESCAPE_TOL_M closer; every other pair trips on entering it."""
        if self._sc["hit"][0] >= 0.5:
            np.subtract(self._sc_gaps, SC_ESCAPE_TOL_M, out=self._sc_thr)
        else:
            self._sc_thr.fill(np.inf)
        inside = np.flatnonzero(np.isfinite(self._sc_thr))
        if inside.size:
            names = ", ".join(f"{self._sc_names[k // 7]} and {self._sc_names[k % 7]} {1e3 * self._sc_gaps[k]:.1f} mm"
                              for k in inside)
            logger.warning(f"self-collision: this active period starts with {names} — within the monitor "
                            f"margin already (escape rule): moving apart is allowed; SELF_COLLISION if a pair gets "
                            f"{1e3 * SC_ESCAPE_TOL_M:g} mm closer or another pair comes within the margin")

    def _self_collision_step(self, fired):
        """SELF_COLLISION / its warning / STALE from the worker's fields, one tick. Allocation-free unless something
        starts or ends (then a log line and a key)."""
        sc = self._sc
        hb = sc["heartbeat"][0]
        if hb != self._sc_last_hb:
            self._sc_last_hb = hb
            self._sc_hb_ticks = 0
        else:
            self._sc_hb_ticks += 1
        stale = self._sc_hb_ticks >= self._sc_stale_n
        if stale != self._sc_stale:                              # cold: the worker stopped, or came back
            self._sc_stale = stale
            if stale:
                self.warnings.add(SELF_COLLISION_STALE)
                logger.warning(f"the self-collision worker has not updated shared memory for {SC_STALE_S:g} s — "
                                f"self-collisions are NOT being watched (SELF_COLLISION_STALE)")
            else:
                self.warnings.discard(SELF_COLLISION_STALE)
                logger.info("the self-collision worker is back")
        if stale:
            self._sc_c = 0
            return fired
        if self._sc_rebase:                                       # cold: the first fresh tick of an active period
            self._sc_rebase = False
            self._sc_take_baseline()
        hit = sc["hit"][0] >= 0.5
        if hit:                                                   # which pairs are past their own line (escape rule)
            np.less(self._sc_gaps, self._sc_thr, out=self._sc_viol)
            hit = bool(self._sc_viol.any())
        if hit:
            self._sc_c += 1
            if self._sc_c == self._sc_n and self._sc_armed:        # cold: fire once per excursion
                self._sc_armed = False
                k = int(np.argmax(self._sc_viol))
                a, b = divmod(k, 7)
                pair = (f"{self._sc_names[a]} and {self._sc_names[b]}" if 0 <= a < len(self._sc_names) and 0 <= b < len(self._sc_names)
                        else f"frames {a} and {b}")
                gap_mm = 1e3 * float(self._sc_gaps[k])
                why = ("" if not np.isfinite(self._sc_thr[k]) else
                       f" — closer than the {1e3 * (self._sc_thr[k] + SC_ESCAPE_TOL_M):.1f} mm they started this active period at")
                msg = f"{pair} are {gap_mm:.1f} mm apart on the measured pose{why}"
                if self.sc_mode == "warn":
                    self.warnings.add(SELF_COLLISION_WARNING)
                    self._sc_live = True
                    self.details[SELF_COLLISION_WARNING] = msg
                    logger.warning(f"self-collision (warn): {msg} — no stop (warn mode)")
                else:
                    self.details[SELF_COLLISION] = msg
                    logger.error(f"{SELF_COLLISION}: {msg}")
                    fired = fired if fired is not None else []
                    fired.append(SELF_COLLISION)
        else:
            self._sc_c = 0
            if not self._sc_armed:                               # cold: the excursion is over
                self._sc_armed = True
                if self._sc_live:
                    self._sc_live = False
                    self.warnings.discard(SELF_COLLISION_WARNING)
                    logger.info("self-collision warning cleared: the links are apart again")
        return fired

    def set_collision_mode(self, mode):
        """The runtime switch (RTI's collision_detection command): off | warn | stop, until RTI restarts — for a PC
        program that pushes on purpose. Cold. Live warnings end; the next tick takes a new baseline. Returns the mode
        now in force ("off" when there is nothing to check)."""
        self.collision_mode = check_mode(mode)
        self.check_collision = self._cl_possible and self.collision_mode != "off"
        if self._cl_any:
            self._collision_clear_all("the check was switched")
        self._collision_reset()
        return self.collision_mode if self._cl_possible else "off"

    def _collision_reset(self):
        """No excursion, no counting; the next checked tick takes a fresh baseline. The warning KEYS are the caller's
        (_disarm clears the whole set; the switch logs and drops them first)."""
        self._cl_cc.fill(0)
        self._cl_oc.fill(0)
        self._cl_live_c.fill(False)
        self._cl_live_w.fill(False)
        self._cl_pk.fill(0.0)
        self._cl_armed.fill(True)
        self._cl_any = self._cl_counting = False
        self._cl_fresh = True
        self._cl_since = self._cl_settle_n

    def _collision_step(self, fired, mode):
        """CONTACT_WARNING / COLLISION(_WARNING), one tick (see the module docstring). Allocation-free unless something
        starts or ends: ~8 array operations while moving, ~6 at rest. _cl_any = something to follow: a live warning or
        a joint that fired in stop mode and has not come back under (it re-arms there)."""
        if mode not in COLLISION_MODES:                     # freedrive, hand guiding, ACTIVE_ERROR: nothing to judge
            if self._cl_any:
                self._collision_clear_all("the robot left EXEC / JOG / RTI")
            if not self._cl_fresh:
                self._collision_reset()
            return fired
        obs = self._obs
        if obs is not None and (not obs.running or obs.resets != self._cl_resets):
            self._cl_resets = obs.resets                    # the observer (re)starts: its estimate converges first
            self._cl_hold = self._cl_hold_n
        if self._cl_hold > 0:
            self._cl_hold -= 1
            if self._cl_counting:
                self._cl_cc.fill(0)
                self._cl_oc.fill(0)
                self._cl_counting = False
            return fired
        e, base, d, b = self._te, self._cl_base, self._cl_d, self._cl_b1
        if self._cl_fresh:                                  # the first checked tick of an active period
            np.copyto(base, e)
            self._cl_fresh = False
        np.abs(self._vc, out=d)
        np.greater(d, self._ho_still, out=b)
        if b.any():                                         # commanded to move: the check runs (and settle_n after)
            self._cl_since = 0
        elif self._cl_since < self._cl_settle_n:
            self._cl_since += 1
        np.subtract(e, base, out=d)
        if self._cl_since >= self._cl_settle_n:             # at rest: the baseline follows what the arm rests with
            d *= self._cl_k
            base += d
            if self._cl_any:
                self._collision_clear_all("the arm is at rest")
            if self._cl_counting:
                self._cl_cc.fill(0)
                self._cl_oc.fill(0)
                self._cl_counting = False
            return fired
        np.abs(d, out=d)                                    # d = |estimate - baseline|
        np.greater(d, self._cl_lo, out=b)
        if not b.any() and not self._cl_any:                # the common tick: every joint under both thresholds
            if self._cl_counting:
                self._cl_cc.fill(0)
                self._cl_oc.fill(0)
                self._cl_counting = False
            return fired
        self._cl_counting = True
        b2, b3 = self._cl_b2, self._cl_b3
        np.greater(d, self._cn_thr, out=b2)
        cc = self._cl_cc
        cc += b2
        cc *= b2                                            # consecutive ticks over contact
        np.greater(d, self._co_thr, out=b3)
        oc = self._cl_oc
        oc += b3
        oc *= b3                                            # consecutive ticks over collision
        np.equal(cc, self._cl_n, out=b2)
        np.equal(oc, self._cl_n, out=b3)
        if b2.any() or b3.any():
            fired = self._collision_fire(fired, b2, b3, d)
        if self._cl_any:
            self._collision_track(d)
        return fired

    def _collision_msg(self, i, d, thr, what):
        return (f"{d:.2f} Nm of external torque, over the {thr:.2f} Nm {what} threshold (estimate "
                f"{float(self._te[i]):+.2f} Nm, {float(self._cl_base[i]):+.2f} Nm at rest before the move)")

    def _collision_fire(self, fired, contact, collision, d):
        """Cold path: a joint's contact / collision threshold held for the sustain."""
        for i in np.flatnonzero(contact):
            if self._cl_live_c[i]:
                continue
            self._cl_live_c[i] = True
            self._cl_pk[i] = max(self._cl_pk[i], d[i])
            key = self._k_ct[i]
            self.warnings.add(key)
            self.details[key] = self._collision_msg(i, d[i], self._cn_thr[i], "contact")
            logger.warning(f"J{i + 1} contact: {self.details[key]} — a warning")
        for i in np.flatnonzero(collision):
            if not self._cl_armed[i]:
                continue
            self._cl_armed[i] = False
            self._cl_pk[i] = max(self._cl_pk[i], d[i])
            if self.collision_mode == "warn":
                key = self._k_clw[i]
                self._cl_live_w[i] = True
                self.warnings.add(key)
                self.details[key] = self._collision_msg(i, d[i], self._co_thr[i], "collision")
                logger.warning(f"J{i + 1} collision (warn): {self.details[key]} — no stop (warn mode)")
            else:
                key = self._k_cl[i]
                self.details[key] = self._collision_msg(i, d[i], self._co_thr[i], "collision")
                logger.error(f"J{i + 1} {COLLISION}: {self.details[key]}")
                fired = fired if fired is not None else []
                fired.append(key)
        self._cl_any = bool(self._cl_live_c.any() or self._cl_live_w.any() or not self._cl_armed.all())
        return fired

    def _collision_track(self, d):
        """While a contact / collision warning is live or a joint is disarmed (stop mode): the peak; a warning ends, and
        a joint re-arms, once the joint is under half its contact threshold."""
        live = False
        for i in range(self.n):
            if not (self._cl_live_c[i] or self._cl_live_w[i]):
                if not self._cl_armed[i]:
                    if d[i] < self._cl_clear[i]:
                        self._cl_armed[i] = True
                    else:
                        live = True
                continue
            if d[i] > self._cl_pk[i]:
                self._cl_pk[i] = d[i]
            if d[i] < self._cl_clear[i]:
                self._collision_end(i, "back under half its contact threshold")
            else:
                live = True
        self._cl_any = live

    def _collision_end(self, i, why):
        """Cold path: a joint's contact / collision warnings end; the peak to RTI.log."""
        self.warnings.discard(self._k_ct[i])
        self.warnings.discard(self._k_clw[i])
        c, o = self._cn_thr[i], self._co_thr[i]
        logger.info(f"J{i + 1} external torque over ({why}); it peaked at {self._cl_pk[i]:.2f} Nm "
                     f"(thresholds contact {c:.2f} / collision {o:.2f} Nm)")
        self._cl_live_c[i] = self._cl_live_w[i] = False
        self._cl_armed[i] = True
        self._cl_pk[i] = 0.0

    def _collision_clear_all(self, why):
        for i in range(self.n):
            if self._cl_live_c[i] or self._cl_live_w[i]:
                self._collision_end(i, why)
        self._cl_armed.fill(True)
        self._cl_any = False

    def _hold_osc_fire(self, fire):
        """Cold path: a joint has buzzed for the sustain window — a live warning."""
        for i in np.flatnonzero(fire):
            if self._ho_live[i]:
                continue
            self._ho_live[i] = True
            self._ho_pk[i] = self._ho_ms[i]
            self.warnings.add(self._k_how[i])
            self.details[self._k_how[i]] = (f"{self._ho_ms[i] ** 0.5:.0f} mA of fast current ripple, over "
                                            f"{self._ho_thr[i]:g} mA for {self._ho_sustain_s:.1f} s")
            logger.warning(f"J{i + 1} oscillates while holding: {self._ho_ms[i] ** 0.5:.0f} mA of fast current "
                            f"ripple (over {self._ho_thr[i]:g} mA for {self._ho_sustain_s:.1f} s) — the drive is fighting itself (a "
                            f"buzz/whine): most likely its PID gains are too high; else its encoder (magnet) or the "
                            f"mechanics. No stop (warn).")
        self._ho_any = True

    def _hold_osc_track(self, still):
        """While a HOLD_OSCILLATION_WARNING is live: track its peak; clear it once the level is under half the
        threshold or the joint is commanded to move (the hold is over), logging the peak."""
        live = False
        for i in range(self.n):
            if not self._ho_live[i]:
                continue
            if self._ho_ms[i] > self._ho_pk[i]:
                self._ho_pk[i] = self._ho_ms[i]
            if self._ho_ms[i] < self._ho_clear2[i] or not still[i]:
                self._ho_live[i] = False
                self.warnings.discard(self._k_how[i])
                pk = self._ho_pk[i] ** 0.5
                logger.info(f"J{i + 1} hold oscillation over ({'level back down' if still[i] else 'it moves'});"
                             f" it peaked at {pk:.0f} mA ({100.0 * pk / self._ho_thr[i]:.0f}% of the "
                             f"{self._ho_thr[i]:g} mA threshold)")
            else:
                live = True
        self._ho_any = live

    def _warn(self, hit, armed, wkeys, peak, values, name, what, trip, per_s=False):
        """Cold path, warn mode: a live warning instead of an error, the excursion's peak tracked from here."""
        unit, k = ("deg/s", np.degrees) if per_s else ("deg", np.degrees)
        for i in np.flatnonzero(hit):
            armed[i] = False
            peak[i] = values[i]
            self.warnings.add(wkeys[i])
            self.details[wkeys[i]] = f"{what} {k(values[i]):.2f} {unit} above the trip {k(trip[i]):.2f} {unit}"
            logger.warning(f"J{i + 1} {name} (warn): {self.details[wkeys[i]]} — no stop (warn mode)")
        return True

    def _warn_track(self, armed, wkeys, peak, values, what, ref, per_s=False):
        """While a warn-mode warning is live: track the peak; clear it once re-armed (back under), logging the
        peak against `ref` (the ceiling / the limit). Returns whether any is still live."""
        unit = "deg/s" if per_s else "deg"
        live = False
        for i in range(self.n):
            key = wkeys[i]
            if key not in self.warnings:
                continue
            if values[i] > peak[i]:
                peak[i] = values[i]
            if armed[i]:
                self.warnings.discard(key)
                logger.info(f"J{i + 1} {what} back to normal; the excursion peaked at "
                             f"{np.degrees(peak[i]):.2f} {unit} ({100.0 * peak[i] / ref[i]:.0f}% of "
                             f"{np.degrees(ref[i]):.2f} {unit})")
            else:
                live = True
        return live

    def _fire_passive_bound(self, fired, hit, q):
        """Cold path: a joint pushed through its freedrive wall (its own warn / stop) or, with no wall, past the hard
        limit of a joint without an endstop (stop)."""
        for i in np.flatnonzero(hit):
            self._a_pb[i] = False
            lo, hi = np.degrees(self._pb_in_lo[i]), np.degrees(self._pb_in_hi[i])
            if self._pb_wall[i]:
                detail = (f"{np.degrees(q[i]):.2f} deg, wall at [{lo:.2f}, {hi:.2f}] deg, breakthrough "
                          f"{np.degrees(self._pb_in_lo[i] - self._pb_lo[i]):.1f} deg")
                msg = f"J{i + 1} pushed through its freedrive wall: {detail}"
                if self._pb_warn[i]:
                    self.warnings.add(self._k_wallw[i])
                    self._pb_warned = True
                    self.details[self._k_wallw[i]] = detail
                    logger.warning(f"{msg} — reaction: warn")
                    continue
                self.details[self._k_pb[i]] = detail
                logger.error(f"{JOINT_WALL}: {msg} — reaction: stop")
            else:
                self.details[self._k_pb[i]] = (f"{np.degrees(q[i]):.2f} deg outside the hard limits [{lo:.2f}, {hi:.2f}] "
                                               f"deg (+/- {np.degrees(self.tol):.2f} tolerance)")
                logger.error(f"J{i + 1} {HARD_LIMIT}: {self.details[self._k_pb[i]]}; no endstop: the cable wind-up boundary")
            fired = fired if fired is not None else []
            fired.append(self._k_pb[i])
        return fired

    def _clear_wall_warnings(self, q):
        """Drop a JOINT_WALL_WARNING once its joint is back inside its wall (only runs while one is live)."""
        live = False
        for i in range(self.n):
            key = self._k_wallw[i]
            if key in self.warnings:
                if self._pb_in_lo[i] <= q[i] <= self._pb_in_hi[i]:
                    self.warnings.discard(key)
                    logger.info(f"J{i + 1} back inside its freedrive wall")
                else:
                    live = True
        self._pb_warned = live

    def _soft_warn(self, q):
        """Live SOFT_LIMIT_WARNING per joint (every checked mode). Returns a bool scratch array: which joints
        are past their soft limit. A quiet tick (nothing past, nothing warned) costs three array ops."""
        past, b = self._b4, self._b5
        np.less(q, self._in_lo, out=past)
        np.greater(q, self._in_hi, out=b)
        np.logical_or(past, b, out=past)
        if not self._soft_warned and not past.any():
            return past
        # Something is past or still warned: a warning clears only 0.1 deg back inside, so it cannot flap.
        new = self._b3
        np.greater(q, self._in_lo_h, out=new)
        np.less(q, self._in_hi_h, out=b)
        np.logical_and(new, b, out=b)                        # b = properly back inside
        np.logical_not(b, out=b)
        np.logical_and(self._w_soft, b, out=b)               # still warned, not properly back
        np.logical_or(past, b, out=new)
        np.not_equal(new, self._w_soft, out=b)
        if b.any():                                          # cold: a warning starts or ends
            for i in np.flatnonzero(b):
                if new[i]:
                    self.warnings.add(self._k_soft[i])
                    self.details[self._k_soft[i]] = (f"{np.degrees(q[i]):.2f} deg, limits [{np.degrees(self.soft_lo[i]):.2f}, "
                                                     f"{np.degrees(self.soft_hi[i]):.2f}] deg")
                    logger.warning(f"J{i + 1} is past its soft limit ({np.degrees(q[i]):.2f} deg, "
                                    f"limits [{np.degrees(self.soft_lo[i]):.2f}, {np.degrees(self.soft_hi[i]):.2f}]) "
                                    f"— JOG, EXEC and RTI will only move it back in")
                else:
                    self.warnings.discard(self._k_soft[i])
                    logger.info(f"J{i + 1} back inside its soft limits")
            self._w_soft[:] = new
            self._soft_warned = bool(new.any())
        return past

    def _range_deeper(self, q, fired):
        """Active mode, some joint outside WITHOUT a trip (dragged there in a passive mode, or rested there):
        fire OUT_OF_RANGE if it goes more than the tolerance further out than where it was."""
        d = self._f2
        np.subtract(self._in_lo, q, out=d)
        np.subtract(q, self._in_hi, out=self._f3)
        np.maximum(d, self._f3, out=d)                       # how far past the soft limit (> 0 = past)
        ref, cand, bad = self._r_ref, self._b4, self._b5
        np.isnan(ref, out=bad)
        np.logical_not(bad, out=bad)
        np.logical_and(bad, self._a_rng, out=bad)            # re-armed joints drop their reference
        if bad.any():
            np.copyto(ref, np.nan, where=bad)
        np.greater(d, 0.0, out=cand)
        np.logical_not(self._a_rng, out=bad)
        np.logical_and(cand, bad, out=cand)                  # outside and not armed
        if not cand.any():
            self._c_deep.fill(0)
            return fired
        np.isnan(ref, out=bad)
        np.logical_and(bad, cand, out=bad)
        np.copyto(ref, d, where=bad)                         # first active tick out here: the reference
        np.add(ref, self.tol, out=self._f1)
        np.greater(d, self._f1, out=bad)
        np.logical_and(bad, cand, out=bad)                   # deeper than the reference + tolerance
        self._c_deep += bad
        self._c_deep *= bad
        np.greater_equal(self._c_deep, self._n_pos, out=bad)
        if bad.any():                                        # cold
            fired = fired if fired is not None else []
            for i in np.flatnonzero(bad):
                fired.append(self._k_rng[i])
                self.details[self._k_rng[i]] = (f"driven {np.degrees(d[i] - ref[i]):.2f} deg further past its soft limit, "
                                                f"now {np.degrees(d[i]):.2f} deg past")
                logger.error(f"J{i + 1} {OUT_OF_RANGE}: {self.details[self._k_rng[i]]} while outside it")
                ref[i] = d[i]
                self._c_deep[i] = 0
        return fired

    def _step_zones(self, fired, passive):
        """Zones, one tick. Allocation-free unless something fires (scalar math, state in lists).
        Passive (freedrive) modes use each zone's freedrive reaction and, for haptic zones, the breakthrough
        depth; active modes its reaction and the tolerance."""
        spheres, tol, n_trip = self._spheres, self._z_tol, self._n_pos
        zones, count, armed, ref = self._zones, self._z_count, self._z_armed, self._z_ref
        cx, cy, cz, nk = self._cx, self._cy, self._cz, self._z_n
        warn_now = self._z_pas_warn if passive else self._z_warn
        trip = self._z_pas_trip if passive else self._z_act_trip
        sphere_centres(self._pose, spheres, cx, cy, cz)          # once per tick, for every zone
        for i in range(len(zones)):
            pen, k = penetration_at(zones[i], cx, cy, cz, spheres, nk[i], 0.0)
            if pen <= 0.0:                                   # clear of the zone: re-arm
                count[i] = 0
                if not armed[i]:
                    armed[i] = True
                    ref[i] = math.nan
                    if self._z_wkeys[i] in self.warnings:
                        self.warnings.discard(self._z_wkeys[i])
                        logger.info(f"left zone '{zones[i]['name']}'")
                continue
            if warn_now[i] and not armed[i]:
                ref[i] = math.nan                            # a warned excursion carries no reference onward
            if pen <= trip[i]:
                count[i] = 0
                continue
            count[i] += 1
            if count[i] < n_trip:
                continue
            if armed[i]:
                fired = self._fire_zone(fired, i, pen, k, False, warn_now[i])
            elif not warn_now[i]:
                if ref[i] != ref[i]:                         # NaN: inside since an exempt or warned period
                    ref[i] = pen
                elif pen > ref[i] + tol:
                    fired = self._fire_zone(fired, i, pen, k, True, False)
        return fired

    def _fire_zone(self, fired, i, pen, k, deeper, warn):
        """Cold path: log and report one zone trip."""
        zone, m = self._zones[i], self._pose
        self._z_armed[i] = False
        self._z_ref[i] = pen
        what = "the TCP" if k == 0 else f"tool sphere {k}"
        if zone["kind"] == "allowed":
            verb = "went further out of" if deeper else "left"
        else:
            verb = "went deeper into" if deeper else "entered"
        msg = (f"{what} {verb} {zone['kind']} {zone['shape']} zone '{zone['name']}': "
               f"{1000.0 * pen:.1f} mm past its surface, "
               f"TCP at ({m[3]:.3f}, {m[7]:.3f}, {m[11]:.3f}) m")
        if warn:
            self.warnings.add(self._z_wkeys[i])
            self.details[self._z_wkeys[i]] = msg
            logger.warning(msg + " — reaction: warn")
            return fired
        self.details[self._z_ekeys[i]] = msg
        logger.error(msg)
        fired = fired if fired is not None else []
        fired.append(self._z_ekeys[i])
        return fired

    def _fire(self, fired, hit, armed, keys, name, values, what, lo, hi, bound, per_s=False, tol=None):
        """Cold path: log what tripped, disarm those joints, return the growing key list."""
        fired = fired if fired is not None else []
        for i in np.flatnonzero(hit):
            armed[i] = False
            fired.append(keys[i])
            v = np.degrees(values[i])
            unit = "deg/s" if per_s else "deg"
            if lo is None:
                detail = f"{what} {v:.2f} {unit} above the {bound} {np.degrees(hi[i]):.2f} {unit}"
            else:
                detail = (f"{what} {v:.2f} deg outside the {bound} [{np.degrees(lo[i]):.2f}, {np.degrees(hi[i]):.2f}] deg "
                          f"(+/- {np.degrees(self.tol if tol is None else tol):.2f} tolerance)")
            self.details[keys[i]] = detail
            logger.error(f"J{i + 1} {name}: {detail}")
        return fired
