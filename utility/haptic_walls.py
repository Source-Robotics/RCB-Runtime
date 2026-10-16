"""Haptic walls — feel the safety zones in IDLE gravity-comp freedrive (a "virtual fixture" in haptics terms).

Decided 2026-10-01 (TODO/safety_zones_plan.md): in freedrive a zone with haptic="1" (the default) pushes back
like a spring-damper instead of being a line you cross without noticing. On top of the gravity torque IDLE
already sends, each robot sphere that has entered a zone's margin from OUTSIDE gets

    F = -clamp(k * depth + d * v_in, 0, max_force) * n

where depth is how far it is past the zone's margin boundary, n the direction its depth grows and v_in its
speed along n (negative = coming back out). The joint torque is J_p^T F for that sphere's point:
tau = J_v^T F + J_w^T (r x F), r from the TCP to the sphere.

WHAT IT IS NOT:
  * It cannot make a zone impassable. The force is capped (motors, and what is sane to push a person with);
    push harder and you go through. Past <breakthrough> the safety monitor applies the zone's `freedrive`
    reaction (warn by default, or stop).
  * It never pulls: the clamp at 0 means the damping can slow you coming OUT but never drags you in.
  * It never pushes on its own. A sphere that is ALREADY inside when freedrive starts (homed inside a zone, a
    zone reloaded around it) feels nothing until it has been outside once; every new freedrive session and
    every reload re-arms from scratch. Let go while pressing an armed wall and it springs back to the surface
    — that is what a wall does.
  * Only the modelled spheres (TCP + tool spheres) feel it; the arm links do not (yet).
  * Near a singularity some push directions cannot be resisted (J^T loses rank).

STABILITY — measured in simulation 2026-10-01 (test_haptic_walls.py repeats it). A sampled, delayed spring adds
energy; damping removes it, but damping computed from a LAGGED velocity adds energy itself, so stable
stiffness/damping is a window, not a floor. Findings (mass 0.5-5 kg on the wall, no physical damping, the
state 1-2 ticks old as in RTI where the handler sees the previous tick's pose):
  * damping only INTO the wall: k >= 2000 N/m can never be made passive (the delayed spring gains energy on the
    way out, where nothing damps it) -> two-sided damping, clamped so it never pulls;
  * damping from TCP_speed_FILTERED (butter2, tau 10 ms, ~14 ms lag): k = 1000 is unstable at ANY damping ->
    the wall keeps its own light EMA (velocity_tau, 4 ms) of the RAW twist instead;
  * THE ARM IS LIGHTER THAN THAT: along J5's direction of travel the model's effective TCP mass is 0.13 kg. The
    first shipped wall (1000 N/m, 20 N.s/m) has no stable damping at all down there — on the arm it vibrated
    while pressed. Over 0.1-5 kg with the state 1 tick old: 200 N/m is passive for d in [2, 18], 500 N/m for
    [5, 14], 1000 N/m for none (4 ms filter). config/xml_parser.HAPTIC_STABLE holds that table and refuses
    anything outside it; the default (200 N/m, 6 N.s/m) is also passive at 2 ticks of delay.
A brick-hard wall is not possible at 250 Hz with a 0.1 kg wrist — expect rubber. Tune on the arm with
api_tests/51_haptic_wall.py --tune / --record.

MASS-NORMALISED ZONE WALLS (Z13, 2026-10-02; <zone_mass_scaled>, default on): stiffness and damping are written for
the 0.1 kg reference (200 N/m, 6 N.s/m = 7.1 Hz, damping ratio 0.67) and applied x m_eff / 0.1 kg, where m_eff is
the mass the contact point has ALONG THE PUSH with every joint free: m_eff = 1 / (a^T M^-1 a), a = J_v^T n +
J_w^T (r x n) (n the push direction, r the point's offset from the TCP). Same frequency and damping ratio in every
direction and pose — the dynamics the table proves — so the wall is up to ~10x stiffer where the arm is heavy and
SOFTER where it is lighter than 0.1 kg (no floor: below 0.1 kg the fixed wall was unproven; the TCP's lightest
direction measured 0.12-0.13 kg over 2000 poses). At most ZONE_SCALE_MAX (5 kg): heavier directions (to ~300 kg near a
singularity) keep the 5 kg wall, which the table proves passive up to 250 kg. M^-1 is refreshed
(Pinocchio, ~12 us) only on a contact tick after the arm moved more than POSE_RESCALE_RAD.

JOINT WALLS (2026-10-01, user requests): every joint gets the same wall in joint space, in freedrive:
tau_j = -clamp(s_j(q) * (k_j * depth + d_j * v_in), 0, max_torque_j) at a per-joint position — the SOFT limit by
default (you feel it before the endstop), the HARD limit for a joint without an endstop (J6: the cable wind-up
boundary). Per-joint settings live in the robot XML (<freedrive_wall>, tuned live in the dashboard's Walls tab).
  * k_j, d_j are set for the joint's LIGHTEST pose: the smallest inertia that joint's wall can push, with the
    OTHER JOINTS FREE (1/[M^-1]_jj — what a wall pushes in freedrive; the first version used the diagonal M_jj,
    i.e. the other joints held, which is up to 4.7x heavier for J3 and was not provably stable). "auto" = the
    zone wall's 200 N/m / 6 N.s/m scaled by that inertia over HAPTIC_REFERENCE_MASS — exactly the dynamics the
    stability table covers, and any setting is checked against the same table in those units.
  * s_j(q) = I_j(q) / I_j,min >= 1 (POSE-DEPENDENT, user decision 2026-10-01): stiffness AND damping both follow
    the inertia at the current pose, so (k/I, d/I) — and with them the wall's closed-loop dynamics — stay those of
    the proven-stable lightest pose, while the wall gets stiffer where the arm is heavier (J1 up to ~12x).
    Recomputed (Pinocchio, ~12 us) only while a joint wall is touched and the arm moved > 0.2 deg.
  * scaling (per joint, 2026-10-02, user request — to compare): "adaptive" (auto) = the pose scale above; "fixed" =
    the lightest pose's stiffness and damping everywhere (s = 1). Fixed is the proven lightest-pose setting on a
    heavier inertia — a lower frequency, inside the table — so it feels softer where the joint carries more.
  * Each joint has its own speed filter for the damping (filter_ms, 2026-10-02; auto = the policy's velocity_tau):
    a stiffer wall needs less filtering, a joint with strong stepper ripple may want more. Its stable frequency and
    damping-ratio range follow from that filter's column of the stability table.
  * max_torque_j is NOT scaled: it is the cap in Nm. Auto: the spring at the breakthrough at the heaviest pose
    (k x scale_max x breakthrough), at most AUTO_JOINT_MAX_TORQUE_NM; anything set is capped by the policy ceiling
    <haptic_walls><joint_max_torque>.

HOT PATH: step() runs every IDLE gravity-comp tick. With no sphere in contact it does scalar math only
(the velocity EMA, the sphere centres, one penetration per zone and sphere) and returns None; in contact it adds
six signed-distance evaluations per contact and one 6xN matrix-vector product into a preallocated buffer.
No allocation either way.
"""
from __future__ import annotations

from collections.abc import Sequence
from collections.abc import Callable
from typing import Any

import logging
logger = logging.getLogger("rti.safety")  # docs/logging.md "Logger names"

import numpy as np

from config.xml_parser import HAPTIC_REFERENCE_MASS
from utility.zones import _sd, push_direction, robot_spheres, sphere_centres


AUTO_JOINT_MAX_TORQUE_NM = 5.0    # auto max torque cap; a setting may go up to the policy ceiling (20 Nm)
POSE_RESCALE_RAD = np.radians(0.2)  # recompute the pose-dependent scale once the arm moved this far
ZONE_SCALE_MAX = 50.0             # m_eff / 0.1 kg at most: the stability table's heaviest mass (5 kg). Near a
                                  # singularity m_eff reaches ~300 kg — 600 000 N/m would turn encoder noise into force


def make_effective_inertia(robot: Any) -> Callable[[np.ndarray, np.ndarray], None] | None:
    """fn(q, out): out[j] = the inertia joint j's wall pushes at pose q with the other joints FREE,
    1/[M^-1(q)]_jj (kg m^2). Pinocchio dynamics model (utility/pinocchio_attach.build_pinocchio_dynamics) with its
    OWN Data, so use one fn per thread. ~12 us; allocates one 6x6 array per call. None if the model has no
    dynamics."""
    try:
        import pinocchio as pin
        from utility.pinocchio_attach import build_pinocchio_dynamics
        model, data = build_pinocchio_dynamics(robot)
    except Exception as exc:
        logger.warning(f"joint walls: no dynamics model ({exc}) — joint walls off")
        return None
    nv = model.nv

    def fn(q, out):
        mi = pin.computeMinverse(model, data, q)
        for j in range(nv):
            out[j] = 1.0 / mi[j, j]
    return fn


def make_inverse_mass(robot: Any) -> Callable[[np.ndarray, np.ndarray], None] | None:
    """fn(q, out): out[:, :] = M(q)^-1, the full symmetric inverse mass matrix (Pinocchio dynamics model, its OWN
    Data — one fn per thread). Symmetrised from the upper triangle (older Pinocchio fills only that). None if the
    model has no dynamics."""
    try:
        import pinocchio as pin
        from utility.pinocchio_attach import build_pinocchio_dynamics
        model, data = build_pinocchio_dynamics(robot)
    except Exception as exc:
        logger.warning(f"zone walls: no dynamics model ({exc}) — not mass-normalised")
        return None
    nv = model.nv

    def fn(q, out):
        mi = pin.computeMinverse(model, data, q)
        for r in range(nv):
            for c in range(r, nv):
                out[r, c] = out[c, r] = mi[r, c]
    return fn


def effective_inertia_range(fn: Callable[[np.ndarray, np.ndarray], None], lo: np.ndarray, hi: np.ndarray,
                            samples: int = 3000, seed: int = 7, refine: int = 3) -> tuple[np.ndarray, np.ndarray] | None:
    """(min, max) per joint of fn's inertia over the box [lo, hi] (cold path, ~0.1 s): `samples` seeded random
    poses, then the `refine` lowest per joint polished by a bounded Nelder-Mead — the MINIMUM is the wall's worst
    case and random sampling alone misses it (20 samples overestimated J1's by 30%). None if fn is None."""
    if fn is None:
        return None
    try:
        from scipy.optimize import minimize
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        n = lo.size
        rng = np.random.default_rng(seed)
        qs = lo + rng.random((samples, n)) * (hi - lo)
        vals = np.zeros((samples, n))
        out = np.zeros(n)
        for i in range(samples):
            fn(qs[i], out)
            vals[i] = out
        lo_v, hi_v = vals.min(axis=0), vals.max(axis=0)
        bounds = list(zip(lo, hi))
        for j in range(n):
            def f(q, j=j):
                fn(np.clip(q, lo, hi), out)
                return out[j]
            for i in np.argsort(vals[:, j])[:refine]:
                r = minimize(f, qs[i], method="Nelder-Mead", bounds=bounds, options={"xatol": 1e-4, "fatol": 1e-9})
                lo_v[j] = min(lo_v[j], float(r.fun))
        return lo_v, hi_v
    except Exception as exc:
        logger.warning(f"joint walls: inertia range failed ({exc}) — joint walls off")
        return None


def joint_drive_torque(cfg: dict) -> np.ndarray:
    """Each joint's torque at its drive's current limit, Ilim x Kt x |gear| x efficiency (Nm). DISPLAY ONLY — the
    Walls tab's max-torque slider range: a wall asking for more is clipped by the drive itself anyway."""
    n = int(cfg["joint_num"])
    f = lambda k: np.asarray(cfg[k], dtype=float)[:n]
    return f("motor_Ilim") / 1000.0 * f("motor_kt") * np.abs(f("gear_ratio")) * f("gear_efficiency")


def _row_k_max(tau_s: float) -> float:
    """Largest stiffness (N/m at the reference mass) the stability table holds for this filter, or 0."""
    from config.xml_parser import HAPTIC_STABLE
    taus = [t for t in sorted(HAPTIC_STABLE) if t >= tau_s * 1000.0 - 1e-9]
    return float(max(HAPTIC_STABLE[taus[0]])) if taus else 0.0


def joint_damping_window(k: float, inertia: float, tau_s: float) -> tuple[float, float] | None:
    """Stable damping (Nm.s/rad) for a joint wall of stiffness k (Nm/rad) on a joint of this inertia (kg m^2): the
    stability table in its reference-mass units (k / s, d / s with s = inertia / HAPTIC_REFERENCE_MASS). None = no
    stable damping at that stiffness (or k <= 0: a wall needs a spring)."""
    from config.xml_parser import haptic_damping_window
    s = float(inertia) / HAPTIC_REFERENCE_MASS
    if not (k > 0.0 and s > 0.0):
        return None
    win = haptic_damping_window(k / s, tau_s)
    return None if win is None else (win[0] * s, win[1] * s)


def wall_dynamics_limits(policy: dict, tau_s: float | None = None) -> tuple[float, float, float]:
    """The wall settings every joint shares, from the policy and the stability table: (f_auto Hz, zeta_auto,
    f_max Hz). A wall of natural frequency f and damping ratio zeta has, at the reference mass M0, the stiffness
    (2 pi f)^2 M0 and damping 2 zeta (2 pi f) M0 the table is written in — the same dynamics at every joint and
    every pose, because each joint's Nm/rad follow its inertia (JOINT WALLS). f_max is for the speed filter
    `tau_s` (s; default the policy's velocity_tau)."""
    M0 = HAPTIC_REFERENCE_MASS
    k, d = float(policy.get("stiffness", 200.0)), float(policy.get("damping", 6.0))
    f_auto = np.sqrt(k / M0) / (2 * np.pi)
    zeta_auto = d / (2.0 * np.sqrt(k * M0))
    tau = float(policy.get("velocity_tau", 0.004)) if tau_s is None else float(tau_s)
    f_max = np.sqrt(_row_k_max(tau) / M0) / (2 * np.pi)
    return float(f_auto), float(zeta_auto), float(f_max)


def damping_ratio_window(f_hz: float, tau_s: float) -> tuple[float, float] | None:
    """Damping ratios a wall of natural frequency f_hz may have (the stability table at the reference mass), or
    None when no damping is stable at that frequency."""
    from config.xml_parser import haptic_damping_window
    w = 2 * np.pi * float(f_hz)
    if w <= 0.0:
        return None
    M0 = HAPTIC_REFERENCE_MASS
    win = haptic_damping_window(w * w * M0, tau_s)
    return None if win is None else (win[0] / (2 * w * M0), win[1] / (2 * w * M0))


def resolve_joint_walls(specs: Sequence[dict], policy: dict, soft_lo: np.ndarray, soft_hi: np.ndarray, hard_lo: np.ndarray,
                        hard_hi: np.ndarray, has_endstop: Sequence[bool] | np.ndarray, inertia: np.ndarray,
                        inertia_max: np.ndarray | None = None) -> list[dict]:
    """Per-joint wall parameters from the robot XML specs (config/xml_parser.wall_spec; user units) and the policy.

    Returns one dict per joint, SI inside ("lo"/"hi" rad, "k" Nm/rad, "d" Nm.s/rad, "tmax" Nm, "bt" rad), its
    "f" (Hz) and "zeta" — the wall's natural frequency and damping ratio, set per joint (2026-10-02), k and d follow
    from them at the lightest pose — its speed filter "tau" (s), the stable ranges for that filter, "bt_max" (rad,
    inf = none) and "note" when a requested value was refused (frequency, ratio and filter then all go auto). A joint
    without inertia (no dynamics in the model) or without finite limits gets no wall. `inertia` = each joint's
    LIGHTEST effective inertia (effective_inertia_range min) — k and d hold there; `inertia_max` (optional) only
    reports how much stiffer the pose-dependent wall gets at the heaviest pose ("scale_max").

    A joint WITH an endstop: its hard limit IS the endstop, so a wall there never engages (at="hard" -> the soft
    limit) and it cannot be pushed further than the endstop (breakthrough capped at the soft-to-hard margin).
    """
    from config.xml_parser import wall_spec
    deg = np.pi / 180.0
    tau_auto = float(policy.get("velocity_tau", 0.004))
    walls_on = bool(policy.get("enabled", 1)) and bool(policy.get("joint_walls", 1))
    ceiling = float(policy.get("joint_max_torque", 20.0))
    f_auto, zeta_auto, f_max_auto = wall_dynamics_limits(policy)
    out = []
    n = len(soft_lo)
    for j in range(n):
        spec = wall_spec(specs[j] if specs is not None and j < len(specs) else None)
        endstop = bool(has_endstop[j])
        notes = []
        at = spec["at"] or ("soft" if endstop else "hard")
        if endstop and at == "hard":
            notes.append("at=hard refused: the hard limit is this joint's endstop, a wall there never engages — "
                         "using the soft limit")
            at = "soft"
        lo, hi = (soft_lo[j], soft_hi[j]) if at == "soft" else (hard_lo[j], hard_hi[j])
        margin = min(soft_lo[j] - hard_lo[j], hard_hi[j] - soft_hi[j])
        bt_max = float(margin) if endstop and np.isfinite(margin) and margin > 0 else float("inf")
        I = None if inertia is None else float(inertia[j])
        ok = (walls_on and I is not None and np.isfinite(I) and I > 0 and all(np.isfinite([lo, hi])))
        adaptive = spec["scaling"] != "fixed"
        w = {"joint": j, "enabled": bool(ok), "at": at, "lo": float(lo), "hi": float(hi), "freedrive": spec["freedrive"],
             "spec": spec, "inertia": I, "bt_max": bt_max, "note": "", "adaptive": adaptive,
             "scale_max": (float(inertia_max[j]) / I if adaptive and inertia_max is not None and I else 1.0)}
        if not ok:
            w.update(k=0.0, d=0.0, tmax=0.0, bt=0.0, k_max=0.0, d_window=(0.0, 0.0), f=0.0, zeta=0.0,
                     zeta_window=(0.0, 0.0), tau=tau_auto, f_max=f_max_auto)
            out.append(w)
            continue
        f = f_auto if spec["frequency_hz"] is None else spec["frequency_hz"]
        zeta = zeta_auto if spec["damping_ratio"] is None else spec["damping_ratio"]
        tau = tau_auto if spec["filter_ms"] is None else spec["filter_ms"] / 1000.0
        f_max = wall_dynamics_limits(policy, tau)[2]
        zwin = damping_ratio_window(f, tau)
        if zwin is None or not (zwin[0] <= zeta <= zwin[1]):
            notes.append(f"refused {f:.3g} Hz / damping ratio {zeta:.3g} with a {1000 * tau:g} ms filter (outside the "
                         f"stable table: up to {f_max:.1f} Hz there) — using auto")
            f, zeta, tau, f_max = f_auto, zeta_auto, tau_auto, f_max_auto
            zwin = damping_ratio_window(f, tau)
        om = 2 * np.pi * f
        k, d = om * om * I, 2.0 * zeta * om * I          # at the LIGHTEST pose; x I(q)/I there (_refresh_scale)
        win = (2.0 * zwin[0] * om * I, 2.0 * zwin[1] * om * I)
        bt_policy = float(policy.get("joint_breakthrough", 15.0)) * deg
        if at == "soft":               # pushed through at the hard limit (the endstop), at most the policy's angle
            bt_auto = min(max(float(margin), 2.0 * deg), bt_policy) if np.isfinite(margin) else bt_policy
        else:
            bt_auto = bt_policy
        bt = bt_auto if spec["breakthrough_deg"] is None else spec["breakthrough_deg"] * deg
        if bt > bt_max + 1e-9:
            notes.append(f"breakthrough {bt / deg:.1f} deg is past the endstop ({bt_max / deg:.1f} deg past the wall) "
                         f"— using {bt_max / deg:.1f}")
            bt = bt_max
        # The spring at the breakthrough at the HEAVIEST pose (the wall is stiffer there): no early flat top before
        # the breakthrough at any pose — a flat top is where a push suddenly gives way (J3, 2026-10-01).
        tmax_auto = min(k * w["scale_max"] * bt, AUTO_JOINT_MAX_TORQUE_NM, ceiling)
        tmax = tmax_auto if spec["max_torque_nm"] is None else min(spec["max_torque_nm"], ceiling)
        w.update(k=float(k), d=float(d), tmax=float(tmax), bt=float(bt), k_max=float((2 * np.pi * f_max) ** 2 * I),
                 d_window=(float(win[0]), float(win[1])), f=float(f), zeta=float(zeta),
                 zeta_window=(float(zwin[0]), float(zwin[1])), tau=float(tau), f_max=float(f_max),
                 note="; ".join(notes))
        out.append(w)
    return out


def walls_public(walls: Sequence[dict]) -> list[dict]:
    """The resolved walls in user units (Hz, damping ratio, deg, Nm) — for get_joint_walls and the dashboard.
    stiffness_nm_per_deg / damping_nm_s_per_deg are what the frequency and ratio give at the joint's LIGHTEST pose
    (information; x the pose scale elsewhere)."""
    deg = np.pi / 180.0
    out = []
    for w in walls:
        out.append({"joint": w["joint"] + 1, "enabled": w["enabled"], "at": w["at"], "freedrive": w["freedrive"],
                    "lo_deg": w["lo"] / deg, "hi_deg": w["hi"] / deg,
                    "frequency_hz": w["f"], "damping_ratio": w["zeta"], "frequency_max_hz": w["f_max"],
                    "damping_ratio_window": list(w["zeta_window"]), "filter_ms": 1000.0 * w["tau"],
                    "stiffness_nm_per_deg": w["k"] * deg, "damping_nm_s_per_deg": w["d"] * deg,
                    "max_torque_nm": w["tmax"], "breakthrough_deg": w["bt"] / deg,
                    "breakthrough_max_deg": None if not np.isfinite(w["bt_max"]) else w["bt_max"] / deg,
                    "stiffness_max_nm_per_deg": w["k_max"] * deg,
                    "damping_window_nm_s_per_deg": [w["d_window"][0] * deg, w["d_window"][1] * deg],
                    "inertia_kg_m2": w["inertia"], "stiffness_scale_max": w["scale_max"],
                    "scaling": "adaptive" if w.get("adaptive", True) else "fixed",
                    "spec": dict(w["spec"]), "note": w["note"]})
    return out


class HapticWalls:
    """Built once before the RT loop (RTI.py); IDLE's handler calls step() each gravity-comp tick."""

    def __init__(self, joint_num, dt, *, tcp_pose, tcp_twist_raw, jacobian, policy, zones=(), tool_spheres=(),
                 q=None, qd_raw=None, joint_walls=None, inertia_fn=None, minv_fn=None):
        """tcp_pose: the measured flat row-major 4x4 TCP pose (shared-memory view). tcp_twist_raw: the RAW
        measured TCP twist [vx vy vz wx wy wz] (view) — NOT the filtered one, see STABILITY. jacobian: a (6,
        joint_num) buffer RTI refreshes every tick with the TCP Jacobian [J_v; J_w] at the measured pose."""
        self.n = int(joint_num)
        self.dt = float(dt)
        self._pose, self._twist = tcp_pose, tcp_twist_raw
        self._v = [0.0] * 6                          # the wall's own light EMA of the raw twist
        self._JT = jacobian.T                       # a fixed view: the data under it changes in place
        self._w = np.zeros(6)
        self._tau = np.zeros(self.n)
        self._out = np.zeros(self.n)
        self._last_tick = -2
        self.contacts = 0                           # spheres + joints pushing this tick (telemetry / tests)
        # Joint walls (see JOINT WALLS): resolved per joint by resolve_joint_walls(); set_joint_walls() installs them.
        n = self.n
        self._q, self._qd = q, qd_raw
        self._jarmed = [False] * n
        self._jv = [0.0] * n
        self._jalpha = [1.0] * n                    # each joint's own speed-filter weight (its filter_ms)
        self._tau_j = np.zeros(n)
        self._wj, self._jlo, self._jhi = [], [0.0] * n, [0.0] * n
        self._jk, self._jd, self._jtmax = [0.0] * n, [0.0] * n, [0.0] * n
        # Pose-dependent scale (JOINT WALLS): s_j = I_j(q) / I_j,min, refreshed by _refresh_scale while touched.
        self._inertia_fn = inertia_fn
        self._jIref = [0.0] * n
        self._jscale = [1.0] * n
        self._jadapt = [True] * n                   # per joint: adaptive (pose-scaled) or fixed
        self._Ieff = np.zeros(n)
        self._q_scaled = np.full(n, np.nan)          # pose of the last scale computation (NaN = never)
        # Mass-normalised zone walls (MASS-NORMALISED ZONE WALLS): M^-1 at the last refresh, buffers for a and M^-1 a.
        self._minv_fn = minv_fn
        self._minv = np.zeros((n, n))
        self._minv_q = np.full(n, np.nan)
        self._a, self._ma, self._w6 = np.zeros(n), np.zeros(n), np.zeros(6)
        self.zone_scale = 1.0                        # m_eff / 0.1 kg of the last zone contact (telemetry / tests)
        self.joint_walls = False
        self.set_policy(policy)
        self.set_zones(zones, tool_spheres)
        self.set_joint_walls(joint_walls)

    # ------------------------------------------------------------------ cold path
    def set_policy(self, policy):
        self.enabled = bool(policy.get("enabled", 1))
        self.k = float(policy.get("stiffness", 1000.0))
        self.d = float(policy.get("damping", 20.0))
        self.f_max = float(policy.get("max_force", 15.0))
        tau = float(policy.get("velocity_tau", 0.004))
        self.alpha = 1.0 if tau <= 0.0 else 1.0 - float(np.exp(-self.dt / tau))
        self.mass_scaled = bool(policy.get("zone_mass_scaled", 1))
        self._last_tick = -2

    def set_dynamics(self, inertia_fn, minv_fn):
        """Swap the dynamics functions (set_payload rebuilt them from the edited tool link, utility/payload.py). Cold
        path. The cached poses are forgotten so the next contact tick recomputes the scale / M^-1 with the new
        model; the per-joint reference inertias (_jIref, the no-payload lightest poses) stay — see payload.py."""
        self._inertia_fn = inertia_fn
        self._minv_fn = minv_fn
        self._q_scaled[:] = np.nan
        self._minv_q[:] = np.nan

    def set_zones(self, zones, tool_spheres):
        """Install the haptic zones of the active cell (startup and reload_zones). Re-arms everything: a reload
        must never make the robot push on its own."""
        self._zones = tuple(z for z in zones if z["enabled"] and z["haptic"])
        self._spheres = robot_spheres(tool_spheres)
        ns = len(self._spheres)
        self._nk = [1 if z["checks"] == "tcp" else ns for z in self._zones]
        self._armed = [False] * (len(self._zones) * ns)
        self._ns = ns
        self._cx, self._cy, self._cz = [0.0] * ns, [0.0] * ns, [0.0] * ns
        self._last_tick = -2

    def set_joint_walls(self, walls):
        """Install resolved per-joint walls (resolve_joint_walls) — startup, live tuning, reload. Re-arms every joint
        wall: a change must never make the robot push on its own."""
        n = self.n
        self._wj = [w["joint"] for w in (walls or ()) if w["enabled"] and w["joint"] < n and (self._q is not None)
                    and (self._qd is not None)]
        for w in walls or ():
            j = w["joint"]
            if j < n:
                self._jlo[j], self._jhi[j] = w["lo"], w["hi"]
                self._jk[j], self._jd[j], self._jtmax[j] = w["k"], w["d"], w["tmax"]
                self._jIref[j] = float(w["inertia"] or 0.0)
                self._jadapt[j] = bool(w.get("adaptive", True))
                tau = w.get("tau")                   # resolve_joint_walls always gives it; else the zone walls' filter
                self._jalpha[j] = (self.alpha if tau is None else
                                   1.0 if tau <= 0.0 else 1.0 - float(np.exp(-self.dt / float(tau))))
        for j in range(n):
            self._jarmed[j] = False
            self._tau_j[j] = 0.0
            self._jscale[j] = 1.0
        self._q_scaled.fill(np.nan)
        self.joint_walls = bool(self._wj)
        self._last_tick = -2

    @property
    def active(self):
        return self.enabled and ((bool(self._zones) and self.f_max > 0.0) or self.joint_walls)

    # ------------------------------------------------------------------ per tick
    def step(self, tick, gravity):
        """gravity + wall torque (Nm) in a preallocated buffer, or None when no wall pushes this tick.

        `tick` detects a new freedrive session: any gap since the last call re-arms every wall."""
        if not self.active:
            return None
        armed, t, v = self._armed, self._twist, self._v
        jarmed, jv, qd = self._jarmed, self._jv, self._qd
        if tick != self._last_tick + 1:                  # a new freedrive session: nothing armed yet
            for j in range(len(armed)):
                armed[j] = False
            for j in range(6):
                v[j] = float(t[j])                       # start the velocity filter where the arm is
            for j in self._wj:
                jarmed[j] = False
                jv[j] = float(qd[j])
        self._last_tick = tick
        a = self.alpha
        for j in range(6):
            v[j] += a * (float(t[j]) - v[j])
        ja = self._jalpha
        for j in self._wj:
            jv[j] += ja[j] * (float(qd[j]) - jv[j])
        contacts = self._step_joints() if self.joint_walls else 0
        if not self._zones:
            self.contacts = contacts
            if contacts == 0:
                return None
            np.add(gravity, self._tau_j, out=self._out)
            return self._out

        spheres, cx, cy, cz = self._spheres, self._cx, self._cy, self._cz
        sphere_centres(self._pose, spheres, cx, cy, cz)
        m = self._pose
        px, py, pz = float(m[3]), float(m[7]), float(m[11])
        vx0, vy0, vz0, wx, wy, wz = v[0], v[1], v[2], v[3], v[4], v[5]
        k, d, f_max, ns = self.k, self.d, self.f_max, self._ns
        fx = fy = fz = mx = my = mz = 0.0
        zone_contacts = 0
        scaled = self.mass_scaled and self._minv_fn is not None and self._q is not None
        fresh = False
        for i in range(len(self._zones)):
            zone = self._zones[i]
            c_allowed = zone["_allowed"]
            margin = zone["margin"]
            for s in range(self._nk[i]):
                x, y, z, r = cx[s], cy[s], cz[s], spheres[s][3]
                sd = _sd(zone["_c"], x, y, z)
                depth = sd + r + margin if c_allowed else r + margin - sd
                j = i * ns + s
                if depth <= 0.0:
                    armed[j] = True                      # seen outside: this wall may now push
                    continue
                if not armed[j]:
                    continue                             # inside since before: never push on its own
                n = push_direction(zone, x, y, z)
                if n is None:
                    continue
                nx, ny, nz = n
                rx, ry, rz = x - px, y - py, z - pz
                vx = vx0 + wy * rz - wz * ry             # v_point = v + w x r
                vy = vy0 + wz * rx - wx * rz
                vz = vz0 + wx * ry - wy * rx
                v_in = vx * nx + vy * ny + vz * nz
                mag = k * depth + d * v_in               # two-sided damping (see STABILITY)...
                if scaled:                               # x m_eff / 0.1 kg (MASS-NORMALISED ZONE WALLS)
                    if not fresh:
                        self._refresh_minv()
                        fresh = True
                    w6 = self._w6
                    w6[0], w6[1], w6[2] = nx, ny, nz
                    w6[3], w6[4], w6[5] = ry * nz - rz * ny, rz * nx - rx * nz, rx * ny - ry * nx    # r x n
                    np.matmul(self._JT, w6, out=self._a)                 # a = J_v^T n + J_w^T (r x n)
                    np.matmul(self._minv, self._a, out=self._ma)
                    inv_m = float(self._a @ self._ma)                    # n^T Lambda^-1 n = 1 / m_eff
                    if inv_m > 0.0:
                        sc = 1.0 / (inv_m * HAPTIC_REFERENCE_MASS)
                        self.zone_scale = sc if sc < ZONE_SCALE_MAX else ZONE_SCALE_MAX
                        mag *= self.zone_scale
                if mag <= 0.0:
                    continue                             # ...clamped: the wall never pulls
                if mag > f_max:
                    mag = f_max
                Fx, Fy, Fz = -mag * nx, -mag * ny, -mag * nz
                fx += Fx
                fy += Fy
                fz += Fz
                mx += ry * Fz - rz * Fy                  # r x F about the TCP
                my += rz * Fx - rx * Fz
                mz += rx * Fy - ry * Fx
                zone_contacts += 1
        self.contacts = contacts + zone_contacts
        if self.contacts == 0:
            return None
        if zone_contacts:
            w = self._w
            w[0], w[1], w[2], w[3], w[4], w[5] = fx, fy, fz, mx, my, mz
            np.matmul(self._JT, w, out=self._tau)        # J^T [F; M]
            np.add(gravity, self._tau, out=self._out)
        else:
            self._out[:] = gravity
        if contacts:
            np.add(self._out, self._tau_j, out=self._out)
        return self._out

    def _refresh_minv(self):
        """M^-1 at the measured pose, if the arm moved more than POSE_RESCALE_RAD since the last refresh (or never).
        Only called on a tick some zone wall is touched."""
        q, qs = self._q, self._minv_q
        for j in range(self.n):
            dq = float(q[j]) - qs[j]
            if not (-POSE_RESCALE_RAD <= dq <= POSE_RESCALE_RAD):   # NaN (never computed) counts as moved
                self._minv_fn(q, self._minv)
                qs[:] = q
                return

    def _refresh_scale(self, q):
        """s_j(q) = I_j(q) / I_j,min for every joint, if the arm moved more than POSE_RESCALE_RAD since the last
        computation (or never computed). Only called on a tick some joint wall is touched."""
        qs = self._q_scaled
        moved = False
        for j in range(self.n):
            d = float(q[j]) - qs[j]
            if not (-POSE_RESCALE_RAD <= d <= POSE_RESCALE_RAD):   # NaN (never computed) counts as moved
                moved = True
                break
        if not moved:
            return
        self._inertia_fn(q, self._Ieff)
        qs[:] = q
        for j in range(self.n):
            ref = self._jIref[j]
            s = self._Ieff[j] / ref if ref > 0.0 and self._jadapt[j] else 1.0      # fixed: always 1
            self._jscale[j] = s if s > 1.0 else 1.0         # never softer than the proven lightest-pose setting

    def _step_joints(self):
        """Joint walls (see JOINT WALLS), into self._tau_j. Returns how many joints push. Scalar; the scale refresh
        (only while touched and moving) allocates one 6x6 array."""
        q, jv, jarmed, tau_j = self._q, self._jv, self._jarmed, self._tau_j
        contacts = 0
        scaled = self._inertia_fn is None                  # no dynamics model: the scale stays 1
        for j in self._wj:
            tau_j[j] = 0.0
            qj = float(q[j])
            if qj > self._jhi[j]:
                side, depth = 1.0, qj - self._jhi[j]
            elif qj < self._jlo[j]:
                side, depth = -1.0, self._jlo[j] - qj
            else:
                jarmed[j] = True                         # inside its range: this wall may now push
                continue
            if not jarmed[j]:
                continue                                 # outside since before: never push on its own
            if not scaled:
                self._refresh_scale(q)
                scaled = True
            mag = self._jscale[j] * (self._jk[j] * depth + self._jd[j] * side * jv[j])
            if mag <= 0.0:
                continue
            if mag > self._jtmax[j]:
                mag = self._jtmax[j]
            tau_j[j] = -side * mag
            contacts += 1
        return contacts
