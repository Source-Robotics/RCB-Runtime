"""Friction compensation in freedrive — a small push in the direction each joint is already moving.

WHAT IT FIXES (measured 2026-10-02, api_tests/54): every joint's motor and gearbox resist motion with friction —
roughly constant once sliding, a bit more to break loose. J5 0.06-0.09 Nm, J6 0.11-0.15, J4 0.13-0.19. In freedrive
(IDLE, gravity comp ON) only your hand overcomes it, so the arm feels sticky, slow motions stick-slip (J5 in
api_tests/56: stuck, then jumped), and a wall can push a released joint back only while its push beats that friction.
Giving back PART of the friction makes the arm lighter and smoother, and lets the walls return it further.

THE LAW, per joint j, every freedrive tick:

    tau_j = c_j * s(v_j) * fade(|v_j|)

    v_j      the joint's FILTERED speed (rad/s; the raw one carries stepper ripple)
    s(v)     clamp(v / v_band, -1, 1): a smooth sign — ZERO AT REST, full beyond v_band. So it never breaks a joint
             loose: a released arm is still held by its static friction, and it cannot chatter around zero speed.
    fade     1 below fade_start (20 deg/s), linearly 0 at fade_end (40): friction matters most for SLOW motion —
             fine positioning, stick-slip; fast hand motion is dominated by inertia — and it bounds how fast a
             joint could glide on its own (below).
    c_j      Nm = fraction x max(0, share x (sliding_j + load_j x |g_j|) - gravity_error_j), from the robot XML
             (<friction_coulomb_nm>: the SLIDING friction at a steady speed, api_tests/77 — the old
             <friction_sliding_nm> when it is missing; <friction_load_per_nm>: how much it grows with the joint's
             gravity torque g_j, J2 only on PAR6; <gravity_error_nm>: the largest gravity-model error measured while
             sliding) and the <friction_comp> policy (fraction 0.5). A joint with no sliding measurement falls back to
             sliding_ratio (0.7) x its breakaway friction <friction_static_nm>. The same model as the friction
             feedforward and the external-torque estimate (utility/friction_model.py, 2026-10-17); here only its
             Coulomb + load part — the viscous part grows with speed, where the fade already takes it all back.
             share: <model_share> (0.7) — the model is the TYPICAL friction (api_tests/77), and the friction measured
             went down to 0.69x it (J4 at 0 deg); freedrive relies only on what is there at the lowest. On PAR6
             2026-10-17: J1 0.040, J2 0 (unloaded) .. 0.10 (7.4 Nm of gravity), J4 0.047 Nm; J3 / J5 / J6 0 — their
             gravity-model error is as large as their friction.

WHY THAT c_j: while the compensation is less than the joint's real friction, the net friction (real - compensation)
still opposes motion — the arm still loses energy, it just loses less; it cannot run away. The one way it can glide
on its own: a joint released WHILE MOVING in a pose where the gravity model is off by more than the friction that
is left (friction - c_j) keeps moving downhill. So c_j only gives back a fraction of the margin ABOVE the gravity
error; J1-J3 are not measured, so 0. History (2026-10-02): first c came from the BREAKAWAY ramp x 0.7 (with c from
the breakaway itself J5 glided at ~28 deg/s in simulation) and was tiny (J4 0, J5 0.004, J6 0.017 Nm — "no
noticeable change"). Measured SLIDING, J4/J5 need 20-46 % MORE than their "breakaway": the encoders are on the
motors, so the ramp's 0.08 deg onset includes give in the gearing, and the push that unloads the gears (gravity
decides which) moves early — which the ramp also read as J4's ±0.13 Nm "gravity error" (sliding: ±0.03). Now:
J4 0.068, J5 0.024, J6 0.027 Nm. Any glide is capped near fade_start: above it the joint's full friction comes back.

THE OTHER MODES: JOG / EXEC / RTI follow a commanded motion through the drives' own loop; there the friction
feedforward on the COMMANDED speed (utility/mode_dispatch._add_friction_ff, 2026-10-17, its own switch) gives back the
whole model — no feedback, so it cannot run away. Same model (utility/friction_model.py), different rule: here a
fraction of the margin above the gravity error, because here the MEASURED speed drives it.
HOMING never: it finds the endstops by the current a stall draws.

HOT PATH: step() is a scalar loop into a preallocated buffer; no allocation.
"""
from __future__ import annotations

from collections.abc import Sequence

import math

import numpy as np


def _num(x):
    return float(x) if x is not None and math.isfinite(float(x)) else 0.0


def comp_amounts(friction_static: Sequence[float], gravity_error: Sequence[float], fraction: float, sliding_ratio: float = 0.7,
                 friction_sliding: Sequence[float] | None = None, model_share: float = 1.0) -> list[float]:
    """Per-joint compensation (Nm): fraction x max(0, sliding friction - gravity error). The sliding friction is
    model_share x the MEASURED one (friction_sliding: the model's Coulomb part, api_tests/77) where it is > 0, else
    sliding_ratio x the breakaway friction. Unmeasured (both 0) -> 0."""
    if friction_sliding is None:
        friction_sliding = [0.0] * len(friction_static)
    out = []
    for f, e, fsl in zip(friction_static, gravity_error, friction_sliding):
        f, e, fsl = _num(f), _num(e), _num(fsl)
        slide = model_share * fsl if fsl > 0.0 else sliding_ratio * f
        out.append(fraction * max(0.0, slide - abs(e)) if slide > 0.0 else 0.0)
    return out


class FrictionComp:
    """Built once before the RT loop (RTI.py); IDLE's freedrive calls step() while the operator switch is on."""

    def __init__(self, joint_num, friction_static, gravity_error, policy, friction_sliding=None, friction_load=None):
        self.n = int(joint_num)
        self._fs = [float(x) for x in friction_static][:self.n]
        self._ge = [float(x) for x in gravity_error][:self.n]
        self._fsl = [float(x) for x in (friction_sliding if friction_sliding is not None else [0.0] * self.n)][:self.n]
        # Nm of friction per Nm of gravity torque (<friction_load_per_nm>, 2026-10-17); 0 = no load term
        self._load = [_num(x) for x in (list(friction_load or []) + [0.0] * self.n)[:self.n]]
        self.tau = np.zeros(self.n)
        self.c = [0.0] * self.n
        self._slide = [0.0] * self.n          # the sliding friction used (measured, or the breakaway fallback), Nm
        self.set_policy(policy)

    def set_policy(self, policy):
        self.fraction = float(policy.get("fraction", 0.5))
        self.sliding_ratio = float(policy.get("sliding_ratio", 0.7))
        self.model_share = float(policy.get("model_share", 0.7))
        self.v_band = math.radians(float(policy.get("band_dps", 2.0)))
        self.f0 = math.radians(float(policy.get("fade_start_dps", 20.0)))
        self.f1 = math.radians(float(policy.get("fade_end_dps", 40.0)))
        self.c[:] = comp_amounts(self._fs, self._ge, self.fraction, self.sliding_ratio, self._fsl, self.model_share)
        for j in range(self.n):
            fsl = _num(self._fsl[j]) if j < len(self._fsl) else 0.0
            fs = _num(self._fs[j]) if j < len(self._fs) else 0.0
            self._slide[j] = self.model_share * fsl if fsl > 0.0 else self.sliding_ratio * fs

    @property
    def active(self):
        return any(c > 0.0 for c in self.c) or any(
            ld > 0.0 and sl > 0.0 for ld, sl in zip(self._load, self._slide))

    def amount(self, j, g=None):
        """Joint j's compensation at full speed (Nm): c_j, with the load term at gravity torque g (Nm) when given."""
        ld = self._load[j]
        if g is None or ld <= 0.0 or self._slide[j] <= 0.0:
            return self.c[j]
        gj = float(g)
        if gj != gj:                          # NaN: no load term this tick
            return self.c[j]
        e = _num(self._ge[j]) if j < len(self._ge) else 0.0
        c = self._slide[j] + self.model_share * ld * (gj if gj >= 0.0 else -gj) - (e if e >= 0.0 else -e)
        return self.fraction * c if c > 0.0 else 0.0

    def step(self, v, g=None):
        """Compensation torque (Nm) for the filtered joint speeds `v` (rad/s), in self.tau; `g` the gravity model's
        joint torques (Nm) for the load term (None: Coulomb only)."""
        tau, band, f0, f1 = self.tau, self.v_band, self.f0, self.f1
        for j in range(self.n):
            c = self.c[j] if g is None else self.amount(j, g[j])
            if c <= 0.0:
                tau[j] = 0.0
                continue
            vj = float(v[j])
            a = vj if vj >= 0.0 else -vj
            s = vj / band
            if s > 1.0:
                s = 1.0
            elif s < -1.0:
                s = -1.0
            if a <= f0:
                fade = 1.0
            elif a >= f1:
                fade = 0.0
            else:
                fade = (f1 - a) / (f1 - f0)
            tau[j] = c * s * fade
        return tau
