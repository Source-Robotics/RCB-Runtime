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
    c_j      Nm = fraction x max(0, sliding_j - gravity_error_j), from the robot XML (<friction_sliding_nm>: the
             SLIDING friction measured at a steady speed — the lowest over the angles tested; <gravity_error_nm>: the
             largest gravity-model error measured while sliding; both from api_tests/54) and the <friction_comp>
             policy (fraction 0.5). A joint with no sliding measurement falls back to sliding_ratio (0.7) x its
             breakaway friction <friction_static_nm>.

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

NOT IN THE OTHER MODES (yet): JOG / EXEC / RTI hold a position through the drives' own loop; there a feed-forward
from the COMMANDED speed (not the measured one) would cut tracking error — planned, TODO/friction_compensation_plan.md.
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
                 friction_sliding: Sequence[float] | None = None) -> list[float]:
    """Per-joint compensation (Nm): fraction x max(0, sliding friction - gravity error). The sliding friction is the
    MEASURED one (friction_sliding, api_tests/54 at a steady speed) where it is > 0, else sliding_ratio x the
    breakaway friction. Unmeasured (both 0) -> 0."""
    if friction_sliding is None:
        friction_sliding = [0.0] * len(friction_static)
    out = []
    for f, e, fsl in zip(friction_static, gravity_error, friction_sliding):
        f, e, fsl = _num(f), _num(e), _num(fsl)
        slide = fsl if fsl > 0.0 else sliding_ratio * f
        out.append(fraction * max(0.0, slide - abs(e)) if slide > 0.0 else 0.0)
    return out


class FrictionComp:
    """Built once before the RT loop (RTI.py); IDLE's freedrive calls step() while the operator switch is on."""

    def __init__(self, joint_num, friction_static, gravity_error, policy, friction_sliding=None):
        self.n = int(joint_num)
        self._fs = [float(x) for x in friction_static][:self.n]
        self._ge = [float(x) for x in gravity_error][:self.n]
        self._fsl = [float(x) for x in (friction_sliding if friction_sliding is not None else [0.0] * self.n)][:self.n]
        self.tau = np.zeros(self.n)
        self.c = [0.0] * self.n
        self.set_policy(policy)

    def set_policy(self, policy):
        self.fraction = float(policy.get("fraction", 0.5))
        self.sliding_ratio = float(policy.get("sliding_ratio", 0.7))
        self.v_band = math.radians(float(policy.get("band_dps", 2.0)))
        self.f0 = math.radians(float(policy.get("fade_start_dps", 20.0)))
        self.f1 = math.radians(float(policy.get("fade_end_dps", 40.0)))
        self.c[:] = comp_amounts(self._fs, self._ge, self.fraction, self.sliding_ratio, self._fsl)

    @property
    def active(self):
        return any(c > 0.0 for c in self.c)

    def step(self, v):
        """Compensation torque (Nm) for the filtered joint speeds `v` (rad/s), in self.tau."""
        tau, band, f0, f1 = self.tau, self.v_band, self.f0, self.f1
        for j in range(self.n):
            c = self.c[j]
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
