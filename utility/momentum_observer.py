"""The external joint torque while the arm moves: a generalized-momentum observer (2026-10-16).

Until now the external torque was the measured joint torque minus gravity — right only at rest: while the arm moves,
the motors also accelerate its mass (inertia), carry the Coriolis / centrifugal forces and overcome friction, and all
of that read as "external" (CLAUDE.md §17.1). With the full dynamics model (the Pinocchio model the inertia / Coriolis
feedforward uses) those can be taken out. The standard way, used by research and commercial arms for collision
detection, is the momentum observer: it compares the arm's generalized momentum p = M(q)·q̇ with what the motor
torque, gravity, Coriolis and friction should have made of it — the difference is the external torque. It never
needs the joint acceleration (from motor encoders a noisy second difference).

    the arm:      M(q)·q̈ + C(q, q̇)·q̇ + g(q) + τ_f(q̇) = τ_m + τ_ext
    so            ṗ = τ_m + τ_ext - τ_f + C(q, q̇)ᵀ·q̇ - g(q)           (Ṁ = C + Cᵀ)
    the observer: r = K·( p - p(0) - ∫ (τ_m - τ_f - β + r) dt ),   β = g(q) - C(q, q̇)ᵀ·q̇

which gives ṙ = K·(τ_ext - r): r follows the external torque through a first-order low-pass of time constant 1/K
(K = 50/s -> 20 ms). Discrete: stable for K·dt < 2 (here 0.2). r is the torque the OUTSIDE exerts on the joints;
RTI publishes -r as tcp['torque_external'] — measured minus model, the sign that field always had (the joints'
reaction: a lever pressing a table with 1 Nm reads +1, CLAUDE.md §17.2).

Inputs per tick (RTI, after the send — data/update_shared_data.update_joint_rest): the joint position, the FILTERED
joint speed, the RAW measured joint torque (motor current x Kt x gear ratio; the observer's integral is the low-pass)
and the time since the last tick's estimate (`elapsed`: a perf_counter reading at the same point of every tick, right
after its CAN read — RTI's loop_period is written later in the tick, one tick behind). The model terms come from utility/pinocchio_attach.make_pinocchio_momentum_terms
(robot.pinocchio_momentum; rebuilt by set_payload — a new function resets the observer). Friction: THE friction
model (utility/friction_model.py, 2026-10-17; the robot XML's <friction_coulomb_nm> + <friction_load_per_nm> x |the
gravity model's torque| rounded through zero speed by tanh, + <friction_viscous_nm_s> x speed — measured by
api_tests/77; the gravity torque comes in per tick, `step(..., gravity=)`). What the model gets wrong — friction that
varies, a mass a little off, the stiction at rest — stays in r: that is what api_tests/76_external_torque.py measures
before any threshold acts on it.

Reset (r = 0, then it converges within ~3/K): at the first valid tick, after an input that is not finite (a lost
drive), after a gap in time (> MAX_GAP_S, e.g. a stalled tick) and when the model function changes (a payload).
step() allocates no array.
"""
from __future__ import annotations

import math

import numpy as np

from utility.friction_model import FrictionModel

OBSERVER_GAIN = 50.0          # 1/s: r follows the external torque with a 20 ms time constant
FRICTION_V_SMOOTH = 0.02      # rad/s (~1.1 deg/s): the Coulomb friction's sign is tanh(v / this)
MAX_GAP_S = 0.05              # a period longer than this (12 ticks) restarts the observer


class MomentumObserver:
    def __init__(self, joint_num, gain=OBSERVER_GAIN, friction_coulomb=None, friction_viscous=None,
                 v_smooth=FRICTION_V_SMOOTH, friction_load=None):
        n = int(joint_num)
        self.n = n
        self.gain = float(gain)
        # THE friction model (utility/friction_model.py, 2026-10-17): Coulomb (+ load x |gravity|) x tanh(v / v_smooth)
        # + viscous x v — the same model the feedforward and freedrive use. fc / fv are its arrays (the same objects).
        self.model = FrictionModel([0.0 if x is None else x for x in (list(friction_coulomb or []) + [0.0] * n)[:n]],
                                   friction_viscous, friction_load, v_smooth=v_smooth)
        self.fc, self.fv = self.model.coulomb, self.model.viscous
        self.v_smooth = float(v_smooth)
        self.r = np.zeros(n)                  # the estimate: the external joint torque, Nm
        self.friction = np.zeros(n)           # the friction model's torque this tick, Nm (published for the curious)
        self._p0 = np.zeros(n)
        self._integral = np.zeros(n)
        self._tmp = np.zeros(n)
        self._terms = None
        self._started = False
        self._t_last = None
        self.resets = 0                       # how often it restarted (a lost drive, a gap, a payload)

    @property
    def running(self) -> bool:
        """True once it estimates (False before its first valid tick, and after a reset until the next one) — the
        safety monitor's collision check waits for it (utility/safety_monitor.py)."""
        return self._started

    def elapsed(self, t) -> float:
        """The time since the previous call (s; 0.0 the first time) — dt for step()."""
        dt = t - self._t_last if self._t_last is not None else 0.0
        self._t_last = t
        return dt

    def reset(self) -> None:
        if self._started:
            self.resets += 1
        self._started = False
        self.r.fill(0.0)

    def _friction(self, v, g=None) -> None:
        self.model.torque(v, g, out=self.friction)

    def step(self, q, v, tau_m, dt, terms_fn, gravity=None) -> bool:
        """One tick; q, v, tau_m: arrays of the joint count (RTI's shared-memory views). Returns False (and resets)
        when it cannot estimate this tick: no model, an input not finite, a gap in time; r then holds 0 until it
        starts again."""
        if terms_fn is None or not 0.0 < dt <= MAX_GAP_S:
            self.reset()
            return False
        if not math.isfinite(float(q.sum()) + float(v.sum()) + float(tau_m.sum())):
            self.reset()
            return False
        if terms_fn is not self._terms:       # a new model (set_payload): start again on it
            self.reset()
            self._terms = terms_fn
        p, beta = terms_fn(q, v)
        if not self._started:
            self._p0[:] = p
            self._integral.fill(0.0)
            self.r.fill(0.0)
            self._started = True
            return True
        # integral += (τ_m - τ_f - β + r)·dt
        self._friction(v, gravity)
        t = self._tmp
        np.subtract(tau_m, self.friction, out=t)
        np.subtract(t, beta, out=t)
        np.add(t, self.r, out=t)
        np.multiply(t, dt, out=t)
        np.add(self._integral, t, out=self._integral)
        # r = K·(p - p0 - integral)
        np.subtract(p, self._p0, out=self.r)
        np.subtract(self.r, self._integral, out=self.r)
        np.multiply(self.r, self.gain, out=self.r)
        return True
