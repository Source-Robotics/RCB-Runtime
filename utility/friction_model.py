"""THE friction model of the arm's joints (2026-10-17, TODO/friction_compensation_plan.md F2) — one model for every
user: the external torque estimate (utility/momentum_observer.py), the friction feedforward on the commanded speed
(utility/mode_dispatch._add_friction_ff: EXEC, JOG, RTI / SOFT_RTI position and velocity commands) and freedrive
friction compensation (utility/friction_comp.py). Never homing: it finds the endstops by the current a stall draws.

    F_j(v, g) = (coulomb_j + load_j * |g_j|) * tanh(v / v_smooth) + viscous_j * v

    coulomb   Nm — the robot XML's <friction_coulomb_nm>
    load      Nm of friction per Nm of the gravity model's torque on the joint — <friction_load_per_nm> (J2: a gearbox
              carrying more load rubs more; 0 for joints where the data shows no load effect)
    viscous   Nm per rad/s — <friction_viscous_nm_s>
    g         the gravity model's torque on the joint at the measured pose (joint['joint_gravity_torque'], Nm)
    v_smooth  0.02 rad/s (~1.1 deg/s): the sign rounded through zero speed (no chatter at rest)

Measured by api_tests/77_friction_id.py (`--apply` writes the three tags; a robot XML without them falls back to the
old <friction_sliding_nm> as the Coulomb part). Validated 2026-10-17: 76's moves recorded in the same session, replayed
through the observer — at 77's values every joint sat at (or within ~3 % RMS of) the best the moves allow; J2's false
torque while moving -38 % against the XML before (no J2 friction).

HOT PATH: torque() writes into its own buffer (or out=), allocates nothing; the callers pass fixed views.
"""
from __future__ import annotations

from collections.abc import Sequence

import numpy as np

V_SMOOTH = 0.02           # rad/s


class FrictionModel:
    """Per-joint friction (see the module docstring). ff_fraction: the share the feedforward gives back (system.xml
    <dynamics_feedforward><friction_fraction>, 1.0 as shipped)."""

    def __init__(self, coulomb: Sequence[float], viscous: Sequence[float] | None = None,
                 load: Sequence[float] | None = None, v_smooth: float = V_SMOOTH, ff_fraction: float = 1.0):
        n = len(coulomb)
        self.n = n

        def arr(x):
            a = np.zeros(n)
            for j, val in enumerate(list(x if x is not None else [])[:n]):
                val = float(val) if val is not None else 0.0
                a[j] = val if np.isfinite(val) and val > 0.0 else 0.0
            return a
        self.coulomb, self.viscous, self.load = arr(coulomb), arr(viscous), arr(load)
        self.has_load = bool(self.load.any())
        self.v_smooth = float(v_smooth)
        self.ff_fraction = float(ff_fraction)
        self.out = np.zeros(n)
        self._eff = np.zeros(n)
        self._tmp = np.zeros(n)

    @classmethod
    def from_config(cls, cfg: dict, n: int, ff_fraction: float = 1.0) -> "FrictionModel":
        """From load_full_robot_config's dict (the parser's fallbacks already applied)."""
        return cls(list(cfg.get("friction_coulomb_nm") or [])[:n], list(cfg.get("friction_viscous_nm_s") or [])[:n],
                   list(cfg.get("friction_load_per_nm") or [])[:n], ff_fraction=ff_fraction)

    @property
    def active(self) -> bool:
        return bool(self.coulomb.any() or self.viscous.any() or self.load.any())

    def torque(self, v, g=None, out=None, fraction: float = 1.0):
        """F(v, g) per joint into `out` (default: the model's own buffer, overwritten by the next call). v, g: arrays of
        the joint count (rad/s, Nm); g None = no load term."""
        f = self.out if out is None else out
        np.divide(v, self.v_smooth, out=f)
        np.tanh(f, out=f)
        if g is not None and self.has_load:
            np.abs(g, out=self._eff)
            np.multiply(self._eff, self.load, out=self._eff)
            np.add(self._eff, self.coulomb, out=self._eff)
            np.multiply(f, self._eff, out=f)
        else:
            np.multiply(f, self.coulomb, out=f)
        np.multiply(v, self.viscous, out=self._tmp)
        np.add(f, self._tmp, out=f)
        if fraction != 1.0:
            np.multiply(f, fraction, out=f)
        return f

    def describe(self) -> str:
        return ", ".join(f"J{j + 1} {self.coulomb[j]:.3f}" + (f"+{self.load[j]:.3f}|g|" if self.load[j] else "")
                         + (f"+{self.viscous[j]:.3f}v" if self.viscous[j] else "") for j in range(self.n))
