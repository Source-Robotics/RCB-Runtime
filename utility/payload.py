"""Runtime payload (2026-10-04, TODO/rti_review_2026-10-03.md §7.3; the set-payload call industrial arms commonly offer).

A program that picks a part tells the robot what it holds: a mass [kg] and its centre of mass [m] in the TOOL
link's frame (the gripper's <kinematics> frame = link 6 of the DH chain, the frame AFTER the gripper's DH
transform; +z points out of the flange toward the jaws on PAR6's gripper XMLs). The gripper's own body — what
the gripper XML declares — stays what it is; the part is ADDED to it (parallel-axis combination), and the
dynamics models are rebuilt from the edited link: the gravity feedforward in every mode, the freedrive walls'
effective inertia and the zone walls' inverse mass. set_payload(0) brings the gripper alone back.

NOT rtb's Dynamics.payload(): that REPLACES link 6's mass and COM with the point mass, and link 6 here IS the
gripper — the gripper would vanish from the model the moment a part was declared.

Rebuilding the three Pinocchio dynamics models costs ~1.1 ms warm (measured on the Pi; the first build at
start-up pays the import), so the RTI command handler does it in place — a cold path, no file I/O, one late tick
at most. What is NOT rebuilt: the walls' no-payload inertia RANGE (3000-pose search, seconds) — the wall
settings keep their no-payload lightest/heaviest poses, so with a payload a joint wall is up to m_part/m_link
stiffer at its lightest pose (same natural frequency on the heavier link — correct physics) and its max torque
clamp binds a little earlier. Documented, accepted (user, 2026-10-04).

Not persisted: a payload is a program's claim (as is common); the XML stays the no-load default and every RTI
start begins at 0.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

MAX_PAYLOAD_OFFSET_M = 0.5        # |COM| beyond this is a typo (half a metre from the flange)


def combined_body(m0: float, r0: Any, i0: Any, mass: float, com: Any, inertia: Any = None) -> tuple:
    """The gripper body (m0, r0, I0 about ITS COM, link frame) plus a part (mass at com; `inertia` = the part's
    own 3x3 tensor about ITS COM, None = point mass) -> (m, r, I) of the combination: total mass, combined COM,
    inertia about the combined COM (parallel axis for both bodies). Pure; the same convention rtb's Link and
    pin.Inertia use (I about the COM, expressed in the link frame)."""
    r0 = np.asarray(r0, dtype=np.float64).reshape(3)
    i0 = np.asarray(i0, dtype=np.float64).reshape(3, 3)
    com = np.asarray(com, dtype=np.float64).reshape(3)
    m0, mass = float(m0), float(mass)
    m = m0 + mass
    if mass == 0.0 or m <= 0.0:          # no part: the base body, bit-identical (no (m0*r0)/m0 rounding)
        return m0, r0.copy(), i0.copy()
    r = (m0 * r0 + mass * com) / m
    eye = np.eye(3)

    def shift(mb, d):                       # a body of mass mb whose COM is displaced by d from the new COM
        return mb * (float(d @ d) * eye - np.outer(d, d))

    ip = np.zeros((3, 3)) if inertia is None else np.asarray(inertia, dtype=np.float64).reshape(3, 3)
    i = i0 + shift(m0, r0 - r) + ip + shift(mass, com - r)
    return m, r, i


def check_payload(mass: Any, com: Any, max_mass: float, max_offset: float = MAX_PAYLOAD_OFFSET_M) -> tuple[bool, str]:
    """(ok, why) — the bounds every entry point applies (executor and RTI): finite, 0 <= mass <= max_mass,
    three finite COM numbers within max_offset of the tool frame's origin."""
    try:
        mass = float(mass)
        c = [float(v) for v in com]
    except (TypeError, ValueError):
        return False, "payload needs a mass [kg] and a 3-vector COM [m]"
    if len(c) != 3 or not all(math.isfinite(v) for v in c) or not math.isfinite(mass):
        return False, "payload mass and COM must be finite numbers (COM: x y z)"
    if mass < 0.0:
        return False, f"payload mass must be >= 0 kg, got {mass:g}"
    if mass > float(max_mass):
        return False, f"payload mass {mass:g} kg is above the robot's <max_payload_kg> {float(max_mass):g}"
    if math.sqrt(sum(v * v for v in c)) > max_offset:
        return False, f"payload COM {c} is more than {max_offset:g} m from the tool frame — a typo?"
    return True, ""


class PayloadModel:
    """Owns link N's base body (captured once) and applies a payload to the rtb robot's link in place."""

    def __init__(self, robot: Any, link_index: int = -1):
        self.robot = robot
        self.link = robot.links[link_index]
        self.base_m = float(self.link.m)
        self.base_r = np.array(np.asarray(self.link.r, dtype=np.float64).reshape(3))
        self.base_i = np.array(np.asarray(self.link.I, dtype=np.float64).reshape(3, 3))
        self.mass = 0.0
        self.com = np.zeros(3)

    def apply(self, mass: float, com: Any) -> tuple:
        """Edit the link to gripper + part; returns the combined (m, r, I). mass 0 restores the base body exactly."""
        m, r, i = combined_body(self.base_m, self.base_r, self.base_i, mass, com)
        self.link.m = m
        self.link.r = r
        self.link.I = i
        self.mass = float(mass)
        self.com = np.asarray(com, dtype=np.float64).reshape(3).copy()
        return m, r, i

    def restore(self) -> tuple:
        return self.apply(0.0, (0.0, 0.0, 0.0))


def rebuild_dynamics(robot: Any) -> tuple:
    """Fresh dynamics functions from the robot's CURRENT links: (gravity_fn(q) -> g, inertia_fn(q, out),
    minv_fn(q, out), dynamics_ff_fn(q, v, a, inertia, coriolis), momentum_fn(q, v) -> (p, beta) — the external torque
    observer's terms, 2026-10-16) — each with its own Pinocchio model and Data, exactly
    as start-up builds them (utility/pinocchio_attach.attach_pinocchio, utility/haptic_walls.make_effective_inertia /
    make_inverse_mass). Raises if any of them does not build; the caller then keeps the functions it had."""
    from utility.haptic_walls import make_effective_inertia, make_inverse_mass
    from utility.pinocchio_attach import (_make_pinocchio_gravity, build_pinocchio_dynamics, make_pinocchio_dynamics_ff,
                                          make_pinocchio_momentum_terms)
    model, data = build_pinocchio_dynamics(robot)
    gravity_fn = _make_pinocchio_gravity(model, data)
    inertia_fn = make_effective_inertia(robot)
    minv_fn = make_inverse_mass(robot)
    if inertia_fn is None or minv_fn is None:
        raise RuntimeError("the walls' dynamics model did not build")
    return gravity_fn, inertia_fn, minv_fn, make_pinocchio_dynamics_ff(robot), make_pinocchio_momentum_terms(robot)


def apply_payload_command(cmd_dict: dict, model: PayloadModel, haptic_walls: Any, rti: Any, max_mass: float) -> tuple[bool, str]:
    """RTI's set_payload handler, as a function the net can run on the real model: validate, edit the link,
    rebuild the dynamics, swap them in (robot.pinocchio_gravity, the walls' two functions), publish to shared
    memory. On a rebuild failure the link goes back to the PREVIOUS payload so the link and the models in use
    agree. Returns (ok, message)."""
    mass = cmd_dict.get("mass") if isinstance(cmd_dict, dict) else None
    com = cmd_dict.get("com", (0.0, 0.0, 0.0)) if isinstance(cmd_dict, dict) else None
    ok, why = check_payload(mass, com, max_mass)
    if not ok:
        return False, why
    prev_mass, prev_com = model.mass, model.com.copy()
    model.apply(float(mass), com)
    try:
        gravity_fn, inertia_fn, minv_fn, dynamics_ff_fn, momentum_fn = rebuild_dynamics(model.robot)
    except Exception as exc:                      # the link goes back to what the models in use describe
        model.apply(prev_mass, prev_com)
        return False, f"dynamics rebuild failed ({exc}) — payload unchanged ({prev_mass:g} kg)"
    model.robot.pinocchio_gravity = gravity_fn
    model.robot.pinocchio_dynamics_ff = dynamics_ff_fn          # inertia / Coriolis feedforward follows the payload
    model.robot.pinocchio_momentum = momentum_fn                # so does the external torque (the observer restarts)
    if haptic_walls is not None:
        haptic_walls.set_dynamics(inertia_fn, minv_fn)
    if rti is not None:
        rti["payload_mass"][0] = model.mass
        rti["payload_com_x"][0], rti["payload_com_y"][0], rti["payload_com_z"][0] = model.com
    c = model.com
    return True, (f"payload {model.mass:g} kg at ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}) m in the tool frame — tool link now "
                  f"{float(model.link.m):.3f} kg, gravity + feedforward + wall dynamics rebuilt")
