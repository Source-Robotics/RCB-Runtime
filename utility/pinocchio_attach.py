"""
Attach Pinocchio FK/Jacobian methods to a roboticstoolbox DHRobot.

After attach_pinocchio(robot), the original robotics-toolbox robot still works,
and these extra methods are available:

    robot.pinocchio_fkine(q)             -> 4x4 ndarray, like robot.fkine(q).A
    robot.pinocchio_jacob0(q)            -> 6xN ndarray, like robot.jacob0(q)
    robot.pinocchio_fkine_and_jacob0(q)  -> (4x4 ndarray, 6xN ndarray)
    robot.pinocchio_fkine_jacob0_jdot(q, qd)
        -> (4x4 ndarray, 6xN Jacobian, 6xN Jacobian time derivative)

The one-shot fkine_and_jacob0 call is intended for the RT loop: it shares one
Pinocchio FK pass between pose extraction and the base-frame spatial Jacobian.

Supported model type:
    Classical-DH revolute chains. Modified DH and prismatic joints need a
    separate conversion.
"""

from __future__ import annotations

from typing import Any

import logging
logger = logging.getLogger("kin")  # docs/logging.md "Logger names"
import math

import numpy as np
import utility.quiet_imports  # noqa: F401,E402  # first: imports roboticstoolbox without NumPy's 1.x stderr block
from roboticstoolbox import DHRobot


def _rot_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    T = np.eye(4)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T


def _rot_x(theta):
    c, s = math.cos(theta), math.sin(theta)
    T = np.eye(4)
    T[1, 1] = c
    T[1, 2] = -s
    T[2, 1] = s
    T[2, 2] = c
    return T


def _trans_z(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def _trans_x(a):
    T = np.eye(4)
    T[0, 3] = a
    return T


def _link_transform(d, a, alpha):
    """Classical DH static link transform: Tz(d) * Tx(a) * Rx(alpha)."""
    return _trans_z(d) @ _trans_x(a) @ _rot_x(alpha)


def _build_pinocchio_model(rtb_robot):
    """Build a Pinocchio model that matches a classical-DH DHRobot."""
    import pinocchio as pin

    if not isinstance(rtb_robot, DHRobot):
        raise TypeError(f"Expected DHRobot, got {type(rtb_robot).__name__}")

    if not rtb_robot.links:
        raise ValueError("Cannot build Pinocchio model for robot with no links")

    for i, link in enumerate(rtb_robot.links):
        if not link.isrevolute:
            raise NotImplementedError(
                f"Link {i} is non-revolute (sigma={link.sigma}); only revolute DH chains are supported"
            )

    model = pin.Model()
    model.name = f"{rtb_robot.name}_pinocchio"

    base_T = np.asarray(rtb_robot.base.A, dtype=np.float64) if rtb_robot.base is not None else np.eye(4)
    tool_T = np.asarray(rtb_robot.tool.A, dtype=np.float64) if rtb_robot.tool is not None else np.eye(4)

    parent_joint_id = 0
    prev_link_transform = base_T
    joint_ids = []

    for i, link in enumerate(rtb_robot.links):
        placement = pin.SE3(prev_link_transform @ _rot_z(link.offset))
        joint_name = link.name or f"j{i + 1}"
        joint_id = model.addJoint(parent_joint_id, pin.JointModelRZ(), placement, joint_name)

        # Inertia is irrelevant for FK/Jacobian correctness, but Pinocchio wants
        # a body attached to the joint for a complete model.
        model.appendBodyToJoint(joint_id, pin.Inertia.Random(), pin.SE3.Identity())

        joint_ids.append(joint_id)
        parent_joint_id = joint_id
        prev_link_transform = _link_transform(link.d, link.a, link.alpha)

    ee_frame = pin.Frame(
        name="ee",
        parent_joint=joint_ids[-1],
        parent_frame=0,
        placement=pin.SE3(prev_link_transform @ tool_T),
        type=pin.FrameType.OP_FRAME,
    )
    ee_frame_id = model.addFrame(ee_frame)
    return model, model.createData(), ee_frame_id


def _make_pinocchio_fkine(model, data, ee_frame_id):
    import pinocchio as pin

    def pinocchio_fkine(q):
        q_arr = np.asarray(q, dtype=np.float64)
        pin.framesForwardKinematics(model, data, q_arr)
        return data.oMf[ee_frame_id].homogeneous.copy()

    return pinocchio_fkine


def _make_pinocchio_jacob0(model, data, ee_frame_id):
    import pinocchio as pin

    def pinocchio_jacob0(q):
        q_arr = np.asarray(q, dtype=np.float64)
        return pin.computeFrameJacobian(
            model, data, q_arr, ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        ).copy()

    return pinocchio_jacob0


def _make_pinocchio_fkine_and_jacob0(model, data, ee_frame_id):
    import pinocchio as pin

    def pinocchio_fkine_and_jacob0(q):
        q_arr = np.asarray(q, dtype=np.float64)
        pin.computeJointJacobians(model, data, q_arr)
        pin.updateFramePlacements(model, data)
        pose = data.oMf[ee_frame_id].homogeneous.copy()
        jacobian = pin.getFrameJacobian(
            model, data, ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        ).copy()
        return pose, jacobian

    return pinocchio_fkine_and_jacob0


def _make_pinocchio_fkine_jacob0_jdot(model, data, ee_frame_id):
    import pinocchio as pin

    def pinocchio_fkine_jacob0_jdot(q, qd):
        q_arr = np.asarray(q, dtype=np.float64)
        qd_arr = np.asarray(qd, dtype=np.float64)
        pin.computeJointJacobiansTimeVariation(model, data, q_arr, qd_arr)
        pin.updateFramePlacements(model, data)
        pose = data.oMf[ee_frame_id].homogeneous.copy()
        jacobian = pin.getFrameJacobian(
            model, data, ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        ).copy()
        jacobian_dot = pin.getFrameJacobianTimeVariation(
            model, data, ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        ).copy()
        return pose, jacobian, jacobian_dot

    return pinocchio_fkine_jacob0_jdot


def build_pinocchio_dynamics(rtb_robot: Any) -> tuple[Any, Any]:
    """A Pinocchio model of the DHRobot WITH its real dynamics, for the mass matrix: each link's mass, centre of
    mass and inertia tensor (rtb gives them in link frame i, the frame AFTER the link's DH transform), plus the
    motor rotor reflected to the joint as armature Jm*G^2 — exactly what rtb's inertia() includes. Matches
    rtb's inertia() to ~1e-15 relative (dummy_test_scripts/test_haptic_walls.py). Returns (model, data); the
    caller owns the Data: one per thread (one Pinocchio Data is not thread-safe).

    (The FK model built by _build_pinocchio_model has RANDOM inertias on purpose — FK/Jacobians do not use
    them — so it must never be used for dynamics.)"""
    import pinocchio as pin

    if not isinstance(rtb_robot, DHRobot):
        raise TypeError(f"Expected DHRobot, got {type(rtb_robot).__name__}")
    model = pin.Model()
    model.name = f"{rtb_robot.name}_pinocchio_dynamics"
    prev = np.asarray(rtb_robot.base.A, dtype=np.float64) if rtb_robot.base is not None else np.eye(4)
    parent = 0
    for i, link in enumerate(rtb_robot.links):
        if not link.isrevolute:
            raise NotImplementedError(f"Link {i} is non-revolute; only revolute DH chains are supported")
        joint_id = model.addJoint(parent, pin.JointModelRZ(), pin.SE3(prev @ _rot_z(link.offset)), link.name or f"j{i + 1}")
        link_T = _link_transform(link.d, link.a, link.alpha)
        inertia = pin.Inertia(float(link.m), np.asarray(link.r, dtype=np.float64), np.asarray(link.I, dtype=np.float64))
        model.appendBodyToJoint(joint_id, inertia.se3Action(pin.SE3(link_T)), pin.SE3.Identity())
        model.armature[joint_id - 1] = float(link.Jm) * float(link.G) ** 2
        parent, prev = joint_id, link_T
    return model, model.createData()


def _make_pinocchio_gravity(model, data):
    """fn(q) -> the dynamics model's generalized gravity g(q) (Nm, joint space) in a buffer the closure owns —
    use it within the tick, it is overwritten by the next call."""
    import pinocchio as pin
    out = np.zeros(model.nq, dtype=np.float64)

    def gravity(q):
        np.copyto(out, pin.computeGeneralizedGravity(model, data, np.asarray(q, dtype=np.float64)))
        return out
    return gravity


def make_pinocchio_dynamics_ff(rtb_robot: Any):
    """fn(q, v, a, inertia, coriolis) -> (tau_inertia, tau_coriolis), Nm, joint space (2026-10-10,
    TODO/dynamics_feedforward_plan.md): tau_inertia = M(q)·a — the links AND the motor rotors (the model's armature
    Jm*G^2) — and tau_coriolis = C(q, v)·v, each from rnea on a GRAVITY-FREE copy of the dynamics model (gravity has
    its own model and switch). Equal to rtb's inertia(q) @ a and coriolis(q, v) @ v to ~1e-16 Nm (net
    test_dynamics_ff). A term that is switched off is returned as zeros without computing it. The two arrays are the
    closure's own buffers: use them within the tick. ~2.7 us per term on the Pi."""
    import pinocchio as pin
    model, data = build_pinocchio_dynamics(rtb_robot)
    model.gravity.setZero()
    zero = np.zeros(model.nv, dtype=np.float64)
    tau_i = np.zeros(model.nv, dtype=np.float64)
    tau_c = np.zeros(model.nv, dtype=np.float64)

    def dynamics_ff(q, v, a, inertia, coriolis):
        if inertia:
            np.copyto(tau_i, pin.rnea(model, data, q, zero, a))
        else:
            tau_i.fill(0.0)
        if coriolis:
            np.copyto(tau_c, pin.rnea(model, data, q, v, zero))
        else:
            tau_c.fill(0.0)
        return tau_i, tau_c
    return dynamics_ff


def make_pinocchio_momentum_terms(rtb_robot: Any):
    """fn(q, v) -> (p, beta) (2026-10-16, utility/momentum_observer.py): the generalized momentum p = M(q)·v (Nm·s;
    the links AND the rotors' armature) and beta = g(q) - C(q, v)ᵀ·v (Nm) — the two model terms a momentum observer
    needs to estimate the external joint torque during motion without the joint acceleration. M(q)·v is rnea on a
    gravity-free copy with v as the acceleration and no velocity; C from computeCoriolisMatrix; g from the model with
    gravity. Their own models and Data (one Pinocchio Data is not thread-safe). The two arrays are the closure's
    buffers: use them within the tick. Equal to rtb's inertia(q) @ v and gravload(q) - coriolis(q, v).T @ v to ~1e-12
    (net test_momentum_observer)."""
    import pinocchio as pin
    model, data = build_pinocchio_dynamics(rtb_robot)
    model_ng, data_ng = build_pinocchio_dynamics(rtb_robot)
    model_ng.gravity.setZero()
    zero = np.zeros(model.nv, dtype=np.float64)
    p = np.zeros(model.nv, dtype=np.float64)
    beta = np.zeros(model.nv, dtype=np.float64)

    def momentum_terms(q, v):
        np.copyto(p, pin.rnea(model_ng, data_ng, q, zero, v))                  # M(q)·v
        np.dot(pin.computeCoriolisMatrix(model, data, q, v).T, v, out=beta)    # C(q, v)ᵀ·v
        np.subtract(pin.computeGeneralizedGravity(model, data, q), beta, out=beta)
        return p, beta
    return momentum_terms


def attach_pinocchio(rtb_robot: Any) -> Any:
    """Attach Pinocchio model/data and FK/Jacobian helpers to rtb_robot."""
    model, data, ee_frame_id = _build_pinocchio_model(rtb_robot)

    rtb_robot.pinocchio_model = model
    rtb_robot.pinocchio_data = data
    rtb_robot.pinocchio_ee_frame_id = ee_frame_id
    rtb_robot.pinocchio_fkine = _make_pinocchio_fkine(model, data, ee_frame_id)
    rtb_robot.pinocchio_jacob0 = _make_pinocchio_jacob0(model, data, ee_frame_id)
    rtb_robot.pinocchio_fkine_and_jacob0 = _make_pinocchio_fkine_and_jacob0(
        model, data, ee_frame_id
    )
    rtb_robot.pinocchio_fkine_jacob0_jdot = _make_pinocchio_fkine_jacob0_jdot(
        model, data, ee_frame_id
    )

    # Gravity for the RT loop (2026-10-03): the DYNAMICS model's g(q) — equal to rtb's rne(q, 0, 0) to 1.8e-15 Nm
    # over 300 poses, at 1.3 us instead of 24 (tools/bench_rti_tick.py). Its own Data, one per function, as the
    # walls' inertia helpers have. Best effort: a model whose dynamics do not build keeps the rne path
    # (robotics/dynamics.gravity_torque_rt).
    try:
        dyn_model, dyn_data = build_pinocchio_dynamics(rtb_robot)
        rtb_robot.pinocchio_gravity = _make_pinocchio_gravity(dyn_model, dyn_data)
    except Exception as exc:
        logger.warning("Pinocchio: no dynamics model for gravity (%s) — rtb rne stays", exc)
    # Inertia + Coriolis feedforward (2026-10-10): its own gravity-free model and Data. No model -> no feedforward
    # (mode_dispatch._add_dynamics_ff adds nothing), never a fallback to something slower in the tick.
    try:
        rtb_robot.pinocchio_dynamics_ff = make_pinocchio_dynamics_ff(rtb_robot)
    except Exception as exc:
        logger.warning("Pinocchio: no dynamics model for inertia / Coriolis feedforward (%s)", exc)
    # The external torque estimate's model terms (2026-10-16, utility/momentum_observer.py). No model -> the external
    # torque falls back to the static estimate (measured - gravity), as before.
    try:
        rtb_robot.pinocchio_momentum = make_pinocchio_momentum_terms(rtb_robot)
    except Exception as exc:
        logger.warning("Pinocchio: no dynamics model for the external torque observer (%s)", exc)

    return rtb_robot
