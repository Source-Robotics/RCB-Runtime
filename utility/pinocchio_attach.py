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

import math

import numpy as np
from roboticstoolbox import DHRobot


def _Rz(theta):
    c, s = math.cos(theta), math.sin(theta)
    T = np.eye(4)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T


def _Rx(theta):
    c, s = math.cos(theta), math.sin(theta)
    T = np.eye(4)
    T[1, 1] = c
    T[1, 2] = -s
    T[2, 1] = s
    T[2, 2] = c
    return T


def _Tz(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def _Tx(a):
    T = np.eye(4)
    T[0, 3] = a
    return T


def _link_transform(d, a, alpha):
    """Classical DH static link transform: Tz(d) * Tx(a) * Rx(alpha)."""
    return _Tz(d) @ _Tx(a) @ _Rx(alpha)


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
        placement = pin.SE3(prev_link_transform @ _Rz(link.offset))
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


def attach_pinocchio(rtb_robot):
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

    return rtb_robot
