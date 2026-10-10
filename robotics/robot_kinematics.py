#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: robot_kinematics.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description: 
    Robot kinematics calculations including forward kinematics,
    inverse kinematics, Jacobian, and TCP transformations.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import numpy as np
from spatialmath import SE3

from utility.kinematics_backend import (
    require_selected_backend_available,
    should_use_pinocchio,
    write_rpy_xyz,
)


# ==========================================================================
# Forward Kinematics
# ==========================================================================

def calculate_forward_kinematics(robot, joint_position):
    """
    Calculate forward kinematics for given joint positions.
    
    Args:
        robot: Robot object (PAR6, INDIGO, or SKAR)
        joint_position (np.array): Joint positions in radians
    
    Returns:
        SE3: Transformation matrix (4x4) from base to TCP
    """
    require_selected_backend_available(robot)
    if should_use_pinocchio(robot):
        return SE3(robot.pinocchio_fkine(joint_position), check=False)
    return robot.fkine(joint_position)


def get_tcp_pose_rpy(robot, joint_position, order='xyz'):
    """
    Get TCP pose as [x, y, z, roll, pitch, yaw].
    
    Args:
        robot: Robot object
        joint_position (np.array): Joint positions in radians
        order (str): Rotation order for RPY (default: 'xyz')
    
    Returns:
        np.array: 6-element array [x, y, z, rx, ry, rz] in meters and radians
    """
    TT = calculate_forward_kinematics(robot, joint_position)
    rpy_angles = TT.rpy(order=order)  # Returns [roll, pitch, yaw] in radians
    translation_xyz = TT.t  # Returns [x, y, z] translation
    
    return np.concatenate((translation_xyz, rpy_angles))


# ==========================================================================
# Inverse Kinematics
# ==========================================================================

def calculate_inverse_kinematics(robot, target_pose, q0=None, method='chan',
                                 ilimit=25, slimit=25, k=0.1, tol=1e-4):
    """
    Calculate inverse kinematics for target TCP pose.
    
    Args:
        robot: Robot object
        target_pose (SE3): Target transformation matrix
        q0 (np.array): Initial joint guess (default: None, uses current pose)
        method (str): IK method - 'chan', 'sugihara', etc. (default: 'chan')
        ilimit (int): Iteration limit (default: 25)
        slimit (int): Step limit (default: 25)
        k (float): Gain parameter (default: 0.1)
        tol (float): Tolerance (default: 1e-4)
    
    Returns:
        np.array: Joint positions solution, or None if failed
    """
    try:
        solution = robot.ik_LM(
            target_pose,
            q0=q0,
            ilimit=ilimit,
            slimit=slimit,
            k=k,
            method=method,
            tol=tol
        )
        if hasattr(solution, 'success') and not bool(solution.success):
            return None
        if hasattr(solution, 'q'):
            return solution.q
        if isinstance(solution, (tuple, list)) and solution:
            if len(solution) > 1 and isinstance(solution[1], (bool, np.bool_)) and not bool(solution[1]):
                return None
            return solution[0]
        return solution
    except Exception as e:
        print(f"[Kinematics][WARNING] IK failed: {e}")
        return None


# ==========================================================================
# Jacobian Calculations
# ==========================================================================

def calculate_jacobian(robot, joint_position, T=None):
    """
    Calculate Jacobian matrix (world frame).
    
    Args:
        robot: Robot object
        joint_position (np.array): Joint positions in radians
        T (SE3): Transformation matrix (optional, will calculate if None)
    
    Returns:
        np.array: 6xN Jacobian matrix
    """
    require_selected_backend_available(robot)
    if should_use_pinocchio(robot):
        return robot.pinocchio_jacob0(joint_position)

    if T is None:
        T = robot.fkine(joint_position)
    
    return robot.jacob0(q=joint_position, T=T)


# ==========================================================================
# TCP Velocity Calculations
# ==========================================================================

# ==========================================================================
# Manipulability
# ==========================================================================

def calculate_manipulability_yoshikawa(robot, joint_position, J=None):
    """
    Calculate Yoshikawa manipulability measure.
    
    Args:
        robot: Robot object
        joint_position (np.array): Joint positions
        J (np.array): Jacobian matrix (optional, will calculate if None)
    
    Returns:
        float: Manipulability measure
    """
    if J is None:
        J = calculate_jacobian(robot, joint_position)
    
    w = np.sqrt(np.linalg.det(J @ J.T))
    return w


def calculate_condition_number(J):
    """
    Calculate condition number of Jacobian (measure of singularity).
    
    Args:
        J (np.array): Jacobian matrix
    
    Returns:
        float: Condition number (higher = closer to singularity)
    """
    return np.linalg.cond(J)


# ==========================================================================
# Complete Kinematics Calculation
# ==========================================================================


def calculate_all_kinematics(
    robot,
    joint_position,
    joint_speed=None,
    include_diagnostics=True,
    return_transform=True,
    joint_acceleration=None,
):
    """Calculate pose, Jacobian, twist, acceleration, and diagnostics together.

    ``tcp_acceleration`` is calculated only when both joint velocity and joint
    acceleration are supplied. It is the geometric spatial acceleration in the
    base frame: ``J @ qdd + J_dot @ qd``.
    """
    require_selected_backend_available(robot)

    if joint_acceleration is not None and joint_speed is None:
        raise ValueError("joint_speed is required when calculating TCP acceleration")

    jacobian_dot = None
    if should_use_pinocchio(robot):
        if joint_acceleration is not None:
            if not hasattr(robot, "pinocchio_fkine_jacob0_jdot"):
                raise RuntimeError(
                    "Pinocchio acceleration support is not attached to this robot"
                )
            pose_4x4, J, jacobian_dot = robot.pinocchio_fkine_jacob0_jdot(
                joint_position, joint_speed
            )
        else:
            pose_4x4, J = robot.pinocchio_fkine_and_jacob0(joint_position)

        T = SE3(pose_4x4, check=False) if return_transform else None
        tcp_pose = np.empty(6, dtype=np.float64)
        tcp_pose[:3] = pose_4x4[:3, 3]
        write_rpy_xyz(pose_4x4, tcp_pose, start=3)
        tcp_pose_matrix = pose_4x4.reshape(16)
    else:
        # Forward kinematics - single fkine call, reused by the Jacobian.
        T_full = robot.fkine(joint_position)
        T = T_full if return_transform else None
        tcp_pose = np.concatenate((T_full.t, T_full.rpy(order="xyz")))
        tcp_pose_matrix = T_full.A.flatten()
        J = robot.jacob0(q=joint_position, T=T_full)

    tcp_speed = None
    if joint_speed is not None:
        tcp_speed = J @ np.asarray(joint_speed, dtype=np.float64)

    tcp_acceleration = None
    if joint_acceleration is not None:
        qd = np.asarray(joint_speed, dtype=np.float64)
        qdd = np.asarray(joint_acceleration, dtype=np.float64)
        if jacobian_dot is not None:
            convective_acceleration = jacobian_dot @ qd
        else:
            hessian = robot.hessian0(q=joint_position, J0=J)
            # RTB stores H as [derivative_joint, Cartesian_axis, Jacobian_joint].
            convective_acceleration = np.einsum(
                "kij,k,j->i", hessian, qd, qd
            )
        tcp_acceleration = J @ qdd + convective_acceleration

    manipulability = None
    condition_number = None
    jacobian_sigma_min = None
    if include_diagnostics:
        singular_values = np.linalg.svd(J, compute_uv=False)
        sigma_max = float(singular_values[0])
        jacobian_sigma_min = float(singular_values[-1])
        condition_number = (
            float("inf")
            if jacobian_sigma_min <= 1e-12
            else sigma_max / jacobian_sigma_min
        )
        manipulability = float(np.prod(singular_values))

    return {
        "T": T,
        "tcp_pose": tcp_pose,
        "tcp_pose_matrix": tcp_pose_matrix,
        "J": J,
        "tcp_speed": tcp_speed,
        "tcp_acceleration": tcp_acceleration,
        "manipulability": manipulability,
        "condition_number": condition_number,
        "jacobian_sigma_min": jacobian_sigma_min,
    }

if __name__ == "__main__":
    """Test kinematics functions"""
    print("Robot Kinematics Test")
    print("-" * 50)
    
    print("\nNote: This test requires a robot model (PAR6, INDIGO, or SKAR)")
    print("Attempting to create PAR6 robot...")
    
    try:
        from robots.PAR6 import PAR6
        
        # Create robot
        robot = PAR6(symbolic=False, gripper_name="Flange")
        print("✓ PAR6 robot created successfully")
        
        # Test joint positions (home position)
        q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        print(f"\nTest joint position: {q}")
        
        # Test forward kinematics
        print("\n1. Forward Kinematics:")
        tcp_pose = get_tcp_pose_rpy(robot, q)
        print(f"   TCP Pose [x,y,z,rx,ry,rz]: {tcp_pose}")
        
        # Test Jacobian
        print("\n2. Jacobian:")
        J = calculate_jacobian(robot, q)
        print(f"   Jacobian shape: {J.shape}")
        print(f"   Jacobian:\n{J}")
        
        # Test manipulability
        print("\n3. Manipulability:")
        w = calculate_manipulability_yoshikawa(robot, q, J)
        print(f"   Yoshikawa measure: {w:.6f}")
        
        kappa = calculate_condition_number(J)
        print(f"   Condition number: {kappa:.6f}")
        
        # Test complete kinematics
        print("\n4. Complete Kinematics Calculation:")
        joint_speed = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        kin_data = calculate_all_kinematics(robot, q, joint_speed)
        
        print(f"   TCP Speed: {kin_data['tcp_speed']}")
        print(f"   Manipulability: {kin_data['manipulability']:.6f}")
        
        print("\n✓ All kinematics tests passed!")
        
    except ImportError as e:
        print(f"\n✗ Could not import robot model: {e}")
        print("Skipping hardware-dependent tests")
