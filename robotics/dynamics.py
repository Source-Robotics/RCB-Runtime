#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: dynamics.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description: 
    Robot dynamics calculations including inverse dynamics,
    gravity compensation, external force/torque estimation, and
    torque compensation calculations.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import numpy as np

from utility.kinematics_backend import should_use_pinocchio


# ==========================================================================
# Inverse Dynamics
# ==========================================================================

def calculate_inverse_dynamics(robot, q, qd, qdd):
    """
    Calculate inverse dynamics (torques from motion) using Recursive Newton-Euler.
    
    Args:
        robot: Robot object
        q (np.array): Joint positions (rad)
        qd (np.array): Joint velocities (rad/s)
        qdd (np.array): Joint accelerations (rad/s²)
    
    Returns:
        np.array: Joint torques (Nm)
    """
    return robot.rne(q=q, qd=qd, qdd=qdd)


def calculate_gravity_compensation(robot, q):
    """
    Calculate gravity compensation torques (inverse dynamics with zero velocity/acceleration).
    
    Args:
        robot: Robot object
        q (np.array): Joint positions (rad)
    
    Returns:
        np.array: Gravity compensation torques (Nm)
    """
    joint_num = len(q)
    return robot.rne(q=q, qd=np.zeros(joint_num), qdd=np.zeros(joint_num))


# ==========================================================================
# Motor Torque Conversion
# ==========================================================================

def gravity_torque_rt(robot, q):
    """The per-tick gravity torque for the RT loop (data/update_shared_data.py, 2026-10-03).

    The Pinocchio dynamics model's g(q) when attach_pinocchio() could build it (1.3 us; equal to rne(q, 0, 0) to
    1e-15 — net test_gravity_pinocchio), else rtb's rne. The planner keeps calculate_gravity_compensation() so the
    trajectory goldens stay bit-identical. The returned array is the helper's own buffer: use it within the tick.
    """
    fn = getattr(robot, "pinocchio_gravity", None)
    if fn is not None and should_use_pinocchio(robot):
        return fn(q)
    return calculate_gravity_compensation(robot, q)


def build_torque_to_ma_factor(gear_ratio, gear_efficiency, motor_kt, motor_signs):
    """Build the per-joint scaling factor for joint Nm → motor mA.

    The factor is constant for the lifetime of a robot configuration. Compute
    once at startup and reuse every tick — one in-place multiply per call.

        motor_mA[i] = trq_nm[i] * factor[i]

    where

        factor[i] = motor_signs[i] * 1000 / (gear_ratio[i] * gear_efficiency[i] * motor_kt[i])

    This is the canonical source of the Nm → mA formula. Both the per-tick
    shared-memory update (data/update_shared_data.py → torque_to_motor_current)
    and the dispatch CAN-send path (utility/mode_dispatch.py → _to_motor_space)
    derive from this single definition — no risk of formula drift.

    Args:
        gear_ratio (np.array):      Per-joint gear ratios
        gear_efficiency (np.array): Per-joint gear efficiencies (0-1)
        motor_kt (np.array):        Per-joint motor torque constants (Nm/A)
        motor_signs (np.array):     Per-joint direction signs (-1 or 1)

    Returns:
        np.ndarray (float64): Per-joint scaling factor
    """
    gear_ratio      = np.asarray(gear_ratio,      dtype=np.float64)
    gear_efficiency = np.asarray(gear_efficiency, dtype=np.float64)
    motor_kt        = np.asarray(motor_kt,        dtype=np.float64)
    motor_signs     = np.asarray(motor_signs,     dtype=np.float64)
    return (motor_signs * 1000.0) / (gear_ratio * gear_efficiency * motor_kt)


def torque_to_motor_current(joint_torque, gear_ratio, gear_efficiency, motor_kt, motor_signs):
    """
    Convert joint torques to motor currents.

    Args:
        joint_torque (np.array): Joint torques in Nm
        gear_ratio (np.array): Gear ratios for each joint
        gear_efficiency (np.array): Gear efficiencies (0-1)
        motor_kt (np.array): Motor torque constants (Nm/A)
        motor_signs (np.array): Motor direction signs (-1 or 1)

    Returns:
        np.array: Motor currents in mA (int)

    Uses build_torque_to_ma_factor() under the hood — same canonical formula
    as the RT-loop dispatch path. Recomputes the factor each call; for per-tick
    code, prefer caching the factor and using `int(trq * factor)` directly.
    """
    factor = build_torque_to_ma_factor(gear_ratio, gear_efficiency, motor_kt, motor_signs)
    return (np.asarray(joint_torque, dtype=np.float64) * factor).astype(int)


def motor_current_to_joint_torque(motor_current, gear_ratio, gear_efficiency, motor_kt, motor_signs):
    """
    Convert motor currents to joint torques.
    
    Args:
        motor_current (np.array): Motor currents in mA
        gear_ratio (np.array): Gear ratios for each joint
        gear_efficiency (np.array): Gear efficiencies (0-1)
        motor_kt (np.array): Motor torque constants (Nm/A)
        motor_signs (np.array): Motor direction signs (-1 or 1)
    
    Returns:
        np.array: Joint torques in Nm
    """
    return (motor_current * gear_ratio * gear_efficiency * motor_kt * 0.001 * motor_signs)


# ==========================================================================
# Compensation Torque Calculation
# ==========================================================================

# ==========================================================================
# External Force/Torque Estimation
# ==========================================================================


def estimate_external_force_tcp(external_joint_torque, J):
    """
    Estimate external force/torque at TCP from joint torques using Jacobian.
    
    Args:
        external_joint_torque (np.array): External torques on joints (Nm)
        J (np.array): Jacobian matrix (6xN)
    
    Returns:
        np.array: External wrench at TCP [Fx, Fy, Fz, Mx, My, Mz]
                 Forces in N, moments in Nm
    """
    # F_ext = (J^T)^-1 * tau_ext. For our 6-DOF arm J.T is square, so solve()
    # is much cheaper than pinv(); fall back to pseudo-inverse near singularities.
    Jt = J.T
    try:
        return np.linalg.solve(Jt, external_joint_torque)
    except np.linalg.LinAlgError:
        return np.linalg.pinv(Jt) @ external_joint_torque


# ==========================================================================
# Complete Dynamics Calculation
# ==========================================================================


# ==========================================================================
# Simplified Interface for RTI Main Loop
# ==========================================================================
