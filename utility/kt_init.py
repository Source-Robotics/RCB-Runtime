"""Kt (motor torque constant) resolution at startup.

Reads Kt values from CAN bus (or falls back to XML config), then preloads
Motor_Kt_data and shared memory so the GUI reflects resolved values immediately.
"""

from __future__ import annotations

from typing import Any
from collections import deque

import numpy as np

import logging
logger = logging.getLogger("rti.startup")  # docs/logging.md "Logger names"
from communication.can_message_handlers import fetch_kt_from_can

_KT_FETCH_TIMEOUT_S = 0.35
_KT_FETCH_RETRIES = 3
_KT_FETCH_ROUNDS = 2
_KT_FETCH_SETTLE_S = 0.2


def resolve_and_preload_kt(
    _sys: dict,
    Motor: list,
    communication1: Any,
    joint_num: int,
    motor_kt: np.ndarray,
    gripper_kt: float,
    active_CAN_gripper: int,
    Motor_Kt_data: dict[int, deque],
    shared_data: Any,
) -> tuple[np.ndarray, float, tuple]:
    """Resolve Kt values from CAN or XML, then preload data structures.

    Returns the (possibly updated) motor_kt list, the gripper_kt float, and the warning keys for drives whose Kt was
    outside system.xml's <kt_tolerance> (J{n}:KT_MISMATCH_WARNING / GRIPPER:KT_MISMATCH_WARNING, 2026-11-15) — RTI
    passes them to the error system every tick, so they show for the whole run.
    """
    kt_source_cfg = _sys.get('kt_source', 'auto')
    # Provenance per joint, published to shared memory so the GUI can show whether each Kt was
    # actually read off the motor or is just the XML fallback. 1 = CAN, 0 = XML.
    kt_from_can = [0.0] * joint_num
    gripper_kt_from_can = 0.0
    kt_warnings: tuple = ()

    if kt_source_cfg == 'xml':
        logger.info("Kt: kt_source=xml — using XML Kt values, skipping CAN fetch")
        for _i in range(joint_num):
            logger.info(f"Kt: J{_i+1} Kt = {motor_kt[_i]:.6f} (XML)")
        if active_CAN_gripper:
            logger.info(f"Kt: Gripper Kt = {gripper_kt:.6f} (XML)")
    else:
        logger.info(
            f"Kt: Fetching Kt from CAN "
            f"({_KT_FETCH_ROUNDS} round(s), {_KT_FETCH_RETRIES} attempt(s), "
            f"{_KT_FETCH_TIMEOUT_S}s each, {_KT_FETCH_SETTLE_S}s settle)..."
        )
        motor_kt, _kt_sources, gripper_kt, _gripper_kt_src = fetch_kt_from_can(
            Motor, communication1, joint_num, motor_kt, _KT_FETCH_TIMEOUT_S,
            gripper_idx=joint_num if active_CAN_gripper else None,
            gripper_kt_xml=gripper_kt,
            retries=_KT_FETCH_RETRIES,
            rounds=_KT_FETCH_ROUNDS,
            settle_s=_KT_FETCH_SETTLE_S,
            tolerance=_sys.get('kt_tolerance', 0.3),      # system.xml <kt_tolerance> (2026-11-15)
        )
        for _i in range(joint_num):
            logger.info(f"Kt: J{_i+1} Kt = {motor_kt[_i]:.6f} ({_source_text(_kt_sources[_i])})")
        if active_CAN_gripper:
            logger.info(f"Kt: Gripper Kt = {gripper_kt:.6f} ({_source_text(_gripper_kt_src)})")
        kt_from_can = [1.0 if s == 'CAN' else 0.0 for s in _kt_sources]
        gripper_kt_from_can = 1.0 if _gripper_kt_src == 'CAN' else 0.0
        kt_warnings = tuple(f"J{_i + 1}:KT_MISMATCH_WARNING" for _i, s in enumerate(_kt_sources) if s == 'BAND')
        if active_CAN_gripper and _gripper_kt_src == 'BAND':
            kt_warnings += ("GRIPPER:KT_MISMATCH_WARNING",)

    # Preload Motor_Kt_data and shared memory so the GUI shows the resolved
    # values immediately, without waiting for the first 500-tick round-robin update.
    for _i in range(joint_num):
        Motor_Kt_data[_i][-1]["Kt"] = motor_kt[_i]
    shared_data.motor['motor_Kt'][:] = motor_kt
    shared_data.motor['motor_Kt_source'][:joint_num] = kt_from_can
    if active_CAN_gripper:
        Motor_Kt_data[joint_num][-1]["Kt"]       = gripper_kt
        shared_data.gripper['gripper_motor_kt'][0] = gripper_kt
        shared_data.gripper['gripper_motor_kt_source'][0] = gripper_kt_from_can

    return motor_kt, gripper_kt, kt_warnings


def _source_text(source):
    """The start-up log's word for where a Kt came from."""
    if source == 'BAND':
        return "XML: the drive's value is outside <kt_tolerance>, see the warning above"
    return source
