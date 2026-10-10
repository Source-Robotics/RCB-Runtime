#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: system_temperature.py
Description: Shared utility for reading Raspberry Pi CPU temperature.
    Centralizes temperature reading to avoid duplication across modules.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def get_raspberry_pi_temperature() -> float | None:
    """
    Retrieve the current temperature of the Raspberry Pi.

    The function reads the CPU temperature from the system file located at
    '/sys/class/thermal/thermal_zone0/temp', which provides the temperature
    in millidegrees Celsius. The value is converted to degrees Celsius before returning.

    Returns:
        float: The CPU temperature in degrees Celsius.
        None: If the temperature file is not found (e.g., running on a non-Raspberry Pi system).
    """
    try:
        # Read the temperature from the Raspberry Pi's thermal zone
        temp = int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000.0
        return temp
    except FileNotFoundError:
        return None
    except Exception:
        return None
