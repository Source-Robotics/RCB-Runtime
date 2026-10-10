"""Parse the robot, gripper, system, cell and homing XMLs — the package behind `import config.xml_parser`.

Until 2026-10-03 this was one 1 626-line file; it is now one module per document (refactor Phase 4):

    xml_access   element access for the robot and gripper XMLs (get/set values, parse a file)
    system       config/system.xml: active robot/gripper/cell, jog/RTI/maintenance/friction/monitor policy
    zones        cells, tool spheres, the haptic-wall policy and its stability table
    walls        per-joint freedrive walls
    homing       the homing sequence XML
    robot        load_full_robot_config() and the fallbacks for fields a robot XML may omit
    validate     validate_robot_config()

This file re-exports every name the single file had — public AND private (nets use several) — so no caller
changes: `import config.xml_parser as xp; xp.load_full_robot_config(...)` works as before. New code may
import from the submodule directly. dummy_test_scripts/test_xml_parser_facade.py pins the surface.
"""

# The single file's module-level imports, which callers reach through it (`read_XML2.ET.parse`).
import logging  # noqa: F401
import math  # noqa: F401
import xml.etree.ElementTree as ET  # noqa: F401
from pathlib import Path  # noqa: F401

import numpy as np  # noqa: F401

from config.xml_parser.xml_access import (  # noqa: F401
    parse_robot_config, get_robot_name, get_joint_count, get_active_gripper, get_single_value, get_float_value,
    get_child_float, get_all_values, get_all_strings, get_gripper_values, load_gripper_element, get_gripper_value,
    set_single_value, set_all_values, save_robot_config, set_value, _convert_to_float,
)
from config.xml_parser.system import (  # noqa: F401
    load_system_config, _JOG_DEFAULTS_FALLBACK, _RTI_DEFAULTS_FALLBACK, _RTI_DEFAULT_BOUNDS,
    _MAINTENANCE_DEFAULTS_FALLBACK, _MAINTENANCE_DEFAULT_BOUNDS, _parse_maintenance_defaults,
    _FRICTION_COMP_FALLBACK, _FRICTION_COMP_BOUNDS, _parse_friction_comp, _SAFETY_MONITOR_FALLBACK,
    _SAFETY_MONITOR_BOUNDS, _parse_safety_monitor, _parse_rti_defaults, _parse_jog_defaults,
)
from config.xml_parser.zones import (  # noqa: F401
    _ZONE_TAGS, _ZONE_ATTRS, _HAPTIC_FALLBACK, _HAPTIC_BOUNDS, HAPTIC_STABLE, HAPTIC_REFERENCE_MASS,
    haptic_damping_window, _parse_haptic_walls, _zone_numbers, parse_cell_zones, parse_tool_spheres, load_zone_setup, zone_setup_from_wire,
)
from config.xml_parser.walls import (  # noqa: F401
    WALL_NUMBERS, WALL_SPEC_AUTO, WALL_SCALINGS, WALL_FILTER_MAX_MS, _WALL_FIELDS_BEFORE_2026_10_02, wall_spec,
    get_freedrive_walls, _wall_attr, write_freedrive_walls,
)
from config.xml_parser.homing import (  # noqa: F401
    parse_homing_sequence,
)
from config.xml_parser.robot import (  # noqa: F401
    load_software_joint_limits, get_safety_config, _DERIVED_ACCEL_TIME_S, _DERIVED_JERK_FACTOR,
    _DERIVED_TRQ_RATE_PER_TAU, _fill_motion_limit_defaults, _MODE_LIMIT_MAP, _fill_mode_limit_defaults,
    DEFAULT_VOLTAGE_LIMIT_MV, MIN_VOLTAGE_LIMIT_MV, clamp_voltage_limit, _voltage_limit_or_default, _nonneg,
    load_full_robot_config,
)
from config.xml_parser.validate import (  # noqa: F401
    _val_logger, _REQUIRED_JOINT_TAGS, _VALID_ENCODER_BITS, _cfg_array, _check_joint_count, _check_required_arrays,
    _POSITIVE_FIELDS, _check_positive, _check_encoder_bits, _check_hard_limits, _check_soft_limits,
    _check_loop_timing, _check_gripper, _log_issues, validate_robot_config,
)

# `from config.xml_parser import *` exports what the single file exported (its public names), not the
# submodule names the package namespace now also holds.
__all__ = [
    "DEFAULT_VOLTAGE_LIMIT_MV",
    "ET",
    "HAPTIC_REFERENCE_MASS",
    "HAPTIC_STABLE",
    "MIN_VOLTAGE_LIMIT_MV",
    "Path",
    "WALL_FILTER_MAX_MS",
    "WALL_NUMBERS",
    "WALL_SCALINGS",
    "WALL_SPEC_AUTO",
    "clamp_voltage_limit",
    "get_active_gripper",
    "get_all_strings",
    "get_all_values",
    "get_child_float",
    "get_float_value",
    "get_freedrive_walls",
    "get_gripper_value",
    "get_gripper_values",
    "get_joint_count",
    "get_robot_name",
    "get_safety_config",
    "get_single_value",
    "haptic_damping_window",
    "load_full_robot_config",
    "load_gripper_element",
    "load_software_joint_limits",
    "load_system_config",
    "load_zone_setup",
    "zone_setup_from_wire",
    "logging",
    "math",
    "np",
    "parse_cell_zones",
    "parse_homing_sequence",
    "parse_robot_config",
    "parse_tool_spheres",
    "save_robot_config",
    "set_all_values",
    "set_single_value",
    "set_value",
    "validate_robot_config",
    "wall_spec",
    "write_freedrive_walls",
]
