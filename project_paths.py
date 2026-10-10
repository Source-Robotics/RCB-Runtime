#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: project_paths.py
Author: Petar Crnjak
Company: Source Robotics
Description:
    Central definition of all project-level paths.
    Import this module instead of hardcoding paths anywhere in the codebase.

Usage:
    from project_paths import PROJECT_ROOT, CONFIG_DIR, LOGS_DIR, ROBOTS_DIR
    xml_path = ROBOTS_DIR / "PAR6.xml"
"""

from pathlib import Path

# Root of the RCB Control Runtime project
PROJECT_ROOT = Path(__file__).resolve().parent

# ─── Sub-directories ────────────────────────────────────────────────────────
CONFIG_DIR       = PROJECT_ROOT / "config"
LOGS_DIR         = PROJECT_ROOT / "logs"
ROBOTS_DIR       = PROJECT_ROOT / "robots"
GRIPPERS_DIR     = PROJECT_ROOT / "grippers"
HARDWARE_DIR     = PROJECT_ROOT / "hardware"
DATA_DIR         = PROJECT_ROOT / "data"
COMMUNICATION_DIR = PROJECT_ROOT / "communication"
ROBOTICS_DIR     = PROJECT_ROOT / "robotics"
UTILITY_DIR      = PROJECT_ROOT / "utility"
IMAGES_DIR       = PROJECT_ROOT / "images"
TEMP_DATA_DIR    = PROJECT_ROOT / "temp_data"
DUMMY_TESTS_DIR  = PROJECT_ROOT / "dummy_test_scripts"

# ─── Common files ────────────────────────────────────────────────────────────
SYSTEM_XML     = CONFIG_DIR  / "system.xml"           # Top-level: selects active robot
CELLS_DIR      = CONFIG_DIR  / "cells"                # Safety-zone cell files, chosen by <active_cell>
# Source Robotics' PAR6 description package 2.0 (URDFs + meshes) — the source of the self-collision geometry (user,
# 2026-10-07: "use it for everything"); outside the repo. tools/self_collision_capsules.py and the dashboard's jaw view.
PAR6_DESCRIPTION_DIR = PROJECT_ROOT.parent / "PAR6 Description" / "urdf" / "par6_description"
FILTER_CONFIG_PATH = CONFIG_DIR / "filters.xml"       # Per-signal filter specs (utility/filter_config.py); hot-reloaded by reload_filters
PAR6_XML       = ROBOTS_DIR / "PAR6.xml"             # Direct path kept for tools/tests
DATA_RECIPE_DIR  = PROJECT_ROOT / "data_recipe"       # Directory of per-recipe *.xml files
JOBLIB_DATA      = TEMP_DATA_DIR / "joblib_data.pkl"  # Cached trajectory data
# Dropped by the flasher after a SUCCESSFUL flash; consumed by RTI when it leaves
# FLASHING, which then invalidates homing. A file rather than a shared-memory
# flag on purpose: the flasher is a separate process that must keep working with
# the runtime down, and adding an shm field would force a coordinated restart
# (and therefore a re-home) on every deploy. Written cold, read once per mode
# exit — never on the RT hot path.
FLASH_PERFORMED_MARKER = TEMP_DATA_DIR / ".flash_performed"
SUPERVISOR_SETTINGS = CONFIG_DIR / "supervisor_settings.json"  # OLED menu user prefs

# ─── Log files ───────────────────────────────────────────────────────────────
RTI_LOG          = LOGS_DIR / "RTI.log"
SUPERVISOR_LOG   = LOGS_DIR / "supervisor.log"
HIGH_LEVEL_LOG   = LOGS_DIR / "high_level.log"
GUI_LOG          = LOGS_DIR / "gui.log"
MOTOR_DATA_LOG   = LOGS_DIR / "motor_data.log"
CRITICAL_LOG     = LOGS_DIR / "critical_errors.log"   # Persistent critical error log
ACTIVITY_RTI_LOG  = LOGS_DIR / "activity_rti.log"     # Unified activity log — RTI-owned entries
ACTIVITY_EXEC_LOG = LOGS_DIR / "activity_exec.log"    # Unified activity log — EXEC-owned entries

# ─── Font / assets ───────────────────────────────────────────────────────────
OLED_FONT        = IMAGES_DIR / "Roboto-VariableFont_wdth,wght.ttf"
OLED_UI_FONT     = IMAGES_DIR / "DejaVuSans.ttf"   # supervisor OLED menu UI font

