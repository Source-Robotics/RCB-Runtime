#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: logger_setup.py
Author: Petar Crnjak
Date: 2024-06-11
Version: 1.0
Description: 
    Helper function for setting up logging.
    

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
Dependencies: os, sys
"""

import logging
from logging.handlers import RotatingFileHandler
import os


_FMT = '%(levelname)-8s | %(created)f | %(name)s | %(message)s'


def setup_logging(log_file="logs/system.log", log_to_console=True, critical_log_file=None):
    """
    Configure the root logger for the process.

    Call this once at the top of each main script (RTI.py, supervisor.py, etc.).
    All submodules that use logging.getLogger(__name__) automatically inherit
    the configuration — no extra setup needed in submodules.

    Args:
        log_file (str):           Path to the main rotating log file.
        log_to_console (bool):    If True, also print output to the terminal.
        critical_log_file (str):  Optional path for a persistent CRITICAL-only log
                                  that survives multiple restarts and is not rotated
                                  as aggressively. Pass str(CRITICAL_LOG) from project_paths.
    """
    root = logging.getLogger()
    if root.handlers:
        return  # Already configured — avoid duplicate handlers on re-import

    os.makedirs(os.path.dirname(log_file) or "logs", exist_ok=True)
    formatter = logging.Formatter(_FMT)

    # Main rotating file handler (10 MB x 5 backups = up to 50 MB history)
    file_handler = RotatingFileHandler(log_file, maxBytes=10_000_000, backupCount=5)
    file_handler.setFormatter(formatter)
    root.addHandler(file_handler)

    # Optional console output
    if log_to_console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        root.addHandler(console_handler)

    # Optional persistent CRITICAL-only log (larger backups, survives restarts)
    if critical_log_file:
        os.makedirs(os.path.dirname(critical_log_file) or "logs", exist_ok=True)
        critical_handler = RotatingFileHandler(critical_log_file, maxBytes=5_000_000, backupCount=10)
        critical_handler.setLevel(logging.CRITICAL)
        critical_handler.setFormatter(formatter)
        root.addHandler(critical_handler)

    root.setLevel(logging.INFO)