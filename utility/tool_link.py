"""A gripper's tool link: the last link of the arm model (DH d / a / alpha, mass, centre of mass, inertia, rotor inertia,
gear, damping, Coulomb friction, joint range) from the gripper XML's <kinematics> block.

One reader for both users (2026-11-17, TODO/live_gripper_swap_plan.md P2a): robots/PAR6.py builds its gripper links
with it, and RTI's live gripper swap edits the running model's last link in place with it, so the swapped link is
exactly what a fresh start builds. Pure (xml.etree only); the qlim the robot model then overrides with its own J6 range.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET  # nosec B405 — our own configuration files
from math import pi
from pathlib import Path

LINK_ATTRS = ("d", "a", "alpha", "m", "r", "I", "Jm", "G", "B", "Tc")     # what a live swap copies into the link


def tool_link_params(root: ET.Element) -> dict | None:
    """{d, a, alpha, m, r, I, Jm, G, B, Tc, qlim} of a gripper XML's root, or None without a <kinematics> block (the
    model then uses the Flange's link). Raises ValueError for a <kinematics> block with a missing / malformed value."""
    kin = root.find("kinematics")
    if kin is None:
        return None

    def text(tag, default=None):
        value = kin.findtext(tag, default=default)
        if value is None:
            raise ValueError(f"no <{tag}>")
        return value

    try:
        return {
            "d": float(text("d")),
            "a": float(text("a", "0")),
            "alpha": float(text("alpha", str(pi))),
            "m": float(text("mass")),
            "r": [float(x) for x in text("com").split()],
            "I": [float(x) for x in text("inertia").split()],
            "Jm": float(text("Jm", "1.5e-6")),
            "G": float(text("G", "10")),
            "B": float(text("B", "1e-4")),
            "Tc": [float(x) for x in text("Tc", "0.02 -0.02").split()],
            "qlim": [float(x) * pi / 180 for x in text("qlim_deg").split()],
        }
    except ValueError as e:
        raise ValueError(f"gripper {root.get('name')!r}: a malformed <kinematics> block ({e})") from None


def tool_link_params_of(grippers_dir, name: str) -> dict | None:
    """tool_link_params of grippers/<name>.xml (the Flange's when it has no <kinematics>, as robots/PAR6.py does).
    ValueError for a missing or malformed file."""
    path = Path(grippers_dir) / f"{name}.xml"
    try:
        root = ET.parse(path).getroot()     # nosec B314 — our own configuration file
    except (OSError, ET.ParseError) as e:
        raise ValueError(f"gripper {name!r}: {e}") from None
    params = tool_link_params(root)
    if params is None and name != "Flange":
        return tool_link_params_of(grippers_dir, "Flange")
    return params
