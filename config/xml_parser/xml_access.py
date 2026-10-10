"""Element access for the robot and gripper XMLs: parse a file, read one value / a per-joint array / a
gripper field, write values back. The building blocks every other part of the parser uses.

Split out of the single-file config/xml_parser.py on 2026-10-03 (refactor Phase 4); definitions moved verbatim.
"""

from __future__ import annotations

from collections.abc import Iterable
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def parse_robot_config(xml_path: str | Path) -> tuple[ET.Element, ET.Element]:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    joints = root.find('joints')
    return root, joints


def get_robot_name(root: ET.Element) -> str:
    return root.findtext('name', default='Unknown').strip()


def get_joint_count(joints: ET.Element) -> int:
    return int(joints.get('count', default='0'))


def get_active_gripper(root: ET.Element) -> str:
    return root.findtext('active_gripper', default='Unknown').strip()


def get_single_value(joints: ET.Element, joint_id: int, tag: str) -> float | str | None:
    joint = joints.find(f"./joint[@id='{joint_id}']")
    if joint is not None:
        element = joint.find(tag)
        if element is not None:
            return _convert_to_float(element.text.strip())
    return None


def get_float_value(root: ET.Element, tag: str) -> float | None:
    text = root.findtext(tag)
    if text:
        try:
            return float(text.strip())
        except ValueError:
            return None
    return None


def get_child_float(parent: ET.Element | None, tag: str, default: float) -> float:
    if parent is None:
        return default
    text = parent.findtext(tag)
    if text is None:
        return default
    try:
        return float(text.strip())
    except ValueError:
        return default


def get_all_values(joints: ET.Element, tag: str) -> np.ndarray:
    values = []
    for joint in joints.findall("joint"):
        element = joint.find(tag)
        if element is not None:
            values.append(_convert_to_float(element.text.strip()))
        else:
            values.append(np.nan)
    return np.array(values)


def get_all_strings(joints: ET.Element, tag: str, default: str | None = None) -> list[str | None]:
    """Per-joint STRING values (get_all_values coerces to float, which would turn
    a driver name into NaN). Missing entries become `default` — for driver_type
    that is None meaning "not declared", never a guessed product."""
    values = []
    for joint in joints.findall("joint"):
        element = joint.find(tag)
        text = (element.text or "").strip() if element is not None else ""
        values.append(text.lower() if text else default)
    return values


def get_gripper_values(grippers: ET.Element, gripper_name: str, tag: str) -> float | str:
    """
    Returns a single value from a named gripper (embedded <grippers> element).
    """
    gripper = grippers.find(f"./gripper[@name='{gripper_name}']")
    if gripper is None:
        raise ValueError(f"Gripper '{gripper_name}' not found")

    element = gripper.find(tag)
    if element is None or element.text is None:
        return np.nan

    return _convert_to_float(element.text.strip())


def load_gripper_element(grippers_dir: str | Path, gripper_name: str) -> ET.Element | None:
    """
    Parse a standalone gripper XML file from grippers_dir and return its root element.
    File is expected at: grippers_dir / f"{gripper_name}.xml"
    Returns None if the file does not exist (robot has no gripper).
    """
    path = Path(grippers_dir) / f"{gripper_name}.xml"
    if not path.exists():
        return None
    tree = ET.parse(str(path))
    return tree.getroot()


def get_gripper_value(gripper_elem: ET.Element | None, tag: str) -> float | str:
    """
    Return a single value from a standalone gripper element.
    Returns np.nan when the element or its text is missing.
    """
    if gripper_elem is None:
        return np.nan
    element = gripper_elem.find(tag)
    if element is None or element.text is None:
        return np.nan
    return _convert_to_float(element.text.strip())


def set_single_value(joints: ET.Element, joint_id: int, tag: str, value: object) -> None:
    joint = joints.find(f"./joint[@id='{joint_id}']")
    if joint is not None:
        element = joint.find(tag)
        if element is None:
            element = ET.SubElement(joint, tag)
        element.text = str(value)


def set_all_values(joints: ET.Element, tag: str, values: Iterable[object]) -> None:
    for i, joint in enumerate(joints.findall("joint")):
        if i >= len(values):
            break
        element = joint.find(tag)
        if element is None:
            element = ET.SubElement(joint, tag)
        element.text = str(values[i])


def save_robot_config(root: ET.Element, output_path: str | Path) -> None:
    tree = ET.ElementTree(root)
    tree.write(output_path, encoding='utf-8', xml_declaration=True)


def set_value(root: ET.Element, tag: str, value: object) -> None:
    element = root.find(tag)
    if element is None:
        element = ET.SubElement(root, tag)
    element.text = str(value)


def _convert_to_float(text: str) -> float | str:
    try:
        return float(text)
    except ValueError:
        return text
