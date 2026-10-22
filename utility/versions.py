"""Every part's version in one report (2026-10-21; versioning step V1).

    runtime_version(root=PROJECT_ROOT) -> "0.9.0" | "unknown"     the VERSION file at the repository root
    static_report(root=PROJECT_ROOT)   -> dict                     the runtime, Python, the OS, the description package
                                                                   (+ the robot XML's minimum, V4), the protocols'
                                                                   versions (V3), the arm's / gripper's hardware revision
                                                                   and driver types (V4, robot_hardware());
                                                                   built once per process, then cached
    report(shared_data, joint_num, has_gripper, pcb_version) -> dict
                                                                   static_report + what the drives, the gripper and the
                                                                   control box report now (shared memory)

The runtime's version lives in ONE place, the VERSION file (`MAJOR.MINOR.PATCH`); pyproject.toml reads the same file.
PATCH: nothing a program or a configuration file can see changed. MINOR: new commands, fields or keys, backward
compatible. MAJOR: a protocol or a file format broke.

The drives, the gripper and the control box report their own versions (CAN, UART) into shared memory; this module only
reads it. No subprocess and no `git` child: RTI may call runtime_version after it has taken its real-time priority.
A value a part has not reported yet (shared memory starts at 0, the CAN layer at -1) is None.
"""
from __future__ import annotations

import copy
import os
import platform
import re
import time
from typing import Any

from communication.protocol import PROTOCOL_VERSION
from communication.rti_protocol import RTI_PROTOCOL_VERSION
from config.schema_versions import report as config_formats_report
from hardware import firmware_support
from project_paths import GRIPPERS_DIR, PAR6_DESCRIPTION_DIR, PROJECT_ROOT, ROBOTS_DIR, SYSTEM_XML
from utility.code_version import git_commit
from utility.serialization import TELEMETRY_FORMAT

# The three protocols' versions (versioning V3): what the executor publishes in every status message and what
# get_versions reports. A client refuses a robot whose number of a protocol it speaks differs from its own.
PROTOCOLS = {"commands": PROTOCOL_VERSION, "rti": RTI_PROTOCOL_VERSION, "telemetry": TELEMETRY_FORMAT}

_SEMVER = re.compile(r"^\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.\-+]+)?$")
_static_cache: dict[str, dict] = {}


def _read(path) -> str | None:
    try:
        with open(path, errors="replace") as fh:
            return fh.read()
    except OSError:
        return None


def runtime_version(root=PROJECT_ROOT) -> str:
    """The runtime's version from `<root>/VERSION`: "0.9.0", or "unknown" when the file is missing, empty or not
    MAJOR.MINOR.PATCH (with an optional -suffix / +build). Never raises."""
    text = _read(os.path.join(str(root), "VERSION"))
    if text is None:
        return "unknown"
    text = text.strip()
    return text if _SEMVER.match(text) else "unknown"


def description_package_version(pkg_dir=PAR6_DESCRIPTION_DIR) -> str | None:
    """The PAR6 description package's version: `<version>` in its package.xml, else the first x.y.z in its
    CHANGELOG.md's first heading. None when neither says (or the package is not on this machine)."""
    xml = _read(os.path.join(str(pkg_dir), "package.xml"))
    if xml:
        m = re.search(r"<version>\s*([^<\s]+)\s*</version>", xml)
        if m:
            return m.group(1)
    log = _read(os.path.join(str(pkg_dir), "CHANGELOG.md"))
    if log:
        for line in log.splitlines():
            if line.startswith("## "):
                m = re.search(r"\b(\d+\.\d+\.\d+)\b", line)
                return m.group(1) if m else None
    return None


def version_tuple(text) -> tuple[int, ...] | None:
    """"2.0.0" -> (2, 0, 0), "v1.2" -> (1, 2); None when the text starts with no number."""
    m = re.match(r"^\s*v?(\d+(?:\.\d+)*)", str(text or ""))
    return tuple(int(p) for p in m.group(1).split(".")) if m else None


def at_least(version, minimum) -> bool | None:
    """version >= minimum, comparing the numbers ("2.0.0" >= "2.0" is True); None when either is unknown."""
    a, b = version_tuple(version), version_tuple(minimum)
    if a is None or b is None:
        return None
    width = max(len(a), len(b))
    return a + (0,) * (width - len(a)) >= b + (0,) * (width - len(b))


def robot_hardware(system_xml=SYSTEM_XML, robots_dir=ROBOTS_DIR, grippers_dir=GRIPPERS_DIR) -> dict:
    """What the active robot and gripper XMLs declare about their hardware (versioning V4): {"robot", "revision",
    "driver_types" (per joint), "gripper", "gripper_revision", "gripper_driver_type", "description_package_min"}. A
    value a file does not declare is None. Never raises: a file it cannot read adds "error" with the reason."""
    out: dict[str, Any] = {"robot": None, "revision": None, "driver_types": [], "gripper": None,
                           "gripper_revision": None, "gripper_driver_type": None, "description_package_min": None}
    try:
        from config.xml_parser.system import load_system_config
        from config.xml_parser.xml_access import get_all_strings, load_gripper_element, parse_robot_config
        sysc = load_system_config(system_xml, robots_dir)
        root, joints = parse_robot_config(sysc["robot_xml_path"])
        out["robot"] = sysc.get("active_robot")
        out["revision"] = (root.findtext("hardware_revision") or "").strip() or None
        dp = root.find("description_package")
        if dp is not None:
            out["description_package_min"] = (dp.get("min") or "").strip() or None
        if joints is not None:
            out["driver_types"] = list(get_all_strings(joints, "driver_type"))
        gripper = (sysc.get("active_gripper") or "").strip()
        if gripper and gripper.lower() != "none":
            out["gripper"] = gripper
            el = load_gripper_element(grippers_dir, gripper)
            if el is not None:
                out["gripper_revision"] = (el.findtext("hardware_revision") or "").strip() or None
                out["gripper_driver_type"] = (el.findtext("driver_type") or "").strip().lower() or None
    except Exception as exc:                        # noqa: BLE001 — a report must never take its caller down
        out["error"] = str(exc)
    return out


def _firmware_verdict(entry: dict, driver_type) -> None:
    """Adds the drive's driver type and the verdict on its firmware (hardware/firmware_support.py) to a report entry."""
    entry["driver_type"] = driver_type
    entry["supported"] = firmware_support.check(driver_type, entry.get("firmware"))
    entry["supported_range"] = firmware_support.range_text(driver_type)


def os_info() -> dict:
    """{"name": "Ubuntu 24.04.2 LTS", "kernel": "6.8.4-rt11-raspi", "realtime": True | False | None}."""
    name = None
    rel = _read("/etc/os-release") or ""
    m = re.search(r'^PRETTY_NAME="?([^"\n]*)"?', rel, re.M)
    if m:
        name = m.group(1)
    rt = (_read("/sys/kernel/realtime") or "").strip()
    return {"name": name, "kernel": platform.release(), "realtime": (rt == "1") if rt else None}


def static_report(root=PROJECT_ROOT) -> dict:
    """What does not change while the process runs; computed once per root, then a copy of the cached dict."""
    key = str(root)
    if key not in _static_cache:
        branch, sha = git_commit(root)
        hw = robot_hardware()
        pkg = description_package_version()
        _static_cache[key] = {
            "runtime": {"version": runtime_version(root), "branch": branch, "commit": sha},
            "python": platform.python_version(),
            "os": os_info(),
            # + the oldest version the robot XML accepts (V4) and whether the installed one is at least that.
            "description_package": {"version": pkg, "path": str(PAR6_DESCRIPTION_DIR),
                                    "min": hw["description_package_min"],
                                    "ok": at_least(pkg, hw["description_package_min"])},
            "protocols": dict(PROTOCOLS),
            "hardware": hw,                          # the arm's / gripper's revisions and driver types (V4)
        }
    return copy.deepcopy(_static_cache[key])


def _reported(table, field, i=0):
    """A version number from shared memory, or None when the part has not reported it (<= 0) or the field is absent
    (an older shared-memory layout)."""
    try:
        v = int(table[field][i])
    except (KeyError, IndexError, TypeError, ValueError):
        return None
    return v if v > 0 else None


def report(shared_data, joint_num, has_gripper, pcb_version) -> dict:
    """static_report() plus what the hardware reports now. Pure reads; microseconds after the first call."""
    out = static_report()
    motor = getattr(shared_data, "motor", None) or {}
    out["drives"] = [{"joint": i + 1, "can_id": i,
                      "firmware": _reported(motor, "SOFTWARE_VERSION", i),
                      "hardware": _reported(motor, "HARDWARE_VERSION", i),
                      "serial": _reported(motor, "SERIAL_NUMBER", i),
                      "batch": _reported(motor, "BATCH_DATE", i)} for i in range(int(joint_num))]
    gripper = getattr(shared_data, "gripper", None) or {}
    out["gripper"] = ({"can_id": int(joint_num),
                       "firmware": _reported(gripper, "gripper_motor_sw_version"),
                       "hardware": _reported(gripper, "gripper_motor_hw_version"),
                       "serial": _reported(gripper, "gripper_motor_serial")} if has_gripper else None)
    board = getattr(shared_data, "control_board", None) or {}
    out["control_box"] = {"pcb_configured": int(pcb_version) if pcb_version is not None else None,
                          "hardware": _reported(board, "cb_HARDWARE_VERSION"),
                          "software": _reported(board, "cb_SOFTWARE_VERSION"),
                          "serial": _reported(board, "cb_SERIAL_NUMBER"),
                          "batch": _reported(board, "cb_BATCH_DATE")}
    # Each drive's firmware against what this runtime supports for its driver type (V4; hardware/firmware_support.py).
    types = out["hardware"].get("driver_types") or []
    for d in out["drives"]:
        _firmware_verdict(d, types[d["joint"] - 1] if d["joint"] - 1 < len(types) else None)
    if out["gripper"] is not None:
        _firmware_verdict(out["gripper"], out["hardware"].get("gripper_driver_type"))
    # The configuration file formats this runtime reads, and the format of each file this process read (V2).
    out["config_formats"] = config_formats_report()
    out["time"] = time.time()
    return out


def summary_line(rep: dict) -> str:
    """One line for a log or a header: 'runtime 0.9.0 (feat/x @ 80d9701c9a2b) · drives fw 103 · PCB v2 · package 2.0.0'.
    Drives that disagree are listed per joint ('fw J1 103, J2 109, ...')."""
    rt = rep.get("runtime", {})
    sha = (rt.get("commit") or "")[:12]
    parts = [f"runtime {rt.get('version', 'unknown')}" + (f" ({rt.get('branch') or 'detached'} @ {sha})" if sha else "")]
    fws = [d.get("firmware") for d in rep.get("drives", [])]
    if fws:
        known = {f for f in fws if f is not None}
        if len(known) == 1 and None not in fws:
            parts.append(f"drives fw {known.pop()}")
        else:
            parts.append("drives fw " + ", ".join(f"J{i + 1} {f if f is not None else '?'}" for i, f in enumerate(fws)))
    cb = rep.get("control_box") or {}
    if cb.get("pcb_configured") is not None:
        parts.append(f"PCB v{cb['pcb_configured']}")
    pkg = (rep.get("description_package") or {}).get("version")
    if pkg:
        parts.append(f"package {pkg}")
    return " · ".join(parts)
