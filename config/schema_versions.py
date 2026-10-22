"""Which format of each configuration file this runtime reads (2026-10-21; versioning step V2).

A file states its format on its root element: <robot schema="1">. A missing attribute means format 1, the format every
file had before formats were numbered, so no existing file needs editing. This runtime reads each kind up to
CURRENT[kind]:

    the file's format is newer  -> SchemaError: the file was written for a newer runtime and this one does not know what
                                   its keys mean. Each loader treats it exactly as it treats a broken file of its kind:
                                   the system, robot, gripper and homing files stop the process at start, a cell file is
                                   refused (ZONE_CONFIG), a filters file falls back to the default filters, a recipe is
                                   skipped; the reason is logged in every case.
    the file's format is older  -> read; the parser's defaults fill what it lacks; one WARNING per file and process.
    not a whole number >= 1     -> SchemaError.

Bump CURRENT[kind] only for a BREAKING change of that kind, in the same commit as the parser change, and say so in the
release notes: an element or attribute renamed or removed, a key that changes its meaning or its unit, a changed
default, a key that becomes required. A new OPTIONAL key is not a new format: the parsers look keys up by name and
ignore what they do not know, and they have a default for a missing key (config/xml_parser/robot.py, the
_fill_*_defaults functions). Exception: the cell parser (config/xml_parser/zones.py) refuses unknown elements and
attributes, so an older runtime already refuses a cell file with a new key; a zone key is still not a bump.
The first bump of a kind must also make its older files read CORRECTLY, not only allowed: either the parser keeps
reading the old layout, or (preferred) an upgrade step turns an old root element into the current layout before the
parser sees it, one small step per format, so the parsers only ever handle the current format.

No file I/O here: the loaders hand in the parsed root element.
"""
from __future__ import annotations

import logging

logger = logging.getLogger("config")  # docs/logging.md "Logger names"

# The newest format of each kind this runtime reads (and writes).
CURRENT: dict[str, int] = {
    "system": 1,     # config/system.xml
    "robot": 1,      # robots/<robot>.xml
    "gripper": 1,    # grippers/<gripper>.xml
    "homing": 1,     # config/<robot>_homing.xml
    "cell": 1,       # config/cells/<cell>.xml
    "filters": 1,    # config/filters.xml
    "recipe": 1,     # data_recipe/<recipe>.xml
}

_loaded: dict[str, dict] = {}      # file name -> {"kind", "schema"}: what this process has read (report())
_warned: set[str] = set()          # files already warned about (an older format), once per process


class SchemaError(ValueError):
    """A configuration file in a format this runtime cannot read: newer than CURRENT, or not a format number.
    A ValueError, so every loader's existing handling of a broken file applies to it."""


def check_schema(root, kind: str, source) -> int:
    """The format of a parsed configuration file (`root` = its root element). Raises SchemaError when this runtime
    cannot read it (see the module text); warns once for an older format. `source` names the file in the messages and
    in report()."""
    if kind not in CURRENT:
        raise KeyError(f"unknown configuration kind {kind!r}")       # a programming error, not a file's
    name = str(source)
    raw = root.get("schema")
    if raw is None:
        fmt = 1
    else:
        try:
            fmt = int(str(raw).strip())
        except ValueError:
            fmt = 0
        if fmt < 1:
            raise SchemaError(f"{name}: schema={raw!r} on <{root.tag}> is not a format number (a whole number, 1 or "
                              f"more)")
    reads = CURRENT[kind]
    if fmt > reads:
        raise SchemaError(f"{name}: a {kind} file in format {fmt}; this runtime reads {kind} files up to format {reads}. "
                          f"The file was written for a newer runtime: update the runtime, or use a file written for "
                          f"this one.")
    if fmt < reads and name not in _warned:
        _warned.add(name)
        logger.warning("%s: a %s file in format %d, older than this runtime's %d; read with the defaults for what it "
                       "lacks", name, kind, fmt, reads)
    _loaded[name] = {"kind": kind, "schema": fmt}
    return fmt


def report() -> dict:
    """{"reads": {kind: newest format}, "files": {file: {"kind", "schema"}}} — for utility/versions.py."""
    return {"reads": dict(CURRENT), "files": {k: dict(v) for k, v in _loaded.items()}}
