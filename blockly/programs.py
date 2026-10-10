"""Block programs saved on the robot: one JSON file each in programs/ — TODO/blockly_plan.md §4, D8 (local only).

    {"format": 1, "name": "pick_place", "robot": "PAR6", "gripper": "SSG48", "modified": "2026-10-04T12:00:00Z",
     "points": {"above_pick": {"joints_deg": [6 numbers], "pose": [x, y, z (m), rx, ry, rz (rad)]}},
     "workspace": {<Blockly.serialization.workspaces.save()>}}

The workspace is the source of truth; the Python is generated from it whenever it is needed and never stored.
A point keeps BOTH the joint angles (what a joint move uses: no IK, no surprise arm configuration) and the TCP pose
(what a straight-line move uses), captured together from the arm; `pose` is in the units the robot API uses.

Limits checked here (everything a file can carry is checked when it is SAVED, so a file that loads is sane):
- nesting depth: Python's json gives up near 1000 levels and Blockly nests each next block inside the one before,
  so one stack of more than about 400 blocks cannot be stored — refused with that number (MAX_JSON_DEPTH).
- names: program names become file names (no path tricks), point names are short labels.
"""
import json
import math
import os
import re
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FORMAT = 1
MAX_JSON_DEPTH = 900                # json.dumps / loads hit RecursionError near 1000 (measured 2026-10-04)
MAX_FILE_BYTES = 4_000_000
MAX_POINTS = 500
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _-]{0,63}$")
POINT_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _-]{0,31}$")


class ProgramError(Exception):
    """A program that cannot be stored or read; the message is for the user."""


def programs_dir():
    """programs/ in the repo, or RCB_PROGRAMS_DIR (the nets use their own)."""
    return os.environ.get("RCB_PROGRAMS_DIR") or os.path.join(ROOT, "programs")


def program_path(name):
    check_name(name)
    return os.path.join(programs_dir(), name + ".json")


def check_name(name):
    if not isinstance(name, str) or not NAME_RE.match(name) or name != name.strip():
        raise ProgramError("a program name is 1-64 letters, digits, spaces, '_' or '-', starting with a letter or digit")


def list_programs():
    """[{"name", "modified", "bytes"}], by name. Files that are not programs are skipped."""
    out = []
    folder = programs_dir()
    if not os.path.isdir(folder):
        return out
    for fn in sorted(os.listdir(folder)):
        name, ext = os.path.splitext(fn)
        if ext != ".json" or not NAME_RE.match(name):
            continue
        st = os.stat(os.path.join(folder, fn))
        out.append({"name": name, "modified": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(st.st_mtime)),
                    "bytes": st.st_size})
    return out


def load_program(name):
    path = program_path(name)
    try:
        with open(path) as f:
            raw = f.read(MAX_FILE_BYTES + 1)
    except FileNotFoundError:
        raise ProgramError(f"there is no program {name!r}") from None
    return parse_program(raw, name)


def parse_program(raw, name=None):
    """A program from its JSON text, checked. Raises ProgramError."""
    if len(raw) > MAX_FILE_BYTES:
        raise ProgramError("the program file is too large")
    try:
        program = json.loads(raw)
    except RecursionError:
        raise ProgramError(_too_deep()) from None
    except ValueError as e:
        raise ProgramError(f"the program file is not valid JSON ({e})") from None
    return check_program(program, name)


def save_program(name, program):
    """Check and write atomically (a temporary file, then a rename: a crash never leaves half a program)."""
    return write_program_file(program_path(name), dict(program, name=name), name)


def write_program_file(path, program, name=None):
    """Check `program` and write it atomically to `path` — save_program's writer, and the server's for a program it
    builds itself ("Move here")."""
    program = check_program(program, name)
    program["format"] = FORMAT
    program["modified"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    try:
        text = json.dumps(program, indent=1)
    except RecursionError:
        raise ProgramError(_too_deep()) from None
    if len(text) > MAX_FILE_BYTES:
        raise ProgramError("the program is too large to store")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    return program


def delete_program(name):
    try:
        os.remove(program_path(name))
    except FileNotFoundError:
        raise ProgramError(f"there is no program {name!r}") from None


def check_program(program, name=None):
    """The program, checked: its shape, its points, its depth. Raises ProgramError."""
    if not isinstance(program, dict):
        raise ProgramError("a program is a JSON object")
    if program.get("format", FORMAT) != FORMAT:
        raise ProgramError(f"program format {program.get('format')!r} is not one this editor reads ({FORMAT})")
    if name is not None:
        check_name(name)
        program["name"] = name
    workspace = program.get("workspace", {"blocks": {"languageVersion": 0, "blocks": []}})
    if not isinstance(workspace, dict):
        raise ProgramError("the program has no block workspace")
    if json_depth(workspace) > MAX_JSON_DEPTH:
        raise ProgramError(_too_deep())
    program["workspace"] = workspace
    program["points"] = check_points(program.get("points", {}))
    for key in ("robot", "gripper"):
        if program.get(key) is not None and not isinstance(program[key], str):
            raise ProgramError(f"'{key}' must be a name")
    return program


def check_points(points):
    if not isinstance(points, dict):
        raise ProgramError("the points must be a table")
    if len(points) > MAX_POINTS:
        raise ProgramError(f"more than {MAX_POINTS} points")
    out = {}
    for pname, point in points.items():
        check_point_name(pname)
        if not isinstance(point, dict):
            raise ProgramError(f"point {pname!r} is damaged")
        out[pname] = {"joints_deg": _numbers(point.get("joints_deg"), 6, f"point {pname!r}: joints"),
                      "pose": _numbers(point.get("pose"), 6, f"point {pname!r}: pose")}
    return out


def check_point_name(pname):
    if not isinstance(pname, str) or not POINT_RE.match(pname) or pname != pname.strip():
        raise ProgramError("a point name is 1-32 letters, digits, spaces, '_' or '-', starting with a letter or digit")


def json_depth(value):
    """Nesting depth of a JSON value, without recursion (it must work on values json itself cannot write)."""
    deepest = 0
    todo = [(value, 1)]
    while todo:
        v, d = todo.pop()
        if isinstance(v, dict):
            deepest = max(deepest, d)
            todo.extend((x, d + 1) for x in v.values())
        elif isinstance(v, list):
            deepest = max(deepest, d)
            todo.extend((x, d + 1) for x in v)
    return deepest


def _numbers(values, n, what):
    ok = isinstance(values, list) and len(values) == n
    if ok:
        for v in values:
            try:
                ok = ok and not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
            except OverflowError:
                ok = False
    if not ok:
        raise ProgramError(f"{what} must be {n} numbers")
    return [float(v) for v in values]


def _too_deep():
    return ("the program is nested too deeply to store: one stack holds at most about 400 blocks — "
            "split it with loops or more programs")
