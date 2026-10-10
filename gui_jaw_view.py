"""Live CAD view of the gripper's jaws, for the dashboard's Gripper tab (2026-10-08).

The user: "connect the mapping I have from motor and firmware modes to the CAD so that I can confirm and measure that it is
correct". This shows the gripper's own CAD meshes (Source Robotics' PAR6 description package — the meshes the collision
pieces are fitted to) with the jaws placed EXACTLY as the collision worker places them for <jaws>measured</jaws>:
self_collision_worker.jaw_closed_fraction on the live report (firmware mode's 0-255, or motor mode's opening in metres),
each jaw moved that fraction of its stroke from the open end (URDF 0 — the fit finds it from the meshes and the nets pin
it) toward closed. Beside it: the report, the fraction, and the gap between the jaws' inner faces that it means — open the
real gripper to any opening and compare with a ruler; and what the worker uses NOW (measured / swept).

Reads shared memory only; the meshes are served from the package folder (`project_paths.PAR6_DESCRIPTION_DIR`). The jaws
move only when the fraction changes (5 checks a second).
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from project_paths import PAR6_DESCRIPTION_DIR

# the gripper's collision file -> its URDF in the package
URDF_OF = {"MSG_collision.json": "par6_msg.urdf", "SSG48_collision.json": "par6_ssg48.urdf"}
# the CAD's gap between the jaws' inner faces when fully OPEN (URDF 0), from the package's meshes (2026-10-07): MSG jaw1 /
# jaw2 inner faces at y = -52.1 / +52.1 mm; SSG48 fingertips at -23.7 / +23.7 mm. api_tests/62 uses the same numbers.
OPEN_GAP_MM = {"MSG_collision.json": 104.2, "SSG48_collision.json": 47.4}
MESH_URL = "/par6_cad"                     # where the dashboard serves <package>/meshes


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    """URDF rpy -> rotation: Rz(y) Ry(p) Rx(r)."""
    cx, sx, cy, sy, cz, sz = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1.0]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))


def cad_for(collision_file: str | None, package_dir: str = str(PAR6_DESCRIPTION_DIR)) -> dict | None:
    """The gripper's CAD in its own (tool link) frame, from the package URDF: {"body": mesh path under meshes/, "jaws":
    [{"mesh", "R" (3x3), "t" (3,), "axis" (3,), "far" (signed far limit, m)}], "stroke_m", "open_gap_mm"}; None for a
    gripper without jaws here (the flange, a gripper without a CAD)."""
    name = URDF_OF.get(collision_file or "")
    if name is None:
        return None
    root = ET.parse(os.path.join(package_dir, name)).getroot()
    links = {l.get("name"): l for l in root.findall("link")}

    def mesh_of(link):
        m = links[link].find("visual/geometry/mesh")
        return m.get("filename").split("meshes/", 1)[1]

    jaws, tool = [], None
    for j in root.findall("joint"):
        if j.get("type") != "prismatic":
            continue
        tool = j.find("parent").get("link")
        o = j.find("origin")
        lo, hi = float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))
        jaws.append({"mesh": mesh_of(j.find("child").get("link")),
                     "R": _rpy(*[float(v) for v in o.get("rpy", "0 0 0").split()]),
                     "t": np.array([float(v) for v in o.get("xyz", "0 0 0").split()]),
                     "axis": np.array([float(v) for v in j.find("axis").get("xyz").split()]),
                     "far": lo if abs(lo) > abs(hi) else hi})
    if tool is None or not jaws:
        return None
    return {"body": mesh_of(tool), "jaws": jaws, "stroke_m": abs(jaws[0]["far"]), "open_gap_mm": OPEN_GAP_MM[collision_file]}


def jaw_pose(jaw: dict, closed: float) -> tuple[np.ndarray, np.ndarray]:
    """(R, t) of a jaw mesh in the tool link frame at a closing fraction (0 open .. 1 closed): the URDF's prismatic joint
    at closed x its far limit (open at URDF 0 — the worker's mapping)."""
    return jaw["R"], jaw["t"] + jaw["R"] @ (jaw["axis"] * (closed * jaw["far"]))


def inner_gap_mm(cad: dict, closed: float) -> float:
    """The gap between the jaws' inner faces at a closing fraction, by the CAD (0 when they touch)."""
    return max(0.0, cad["open_gap_mm"] - 2.0 * closed * 1e3 * cad["stroke_m"])


def report_text(rti, gripper, joint_num: int, homed_state: str, motor_ok: bool, closed, cad: dict) -> str:
    """The lines beside the view (pure, for the nets). closed: jaw_closed_fraction's answer (None = not usable)."""
    mode = int(rti["gripper_ctrl_mode"][0])
    if mode == 2:
        rep = f"firmware mode: {int(gripper['gripper_position'][0])} / 255 (0 open, 255 closed)"
    elif mode == 1:
        rep = (f"motor mode: {1e3 * float(gripper['gripper_joint_position'][0]):.1f} mm open of "
               f"{1e3 * float(gripper['gripper_stroke_m'][0]):.0f}")
    else:
        rep = "no CAN gripper"
    if closed is None:
        why = ("data stale" if int(rti["old_data_warning"][joint_num]) != 0 else
               "not calibrated" if mode == 2 else
               ("homes toward its closed end" if not motor_ok else "the robot or the gripper is not homed, or the opening is outside the stroke")
               if mode == 1 else "no report")
        lines = [f"Gripper report: {rep}", f"NOT USABLE ({why}) — the jaws are shown fully open"]
    else:
        lines = [f"Gripper report: {rep}", f"→ jaws {100 * closed:.0f} % closed (the collision worker's own mapping)",
                 f"CAD gap between the jaws' inner faces: {inner_gap_mm(cad, closed):.1f} mm "
                 f"(open {cad['open_gap_mm']:.1f}, closed 0) — compare with a ruler"]
    w = float(rti["sc_jaw_closed"][0])
    lines.append("Collision worker now: " + (f"MEASURED jaws, {100 * w:.0f} % closed" if w >= 0
                                             else "SWEPT jaws (the whole stroke — <jaws>swept</jaws>, or the report not usable)"))
    return "\n".join(lines)


def add_jaw_view(shared_data, cfg: dict, joint_num: int):
    """The card for the Gripper tab: the 3D view + the report. Returns its refresh function (None without jaws)."""
    from nicegui import app, ui
    from self_collision_worker import jaw_closed_fraction
    cad = cad_for((cfg.get("gripper_collision") or {}).get("file"))
    with ui.card().classes("w-full mb-3"):
        ui.label("Jaws: the CAD where the collision check puts them").classes(
            "text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
        if cad is None:
            ui.label("This gripper has no CAD jaws here (only the MSG and the SSG48 do).").classes("text-sm text-gray-500")
            return None
        if MESH_URL not in [r.path for r in app.routes if hasattr(r, "path")]:
            app.add_static_files(MESH_URL, os.path.join(str(PAR6_DESCRIPTION_DIR), "meshes"))
        hd = cfg.get("gripper_homing_direction")
        motor_ok = hd is not None and hd == hd and int(hd) == 0
        with ui.row().classes("items-start gap-4 w-full"):
            with ui.scene(width=420, height=320, grid=(0.4, 20), background_color="#f5f6f8") as scene:
                scene.stl(f"{MESH_URL}/{cad['body']}").material("#9aa6b4")
                jaw_objs = [scene.stl(f"{MESH_URL}/{j['mesh']}").material("#c9a227") for j in cad["jaws"]]
            info = ui.label("").classes("font-mono text-sm whitespace-pre-wrap")
        scene.move_camera(x=0.30, y=-0.26, z=0.02, look_at_x=-0.01, look_at_y=0.0, look_at_z=-0.10, duration=0)
    state = {"c": None, "text": None}

    def refresh():
        rti, grip = shared_data.rti, shared_data.gripper
        hs = shared_data.homed_state.get()
        c = jaw_closed_fraction(rti, grip, joint_num, hs, motor_ok)
        shown = 0.0 if c is None else c
        if state["c"] is None or abs(shown - state["c"]) > 1e-3:
            for obj, jw in zip(jaw_objs, cad["jaws"]):
                R, t = jaw_pose(jw, shown)
                obj.rotate_R(R.tolist())
                obj.move(*[float(v) for v in t])
            state["c"] = shown
        text = report_text(rti, grip, joint_num, hs, motor_ok, c, cad)
        if text != state["text"]:
            info.set_text(text)
            state["text"] = text

    ui.timer(0.2, refresh)
    return refresh
