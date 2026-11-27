"""MCAP recordings -> a LeRobot dataset (format v3), through LeRobot's own writer (TODO/p4_recording_plan.md 3.3 and
section 6 step 4, 2026-11-27).

    python -m robotclient.recording export recordings/2026-11-27_13-49-02 --out datasets/freedrive

Needs, on the PC: Python 3.12 or newer and ``pip install "lerobot[dataset]" mcap`` (LeRobot 0.7: PyTorch, and PyAV
with its own FFmpeg, enough to WRITE a dataset; Pillow decodes and resizes the JPEGs). The robot's Pi cannot run it
(its Python is 3.11). Nothing here is imported until an export runs: ``import robotclient.recording`` stays free of
third-party packages.

One dataset row per camera frame, at the camera's rate (30 fps); LeRobot puts each row at frame_index / fps. Per row:

================================  ==========  ===============================================================
column                            dtype       what
================================  ==========  ===============================================================
``observation.images.wrist``      video       the frame, decoded and resized (default 640x360), RGB
``observation.state``             float32 7   j1.pos .. j6.pos (rad) at the image's moment, each joint
                                              interpolated on its own measurement times (j_rx:
                                              robotclient.joint_times), and gripper.pos: g_p / 255 on g_rx (0 open,
                                              1 closed). The names follow LeRobot's convention (``.pos``), which its
                                              policy runner needs for the start pose and the hold.
``action``                        float32 7   by the episode's action source: freedrive (IDLE only) = the next
                                              row's state (the last frame has none: dropped); any other mode =
                                              what was commanded at that moment (j_pt, g_pt / 255)
``frame.time``                    float64 1   the image's moment, s from the episode's first row (the true
                                              spacing; LeRobot's own timestamp is frame_index / fps)
``frame.tick``                    float64 1   the robot's control-loop tick at the image's moment
``frame.calibrated``              bool 1      the image's moment includes the camera's calibrated offset
``frame.filled``                  bool 1      a missing frame, filled with the image before it
``episode.success``               int64 1     the operator's answer: 1 yes, 0 no, -1 not said
``episode.note``                  string      the operator's note
================================  ==========  ===============================================================

Each row's task is the episode's task. A gap between two exposures longer than FILL_PERIODS frame periods is filled
(each missing frame: the image before it, the state at its own moment, frame.filled); a gap longer than SPLIT_S
splits the episode in two. Frames outside the joints' measured span are dropped (that happens only at the ends). An
episode whose checks failed is left out unless `include_failed`. Next to LeRobot's files, ``meta/rcb_episodes.jsonl``
gets one line per dataset episode: its MCAP file, the part of it, the recorder's metadata, the export's settings.

`columns`, `state_at` and `image` are shared with the LeRobot plugin (lerobot_robot_par6), so that a policy's live
observations are made exactly as its training rows were.
"""
from __future__ import annotations

import glob
import io
import json
import os
import re

CAMERA = "wrist"
WIDTH, HEIGHT = 640, 360
CODECS = {"av1": "libsvtav1", "h264": "h264"}     # --codec -> LeRobot's RGBEncoderConfig(vcodec=...)
FILL_PERIODS = 1.5          # a gap longer than this many frame periods: the frames missing in it are filled
SPLIT_S = 0.2               # a gap longer than this (s) splits the episode
FREEDRIVE_MODES = ("IDLE",)  # an episode only in these modes is freedrive: its action = the next row's state
GRIP_SCALE = 255.0          # the gripper's own scale (g_p, g_pt): 0 open .. 255 closed
JOINTS = 6
NAMES = [f"j{i}.pos" for i in range(1, JOINTS + 1)] + ["gripper.pos"]   # LeRobot's convention (user 2026-11-27)
TAGS = (("timestamp", 1), ("tick", 1), ("j_p", JOINTS), ("j_rx", JOINTS), ("j_pt", JOINTS), ("g_p", 1), ("g_rx", 1),
        ("g_pt", 1))           # the packets' tags an export (and the plugin) reads, with their widths
SIDECAR = os.path.join("meta", "rcb_episodes.jsonl")
ROBOT_TYPE = "par6"
_TEXT_KEYS = {"task", "note", "operator", "session", "recorder_version", "start_utc", "end_utc", "time_base"}


def missing() -> str | None:
    """None when an export can run here, else what to install."""
    try:
        import mcap.reader  # noqa: F401
        import numpy  # noqa: F401
        import PIL.Image  # noqa: F401
        from lerobot.configs import RGBEncoderConfig  # noqa: F401
        from lerobot.datasets import LeRobotDataset  # noqa: F401
    except ImportError as e:
        return f'the LeRobot export needs Python 3.12+ and: pip install "lerobot[dataset]" mcap ({e})'
    return None


def features(width: int = WIDTH, height: int = HEIGHT) -> dict:
    """The dataset's columns (LeRobot's `features`: shapes as tuples, as its writer compares them)."""
    one = {"shape": (1,), "names": None}
    return {
        f"observation.images.{CAMERA}": {"dtype": "video", "shape": (height, width, 3),
                                         "names": ["height", "width", "channels"]},
        "observation.state": {"dtype": "float32", "shape": (len(NAMES),), "names": list(NAMES)},
        "action": {"dtype": "float32", "shape": (len(NAMES),), "names": list(NAMES)},
        "frame.time": {"dtype": "float64", **one},
        "frame.tick": {"dtype": "float64", **one},
        "frame.calibrated": {"dtype": "bool", **one},
        "frame.filled": {"dtype": "bool", **one},
        "episode.success": {"dtype": "int64", **one},
        "episode.note": {"dtype": "string", **one},
    }


def _meta_value(key: str, text: str):
    if key in _TEXT_KEYS:
        return text
    try:
        return json.loads(text)
    except ValueError:
        return text


def read_episode(path: str, robot: str | None = None) -> dict:
    """One MCAP episode's data for `robot` (default: the recording's first robot): {"robot", "meta": {"recording",
    "episode"} (JSON values decoded), "packets": {tag: rows x n array, "mode": [text]}, "frames": [{"info": dict,
    "jpeg": bytes}] in the order of their moments}.

    Raises:
        ValueError: no robot of that name in the file, or no packet of it.
    """
    from mcap.reader import make_reader

    from ..telemetry import decode_packet
    from . import cdr
    with open(path, "rb") as f:
        reader = make_reader(f)
        meta = {m.name: {k: _meta_value(k, v) for k, v in m.metadata.items()} for m in reader.iter_metadata()}
        sources = meta.get("recording", {}).get("sources") or []
        names = [s.get("name") for s in sources if isinstance(s, dict)]
        robot = robot or (names[0] if names else "arm1")
        if names and robot not in names:
            raise ValueError(f"no robot {robot!r} in {os.path.basename(path)}: it has {', '.join(map(str, names))}")
        topics = [f"/{robot}/telemetry", f"/{robot}/camera/{CAMERA}", f"/{robot}/camera/{CAMERA}/info"]
        rows, jpegs, infos = [], {}, {}
        for _schema, channel, msg in reader.iter_messages(topics=topics, log_time_order=False):
            if channel.topic == topics[0]:
                rows.append(decode_packet(msg.data))
            elif channel.topic == topics[1]:
                jpegs[msg.sequence] = cdr.decode_image(msg.data)[3]
            else:
                infos[msg.sequence] = json.loads(msg.data)
    if not rows:
        raise ValueError(f"no packet of {robot} in {os.path.basename(path)}")
    rows.sort(key=lambda d: d.get("tick", 0.0))
    packets = columns(rows)
    frames = sorted(({"info": infos[s], "jpeg": jpegs[s]} for s in infos if s in jpegs),
                    key=lambda fr: fr["info"].get("exposure_time", 0.0))
    return {"robot": robot, "path": path, "meta": meta, "packets": packets, "frames": frames}


def columns(rows) -> dict:
    """Decoded packets (dicts, in tick order) -> {tag: rows x width float array for each of TAGS (NaN where a packet
    lacks the tag), "mode": [each packet's mode]}."""
    import numpy as np
    out: dict = {}
    for tag, n in TAGS:
        a = np.full((len(rows), n), np.nan)
        for i, d in enumerate(rows):
            v = d.get(tag)
            if isinstance(v, (int, float)) and n == 1:
                a[i, 0] = v
            elif isinstance(v, (list, tuple)) and len(v) >= n:
                a[i, :] = v[:n]
        out[tag] = a
    out["mode"] = [d.get("mode") for d in rows]
    return out


def state_at(times, p: dict):
    """The state at each of `times` (the robot's clock, s) from `columns()`' arrays: j1..j6 (rad), each interpolated
    on its own measurement times (robotclient.joint_times), and the gripper (g_p / 255 on g_rx); NaN outside a joint's
    measured span. -> (len(times) x 7 array, {"joints": kind, "gripper": kind}): "measured", or "rows" when the
    packets carried no reply times (the rows' timestamps then stand in)."""
    import numpy as np

    from ..joint_times import REPLY_AFTER_MEASURE_S, joint_at
    jrx, kind_j = _reply_times(p, "j_rx", JOINTS)
    grx, kind_g = _reply_times(p, "g_rx", 1)
    # joint_at takes reply times: a row's timestamp is not one, so undo its measurement shift
    shift_j = REPLY_AFTER_MEASURE_S if kind_j == "rows" else 0.0
    shift_g = REPLY_AFTER_MEASURE_S if kind_g == "rows" else 0.0
    joints = joint_at(times, jrx + shift_j, p["j_p"])
    grip = joint_at(times, grx + shift_g, p["g_p"]) / GRIP_SCALE
    return np.hstack([joints, grip]), {"joints": kind_j, "gripper": kind_g}


def image(jpeg: bytes, width: int = WIDTH, height: int = HEIGHT):
    """A camera frame's JPEG -> RGB uint8, height x width x 3, resized with Pillow's bilinear filter."""
    import numpy as np
    from PIL import Image
    pic = Image.open(io.BytesIO(jpeg)).convert("RGB")
    return np.asarray(pic.resize((width, height), Image.Resampling.BILINEAR), dtype=np.uint8)


def action_source(ep: dict) -> str:
    """"freedrive" when the episode's robot was only in FREEDRIVE_MODES, else "commanded"."""
    src = (ep["meta"].get("episode", {}).get("action_source") or {}).get(ep["robot"]) or {}
    modes = src.get("modes") or [m for m in dict.fromkeys(ep["packets"]["mode"]) if isinstance(m, str)]
    return "freedrive" if modes and all(m in FREEDRIVE_MODES for m in modes) else "commanded"


def _reply_times(p: dict, reply: str, n: int):
    """The reply times to interpolate a channel on: the packets' own (j_rx / g_rx); a joint without any falls back to
    the rows' timestamps (its angles then 3-4.5 ms older than they say). -> (times, which kind)."""
    import numpy as np
    rx = p[reply].copy()
    kind = "measured"
    for j in range(n):
        if not np.isfinite(rx[:, j]).any():
            rx[:, j] = p["timestamp"][:, 0]
            kind = "rows"
    return rx, kind


def pair(ep: dict) -> dict:
    """The dataset rows of an episode: {"fps", "parts": [{"t", "tick", "calibrated", "filled", "frame", "state",
    "action"} arrays, one per part], "joint_times", "source"}. See the module's docstring for the rules."""
    import numpy as np
    p, frames = ep["packets"], ep["frames"]
    if not frames:
        return {"fps": 0, "parts": [], "joint_times": None, "source": action_source(ep)}
    t = np.array([fr["info"]["exposure_time"] for fr in frames], dtype=float)
    steps = np.diff(t)
    period = float(np.median(steps)) if steps.size else 1.0 / 30.0
    fps = max(1, int(round(1.0 / period)))
    groups: list = [[]]
    for i in range(len(frames)):
        if groups[-1]:
            gap = t[i] - t[i - 1]
            if gap > SPLIT_S:
                groups.append([])
            elif gap > FILL_PERIODS * period:
                k = int(round(gap / period)) - 1
                groups[-1] += [(t[i - 1] + m * gap / (k + 1), i - 1, True) for m in range(1, k + 1)]
        groups[-1].append((t[i], i, False))
    stamps = p["timestamp"][:, 0]
    source = action_source(ep)
    parts = []
    joint_times: dict = {}
    for group in groups:
        times = np.array([g[0] for g in group])
        idx = np.array([g[1] for g in group], dtype=int)
        filled = np.array([g[2] for g in group], dtype=bool)
        state, joint_times = state_at(times, p)
        held = np.searchsorted(stamps, times, side="right") - 1          # the packet in force at each moment
        ok_held = held >= 0
        target = np.full_like(state, np.nan)
        target[ok_held, :JOINTS] = p["j_pt"][held[ok_held]]
        target[ok_held, JOINTS] = p["g_pt"][held[ok_held], 0] / GRIP_SCALE
        tick = np.array([frames[i]["info"].get("rti_tick") for i in idx], dtype=float)
        tick[filled] = np.interp(times[filled], stamps, p["tick"][:, 0]) if filled.any() else tick[filled]
        calibrated = np.array([bool(frames[i]["info"].get("calibrated")) for i in idx], dtype=bool)
        keep = np.isfinite(state).all(axis=1)
        if source == "freedrive":
            action = np.vstack([state[1:], np.full((1, state.shape[1]), np.nan)]) if len(state) else state
            keep &= np.isfinite(action).all(axis=1)
        else:
            action = target
            keep &= np.isfinite(action).all(axis=1)
        if keep.any():                                  # only the ends can be dropped: keep the run between them
            first, last = int(np.argmax(keep)), len(keep) - 1 - int(np.argmax(keep[::-1]))
            sl = slice(first, last + 1)
        else:
            sl = slice(0, 0)
        parts.append({"t": times[sl], "tick": tick[sl], "calibrated": calibrated[sl], "filled": filled[sl],
                      "frame": idx[sl], "state": state[sl], "action": action[sl]})
    return {"fps": fps, "parts": parts, "joint_times": joint_times, "source": source}


def _files(paths) -> list:
    out: list = []
    for p in ([paths] if isinstance(paths, str) else paths):
        if os.path.isdir(p):
            out += sorted(glob.glob(os.path.join(p, "*.mcap")))
        elif p.endswith(".mcap"):
            out.append(p)
    return out


def _repo_id(out: str) -> str:
    return "local/" + (re.sub(r"[^A-Za-z0-9._-]+", "_", os.path.basename(os.path.abspath(out))) or "dataset")


def _success(value) -> int:
    return 1 if value is True else 0 if value is False else -1


def export(paths, out: str, robot: str | None = None, width: int = WIDTH, height: int = HEIGHT, codec: str = "av1",
           include_failed: bool = False, repo_id: str | None = None, progress=None) -> dict:
    """Export MCAP episodes (files, or folders of them: sessions) into the LeRobot dataset `out` (made when it does
    not exist, else appended to).

    Args:
        paths: .mcap files and / or folders holding them (each folder's in name order).
        out: The dataset's folder (an ASCII path: LeRobot 0.7 on Windows mishandles others when it appends videos).
        robot: Whose data (a recording's robot name); None: each recording's first robot.
        width, height: The images' size in the dataset.
        codec: "av1" (LeRobot's default) or "h264".
        include_failed: Also episodes whose checks failed (marked so in the sidecar).
        repo_id: The dataset's name for LeRobot; None: "local/<the folder's name>".
        progress: Called with one line of text per episode, or None.

    Returns:
        {"dataset", "created", "episodes", "rows", "skipped": [[file, why]], "written": [[file, part, index, rows]]}.

    Raises:
        RuntimeError: LeRobot (or mcap, Pillow) is not installed here: the message says what to install.
        ValueError: no .mcap file in `paths`, or an unknown codec.
    """
    why = missing()
    if why:
        raise RuntimeError(why)
    if codec not in CODECS:
        raise ValueError(f"no codec {codec!r}: {', '.join(CODECS)}")
    files = _files(paths)
    if not files:
        raise ValueError(f"no .mcap file in {paths}")
    import numpy as np
    from lerobot.configs import RGBEncoderConfig
    from lerobot.datasets import LeRobotDataset
    say = progress or (lambda text: None)
    encoder = RGBEncoderConfig(vcodec=CODECS[codec])
    ds = None
    created = False
    result: dict = {"dataset": out, "created": False, "episodes": 0, "rows": 0, "skipped": [], "written": []}
    sidecar: list = []
    try:
        for path in files:
            try:
                ep = read_episode(path, robot)
            except Exception as e:  # noqa: BLE001  a broken file (a crash's leftover, a bad copy) must not stop the others
                result["skipped"].append([path, str(e) or type(e).__name__])
                say(f"{os.path.basename(path)}: skipped ({e})")
                continue
            em = ep["meta"].get("episode", {})
            failed = em.get("checks_failed") or []
            if failed and not include_failed:
                result["skipped"].append([path, "checks failed: " + "; ".join(map(str, failed))])
                say(f"{os.path.basename(path)}: skipped (checks failed; --include-failed exports it)")
                continue
            rows = pair(ep)
            if not any(len(part["t"]) for part in rows["parts"]):
                result["skipped"].append([path, "no frame with the joints' data"])
                say(f"{os.path.basename(path)}: skipped (no frame with the joints' data)")
                continue
            if ds is None:
                if os.path.exists(out):
                    ds = LeRobotDataset.resume(repo_id or _repo_id(out), root=out, rgb_encoder=encoder)
                else:
                    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
                    ds = LeRobotDataset.create(repo_id or _repo_id(out), fps=rows["fps"], features=features(width, height),
                                               root=out, robot_type=ROBOT_TYPE, use_videos=True, rgb_encoder=encoder)
                    created = True
            ds_fps = getattr(ds, "fps", rows["fps"])
            if ds_fps != rows["fps"]:
                result["skipped"].append([path, f"its camera ran at {rows['fps']} fps, the dataset is {ds_fps} fps"])
                say(f"{os.path.basename(path)}: skipped ({rows['fps']} fps, the dataset {ds_fps})")
                continue
            task = em.get("task") or ep["meta"].get("recording", {}).get("task") or ""
            success = _success(em.get("success"))
            note = em.get("note") if isinstance(em.get("note"), str) else ""
            for k, part in enumerate(rows["parts"]):
                n = len(part["t"])
                if n == 0:
                    continue
                index = getattr(getattr(ds, "meta", None), "total_episodes", None)
                t0 = float(part["t"][0])
                last_frame, picture = -1, None
                for r in range(n):
                    fi = int(part["frame"][r])
                    if fi != last_frame:
                        picture = image(ep["frames"][fi]["jpeg"], width, height)
                        last_frame = fi
                    ds.add_frame({
                        f"observation.images.{CAMERA}": picture,
                        "observation.state": part["state"][r].astype(np.float32),
                        "action": part["action"][r].astype(np.float32),
                        "frame.time": np.array([float(part["t"][r]) - t0], dtype=np.float64),
                        "frame.tick": np.array([float(part["tick"][r])], dtype=np.float64),
                        "frame.calibrated": np.array([bool(part["calibrated"][r])], dtype=bool),
                        "frame.filled": np.array([bool(part["filled"][r])], dtype=bool),
                        "episode.success": np.array([success], dtype=np.int64),
                        "episode.note": note,
                        "task": task,
                    })
                ds.save_episode()
                result["episodes"] += 1
                result["rows"] += n
                result["written"].append([path, k, index, n])
                sidecar.append({"episode_index": index, "source": os.path.abspath(path), "robot": ep["robot"],
                                "part": k, "parts": len(rows["parts"]), "rows": n, "action_source": rows["source"],
                                "joint_times": rows["joint_times"], "checks_failed": failed,
                                "export": {"width": width, "height": height, "codec": codec, "fps": rows["fps"]},
                                "recording": ep["meta"].get("recording", {}), "episode": em})
                part_text = f" part {k + 1} of {len(rows['parts'])}" if len(rows["parts"]) > 1 else ""
                say(f"{os.path.basename(path)}{part_text}: dataset episode {index}, {n} rows ({rows['source']})")
    finally:
        if ds is not None:
            ds.finalize()
            if sidecar:
                os.makedirs(os.path.join(out, "meta"), exist_ok=True)
                with open(os.path.join(out, SIDECAR), "a", encoding="utf-8") as f:
                    for line in sidecar:
                        f.write(json.dumps(line, default=str) + "\n")
    result["created"] = created
    return result
