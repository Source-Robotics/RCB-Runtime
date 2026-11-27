"""The `lerobot` recording format (``--format lerobot``, 2026-11-27, TODO/p4_recording_plan.md 3.1 / step 4): each
episode goes into one LeRobot dataset per session, ``<session>/lerobot``.

While recording, an episode is written to a temporary MCAP file next to the session's episodes
(``episode_NNN.lerobot.mcap``, through the `mcap` format: crash safe). When the episode is kept, that file is exported
into the dataset (robotclient.recording.export: the same code and the same rows as a later
``python -m robotclient.recording export``) and deleted; the next episode waits for the conversion (seconds). When
the export fails, the temporary file stays and the episode's answer names the error, so nothing is lost. A deleted
episode only drops its temporary file. An episode kept although a check failed (the k key) goes in too, its failed
checks in the dataset's sidecar (meta/rcb_episodes.jsonl). With ``--format mcap,lerobot`` the raw episode is kept as
well (recommended: another export can be made from it later, with other choices).

Needs what the export needs (Python 3.12+, ``pip install "lerobot[dataset]" mcap``). The dataset is of the
recording's FIRST robot (a dataset with two arms in one state vector: later). `delete_last()` (the r key) does not
apply: the episode is in the dataset by then.
"""
from __future__ import annotations

import os

from . import export, mcap_format

NAME = "lerobot"
EXTENSION = ".lerobot.mcap"           # the temporary MCAP; the episode itself goes into <session>/lerobot
DATASET = "lerobot"                   # the session's dataset folder


def missing() -> str | None:
    """None when this format can be written here, else what to install."""
    return mcap_format.missing() or export.missing()


def open_episode(path: str, info: dict) -> dict:
    """Start the episode's temporary MCAP at `path`; returns the handle."""
    sources = info.get("sources") or []
    robot = sources[0].get("name") if sources and isinstance(sources[0], dict) else None
    return {"mcap": mcap_format.open_episode(path, info), "tmp": path, "robot": robot,
            "dataset": os.path.join(os.path.dirname(path), DATASET)}


def add_packet(h: dict, robot: str, log_ns: int, arrival_ns: int, raw: bytes, decoded: dict) -> None:
    mcap_format.add_packet(h["mcap"], robot, log_ns, arrival_ns, raw, decoded)


def add_frame(h: dict, robot: str, camera: str, log_ns: int, arrival_ns: int, frame) -> None:
    mcap_format.add_frame(h["mcap"], robot, camera, log_ns, arrival_ns, frame)


def add_clock(h: dict, robot: str, log_ns: int, sample: dict) -> None:
    mcap_format.add_clock(h["mcap"], robot, log_ns, sample)


def add_status(h: dict, robot: str, log_ns: int, status: dict) -> None:
    mcap_format.add_status(h["mcap"], robot, log_ns, status)


def add_event(h: dict, robot: str, log_ns: int, event: dict) -> None:
    mcap_format.add_event(h["mcap"], robot, log_ns, event)


def close_episode(h: dict, meta: dict) -> str:
    """Finish the temporary MCAP (with the episode's metadata), export it into the session's dataset, delete it.
    Returns the dataset's folder. Raises what the export raises (the temporary file then stays)."""
    tmp = mcap_format.close_episode(h["mcap"], meta)
    result = export.export([tmp], h["dataset"], robot=h["robot"], include_failed=True)
    if result["skipped"]:
        raise RuntimeError(f"not exported (kept in {tmp}): {result['skipped'][0][1]}")
    os.remove(tmp)
    return h["dataset"]


def discard_episode(h: dict) -> None:
    """Drop the episode: its temporary file is deleted; the dataset is untouched."""
    mcap_format.discard_episode(h["mcap"])
