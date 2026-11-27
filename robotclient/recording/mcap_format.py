"""The `mcap` recording format: one MCAP file per episode, everything kept exactly as received.

MCAP (mcap.dev) is a file of time-stamped channels with compressed chunks, an index and metadata; ROS 2 records in
it, and Foxglove, Lichtblick and PlotJuggler open it. Needs ``pip install mcap`` (it brings zstandard and lz4).

One episode = ``<name>.mcap``, written while recording as ``<name>.mcap.part`` and renamed when the episode is
closed: a ``.part`` file is one a crash left (``mcap recover`` repairs it; it is also readable as it is, in file
order: a reader that sorts by time waits for the missing end).
Chunks of 1 MB, zstd: a crash loses at most the last chunk (under a second). For each robot (its name, e.g. ``arm1``):

==============================  ==========================  ==============================================
topic                           encoding (schema)           each message
==============================  ==========================  ==============================================
``/<robot>/telemetry``          ``rcb.telemetry`` (none)    a packet exactly as the robot sent it, text or
                                                            binary: ``robotclient.telemetry.decode_packet``
                                                            reads it; sequence = its tick
``/<robot>/state``              cdr (``rcb_msgs/msg/        the same packet decoded, one field per tag, for
                                State``, ros2msg)           viewers to plot; a new definition (and channel)
                                                            when the packet's tags change
``/<robot>/camera/<camera>``    cdr (``sensor_msgs/msg/     the camera's JPEG, untouched; header.stamp = the
                                CompressedImage``)          image's moment; sequence = the robot's frame
                                                            counter
``/<robot>/camera/<camera>/     json (jsonschema)           every field of the frame's header
info``                                                      (robotclient.CameraFrame but `jpeg` and
                                                            `received`), its moment on the robot's clock
                                                            included; same log time and sequence as its image
``/<robot>/clock``              json (jsonschema)           the robot's clock against this PC's, at each
                                                            settled estimate (recording.clock)
==============================  ==========================  ==============================================

Times: log time = when it HAPPENED, on this PC's clock in nanoseconds since 1970 (a packet: the robot's `timestamp`;
a frame: the image's moment; both through the robot's clock mapping), so a viewer lines images and values up;
publish time = when it ARRIVED at this PC (same clock). Metadata records: ``recording`` (at the start: this format's
number, the recorder's and robots' versions, the sources) and ``episode`` (at the end: the task, the times, the counts).
Every metadata value is text (MCAP's rule); a structured one is JSON.
"""
from __future__ import annotations

import json
import os
import struct

from . import cdr

NAME = "mcap"
EXTENSION = ".mcap"
RAW_FORMAT = 1                   # this format's own number: what is in the file and how (raise for an incompatible change)
CHUNK_BYTES = 1 << 20
PACKET_ENCODING = "rcb.telemetry"

_INFO_SCHEMA = {"title": "rcb.CameraFrameInfo", "type": "object",
                "description": "Every field of a camera frame's header (robotclient.CameraFrame) but the JPEG."}
_CLOCK_SCHEMA = {"title": "rcb.RobotClock", "type": "object",
                 "description": "The robot's clock against the recording PC's (robotclient.recording.clock)."}


def missing() -> str | None:
    """None when this format can be written here, else what to install."""
    try:
        import mcap.writer  # noqa: F401
        import zstandard  # noqa: F401
    except ImportError as e:
        return f"the mcap format needs the mcap package: pip install mcap ({e})"
    return None


def _text(value) -> str:
    return value if isinstance(value, str) else json.dumps(value, separators=(",", ":"), default=str)


def open_episode(path: str, info: dict) -> dict:
    """Start writing an episode to `path` (``.mcap``; written as ``path + ".part"`` until `close_episode`).
    `info`: the ``recording`` metadata (each value text or JSON-able). Returns the episode's handle."""
    from mcap.writer import CompressionType, Writer
    part = path + ".part"
    f = open(part, "wb")
    try:
        w = Writer(f, chunk_size=CHUNK_BYTES, compression=CompressionType.ZSTD)
        w.start(profile="", library=f"robotclient.recording {info.get('recorder_version', '')}".strip())
        w.add_metadata("recording", {k: _text(v) for k, v in {"format": RAW_FORMAT, **info}.items()})
    except Exception:
        f.close()
        os.remove(part)
        raise
    return {"w": w, "f": f, "path": path, "part": part, "channels": {}, "schemas": {}, "layouts": {},
            "messages": 0, "state_errors": 0}


def _schema(ep: dict, name: str, encoding: str, data: bytes) -> int:
    key = (name, encoding, data)
    sid = ep["schemas"].get(key)
    if sid is None:
        sid = ep["w"].register_schema(name=name, encoding=encoding, data=data)
        ep["schemas"][key] = sid
    return sid


def _channel(ep: dict, key, topic: str, encoding: str, schema_id: int = 0, metadata: dict | None = None) -> int:
    cid = ep["channels"].get(key)
    if cid is None:
        cid = ep["w"].register_channel(topic=topic, message_encoding=encoding, schema_id=schema_id,
                                       metadata=metadata or {})
        ep["channels"][key] = cid
    return cid


def _json_channel(ep: dict, topic: str, schema: dict) -> int:
    sid = _schema(ep, schema["title"], "jsonschema", json.dumps(schema).encode())
    return _channel(ep, topic, topic, "json", sid)


def _add(ep: dict, cid: int, log_ns: int, arrival_ns: int, data: bytes, sequence: int) -> None:
    ep["w"].add_message(channel_id=cid, log_time=int(log_ns), publish_time=int(arrival_ns), data=data,
                        sequence=int(sequence) & 0xFFFFFFFF)
    ep["messages"] += 1


def add_packet(ep: dict, robot: str, log_ns: int, arrival_ns: int, raw: bytes, decoded: dict) -> None:
    """One robot packet: `raw` exactly as received, and its decoded form for viewers (a packet the viewer form cannot
    hold is counted in ``state_errors``; the raw packet is always written)."""
    tick = decoded.get("tick")
    seq = int(tick) if isinstance(tick, (int, float)) and tick == tick else 0
    topic = f"/{robot}/telemetry"
    cid = _channel(ep, topic, topic, PACKET_ENCODING,
                   metadata={"decoder": "robotclient.telemetry.decode_packet", "sequence": "the packet's tick"})
    _add(ep, cid, log_ns, arrival_ns, bytes(raw), seq)
    try:
        layout = cdr.state_layout(decoded)
        data = cdr.encode_state(layout, decoded)           # first: a packet that does not fit registers nothing
    except (KeyError, TypeError, ValueError, OverflowError, struct.error) as e:
        ep["state_errors"] += 1
        ep["last_state_error"] = repr(e)
        return
    sid = _schema(ep, cdr.STATE_SCHEMA_NAME, "ros2msg", cdr.state_definition(layout).encode())
    cid = _channel(ep, (f"/{robot}/state", sid), f"/{robot}/state", "cdr", sid)
    _add(ep, cid, log_ns, arrival_ns, data, seq)


def add_frame(ep: dict, robot: str, camera: str, log_ns: int, arrival_ns: int, frame) -> None:
    """One camera frame (robotclient.CameraFrame): the JPEG once, as a standard image, and its header's fields."""
    topic = f"/{robot}/camera/{camera}"
    sid = _schema(ep, cdr.IMAGE_SCHEMA_NAME, "ros2msg", cdr.IMAGE_DEFINITION.encode())
    cid = _channel(ep, topic, topic, "cdr", sid)
    _add(ep, cid, log_ns, arrival_ns, cdr.encode_image(log_ns, f"{robot}/{camera}", "jpeg", frame.jpeg), frame.frame)
    info = {k: v for k, v in frame._asdict().items() if k not in ("jpeg", "received")}
    info["jpeg_bytes"] = len(frame.jpeg)
    _add(ep, _json_channel(ep, topic + "/info", _INFO_SCHEMA), log_ns, arrival_ns,
         json.dumps(info, separators=(",", ":")).encode(), frame.frame)


def add_clock(ep: dict, robot: str, log_ns: int, sample: dict) -> None:
    """One settled estimate of the robot's clock (a dict: JSON)."""
    _add(ep, _json_channel(ep, f"/{robot}/clock", _CLOCK_SCHEMA), log_ns, log_ns,
         json.dumps(sample, separators=(",", ":")).encode(), 0)


def close_episode(ep: dict, meta: dict) -> str:
    """Write the ``episode`` metadata (`meta`), finish the file (index, summary) and give it its name. Returns the
    path."""
    w = ep["w"]
    w.add_metadata("episode", {k: _text(v) for k, v in meta.items()})
    w.finish()
    ep["f"].close()
    os.replace(ep["part"], ep["path"])
    return ep["path"]


def discard_episode(ep: dict) -> None:
    """Drop the episode: its file is deleted."""
    try:
        ep["f"].close()
    finally:
        if os.path.exists(ep["part"]):
            os.remove(ep["part"])
