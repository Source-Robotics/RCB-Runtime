"""The camera stream's wire format and the camera server's shared-memory names (2026-11-23, TODO/camera_plan.md).

The camera server (camera_server.py) answers `GET /frames` on its HTTP port (config/networking.CAMERA_PORT) with an HTTP
response header (ending in an empty line), then sends every frame as a fixed little-endian header followed by the
camera's own JPEG bytes (never decoded or re-encoded on the robot):

  offset  type  field
   0      4s    magic            b"RCBF"
   4      u16   version          CAMERA_PROTOCOL_VERSION
   6      u16   header_length    HEADER.size (a reader skips anything beyond the fields it knows)
   8      u32   frame            the server's frame counter (a gap = frames skipped for THIS client, which was slow)
  12      u32   driver_seq       the camera driver's sequence number (a gap = frames the driver lost)
  16      u16   width
  18      u16   height
  20      u32   flags            FLAG_* below
  24      f64   exposure_time    s, the robot's CLOCK_MONOTONIC: when the image was taken (what it means: the flags)
  32      f64   rti_tick         RTI's loop tick at exposure_time, fractional; NaN when RTI's trace was not readable
  40      u32   exposure_us      the exposure length, 0 = not known (auto exposure)
  44      u32   pts              the camera's own timestamp (its clock), 0 = not read
  48      f64   send_time        s, the robot's CLOCK_MONOTONIC when the server queued the frame for this client
  56      u32   jpeg_length      bytes of JPEG after the header

exposure_time is the driver's timestamp until the camera's own timestamps are used (plan P2: FLAG_CAMERA_CLOCK) and
the calibrated offset to the middle of the exposure is applied (P3: FLAG_CALIBRATED). The driver's timestamp comes in
4 ms steps (one USB transfer) and marks when the frame's first packet arrived, ~1 ms after the camera started reading
the sensor out (measured 2026-11-23).

CAMERA_PROTOCOL_VERSION is raised ONLY for an incompatible change of this layout (VERSIONING.md); a new field goes
after the last one with header_length grown, which old readers skip.
"""
from __future__ import annotations

import math
import struct

CAMERA_PROTOCOL_VERSION = 1
MAGIC = b"RCBF"
HEADER = struct.Struct("<4sHHIIHHIddIIdI")
assert HEADER.size == 60

FLAG_CAMERA_CLOCK = 0x01        # exposure_time from the camera's own timestamp (else the driver's)
FLAG_CALIBRATED = 0x02          # exposure_time includes the calibrated offset to the middle of the exposure
FLAG_AUTO_EXPOSURE = 0x04       # the camera chose the exposure itself (exposure_us 0)
FLAG_TICK = 0x08                # rti_tick is valid

# Shared memory (data.shared_data.SharedString, RCB_SHM_PREFIX-aware): the server's status as JSON, for the dashboard
# and the supervisor's hang check (its "beat" counter). Settings travel through config/system.xml <camera> instead.
STATUS_NAME, STATUS_SIZE = "camera_status", 16384


def pack(frame: int, driver_seq: int, width: int, height: int, flags: int, exposure_time: float, rti_tick: float,
         exposure_us: int, pts: int, send_time: float, jpeg_length: int) -> bytes:
    return HEADER.pack(MAGIC, CAMERA_PROTOCOL_VERSION, HEADER.size, frame & 0xFFFFFFFF, driver_seq & 0xFFFFFFFF,
                       width, height, flags, exposure_time, rti_tick, exposure_us, pts & 0xFFFFFFFF, send_time,
                       jpeg_length)


def unpack(data: bytes) -> dict:
    """The fields of a header (the first HEADER.size bytes). Raises ValueError for a wrong magic or a version this
    code does not read."""
    magic, version, hlen, frame, seq, w, h, flags, t_exp, tick, exp_us, pts, t_send, n = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError(f"not a camera frame header (magic {magic!r})")
    if version != CAMERA_PROTOCOL_VERSION:
        raise ValueError(f"camera protocol {version}, this code reads {CAMERA_PROTOCOL_VERSION}")
    return {"header_length": hlen, "frame": frame, "driver_seq": seq, "width": w, "height": h, "flags": flags,
            "exposure_time": t_exp, "rti_tick": tick if not math.isnan(tick) else None, "exposure_us": exp_us,
            "pts": pts, "send_time": t_send, "jpeg_length": n}
