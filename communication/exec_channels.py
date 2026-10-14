"""
EXEC-mode trajectory batch serialization (Phase 3, Phase A of the EXEC checklist).

The planner thread (in command_executor) packs a whole trajectory — or a chunk of
one — into a single self-describing bytes blob and PUSHes it over
``ZMQ_EXEC_TRAJECTORY_SOCKET``. RTI PULLs the blob non-blocking in the RT loop and
reconstructs the arrays with ``np.frombuffer`` (no re-parsing, no JSON on the hot path).

Wire format (little-endian, one contiguous bytes object)::

    offset  type        field
    0       4s          MAGIC  = b'RCBX'
    4       uint8       version = 1
    5       uint8       is_last_chunk (0/1)
    6       uint32      num_points  (N)
    10      uint16      joint_num   (J)
    12      uint16      checkpoint_len (L)
    14      uint32      flags (bitfield)
    18      L bytes     checkpoint_id (utf-8)  — e.g. "aB3xYz" from generate_short_id()
    ...     float64[N*J] joint_positions   (radians)
    ...     float64[N*J] joint_velocities  (rad/s)
    ...     float32[N*J] torque_ff         (Nm feedforward)

The header is self-describing: ``unpack_batch`` needs no external knowledge of N or J.
"""

import struct
import numpy as np

_MAGIC = b"RCBX"
_VERSION = 1

# 4s B B I H H I  →  MAGIC, version, is_last, num_points, joint_num, checkpoint_len, flags
_HEADER_STRUCT = struct.Struct("<4sBBIHHI")
_HEADER_SIZE = _HEADER_STRUCT.size  # 18 bytes

# Dtypes are fixed by the wire format — positions/velocities are float64, torque float32.
_POS_DTYPE = np.float64
_VEL_DTYPE = np.float64
_TRQ_DTYPE = np.float32

# Batch flags. The wire field is uint32; unknown flags are ignored by older readers.
BATCH_FLAG_BLEND_CONTINUES = 1 << 0

# A FENCE (2026-10-14): a separate 12-byte message on the trajectory socket, b'RCBF' + uint64 epoch. The planner sends
# one at every flush (a Stop); everything before it on the socket belongs to the stopped program, so RTI drops up to
# it and keeps what follows. Its own magic, not a batch flag: an older reader would play a flagged 1-point batch, while
# it drops a message with another magic as malformed (nothing moves).
_FENCE_STRUCT = struct.Struct("<4sQ")
_FENCE_MAGIC = b"RCBF"


def pack_fence(epoch):
    """The fence message for stop epoch `epoch` (an int >= 1)."""
    return _FENCE_STRUCT.pack(_FENCE_MAGIC, int(epoch))


def fence_epoch(raw):
    """The epoch of a fence message, or None when `raw` is not one (a batch)."""
    if len(raw) == _FENCE_STRUCT.size and raw[:4] == _FENCE_MAGIC:
        return _FENCE_STRUCT.unpack(raw)[1]
    return None


def pack_batch(joint_positions, joint_velocities, torque_ff,
               checkpoint_id="", is_last_chunk=True, flags=0):
    """Serialize one trajectory batch/chunk to a single bytes object.

    Args:
        joint_positions:  array-like (N, J) — radians.
        joint_velocities: array-like (N, J) — rad/s.
        torque_ff:        array-like (N, J) — Nm feedforward.
        checkpoint_id:    str command id to report on completion (last chunk only);
                          "" for intermediate chunks.
        is_last_chunk:    True if this is the final chunk of the command.
        flags:            bitfield, e.g. BATCH_FLAG_BLEND_CONTINUES.

    Returns:
        bytes — ready for ``socket.send(..., copy=False)`` / ``send_bytes``.
    """
    pos = np.ascontiguousarray(joint_positions, dtype=_POS_DTYPE)
    vel = np.ascontiguousarray(joint_velocities, dtype=_VEL_DTYPE)
    trq = np.ascontiguousarray(torque_ff, dtype=_TRQ_DTYPE)

    if pos.ndim != 2 or vel.shape != pos.shape or trq.shape != pos.shape:
        raise ValueError(
            f"pack_batch: shapes must match and be 2-D, got "
            f"pos={pos.shape}, vel={vel.shape}, trq={trq.shape}")

    num_points, joint_num = pos.shape
    cp_bytes = checkpoint_id.encode("utf-8")

    header = _HEADER_STRUCT.pack(
        _MAGIC, _VERSION, 1 if is_last_chunk else 0,
        num_points, joint_num, len(cp_bytes), flags & 0xFFFFFFFF,
    )
    return b"".join((header, cp_bytes, pos.tobytes(), vel.tobytes(), trq.tobytes()))


def unpack_batch(raw):
    """Reconstruct a trajectory batch from bytes produced by ``pack_batch``.

    The returned arrays are views into ``raw`` (read-only, zero-copy). Callers that
    keep them past the lifetime of ``raw`` should copy, but the RT consumer only
    reads one row per tick into its own pre-allocated buffers, so views are fine.

    Returns:
        dict with keys: num_points, joint_num, joint_positions (N,J), joint_velocities
        (N,J), torque_ff (N,J), checkpoint_id (str), is_last_chunk (bool), flags (int).
    """
    if len(raw) < _HEADER_SIZE:
        raise ValueError(f"unpack_batch: buffer too small ({len(raw)} bytes)")

    magic, version, is_last, num_points, joint_num, cp_len, flags = \
        _HEADER_STRUCT.unpack_from(raw, 0)
    if magic != _MAGIC:
        raise ValueError(f"unpack_batch: bad magic {magic!r}")
    if version != _VERSION:
        raise ValueError(f"unpack_batch: unsupported version {version}")

    off = _HEADER_SIZE
    checkpoint_id = raw[off:off + cp_len].decode("utf-8")
    off += cp_len

    count = num_points * joint_num
    pos_nbytes = count * _POS_DTYPE().itemsize
    vel_nbytes = count * _VEL_DTYPE().itemsize
    trq_nbytes = count * _TRQ_DTYPE().itemsize

    expected = off + pos_nbytes + vel_nbytes + trq_nbytes
    if len(raw) != expected:
        raise ValueError(
            f"unpack_batch: length mismatch, expected {expected}, got {len(raw)}")

    shape = (num_points, joint_num)
    pos = np.frombuffer(raw, dtype=_POS_DTYPE, count=count, offset=off).reshape(shape)
    off += pos_nbytes
    vel = np.frombuffer(raw, dtype=_VEL_DTYPE, count=count, offset=off).reshape(shape)
    off += vel_nbytes
    trq = np.frombuffer(raw, dtype=_TRQ_DTYPE, count=count, offset=off).reshape(shape)

    return {
        "num_points": num_points,
        "joint_num": joint_num,
        "joint_positions": pos,
        "joint_velocities": vel,
        "torque_ff": trq,
        "checkpoint_id": checkpoint_id,
        "is_last_chunk": bool(is_last),
        "flags": flags,
    }
