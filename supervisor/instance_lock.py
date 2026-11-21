"""One supervisor per robot (review I8, 2026-11-21).

Two supervisors — two service units both enabled, or one started by hand next to the service — start two RTIs on one
CAN bus, and since the shared-memory ownership (review I3) both would own the layout. The supervisor takes this lock
before anything else; a second one finds it held and refuses to start, naming the holder.

An exclusive flock on a file in /dev/shm (RCB_SHM_PREFIX-aware, like the segments): the kernel drops it when the
holder ends, however it ends, so there is never a stale lock to clear by hand. The descriptor is not inherited by the
supervisor's children (Python opens it close-on-exec), so a child left running never holds it.
"""
from __future__ import annotations

import fcntl
import os


def lock_path(prefix: str | None = None) -> str:
    prefix = os.environ.get("RCB_SHM_PREFIX", "") if prefix is None else prefix
    return f"/dev/shm/{prefix}rcb-supervisor.lock"  # nosec B108 — POSIX shared memory's own folder, as the segments


def acquire(path: str | None = None) -> tuple[int | None, int | None]:
    """(descriptor, None) when this process holds the lock now — keep the descriptor open for the process's life;
    (None, the holder's pid, 0 when unknown) when another process holds it."""
    path = path or lock_path()
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        try:
            holder = int(os.read(fd, 32).decode("ascii", "replace").strip() or 0)
        except ValueError:
            holder = 0
        os.close(fd)
        return None, holder
    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode("ascii"))
    return fd, None
