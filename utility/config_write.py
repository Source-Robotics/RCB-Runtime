"""Editing a configuration file safely from several processes (review D10, 2026-11-20).

config/system.xml has writers in two processes — the executor (the sound settings, the active gripper and its
put-back), the dashboard (the recorder settings, the display time zone) — and the robot / gripper XML two more (a
drive parameter saved, the freedrive walls). Each read the file, edited its text and swapped a temporary file in, all
through ONE temporary name and with no lock: two edits at once lost one of them (the second wrote its copy of the old
text over the first's change), or one writer's half-written temporary file was renamed in by the other.

    with edit_lock(path):                # an exclusive lock for the whole read - edit - write - check
        text = read(path)
        ...
        write_atomic(path, new_text)     # a temporary file of this process and thread, synced, then renamed in

The lock is a file in /dev/shm named after the file's absolute path (not next to the file: the config folder stays
clean, and a lock taken on the file itself is lost when the rename replaces it). flock: every process and every thread
that opens it waits for the holder; it goes when the holder ends, even if it crashed. Not re-entrant — a function that
runs under the lock writes with write_atomic, it never takes the lock again.
"""
from __future__ import annotations

import fcntl
import hashlib
import os
import threading
import time
from contextlib import contextmanager

LOCK_DIR = "/dev/shm"  # nosec B108 — a lock file, opened O_NOFOLLOW (a planted link is refused), holds no data
WAIT_S = 5.0                     # an edit takes milliseconds; longer = something hangs: refuse rather than wait for ever


def lock_path(path) -> str:
    key = hashlib.sha1(os.path.abspath(str(path)).encode()).hexdigest()[:16]   # nosec B324 — a name, not security
    return os.path.join(LOCK_DIR, f"rcb-edit-{key}.lock")


@contextmanager
def edit_lock(path, wait_s: float = WAIT_S):
    """Hold the edit lock of `path` for the block. Raises OSError when another edit holds it longer than wait_s."""
    fd = os.open(lock_path(path), os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        deadline = time.monotonic() + wait_s
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise OSError(f"{path} is being edited by another process (waited {wait_s:g} s)") from None
                time.sleep(0.01)
        yield
    finally:
        os.close(fd)                     # closing the descriptor releases the lock


def write_atomic(path, text: str, encoding: str = "utf-8") -> None:
    """`text` into `path`: a temporary file of THIS process and thread next to it, synced, then renamed over it — a
    reader sees the old file or the new one, never half of either; a failed write (a full SD card) leaves the old."""
    tmp = f"{path}.{os.getpid()}-{threading.get_ident()}.tmp"
    try:
        with open(tmp, "w", encoding=encoding) as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:  # nosec B110 — the temporary file may never have been made; the first error is raised
            pass
        raise
