"""What the dashboard does once per PROCESS, and the guard for its refresh timers (review G6 / G8, 2026-11-21).

NiceGUI 3.3's script mode runs nicegui_test.py again at every page load while the modules it imports stay as first
imported, so a flag kept HERE survives page loads and one kept in the page script does not (the same reason the
firmware flasher's session lives in gui_firmware_session.py). This module has no NiceGUI import, so the nets can run
it without a dashboard.
"""
from __future__ import annotations

import gc
import logging
import time

_FROZEN = [False]


def freeze_once() -> bool:
    """`gc.collect()` + `gc.freeze()` at the end of the FIRST page load only; True when it froze now.

    Freezing moves everything alive into the collector's permanent generation, so a full collection never scans the
    libraries and the first page again. Done at every page load (as it was) it pinned every page for good: a frozen
    object is never collected, and a page's namespace is a cycle (its functions hold its globals), so each load kept
    the last page's whole UI, plot buffers and plan preview until the dashboard restarted. Once, the first page's
    objects stay (one page, bounded); later pages are collected when they close.
    """
    if _FROZEN[0]:
        return False
    gc.collect()
    gc.freeze()
    _FROZEN[0] = True
    return True


COLLECT_S = 30.0          # a full collection at most this often, and only after a page went
_GONE = [0]               # pages deleted since the last collection
_COLLECTOR = [False]      # the app timer registered (once per process)


def page_gone() -> None:
    """A page was deleted (its client's delete hook): its objects are freed at the next collect_gone_pages()."""
    _GONE[0] += 1


def release_waiters(client) -> bool:
    """A page deleted without ever having connected (curl, a link preview, a tab closed or reloaded while it loaded):
    wake what waits for its connection; True when there was something to wake.

    Every `ui.timer` waits for its page's browser before its first tick (NiceGUI 3.3: `await client.connected()`, no
    timeout). NiceGUI deletes a never-connected client after a while but never wakes that wait, so each of the page's
    timers stayed blocked for good and held, through its callback, the page's whole namespace — ~8 MB per such load,
    measured 2026-11-22 (8 curl loads: 8 namespaces alive, 2 clients). Called from the page's delete hook, BEFORE the
    client's elements go: the timers resume after the deletion, find themselves deleted and end (their callback
    dropped), so the page is freed at the next collect_gone_pages()."""
    if getattr(client, "has_socket_connection", True):
        return False
    event = getattr(client, "_connected", None)       # NiceGUI's own event (client.py); missing = nothing to do
    if event is None or not hasattr(event, "set"):
        return False
    event.set()
    return True


def collect_gone_pages() -> bool:
    """A full collection when a page went since the last one; True when it ran.

    A closed page is one big reference cycle (its functions hold its globals), freed only by a FULL collection, and
    the dashboard's thresholds (nicegui_test.py: 50 000, 500, 1000, for the fast plot frames) practically never start
    one. The collection at every page load that used to free them went with the freeze it came with (freeze_once): 20
    page loads then kept ~160 MB for good (measured 2026-11-21, twice). Run by an app timer, never inside a page's own
    delete hook (NiceGUI removes the page's elements only after its hooks ran); one collection however many pages went.
    """
    if not _GONE[0]:
        return False
    _GONE[0] = 0
    gc.collect()
    return True


def start_collector(timer) -> bool:
    """Run collect_gone_pages every COLLECT_S on the app's timer (`timer` = NiceGUI's app.timer, not a page's) — once
    per process; True the first time."""
    if _COLLECTOR[0]:
        return False
    timer(COLLECT_S, collect_gone_pages)
    _COLLECTOR[0] = True
    return True


class Guarded:
    """`fn` for a `ui.timer` or a call while the page is built, that never lets an exception out.

    NiceGUI logs every exception a timer raises with its traceback: for the 10 Hz tables refresh, one missing
    shared-memory key was ten tracebacks a second. Here the FIRST failure of a kind (its type and message) is logged
    with its traceback, a repeat at most once every `every_s` as one line with a count, and the first call that works
    again says so. The call returns None when it failed.
    """

    def __init__(self, fn, what: str, log: logging.Logger | None = None, every_s: float = 60.0,
                 clock=time.monotonic):
        self._fn = fn
        self._what = what
        self._log = log or logging.getLogger("gui")
        self._every_s = float(every_s)
        self._clock = clock
        self._key = None          # (type name, message) of the failure being repeated, None while it works
        self._repeats = 0         # repeats since the last line about it
        self._last = 0.0          # when that line was written

    def __call__(self, *args, **kwargs):
        try:
            out = self._fn(*args, **kwargs)
        except Exception as exc:
            key = (type(exc).__name__, str(exc))
            now = self._clock()
            if key != self._key:
                self._key, self._repeats, self._last = key, 0, now
                self._log.exception("%s failed", self._what)
            else:
                self._repeats += 1
                if now - self._last >= self._every_s:
                    self._log.warning("%s still failing (%s: %s) — %d more time(s)", self._what, key[0], key[1],
                                      self._repeats)
                    self._repeats, self._last = 0, now
            return None
        if self._key is not None:
            self._log.info("%s works again", self._what)
            self._key, self._repeats = None, 0
        return out
