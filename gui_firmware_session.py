"""The Firmware tab's flasher session, ONE for the whole dashboard (review G5, 2026-11-20).

The dashboard script (nicegui_test.py) runs again at every page load (NiceGUI 3.3 script mode), but the modules it
imports stay as first imported. The flasher's state lived in the script's globals, so each page load started with
"idle": a reloaded page showed no running flash, hid Cancel and enabled "Take bus back" — which puts RTI back on the
bus while the flasher writes, the two-transmitter failure the tab exists to prevent. Here the state lives in a module:
every page sees the one flasher, can cancel it, and is locked out while it runs. The First run tab keeps its session
the same way.
"""
from __future__ import annotations


class FlasherSession:
    """The running flasher (an asyncio subprocess) or None, and `starting` — claimed synchronously before the spawn's
    await, so two clicks (two pages, even two browsers) can never both start one."""

    def __init__(self):
        self.proc = None
        self.starting = False
        self.shutdown_hooked = False

    def busy(self) -> bool:
        return self.proc is not None or self.starting

    def kill(self) -> None:
        """The dashboard stops: a flasher must not keep transmitting after it (never raises)."""
        proc = self.proc
        if proc is not None:
            try:
                proc.kill()
            except (ProcessLookupError, OSError):
                pass


SESSION = FlasherSession()
