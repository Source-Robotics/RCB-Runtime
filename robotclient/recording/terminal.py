"""The recorder program's keyboard (python -m robotclient.recording, 2026-11-27): keys without Enter, lines of text.

In a terminal each key acts at once: on Windows through `msvcrt`; on Linux and macOS the terminal is put in cbreak
mode (no echo, no line editing; Ctrl+C still interrupts) and `close_keyboard()` puts it back. When the input is not a
terminal (a pipe, a script, the nets) each LINE is one key: an empty line is Enter, and a key that takes text may carry
it on its line (``t pick the cube``). A foot pedal that types a key is a key like any other.

A reader thread puts what it reads into a queue, so the program can wait for a key with a timeout (an episode that
ends by itself, a reset countdown). `next_key()` waits in short slices, so Ctrl+C is felt at once on every system.
These are the program's own screen and keyboard: the only robotclient code besides `__main__` that prints (it echoes
what is typed).
"""
from __future__ import annotations

import builtins
import os
import queue
import sys
import threading
import time

ENTER = "\n"
_SLICE_S = 0.2                    # a wait for a key is cut into slices this long (Ctrl+C on Windows)
_BACKSPACE = ("\x08", "\x7f")


def open_keyboard(raw: bool | None = None) -> dict:
    """Start reading the keyboard. `raw`: keys without Enter (True), one line per key (False), or None: raw when the
    input is a terminal. Returns the keyboard (a dict) for `next_key()`, `read_text()` and `close_keyboard()`."""
    if raw is None:
        isatty = getattr(sys.stdin, "isatty", None)
        raw = bool(isatty and isatty())
    kb: dict = {"q": queue.Queue(), "raw": raw, "text": None, "restore": None}
    if not raw:
        line = builtins.input                           # the one in force now (a script may have replaced it)

        def read():
            return line()
    elif os.name == "nt":
        import msvcrt

        def read():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):          # an arrow or function key: two codes, ignored
                msvcrt.getwch()
                return None
            return ch
    else:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        kb["restore"] = lambda: termios.tcsetattr(fd, termios.TCSADRAIN, old)

        def read():
            ch = sys.stdin.read(1)
            if ch == "":
                raise EOFError
            return ch
    threading.Thread(target=_reader, args=(kb["q"], read), name="keyboard", daemon=True).start()
    return kb


def _reader(q: queue.Queue, read) -> None:
    while True:
        try:
            item = read()
        except BaseException as e:  # noqa: BLE001  EOF, a Ctrl+C in input(): raised again by next_key()
            q.put(e)
            return
        if item is not None:
            q.put(item)


def _get(kb: dict, timeout: float | None):
    """The next thing read (a character or a line), or None after `timeout` s. Raises what the reader caught."""
    end = None if timeout is None else time.monotonic() + timeout
    while True:
        wait = _SLICE_S if end is None else min(_SLICE_S, max(0.0, end - time.monotonic()))
        try:
            item = kb["q"].get(timeout=wait)
        except queue.Empty:
            if end is not None and time.monotonic() >= end:
                return None
            continue
        if isinstance(item, BaseException):
            raise item
        return item


def next_key(kb: dict, timeout: float | None = None) -> str | None:
    """The next key: ENTER (``"\\n"``) for Enter, else the character as typed; None when `timeout` s pass without
    one. When the input is not a terminal: a line's first word when it is one character (the rest of the line is
    kept for `read_text()`), else the whole line.

    Raises:
        KeyboardInterrupt: Ctrl+C.
        EOFError: the input ended.
    """
    item = _get(kb, timeout)
    if item is None:
        return None
    if kb["raw"]:
        if item == "\x03":
            raise KeyboardInterrupt
        return ENTER if item in ("\r", "\n") else item
    line = item.strip()
    if not line:
        kb["text"] = None
        return ENTER
    key, _, rest = line.partition(" ")
    if len(key) == 1:
        kb["text"] = rest.strip() or None
        return key
    kb["text"] = None
    return line


def read_text(kb: dict, prompt: str) -> str:
    """A line of text (the task, a note), without its spaces at the ends; "" for Enter alone. In a terminal it is
    typed after `prompt` (Backspace works) and ended by Enter; otherwise it is the rest of the key's line
    (``t pick the cube``), or the next line.

    Raises:
        KeyboardInterrupt: Ctrl+C.
        EOFError: the input ended.
    """
    if not kb["raw"]:
        if kb["text"] is not None:
            text, kb["text"] = kb["text"], None
            return text
        print(prompt, end="", flush=True)
        line = _get(kb, None)
        print(flush=True)
        return line.strip()
    print(prompt, end="", flush=True)
    chars: list = []
    while True:
        ch = _get(kb, None)
        if ch == "\x03":
            print(flush=True)
            raise KeyboardInterrupt
        if ch in ("\r", "\n"):
            print(flush=True)
            return "".join(chars).strip()
        if ch in _BACKSPACE:
            if chars:
                chars.pop()
                print("\b \b", end="", flush=True)
        elif ch.isprintable():
            chars.append(ch)
            print(ch, end="", flush=True)


def close_keyboard(kb: dict) -> None:
    """Give the terminal back as it was (Linux / macOS). The reader thread ends with the program."""
    restore = kb.get("restore")
    if restore is not None:
        kb["restore"] = None
        restore()
