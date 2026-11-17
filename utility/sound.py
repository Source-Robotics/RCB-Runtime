"""Sound through the motors (2026-11-17, user: homing done, an error, identify this arm, tones for programs — each
switched on / off; after the listening test: a boot chime too, the robot's own sounds on q, programs' tones on d;
TODO/driver_110_rtb15_plan.md).

Firmware 109+ plays tones through a motor: a queue of up to 31 events per drive (a frequency, a loudness as a current,
how long until the next event; frequency 0 silences), started with one play command. A drive plays only in a
closed loop (position, velocity, current or PD mode) — RTI keeps every drive in one except with the motors off
(SAFETY_STOP), while flashing and while starting: a disabled arm and ACTIVE_ERROR hold at zero speed, so they play. One
drive plays — <sound><node> in system.xml (1 = J2, the loudest on PAR6: the largest gear and the upper arm's shell) —
at most <limit_mv> (parameter 81). The axis (parameter 82): d = the sound's current makes no torque (quieter); q =
louder, the tone is a real torque at its pitch (J2 at 250 mA: 1.84 Nm). The robot's own sounds (boot, homing done, an
error, identify) play on <sound><axis> (q): they play at rest, where nothing judges the torque. Programs' tones always
play on d (PROGRAMS_AXIS, user 2026-11-17): they can sound during a move, where the contact / collision checks read the
motor current. Every tune sends both parameters first, so the axis is right per tune and a drive that restarted gets
them again. A Spectral Micro (the gripper's drive) plays only up to 2500 Hz.

SoundPlayer is RTI's side: a queue of single frames sent in the shared status-poll slot (one every other free slot,
like the parameter read and the drive capture), so a three-note chime is ~5 frames over ~40 ms and the per-tick data is
never delayed (a tune: its two parameters, clear, the notes, a silence, play). Pure helpers for the executor: note
names, a tune's checks, the <sound> block's text edit.
"""
from __future__ import annotations

import re
from collections import deque

MAX_EVENTS = 30                           # the drive's queue holds 31: the last is the silence
MIN_HZ, MAX_HZ = 20.0, 3000.0             # the drive plays 20 Hz to under half its loop rate (3125 Hz on a STEPFOC)
SWITCHES = ("boot", "homing_done", "error", "identify", "programs")
PROGRAMS_AXIS = "d"                       # programs' tones: never a torque during a move (user, 2026-11-17)
SETTINGS = SWITCHES + ("axis", "current_ma", "node", "limit_mv")
P_SOUND_LIMIT_MV, P_SOUND_AXIS = 81, 82
# (note or Hz, seconds); 0 = a rest
TUNES = {
    "boot": (("G4", 0.12), ("D5", 0.12), ("G5", 0.30)),
    "homing_done": (("C5", 0.11), ("E5", 0.11), ("G5", 0.22)),
    "error": (("A3", 0.18), (0, 0.09), ("A3", 0.26)),
    "identify": (("G4", 0.12), ("C5", 0.12), ("E5", 0.12), ("G5", 0.12), ("E5", 0.12), ("G5", 0.36)),
    "test": (("A4", 0.4),),
}
_NOTE = re.compile(r"^([A-Ga-g])([#b]?)(\d)$")
_OFFSETS = {"c": 0, "d": 2, "e": 4, "f": 5, "g": 7, "a": 9, "b": 11}


def note_hz(note) -> float:
    """A note name ('A4' = 440 Hz, 'C#5', 'Eb3') or a frequency in Hz -> Hz. ValueError for anything else."""
    if isinstance(note, bool):
        raise ValueError(f"not a note: {note!r}")
    if isinstance(note, (int, float)):
        return float(note)
    m = _NOTE.match(str(note).strip())
    if m is None:
        raise ValueError(f"not a note: {note!r} (a name such as 'A4', 'C#5', or a frequency in Hz)")
    n = 12 * (int(m.group(3)) + 1) + _OFFSETS[m.group(1).lower()] + {"#": 1, "b": -1, "": 0}[m.group(2)]
    return 440.0 * 2 ** ((n - 69) / 12)


def tune_events(notes) -> list:
    """[(Hz, ms)] of a tune [(note or Hz, seconds), ...]: up to MAX_EVENTS notes, 0 Hz (a rest) or 20-3000 Hz (a
    STEPFOC plays up to half its 6250 Hz loop; above, silence), each 0.02-5 s. ValueError naming what is wrong."""
    notes = list(notes)
    if not 1 <= len(notes) <= MAX_EVENTS:
        raise ValueError(f"a tune has 1 to {MAX_EVENTS} notes, not {len(notes)}")
    out = []
    for k, item in enumerate(notes):
        try:
            note, seconds = item
        except (TypeError, ValueError):
            raise ValueError(f"note {k + 1}: {item!r} is not (note, seconds)") from None
        hz = note_hz(note)
        if hz != 0 and not MIN_HZ <= hz <= MAX_HZ:
            raise ValueError(f"note {k + 1}: {hz:g} Hz is outside {MIN_HZ:g}-{MAX_HZ:g} Hz (0 = a rest)")
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not 0.02 <= seconds <= 5.0:
            raise ValueError(f"note {k + 1}: {seconds!r} s is outside 0.02-5 s")
        out.append((round(hz, 1), int(round(seconds * 1000))))
    return out


def encode(events) -> str:
    """RTI's message form of [(Hz, ms)]: 'hz,ms;hz,ms;...'."""
    return ";".join(f"{hz:g},{ms}" for hz, ms in events)


def decode(text) -> list:
    """The inverse of encode (RTI's side), checked again — a value out of the library's range would raise inside the
    send, in RTI's tick: ValueError for a malformed message or any value out of range."""
    out = []
    for part in str(text).split(";"):
        hz_text, ms_text = part.split(",")
        hz, ms = float(hz_text), int(ms_text)
        if (hz != 0 and not MIN_HZ <= hz <= MAX_HZ) or not 20 <= ms <= 5000:
            raise ValueError(f"{hz:g} Hz / {ms} ms out of range")
        out.append((hz, ms))
    if not 1 <= len(out) <= MAX_EVENTS:
        raise ValueError(f"{len(out)} notes")
    return out


def sound_text(key: str, value) -> str:
    """A <sound> setting's text in system.xml: a switch / the node as an integer, the axis as it is, a number as %g."""
    if key in SWITCHES or key == "node":
        return str(int(value))
    if key == "axis":
        return str(value)
    return f"{float(value):g}"


def set_values_in_block(text: str, block: str, values: dict) -> str:
    """`text` (system.xml) with <tag>value</tag> replaced inside <block>...</block> for each tag, every other character
    kept (comments, layout). ValueError when the block or a tag is missing."""
    m = re.search(rf"<{block}>", text)
    end = text.find(f"</{block}>", m.end()) if m else -1
    if m is None or end < 0:
        raise ValueError(f"no <{block}> block")
    inner = text[m.end():end]
    for tag, value in values.items():
        tm = re.search(rf"<{tag}>([^<]*)</{tag}>", inner)
        if tm is None:
            raise ValueError(f"no <{tag}> in <{block}>")
        inner = inner[:tm.start(1)] + str(value) + inner[tm.end(1):]
    return text[:m.end()] + inner + text[end:]


class SoundPlayer:
    """RTI's side: the settings, and the frames to send ("param", node, number, value) | ("clear", node) |
    ("event", node, hz, mA, ms) | ("play", node), one per free poll slot."""

    def __init__(self, settings: dict):
        self.settings = dict(settings)
        self.queue: deque = deque()

    @property
    def node(self) -> int:
        return int(self.settings.get("node", 1))

    def apply(self, changes: dict) -> None:
        """New settings (the dashboard / set_sound): the next tune sends them."""
        self.settings.update(changes)

    def play(self, events, axis=None) -> None:
        """Queue a tune [(Hz, ms)] on the playing drive: its sound voltage cap and axis (parameters 81, 82; `axis`
        None = the settings' — the robot's own sounds), its queue cleared, the notes, a silence, play."""
        n, current = self.node, int(self.settings.get("current_ma", 250))
        ax = self.settings.get("axis", "q") if axis is None else axis
        self.queue.append(("param", n, P_SOUND_LIMIT_MV, int(self.settings.get("limit_mv", 2500))))
        self.queue.append(("param", n, P_SOUND_AXIS, 1 if ax == "q" else 0))
        self.queue.append(("clear", n))
        for hz, ms in events:
            self.queue.append(("event", n, hz, current if hz else 0, ms))
        self.queue.append(("event", n, 0.0, 0, 0))
        self.queue.append(("play", n))

    def event(self, name: str, holds: bool) -> bool:
        """A built-in tune for an event (boot / homing_done / error) when its switch is on and the drives hold. True =
        queued."""
        if not holds or not int(self.settings.get(name, 0)):
            return False
        self.play([(note_hz(n), int(s * 1000)) for n, s in TUNES[name]])
        return True

    def request(self, switch: str, events, holds: bool) -> str | None:
        """A tune asked for (identify / programs): why not, or None and it is queued."""
        if not int(self.settings.get(switch, 0)):
            return f"sound for {switch} is switched off (system.xml <sound>, the dashboard's Sound card)"
        if not holds:
            return ("the drives are not holding (motors off, flashing or starting): a drive plays only in a closed "
                    "loop")
        self.play(events, PROGRAMS_AXIS if switch == "programs" else None)
        return None

    def send_next(self, motors) -> None:
        if not self.queue:
            return
        item = self.queue.popleft()
        m = motors[item[1]]
        kind = item[0]
        if kind == "param":
            m.Send_Parameter(item[2], item[3])
        elif kind == "clear":
            m.Send_Sound_Clear()
        elif kind == "event":
            m.Send_Sound_Event(item[2], item[3], item[4])
        else:
            m.Send_Sound_Play()
