"""Sound through the motors (2026-11-17, user: homing done, an error, identify this arm, tones for programs — each
switched on / off; after the listening test: a boot chime too, the robot's own sounds on q, programs' tones on d;
then a library of 15 tunes, one chosen per robot sound; TODO/driver_110_rtb15_plan.md).

Firmware 109+ plays tones through a motor: a queue of up to 31 events per drive (a frequency, a loudness as a current,
how long until the next event; frequency 0 silences), started with one play command. A drive plays only in a
closed loop (position, velocity, current or PD mode) — RTI keeps every drive in one except with the motors off
(SAFETY_STOP), while flashing and while starting: a disabled arm and ACTIVE_ERROR hold at zero speed, so they play. One
drive plays — <sound><node> in system.xml (1 = J2, the loudest on PAR6: the largest gear and the upper arm's shell) —
at most <limit_mv> (parameter 81). The axis (parameter 82): d = the sound's current makes no torque (quieter); q =
louder, the tone is a real torque at its pitch (J2 at 250 mA: 1.84 Nm). The robot's own sounds (boot, homing done, an
error, identify) and a library tune (play_sound) play on <sound><axis> (q) at rest, where nothing judges the torque, and
on d in EXEC / JOG / RTI / SOFT_RTI, where the contact / collision checks read the motor current (RTI's _sound_axis).
Programs' raw tones (beep / play_tune) always play on d (PROGRAMS_AXIS, user 2026-11-17). Each robot sound plays the
tune chosen for it from TUNES (<sound><EVENT_tune>; DEFAULT_TUNES). Every tune sends both parameters first, so the axis is right per tune and a drive that restarted gets
them again. A Spectral Micro (the gripper's drive) plays only up to 2500 Hz.

SoundPlayer is RTI's side: a queue of single frames sent in the shared status-poll slot (one every other free slot,
like the parameter read and the drive capture), so a three-note chime is ~5 frames over ~40 ms and the per-tick data is
never delayed (a tune: its two parameters, clear, the notes, a silence, play). Pure helpers for the executor: note
names, a tune's checks, the <sound> block's text edit.
"""
from __future__ import annotations

import logging

import re
from collections import deque

MAX_EVENTS = 30                           # the drive's queue holds 31: the last is the silence
MIN_HZ, MAX_HZ = 20.0, 3000.0             # the drive plays 20 Hz to under half its loop rate (3125 Hz on a STEPFOC)
SWITCHES = ("boot", "homing_done", "error", "identify", "programs")
EVENTS = ("boot", "homing_done", "error", "identify")       # the robot's own sounds: each plays a tune chosen from TUNES
TUNE_SETTINGS = tuple(f"{e}_tune" for e in EVENTS)
PROGRAMS_AXIS = "d"                       # programs' raw tones: never a torque during a move (user, 2026-11-17)
SETTINGS = SWITCHES + ("axis", "current_ma", "node", "limit_mv") + TUNE_SETTINGS
P_SOUND_LIMIT_MV, P_SOUND_AXIS = 81, 82
# The library (2026-11-17, user: "a list of 15 different sounds ... select what i like the most"): original short tunes,
# (note or Hz, seconds); 0 = a rest. A name is a setting's value in system.xml: rename one only with its files.
TUNES = {
    "power_up": (("G4", 0.12), ("D5", 0.12), ("G5", 0.30)),
    "rise": (("C5", 0.11), ("E5", 0.11), ("G5", 0.22)),
    "low_double": (("A3", 0.18), (0, 0.09), ("A3", 0.26)),
    "fanfare": (("G4", 0.12), ("C5", 0.12), ("E5", 0.12), ("G5", 0.12), ("E5", 0.12), ("G5", 0.36)),
    "sweep_up": (("C5", 0.05), ("D5", 0.05), ("E5", 0.05), ("F5", 0.05), ("G5", 0.05), ("A5", 0.05), ("B5", 0.05),
                 ("C6", 0.15)),
    "sweep_down": (("C6", 0.05), ("B5", 0.05), ("A5", 0.05), ("G5", 0.05), ("F5", 0.05), ("E5", 0.05), ("D5", 0.05),
                   ("C5", 0.15)),
    "ding_dong": (("E5", 0.18), ("C5", 0.32)),
    "double_beep": (("A5", 0.08), (0, 0.06), ("A5", 0.08)),
    "chirp": (("G5", 0.05), ("C6", 0.05), ("G6", 0.12)),
    "fall": (("G5", 0.11), ("E5", 0.11), ("C5", 0.22)),
    "minor": (("A4", 0.12), ("C5", 0.12), ("E5", 0.24)),
    "alarm": (("A5", 0.12), ("E5", 0.12), ("A5", 0.12), ("E5", 0.12), ("A5", 0.12), ("E5", 0.12)),
    "triple_low": (("C4", 0.14), (0, 0.08), ("C4", 0.14), (0, 0.08), ("C4", 0.14)),
    "heartbeat": (("C4", 0.08), (0, 0.08), ("C4", 0.14), (0, 0.35), ("C4", 0.08), (0, 0.08), ("C4", 0.14)),
    "arpeggio": (("C4", 0.07), ("E4", 0.07), ("G4", 0.07), ("C5", 0.07), ("E5", 0.07), ("G5", 0.07), ("C6", 0.20)),
}
DEFAULT_TUNES = {"boot": "power_up", "homing_done": "rise", "error": "low_double", "identify": "fanfare"}
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


def tune_ms(name: str) -> list:
    """[(Hz, ms)] of a library tune (TUNES)."""
    return [(note_hz(n), int(round(s * 1000))) for n, s in TUNES[name]]


def sound_text(key: str, value) -> str:
    """A <sound> setting's text in system.xml: a switch / the node as an integer, the axis / a tune's name as it is, a
    number as %g."""
    if key in SWITCHES or key == "node":
        return str(int(value))
    if key == "axis" or key in TUNE_SETTINGS:
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
        self.on_q = False         # the last tune went out on q (a joint torque): silence_torque() stops it
        # Told (node, seconds) when a tune goes out on q (RTI: SafetyMonitor.mute_hold_osc — review C19, 2026-11-22:
        # the tune's swinging current read as a joint buzzing while holding).
        self.on_q_tune = None
        # node -> that drive's own current limit, mA (RTI: the robot XML's <Ilim> per joint, the gripper's): a tone never
        # asks more (2026-11-22; J6's limit is 1200 mA). {} = no clamp.
        self.max_ma: dict = {}
        # Nodes whose drive plays nothing (RTI: the CAN gripper's — it runs in controller mode 7, the firmware plays
        # only in modes 1-4; 2026-11-24: a <sound><node> left on it silenced every robot sound). A tune for one is
        # dropped, once logged.
        self.silent_nodes: set = set()

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
        if n in self.max_ma:
            current = min(current, int(self.max_ma[n]))
        ax = self.settings.get("axis", "q") if axis is None else axis
        # A new tune REPLACES what still waits (2026-11-18 review S2): the queue never grows past one tune — a program
        # beeping in a loop queued thousands of frames, played them for a minute and starved RTI's other polls.
        self.queue.clear()
        self.on_q = ax == "q"
        self.queue.append(("param", n, P_SOUND_LIMIT_MV, int(self.settings.get("limit_mv", 2500))))
        self.queue.append(("param", n, P_SOUND_AXIS, 1 if ax == "q" else 0))
        self.queue.append(("clear", n))
        for hz, ms in events:
            self.queue.append(("event", n, hz, current if hz else 0, ms))
        self.queue.append(("event", n, 0.0, 0, 0))
        self.queue.append(("play", n))
        if self.on_q and self.on_q_tune is not None:
            self.on_q_tune(n, sum(ms for _, ms in events) / 1000.0)

    def silence(self) -> None:
        """A Stop, a Disable or an error latching (2026-11-22, the user's stop_sound decision (b): they silence a
        playing tune — no command of its own): what waits is dropped and one clear frame stops the playing drive,
        whatever the axis. An error's own tune is queued after it."""
        self.queue.clear()
        self.queue.append(("clear", self.node))
        self.on_q = False

    def silence_torque(self) -> None:
        """Before a move (EXEC / JOG / RTI / SOFT_RTI, whose contact checks read the motor current): a tune that went
        out on q is stopped — what waits dropped, one clear frame (2026-11-18 review S1). Nothing when the last tune
        was on d."""
        if not self.on_q:
            return
        self.queue.clear()
        self.queue.append(("clear", self.node))
        self.on_q = False

    def tune(self, event: str) -> list:
        """[(Hz, ms)] of the tune chosen for one of the robot's own sounds (<sound><EVENT_tune>; a name not in the
        library: that sound's default)."""
        name = self.settings.get(f"{event}_tune", DEFAULT_TUNES[event])
        return tune_ms(name if name in TUNES else DEFAULT_TUNES[event])

    def event(self, name: str, holds: bool, axis=None) -> bool:
        """One of the robot's own sounds (boot / homing_done / error) when its switch is on and the drives hold, on
        `axis` (None = the settings'). True = queued."""
        if not holds or not int(self.settings.get(name, 0)):
            return False
        self.play(self.tune(name), axis)
        return True

    def request(self, switch: str, events, holds: bool, axis=None) -> str | None:
        """A tune asked for (identify / programs), on `axis` (None = the settings'): why not, or None and it is
        queued."""
        if not int(self.settings.get(switch, 0)):
            return f"sound for {switch} is switched off (system.xml <sound>, the dashboard's Sound card)"
        if not holds:
            return ("the drives are not holding (motors off, flashing or starting): a drive plays only in a closed "
                    "loop")
        self.play(events, axis)
        return None

    def send_next(self, motors) -> None:
        if not self.queue:
            return
        item = self.queue.popleft()
        if int(item[1]) in self.silent_nodes:
            self.queue.clear()
            if not getattr(self, "_silent_node_logged", False):
                self._silent_node_logged = True
                logging.getLogger("rti").warning(
                    "sound: node %s is the gripper's drive, which plays no sound (it runs in controller mode 7; "
                    "sounds play in modes 1-4) — no tune played; choose a joint (the Sound card, set_sound(drive=...))",
                    item[1])
            return
        if not 0 <= int(item[1]) < len(motors):
            # 2026-11-18 (review D1): a node this robot does not have (a hand-edited <sound><node> of 7-15 passes the
            # parser's 0-15) indexed past the drives in RTI's unguarded poll slot and ended RTI at the boot chime, 12.8 s
            # after every start. The tune is dropped, once logged.
            self.queue.clear()
            if not getattr(self, "_bad_node_logged", False):
                self._bad_node_logged = True
                logging.getLogger("rti").warning(
                    "sound: node %s is no drive of this robot (0-%d) — no tune played; fix system.xml <sound><node>",
                    item[1], len(motors) - 1)
            return
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
