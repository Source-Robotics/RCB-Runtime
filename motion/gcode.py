"""Minimal G-code interpreter — the toolpath front end for EXEC mode.

WHAT THIS IS FOR. A slicer or CAM tool emits a bare list of moves and says NOTHING about
blending: G-code has no corner-rounding word. The corner policy is entirely the
controller's. This module turns that list into `move_linear` commands carrying our
`blend_mode="tolerance"` (the `G64 P<tol>` knob), which is what lets a sliced model stream
through thousands of corners without stopping.

SCOPE (deliberately minimal — see TODO/executive_mode_plan.md Phase 5):

    G20 / G21        units: inch / mm                        (mm default)
    G90 / G91        absolute / relative positioning         (absolute default)
    G0  / G1         rapid / linear move  (X Y Z F)          -> move_linear
    G64 P<tol>       blend within <tol> of the programmed path
    G61              exact stop (blending off — every corner comes to rest)
    G4  P<sec>       dwell                                   -> delay
    M0 / M2 / M30    pause / end of program
    F<feed>          feedrate, mm/min (or in/min under G20)

NOT SUPPORTED, on purpose:

  * G2 / G3 arcs. Slicers almost never emit them; CAM tools do. Adding them means an arc
    path builder, which is real work and not needed to run a sliced model.
  * EXTRUSION. `E` words are PARSED AND IGNORED — this arm has six joints and no extruder
    axis. A sliced Benchy will therefore trace the MOTION of the print and extrude nothing.
    That is stated loudly (once, at parse time) rather than left for you to discover: an
    interpreter that silently swallows E would look like it prints when it cannot.
  * Temperature/fan (M104/M109/M106/...). Same reason — no heaters. Ignored with a warning.

Anything else raises. Silently skipping an unrecognised word in a toolpath is how you get a
crash or a ruined part; this project's rule is to degrade LOUDLY.
"""
from __future__ import annotations

from collections.abc import Sequence

import logging
import re
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger("exec.planner")  # docs/logging.md "Logger names"

# Words we accept but cannot act on, mapped to why. Parsed, ignored, warned about ONCE.
IGNORED_WORDS = {"E": "no extruder axis on this arm"}
IGNORED_MCODES = {
    104: "hotend temperature (no heaters)",
    109: "hotend temperature + wait (no heaters)",
    140: "bed temperature (no heaters)",
    190: "bed temperature + wait (no heaters)",
    106: "fan on (no fan)",
    107: "fan off (no fan)",
    82: "extruder absolute (no extruder)",
    83: "extruder relative (no extruder)",
    84: "disable steppers (use robot.disable())",
}

# `; comment` or `(comment)`
_COMMENT = re.compile(r";.*$|\([^)]*\)")
# A G-code word: a letter followed by a number. Case-insensitive.
_WORD = re.compile(r"([A-Za-z])\s*([-+]?[0-9]*\.?[0-9]+)")

MM_PER_INCH = 25.4


@dataclass
class GcodeMove:
    """One linear move. Coordinates are ABSOLUTE MILLIMETRES in the work frame."""

    x: float
    y: float
    z: float
    feed_mm_min: Optional[float]        # None = no F seen yet; caller decides the default
    rapid: bool                          # True for G0
    blend_mm: float                      # 0.0 => exact stop (G61)
    line_no: int                         # source line, for error messages


@dataclass
class GcodeDwell:
    seconds: float
    line_no: int


@dataclass
class GcodePause:
    line_no: int


@dataclass
class GcodeProgram:
    ops: list = field(default_factory=list)
    ignored: dict = field(default_factory=dict)     # word/code -> count

    def moves(self):
        return [o for o in self.ops if isinstance(o, GcodeMove)]


class GcodeError(ValueError):
    """A line we will not guess at. Carries the line number and the offending text."""


def parse(text: str, start_xyz: Sequence[float] = (0.0, 0.0, 0.0), default_blend_mm: float = 0.05) -> GcodeProgram:
    """Parse G-code into a GcodeProgram.

    `start_xyz` is where the tool already is, in mm — needed because G-code moves are
    incremental in the sense that an unspecified axis KEEPS ITS VALUE. A `G1 X10` after
    `G1 X0 Y5 Z2` means (10, 5, 2), not (10, 0, 0). Getting that wrong sends the tool to
    the wrong place, so the modal state is tracked explicitly rather than defaulted.

    `default_blend_mm` is the tolerance used before any `G64` is seen. It exists because a
    slicer's output typically has no G64 at all — the corner policy is the controller's, and
    something must decide. 0.05mm is well below a printer's mechanical accuracy.
    """
    prog = GcodeProgram()
    pos = [float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2])]
    feed = None
    to_mm = 1.0                       # G21
    absolute = True                   # G90
    blend_mm = float(default_blend_mm)

    def note_ignored(key, why):
        if key not in prog.ignored:
            logger.warning(f"G-code: ignoring {key}: {why}")
            prog.ignored[key] = 0
        prog.ignored[key] += 1

    for line_no, raw in enumerate(text.splitlines(), start=1):
        line = _COMMENT.sub("", raw).strip()
        if not line:
            continue
        words = {}
        for letter, number in _WORD.findall(line):
            letter = letter.upper()
            # A line may legitimately repeat only G/M; anything else repeating is malformed.
            if letter in words and letter not in ("G", "M"):
                raise GcodeError(f"line {line_no}: '{letter}' given twice in {raw.strip()!r}")
            words.setdefault(letter, []).append(float(number))
        if not words:
            continue
        # Strip leading N (line number) — it carries no motion meaning.
        words.pop("N", None)

        for w, why in IGNORED_WORDS.items():
            if w in words:
                note_ignored(w, why)
                words.pop(w)

        gs = [int(v) for v in words.pop("G", [])]
        ms = [int(v) for v in words.pop("M", [])]

        # Modal settings first — they affect how this same line's motion is read.
        motion = None
        for g in gs:
            if g == 20:
                to_mm = MM_PER_INCH
            elif g == 21:
                to_mm = 1.0
            elif g == 90:
                absolute = True
            elif g == 91:
                absolute = False
            elif g == 64:
                p = words.pop("P", [None])[0]
                if p is None:
                    raise GcodeError(f"line {line_no}: G64 needs a P<tolerance> — got {raw.strip()!r}")
                blend_mm = float(p) * to_mm
                if blend_mm <= 0.0:
                    raise GcodeError(f"line {line_no}: G64 P must be > 0 (use G61 for exact stop)")
            elif g == 61:
                blend_mm = 0.0        # exact stop: every corner comes to rest
            elif g in (0, 1):
                motion = g
            elif g == 4:
                p = words.pop("P", [None])[0]
                if p is None:
                    raise GcodeError(f"line {line_no}: G4 needs a P<seconds> — got {raw.strip()!r}")
                prog.ops.append(GcodeDwell(seconds=float(p), line_no=line_no))
            elif g in (2, 3):
                raise GcodeError(
                    f"line {line_no}: G{g} (arc) is not supported — this interpreter handles "
                    f"linear moves only. Re-post the toolpath with arcs linearised, or add an "
                    f"arc path builder.")
            else:
                raise GcodeError(f"line {line_no}: unsupported G{g} in {raw.strip()!r}")

        for m in ms:
            if m in IGNORED_MCODES:
                note_ignored(f"M{m}", IGNORED_MCODES[m])
            elif m == 0:
                prog.ops.append(GcodePause(line_no=line_no))
            elif m in (2, 30):
                return _finish(prog)
            else:
                raise GcodeError(f"line {line_no}: unsupported M{m} in {raw.strip()!r}")

        if "F" in words:
            feed = float(words.pop("F")[0]) * to_mm

        axes = {a: words.pop(a)[0] * to_mm for a in ("X", "Y", "Z") if a in words}
        leftover = {k: v for k, v in words.items() if k not in ("P", "S")}
        if leftover:
            raise GcodeError(
                f"line {line_no}: unsupported word(s) {sorted(leftover)} in {raw.strip()!r}")

        if not axes:
            continue                    # a modal-only line (e.g. "G21", "F1500")
        if motion is None:
            # Bare "X10 Y10" is legal G-code (motion mode is modal), but relying on a mode
            # set many lines ago is exactly how a toolpath goes wrong quietly. Refuse it.
            raise GcodeError(
                f"line {line_no}: coordinates without a G0/G1 on the same line "
                f"({raw.strip()!r}) — modal motion carry-over is not supported")

        target = list(pos)
        for i, a in enumerate(("X", "Y", "Z")):
            if a in axes:
                target[i] = axes[a] if absolute else pos[i] + axes[a]
        prog.ops.append(GcodeMove(x=target[0], y=target[1], z=target[2],
                                  feed_mm_min=feed, rapid=(motion == 0),
                                  blend_mm=blend_mm, line_no=line_no))
        pos = target

    return _finish(prog)


def _finish(prog):
    if prog.ignored:
        logger.warning(
            "G-code: this program contained words this arm cannot act on: "
            + ", ".join(f"{k} x{n}" for k, n in prog.ignored.items())
            + ". The MOTION will run; nothing is extruded or heated.")
    return prog
