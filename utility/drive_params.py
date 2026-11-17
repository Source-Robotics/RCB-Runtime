"""Every drive's parameters, read on demand (2026-11-16, user: "all actual parameters listed in the GUI";
TODO/driver_110_rtb15_plan.md, 110 feature 2).

Firmware 109+ answers a parameter request (CAN command 37 -> reply 38: [number][value i32], or [number] alone for a
number it does not know). The catalogue here is the installed library's (Spectral_BLDC.PARAMETER_NAMES + the STEPFOC loop
profile, 92 numbers), grouped for the dashboard, with the robot XML key of the ten values RTI itself sends every drive at
its start and after a reconnect (gains and limits) — shown next to what the drive holds.

ParamRead is RTI's side: a read is started on demand (read_params:<node>, the executor's read_drive_parameters) and runs
inside RTI's round-robin status poll — every other free poll slot sends ONE request instead of a telemetry request, so the
frames per tick stay what they are today; nothing waits for an answer. The library decodes the answer into the motor
object (Motor.parameters) in the normal receive path; a few ticks after asking, ParamRead copies it to shared memory:
  rti['drive_params']        float64 [(node * 256) + number]   the value
  rti['drive_params_state']  float64, the same layout          0 not read, 1 read, 2 unknown to this firmware,
                                                               3 no answer, 4 asked (in flight)
  rti['drive_params_read']   float64 [5]                       state (0 idle, 1 reading, 2 done), done, total,
                                                               the node being read (-1 none), the read's sequence
  rti['drive_params_time']   float64 [nodes]                   when each node's read finished (Unix time, 0 = never)
A full read of 7 drives is 644 requests, one every other tick: ~5 s. The requests go number by number across the drives
(param 1 of every drive, then param 2, ...): a drive gets one request every ~14 ticks, so its answers never overlap.
"""
from __future__ import annotations

import time
from collections import deque

SLOTS = 256                     # numbers per node in shared memory (a parameter number is one byte)
NOT_READ, READ, UNKNOWN, NO_ANSWER, ASKED = 0, 1, 2, 3, 4
WAIT_TICKS = 6                  # copy an answer this many ticks after asking (it arrives within ~1-2 ms)


def _library():
    from Spectral_BLDC import Spectral_BLDC as Spectral
    return Spectral.SpectralCAN


def catalogue() -> dict:
    """{number: name} of every parameter the installed library names (+ the STEPFOC loop profile)."""
    m = _library()
    names = dict(getattr(m, "PARAMETER_NAMES", {}))
    for n, label in dict(getattr(m, "PARAM_PROFILE", {})).items():
        names.setdefault(n, f"loop profile: {label} (us)")
    return dict(sorted(names.items()))


def float_parameters() -> frozenset:
    """The numbers whose value travels as a float (the gains and limits, Kt)."""
    return frozenset(getattr(_library(), "FLOAT_PARAMETERS", ()))


# The robot XML key (without its motor_ / gripper_ prefix) of each value RTI sends every drive (load_motor_configs):
# the dashboard shows the XML's value next to the drive's.
XML_KEYS = {8: "KPP", 9: "KPV", 10: "KIV", 11: "KPIQ", 12: "KIIQ", 13: "KP", 14: "KD", 15: "velocity_limit",
            16: "Ilim", 17: "voltage_limit"}

# Writing (2026-11-16, the user's decisions): the values RTI sends are applied live (set_pid_gains, its key names) and
# saved to the robot / gripper XML (its tag names) — never written to the drive alone, RTI would overwrite them at the
# next start or reconnect; the drive-only values below are written to the drive's RAM with the arm disabled and kept
# only after "Save to drive" (the drive's whole working config, one save command); everything else is read-only (the ids
# and build, pole pairs, Kt, the measured current-sense zeros and trims, calibration runs, the scope, status bits, the
# gripper's mode / stroke, brake on idle: RTI asserts brake at every start).
PID_KEYS = {8: "KPP", 9: "KPV", 10: "KIV", 11: "KPIQ", 12: "KIIQ", 13: "KP", 14: "KD", 15: "vel_lim", 16: "Ilim",
            17: "vlim"}
XML_TAGS = {8: "KPP", 9: "KPV", 10: "KIV", 11: "KPIQ", 12: "KIIQ", 13: "KP", 14: "KD", 15: "Velocity_limit",
            16: "Ilim", 17: "voltage_limit"}
INT_XML = frozenset({15, 16, 17})              # written as whole numbers (ticks/s, mA, mV)
DRIVE_EDITABLE = (76, 77, 49, 50, 51, 64, 74, 75, 58, 90, 91, 2, 3, 4, 5, 19, 81, 37, 38, 53, 46)


def kind(number: int) -> str:
    """"rti" (RTI sends it from the XML), "drive" (drive-only, editable) or "read-only"."""
    if number in XML_KEYS:
        return "rti"
    return "drive" if number in DRIVE_EDITABLE else "read-only"


def xml_text(number: int, value: float) -> str:
    """A value as it is written into the XML: whole numbers for the limits, otherwise the shortest exact form."""
    if number in INT_XML:
        return str(int(round(value)))
    text = f"{float(value):.10g}"
    return text


def xml_set_value(text: str, tag: str, value_text: str, joint_id: int | None = None) -> str:
    """`text` (an XML file) with ONE value replaced in place — the first <tag>...</tag> inside <joint id="joint_id"> (a
    robot XML), or the first one in the file (a gripper XML: its values sit at the top level). Every other character —
    comments, layout — stays as it was (ElementTree would drop the comments). ValueError when it is not found."""
    import re
    lo, hi = 0, len(text)
    if joint_id is not None:
        m = re.search(rf'<joint\s+id="{int(joint_id)}"\s*>', text)
        if m is None:
            raise ValueError(f'no <joint id="{joint_id}"> in the file')
        end = text.find("</joint>", m.end())
        if end < 0:
            raise ValueError(f'<joint id="{joint_id}"> is not closed')
        lo, hi = m.end(), end
    m = re.compile(rf"<{re.escape(tag)}>([^<]*)</{re.escape(tag)}>").search(text, lo, hi)
    if m is None:
        raise ValueError(f"no <{tag}> {'in joint ' + str(joint_id) if joint_id is not None else 'in the file'}")
    return text[:m.start(1)] + value_text + text[m.end(1):]


# The dashboard's groups, in this order; a number in none of them goes to "Other".
GROUPS = [
    ("Identity and build", [86, 87, 84, 85, 23, 1, 78, 83]),
    ("Gains and limits (RTI sends these from the robot XML)", [8, 9, 10, 11, 12, 13, 14, 15, 16, 17]),
    ("Behaviour", [18, 91, 92, 93, 90]),
    ("Watchdog", [6, 7, 2, 3, 4, 5, 19]),
    ("Motor model and current sensing", [76, 77, 20, 58, 21, 22, 40, 41, 42, 46, 47, 48, 59, 60, 61, 62, 63, 88, 89]),
    ("Control loops and estimates", [49, 50, 51, 64, 74, 75, 65, 71, 72]),
    ("Encoder and anti-cogging", [53, 52, 54, 55, 56, 57, 37, 38, 43, 44, 45]),
    ("Status and sound", [79, 80, 81, 82]),
    ("Scope (capture)", [24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 39, 34, 35, 36]),
    ("Loop profile", [66, 67, 68, 69, 70]),
]


def grouped(names: dict | None = None) -> list:
    """[(title, [number, ...])] covering every catalogued number once, the GROUPS order, then "Other"."""
    names = catalogue() if names is None else names
    seen, out = set(), []
    for title, numbers in GROUPS:
        nums = [n for n in numbers if n in names and n not in seen]
        seen.update(nums)
        if nums:
            out.append((title, nums))
    rest = [n for n in names if n not in seen]
    if rest:
        out.append(("Other", rest))
    return out


def fmt(number: int, value) -> str:
    """A value as the dashboard shows it: hex for the board id / build commit, the build time as a UTC date, the bit
    fields in hex, floats to 6 significant digits, integers as they are."""
    if value is None:
        return ""
    if number in (86, 84):
        return f"{int(value) & 0xFFFFFFFF:08X}"
    if number in (79, 80):
        return f"0x{int(value) & 0xFFFFFFFF:X}"
    if number == 85 and value:
        return time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime(int(value)))
    if number in float_parameters():
        return f"{float(value):.6g}"
    return str(int(value))


class ParamRead:
    """RTI's paced read (see the module docstring). `values` / `state` / `read` / `times`: the shared memory arrays
    (or stand-ins); `numbers`: the numbers to read per node."""

    def __init__(self, values, state, read, times, numbers, clock=time.time):
        self.values, self.state, self.read, self.times = values, state, read, times
        self.numbers = tuple(numbers)
        self.clock = clock
        self.queue: deque = deque()          # (node, number) still to ask
        self.inflight: deque = deque()       # (node, number, tick asked)
        self.tick = 0
        self._left: dict = {}                # node -> numbers not finished (its read time when it reaches 0)
        self.read[0], self.read[3] = 0.0, -1.0

    @property
    def active(self) -> bool:
        return bool(self.queue or self.inflight)

    def start(self, nodes) -> None:
        """Read every number of `nodes` (a new read replaces one running). Their old values are marked not read."""
        nodes = list(nodes)
        self.queue.clear()
        self.inflight.clear()
        self._left = {n: len(self.numbers) for n in nodes}
        for n in nodes:
            base = n * SLOTS
            for num in self.numbers:
                self.state[base + num] = NOT_READ
        for num in self.numbers:                         # number by number across the drives
            for n in nodes:
                self.queue.append((n, num))
        self.read[0], self.read[1], self.read[2] = 1.0, 0.0, float(len(self.queue))
        self.read[3] = float(nodes[0]) if nodes else -1.0
        self.read[4] += 1.0

    def add(self, node: int, numbers) -> None:
        """Ask these numbers of one node (again: after a write) — appended, a read that runs goes on."""
        if not self.active:
            self.read[0], self.read[1], self.read[2] = 1.0, 0.0, 0.0
            self.read[4] += 1.0
            self._left = {}
        numbers = list(numbers)
        for num in numbers:
            self.state[node * SLOTS + num] = NOT_READ
            self.queue.append((node, num))
        self.read[2] += float(len(numbers))
        self._left[node] = self._left.get(node, 0) + len(numbers)

    def send_next(self, motors) -> None:
        """Ask the next (node, number); its old answer is dropped first, so a missing answer is never an old one."""
        if not self.queue:
            return
        n, num = self.queue.popleft()
        m = motors[n]
        params = getattr(m, "parameters", None)
        if params is not None:
            params.pop(num, None)
        m.Send_Get_Parameter(num)
        self.state[n * SLOTS + num] = ASKED
        self.read[3] = float(n)
        self.inflight.append((n, num, self.tick))

    def collect(self, motors) -> None:
        """Called every poll tick while a read runs: copy each answer WAIT_TICKS after asking; the read's progress."""
        self.tick += 1
        while self.inflight and self.tick - self.inflight[0][2] >= WAIT_TICKS:
            n, num, _ = self.inflight.popleft()
            m = motors[n]
            params = getattr(m, "parameters", None)
            i = n * SLOTS + num
            if params is not None and num in params:
                self.values[i] = float(params[num])
                self.state[i] = READ
            else:
                self.state[i] = UNKNOWN if getattr(m, "unknown_parameter", None) == num else NO_ANSWER
            self.read[1] += 1.0
            self._left[n] = self._left.get(n, 1) - 1
            if self._left[n] == 0:
                self.times[n] = self.clock()
        if not self.active and self.read[0] == 1.0:
            self.read[0], self.read[3] = 2.0, -1.0
