"""Which arm is this, and did every joint home where THIS arm's joint always homes? (2026-10-09, TODO/homing_plan.md H1)

One control box drives several arms (user: grey, light_grey, black, ...) and the drivers all report SERIAL_NUMBER 1, so
nothing tells the box which arm is connected. Homing does: each joint's motor stops at its endstop at a spot within one
motor revolution that is fixed by how that arm was built (where the encoder magnet sat when the motor was mounted) —
it repeats within a few motor ticks on one arm (±0.01-0.05 deg at the joint, measured 2026-10-09) and differs by
thousands of ticks between arms. So after homing, the latches recognise the arm — and a joint that latched somewhere
else on an arm that is otherwise recognised is a FAULT: a false stall both passes repeated (the black arm's J2 stopped
4.8 deg short of its endstop), a skipped belt (grey J3 moved ~0.48 motor revolution) or a slipping coupling. Those
references would be wrong by degrees and nothing else notices.

User decisions (2026-10-09): such a joint FAILS homing; arms are LEARNED automatically (known after 3 homings that
agree); the tolerance is 1 deg at the joint. 2026-10-14 (user, after a pose where J1 itself false-stalled — "another arm?"
let J2 / J3 home wrong and the sequence go on to the gripper): once an arm is KNOWN, first joints that match none of the
known arms FAIL homing at once, and a new arm is learned only after "learn a new arm on the next homing" (the dashboard's
Arms card / `tools/homing_arms.py --learn-next`: `learn_next` in the table, for one homing). And an <approach> pre-move
is checked where it stops against where it stops on the known arms (`check_approach`).

The table (temp_data/homing_arms.json, machine-specific):
    {"version": 1, "arms": [{"id": "arm-1", "uid": "PAR6-7F3A9C", "name": "grey", "known": true, "homings": 7,
                             "joints": {"J1": [latches...], "J4@MSG_small_motor_150mm_rail": [...], ...},
                             "first_utc": "...", "last_utc": "...",
                             "history": [{"utc", "kind", "duration_s", "devs": {"J1": deg}, "two_pass": {"J2": ticks},
                                          "relaxed": {"J2": ticks}}, ...  the newest HISTORY homings]}],
     "last_failed": {"arm": "arm-1", "gripper": "...", "latches": {"1": ...}, "utc": "..."} | null}
A joint whose home offset depends on the gripper (`home_offset_gripper_dependent`: PAR6 J4 / J5 — J5 homes against the
gripper) is stored per gripper, so a new gripper is learned instead of failing. The newest WINDOW latches per joint are
kept; the reference is their circular mean within one motor revolution. A shift of a WHOLE motor revolution cannot be
told apart from the multi-turn count after a power cycle and is not caught.

RTI: the table is read at start; RTI's real-time thread only touches the dict in memory — a worker thread that moves
itself off RTI's core (as utility/incident_dump does) re-reads the file when a homing starts (to see `tools/homing_arms.py`
edits) and writes it after one ends. `tools/homing_arms.py`: list, name, forget, accept the last failed homing.
"""
from __future__ import annotations

import copy
import hashlib
import json
import logging
import math
import os
import queue
import threading
import time

TOL_DEG = 1.0          # a joint further than this (at the joint) from its usual spot is off (user, 2026-10-09)
KNOWN_AFTER = 3        # homings that agree before an arm counts as known (user, 2026-10-09)
APPROACH_TOL_DEG = 2.0  # an <approach> stopping further than this from where it stops on a known arm fails (2026-10-14)
APPROACH_MIN = 3       # homings with an approach before that spot is learned (until then: only "not short of the latch")
SAME_ARM_MIN = 4       # joints that must agree with a known arm for "this IS that arm" (6 joints; by chance ~1e-4)
WINDOW = 10            # latches kept per joint
HISTORY = 100          # homings kept per arm (the dashboard's history, 2026-10-09)


def make_uid(prefix: str, seed: str) -> str:
    """An arm's ID, e.g. 'PAR6-7F3A9C' (user, 2026-10-09: a random ID on top of the editable name). Derived from the
    arm's first homing so every program that sees the table derives the same one; stored once made, never changed."""
    return f"{prefix}-{hashlib.sha256(seed.encode()).hexdigest()[:6].upper()}"


def ensure_uids(table: dict, prefix: str) -> bool:
    """Give every arm without one its ID (arms learned before IDs existed). True when the table changed (save it)."""
    changed = False
    taken = {a.get("uid") for a in table["arms"]}
    for arm in table["arms"]:
        if not arm.get("uid"):
            seed = f"{arm['id']}|{json.dumps(arm['joints'], sort_keys=True)}"      # no clock: every reader derives the same
            uid = make_uid(prefix, seed)
            while uid in taken:
                seed += "+"
                uid = make_uid(prefix, seed)
            arm["uid"], changed = uid, True
            taken.add(uid)
    return changed


def empty_table() -> dict:
    return {"version": 1, "arms": [], "last_failed": None}


def joint_key(j: int, gripper_dependent, gripper: str) -> str:
    return f"J{j + 1}@{gripper}" if gripper_dependent[j] else f"J{j + 1}"


def circular_centre(latches, counts: int) -> float:
    """The circular mean of latches within one motor revolution, in [0, counts)."""
    s = sum(math.sin(2 * math.pi * (x % counts) / counts) for x in latches)
    c = sum(math.cos(2 * math.pi * (x % counts) / counts) for x in latches)
    return (math.atan2(s, c) % (2 * math.pi)) / (2 * math.pi) * counts


def off_deg(latch, centre: float, counts: int, gear: float) -> float:
    """How far a latch is from a centre within one motor revolution, in degrees at the joint (signed)."""
    d = (latch % counts - centre + counts / 2.0) % counts - counts / 2.0
    return d / counts / abs(gear) * 360.0


def compare(arm: dict, latches: dict, keys: dict, counts, gears) -> tuple[int, int, list, dict]:
    """(joints that agree, joints compared, [(j, deg) for the ones that do not], {j: deg} for every compared joint) for
    one stored arm — deg = how far this latch is from the joint's usual spot."""
    agree, compared, off, devs = 0, 0, [], {}
    for j, latch in latches.items():
        stored = arm["joints"].get(keys[j])
        if not stored:
            continue
        compared += 1
        d = off_deg(latch, circular_centre(stored, counts[j]), counts[j], gears[j])
        devs[j] = d
        if abs(d) <= TOL_DEG:
            agree += 1
        else:
            off.append((j, d))
    return agree, compared, off, devs


def judge(table: dict, latches: dict, gripper: str, gears, gripper_dependent, counts) -> dict:
    """What this homing's latches ({joint index: motor ticks}) say. kind:
      "known"    — a known arm, every compared joint within TOL_DEG (it is that arm, homed right)
      "fault"    — a known arm by >= SAME_ARM_MIN joints, but `off` joints elsewhere -> homing FAILS
      "learning" — an arm still being learned, every compared joint agrees (one more agreeing homing)
      "relearn"  — an arm being learned, recognised but some joints disagree (it starts over from this homing)
      "new"      — no stored arm matches"""
    keys = {j: joint_key(j, gripper_dependent, gripper) for j in latches}
    best = None
    for arm in table["arms"]:
        agree, compared, off, devs = compare(arm, latches, keys, counts, gears)
        rank = (agree, arm["known"])
        if best is None or rank > best[0]:
            best = (rank, arm, agree, compared, off, devs)
    if best is None or best[2] < SAME_ARM_MIN:
        return {"kind": "new", "arm": None, "agree": 0 if best is None else best[2], "compared": 0, "off": [], "devs": {},
                "keys": keys}
    _, arm, agree, compared, off, devs = best
    if arm["known"]:
        kind = "known" if not off else "fault"
    else:
        kind = "learning" if not off else "relearn"
    return {"kind": kind, "arm": arm["id"], "agree": agree, "compared": compared, "off": off, "devs": devs, "keys": keys}


def history_entry(verdict: dict, utc: str, stats: dict | None) -> dict:
    """One homing in an arm's history: when, the verdict, how long, each joint's distance from its usual spot (deg), and
    from homing itself each joint's two-pass difference and release relaxation (motor ticks)."""
    stats = stats or {}
    return {"utc": utc, "kind": verdict["kind"], "duration_s": stats.get("duration_s"),
            "devs": {f"J{j + 1}": round(float(d), 4) for j, d in sorted(verdict.get("devs", {}).items())},
            "two_pass": {f"J{j + 1}": int(v) for j, v in sorted((stats.get("two_pass") or {}).items()) if v is not None},
            "relaxed": {f"J{j + 1}": int(v) for j, v in sorted((stats.get("relaxed") or {}).items()) if v is not None},
            # pass 1's travel from where the joint started to its endstop, motor ticks (2026-10-14: measured first)
            "travel": {f"J{j + 1}": int(v) for j, v in sorted((stats.get("travel") or {}).items()) if v is not None},
            # where an <approach> stopped (at its soft contact), minus the joint's latch, motor ticks (2026-10-14:
            # check_approach learns it; "approach" held the earlier, pressed arrivals — not comparable, not read)
            "approach_contact": {f"J{j + 1}": int(v) for j, v in sorted((stats.get("approach") or {}).items())
                                 if v is not None}}


def apply(table: dict, verdict: dict, latches: dict, gripper: str, utc: str, stats: dict | None = None,
          prefix: str = "ARM") -> None:
    """Fold this homing into the table (in place) according to `verdict`; every homing joins its arm's history."""
    keys = verdict["keys"]
    entry = history_entry(verdict, utc, stats)
    if verdict["kind"] == "fault":
        table["last_failed"] = {"arm": verdict["arm"], "gripper": gripper, "utc": utc,
                                "latches": {str(j): int(v) for j, v in latches.items()}}
        arm = next(a for a in table["arms"] if a["id"] == verdict["arm"])
        arm["history"] = (arm.get("history", []) + [entry])[-HISTORY:]
        return
    if verdict["kind"] == "new":
        n = 1 + max([int(a["id"].split("-")[1]) for a in table["arms"] if a["id"].startswith("arm-")] or [0])
        joints = {keys[j]: [int(v)] for j, v in latches.items()}
        table["arms"].append({"id": f"arm-{n}", "name": "", "known": False, "homings": 1, "first_utc": utc, "last_utc": utc,
                              "joints": joints, "history": [entry]})
        ensure_uids(table, prefix)
        verdict["arm"] = f"arm-{n}"
        return
    arm = next(a for a in table["arms"] if a["id"] == verdict["arm"])
    arm["history"] = (arm.get("history", []) + [entry])[-HISTORY:]
    if verdict["kind"] == "relearn":
        arm["joints"] = {keys[j]: [int(v)] for j, v in latches.items()}
        arm["homings"] = 1
    else:
        for j, v in latches.items():
            arm["joints"].setdefault(keys[j], []).append(int(v))
            arm["joints"][keys[j]] = arm["joints"][keys[j]][-WINDOW:]
        arm["homings"] += 1
        if not arm["known"] and arm["homings"] >= KNOWN_AFTER:
            arm["known"] = True
    arm["last_utc"] = utc


def describe(table: dict, verdict: dict) -> str:
    """The log line for a verdict (after apply)."""
    arm = next((a for a in table["arms"] if a["id"] == verdict["arm"]), None)
    label = arm_label(arm) if arm else "?"
    kind = verdict["kind"]
    if kind == "known":
        return f"[Homing] arm recognised: {label} — {verdict['agree']}/{verdict['compared']} joints within {TOL_DEG:g} deg of their usual spot"
    if kind == "fault":
        off = ", ".join(f"J{j + 1} {d:+.2f} deg" for j, d in verdict["off"])
        return (f"[Homing] FAILED: {off} away from where arm {label} always homes "
                f"(a false stall, a skipped belt or a slipping coupling?) — home again; if that joint's mechanics were "
                f"changed on purpose: .venv/bin/python tools/homing_arms.py --accept-last")
    if kind == "learning":
        return (f"[Homing] arm {label} — learning its homing spots, {arm['homings']}/{KNOWN_AFTER}"
                + (" — now KNOWN: a joint more than 1 deg off fails homing from now on" if arm["known"] else ""))
    if kind == "relearn":
        off = ", ".join(f"J{j + 1} {d:+.2f} deg" for j, d in verdict["off"])
        return f"[Homing] arm {label} — homed differently than before ({off}) while still learning: learning starts over, 1/{KNOWN_AFTER}"
    return (f"[Homing] no known arm matches — learning a new one, {label}, 1/{KNOWN_AFTER} "
            f"(name it: .venv/bin/python tools/homing_arms.py --name {verdict['arm']} NAME)")


# ---- editing the table (tools/homing_arms.py and the dashboard's System Info card share these) ------------------------

def arm_label(arm: dict) -> str:
    """'grey [PAR6-7F3A9C]' — the name (or, unnamed, the table's id 'arm-1') and the arm's ID."""
    base = arm["name"] if arm.get("name") else arm["id"]
    return f"{base} [{arm['uid']}]" if arm.get("uid") else base


def joint_spots(arm: dict, gears, counts) -> list:
    """[(key, centre ticks, homings, spread deg)] per stored joint of one arm, sorted by key."""
    out = []
    for key, lat in sorted(arm["joints"].items()):
        j = int(key.split("@")[0][1:]) - 1
        c = circular_centre(lat, counts[j])
        out.append((key, c, len(lat), max(abs(off_deg(x, c, counts[j], gears[j])) for x in lat)))
    return out


def last_failed_offs(table: dict, gears, gripper_dependent, counts) -> list:
    """[(joint index, deg)] of the stored last failed homing that were off its arm's usual spots ([] if none)."""
    lf = table.get("last_failed")
    arm = next((a for a in table["arms"] if lf and a["id"] == lf["arm"]), None)
    if arm is None:
        return []
    latches = {int(j): v for j, v in lf["latches"].items()}
    keys = {j: joint_key(j, gripper_dependent, lf["gripper"]) for j in latches}
    return compare(arm, latches, keys, counts, gears)[2]


def joint_stats(arm: dict, gears, counts) -> list[dict]:
    """Per stored joint of one arm: homings in the window, how far the window's latches spread (max, sd, deg), the
    largest distance from the usual spot over the history, and the mean two-pass difference / release relaxation."""
    hist = arm.get("history", [])
    rows = []
    for key, lat in sorted(arm["joints"].items()):
        j = int(key.split("@")[0][1:]) - 1
        name = f"J{j + 1}"
        c = circular_centre(lat, counts[j])
        dev = [off_deg(x, c, counts[j], gears[j]) for x in lat]
        hd = [abs(h["devs"][name]) for h in hist if name in h.get("devs", {})]
        tp = [h["two_pass"][name] for h in hist if name in h.get("two_pass", {})]
        rx = [h["relaxed"][name] for h in hist if name in h.get("relaxed", {})]
        rows.append({"joint": key, "homings": len(lat), "within_deg": max(abs(d) for d in dev),
                     "sd_deg": math.sqrt(sum(d * d for d in dev) / len(dev)),
                     "worst_seen_deg": max(hd) if hd else None,
                     "two_pass_ticks": sum(tp) / len(tp) if tp else None, "relaxed_ticks": sum(rx) / len(rx) if rx else None})
    return rows


def set_learn_next(table: dict, on: bool = True) -> str:
    """'Learn a new arm on the next homing' (2026-10-14): once an arm is known, first joints that match no known arm
    fail homing — unless this is set; the next homing that gets to the end clears it."""
    table["learn_next"] = bool(on)
    return ("the next homing may learn a NEW arm (once) — home the new arm from its park pose" if on
            else "learning a new arm: off")


def approach_spot(arm: dict, key: str):
    """Where joint `key`'s <approach> stops on this arm, relative to its latch (the median of the newest 20 homings that
    recorded it and homed consistently — new / learning / known, not a fault or a relearn — motor ticks) — None until
    APPROACH_MIN homings recorded it."""
    name = key.split("@")[0]
    vals = [e["approach_contact"][name] for e in arm.get("history", [])
            if e.get("kind") in ("new", "learning", "known") and name in (e.get("approach_contact") or {})][-20:]
    vals.sort()
    return vals[len(vals) // 2] if len(vals) >= APPROACH_MIN else None


def rename(table: dict, arm_id: str, name: str) -> str:
    arm = next((a for a in table["arms"] if a["id"] == arm_id), None)
    if arm is None:
        raise KeyError(f"no arm {arm_id}")
    arm["name"] = name.strip()
    return f"{arm_id} is now '{arm['name']}'"


def forget(table: dict, arm_id: str) -> str:
    before = len(table["arms"])
    table["arms"] = [a for a in table["arms"] if a["id"] != arm_id]
    if len(table["arms"]) == before:
        raise KeyError(f"no arm {arm_id}")
    if (table.get("last_failed") or {}).get("arm") == arm_id:
        table["last_failed"] = None
    return f"forgot {arm_id} — it is learned again from scratch the next time it homes"


def accept_last(table: dict, gears, gripper_dependent, counts) -> str:
    """After a DELIBERATE mechanical change: the joints of the last failed homing that were off take its spot as their
    reference (the others keep their history). Returns a report line."""
    lf = table.get("last_failed")
    if not lf:
        return "nothing to accept: no failed homing stored"
    arm = next((a for a in table["arms"] if a["id"] == lf["arm"]), None)
    if arm is None:
        table["last_failed"] = None
        return f"arm {lf['arm']} no longer exists — cleared"
    changed = []
    for j, d in last_failed_offs(table, gears, gripper_dependent, counts):
        arm["joints"][joint_key(j, gripper_dependent, lf["gripper"])] = [int(lf["latches"][str(j)])]
        changed.append(f"J{j + 1} ({d:+.2f} deg)")
    table["last_failed"] = None
    return (f"{arm_label(arm)}: new reference for " + ", ".join(changed)) if changed else "nothing was off — cleared"


def load_table(path) -> dict:
    try:
        with open(path) as fh:
            t = json.load(fh)
        return t if isinstance(t, dict) and isinstance(t.get("arms"), list) else empty_table()
    except (OSError, ValueError):
        return empty_table()


def save_table(path, table: dict) -> None:
    tmp = f"{path}.tmp"
    with open(tmp, "w") as fh:
        json.dump(table, fh, indent=1)
    os.replace(tmp, path)


class HomingArms:
    """RTI's holder: the table in memory, judged on the real-time thread at the end of a homing; the file read and
    written by a worker thread off RTI's core. path=None: memory only (the nets)."""

    def __init__(self, path, gears, gripper_dependent, counts, gripper: str, prefix: str = "ARM"):
        self.path = None if path is None else str(path)
        self.gears = [float(g) for g in gears]
        self.gripper_dependent = [bool(int(f)) for f in gripper_dependent]
        self.counts = [int(c) for c in counts]
        self.gripper = str(gripper)
        self.prefix = str(prefix)
        self.table = load_table(self.path) if self.path else empty_table()
        self._fresh = None
        self._q: queue.Queue = queue.Queue()
        self._thread = None
        self._so_far_arm = None        # check_so_far: the arm recognised from the first joints homed
        self._so_far_off = False       # ... none recognised: nothing judged before the end

    def start(self) -> None:
        if self.path and self._thread is None:
            self._thread = threading.Thread(target=self._run, name="homing-arms", daemon=True)
            self._thread.start()

    def on_homing_start(self) -> None:
        """Pick up `tools/homing_arms.py` edits made since the last homing (read off the RT thread)."""
        self._so_far_arm = None
        self._so_far_off = False
        if self._thread is not None:
            self._q.put(("load", None))

    def check_so_far(self, latches: dict, j: int) -> dict:
        """Mid-homing (2026-10-14, user: "check each joint at once"): joint `j` just homed; `latches` = every arm joint
        homed so far. The arm is recognised as soon as one KNOWN arm has every joint homed so far at its usual spot;
        each joint homed after that must be at that arm's spot too, else {"kind": "fault"} — recorded like the final
        check's fault (last_failed + the history: 'Accept as new reference' works the same). When the first joints
        match no known arm (another arm?) nothing is judged before the end: the final check decides, as before. In
        memory on the RT thread (the save is queued)."""
        fresh, self._fresh = self._fresh, None
        if fresh is not None:
            self.table = fresh
        if self._so_far_off:
            return {"kind": "skip"}
        keys = {i: joint_key(i, self.gripper_dependent, self.gripper) for i in latches}
        if self._so_far_arm is None:
            known = [a for a in self.table["arms"] if a.get("known")]
            closest = None
            for arm in known:
                _, compared, off, devs = compare(arm, latches, keys, self.counts, self.gears)
                if compared and not off:
                    self._so_far_arm = arm["id"]
                    return {"kind": "ok", "text": f"[Homing] arm recognised from the joints homed so far: "
                                                  f"{arm_label(arm)} — each joint homed from now on is checked at once"}
                if compared and (closest is None or max(abs(d) for _, d in off) < closest[0]):
                    closest = (max(abs(d) for _, d in off), arm, off)
            self._so_far_off = True
            if closest is None:                 # still learning the first arm (or nothing comparable): the end check
                return {"kind": "skip"}
            if self.table.get("learn_next"):
                return {"kind": "skip", "text": "[Homing] the first joint(s) homed match none of the known arms — "
                                                "learning a new arm (requested: 'learn a new arm on the next homing')"}
            # 2026-10-14 (user): no longer "another arm?" — a joint that false-stalled first (J1 could not turn in a
            # folded pose) made every later joint unchecked and the sequence went on to the gripper on wrong references
            _, arm, off = closest
            why = (f"homed where no known arm's {', '.join(f'J{i + 1}' for i, _ in off)} homes (closest: "
                   f"{arm_label(arm)}, {', '.join(f'J{i + 1} {d:+.2f} deg' for i, d in off)}) — a false stall (could it "
                   f"move from where it was? the arm touching itself or something?), or a different arm. Move the arm "
                   f"to its park pose and home again; a different arm: System Info > 'Arms (homing)' > 'Learn a new arm "
                   f"on the next homing'")
            return {"kind": "fault", "unknown": True, "off": off, "reason": f"it {why}",
                    "text": f"[Homing] FAILED: {', '.join(f'J{i + 1}' for i, _ in off)} — it {why}"}
        arm = next((a for a in self.table["arms"] if a["id"] == self._so_far_arm), None)
        if arm is None:
            return {"kind": "skip"}
        _, _, off, devs = compare(arm, {j: latches[j]}, keys, self.counts, self.gears)
        if not off:
            return {"kind": "ok"}
        verdict = {"kind": "fault", "arm": arm["id"], "off": off, "devs": devs, "keys": keys}
        apply(self.table, verdict, latches, self.gripper, time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()), {},
              self.prefix)
        if self._thread is not None:
            self._q.put(("save", copy.deepcopy(self.table)))
        d = off[0][1]
        why = (f"it homed {d:+.2f} deg from where arm {arm_label(arm)} always homes — checked as soon as it homed: a "
               f"false stall (could it move? was it at its park pose? something in the way?), a skipped belt or a "
               f"slipping coupling. Move it to its park pose and home again; after a deliberate mechanical change: "
               f"System Info > 'Arms (homing)' > 'Accept as new reference'")
        verdict["reason"] = why
        verdict["text"] = f"[Homing] FAILED: J{j + 1} — {why}"
        return verdict

    def judge_and_learn(self, latches: dict, stats: dict | None = None, utc: str | None = None) -> dict:
        """Called on the real-time thread when the homing sequence is complete: judge, fold in (with homing's own stats
        for the history), queue the save. Returns the verdict with its log line under "text"."""
        fresh, self._fresh = self._fresh, None
        if fresh is not None:
            self.table = fresh
        ensure_uids(self.table, self.prefix)
        utc = utc or time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        verdict = judge(self.table, latches, self.gripper, self.gears, self.gripper_dependent, self.counts)
        learn = bool(self.table.pop("learn_next", False))
        unknown = self._unknown_at_end(latches) if verdict["kind"] == "new" and not learn else None
        if unknown is not None:
            # 2026-10-14 (user): a new arm only on request — here no known arm matches and none was requested
            if self._thread is not None:
                self._q.put(("save", copy.deepcopy(self.table)))
            return unknown
        apply(self.table, verdict, latches, self.gripper, utc, stats, self.prefix)
        verdict["text"] = describe(self.table, verdict)
        if self._thread is not None:
            self._q.put(("save", copy.deepcopy(self.table)))
        return verdict

    def _unknown_at_end(self, latches: dict) -> dict | None:
        """The end check's verdict when no known arm matches and no new arm was requested: FAILED, each joint off the
        closest known arm named (the table is not changed). None when there is no known arm, or the closest one has no
        joint off (too few joints compared to call it that arm): learned as before."""
        keys = {i: joint_key(i, self.gripper_dependent, self.gripper) for i in latches}
        best: tuple | None = None
        for arm in (a for a in self.table["arms"] if a.get("known")):
            agree, _, off, _ = compare(arm, latches, keys, self.counts, self.gears)
            if best is None or agree > best[0]:
                best = (agree, arm, off)
        if best is None or not best[2]:
            return None
        _, arm, off = best
        why = (f"no known arm matches (closest: {arm_label(arm)}, "
               f"{', '.join(f'J{i + 1} {d:+.2f} deg' for i, d in off)}) — false stalls, or a different arm: System "
               f"Info > 'Arms (homing)' > 'Learn a new arm on the next homing'")
        return {"kind": "fault", "unknown": True, "arm": arm["id"], "off": off,
                "reasons": {i: f"it homed {d:+.2f} deg from where the closest known arm's J{i + 1} homes — {why}"
                            for i, d in off}, "text": f"[Homing] FAILED: {why}"}

    def check_approach(self, j: int, arrival: int, toward: int) -> dict:
        """<approach> (2026-10-14, user: J3's approach stopped 7 deg short, against the arm's own base, and homing went
        on): where joint j's approach stopped (motor ticks; `toward` = +1 / -1, the side its endstop is on) against the
        known arms' latch for it — it must not stop SHORT (something in the way): with APPROACH_MIN homings that
        recorded it, not more than APPROACH_TOL_DEG short of the spot it stops at on that arm (the history's median
        offset); before that, not more than APPROACH_TOL_DEG short of the latch (a pressed approach stops ~3 deg BEYOND
        it). Stopping beyond is left to the joint's own arm check (a skipped belt, a deliberate change: recorded there for
        'Accept as new reference'). {"kind": "ok" | "skip" | "fault", "reason", "text"}."""
        fresh, self._fresh = self._fresh, None
        if fresh is not None:
            self.table = fresh
        key = joint_key(j, self.gripper_dependent, self.gripper)
        known = [a for a in self.table["arms"] if a.get("known") and a["joints"].get(key)]
        if not known or self.table.get("learn_next"):      # nothing to compare / a new arm is being learned
            return {"kind": "skip"}
        best: tuple | None = None
        for arm in known:
            centre = circular_centre(arm["joints"][key], self.counts[j])
            spot = approach_spot(arm, key)
            d = off_deg(arrival - (spot or 0), centre, self.counts[j], self.gears[j]) * (1 if toward >= 0 else -1)
            ok, miss = d >= -APPROACH_TOL_DEG, -d
            if ok:
                return {"kind": "ok"}
            if best is None or miss < best[0]:
                best = (miss, d, arm, spot)
        if best is None:
            return {"kind": "skip"}
        _, d, arm, spot = best
        where = ("short of where it always stops" if spot is not None else "short of where its endstop always is")
        why = (f"its controlled approach stopped {abs(d):.1f} deg {where} on arm {arm_label(arm)} — something in the "
               f"way (the arm against itself or its base?), or the joint could not carry the arm there. Move the arm to "
               f"its park pose and home again; a different arm: System Info > 'Arms (homing)' > 'Learn a new arm on the "
               f"next homing'")
        return {"kind": "fault", "reason": f"{why}", "text": f"[Homing] FAILED: J{j + 1} — {why}"}

    def _run(self) -> None:
        from utility.incident_dump import _leave_the_rt_core
        _leave_the_rt_core()
        while True:
            what, table = self._q.get()
            try:
                if what == "load":
                    self._fresh = load_table(self.path)
                elif what == "save":
                    save_table(self.path, table)
            except Exception as exc:                       # never take homing down over a file
                logging.getLogger(__name__).warning("[Homing] arm table %s failed: %s", what, exc)
