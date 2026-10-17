"""Fault injection against the simulated robot (TODO/simulated_robot_plan.md S4, 2026-10-13): each scenario makes
something go wrong — a drive drops off the bus, reports a fault, loses replies, reboots; the executor or RTI dies in
the middle of a motion; the E-stop is pressed (the simulated I/O, hardware/gpio_handler.py); S6 (2026-10-13): the five
drive faults no other scenario raised, and the disturbances of sim/drive_sim.py — a hand pushing a joint past its soft
limit, a runaway, a lagging drive, a buzzing hold — each tripping its safety-monitor check; 2026-10-14: a joint that
cannot reach its endstop while homing (an obstacle: a false stall) — and checks the runtime's reaction in the simulated activity log and the drives' real state
(the simulator's control channel, sim/drives.request). Run by tools/sim_run.py --faults; RTI's death ends the run, so
'rti_lost' runs last.
"""
from __future__ import annotations

import json
import math
import os
import signal
import time

from data.shared_memory_index import ESTOP_1, ESTOP_2
from sim.drives import request
from utility import sim_mode

ACTIVITY = os.path.join("temp_data", "sim", "logs", "activity_rti.log")
ORDER = ("node_lost", "drive_fault", "drive_flags", "reply_loss", "drive_reboot", "hand_push", "push_through",
         "runaway", "tracking_lag", "hold_buzz", "homing_false_stall", "executor_lost", "estop", "rti_lost")
ARMS = os.path.join("temp_data", "sim", "data", "homing_arms.json")    # the simulated arm table (project_paths)
# gpio_states (uint8[16], data/shared_data.py) read as a FILE, with the name prefix shared_data applies — never
# attached (importing data.shared_data attaches every segment). In simulation the inputs live here.
GPIO_SHM = "/dev/shm/" + os.environ.get("RCB_SHM_PREFIX", "") + "GPIO"   # nosec B108 — shared memory, not a temp file


def _all_rtis_simulated(proc_root="/proc"):
    """True when at least one RTI runs and EVERY running RTI has RCB_SIM=1 — the GPIO segment is a simulated one."""
    pids = sim_mode.other_rti_pids(proc_root)
    if not pids:
        return False
    for pid in pids:
        try:
            with open(os.path.join(proc_root, str(pid), "environ"), "rb") as fh:
                if b"RCB_SIM=1" not in fh.read().split(b"\0"):
                    return False
        except OSError:
            return False
    return True


class Context:
    """What a scenario may touch: the robot (a fresh client per call), the simulator's control channel, the run's own
    processes (kill / restart — never anything else), the simulated activity log."""

    def __init__(self, root, kill, restart, wait_executor):
        self.root = root
        self.kill = kill                    # kill(name, sig)
        self.restart = restart              # restart(name)
        self.wait_executor = wait_executor  # wait_executor(timeout) -> bool
        self.homed = False

    def robot(self):
        from robotclient.client import RobotClient
        r = RobotClient(sync=False)
        r.connect("127.0.0.1", 29000)
        return r

    def sim(self, **msg):
        return request(msg)

    def gpio(self, index, value):
        """Set a SIMULATED input (one byte of the GPIO segment). Refused unless every running RTI is a simulated one:
        a real robot's inputs are never written."""
        if not _all_rtis_simulated():
            raise RuntimeError("refusing to write GPIO inputs: the running RTI is not a simulated one")
        fd = os.open(GPIO_SHM, os.O_WRONLY)
        try:
            os.pwrite(fd, bytes([int(value)]), int(index))
        finally:
            os.close(fd)

    def nodes(self):
        return {int(k): v for k, v in self.sim(cmd="state")["nodes"].items()}

    def activity_mark(self):
        p = os.path.join(self.root, ACTIVITY)
        return os.path.getsize(p) if os.path.exists(p) else 0

    def activity_since(self, mark):
        out = []
        try:
            with open(os.path.join(self.root, ACTIVITY)) as fh:
                fh.seek(mark)
                for line in fh:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        pass
        except OSError:
            pass
        return out

    @staticmethod
    def names_joint(entry, joint):
        """The entry is about this joint (1-based): its message is "joint <n>", or "joint <n>: <the values>" — RTI puts
        a fault's measured values after the colon (utility/fault_details.py, 2026-10-15: "joint 1: 38 C"). Its clear
        ("joint <n>: cleared") is not the event."""
        msg, head = entry.get("message") or "", f"joint {joint}"
        return msg == head or (msg.startswith(head + ": ") and msg != head + ": cleared")

    def has(self, mark, code, joint=None):
        """An activity entry with this code since `mark` — for a joint, one that names it (names_joint)."""
        return any(e.get("code") == code and (joint is None or self.names_joint(e, joint))
                   for e in self.activity_since(mark))

    def joint_errors(self, mark, joint):
        return [e.get("code") for e in self.activity_since(mark)
                if e.get("level") == "error" and self.names_joint(e, joint)]

    def ready(self, robot, home=False):
        """Errors cleared, ENABLED (and homed when asked) — the state every motion scenario starts from."""
        robot.clear_errors(wait=True)
        time.sleep(0.3)
        robot.enable(wait=True)
        st = robot.status()
        if home and st.get("homed") is not True:
            robot.home(wait=True)          # waits WAIT_HOMED_TIMEOUT (robotclient)
            self.homed = True
        return robot.status()


_CFG: dict = {}


def _cfg():
    """The active robot's config (read once): ticks per radian, speed ceilings, soft limits — for the S6 scenarios."""
    if not _CFG:
        import config.xml_parser as xp
        from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
        sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
        _CFG.update(xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                              active_gripper=sysc["active_gripper"]))
    return _CFG


def _ticks_per_rad(j):
    c = _cfg()
    return (2 ** float(c["motor_encoder"][j])) * abs(float(c["gear_ratio"][j])) / (2 * math.pi)


def _push_to(ctx, robot, j, target_deg, deg_per_s=15.0, timeout=25.0):
    """Turn joint j (0-based) by HAND (the simulator's push: a limp joint only) until it reads target_deg. The motor's
    direction is found by trying: a short push of +ticks, and whether the joint's ANGLE went up or down."""
    speed = math.radians(deg_per_s) * _ticks_per_rad(j)
    t0 = time.monotonic()
    a0 = float(robot.get_joints()[j])
    ctx.sim(cmd="physics", node=j, kind="push", speed=speed, ms=300)
    time.sleep(0.35)
    a1 = float(robot.get_joints()[j])
    sign = 1.0 if a1 > a0 else -1.0                      # the tick direction that RAISES the angle
    while time.monotonic() - t0 < timeout:
        a = float(robot.get_joints()[j])
        if abs(a - target_deg) < 0.4:
            break
        ctx.sim(cmd="physics", node=j, kind="push", speed=(sign if a < target_deg else -sign) * speed, ms=120)
        time.sleep(0.1)
    ctx.sim(cmd="physics", node=j, kind="clear")
    return float(robot.get_joints()[j])


def _wait(pred, timeout, step=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        v = pred()
        if v:
            return v
        time.sleep(step)
    return pred()


def node_lost(ctx, notes):
    """J3's drive silent 0.6 s (a cable pulled): J3:CAN_LOST latches (an error), then Clear Errors recovers."""
    r = ctx.robot()
    ctx.ready(r)
    mark = ctx.activity_mark()
    ctx.sim(cmd="silence", node=2, ms=600)
    got = _wait(lambda: ctx.has(mark, "CAN_LOST", 3), 2.0)
    _wait(lambda: r.status().get("err") == 1, 1.5)       # status() is the event stream's latest snapshot: it can lag
    st = r.status()
    notes.append(f"J3 CAN_LOST {'latched' if got else 'NOT latched'} (and J3 WATCHDOG: {ctx.has(mark, 'WATCHDOG', 3)} — the silent "
                 f"drive received nothing either); err={st.get('err')} state={st.get('state')}")
    time.sleep(0.8)
    r.clear_errors(wait=True)
    rec = _wait(lambda: r.status().get("err") == 0, 3.0)
    notes.append(f"after the drive answers again + Clear Errors: err={r.status().get('err')}")
    r.disconnect()
    return bool(got) and st.get("err") == 1 and bool(rec)


def drive_fault(ctx, notes):
    """J2's drive reports a driver fault: J2:DRIVER latches; cleared once the fault is gone and Clear Errors."""
    r = ctx.robot()
    ctx.ready(r)
    mark = ctx.activity_mark()
    ctx.sim(cmd="fault", node=1, kind="driver")
    got = _wait(lambda: ctx.has(mark, "DRIVER", 2), 2.0)
    _wait(lambda: r.status().get("err") == 1, 1.5)
    st = r.status()
    notes.append(f"J2:DRIVER {'latched' if got else 'NOT latched'}; err={st.get('err')}")
    ctx.sim(cmd="fault", node=1, kind="clear")
    time.sleep(0.2)
    r.clear_errors(wait=True)
    rec = _wait(lambda: r.status().get("err") == 0, 3.0)
    notes.append(f"fault gone + Clear Errors: err={r.status().get('err')}")
    r.disconnect()
    return bool(got) and st.get("err") == 1 and bool(rec)


def reply_loss(ctx, notes):
    """J6's drive loses every 2nd reply for 5 s: J6:CAN_REPLY_MISSED_WARNING (warn only — no error, the arm keeps
    working)."""
    r = ctx.robot()
    ctx.ready(r)
    mark = ctx.activity_mark()
    ctx.sim(cmd="drop", node=5, every=2, ms=5000)
    got = _wait(lambda: ctx.has(mark, "CAN_REPLY_MISSED_WARNING", 6), 6.0, step=0.2)
    st = r.status()
    dropped = ctx.nodes()[5]["dropped"]
    notes.append(f"J6:CAN_REPLY_MISSED_WARNING {'raised' if got else 'NOT raised'}; {dropped} replies dropped; "
                 f"err={st.get('err')}")
    time.sleep(2.0)                              # let the drop window end before the next scenario
    r.disconnect()
    return bool(got) and st.get("err") == 0


def drive_reboot(ctx, notes):
    """J4's drive reboots (0.15 s without power: its config gone): RTI's reconnect reload must re-send it — the
    watchdog 100 ms and J4's current limit back on the drive — and nothing latches (a short stale only)."""
    r = ctx.robot()
    ctx.ready(r)
    before = ctx.nodes()[3]
    mark = ctx.activity_mark()
    ctx.sim(cmd="reboot", node=3, ms=150)
    _wait(lambda: not ctx.nodes()[3]["silent"], 1.0)
    after_boot = ctx.nodes()[3]
    restored = _wait(lambda: ctx.nodes()[3]["wd_time_ms"] == before["wd_time_ms"]
                     and ctx.nodes()[3]["current_limit"] == before["current_limit"], 2.0)
    now = ctx.nodes()[3]
    st = r.status()
    errs = ctx.joint_errors(mark, 4)
    notes.append(f"right after the reboot: watchdog {after_boot['wd_time_ms']} ms, current limit "
                 f"{after_boot['current_limit']:.0f}; after RTI's reload: {now['wd_time_ms']} ms, "
                 f"{now['current_limit']:.0f} mA (before: {before['wd_time_ms']} ms, {before['current_limit']:.0f}); "
                 f"err={st.get('err')} {errs or ''}")
    r.disconnect()
    return after_boot["wd_time_ms"] == 0 and bool(restored) and st.get("err") == 0 and not errs


def executor_lost(ctx, notes):
    """The executor killed in the middle of an EXEC move: RTI trips EXEC_LINK_LOST (disables and holds), the arm stops;
    the executor restarted, the robot recovers."""
    r = ctx.robot()
    ctx.ready(r, home=True)
    r.enter_exec_mode(wait=True)
    q = [float(v) for v in r.get_joints()]
    q[0] += 25.0
    p_start = ctx.nodes()[0]["position"]
    r.move_joints(*q, speed=30)                   # ~3 s; a slow move's jerk-limited start barely moves for seconds
    _wait(lambda: abs(ctx.nodes()[0]["position"] - p_start) > 500, 6.0)
    p0 = ctx.nodes()[0]["position"]
    time.sleep(0.2)
    moving = abs(ctx.nodes()[0]["position"] - p0) > 20.0
    mark = ctx.activity_mark()
    ctx.kill("executor", signal.SIGKILL)
    got = _wait(lambda: ctx.has(mark, "EXEC_LINK_LOST"), 3.0)
    p1 = ctx.nodes()[0]["position"]
    time.sleep(0.5)
    p2 = ctx.nodes()[0]["position"]
    notes.append(f"J1 moving before the kill: {moving}; EXEC_LINK_LOST {'latched' if got else 'NOT latched'}; "
                 f"J1 after: {abs(p2 - p1):.1f} ticks in 0.5 s")
    ctx.restart("executor")
    up = ctx.wait_executor(60.0)
    ok_rec = False
    if up:
        r2 = ctx.robot()
        st = ctx.ready(r2)
        ok_rec = st.get("err") == 0 and st.get("state") == "ENABLED"
        notes.append(f"executor restarted: err={st.get('err')} state={st.get('state')} homed={st.get('homed')}")
        r2.disconnect()
    else:
        notes.append("the executor did not come back")
    return moving and bool(got) and abs(p2 - p1) < 5.0 and ok_rec


def estop(ctx, notes):
    """The E-stop pressed in the middle of an EXEC move (the simulated I/O, both channels): ESTOP latches, the robot
    disables and J1 stops; Clear Errors does NOT clear it while the button is pressed; released + Clear Errors +
    Enable, the robot works again."""
    r = ctx.robot()
    try:
        ctx.ready(r, home=True)
        r.enter_exec_mode(wait=True)
        q = [float(v) for v in r.get_joints()]
        q[0] += 25.0
        p_start = ctx.nodes()[0]["position"]
        r.move_joints(*q, speed=30)
        _wait(lambda: abs(ctx.nodes()[0]["position"] - p_start) > 500, 6.0)
        p0 = ctx.nodes()[0]["position"]
        time.sleep(0.2)
        moving = abs(ctx.nodes()[0]["position"] - p0) > 20.0
        mark = ctx.activity_mark()
        ctx.gpio(ESTOP_1, 0)                          # pressed = LOW, both channels as a real button does
        ctx.gpio(ESTOP_2, 0)
        got = _wait(lambda: ctx.has(mark, "ESTOP"), 2.0)
        _wait(lambda: r.status().get("err") == 1, 1.5)
        p1 = ctx.nodes()[0]["position"]
        time.sleep(0.5)
        p2 = ctx.nodes()[0]["position"]
        st = r.status()
        r.clear_errors(wait=False)                    # still pressed: must not clear
        time.sleep(0.5)
        held = r.status().get("err") == 1
        notes.append(f"J1 moving before: {moving}; ESTOP {'latched' if got else 'NOT latched'}; state={st.get('state')} "
                     f"err={st.get('err')}; J1 after: {abs(p2 - p1):.1f} ticks in 0.5 s; Clear Errors while pressed "
                     f"{'refused (err stays 1)' if held else 'CLEARED IT'}")
        ctx.gpio(ESTOP_1, 1)
        ctx.gpio(ESTOP_2, 1)
        time.sleep(0.2)
        r.clear_errors(wait=True)
        rec = _wait(lambda: r.status().get("err") == 0, 3.0)
        r.enable(wait=True)
        st2 = r.status()
        notes.append(f"released + Clear Errors + Enable: err={st2.get('err')} state={st2.get('state')} "
                     f"homed={st2.get('homed')}")
        return (moving and bool(got) and st.get("state") == "DISABLED" and abs(p2 - p1) < 5.0 and held
                and bool(rec) and st2.get("state") == "ENABLED")
    finally:
        try:
            ctx.gpio(ESTOP_1, 1)                      # never leave the simulated button pressed
            ctx.gpio(ESTOP_2, 1)
        except (OSError, RuntimeError):
            pass
        r.disconnect()


def drive_flags(ctx, notes):
    """The five drive faults no other scenario raises — temperature, encoder, vbus, velocity, current — each on its own
    joint: the matching J{n}:<FAULT> latches (the robot disables), and clears once the fault is gone and Clear Errors."""
    r = ctx.robot()
    ok = True
    try:
        for kind, node in (("temperature", 0), ("encoder", 1), ("vbus", 2), ("velocity", 3), ("current", 4)):
            ctx.ready(r)
            mark = ctx.activity_mark()
            ctx.sim(cmd="fault", node=node, kind=kind)
            got = _wait(lambda k=kind, n=node: ctx.has(mark, k.upper(), n + 1), 2.0)
            ctx.sim(cmd="fault", node=node, kind="clear")
            time.sleep(0.2)
            r.clear_errors(wait=True)
            rec = _wait(lambda: r.status().get("err") == 0, 3.0)
            notes.append(f"J{node + 1}:{kind.upper()} {'latched' if got else 'NOT latched'}; after Clear Errors "
                         f"err={r.status().get('err')}")
            ok = ok and bool(got) and bool(rec)
    finally:
        for node in range(5):
            ctx.sim(cmd="fault", node=node, kind="clear")
        r.disconnect()
    return ok


def hand_push(ctx, notes):
    """A hand turns J5 past its soft limit in freedrive (IDLE, the joint limp): J5:SOFT_LIMIT_WARNING — a warning, never
    an error (the robot commands no motion there) — and turned back inside it clears; then the robot can move again."""
    r = ctx.robot()
    try:
        ctx.ready(r, home=True)
        r.enter_idle_mode(wait=True)
        lim = math.degrees(float(_cfg()["software_joint_limit_positive"][4]))
        mark = ctx.activity_mark()
        past = _push_to(ctx, r, 4, lim + 1.5)
        warned = _wait(lambda: ctx.has(mark, "SOFT_LIMIT_WARNING", 5), 2.0)
        err = r.status().get("err")
        inside = _push_to(ctx, r, 4, lim - 4.0)
        time.sleep(0.5)
        live = [e for e in (r.status().get("errors") or []) if "SOFT_LIMIT_WARNING" in str(e)]
        notes.append(f"J5 pushed to {past:.1f} deg (soft limit {lim:.1f}): SOFT_LIMIT_WARNING "
                     f"{'raised' if warned else 'NOT raised'}, err={err}; back to {inside:.1f}: still live: {bool(live)}")
        return bool(warned) and err == 0 and past > lim + 0.5 and inside < lim and not live
    finally:
        ctx.sim(cmd="physics", node=4, kind="clear")
        r.disconnect()


def push_through(ctx, notes):
    """A hand pushes J5 through everything freedrive has (IDLE): past its soft limit -> SOFT_LIMIT_WARNING, through its
    wall's breakthrough -> JOINT_WALL_WARNING (both warnings, no stop), past its hard limit + the endstop tolerance ->
    BEYOND_ENDSTOP (an error: the robot disables and homing is invalidated). Pushed back, cleared and re-homed, the
    robot works. (Found by accident 2026-10-13, when a scenario pushed the wrong way — the stack behaved; now pinned.)"""
    r = ctx.robot()
    try:
        ctx.ready(r, home=True)
        r.enter_idle_mode(wait=True)
        c = _cfg()
        soft, hard = (math.degrees(float(c["software_joint_limit_positive"][4])),
                      math.degrees(float(c["joint_limit_positive"][4])))
        wall = soft + min(hard - soft, 15.0)                  # the auto breakthrough (docs/safety_zones.md §2b)
        mark = ctx.activity_mark()
        _push_to(ctx, r, 4, soft + 2.0)
        w1 = _wait(lambda: ctx.has(mark, "SOFT_LIMIT_WARNING", 5), 2.0)
        _push_to(ctx, r, 4, wall + 2.0)
        w2 = _wait(lambda: ctx.has(mark, "JOINT_WALL_WARNING", 5), 2.0)
        err_mid = r.status().get("err")
        _push_to(ctx, r, 4, hard + 12.0)
        e3 = _wait(lambda: ctx.has(mark, "BEYOND_ENDSTOP", 5), 2.0)
        _wait(lambda: r.status().get("err") == 1, 2.0)
        st = r.status()
        _push_to(ctx, r, 4, soft - 5.0)
        r.clear_errors(wait=True)
        st2 = ctx.ready(r, home=True)
        notes.append(f"soft {soft:.1f} / wall {wall:.1f} / hard {hard:.1f} deg: SOFT_LIMIT_WARNING {w1}, JOINT_WALL_WARNING "
                     f"{w2} (err {err_mid}), BEYOND_ENDSTOP {e3} (err {st.get('err')}, homed {st.get('homed')}); "
                     f"pushed back, cleared, re-homed: err={st2.get('err')} homed={st2.get('homed')}")
        return (bool(w1) and bool(w2) and err_mid == 0 and bool(e3) and st.get("err") == 1 and st.get("homed") is not True
                and st2.get("err") == 0 and st2.get("homed") is True)
    finally:
        ctx.sim(cmd="physics", node=4, kind="clear")
        r.disconnect()


def runaway(ctx, notes):
    """J6 runs away for 0.15 s at 1.6x its speed ceiling while EXEC holds it (a drive moving on its own):
    J6:OVERSPEED_WARNING (the shipped mode is warn — no stop), and the hold brings it back."""
    r = ctx.robot()
    try:
        ctx.ready(r, home=True)
        r.enter_exec_mode(wait=True)
        a0 = float(r.get_joints()[5])
        speed = 1.6 * float(_cfg()["joint_velocity_limit"][5]) * _ticks_per_rad(5)
        mark = ctx.activity_mark()
        ctx.sim(cmd="physics", node=5, kind="runaway", speed=speed, ms=150)
        got = _wait(lambda: ctx.has(mark, "OVERSPEED_WARNING", 6), 2.0)
        back = _wait(lambda: abs(float(r.get_joints()[5]) - a0) < 0.5, 5.0)
        st = r.status()
        notes.append(f"J6:OVERSPEED_WARNING {'raised' if got else 'NOT raised'}; back at its hold: {bool(back)}; "
                     f"err={st.get('err')}")
        return bool(got) and bool(back) and st.get("err") == 0
    finally:
        ctx.sim(cmd="physics", node=5, kind="clear")
        r.disconnect()


def tracking_lag(ctx, notes):
    """J4's drive follows its position target with a 0.3 s lag during an EXEC move (a weak or saturated drive): the
    tracking error grows past J4's limit -> J4:TRACKING_WARNING; the same move without the lag raises none."""
    r = ctx.robot()
    try:
        ctx.ready(r, home=True)
        r.enter_exec_mode(wait=True)
        q = [float(v) for v in r.get_joints()]
        mark = ctx.activity_mark()
        r.move_joints(*[v + (20.0 if k == 3 else 0.0) for k, v in enumerate(q)], speed=50)
        r.wait_idle(timeout=30)
        clean = not ctx.has(mark, "TRACKING_WARNING", 4)
        mark = ctx.activity_mark()
        ctx.sim(cmd="physics", node=3, kind="lag", tau=0.3, ms=15000)
        r.move_joints(*q, speed=50)
        r.wait_idle(timeout=30, raise_on_error=False)
        got = _wait(lambda: ctx.has(mark, "TRACKING_WARNING", 4), 2.0)
        st = r.status()
        notes.append(f"J4 +20 deg at 50 % without a lag: TRACKING_WARNING {'none' if clean else 'RAISED'}; back with a "
                     f"0.3 s lag: {'raised' if got else 'NOT raised'}; err={st.get('err')}")
        return clean and bool(got) and st.get("err") == 0
    finally:
        ctx.sim(cmd="physics", node=3, kind="clear")
        r.disconnect()


def hold_buzz(ctx, notes):
    """J3's measured current swings +-300 mA at 60 Hz while EXEC holds it still (the whine of too-high gains):
    J3:HOLD_OSCILLATION_WARNING after the monitor's 1.5 s sustain — a warning, never a stop; none before 1 s."""
    r = ctx.robot()
    try:
        ctx.ready(r, home=True)
        r.enter_exec_mode(wait=True)
        time.sleep(0.5)
        mark = ctx.activity_mark()
        t0 = time.monotonic()
        ctx.sim(cmd="physics", node=2, kind="buzz", ma=300.0, hz=60.0, ms=4000)
        time.sleep(1.0)
        early = ctx.has(mark, "HOLD_OSCILLATION_WARNING", 3)
        got = _wait(lambda: ctx.has(mark, "HOLD_OSCILLATION_WARNING", 3), 4.0)
        after = time.monotonic() - t0
        st = r.status()
        notes.append(f"J3:HOLD_OSCILLATION_WARNING {'raised' if got else 'NOT raised'} ~{after:.1f} s in (none at 1 s: "
                     f"{not early}); err={st.get('err')}")
        return bool(got) and not early and st.get("err") == 0
    finally:
        ctx.sim(cmd="physics", node=2, kind="clear")
        r.disconnect()


def _known_arm(ctx):
    """True when the simulated arm table holds a KNOWN arm (robotics/homing_arms.py: learned after 3 homings)."""
    try:
        with open(os.path.join(ctx.root, ARMS)) as fh:
            return any(a.get("known") for a in json.load(fh).get("arms", []))
    except (OSError, ValueError):
        return False


def _alias_deg(ctx, j):
    """Where joint j's drive is now against its endstop, in joint degrees: (all the way, modulo one motor revolution).
    The arm check compares latches modulo a revolution, so an obstacle that far is all it can see of a false stall."""
    n = ctx.nodes()[j]
    rev = 2.0 ** float(_cfg()["motor_encoder"][j])
    per_tick = 360.0 / rev / abs(float(_cfg()["gear_ratio"][j]))
    d = abs(n["position"] - n["stop"])
    return d * per_tick, min(d % rev, rev - d % rev) * per_tick


def homing_false_stall(ctx, notes):
    """J2 cannot reach its endstop while homing — an obstacle where it starts (2026-10-14, the user's case: J2 left
    away from its park pose, a load the homing current cannot lift; or something in the way). It stalls there, backs
    off, stalls there again: the two passes agree, so only the arm check can tell. With a known arm J2 must FAIL as soon
    as it homes — J2:HOMING_FAILED latched as an ERROR (disabled, in the status' errors), its reason in the activity
    log, not homed; then the obstacle gone + Clear Errors + Enable + Home -> homed. Starts from a fresh homing, so the
    obstacle sits at the homing sequence's own end pose (deterministic)."""
    r = ctx.robot()
    try:
        ctx.ready(r)
        for n in range(1, 4):                            # the end pose; on a fresh tree the arm table learns the arm
            r.enter_idle_mode(wait=True)                 # homing starts from IDLE (a scenario before may leave EXEC)
            r.home(wait=True)
            if _known_arm(ctx):
                break
            notes.append(f"homing {n}: the arm table is still learning the simulated arm")
            ctx.ready(r)
        if not _known_arm(ctx):
            notes.append("no KNOWN arm in the simulated arm table after 3 homings — the check cannot run")
            return False
        full, seen = _alias_deg(ctx, 1)
        notes.append(f"J2's obstacle: {full:.1f} deg before its endstop — {seen:.2f} deg modulo a motor revolution "
                     f"(what the arm check sees; > 1 deg needed)")
        ctx.ready(r)
        r.enter_idle_mode(wait=True)
        mark = ctx.activity_mark()
        ctx.sim(cmd="physics", node=1, kind="block", ms=600000)
        r.home(wait=False)
        if not _wait(lambda: r.status().get("mode") == "HOMING", 3.0):
            notes.append(f"the homing never started (mode {r.status().get('mode')})")
            return False
        failed = _wait(lambda: [e for e in ctx.activity_since(mark) if e.get("code") == "HOMING_FAILED"], 90.0)
        _wait(lambda: r.status().get("err") == 1 and r.status().get("mode") != "HOMING", 2.0)
        st = r.status()
        msg = failed[0].get("message", "") if failed else ""
        notes.append(f"activity: {failed[0].get('level') if failed else None} HOMING_FAILED '{msg[:150]}'")
        notes.append(f"status: err={st.get('err')} state={st.get('state')} mode={st.get('mode')} "
                     f"homed={st.get('homed')} errors={st.get('errors')}")
        caught = (bool(failed) and failed[0].get("level") == "error" and msg.startswith("joint 2:")
                  and "checked as soon as it homed" in msg and st.get("err") == 1 and st.get("homed") is not True
                  and st.get("state") != "ENABLED" and "J2:HOMING_FAILED" in (st.get("errors") or []))
    finally:
        ctx.sim(cmd="physics", node=1, kind="clear")
    r.clear_errors(wait=True)
    time.sleep(0.3)
    r.enable(wait=True)
    r.enter_idle_mode(wait=True)
    r.home(wait=True)
    st = r.status()
    notes.append(f"obstacle gone + Clear Errors + Enable + Home: homed={st.get('homed')} err={st.get('err')}")
    r.disconnect()
    return caught and seen > 1.0 and st.get("homed") is True and st.get("err") == 0


def rti_lost(ctx, notes):
    """RTI killed while J1 approaches its endstop in SPEED mode (homing): every drive's CAN watchdog (100 ms) must trip
    ~100 ms after the last command, and J1 stop there — not run on to the endstop (with 5 s it would have)."""
    r = ctx.robot()
    ctx.ready(r)
    r.home(wait=False)
    # J1 homes after the first step's pre-moves — J3's <approach> can take up to its 10 s timeout (2026-10-14)
    _wait(lambda: abs(ctx.nodes()[0]["speed"]) > 1000, 20.0)
    speed = ctx.nodes()[0]["speed"]
    ctx.kill("RTI", signal.SIGKILL)
    time.sleep(0.6)
    n = ctx.nodes()
    time.sleep(0.3)
    n2 = ctx.nodes()
    rows, ok = [], True
    for k in sorted(n):
        d = n[k]
        gap = None if d["trip_ms"] is None or d["last_cmd_ms"] is None else d["trip_ms"] - d["last_cmd_ms"]
        good = gap is not None and 100 <= gap <= 110 and d["watchdog_error"]
        ok = ok and good
        rows.append(f"node {k}: trip {gap} ms after its last command{'' if good else '  <-- WRONG'}")
    still = abs(n2[0]["position"] - n[0]["position"])
    notes.append(f"J1 at {speed:.0f} ticks/s when RTI died; after the trip it moved {still:.1f} ticks in 0.3 s")
    notes.extend(rows)
    r.disconnect()
    return ok and still < 1.0 and abs(speed) > 1000 and not math.isnan(speed)


SCENARIOS = {f.__name__: f for f in (node_lost, drive_fault, drive_flags, reply_loss, drive_reboot, hand_push,
                                     push_through, runaway, tracking_lag, hold_buzz, homing_false_stall, executor_lost,
                                     estop, rti_lost)}
