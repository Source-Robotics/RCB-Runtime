"""Fault injection against the simulated robot (TODO/simulated_robot_plan.md S4, 2026-10-13): each scenario makes
something go wrong — a drive drops off the bus, reports a fault, loses replies, reboots; the executor or RTI dies in
the middle of a motion; the E-stop is pressed (the simulated I/O, hardware/gpio_handler.py) — and checks the runtime's reaction in the simulated activity log and the drives' real state
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
ORDER = ("node_lost", "drive_fault", "reply_loss", "drive_reboot", "executor_lost", "estop", "rti_lost")
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

    def has(self, mark, code, joint=None):
        """An activity entry with this code since `mark` — for a joint, its message is "joint <n>" (1-based)."""
        return any(e.get("code") == code and (joint is None or e.get("message") == f"joint {joint}")
                   for e in self.activity_since(mark))

    def joint_errors(self, mark, joint):
        return [e.get("code") for e in self.activity_since(mark)
                if e.get("level") == "error" and e.get("message") == f"joint {joint}"]

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


def rti_lost(ctx, notes):
    """RTI killed while J1 approaches its endstop in SPEED mode (homing): every drive's CAN watchdog (100 ms) must trip
    ~100 ms after the last command, and J1 stop there — not run on to the endstop (with 5 s it would have)."""
    r = ctx.robot()
    ctx.ready(r)
    r.home(wait=False)
    _wait(lambda: abs(ctx.nodes()[0]["speed"]) > 1000, 8.0)
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


SCENARIOS = {f.__name__: f for f in (node_lost, drive_fault, reply_loss, drive_reboot, executor_lost, estop,
                                     rti_lost)}
