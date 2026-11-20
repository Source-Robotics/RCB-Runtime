"""S5 — the homed arm (2026-10-27, TODO/first_run_plan.md P3; user: the scale of motion, the gripper + a move-and-stop,
friction + quiet holds — all four; 2026-11-20 the user removed S5.joints, the scale / friction / holding turn of every
joint: "i dont want these steps at all". The gear-ratio question it asked goes to mastering, TODO/mastering_plan.md).

The arm homed and ENABLED; every move is a normal EXEC move (move_joints), so the planner's limit / zone /
self-collision checks guard it.

    S5.gripper  open, close (the user confirms each), the jaws left open.
    S5.stop     J1 starts a slow move, a Stop comes mid-move: it must stop short, stay in EXEC and hold; then back.
"""
from __future__ import annotations

import math

from first_run import shm_files
from first_run.core import FAIL, PASS, SKIP, WARN, Aborted, Cancelled, claim, result, step
from robotclient import CommandRejected, RobotTimeout

SPEED_PCT = 10.0                 # every move at 10 % of the EXEC speed limits
LEG_TIMEOUT_S = 60.0
NOT_PLANNED = ("JOINT_LIMIT", "SELF_COLLISION", "ZONE_LIMIT", "ZONE_CHECK_UNAVAILABLE")   # refused before it moved
STOP_DEG = 30.0                  # S5.stop's J1 move (2026-11-19, review K8: was 15)
STOP_AT_DEG = 10.0               # the stop comes once J1 has turned this far (a third of the move) ...
STOP_WAIT_S = 20.0               # ... or this long after the move was sent, whichever comes first (2026-11-20, the arm:
#                                  the move at 10 % is planned jerk-limited as ~8.4 s; the old fixed 0.6 s came when J1
#                                  had turned 0.05 deg, so the stop was never tested)
WATCH_S = 0.05
HOLD_DRIFT_DEG = 0.5
STOP_MIN_MOVED_DEG = 0.5     # less when the stop came: J1 had not really started, the stop was not tested
EXEC_SETTLE_S = 0.5          # the stop test's move starts this long after EXEC is entered (2026-11-19: sent 43 ms
#                              after the mode change, J1 had moved 0.04 deg when the stop came 0.6 s later; 2026-11-20:
#                              not the cause — the move's slow jerk-limited start was, see STOP_WAIT_S)
FOLLOW_STOP_DEG = 2.0        # the command moved this much more than the measured J1 at the stop: J1 did not follow
GRIPPER_TIMEOUT_S = 30.0


def _shm(ctx):
    return ctx.get("shm") or shm_files


def _pause(ui, prompt, seconds, interval=0.25):
    """A deliberate hold through the UI's wait (so Ctrl-C still skips): it ends by its own count of polls, never as a
    "timed out"."""
    left = [int(round(seconds / interval))]

    def poll():
        left[0] -= 1
        return True if left[0] < 0 else None
    ui.wait(prompt, poll, seconds + 5.0, interval)


def _joints_deg(robot):
    return [float(v) for v in robot.get_joints()]


def _homed_exec(ctx, ui, st, ask):
    """Common start: homed, IDLE or EXEC, the user's go, control, EXEC. None to go on, else the result to return."""
    robot = ctx["robot"]
    status = robot.get_status() or {}
    if not status.get("homed"):
        return result(st, SKIP, "needs a homed arm (stage 4)")
    if status.get("mode") not in ("IDLE", "EXEC"):
        return result(st, SKIP, f"needs IDLE (the robot is in {status.get('mode')}: clear its errors first)")
    if ui.choose(ask, ["start", "skip"]) != "start":
        return result(st, SKIP, "not started (the user's choice)")
    why = claim(ctx, ui)
    if why:
        return result(st, SKIP, why)
    if status.get("mode") != "EXEC":
        try:
            robot.enter_exec_mode(wait=True)
        except CommandRejected as e:
            return result(st, SKIP, f"the robot refused EXEC mode: {e.message}")
    return None


def _clear_refusal(robot):
    """A planned move the robot refused aborts the program (every later move MOTION_ABORTED until clear_errors, as on
    the real executor): cleared before the other way is tried (2026-11-19: it was not — the other way always failed)."""
    try:
        robot.clear_errors(wait=True)
    except Exception:                  # noqa: BLE001 — the next move then names what is wrong
        pass


def _commanded_j(ctx, j):
    """J(j+1)'s COMMANDED position in degrees (shared memory read as a file, this Pi), or None."""
    v = _shm(ctx).floats("joint_position_commanded", j + 1)
    return None if not v or len(v) <= j else math.degrees(v[j])


def _leave_exec(robot):
    try:
        robot.enter_idle_mode(wait=True)
    except Exception:                  # a stage end must never hide its results
        pass


def check_gripper(ctx, ui):
    st = GRIPPER
    if not (ctx.get("config") or {}).get("can_gripper"):
        return result(st, SKIP, "no gripper on CAN (stage 0's configuration)")
    stop = _homed_exec(ctx, ui, st, "The gripper now closes, then opens. Keep fingers away from the jaws.")
    if stop is not None:
        return stop
    robot, out = ctx["robot"], []
    try:
        for verb, call in (("close", robot.close_gripper), ("open", robot.open_gripper)):
            try:
                call()
                robot.wait_gripper_move_complete(timeout=GRIPPER_TIMEOUT_S)
            except CommandRejected as e:
                out.append(result(st, FAIL, f"the gripper did not {verb} ({e.code}): {e.message}", value=e.code,
                                  hints=["the jaws move freely, nothing between them",
                                         "the gripper's power and CAN cable; its calibration (homing does it)"]))
                break
            except RobotTimeout:
                try:
                    robot.stop(wait=True)     # the jaws held where they are
                except Exception:
                    pass
                out.append(result(st, FAIL, f"the gripper did not finish its {verb} within {GRIPPER_TIMEOUT_S:.0f} s "
                                            "(stopped)", hints=["the jaws move freely, nothing between them",
                                                                "the gripper's power and CAN cable; its calibration "
                                                                "(homing does it)"]))
                break
            try:
                seen = ui.choose(f"Did the jaws {verb} fully?", ["yes", "no"])
            except Aborted as e:              # the run stops here: keep what the gripper showed so far
                e.partial = out
                raise
            if seen == "yes":
                out.append(result(st, PASS, f"the jaws {verb}"))
            else:
                out.append(result(st, FAIL, f"the jaws did not {verb} fully (the robot says the move finished)",
                                  hints=["something between the jaws, or the jaws binding on their rail",
                                         "the gripper XML's open / closed positions for this gripper"]))
    finally:
        _leave_exec(robot)
    return out


def _watch_j1(ui, robot, j1_start):
    """While J1's move runs: "at" once J1 has turned STOP_AT_DEG from j1_start, "done" when the move finished first,
    None after STOP_WAIT_S (the stop comes anyway; the result says how far J1 got). A failure of the move raises its
    CommandRejected after the wait (a poll that raises would leave the dashboard's wait open); a skip, Cancelled."""
    failed: list = []

    def poll():
        try:
            robot.wait_motion_complete(timeout=0.01)
            return "done"
        except RobotTimeout:
            pass
        except CommandRejected as e:
            failed.append(e)
            return "failed"
        return "at" if abs(_joints_deg(robot)[0] - j1_start) >= STOP_AT_DEG else None

    seen = ui.wait(f"J1 turns — the stop comes once it has turned {STOP_AT_DEG:g} deg", poll, STOP_WAIT_S, WATCH_S)
    if failed:
        raise failed[0]
    return seen


def check_stop(ctx, ui):
    st = STOP
    stop = _homed_exec(ctx, ui, st, f"J1 now starts a slow {STOP_DEG:g} deg move and is stopped part-way (as a "
                                    "program's stop() does), then returns. Clear the space around the arm and keep a "
                                    "hand on the E-stop.")
    if stop is not None:
        return stop
    robot = ctx["robot"]
    try:
        _pause(ui, "EXEC settles before the move", EXEC_SETTLE_S)
        q0 = _joints_deg(robot)
        c0 = _commanded_j(ctx, 0)
        for sign in (1, -1):
            target = list(q0)
            target[0] += sign * STOP_DEG
            try:
                robot.move_joints(*target, speed=SPEED_PCT)    # refused at once (MOTION_ABORTED, ...) is caught too
                seen = _watch_j1(ui, robot, q0[0])
            except Cancelled:
                robot.stop(wait=True)
                return result(st, SKIP, "skipped by the user while J1 turned (stopped where it was; not judged)")
            except CommandRejected as e:
                if e.code in NOT_PLANNED and sign > 0:
                    _clear_refusal(robot)
                    continue              # no room the + way: the other way
                return result(st, SKIP, f"J1's move was not possible ({e.code}): {e.message}")
            if seen == "done":
                return result(st, WARN, "J1's move finished before the stop could come — the stop was not tested",
                              hints=[f"J1 finished its {STOP_DEG:g} deg before it had turned {STOP_AT_DEG:g} deg: "
                                     "it moved the other way or stopped short — the dashboard's Activity Log tab"])
            break
        c_stop = _commanded_j(ctx, 0)
        robot.stop(wait=True)
        s = robot.get_status() or {}
        at = _joints_deg(robot)
        moved = abs(at[0] - q0[0])
        commanded = abs(c_stop - c0) if c0 is not None and c_stop is not None else None
        try:
            _pause(ui, "J1 holds after the stop", 1.0)
        except Cancelled:
            return result(st, SKIP, f"J1 stopped after {moved:.1f} of {STOP_DEG:g} deg; the hold after it was skipped by "
                                    "the user (J1 stays where it stopped)", value=round(moved, 2))
        later = _joints_deg(robot)
        drift = abs(later[0] - at[0])
        if s.get("mode") != "EXEC":
            out = result(st, FAIL, f"after the stop the robot is in {s.get('mode')}, not holding in EXEC",
                         value=s.get("mode"), hints=["the dashboard's Errors tab: an error latched with the stop"])
        elif drift > HOLD_DRIFT_DEG:
            out = result(st, FAIL, f"J1 drifted {drift:.2f} deg after the stop", value=round(drift, 3),
                         hints=["a load the joint cannot hold, or a slipping coupling / belt"])
        elif moved >= STOP_DEG - 0.5:
            out = result(st, WARN, f"J1 reached its target ({moved:.1f} deg) — the stop came too late to show",
                         value=round(moved, 2))
        elif commanded is not None and commanded - moved > FOLLOW_STOP_DEG:
            out = result(st, FAIL, f"J1 was commanded {commanded:.1f} deg but moved only {moved:.2f} deg when the stop "
                                   "came — it did not follow its command", value=round(moved, 2),
                         expected=f"within {FOLLOW_STOP_DEG:g} deg of its command",
                         hints=["the dashboard's Errors tab: a drive fault or a tracking error",
                                "J1's coupling / belt slipping, or the joint blocked"])
        elif moved < STOP_MIN_MOVED_DEG:
            told = (f" (its command had moved {commanded:.2f} deg: the move had not started playing)"
                    if commanded is not None else "")
            out = result(st, WARN, f"J1 had moved only {moved:.2f} deg when the stop came — the stop was not "
                                   f"tested{told}",
                         value=round(moved, 2), hints=[f"J1 did not turn within {STOP_WAIT_S:g} s of its move: the "
                                                       "move did not start (refused, an error) or J1 is blocked — the "
                                                       "dashboard's Errors / Activity Log tab"])
        else:
            out = result(st, PASS, f"J1 stopped after {moved:.1f} of {STOP_DEG:g} deg and held (drift "
                                   f"{drift:.2f} deg)", value=round(moved, 2))
        try:
            robot.move_joints(*q0, speed=SPEED_PCT)
            robot.wait_motion_complete(timeout=LEG_TIMEOUT_S)
        except (CommandRejected, RobotTimeout) as e:
            return [out, result(st, WARN, f"J1 did not return to where it started: {e}")]
        return out
    finally:
        _leave_exec(robot)


GRIPPER = step("S5.gripper", 5, "The gripper", check_gripper, needs=("robot", "enabled"),
               about="the gripper closes and opens; you say whether it did")
STOP = step("S5.stop", 5, "A stop mid-move", check_stop, needs=("robot", "enabled"),
            about="J1 is stopped part-way through a slow move; it must stop and hold")
STEPS = [GRIPPER, STOP]
