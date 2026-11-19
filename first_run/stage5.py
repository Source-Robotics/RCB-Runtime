"""S5 — the homed arm (2026-10-27, TODO/first_run_plan.md P3; user: the scale of motion, the gripper + a move-and-stop,
friction + quiet holds — all four).

The arm homed and ENABLED; every move is a normal EXEC move (move_joints), so the planner's limit / zone /
self-collision checks guard it.

    S5.joints   per joint, wrist first: it turns <first_run_move scale_deg> (robot XML) and back at SPEED_PCT, then
                holds HOLD_S. Judged: the user says how far it turned (a wrong gear ratio or encoder setting turns it
                clearly more or less; the coupling: not at all); its friction — RTI's external-torque estimate
                (shared memory torque_external = measured minus model, so friction above the robot XML's model reads
                + in the direction of motion) averaged over the steady part of both legs: the same poses both ways,
                so a gravity-model error cancels and what stays is friction the model lacks; a WARN above
                FRICTION_WARN_SHARE of the model (at least FRICTION_WARN_MIN_NM) — never a FAIL: the reference values
                come from one arm; the hold: no new J<n>:HOLD_OSCILLATION_WARNING (RTI's monitor: >25 Hz current
                ripple while still) while it holds.
    S5.gripper  open, close (the user confirms each), the jaws left open.
    S5.stop     J1 starts a slow move, a Stop comes mid-move: it must stop short, stay in EXEC and hold; then back.
"""
from __future__ import annotations

from first_run import shm_files
from first_run.core import FAIL, INFO, PASS, SKIP, WARN, Aborted, Cancelled, claim, result, step
from robotclient import CommandRejected, RobotTimeout

ORDER = (5, 4, 3, 0, 2, 1)       # J6, J5, J4, J1, J3, J2 — as stage 3
SPEED_PCT = 10.0                 # every move at 10 % of the EXEC speed limits
LEG_TIMEOUT_S = 60.0
POLL_S = 0.05
HOLD_S = 2.0                     # the hold-oscillation warning needs 1.5 s of buzzing (375 ticks)
CRUISE_SHARE = 0.7               # friction samples: where the joint moves at >= 70 % of the leg's top speed
MIN_SAMPLES = 5                  # per leg, else the friction is not judged
FRICTION_WARN_SHARE = 0.5        # friction above the model by more than half of it: WARN
FRICTION_WARN_MIN_NM = 0.1
NOT_PLANNED = ("JOINT_LIMIT", "SELF_COLLISION", "ZONE_LIMIT", "ZONE_CHECK_UNAVAILABLE")   # refused before it moved
STOP_DEG = 30.0                  # S5.stop's J1 move (2026-11-19, review K8: was 15 — a ~0.8 s move, the stop came in
STOP_AFTER_S = 0.6               # its last part) ... stopped this long after it started: at cruise, about a third in
HOLD_DRIFT_DEG = 0.5
STOP_MIN_MOVED_DEG = 0.5     # less when the stop came: J1 had not really started, the stop was not tested
GRIPPER_TIMEOUT_S = 30.0
SCALE_ANSWERS = ["about that", "clearly less (half or less)", "clearly more (double or more)", "I saw no movement"]


class _Ended(Exception):
    """A move ended with a failure other than a refusal before moving: the stage cannot go on (an error latched)."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code, self.message = code, message


def _shm(ctx):
    return ctx.get("shm") or shm_files


def _visual(deg):
    return {90.0: "a quarter turn", 45.0: "half of a right angle", 30.0: "a third of a right angle",
            180.0: "half a turn"}.get(float(deg), f"{deg:g} deg")


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


def _hold_keys(robot):
    return {str(e.get("key")) for e in (robot.errors() or []) if str(e.get("key", "")).endswith("HOLD_OSCILLATION_WARNING")}


def _leg(ctx, ui, j, target, prompt):
    """One queued move to `target` (deg), sampled while it runs: [(speed rad/s, external torque Nm)] of joint j.
    Raises CommandRejected when the robot refused or failed it."""
    robot, shm, n = ctx["robot"], _shm(ctx), len(target)
    robot.move_joints(*target, speed=SPEED_PCT)
    samples: list = []

    def poll():
        v, t = shm.floats("joint_speed", n), shm.floats("torque_external", n)
        if v is not None and t is not None:
            samples.append((v[j], t[j]))
        try:
            robot.wait_motion_complete(timeout=0.02)
        except RobotTimeout:
            return None
        return True

    try:
        done = ui.wait(prompt, poll, LEG_TIMEOUT_S, POLL_S)
    except Cancelled:
        robot.stop(wait=True)
        raise
    if done is None:
        robot.stop(wait=True)
        raise _Ended("TIMEOUT", f"the move did not finish within {LEG_TIMEOUT_S:.0f} s")
    return samples


def leg_friction(samples):
    """The mean of torque x sign(speed) over the leg's steady part (|speed| >= CRUISE_SHARE x its top), with the mean
    |speed| there; None when fewer than MIN_SAMPLES."""
    top = max((abs(v) for v, _ in samples), default=0.0)
    steady = [(v, t) for v, t in samples if top > 0 and abs(v) >= CRUISE_SHARE * top]
    if len(steady) < MIN_SAMPLES:
        return None
    return (sum(t * (1 if v > 0 else -1) for v, t in steady) / len(steady),
            sum(abs(v) for v, _ in steady) / len(steady))


def friction_excess(out_samples, back_samples):
    """(friction above the model, mean speed rad/s) from the two legs over the same poses: a gravity-model error
    adds the same torque on both legs, the direction flips, so it cancels in the mean of torque x sign(speed)."""
    a, b = leg_friction(out_samples), leg_friction(back_samples)
    if a is None or b is None:
        return None
    return (a[0] + b[0]) / 2, (a[1] + b[1]) / 2


def model_friction(cfg, j, speed):
    """The robot XML's friction model at this speed, without its load term (Nm): Coulomb + viscous x speed."""
    c = list(cfg.get("friction_coulomb_nm") or [])
    v = list(cfg.get("friction_viscous_nm_s") or [])
    return (float(c[j]) if j < len(c) else 0.0) + (float(v[j]) if j < len(v) else 0.0) * abs(speed)


def _joint(ctx, ui, st, j, deg, held_before):
    name = f"J{j + 1}"
    if not deg:
        return [result(st, SKIP, f"{name}: no <first_run_move scale_deg> in the robot XML — its turn is not defined")]
    if ui.choose(f"{name}: turns {deg:g} deg ({_visual(deg)}) and back, slowly. Go?", ["go", "skip this joint"]) != "go":
        return [result(st, SKIP, f"{name}: skipped by the user")]
    try:
        return _turn(ctx, ui, st, j, deg, held_before)
    except Cancelled:                    # skips this joint only (its move stopped where it was); the next one follows
        return [result(st, SKIP, f"{name}: skipped by the user (its move stopped where it was)")]


def _turn(ctx, ui, st, j, deg, held_before):
    robot, cfg, name = ctx["robot"], ctx.get("config") or {}, f"J{j + 1}"
    q0 = _joints_deg(robot)
    refused, legs = [], None
    for sign in (1, -1):
        target = list(q0)
        target[j] += sign * deg
        try:
            out = _leg(ctx, ui, j, target, f"{name} turns {sign * deg:+g} deg — watch how far")
        except CommandRejected as e:
            if e.code not in NOT_PLANNED:
                raise _Ended(e.code, e.message)
            refused.append(f"{'+' if sign > 0 else '-'}: {e.message}")
            continue
        try:
            back = _leg(ctx, ui, j, q0, f"{name} turns back")
        except CommandRejected as e:
            raise _Ended(e.code, e.message)
        legs = (sign, out, back)
        break
    if legs is None:
        return [result(st, SKIP, f"{name}: the robot refused its turn both ways (" + "; ".join(refused) + ")")]
    _pause(ui, f"{name} holds still for {HOLD_S:g} s", HOLD_S)
    buzz = sorted(_hold_keys(robot) - held_before)
    held_before |= set(buzz)
    answer = ui.choose(f"Did {name} turn about {deg:g} deg ({_visual(deg)}) and back?", SCALE_ANSWERS)
    sign, out, back = legs
    way = "+" if sign > 0 else "-"
    res = []
    if answer == SCALE_ANSWERS[3]:
        res.append(result(st, FAIL, f"{name} was commanded {way}{deg:g} deg but did not move", hints=[
            "the motor turns but the joint does not: the coupling between motor and gearbox, a belt, a grub screw"]))
    elif answer in SCALE_ANSWERS[1:3]:
        less = answer == SCALE_ANSWERS[1]
        res.append(result(st, FAIL, f"{name} turned clearly {'LESS' if less else 'MORE'} than the {deg:g} deg commanded",
                          expected=f"{deg:g} deg ({_visual(deg)})", hints=[
                              f"{name}'s gear ratio in the robot XML does not match its gearbox",
                              "the drive's encoder setting (its resolution)"]
                          + (["a belt or coupling that slips under load"] if less else [])))
    fr = friction_excess(out, back)
    if fr is not None:
        excess, speed = fr
        ref = model_friction(cfg, j, speed)
        if excess > max(FRICTION_WARN_SHARE * ref, FRICTION_WARN_MIN_NM):
            res.append(result(st, WARN, f"{name} needs about {ref + excess:.2f} Nm to turn; the robot XML's friction "
                                        f"model says {ref:.2f} Nm", value=round(excess, 3),
                              expected=f"about {ref:.2f} Nm",
                              hints=["a belt or gearbox too tight, a bearing, a cable pulling on the joint",
                                     "the model was measured on one arm: a new gearbox can run stiffer for its "
                                     "first hours"]))
    for key in buzz:
        res.append(result(st, WARN, f"{key.split(':')[0]} buzzed while holding (its motor current ripples)", value=key,
                          hints=["the joint's gains (the dashboard's PID Config): too high for this arm",
                                 "the encoder's magnet on the motor shaft: loose or too far from the drive"]))
    if not res:
        fr_txt = (f"; friction {model_friction(cfg, j, fr[1]) + fr[0]:.2f} Nm (reference "
                  f"{model_friction(cfg, j, fr[1]):.2f})" if fr is not None else "; friction not measured")
        res.append(result(st, PASS, f"{name}: turned {way}{deg:g} deg as commanded{fr_txt}; held quietly",
                          value=None if fr is None else round(fr[0], 3)))
    elif fr is None:
        res.append(result(st, INFO, f"{name}: friction not measured (too few readings while it moved)"))
    return res


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


def _leave_exec(robot):
    try:
        robot.enter_idle_mode(wait=True)
    except Exception:                  # a stage end must never hide its results
        pass


def check_joints(ctx, ui):
    st = JOINTS
    stop = _homed_exec(ctx, ui, st, "Each joint now turns a visible amount and back, slowly, J6 first: you judge how "
                                    "far it turned; the robot measures its friction and checks that it holds quietly. "
                                    "Clear the space around the arm and keep a hand on the E-stop.")
    if stop is not None:
        return stop
    robot, cfg = ctx["robot"], ctx.get("config") or {}
    scale = list(cfg.get("first_run_scale_deg") or [])
    n = int(cfg.get("joint_num") or 6)
    held = _hold_keys(robot)
    out: list = []
    try:
        for j in (k for k in ORDER if k < n):
            out += _joint(ctx, ui, st, j, scale[j] if j < len(scale) else None, held)
    except _Ended as e:
        out.append(result(st, FAIL, f"a move did not finish ({e.code}): {e.message}", value=e.code, hints=[
            "an error stopped it: the dashboard's Errors tab names it (a collision, a limit, a drive fault)",
            "clear the errors, then run stage 5 again"]))
    except Aborted as e:
        e.partial = out
        raise
    finally:
        _leave_exec(robot)
    return out


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


def check_stop(ctx, ui):
    st = STOP
    stop = _homed_exec(ctx, ui, st, f"J1 now starts a slow {STOP_DEG:g} deg move and is stopped part-way (as a "
                                    "program's stop() does), then returns. Clear the space around the arm and keep a "
                                    "hand on the E-stop.")
    if stop is not None:
        return stop
    robot = ctx["robot"]
    try:
        q0 = _joints_deg(robot)
        for sign in (1, -1):
            target = list(q0)
            target[0] += sign * STOP_DEG
            try:
                robot.move_joints(*target, speed=SPEED_PCT)    # refused at once (MOTION_ABORTED, ...) is caught too
                robot.wait_motion_complete(timeout=STOP_AFTER_S)
            except RobotTimeout:
                break                     # moving: stop it now
            except CommandRejected as e:
                if e.code in NOT_PLANNED and sign > 0:
                    continue              # no room the + way: the other way
                return result(st, SKIP, f"J1's move was not possible ({e.code}): {e.message}")
            return result(st, WARN, "J1's move finished before the stop could come — the stop was not tested",
                          hints=["the arm's EXEC speed limits are high: this check's STOP_AFTER_S is too long"])
        robot.stop(wait=True)
        s = robot.get_status() or {}
        at = _joints_deg(robot)
        moved = abs(at[0] - q0[0])
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
        elif moved < STOP_MIN_MOVED_DEG:
            out = result(st, WARN, f"J1 had moved only {moved:.2f} deg when the stop came — the stop was not tested",
                         value=round(moved, 2), hints=["the move started late (the robot was busy planning), or it "
                                                       "was refused: the dashboard's Errors / Activity Log tab"])
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


JOINTS = step("S5.joints", 5, "Each joint: how far, friction, holding", check_joints, needs=("robot", "enabled"),
              about="each joint turns a visible amount and back; you judge how far, the robot measures its friction "
                    "and checks it holds quietly")
GRIPPER = step("S5.gripper", 5, "The gripper", check_gripper, needs=("robot", "enabled"),
               about="the gripper closes and opens; you say whether it did")
STOP = step("S5.stop", 5, "A stop mid-move", check_stop, needs=("robot", "enabled"),
            about="J1 is stopped part-way through a slow move; it must stop and hold")
STEPS = [JOINTS, GRIPPER, STOP]
