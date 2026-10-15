"""The moves around homing, as HomingController methods: pre-moves (position / nudge / idle / approach / gripper),
move_to (smooth interpolated joint moves), sequence-level post-moves, and how a joint that is not being moved is held
(homed joints: in position with gravity feedforward)."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

import logging
import math

from robotics.homing.status import STATUS_FAILED


# ------------------------------------------------------------------
def _gravity_ff(self, shared_data, j) -> int:
    """The gravity current (mA, RTI's compensation_torque) for a HOMED arm joint — 0 for a joint not homed yet (its
    pose, so its gravity, is not known), the gripper, or when the operator turned gravity compensation off."""
    if j >= self._arm_count or j not in self._homed_results:
        return 0
    joint = getattr(shared_data, "joint", None)
    if joint is None or "compensation_torque" not in joint:
        return 0
    return int(joint["compensation_torque"][j])


def _send_idle(self, Motor, i, shared_data=None):
    """Send an appropriate idle/poll command for actuator i.

    Homed arm joints: held in POSITION where they are (captured once, or a move_to's target) with gravity feedforward.
    Other arm joints  (i < _arm_count): Send_data_pack_1 with zero speed/current
    Gripper     (i >= _arm_count): replay the last gripper_move command if
        one was issued this homing session, otherwise send an empty poll.
        Replaying holds the gripper at its commanded position throughout
        subsequent homing phases without needing a new command each step.
    """
    if i >= self._arm_count:
        if self._last_gripper_move is not None:
            pos, spd, cur, act, act2, estop, rel = self._last_gripper_move
            Motor[i].Send_gripper_data_pack(
                position_=pos, speed_=spd, current_=cur,
                activate_=act, action_=act2,
                ESTOP_=estop, Release_dir_=rel,
            )
        else:
            Motor[i].Send_gripper_data_pack()
    else:
        if shared_data is not None and i in self._homed_results:
            pos = self._hold_pos.get(i)
            if pos is None and Motor[i].position is not None:
                pos = self._hold_pos[i] = int(Motor[i].position)
            if pos is not None:
                Motor[i].Send_data_pack_1(Position=pos, Speed=0, Current=self._gravity_ff(shared_data, i))
                return
        Motor[i].Send_data_pack_1(Position=None, Speed=0, Current=0)


# ------------------------------------------------------------------
# <approach> (2026-10-14, user: "J3 sometimes freefalls to its endstop in the idle step — move toward the endstop
# more controlled"): a position RAMP in raw motor ticks toward the endstop the joint homes against, at its homing speed —
# the drive's position loop sets the speed whatever gravity does, and works before homing (the encoder counts are
# there, only the joint angles are not) — with the joint's current limit raised to `current_ma` (enough to carry the
# arm: J3 800 mA = the gravity model's worst case + friction + ~30 %). It has arrived when the joint stops following the
# ramp (the endstop) — softly (2026-10-14, user, after the arm still homed J2 / J3 ~0.1-0.3 deg off with the 800 mA
# contact): as soon as it pushes `contact_ma` (150) above the current it needed while moving AND trails the ramp a little;
# the old rule (the ramp 0.1 s ahead) stays as the backstop. Then it BACKS OFF `backoff` ticks (default 3000, ~3.6 deg on J3) the same way and is handed back
# exactly as an <idle> joint was (user, after the first version held it pressed there: J3 then homed ~0.3 deg off its
# usual spot, two passes ~600-870 ticks apart instead of 10-22 — the press at up to 800 mA stayed in the drivetrain):
# its homing current limit again, the drive's Idle for the rest of the pre-moves, the zero-velocity hold after — its
# homing makes its own gentle contact. No arrival within its timeout (default: the joint's homing timeout) fails
# homing with that reason.
_APPROACH_LAG_S = 0.1          # arrived: the joint trails the ramp by more than 0.1 s of travel ...
_APPROACH_LAG_MIN = 400        # ... (at least this many ticks) ...
_APPROACH_LAG_READS = 10       # ... on this many valid reads in a row
_APPROACH_CONTACT_MA = 150     # soft arrival: pushing this much above the moving current ...
_APPROACH_SLOW = 0.3           # ... while it has (nearly) stopped: < 30 % of the ramp's travel over the last reads
                               # (2026-10-14: on the arm a long approach WITH gravity braked at -300 mA; near the endstop
                               # that eased and read as +150-225 mA "contact" ~2000 ticks early, the joint still moving)
_APPROACH_SLOW_TICKS = 3       #     (measured over at least this many ticks)
_APPROACH_FAST = 0.7           # the moving current is learned only while it really moves (>= 70 % of the ramp) and
                               # frozen as it slows (2026-10-14: learned while slowing at the endstop it climbed with the
                               # push — "contact" came at ~800 mA, J3 homed +0.355 deg off again)
_APPROACH_SOFT_LAG_S = 0.005   # ... and trailing the ramp by more than 5 ms of travel (at least 40 ticks) ...
_APPROACH_SOFT_LAG_MIN = 40
_APPROACH_SOFT_READS = 3       # ... on this many valid reads in a row
_APPROACH_SETTLE_S = 0.15      # the start's acceleration: no soft arrival, the moving current not yet learned
_APPROACH_BASE_TAU = 0.1       # s: the moving current's running mean
_APPROACH_BACKOFF = 3000       # ticks backed off after arriving (unless the XML says)
_APPROACH_BACK_TOL = 300       # backed off: within this many ticks of the back-off point ...
_APPROACH_BACK_WAIT_S = 0.5    # ... or this long after the ramp got there
_APPROACH_TRACE_S = 0.5        # the approach's log line: where it was against its ramp every this many s
# "Brake strong, touch light" (2026-10-14, user, after the log showed J3 run 1.6-5.0 deg ahead of its ramp from a
# high pose, braking at its whole 450 mA for ~2 s): with `brake_ma` the limit rises to it while the joint brakes
# harder than this against its travel (gravity pulling it toward the endstop) ...
_APPROACH_BRAKE_ON_MA = 300
_APPROACH_BRAKE_OFF_MA = 150   # ... and is `current_ma` again once the braking eases below this — on the arm before the
                               # contact (J3 was lifting, +280 mA, when it touched), so the contact stays light


def _approach_setup(self, j, pm):
    """The state of one <approach> pre-move (built when its step starts)."""
    speed = abs(float(pm['speed'])) if pm.get('speed') is not None else abs(float(self.homing_speeds[j]))
    speed = -speed if int(self.homing_directions[j]) == 1 else speed
    timeout_s = float(pm['timeout_s']) if pm.get('timeout_s') is not None else float(self._timeout_s[j])
    backoff = abs(float(pm['backoff'])) if pm.get('backoff') is not None else float(_APPROACH_BACKOFF)
    contact = abs(float(pm['contact_ma'])) if pm.get('contact_ma') is not None else float(_APPROACH_CONTACT_MA)
    return {'speed': speed, 'current_ma': float(pm['current_ma']), 'timeout_ticks': max(1, round(timeout_s / self.dt)),
            'lag_max': max(_APPROACH_LAG_MIN, abs(speed) * _APPROACH_LAG_S), 'backoff': backoff, 'phase': 'approach',
            'contact_ma': contact, 'soft_lag': max(_APPROACH_SOFT_LAG_MIN, abs(speed) * _APPROACH_SOFT_LAG_S),
            'dt': float(self.dt),
            'settle_ticks': max(1, round(_APPROACH_SETTLE_S / self.dt)), 'base': None, 'soft': 0, 'how': '', 'trail': [],
            'target': None, 'start': None, 'end': None, 'over': 0, 'ticks': 0, 'back_ticks': 0, 'idle_ticks': 0,
            'last_valid': None,
            'lead': (-math.inf, 0.0), 'brake': (math.inf, 0.0), 'trace': [],
            'trace_every': max(1, round(_APPROACH_TRACE_S / self.dt)),
            'brake_ma': float(pm['brake_ma']) if pm.get('brake_ma') is not None else None, 'braking': False,
            'brake_on': 0, 'brake_ticks': 0}


def _approach_limits(self, Motor, j, current_ma):
    mc = getattr(self, '_motor_config', None)
    vlim = mc['velocity_limit'][j] if mc is not None else abs(self._approach[j]['speed']) * 4
    for _ in range(self._LIMIT_REPEATS):
        Motor[j].Send_Limits(Velocity_limit=int(vlim), Current_limit=int(current_ma))


def _approach_tick(self, Motor, j, shared_data) -> str:
    """One tick of joint j's <approach>: 'running', 'arrived' (backed off and idle, as an <idle> joint) or 'failed'."""
    a = self._approach[j]
    if a['phase'] == 'idle':                         # handed back: the drive's Idle (twice), then polls — as <idle>
        a['idle_ticks'] += 1
        if a['idle_ticks'] <= 2:
            Motor[j].Send_Idle()
        else:
            Motor[j].Send_Respond_Encoder_data()
        return 'arrived'
    label = self._jlabel(j, Motor)
    pos = Motor[j].position
    if pos is not None:
        a['last_valid'] = int(pos)
    if a['phase'] == 'backoff':
        return self._approach_back(Motor, j, pos, label)
    a['ticks'] += 1
    if a['ticks'] == 1:
        self._approach_limits(Motor, j, a['current_ma'])
        logging.info(f"[Homing] {label} approaching its endstop: {a['speed']:+.0f} ticks/s (a position ramp), current "
                     f"limit {a['current_ma']:.0f} mA")
    if a['target'] is None:
        if a['last_valid'] is None:                 # no position yet: hold still until one is read
            Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
        else:
            a['target'] = a['start'] = float(a['last_valid'])
    else:
        a['target'] += a['speed'] * self.dt
    if a['target'] is not None:
        Motor[j].Send_data_pack_1(Position=int(round(a['target'])), Speed=int(a['speed']), Current=0)
        if pos is not None:
            lag = (a['target'] - pos) * (1.0 if a['speed'] > 0 else -1.0)
            a['over'] = a['over'] + 1 if lag > a['lag_max'] else 0
            self._approach_contact(Motor, j, a, lag)
            _approach_watch(a, lag, Motor[j].current)
            if a['brake_ma'] is not None:
                self._approach_brake(Motor, j, a)
            if a['over'] >= _APPROACH_LAG_READS or a['soft'] >= _APPROACH_SOFT_READS:
                _approach_report(a, label)
                self._approach_arrival[j] = int(pos)
                if not self._approach_where(Motor, j, int(pos), a, shared_data):
                    return 'failed'
                travel = self._approach_travel[j] = int(pos) - int(a['start'])
                a['phase'], a['target'] = 'backoff', float(pos)
                a['end'] = float(pos) - (a['backoff'] if a['speed'] > 0 else -a['backoff'])
                how = a['how'] if a['soft'] >= _APPROACH_SOFT_READS else f"the ramp {lag:.0f} ticks ahead"
                logging.info(f"[Homing] {label} reached its endstop (approach): {travel:+d} ticks in "
                             f"{a['ticks'] * self.dt:.3f} s (from {int(a['start'])} to {int(pos)}; {how}) — backing off "
                             f"{a['backoff']:.0f} ticks")
                return 'running'
    if a['ticks'] >= a['timeout_ticks']:
        reason = (f"it did not reach its endstop within {a['timeout_ticks'] * self.dt:.1f} s of its controlled approach "
                  f"({a['speed']:+.0f} ticks/s, {a['current_ma']:.0f} mA) — something in the way, or the joint far "
                  f"from its endstop")
        _approach_report(a, label)
        logging.error(f"[Homing] FAILED: {label} — {reason}")
        if j < len(shared_data.rti["homing_status"]):
            shared_data.rti["homing_status"][j] = STATUS_FAILED
        self._report_failure(j, reason)
        self._seq_phase = self._FAILED
        return 'failed'
    return 'running'


def _approach_contact(self, Motor, j, a, lag) -> None:
    """The soft arrival (2026-10-14): the push (the drive's current toward the endstop) against its running mean while
    moving; `a['soft']` counts the reads it is `contact_ma` above it while the joint trails the ramp."""
    cur, pos = Motor[j].current, Motor[j].position
    if pos is not None:                              # the last few valid reads: how fast it really moves
        a['trail'].append((a['ticks'], int(pos)))
        if len(a['trail']) > 8:
            a['trail'].pop(0)
    if cur is None or a['ticks'] <= a['settle_ticks']:
        return
    push = float(cur) * (1.0 if a['speed'] > 0 else -1.0)
    if a['base'] is None:
        a['base'] = push
        return
    moving = _approach_moving(a)
    if push - a['base'] > a['contact_ma'] and lag > a['soft_lag'] and moving is not None and moving < _APPROACH_SLOW:
        a['soft'] += 1
        a['how'] = (f"contact: pushing {push - a['base']:+.0f} mA over the {a['base']:+.0f} mA it moved with, moving at "
                    f"{moving:.0%} of the ramp")
    else:
        a['soft'] = 0
        if moving is None or moving >= _APPROACH_FAST:
            a['base'] += (push - a['base']) * min(1.0, self.dt / _APPROACH_BASE_TAU)


def _approach_watch(a, lag, cur) -> None:
    """For the approach's log line (2026-10-14, user: from a high pose J3 "drops halfway", then moves slowly): how far
    the joint ran AHEAD of its ramp (gravity beating the current limit) and the most current it braked with (the
    current against its travel; at the limit = it could not hold back more), and every 0.5 s both (no text built
    here)."""
    t = a['ticks'] * a['dt']
    if -lag > a['lead'][0]:
        a['lead'] = (-lag, t)
    push = None if cur is None else float(cur) * (1.0 if a['speed'] > 0 else -1.0)
    if push is not None and push < a['brake'][0]:
        a['brake'] = (push, t)
    if a['ticks'] % a['trace_every'] == 0:
        a['trace'].append((t, -lag, push))


def _approach_brake(self, Motor, j, a) -> None:
    """Brake strong, touch light: `brake_ma` while the joint brakes harder than _APPROACH_BRAKE_ON_MA against its
    travel, `current_ma` again once it brakes less than _APPROACH_BRAKE_OFF_MA (or pushes toward the endstop)."""
    cur = Motor[j].current
    if a['braking']:
        a['brake_ticks'] += 1
    if cur is None:
        return
    push = float(cur) * (1.0 if a['speed'] > 0 else -1.0)
    if not a['braking'] and push < -_APPROACH_BRAKE_ON_MA:
        a['braking'] = True
        a['brake_on'] += 1
        self._approach_limits(Motor, j, a['brake_ma'])
    elif a['braking'] and push > -_APPROACH_BRAKE_OFF_MA:
        a['braking'] = False
        self._approach_limits(Motor, j, a['current_ma'])


def _approach_report(a, label) -> None:
    """The approach's log line: ahead of its ramp / braking, and the 0.5 s trace (ahead of the ramp in ticks / the
    current toward the endstop in mA, negative = braking)."""
    lead, lead_t = a['lead']
    brake, brake_t = a['brake']
    ahead = (f"ran up to {lead:.0f} ticks AHEAD of its ramp (at {lead_t:.2f} s)" if lead > 0
             else f"never ahead of its ramp (trailed it by at least {-lead:.0f} ticks)")
    braked = (f"braked with up to {-brake:.0f} mA (at {brake_t:.2f} s; limit {a['current_ma']:.0f} mA)"
              if math.isfinite(brake) and brake < 0 else "never braked")
    if a['brake_ma'] is not None:
        braked += (f"; its {a['brake_ma']:.0f} mA brake limit on {a['brake_on']} time(s), "
                   f"{a['brake_ticks'] * a['dt']:.2f} s in all{' (STILL ON at the end)' if a['braking'] else ''}")
    trace = ", ".join(f"{t:.1f}s {ld:+.0f}/{'?' if p is None else f'{p:+.0f}'}" for t, ld, p in a['trace'])
    logging.info(f"[Homing] {label} approach: {ahead}; {braked}. Every {_APPROACH_TRACE_S:.1f} s, ticks ahead of the "
                 f"ramp / mA toward the endstop: {trace or 'none'}")


def _approach_moving(a):
    """How fast the joint really moved over its last valid reads (>= _APPROACH_SLOW_TICKS apart), as a fraction of the
    ramp's speed (toward the endstop); None without two such reads."""
    if len(a['trail']) < 2:
        return None
    t1, p1 = a['trail'][-1]
    old = next(((t, p) for t, p in reversed(a['trail'][:-1]) if t1 - t >= _APPROACH_SLOW_TICKS), None)
    if old is None:
        return None
    ramp = abs(a['speed']) * (t1 - old[0]) * a.get('dt', 0.004)
    return (p1 - old[1]) * (1.0 if a['speed'] > 0 else -1.0) / ramp if ramp > 0 else None


def _approach_where(self, Motor, j, pos, a, shared_data) -> bool:
    """Where the approach stopped against where it stops on the known arms (homing_arms.check_approach, 2026-10-14:
    J3 stopped 7 deg short — against the arm's own base — and homing went on). False: FAILED, the joint stopped."""
    if self.arm_check is None:
        return True
    verdict = self.arm_check.check_approach(j, pos, 1 if a['speed'] > 0 else -1)
    if verdict["kind"] != "fault":
        return True
    Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)      # stop pushing
    logging.error(verdict["text"])
    if j < len(shared_data.rti["homing_status"]):
        shared_data.rti["homing_status"][j] = STATUS_FAILED
    self._report_failure(j, verdict["reason"])
    self._seq_phase = self._FAILED
    return False


def _approach_back(self, Motor, j, pos, label) -> str:
    """The back-off after arriving: the ramp back at the approach speed to `end`; there (or 0.5 s after the ramp got
    there), handed back: its homing current limit, then the drive's Idle — 'arrived'."""
    a = self._approach[j]
    step = abs(a['speed']) * self.dt
    to_go = a['end'] - a['target']
    a['target'] = a['end'] if abs(to_go) <= step else a['target'] + (step if to_go > 0 else -step)
    there = a['target'] == a['end']
    a['back_ticks'] += 1 if there else 0
    Motor[j].Send_data_pack_1(Position=int(round(a['target'])), Speed=0 if there else int(-a['speed']), Current=0)
    if there and ((pos is not None and abs(int(pos) - a['end']) <= _APPROACH_BACK_TOL)
                  or a['back_ticks'] * self.dt >= _APPROACH_BACK_WAIT_S):
        self._approach_limits(Motor, j, self.homing_current_ma[j])
        a['phase'] = 'idle'
        logging.info(f"[Homing] {label} backed off to {a['last_valid']} — idle until its homing (current limit "
                     f"{self.homing_current_ma[j]:.0f} mA again)")
    return 'running'


# ------------------------------------------------------------------
def _do_step_pre_move(self, Motor, Joint, shared_data) -> str:
    """Execute pre-moves for the current step."""
    self._move_tick_count += 1
    active_move_joints = set(self._pre_move_targets.keys())

    # Keep non-moving joints sending zero-torque commands so their
    # CAN data stays fresh and the staleness detector never fires.
    for i in range(self.joint_num):
        if i not in active_move_joints:
            self._send_idle(Motor, i, shared_data)

    all_arrived = True
    for j, move_info in self._pre_move_targets.items():

        if move_info[0] == 'idle':
            # Put the motor driver into idle mode for a timed duration.
            # The driver does not respond to Send_Idle, so we send it at
            # least twice (first two ticks) to ensure it is received, then
            # follow up with Send_Respond_Encoder_data to keep getting
            # motor feedback for the remainder of the idle period.
            duration_ticks = move_info[1]
            if self._move_tick_count <= 2:
                Motor[j].Send_Idle()
            else:
                Motor[j].Send_Respond_Encoder_data()
            if self._move_tick_count < duration_ticks:
                all_arrived = False
            continue

        if move_info[0] == 'approach':
            r = self._approach_tick(Motor, j, shared_data)
            if r == 'failed':
                return 'failed'
            if r != 'arrived':
                all_arrived = False
            continue

        if move_info[0] == 'nudge':
            # Velocity-based timed nudge — no position target, just run for duration_ticks.
            # After duration elapses, hold with zero velocity until other pre-moves finish.
            speed = int(move_info[1])
            duration_ticks = move_info[2]
            if self._move_tick_count < duration_ticks:
                Motor[j].Send_data_pack_1(Position=None, Speed=speed, Current=0)
                all_arrived = False
            else:
                Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
            continue

        if move_info[0] == 'gripper_move':
            # Gripper position/speed/current command held for duration_ticks,
            # then replayed by _send_idle for the rest of homing.
            # position and speed are 0-255; current is mA; flags are 0/1.
            _, position, speed, current, activate, action, estop, release_dir, duration_ticks = move_info
            Motor[j].Send_gripper_data_pack(
                position_=position, speed_=speed, current_=current,
                activate_=activate, action_=action,
                ESTOP_=estop, Release_dir_=release_dir,
            )
            self._last_gripper_move = (position, speed, current, activate, action, estop, release_dir)
            if self._move_tick_count < duration_ticks:
                all_arrived = False
            continue

        # Compute position target once (needs Joint object)
        if move_info[0] is None:
            if j < self._arm_count and j < len(Joint):
                target_ticks = int(Joint[j].get_encoder_position(move_info[2]))
            else:
                target_ticks = 0
            self._pre_move_targets[j] = (target_ticks, move_info[1], move_info[2])
            move_info = self._pre_move_targets[j]

        target_ticks = move_info[0]
        speed = int(move_info[1])
        Motor[j].Send_data_pack_1(Position=None, Speed=speed, Current=0)

        # Check arrival
        curr_pos = Motor[j].position
        if curr_pos is not None:
            if abs(int(curr_pos) - target_ticks) < self._MOVE_TOL_TICKS:
                self._pre_move_ok[j] = self._pre_move_ok.get(j, 0) + 1
            else:
                self._pre_move_ok[j] = 0

        if self._pre_move_ok.get(j, 0) < self._move_stable_ticks:
            all_arrived = False

    # Timeout — use per-step timeout that respects timed pre-move durations
    if self._move_tick_count > self._pre_move_timeout_ticks:
        logging.warning(
            f"[Homing] Pre-move timed out at step {self._seq_idx + 1} "
            f"— joint(s) {list(active_move_joints)} could not reach target "
            f"(robot may not be calibrated yet). Continuing to homing."
        )
        all_arrived = True

    if all_arrived:
        for j in active_move_joints:
            self._send_idle(Motor, j, shared_data)
        step = self._steps[self._seq_idx]
        self._enter_homing_phase(step)

    return 'running'


# ------------------------------------------------------------------
def _do_step_post_move(self, Motor, Joint, shared_data) -> str:
    """Execute sequence-level post-moves (park joints after all homing)."""
    self._move_tick_count += 1
    active_move_joints = set(self._pre_move_targets.keys())

    for i in range(self.joint_num):
        if i not in active_move_joints:
            self._send_idle(Motor, i, shared_data)

    all_arrived = True
    for j, move_info in self._pre_move_targets.items():
        if move_info[0] is None:
            pm = [p for p in self._post_move_list if p['joint'] == j][0]
            if j < self._arm_count and j < len(Joint):
                target_ticks = int(Joint[j].get_encoder_position(pm['position_rad']))
            else:
                target_ticks = 0
            self._pre_move_targets[j] = (target_ticks, pm['speed'])
            move_info = self._pre_move_targets[j]

        target_ticks = move_info[0]
        speed = int(move_info[1])   
        Motor[j].Send_data_pack_1(Position=None, Speed=speed, Current=0)

        curr_pos = Motor[j].position
        if curr_pos is not None:
            if abs(int(curr_pos) - target_ticks) < self._MOVE_TOL_TICKS:
                self._pre_move_ok[j] = self._pre_move_ok.get(j, 0) + 1
            else:
                self._pre_move_ok[j] = 0

        if self._pre_move_ok.get(j, 0) < self._move_stable_ticks:
            all_arrived = False

    if self._move_tick_count > self._move_timeout_ticks:
        logging.warning("[Homing] Post-move timed out — finishing")
        all_arrived = True

    if all_arrived:
        for j in active_move_joints:
            Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
        if not self._check_arm(shared_data):
            return 'failed'
        self._seq_phase = self._DONE
        shared_data.homed_state.set("HOMED")
        logging.info("[Homing] All joints homed successfully.")
        logging.info("[Homing] ── Home positions set per joint ─────────────")
        for j in sorted(self._homed_results):
            logging.info(f"[Homing]   {self._jlabel(j, self._Motor)}: endstop encoder pos = {self._homed_results[j]}")
        logging.info("[Homing] ─────────────────────────────────────────────")
        return 'done'

    return 'running'


# ------------------------------------------------------------------
def _do_step_move_to(self, Motor, Joint, shared_data) -> str:
    """Execute smooth interpolated move_to commands for the current step.

    Each joint moves from its current position to a target angle over a
    specified duration using cubic Hermite (smoothstep) interpolation.
    This gives zero velocity at both start and end for smooth motion.
    """
    self._move_to_tick += 1
    active_joints = set(self._move_to_targets.keys())

    # Keep non-moving joints alive (prevent CAN staleness)
    for i in range(self.joint_num):
        if i not in active_joints:
            self._send_idle(Motor, i, shared_data)

    all_done = True
    for j, info in self._move_to_targets.items():
        # Capture start position and compute end position on first tick
        if info['start_ticks'] is None:
            curr_pos = Motor[j].position
            if curr_pos is None:
                # Motor position not yet available — hold and retry next tick
                Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
                all_done = False
                continue
            info['start_ticks'] = int(curr_pos)
            info['end_ticks'] = int(
                Joint[j].get_encoder_position(info['position_rad'])
            )
            logging.info(
                f"[Homing] move_to {self._jlabel(j, Motor)}: "
                f"{info['start_ticks']} → {info['end_ticks']} ticks "
                f"({info['position_rad']:.4f} rad) over "
                f"{info['duration_ticks']} ticks "
                f"({info['duration_ticks'] * self.dt:.2f} s)"
            )

        if info['done']:
            # Hold final position
            Motor[j].Send_data_pack_1(
                Position=info['end_ticks'], Speed=0, Current=self._gravity_ff(shared_data, j)
            )
            continue

        # Cubic Hermite (smoothstep): s(tau) = 3*tau^2 - 2*tau^3
        tau = min(1.0, self._move_to_tick / info['duration_ticks'])
        s = 3.0 * tau * tau - 2.0 * tau * tau * tau
        setpoint = int(
            info['start_ticks']
            + (info['end_ticks'] - info['start_ticks']) * s
        )

        Motor[j].Send_data_pack_1(Position=setpoint, Speed=0, Current=self._gravity_ff(shared_data, j))

        if tau >= 1.0:
            # Interpolation done — now verify motor actually reached target. An EMPTY read is no information and
            # must not reset the count (2026-10-09): Spectral_BLDC.UnpackData() wipes position on every health-poll
            # reply, ~every 7th tick per motor, so the 13 consecutive reads the count needed almost never happened —
            # every timed-out move_to on the arm had its joints at +0.000 deg from the target.
            curr_pos = Motor[j].position
            if curr_pos is not None:
                info['last_pos'] = int(curr_pos)
                if abs(int(curr_pos) - info['end_ticks']) < self._MOVE_TOL_TICKS:
                    info['settle_ok'] = info.get('settle_ok', 0) + 1
                else:
                    info['settle_ok'] = 0

            if info.get('settle_ok', 0) >= self._move_stable_ticks:
                info['done'] = True
                logging.info(f"[Homing] move_to {self._jlabel(j, Motor)} arrived and settled.")
            else:
                all_done = False
        else:
            all_done = False

    # Timeout safety: max duration + 2 seconds margin
    max_duration = max(
        max(info['duration_ticks'] for info in self._move_to_targets.values()), self._move_to_min_ticks
    )
    timeout_ticks = max_duration + max(1, round(2.0 / self.dt))
    if self._move_to_tick > timeout_ticks:
        # which joints did not arrive, and how far short (2026-10-09: the arrival rule is 100 motor ticks — 0.09 deg on
        # J2, 0.55 deg on J4 — and most move_to steps time out; this decides the tolerance per joint, in degrees)
        short = []
        for j, info in self._move_to_targets.items():
            pos = Motor[j].position if Motor[j].position is not None else info.get('last_pos')   # the last VALID read
            if not info.get('done') and pos is not None and info.get('end_ticks') is not None and j < len(Joint):
                jt = Joint[j]
                deg = (int(pos) - info['end_ticks']) * 360.0 / (jt.encoder_max_counts * jt.gear_ratio)
                short.append(f"{self._jlabel(j, Motor).split(' ')[0]} {deg:+.3f} deg")
        logging.warning("[Homing] move_to timed out — advancing"
                        + (f" (not within {self._MOVE_TOL_TICKS} ticks: {', '.join(short)} from the target)" if short else ""))
        for j in active_joints:
            self._hold_after_move_to(Motor, j, shared_data)
        return self._advance_sequence(shared_data)

    if all_done and self._move_to_tick >= self._move_to_min_ticks:     # its <gripper_move>'s duration too
        for j in active_joints:
            self._hold_after_move_to(Motor, j, shared_data)
        return self._advance_sequence(shared_data)

    return 'running'


def _hold_after_move_to(self, Motor, j, shared_data):
    """After a move_to step: a homed joint is held at the move's TARGET (not wherever gravity left it) from now on;
    a joint not homed yet gets the old zero-speed command."""
    info = self._move_to_targets.get(j, {})
    if j in self._homed_results and info.get('end_ticks') is not None:
        self._hold_pos[j] = int(info['end_ticks'])
        Motor[j].Send_data_pack_1(Position=self._hold_pos[j], Speed=0, Current=self._gravity_ff(shared_data, j))
    else:
        Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
