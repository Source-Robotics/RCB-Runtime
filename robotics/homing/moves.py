"""The moves around homing, as HomingController methods: pre-moves (position / nudge / idle / approach / gripper),
move_to (smooth interpolated joint moves), sequence-level post-moves, and how a joint that is not being moved is held
(homed joints: in position with gravity feedforward)."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

import logging

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
# ramp (the endstop). Then it BACKS OFF `backoff` ticks (default 3000, ~3.6 deg on J3) the same way and is handed back
# exactly as an <idle> joint was (user, after the first version held it pressed there: J3 then homed ~0.3 deg off its
# usual spot, two passes ~600-870 ticks apart instead of 10-22 — the press at up to 800 mA stayed in the drivetrain):
# its homing current limit again, the drive's Idle for the rest of the pre-moves, the zero-velocity hold after — its
# homing makes its own gentle contact. No arrival within its timeout (default: the joint's homing timeout) fails
# homing with that reason.
_APPROACH_LAG_S = 0.1          # arrived: the joint trails the ramp by more than 0.1 s of travel ...
_APPROACH_LAG_MIN = 400        # ... (at least this many ticks) ...
_APPROACH_LAG_READS = 10       # ... on this many valid reads in a row
_APPROACH_BACKOFF = 3000       # ticks backed off after arriving (unless the XML says)
_APPROACH_BACK_TOL = 300       # backed off: within this many ticks of the back-off point ...
_APPROACH_BACK_WAIT_S = 0.5    # ... or this long after the ramp got there


def _approach_setup(self, j, pm):
    """The state of one <approach> pre-move (built when its step starts)."""
    speed = abs(float(pm['speed'])) if pm.get('speed') is not None else abs(float(self.homing_speeds[j]))
    speed = -speed if int(self.homing_directions[j]) == 1 else speed
    timeout_s = float(pm['timeout_s']) if pm.get('timeout_s') is not None else float(self._timeout_s[j])
    backoff = abs(float(pm['backoff'])) if pm.get('backoff') is not None else float(_APPROACH_BACKOFF)
    return {'speed': speed, 'current_ma': float(pm['current_ma']), 'timeout_ticks': max(1, round(timeout_s / self.dt)),
            'lag_max': max(_APPROACH_LAG_MIN, abs(speed) * _APPROACH_LAG_S), 'backoff': backoff, 'phase': 'approach',
            'target': None, 'start': None, 'end': None, 'over': 0, 'ticks': 0, 'back_ticks': 0, 'idle_ticks': 0,
            'last_valid': None}


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
            if a['over'] >= _APPROACH_LAG_READS:
                self._approach_arrival[j] = int(pos)
                if not self._approach_where(Motor, j, int(pos), a, shared_data):
                    return 'failed'
                travel = self._approach_travel[j] = int(pos) - int(a['start'])
                a['phase'], a['target'] = 'backoff', float(pos)
                a['end'] = float(pos) - (a['backoff'] if a['speed'] > 0 else -a['backoff'])
                logging.info(f"[Homing] {label} reached its endstop (approach): {travel:+d} ticks in "
                             f"{a['ticks'] * self.dt:.3f} s (from {int(a['start'])} to {int(pos)}) — backing off "
                             f"{a['backoff']:.0f} ticks")
                return 'running'
    if a['ticks'] >= a['timeout_ticks']:
        reason = (f"it did not reach its endstop within {a['timeout_ticks'] * self.dt:.1f} s of its controlled approach "
                  f"({a['speed']:+.0f} ticks/s, {a['current_ma']:.0f} mA) — something in the way, or the joint far "
                  f"from its endstop")
        logging.error(f"[Homing] FAILED: {label} — {reason}")
        if j < len(shared_data.rti["homing_status"]):
            shared_data.rti["homing_status"][j] = STATUS_FAILED
        self._report_failure(j, reason)
        self._seq_phase = self._FAILED
        return 'failed'
    return 'running'


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
        info['duration_ticks'] for info in self._move_to_targets.values()
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

    if all_done:
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
