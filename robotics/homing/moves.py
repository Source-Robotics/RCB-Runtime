"""The moves around homing, as HomingController methods: pre-moves (position / nudge / idle / gripper), move_to
(smooth interpolated joint moves), sequence-level post-moves, and how a joint that is not being moved is held
(homed joints: in position with gravity feedforward)."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

import logging


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
