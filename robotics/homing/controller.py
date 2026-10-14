"""HomingController: the homing sequence — steps, pre-moves, parallel homing, move_to / post-moves, the arm check."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

import logging
import math

import numpy as np

from robotics.homing import config as _config
from robotics.homing import moves as _moves
from robotics.homing.gripper import GripperHomingState
from robotics.homing.joint import JointHomingState
from robotics.homing.status import STATUS_FAILED


class HomingController:
    """
    Homing sequence controller — manages one or more joints.

    Supports two modes:
    1. Legacy sequential: one joint at a time, order from home_sequence list
    2. Parallel step-based: multiple joints per step, with pre-moves,
       controlled by a homing sequence XML file

    Call step() every RT tick while robot_mode == HOMING.
    Returns 'running', 'done', or 'failed'.
    """

    _IDLE      = 'idle'
    _PRE_MOVE  = 'pre_moving'
    _HOMING    = 'homing'
    _POST_MOVE = 'post_moving'
    _MOVE_TO   = 'move_to'
    _DONE      = 'done'
    _FAILED    = 'failed'

    # Time-based constants for pre/post moves (seconds)
    _MOVE_STABLE_TIME   = 0.05   # must hold within tolerance for this long
    _MOVE_TOL_TICKS     = 100    # encoder tick tolerance for arrival
    _MOVE_TIMEOUT_S     = 4.0    # max time for a pre/post move (short — joint may not be calibrated yet)

    def __init__(self, joint_num, home_offsets, homing_speeds, homing_directions,
                 homing_current_ma, has_hall_sensor, dt=0.001,
                 timeout_s=6.0, two_pass_homing=None, backoff_s=0.3,
                 rehome_speed_factor=0.3, home_sequence=None,
                 post_home_positions=None, post_home_speeds=None,
                 arm_count=None, homing_steps=None,
                 release_current_ma=None, release_duration_s=None,
                 release_sample_pct=None,
                 two_pass_max_diff=None):
        """
        Args:
            joint_num            (int)          : total number of actuators (arm + gripper)
            home_offsets         (np.array)     : joint angle [rad] at each endstop
            homing_speeds        (np.array)     : motor speed [ticks/s] during homing
            homing_directions    (np.array)     : 0 = positive motor dir, 1 = negative
            homing_current_ma    (np.array)     : current threshold [mA] for stall detection
            has_hall_sensor      (np.array)     : 1 if joint uses Hall sensor, 0 for stall
            dt                   (float)        : RT loop period in seconds
            timeout_s            (float/array)  : max time allowed per joint [seconds]
            two_pass_homing      (array-like)   : per-joint bool; True = two-pass (3D-printer-style)
            backoff_s            (float)        : time to reverse after first hit [seconds]
            rehome_speed_factor  (float)        : second-pass speed = first-pass × factor
            home_sequence        (list[int])    : joint indices in homing order (legacy mode)
            post_home_positions  (array-like)   : per-joint angle [rad] to move to after homing
            post_home_speeds     (array-like)   : per-actuator speed [ticks/s] for post-home move
            arm_count            (int)          : number of arm joints (0..arm_count-1)
            homing_steps         (list[dict])   : parsed homing sequence steps (parallel mode)
            release_current_ma   (array-like)   : per-joint current [mA] for release phase (NaN=skip)
            release_duration_s   (array-like)   : per-joint release duration [seconds] (NaN=skip)
            release_sample_pct   (array-like)   : per-joint position sample point 0.0–1.0 (NaN=end)
            two_pass_max_diff    (array-like)   : per-joint max allowed tick difference between passes (NaN=200)
        """
        self.joint_num           = joint_num
        self._arm_count          = int(arm_count) if arm_count is not None else joint_num
        self.home_offsets        = np.asarray(home_offsets,      dtype=float)
        self.homing_speeds       = np.asarray(homing_speeds,     dtype=float)
        self.homing_directions   = np.asarray(homing_directions, dtype=int)
        self.homing_current_ma   = np.asarray(homing_current_ma, dtype=float)
        self.has_hall_sensor     = np.asarray(has_hall_sensor,   dtype=int)
        self.rehome_speed_factor = float(rehome_speed_factor)
        self.dt                  = float(dt)

        # Per-actuator timeout in seconds
        _ts = np.broadcast_to(np.asarray(timeout_s, dtype=float), (joint_num,)).copy()
        self._timeout_s = _ts
        self._timeout_ticks = np.round(_ts / self.dt).astype(int)

        self._backoff_s        = np.broadcast_to(np.asarray(backoff_s,        dtype=float), (joint_num,)).copy()

        if two_pass_homing is None:
            self.two_pass_homing = np.ones(joint_num, dtype=bool)
        else:
            self.two_pass_homing = np.asarray(two_pass_homing, dtype=bool)

        if post_home_positions is None:
            self.post_home_positions = np.full(joint_num, np.nan)
        else:
            self.post_home_positions = np.asarray(post_home_positions, dtype=float)

        if post_home_speeds is None:
            self.post_home_speeds = self.homing_speeds.copy()
        else:
            self.post_home_speeds = np.asarray(post_home_speeds, dtype=float)

        # Release phase arrays (NaN = skip)
        if release_current_ma is None:
            self.release_current_ma = np.full(joint_num, np.nan)
        else:
            self.release_current_ma = np.asarray(release_current_ma, dtype=float)
        if release_duration_s is None:
            self.release_duration_s = np.full(joint_num, np.nan)
        else:
            self.release_duration_s = np.asarray(release_duration_s, dtype=float)
        if release_sample_pct is None:
            self.release_sample_pct = np.full(joint_num, np.nan)
        else:
            self.release_sample_pct = np.asarray(release_sample_pct, dtype=float)

        # Two-pass max difference (NaN = default 200 ticks)
        if two_pass_max_diff is None:
            self.two_pass_max_diff = np.full(joint_num, np.nan)
        else:
            self.two_pass_max_diff = np.asarray(two_pass_max_diff, dtype=float)

        # Homing order (legacy mode)
        self.home_sequence: list = list(home_sequence) if home_sequence is not None \
                                   else list(range(joint_num))

        # Parallel homing steps (None = legacy sequential mode)
        self._homing_steps = homing_steps

        # Tick threshold for move arrival
        self._move_stable_ticks = max(1, round(self._MOVE_STABLE_TIME / self.dt))
        self._move_timeout_ticks = max(1, round(self._MOVE_TIMEOUT_S / self.dt))

        # Runtime state
        self._seq_phase = self._IDLE
        self._seq_idx   = 0
        self._active_j  = 0                              # legacy compat
        self._active_state: JointHomingState = None       # legacy single-joint
        self._active_states: dict = {}                    # parallel: {j: JointHomingState}
        self._homed_results: dict = {}                    # {j: triggered_pos} accumulated as joints finish
        self._pre_move_targets: dict = {}                 # {j: (target_ticks, speed)}
        self._pre_move_ok: dict = {}                      # {j: consecutive_ok_count}
        self._move_tick_count = 0
        self._post_move_list = []                         # sequence-level post moves
        self._move_to_targets: dict = {}                  # {j: {position_rad, duration_ticks, start_ticks, end_ticks, done}}
        self._move_to_tick = 0
        self._last_gripper_move: tuple = None             # last gripper_move params — replayed as idle hold
        self._Motor = None                                # stored for log labels (set in enter())
        # robotics/homing_arms.HomingArms (2026-10-09, TODO/homing_plan.md H1) — set by RTI; None = no check
        self.arm_check = None
        self._seq_ticks = 0                               # ticks since start() — the homing's duration
        # 2026-10-09 (homing plan H2): a HOMED arm joint that is not being moved is held where it is, in POSITION with
        # its gravity current as feedforward — the zero-speed hold let J2 / J3 sink back toward their endstops while the
        # later steps ran (16 homings ended with a joint past its soft limit) {j: motor ticks}
        self._hold_pos: dict = {}
        # 2026-10-14 (user): an <approach> pre-move drives a joint to its endstop and holds it there (raw motor ticks,
        # no gravity feedforward — the pose is not known yet) until its own homing step {j: motor ticks}; its travel
        # is added to the joint's pass-1 travel in the history {j: ticks}
        self._approach_hold: dict = {}
        self._approach_travel: dict = {}
        self._approach: dict = {}                         # the running approaches' state {j: dict}
        self._homing_stats: dict = {}                     # {j: (two-pass diff, release relaxation, travel)} history
        # 2026-10-14 (user: a failed homing is a latched ERROR naming its reason): every failure as (error key, reason),
        # taken by RTI (take_failures) and latched by the error pass until Clear Errors. Before, the only trace was
        # homing_status = 3, which the dispatch reset to 0 in the same tick — the error list never showed it.
        self._failures: list = []
        self.has_failures = False

    # ------------------------------------------------------------------
    # Public time-based properties — backward compat
    # ------------------------------------------------------------------

    @property
    def timeout_s(self) -> np.ndarray:
        """Per-actuator homing timeout in seconds (read/write)."""
        return self._timeout_ticks * self.dt

    @timeout_s.setter
    def timeout_s(self, val):
        arr = np.broadcast_to(np.asarray(val, dtype=float), (self.joint_num,)).copy()
        self._timeout_ticks = np.round(arr / self.dt).astype(int)
        self._timeout_s = arr

    @property
    def backoff_s(self) -> np.ndarray:
        """Per-joint back-off duration in seconds (read/write)."""
        return self._backoff_s

    @backoff_s.setter
    def backoff_s(self, val):
        self._backoff_s = np.broadcast_to(np.asarray(val, dtype=float), (self.joint_num,)).copy()

    from_config = classmethod(_config.from_config)

    # ------------------------------------------------------------------
    def _make_joint_state(self, j: int) -> JointHomingState:
        """Create a JointHomingState for joint index j."""
        return JointHomingState(
            j                   = j,
            homing_speed        = float(self.homing_speeds[j]),
            homing_direction    = int(self.homing_directions[j]),
            homing_current_ma   = float(self.homing_current_ma[j]),
            has_hall_sensor     = int(self.has_hall_sensor[j]),
            dt                  = self.dt,
            timeout_s           = float(self._timeout_s[j]),
            two_pass            = bool(self.two_pass_homing[j]),
            backoff_s           = float(self._backoff_s[j]),
            rehome_speed_factor = self.rehome_speed_factor,
            home_offset         = float(self.home_offsets[j]),
            post_home_position  = float(self.post_home_positions[j]),
            post_home_speed     = float(self.post_home_speeds[j]),
            is_arm_joint        = (j < self._arm_count),
            normal_ilim          = (
                float(self._motor_config['Ilim'][j])
                if getattr(self, '_motor_config', None) is not None
                   and j < len(self._motor_config['Ilim'])
                else getattr(self, '_gripper_normal_ilim', None)
                     if j >= self._arm_count else None
            ),
            normal_velocity_limit = (
                float(self._motor_config['velocity_limit'][j])
                if getattr(self, '_motor_config', None) is not None
                   and j < len(self._motor_config['velocity_limit'])
                else getattr(self, '_gripper_normal_velocity_limit', None)
                     if j >= self._arm_count else None
            ),
            release_current_ma   = float(self.release_current_ma[j]),
            release_duration_s   = float(self.release_duration_s[j]),
            release_sample_pct   = float(self.release_sample_pct[j]),
            two_pass_max_diff    = float(self.two_pass_max_diff[j]),
        )

    # ------------------------------------------------------------------
    def enter(self, Motor, joint_num, load_motor_configs_fn, motor_config, shared_data=None):
        """
        Call once when entering HOMING mode.

        Loads reduced current limits for gentle endstop contact, then starts
        the homing sequence.  Keeps motor_config for exit() restore.

        Args:
            Motor                : list of motor objects
            joint_num            : number of arm joints
            load_motor_configs_fn: callable — load_motor_configs from can_message_handlers
            motor_config         : dict with keys: velocity_limit, Ilim, KPP, KD,
                                   KPIQ, KIIQ, KPV, KIV  (normal operating values)
            shared_data          : shared data object (optional, for clearing homed flags)
        """
        self._motor_config = motor_config
        self._load_motor_configs_fn = load_motor_configs_fn   # kept for callers; homing sends only the limits now
        self._Motor = Motor  # stored for log labels

        self._send_limits(Motor, joint_num, motor_config['velocity_limit'], self.homing_current_ma,
                          "homing current limits")

        self.start(shared_data)

    # 2026-10-09: entry and exit used to re-send the WHOLE config (watchdog, limits, four gain groups) 4 times —
    # ~168 frames, over 40 ms of bus: every joint's replies went stale and the reconnect logic reloaded every driver's
    # config, at each homing's start and end. Only the limits change for homing; the gains and watchdog are on the
    # drivers already (boot, ticks 50 / 150 / 300, every reconnect, set_pid_gains — the same arrays). The homing
    # current limit is what keeps a joint gentle at its endstop, so it keeps its 4 sends.
    _LIMIT_REPEATS = 4

    def _send_limits(self, Motor, joint_num, velocity_limit, current_limit, what):
        """Send every arm joint's velocity + current limit, _LIMIT_REPEATS times."""
        for _ in range(self._LIMIT_REPEATS):
            for i in range(joint_num):
                Motor[i].Send_Limits(Velocity_limit=int(velocity_limit[i]), Current_limit=int(current_limit[i]))
        logging.info(f"[Homing] {what} sent to {joint_num} joints ({self._LIMIT_REPEATS}x)")

    # ------------------------------------------------------------------
    def exit(self, Motor, joint_num):
        """
        Call once when leaving HOMING mode (done or failed).

        Restores normal motor current limits.
        """
        mc = getattr(self, '_motor_config', None)
        if mc is not None:
            self._send_limits(Motor, joint_num, mc['velocity_limit'], mc['Ilim'], "normal current limits")

    _gravity_ff = _moves._gravity_ff

    _send_idle = _moves._send_idle
    _approach_setup = _moves._approach_setup
    _approach_tick = _moves._approach_tick

    # ------------------------------------------------------------------
    def _jlabel(self, j, Motor) -> str:
        """Human-readable label for joint j, e.g. 'J4 [CANID 3]' or 'Gripper [CANID 6]'."""
        canid = ""
        if Motor is not None and j < len(Motor):
            nid = getattr(Motor[j], 'node_id', None)
            if nid is not None:
                canid = f" [CANID {nid}]"
        if j >= self._arm_count:
            return f"Gripper{canid}"
        return f"J{j + 1}{canid}"

    # ------------------------------------------------------------------
    def _build_steps(self):
        """
        Build the internal step list from either homing_steps (parallel mode)
        or home_sequence (legacy sequential mode).

        Each step dict has: pre_moves, home_joints, gripper_mode, post_moves (optional).
        In legacy mode, each step homes a single joint with no pre-moves.
        """
        if self._homing_steps is not None:
            # Parallel mode — resolve 'gripper' to actual index
            steps = []
            for raw_step in self._homing_steps:
                step = dict(raw_step)
                resolved = []
                for jref in step.get('home_joints', []):
                    if jref == 'gripper':
                        # gripper is at index _arm_count (= joint_num for legacy,
                        # or total-1 when gripper is appended)
                        resolved.append(self._arm_count)
                    else:
                        resolved.append(int(jref))
                step['home_joints'] = resolved
                steps.append(step)
            return steps

        # Legacy sequential mode — one joint per step
        steps = []
        for j in self.home_sequence:
            steps.append({
                'pre_moves': [],
                'home_joints': [j],
                'gripper_mode': None,
            })
        return steps

    # ------------------------------------------------------------------
    def start(self, shared_data=None):
        """Begin the homing sequence. Call once when entering HOMING mode."""
        self._steps = self._build_steps()
        self._seq_idx = 0
        self._active_states = {}
        self._homed_results = {}
        self._pre_move_targets = {}
        self._pre_move_ok = {}
        self._move_tick_count = 0
        self._post_move_list = []
        self._move_to_targets = {}
        self._move_to_tick = 0
        self._last_gripper_move = None

        # Legacy compat
        if self._steps and self._steps[0].get('home_joints'):
            self._active_j = self._steps[0]['home_joints'][0]
        else:
            self._active_j = 0
        self._active_state = None

        # Clear per-joint/gripper homed flags
        if shared_data is not None:
            if "joint_homed" in shared_data.rti:
                shared_data.rti["joint_homed"][:] = 0
            if hasattr(shared_data, 'gripper') and "gripper_homed" in shared_data.gripper:
                shared_data.gripper["gripper_homed"][0] = 0

        self._seq_ticks = 0
        self._homing_stats = {}
        self._hold_pos = {}
        self._approach_hold = {}
        self._approach_travel = {}
        if self.arm_check is not None:
            self.arm_check.on_homing_start()

        # Enter first step
        self._enter_step(0)

        mode_desc = "parallel" if self._homing_steps is not None else "sequential"
        logging.info(
            f"[Homing] Sequence started ({mode_desc}) — "
            f"{len(self._steps)} steps, {self.joint_num} actuators"
        )

    # ------------------------------------------------------------------
    def _enter_step(self, idx):
        """Initialize the state for step at index idx."""
        if idx >= len(self._steps):
            return

        self._seq_idx = idx
        step = self._steps[idx]

        # Check for move_to step (smooth interpolated joint-space moves)
        if step.get('move_to') and not step['home_joints']:
            self._move_to_targets = {}
            self._move_to_tick = 0
            for mt in step['move_to']:
                j = mt['joint']
                duration_ticks = max(1, round(mt['duration_s'] / self.dt))
                self._move_to_targets[j] = {
                    'position_rad':   mt['position_rad'],
                    'duration_ticks': duration_ticks,
                    'start_ticks':    None,
                    'end_ticks':      None,
                    'done':           False,
                }
            self._seq_phase = self._MOVE_TO
            logging.info(
                f"[Homing] Step {idx}: move_to for joints "
                f"{list(self._move_to_targets.keys())}"
            )
            return

        # Check for post_moves-only step (sequence-level post moves)
        if 'post_moves' in step and not step['home_joints']:
            self._post_move_list = step['post_moves']
            self._pre_move_targets = {}
            self._pre_move_ok = {}
            self._move_tick_count = 0
            for pm in self._post_move_list:
                j = pm['joint']
                self._hold_pos.pop(j, None)
                self._pre_move_targets[j] = (None, pm['speed'])  # target computed at runtime
                self._pre_move_ok[j] = 0
            self._seq_phase = self._POST_MOVE
            return

        # Has pre-moves?
        if step['pre_moves']:
            self._pre_move_targets = {}
            self._pre_move_ok = {}
            self._move_tick_count = 0
            for pm in step['pre_moves']:
                j = pm['joint']
                # Resolve 'gripper' to the actual gripper motor index
                if j == 'gripper':
                    j = self._arm_count
                    if j >= self.joint_num:
                        # No CAN gripper configured — skip silently
                        continue
                self._hold_pos.pop(j, None)
                if pm.get('type') == 'nudge':
                    duration_ticks = max(1, round(pm['duration_s'] / self.dt))
                    self._pre_move_targets[j] = ('nudge', pm['speed'], duration_ticks)
                elif pm.get('type') == 'idle':
                    duration_ticks = max(1, round(pm['duration_s'] / self.dt))
                    self._pre_move_targets[j] = ('idle', duration_ticks)
                elif pm.get('type') == 'approach':
                    self._approach_hold.pop(j, None)
                    self._approach[j] = self._approach_setup(j, pm)
                    self._pre_move_targets[j] = ('approach', self._approach[j]['timeout_ticks'])
                elif pm.get('type') == 'gripper_move':
                    duration_ticks = max(1, round(pm['duration_s'] / self.dt))
                    self._pre_move_targets[j] = (
                        'gripper_move',
                        pm['position'], pm['speed'], pm['current'],
                        pm['activate'], pm['action'], pm['estop'], pm['release_dir'],
                        duration_ticks,
                    )
                else:
                    # position move — target ticks computed on first tick (needs Joint obj)
                    self._pre_move_targets[j] = (None, pm['speed'], pm['position_rad'])
                self._pre_move_ok[j] = 0
            # Ensure the pre-move timeout is at least as long as the longest
            # timed target (idle / nudge / gripper_move) so timed durations
            # are never cut short by the safety timeout.
            max_timed_ticks = 0
            for info in self._pre_move_targets.values():
                if info[0] in ('idle', 'approach'):
                    max_timed_ticks = max(max_timed_ticks, info[1])
                elif info[0] == 'nudge':
                    max_timed_ticks = max(max_timed_ticks, info[2])
                elif info[0] == 'gripper_move':
                    max_timed_ticks = max(max_timed_ticks, info[-1])
            self._pre_move_timeout_ticks = max(self._move_timeout_ticks, max_timed_ticks)
            self._seq_phase = self._PRE_MOVE
        else:
            self._enter_homing_phase(step)

    # ------------------------------------------------------------------
    def _enter_homing_phase(self, step):
        """Start homing the joints listed in this step."""
        home_joints = step['home_joints']
        gripper_mode = step.get('gripper_mode')
        self._active_states = {}
        for j in home_joints:
            self._approach_hold.pop(j, None)            # its own homing takes over from the <approach> hold
        for j in home_joints:
            # Use GripperHomingState for gripper firmware mode
            if gripper_mode == 'firmware' and j >= self._arm_count:
                if j >= self.joint_num:
                    # No CAN gripper selected — arrays were never extended, skip gracefully.
                    logging.warning(
                        f"[Homing] Skipping {self._jlabel(j, self._Motor)} in step {self._seq_idx + 1}: "
                        f"no CAN gripper configured (gripper_mode='firmware' — skipping)"
                    )
                    continue
                state = GripperHomingState(
                    j=j, dt=self.dt,
                    timeout_s=float(self._timeout_s[j]) if j < len(self._timeout_s)
                              else GripperHomingState._CALIBRATE_TIMEOUT_TIME,
                    min_wait_s=getattr(self, '_gripper_calibration_wait_s', None)
                )
            elif j >= self.joint_num:
                # Joint index is beyond the configured arrays — happens when a
                # homing sequence step references the gripper (mode='motor') but
                # the robot is using a non-CAN gripper (do_gripper=False), so the
                # arrays were never extended.  Skip gracefully.
                logging.warning(
                    f"[Homing] Skipping {self._jlabel(j, self._Motor)} in step {self._seq_idx + 1}: "
                    f"index out of range for homing arrays (non-CAN gripper, "
                    f"gripper_mode='{gripper_mode}' — motor homing not applicable)"
                )
                continue
            else:
                state = self._make_joint_state(j)
            self._active_states[j] = state
        # Legacy compat: set _active_j to first joint
        if home_joints:
            self._active_j = home_joints[0]
            self._active_state = self._active_states.get(self._active_j)
        self._seq_phase = self._HOMING
        if len(home_joints) > 1:
            labels = [self._jlabel(j, self._Motor) for j in home_joints]
            logging.info(f"[Homing] Parallel homing [{', '.join(labels)}]")
        elif home_joints:
            logging.info(
                f"[Homing] Starting {self._jlabel(home_joints[0], self._Motor)} "
                f"(step {self._seq_idx + 1}/{len(self._steps)})"
            )

    @property
    def gripper_homing_active(self) -> bool:
        """True when the homing controller owns the gripper CAN slot.
        RTI.py must suppress send_gripper_commands whenever this is True.

        Covers three cases:
          - firmware/motor homing   (_HOMING phase, gripper in _active_states)
          - gripper_move pre_move   (_PRE_MOVE phase, gripper in _pre_move_targets)
          - holding last command    (_last_gripper_move set — keep suppressing so
                                    the replayed command in _send_idle is not
                                    overwritten by RTI's normal gripper loop)
        """
        if self._last_gripper_move is not None:
            return True
        if self._seq_phase == self._HOMING:
            for j in self._active_states:
                if j >= self._arm_count:
                    return True
        if self._seq_phase == self._PRE_MOVE:
            for j in self._pre_move_targets:
                if j >= self._arm_count:
                    return True
        return False

    # ------------------------------------------------------------------
    def step(self, Motor, Joint, shared_data) -> str:
        """
        Call every RT tick while in HOMING mode.

        Returns:
            'running'  — in progress
            'done'     — all joints homed; switch to ENABLED
            'failed'   — a joint failed; switch to ERROR
        """
        if self._seq_phase == self._DONE:
            return 'done'
        if self._seq_phase == self._FAILED:
            return 'failed'
        if self._seq_phase == self._IDLE:
            logging.warning("[Homing] step() called before start()")
            return 'running'
        self._seq_ticks += 1

        if self._seq_phase == self._PRE_MOVE:
            return self._do_step_pre_move(Motor, Joint, shared_data)
        elif self._seq_phase == self._HOMING:
            return self._do_step_homing(Motor, Joint, shared_data)
        elif self._seq_phase == self._POST_MOVE:
            return self._do_step_post_move(Motor, Joint, shared_data)
        elif self._seq_phase == self._MOVE_TO:
            return self._do_step_move_to(Motor, Joint, shared_data)

        return 'running'

    _do_step_pre_move = _moves._do_step_pre_move

    # ------------------------------------------------------------------
    def _do_step_homing(self, Motor, Joint, shared_data) -> str:
        """Tick all active joint homing state machines in parallel."""
        active_joints = set(self._active_states.keys())

        # Keep non-homing joints sending zero-torque commands so their
        # CAN data stays fresh and the staleness detector never fires.
        for i in range(self.joint_num):
            if i not in active_joints:
                self._send_idle(Motor, i, shared_data)

        # Tick each active joint
        done_joints = []
        for j, state in list(self._active_states.items()):
            joint_obj = Joint[j] if j < self._arm_count else None
            result = state.tick(Motor[j], joint_obj, shared_data)

            if result == 'failed':
                self._report_failure(j, getattr(state, 'fail_reason', None) or "it failed (see RTI.log)")
                self._seq_phase = self._FAILED
                return 'failed'
            elif result == 'done':
                done_joints.append(j)

        # Remove completed joints from active set
        for j in done_joints:
            state = self._active_states[j]
            if isinstance(state, GripperHomingState):
                # Firmware gripper homing has no triggered_pos — nothing to record
                pass
            elif state._triggered_pos is not None:
                self._homed_results[j] = state._triggered_pos
                travel = state.travel_ticks
                if j in self._approach_travel:          # the <approach> brought it most of the way (2026-10-14)
                    travel = self._approach_travel[j] + (travel or 0)
                self._homing_stats[j] = (state.two_pass_diff, state.release_relaxed, travel)
                # For gripper: compute and store ticks_per_meter conversion factor
                if j >= self._arm_count and hasattr(self, '_gripper_stroke_mm'):
                    stroke_m = self._gripper_stroke_mm / 1000.0
                    gear_r = getattr(self, '_gripper_gear_r', 0.0)
                    if hasattr(shared_data, 'gripper'):
                        shared_data.gripper['gripper_homed_endstop_ticks'][0] = state._triggered_pos
                        shared_data.gripper['gripper_stroke_m'][0] = stroke_m
                        # Compute ticks_per_meter from mechanical parameters.
                        # Gear_r is the pinion pitch radius.  One pinion drives both jaws,
                        # so total opening per motor revolution = 2 × 2π × Gear_r.
                        # ticks_per_meter_of_opening = encoder_counts / (4π × Gear_r)
                        if gear_r > 0:
                            _GRIPPER_ENCODER_BITS = 14  # All Spectral BLDC motors use 14-bit encoders
                            encoder_counts = 2 ** _GRIPPER_ENCODER_BITS
                            tpm = encoder_counts / (4.0 * math.pi * gear_r)
                            shared_data.gripper['gripper_ticks_per_meter'][0] = tpm
                            logging.info(
                                f"[Homing] Gripper ticks_per_meter = {tpm:.1f} "
                                f"(from Gear_r={gear_r}, encoder={_GRIPPER_ENCODER_BITS}bit, "
                                f"endstop={state._triggered_pos}, stroke={self._gripper_stroke_mm}mm)"
                            )
                        else:
                            # Fallback: estimate from homing travel (only correct if gripper was fully closed at power-on)
                            endstop_ticks = abs(state._triggered_pos)
                            if endstop_ticks > 0 and stroke_m > 0:
                                tpm = endstop_ticks / stroke_m
                                shared_data.gripper['gripper_ticks_per_meter'][0] = tpm
                                logging.warning(
                                    f"[Homing] Gripper Gear_r not set — estimating ticks_per_meter = {tpm:.1f} "
                                    f"from homing travel (may be inaccurate if gripper was not fully closed at power-on)"
                                )
                        if gear_r > 0:
                            shared_data.gripper['gripper_gear_r'][0] = gear_r
            else:
                # A joint that completed without a reference is not homed (2026-10-14: this was a warning, and the
                # sequence went on).
                reason = "it completed without a home reference (no valid position read at the endstop)"
                logging.error(f"[Homing] {self._jlabel(j, Motor)} FAILED: {reason}")
                shared_data.rti["homing_status"][j] = STATUS_FAILED
                self._report_failure(j, reason)
                self._seq_phase = self._FAILED
                return 'failed'
            del self._active_states[j]
            # Each arm joint against where this arm always homes, AT ONCE (2026-10-14, user) — not only once the
            # whole sequence is through: the steps after it move joints on their references (today's J2 false stall
            # would have been driven 1.4 rad on a wrong one, only J3's own failure stopped it).
            if j < self._arm_count and j in self._homed_results and not self._check_arm_so_far(j, shared_data):
                return 'failed'

        # Update legacy compat pointers
        if self._active_states:
            first_active = next(iter(self._active_states))
            self._active_j = first_active
            self._active_state = self._active_states[first_active]

        # All done? Advance to next step
        if not self._active_states:
            return self._advance_sequence(shared_data)

        return 'running'

    _do_step_post_move = _moves._do_step_post_move

    _do_step_move_to = _moves._do_step_move_to

    _hold_after_move_to = _moves._hold_after_move_to

    # ------------------------------------------------------------------
    def _advance_sequence(self, shared_data) -> str:
        """Advance to the next step, or finish if all done."""
        next_idx = self._seq_idx + 1
        if next_idx >= len(self._steps):
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

        self._enter_step(next_idx)
        return 'running'

    # ------------------------------------------------------------------
    def _check_arm(self, shared_data) -> bool:
        """H1 (TODO/homing_plan.md): every arm joint's latch against where this arm's joint always homes. False — and
        the sequence FAILED, the off joints' homing_status FAILED — when a recognised arm has a joint more than
        homing_arms.TOL_DEG off (a false stall, a skipped belt, a slipping coupling). No check attached: True."""
        if self.arm_check is None:
            return True
        latches = {j: int(p) for j, p in self._homed_results.items() if j < self._arm_count}
        arm_stats = {j: v for j, v in self._homing_stats.items() if j < self._arm_count}
        verdict = self.arm_check.judge_and_learn(latches, {
            "duration_s": round(self._seq_ticks * self.dt, 2),
            "two_pass": {j: v[0] for j, v in arm_stats.items()}, "relaxed": {j: v[1] for j, v in arm_stats.items()},
            "travel": {j: v[2] for j, v in arm_stats.items()}})
        if verdict["kind"] == "fault":
            logging.error(verdict["text"])
            for j, d in verdict["off"]:
                shared_data.rti["homing_status"][j] = STATUS_FAILED
                self._report_failure(j, f"it homed {d:+.2f} deg from where this arm always homes — "
                                        f"{verdict['text'].split(' — ', 1)[-1]}")
            self._seq_phase = self._FAILED
            return False
        (logging.warning if verdict["kind"] == "relearn" else logging.info)(verdict["text"])
        return True

    # ------------------------------------------------------------------
    # Backward-compat properties — tests inspect these on the controller
    # ------------------------------------------------------------------

    @property
    def _phase(self):
        """Returns the per-joint phase for backward compat with tests.

        Tests check ctrl._phase for values like 'homing', 'backing_off',
        'post_moving', etc. These are now per-joint phases on JointHomingState.
        """
        if self._active_state is not None:
            return self._active_state.phase
        return self._seq_phase

    @property
    def _pass(self):
        return self._active_state._pass if self._active_state else 1

    @property
    def _tick_count(self):
        return self._active_state._tick_count if self._active_state else 0

    @property
    def _post_move_target(self):
        return self._active_state._post_move_target if self._active_state else None

    # ------------------------------------------------------------------
    def _signed_speed(self, j: int, factor: float = 1.0) -> int:
        """Backward compat — delegates to active state or computes directly."""
        spd = int(self.homing_speeds[j] * factor)
        return -spd if int(self.homing_directions[j]) == 1 else spd

    def _check_arm_so_far(self, j: int, shared_data) -> bool:
        """The arm joints homed so far against the arm the table recognised from them (homing_arms.HomingArms.
        check_so_far). False — the sequence FAILED, the joint's homing_status FAILED — when an arm was recognised from the
        first joints and this one homed off its spot. No check attached: True."""
        if self.arm_check is None:
            return True
        latches = {i: int(p) for i, p in self._homed_results.items() if i < self._arm_count}
        verdict = self.arm_check.check_so_far(latches, j)
        if verdict["kind"] != "fault":
            if verdict.get("text"):
                logging.info(verdict["text"])
            return True
        logging.error(verdict["text"])
        for i, d in verdict["off"]:
            shared_data.rti["homing_status"][i] = STATUS_FAILED
            self._report_failure(i, verdict["reason"])
        self._seq_phase = self._FAILED
        return False

    def _report_failure(self, j: int, reason: str):
        """One failure for the error pass: J{n}:HOMING_FAILED for an arm joint, GRIPPER_HOMING_FAILED for the gripper."""
        key = f"J{j + 1}:HOMING_FAILED" if j < self._arm_count else "GRIPPER_HOMING_FAILED"
        self._failures.append((key, str(reason)))
        self.has_failures = True

    def take_failures(self) -> list:
        """The failures reported since the last call ([] if none) — RTI's error pass latches them."""
        out, self._failures = self._failures, []
        self.has_failures = False
        return out

    def _fail(self, j: int, shared_data, reason: str):
        """Backward compat — fail the sequence."""
        logging.error(f"[Homing] {self._jlabel(j, self._Motor)} FAILED: {reason}")
        self._report_failure(j, reason)
        shared_data.rti["homing_status"][j] = STATUS_FAILED
        self._seq_phase = self._FAILED
