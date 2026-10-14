"""Firmware-level gripper homing: send the calibrate command, wait for the gripper to report calibrated."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

import logging

from robotics.homing.status import STATUS_DONE, STATUS_FAILED, STATUS_RUNNING


class GripperHomingState:
    """
    Firmware-level gripper homing state machine.

    Sends Send_gripper_calibrate() once, then polls gripper_calibrated
    status each tick until it becomes 1 or timeout — accepted only after the
    minimum wait (a whole calibration's time: the firmware keeps the old
    g_cal=1 while it recalibrates).
    """

    _CALIBRATING = 'calibrating'
    _DONE        = 'done'
    _FAILED      = 'failed'

    _CALIBRATE_TIMEOUT_TIME = 10.0  # seconds
    _MIN_WAIT_S             = 2.0   # minimum seconds to wait after sending command
                                    # before accepting g_cal=1, regardless of prior state
    # 2026-10-09: the STEPFOC firmware never clears g_cal when a calibration STARTS, so after the wait g_cal=1 may still
    # be the last calibration's — and the next step's motor commands then abandon this one, whose timeout counter the
    # firmware keeps: on the arm the 12th homing in a row failed (its 24 s ran out). So the wait must cover a WHOLE
    # calibration: per gripper, <homing_calibration_wait_s> in its XML (min_wait_s; missing -> _MIN_WAIT_S).

    def __init__(self, j, dt, timeout_s=None, min_wait_s=None):
        self.j  = int(j)
        self.dt = float(dt)
        self._node_id = None
        timeout = float(timeout_s) if timeout_s is not None else self._CALIBRATE_TIMEOUT_TIME
        wait    = float(min_wait_s) if min_wait_s is not None else self._MIN_WAIT_S
        self._timeout_ticks  = max(1, round(timeout / self.dt))
        self._min_wait_ticks = max(1, round(wait / self.dt))
        self._phase      = self._CALIBRATING
        self._tick_count = 0
        self._sent_cmd   = False
        self.fail_reason = None      # why it failed, for the error the controller raises (2026-10-14)

    @property
    def _label(self) -> str:
        canid = f" [CANID {self._node_id}]" if self._node_id is not None else ""
        return f"Gripper{canid}"

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def timeout_s(self) -> float:
        return self._timeout_ticks * self.dt

    def reset(self):
        self._phase      = self._CALIBRATING
        self._tick_count = 0
        self._sent_cmd   = False
        self.fail_reason = None

    def tick(self, motor, joint_or_none, shared_data) -> str:
        """
        One RT tick. Returns 'running' | 'done' | 'failed'.

        Sends Send_gripper_calibrate() once, then waits a minimum of
        _MIN_WAIT_S seconds before accepting g_cal=1 as completion.
        The minimum wait ensures the gripper has actually run its calibration
        sequence even if g_cal was already 1 before the command was sent
        (some grippers never drop g_cal to 0 during recalibration).
        """
        if self._phase == self._DONE:
            return 'done'
        if self._phase == self._FAILED:
            return 'failed'

        if self._node_id is None:
            self._node_id = getattr(motor, 'node_id', None)

        self._tick_count += 1

        if self._tick_count == 1:
            shared_data.rti["homing_status"][self.j] = STATUS_RUNNING

        # Send calibrate command once
        if not self._sent_cmd:
            if hasattr(motor, 'Send_gripper_calibrate'):
                motor.Send_gripper_calibrate()
                logging.info(f"[Homing] {self._label} firmware calibrate sent")
            else:
                logging.error(
                    f"[Homing] {self._label} motor object has no "
                    f"Send_gripper_calibrate method"
                )
                self.fail_reason = "its driver object cannot send a firmware calibration"
                shared_data.rti["homing_status"][self.j] = STATUS_FAILED
                self._phase = self._FAILED
                return 'failed'
            self._sent_cmd = True

        # Poll gripper status — send empty pack (all None) so the gripper
        # responds with its current state without altering any commands.
        motor.Send_gripper_data_pack()

        # Only accept g_cal=1 after the minimum wait has elapsed
        if self._tick_count >= self._min_wait_ticks:
            calibrated = getattr(motor, 'gripper_calibrated', None)
            if calibrated is not None and int(calibrated) == 1:
                logging.info(
                    f"[Homing] {self._label} firmware calibration done "
                    f"(tick={self._tick_count})"
                )
                shared_data.rti["homing_status"][self.j] = STATUS_DONE
                if hasattr(shared_data, 'gripper') and "gripper_homed" in shared_data.gripper:
                    shared_data.gripper["gripper_homed"][0] = 1
                self._phase = self._DONE
                return 'done'

        # Timeout
        if self._tick_count > self._timeout_ticks:
            self.fail_reason = (f"its firmware calibration did not finish within {self.timeout_s:.1f} s — check that the "
                                f"jaws can move freely and the gripper is powered")
            logging.error(
                f"[Homing] {self._label} firmware calibration timed out "
                f"({self.timeout_s:.1f} s)"
            )
            shared_data.rti["homing_status"][self.j] = STATUS_FAILED
            self._phase = self._FAILED
            return 'failed'

        return 'running'
