"""The PAR6 arm as a LeRobot ``Robot``: its calls handed to `link.Link`, which does the work."""
from __future__ import annotations

import logging
from functools import cached_property

from lerobot.robots import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_par6 import PAR6Config
from .link import CAMERA, NAMES, Link

log = logging.getLogger(__name__)


class PAR6(Robot):
    """The PAR6 arm for LeRobot's programs (``lerobot-rollout``, ``lerobot-record --policy``, ``lerobot-replay``).

    Observations: ``j1.pos`` .. ``j6.pos`` (rad), ``gripper.pos`` (0 open .. 1 closed) and ``wrist`` (the camera's
    image), the state taken at the image's moment, as the export makes a dataset's rows. Actions: the same seven
    names, joint position targets (capped) and the gripper's target. `connect()` refuses an arm that is not homed or
    has errors and enables a disabled one; `disconnect()` leaves it holding, enabled.
    """

    config_class = PAR6Config
    name = "par6"
    supports_command_hold = True

    def __init__(self, config: PAR6Config):
        super().__init__(config)
        self.config = config
        self.link = Link(config.host, config.port, fps=config.fps, camera=config.camera, width=config.width,
                         height=config.height, max_speed_deg_s=config.max_speed_deg_s,
                         gripper_speed=config.gripper_speed, gripper_current=config.gripper_current,
                         transport=config.transport)

    @cached_property
    def observation_features(self) -> dict:
        features: dict = {n: float for n in NAMES}
        if self.config.camera:
            features[CAMERA] = (self.config.height, self.config.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict:
        return {n: float for n in NAMES}

    @property
    def is_connected(self) -> bool:
        return self.link.connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.link.connect()
        log.info("%s connected", self)

    @property
    def is_calibrated(self) -> bool:
        return self.link.homed

    def calibrate(self) -> None:
        """Nothing to do on a homed arm: homing is its calibration, done from the dashboard (never from here)."""
        if not self.link.homed:
            raise RuntimeError("the arm is not homed: home it from the dashboard (this plugin never homes the arm)")

    def configure(self) -> None:
        """Nothing: the robot keeps its own configuration."""

    @check_if_not_connected
    def get_observation(self) -> dict:
        return self.link.observe()

    @check_if_not_connected
    def send_action(self, action: dict) -> dict:
        return self.link.act(action)

    @check_if_not_connected
    def hold(self) -> None:
        self.link.hold()

    def disconnect(self) -> None:
        self.link.close()
