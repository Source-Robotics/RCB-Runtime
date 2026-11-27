"""The PAR6 arm's settings for LeRobot's programs: ``--robot.type=par6 --robot.host=<the robot's address>``."""
from __future__ import annotations

from dataclasses import dataclass

from lerobot.robots import RobotConfig

from robotclient.constants import DEFAULT_PORT
from robotclient.recording import export as X


@RobotConfig.register_subclass("par6")
@dataclass(kw_only=True)
class PAR6Config(RobotConfig):
    """The PAR6 arm, driven over its real-time link (SOFT_RTI) with its wrist camera.

    Attributes:
        host: The robot's address (required).
        port: The robot's command port.
        fps: The rate LeRobot's loop runs at; the stream's rate. Keep the dataset's (the camera's, 30).
        camera: Whether observations carry the wrist camera's image (``wrist``).
        width, height: The images' size: the dataset's (the export's default 640 x 360).
        max_speed_deg_s: How far a joint's target may move per second; 0 = no cap (the robot's own streaming limits
            only). 30 by default (user 2026-11-27): a far target is walked to.
        gripper_speed, gripper_current: The gripper's speed and current for its targets.
        transport: The real-time link: "udp" or "tcp".
    """

    host: str
    port: int = DEFAULT_PORT
    fps: int = 30
    camera: bool = True
    width: int = X.WIDTH
    height: int = X.HEIGHT
    max_speed_deg_s: float = 30.0
    gripper_speed: int = 100
    gripper_current: int = 200
    transport: str = "udp"
