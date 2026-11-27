"""The PAR6 arm for LeRobot (TODO/p4_recording_plan.md step 4b, 2026-11-27).

LeRobot imports this package by itself (its name starts ``lerobot_robot_``): the import registers the robot type
``par6``. Install, next to the ``robotclient`` folder, in the Python 3.12+ environment that has LeRobot::

    pip install -e lerobot_robot_par6

then, for example::

    lerobot-replay --robot.type=par6 --robot.host=<the robot's address> --dataset.repo_id=local/demo \\
        --dataset.root=datasets/demo --dataset.episode=0
"""
from .config_par6 import PAR6Config
from .par6 import PAR6

__version__ = "0.1.0"
__all__ = ["PAR6", "PAR6Config", "__version__"]
