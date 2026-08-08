"""
robot_factory.py
----------------
Central registry for all supported robot models.

To add a new robot:
    1. Add its class to ROBOT_REGISTRY below.
    2. That's it — no changes needed anywhere else.
"""

from robots.PAR6 import PAR6
from robots.INDIGO import INDIGO
from robots.SKAR import SKAR

# Map robot_name (as it appears in system.xml) → robot class
# Add new robots here as they are implemented.
ROBOT_REGISTRY: dict = {
    "PAR6":   PAR6,
    "INDIGO": INDIGO,
    "SKAR":   SKAR,
}


def create_robot(robot_name: str, gripper_name: str, symbolic: bool = False):
    """Instantiate and return a robot object by name.

    Args:
        robot_name:   Name of the robot as defined in system.xml (e.g. "PAR6").
        gripper_name: Name of the attached gripper (may be empty string / None).
        symbolic:     Pass True to build the robot in symbolic (Sympy) mode.

    Returns:
        An instance of the requested robot class.

    Raises:
        ValueError: If robot_name is not found in ROBOT_REGISTRY.
    """
    cls = ROBOT_REGISTRY.get(robot_name)
    if cls is None:
        supported = ", ".join(sorted(ROBOT_REGISTRY))
        raise ValueError(
            f"Unknown robot '{robot_name}'. Supported robots: {supported}"
        )
    return cls(symbolic=symbolic, gripper_name=gripper_name)
