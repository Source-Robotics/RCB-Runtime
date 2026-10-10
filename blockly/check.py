"""Check: a block program run against the real EXEC planner, offline — TODO/blockly_plan.md B7.

The runner (`--dry-run`) runs the program's Python with the client make_client() returns: a DryRunRobotClient
(robotclient/dry_run.py — the executor's planner, limits, IK and safety zones, in memory; it never connects to
anything), starting from the arm's current joints. A move the robot would refuse (JOINT_LIMIT, ZONE_LIMIT, IK, …)
fails here the same way, on its block, before anything moves.

What a dry run cannot know, it ASSUMES, so it follows the program's main path: every input arrives at once, a grip
catches the part, waits take no time (blockly/runtime.py's dry mode). An endless program is checked for its first
runtime.CHECK_MAX_BLOCKS blocks.

HEAVY: the planner (roboticstoolbox, pinocchio) loads in a new process — seconds of CPU and memory traffic on the
Pi, which is what slows RTI (TODO/future_todo.md, 2026-10-01). The runner runs a Check at nice 19, off RTI's core;
api_tests/58_block_check_load.py measures what it costs the RT loop.
"""
from robotclient.client import GripperResult

GRIP_CAUGHT = 2         # GripperResult code: "object detected while closing" — a Check assumes the grip works


def make_client(start_joints):
    """The dry-run client, ready as a block program needs it. Imports the planner — call it in the runner only."""
    from robotclient.dry_run import DryRunRobotClient       # heavy: only a Check imports it

    class CheckClient(DryRunRobotClient):
        """Enabled and homed (it starts DISABLED, not homed); alone, so control is no question; optimistic where it
        cannot know. The real client's gripper_result reads its TCP connection, which a dry run does not have."""

        def __init__(self):
            super().__init__(start_joints)
            self.enable()
            self.home()

        def take_control(self, force=False, timeout=None):
            return {"holder": "check"}

        def release_control(self):
            return None

        def gripper_result(self):
            return GripperResult(GRIP_CAUGHT)

    return CheckClient()


def summary(robot):
    """What the planner planned: {"moves", "motion_s"} (nothing if the client is not a dry-run one — the nets' fakes)."""
    plan_summary = getattr(robot, "plan_summary", None)
    if plan_summary is None:
        return {}
    plan = plan_summary()
    return {"moves": len(plan.segments), "motion_s": round(float(plan.duration_s), 2)}
