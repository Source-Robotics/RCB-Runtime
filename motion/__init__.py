"""Motion module — EXEC-mode trajectory execution (planner-side and RTI-side).

RTI-side lives here:
  - exec_handler.py : ExecHandler (consumes trajectory batches, feeds the controller)

Planner-side (command_executor process) is added in later phases:
  - trajectory_planner.py, trajectory_profiles.py
"""
