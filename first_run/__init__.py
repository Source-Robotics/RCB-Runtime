"""The first-run check — a guided verification that a robot was built and wired correctly (TODO/first_run_plan.md).

One sequence in stages, each doing only what is safe at that point; every step says what it does, the value it
measured, PASS / WARN / FAIL and — when something is wrong — the likely causes in a builder's words.

    first_run.core      results, the runner, the UI contract (confirm / choose / wait / show)
    first_run.stage0    S0: the Pi and the configuration (nothing powered needed)
    first_run.stage1    S1: power and CAN (the drives powered, the arm disabled)
    first_run.stage2    S2: the E-stop, the box's buttons, the isolated I/O, the Hall sensor (nothing moves)
    first_run.stage3    S3: small powered test moves before homing (the arm enabled, homed or not)
    first_run.stage3_hand  S3 by hand instead (--by-hand): each joint limp in turn, turned by the user
    first_run.report    the report: temp_data/first_run/<UTC>/report.json + report.txt
    first_run.steps     every step in order (STEPS)

Front-ends: tools/first_run.py (the command line); the dashboard tab comes in P4. Stages S4-S5 (homing, the homed
arm) are P3 of the plan.
"""
