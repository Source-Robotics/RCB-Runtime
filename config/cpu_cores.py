"""Which CPU core does what on the robot's Pi (4 cores) — the one place that says it (2026-10-12).

  RT_CPU_CORE        RTI.py's real-time loop, SCHED_FIFO 99 (RTI.py, utility/system_config.setup_realtime_process).
  CAN_CPU_CORE       the CAN chip's interrupt threads — and with them the CAN driver's send work, which the kernel
                     runs on the core that queued it — ALONE (hardware/can_hardware.place_can_path, at every RTI
                     start). Measured on the arm (api_tests/69_can_gaps.py, TODO/can_tx_timing_plan.md §8): on RTI's
                     core they froze all CAN traffic while RTI's tick ran; sharing a core with other programs they
                     stalled the exchange under load (134-172 stalls per 10 s); alone on core 2, 3-7 under the same
                     load, as fast as with none.
  GENERAL_CPU_CORES  everything else: the supervisor, the GUI, the executor (its EXEC planner), the self-collision
                     worker, block-program runners, RTI's incident-dump thread, ZMQ's background threads — and every
                     interactive session (SSH, VS Code, scripts, tests: systemd's user.slice, confined by RTI at start).

A leaf module (no imports) so anything can read it without pulling in ZMQ or the robot config.
"""

RT_CPU_CORE = 3
CAN_CPU_CORE = 2
GENERAL_CPU_CORES = (0, 1)
