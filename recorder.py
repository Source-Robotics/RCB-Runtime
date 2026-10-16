"""The recorder: incident dumps and telemetry captures, OUTSIDE RTI's process (2026-10-16).

RTI writes every tick's values into a ring in shared memory (utility/trace_ring.py, ~9 us a tick) and does nothing
else for recording. This process — started by supervisor.py next to RTI, the GUI, the executor and the self-collision
worker; normal priority on the general cores (config/cpu_cores.py) — reads the ring 20 times a second with its own
interpreter and, through utility/trace_recorder.Recorder:

  * writes an incident dump (5 s before a hard error, 2 s after) when a new hard error appears in the error list
    (shared memory's error_state, written by RTI's error pass) -> temp_data/incidents/;
  * runs a capture when the executor posts a request (capture_telemetry: shared memory's capture_request) ->
    temp_data/captures/, with CAPTURE_STARTED / CAPTURE_SAVED / CAPTURE_REFUSED in its own activity log
    (logs/activity_rec.log, source "REC");
  * counts its heartbeat in the trace's head (the health summary turns a frozen one into STALE);
  * applies its settings (config/system.xml <recorder>, utility/recorder_settings.py) at start and within ~1 s of the
    file changing (the Recordings tab writes them), and publishes its status (the settings in force, a running
    capture's progress, the counters) as JSON in shared memory's recorder_status for the dashboard (2026-10-16).

Why a process (user, 2026-10-16: "I don't want them to impact my loop performance in any way"): inside RTI's process
a capture's conversion held the interpreter lock RTI's loop needs — 37 late ticks and a LOOP_DEGRADED every 3 s during
a 60 s capture (api_tests/75). Here it has an interpreter of its own. Logs to logs/recorder.log.
"""
from __future__ import annotations

import logging
import os
import signal
import sys
import time

_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _ROOT)

from config.cpu_cores import GENERAL_CPU_CORES                                   # noqa: E402
from project_paths import LOGS_DIR, SYSTEM_XML, TEMP_DATA_DIR                     # noqa: E402
from utility.logger_setup import setup_logging                                    # noqa: E402

logger = logging.getLogger("rec")  # docs/logging.md "Logger names"
LOG_FILE = os.path.join(str(LOGS_DIR), "recorder.log")
POLL_S = 0.05                     # 20 Hz: ~12 rows a poll; the ring holds 5 s
STATS_EVERY_S = 600.0
SETTINGS_CHECK_S = 1.0            # system.xml's modification time, looked at once a second
STATUS_IDLE_S = 1.0               # the status JSON: every pass while a capture runs, else once a second
_stop = False


def _on_term(_s, _f):
    global _stop
    _stop = True


def _mtime(path):
    try:
        return os.stat(path).st_mtime_ns
    except OSError:
        return None


def main() -> int:
    setup_logging(LOG_FILE, log_to_console=False)
    from utility.process_utils import setup_process
    setup_process(GENERAL_CPU_CORES)                     # normal scheduling, never RTI's or the CAN core
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGINT, _on_term)
    from data.shared_data import SharedString
    from utility.activity_log import get_rec_activity_logger, log_activity
    from utility import recorder_settings
    from utility.trace_recorder import Recorder
    from utility.trace_ring import TraceReader

    error_state = SharedString("error_state", 4096)
    capture_request = SharedString("capture_request", 1024)
    status_str = SharedString("recorder_status", 2048)
    act_logger = get_rec_activity_logger()
    settings, problems = recorder_settings.read(SYSTEM_XML)
    for p in problems:
        logger.warning(f"system.xml <recorder>: {p} — its default is used")
    settings_mtime = _mtime(SYSTEM_XML)
    last_settings_check = last_status = 0.0

    def activity(level, code, message, cid, origin):
        log_activity(act_logger, "REC", level, code, id=cid, message=message, origin=origin)

    reader = rec = None
    last_stats = time.monotonic()
    logger.info("recorder up: waiting for RTI's trace")
    while not _stop:
        t0 = time.monotonic()
        try:
            if reader is None or reader.stale():
                reader = TraceReader.attach()
                if reader is not None:
                    rec = Recorder(reader.layout, TEMP_DATA_DIR / "incidents", TEMP_DATA_DIR / "captures", activity)
                    rec.apply_settings(settings)
                    logger.info(f"attached to RTI's trace: {len(reader.layout['arrays'])} channels, "
                                f"{reader.width} values a tick, generation {reader.generation:.0f}; settings {settings}")
            if t0 - last_settings_check > SETTINGS_CHECK_S:
                last_settings_check = t0
                m = _mtime(SYSTEM_XML)
                if m != settings_mtime:
                    settings_mtime = m
                    new, problems = recorder_settings.read(SYSTEM_XML)
                    for p in problems:
                        logger.warning(f"system.xml <recorder>: {p} — its default is used")
                    if new != settings:
                        logger.info(f"settings changed: {settings} -> {new}")
                        settings = new
                        if rec is not None:
                            rec.apply_settings(settings)
            if reader is not None and rec is not None:
                rows, lost = reader.poll()
                rec.feed(rows, lost, t0)
                rec.on_errors(error_state.get_json(), t0)
                rec.on_request(capture_request.get_json() or None)
                rec.capture_overdue()
                reader.heartbeat()
                if rec.capture is not None or t0 - last_status > STATUS_IDLE_S:
                    last_status = t0
                    status = rec.status()
                    status["time"] = time.time()
                    status_str.set_json(status)
                if t0 - last_stats > STATS_EVERY_S:
                    last_stats = t0
                    logger.info(f"incidents written {len(rec.written)}, dropped (rate) {rec.dropped}, too early "
                                f"{rec.too_early}; captures {len(rec.captured)}")
        except Exception:                                # the recorder must keep running; the supervisor sees a crash
            logger.exception("recorder pass failed")
        time.sleep(max(0.0, POLL_S - (time.monotonic() - t0)))
    logger.info("recorder stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
