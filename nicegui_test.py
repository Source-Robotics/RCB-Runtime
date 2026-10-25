"""NiceGUI robot dashboard — Source Robotics."""

from nicegui import ui, app
from data import shared_data
import math
import asyncio
import time
import psutil
import os
import queue
import json
import gc
import logging
logger = logging.getLogger("gui")  # docs/logging.md "Logger names"
import sys       # Firmware tab: spawns the flasher CLI with this interpreter
import signal    # Firmware tab: SIGINT to cancel a running flash

# ── GC tuning: smooth out the periodic ~1 ms plot freezes ──────────────────
# The live-plot loop rebuilds the whole windowed series each frame (~5k short-
# lived list objects at 20 Hz). That high allocation rate makes CPython's
# generational GC fire a full gen2 collection every few seconds → a brief stall.
# Raising the thresholds (esp. gen2) makes those full collections far rarer; the
# objects are tiny and short-lived, so the small memory trade-off is worth the
# smoothness. Reference counting still frees everything promptly regardless.
gc.set_threshold(50_000, 500, 1000)   # default is (700, 10, 10)

# The dashboard's own log, logs/gui.log (2026-10-15). Until then this process never set up logging: its INFO lines
# were dropped and its warnings reached GUI.out.log through Python's last-resort handler, without a time.
from project_paths import GUI_LOG                       # noqa: E402
from utility.logger_setup import setup_logging          # noqa: E402
setup_logging(str(GUI_LOG), log_to_console=True)

from gui_command_sender import ExecCommandSender, GUI_IDLE_RELEASE_S, auto_release_due
from config.xml_parser import load_system_config, load_full_robot_config
from config.system_config import publish_static_system_info, publish_memory_info, publish_disk_info
from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR, ACTIVITY_RTI_LOG, ACTIVITY_EXEC_LOG, ACTIVITY_REC_LOG, CONFIG_DIR
from utility.signal_filters import FILTER_KINDS
# Pure constants — importing rti_handler pulls no hardware, it is the RT-side motion
# generator module. Used so the RTI panel shows the real budget rather than a copy of it
# that can drift (TODO/RTI_mode_plan.md §9.1 is what happens when numbers get retyped).
from motion.rti_handler import COMMAND_TIMEOUT_S, SUCCESS_WINDOW_TICKS, SUCCESS_WARN, SUCCESS_BAD
from utility.filter_config import ALL_SIGNALS, default_filter_config
from gui_processes import PROCESS_COLUMNS, add_processes_card, process_rows   # System Info: processes / threads / cores
from data.shared_memory_index import ESTOP_1 as _ESTOP_1   # for plot event markers

shared_data.setup(joints=6)

# Populate the System Info shared strings. Nothing else in the stack ever wrote them, so the
# System Info tab read them back empty and showed "---" for every field. The GUI is the natural
# producer: it runs on the box at normal priority and already uses psutil, so this stays well
# clear of RTI's RT loop. Platform/CPU cannot change without a reboot -> publish once, here.
# Memory is refreshed on a slow timer further down (it moves, but not fast).
publish_static_system_info(shared_data)
publish_memory_info(shared_data)
publish_disk_info(shared_data)

JOINTS = 6

_sys_cfg = load_system_config(SYSTEM_XML, ROBOTS_DIR)
# Times shown in the box's chosen zone (config/system.xml <display_timezone>, the System Info tab's "Time" card)
import utility.display_time as display_time  # noqa: E402
display_time.set_zone(_sys_cfg.get("display_timezone"))
_xml_cfg = load_full_robot_config(
    str(_sys_cfg['robot_xml_path']),
    grippers_dir=GRIPPERS_DIR,
    active_gripper=_sys_cfg['active_gripper'],
)

from utility.process_utils import setup_process
setup_process(core=1, rt=False)   # GUI: own core, normal scheduling (only RTI is RT)


# RTI process liveness tracking — compared each GUI refresh tick
_prev_rti_tick  = -1
_rti_stale_cnt  = 0   # consecutive refreshes with no tick change

# Refresh rates — tables and plots run on SEPARATE timers so each can be tuned
# without affecting the other. Tables only need ~10 Hz to be readable (faster
# just makes the numbers jitter); plots benefit from a higher rate for smooth
# scrolling. Plot buffer sizing + rebuild cadence derive from PLOT_REFRESH_HZ.
TABLE_REFRESH_HZ = 10
TABLE_REFRESH_S  = 1.0 / TABLE_REFRESH_HZ
PLOT_REFRESH_HZ  = 20
PLOT_REFRESH_S   = 1.0 / PLOT_REFRESH_HZ

# GUI loop timing
_gui_last_t      = 0.0
_gui_exec_last   = 0.0   # exec time of the previous iteration (ms)
_gui_period_ema  = 0.0
_gui_exec_ema    = 0.0
_gui_period_peak = 0.0
_gui_exec_peak   = 0.0
_GUI_EMA_ALPHA   = 0.1   # smoothing factor for EMA

# rti['rti_substate'] -> (badge text, quasar colour). Codes are owned by
# motion/rti_handler.py::SUBSTATE_CODE; keep the two in step.
_RTI_SUBSTATE_UI = {
    0: ("NO PC PAIRED",   "grey"),
    1: ("CONNECTED",      "blue"),      # streaming state, control not claimed
    2: ("CONTROL ACTIVE", "positive"),  # PC is driving; timing rules enforced
    3: ("STOPPING",       "warning"),   # clean release, holding position
    4: ("STOPPED: ERROR", "negative"),  # timeout / packet loss, error latched
}

# Plot-loop timing (separate timer) — used for the FPS readout on the Plots tab.
_plot_last_t      = 0.0
_plot_period_ema  = 0.0

# ─── Activity Log: gui_activity_tab.py (2026-10-15) ────────────────────────────
_ACTIVITY_LOG_SOURCES = {"RTI": str(ACTIVITY_RTI_LOG), "EXEC": str(ACTIVITY_EXEC_LOG),
                         "REC": str(ACTIVITY_REC_LOG)}      # the recorder's captures (2026-10-16)


def fmt(x, decimals=3):
    """Format a float; returns '---' for non-finite values."""
    if not math.isfinite(x):
        return "---"
    return f"{x:.{decimals}f}"


# ─── Angles toggle (2026-10-21, user): rad or deg on every tab ─────────────────
# Display only. Shared memory, the API and every computation stay in radians; what the dashboard SHOWS converts. One
# factor serves an angle and all its derivatives alike (rad, rad/s, rad/s², rad/s³ -> deg, deg/s, ...). Per PAGE, like
# the Plots tab's Units toggle: NiceGUI 3's script mode runs this file once per page load, so each open browser page
# has its own choice, and a reload starts in radians again.
# Switched in the status bar (_set_angle_unit, next to update_tables).
_ANGLE_UNIT = ['rad']                     # 'rad' | 'deg'
_RAD_TO_DEG = 180.0 / math.pi


def _ang(x):
    """An angle, angular speed, acceleration or jerk (stored in rad...) in the display unit."""
    return x * _RAD_TO_DEG if _ANGLE_UNIT[0] == 'deg' else x


def _ang_tcp(x, i):
    """Component i of a TCP 6-vector [x, y, z, rx, ry, rz] (pose, twist, acceleration) in the display unit: the
    three rotation components convert, x / y / z (metres) never do."""
    return _ang(x) if i >= 3 else x


def _ang_label(text):
    """A unit label in the display unit: 'Speed (rad/s)' -> 'Speed (deg/s)'. Only for labels whose 'rad' is a unit."""
    return text.replace('rad', 'deg') if _ANGLE_UNIT[0] == 'deg' else text


# ─────────────────────────────────────────────────────────────
# Command channel to command_executor
# ─────────────────────────────────────────────────────────────
# Commands go through command_executor's validated TCP JSON protocol (via a
# RobotClient), not a raw ZMQ push to RTI — so RTI has ONE arbitrated command
# source, not two. See TODO/gui_executor_migration_plan.md.
#
# RobotClient calls block on the executor ack and raise on rejection/timeout,
# which a UI handler must not do. So handlers just _exec(fn) and return: one
# background worker owns the client, runs submissions in order, reconnects if
# the executor restarts, and never lets an error escape. Failures are logged by
# the sender and surfaced to the user as a toast, drained by a ui.timer below.
_gui_toasts = queue.Queue()   # (message, type) pending toasts → drained in the UI context


def _on_exec_error(label, exc):
    if label and label.endswith("_poll"):         # a background poll (Walls tab): log it, never toast it
        logger.debug("%s failed: %s", label, exc)
        return
    _gui_toasts.put((f"{label or 'command'}: {exc}", "negative"))


_exec_sender = ExecCommandSender(on_error=_on_exec_error)


# When this dashboard last sent the executor anything. The dashboard is ONE permanent connection,
# so without an idle release a single click would own the robot until the executor's 30 s timeout —
# long enough to make every script started after "Clear Errors, Enable" fail with CONTROL_HELD
# (three times in one session, 2026-09-30). See auto_release_due() for exactly when it lets go.
_gui_last_cmd = [time.monotonic()]
_gui_release_sent = [False]


def _exec(fn, label=""):
    """Submit a RobotClient call to the executor. Non-blocking; never raises into the UI."""
    if label != "release_control":
        _gui_last_cmd[0] = time.monotonic()
        _gui_release_sent[0] = False
    _exec_sender.submit(fn, label=label)


# ─────────────────────────────────────────────────────────────
# Control ownership (TODO/control_arbitration_plan.md §4.3)
# ─────────────────────────────────────────────────────────────
# The executor decides who holds control, per TCP CONNECTION. This dashboard is ONE connection
# (the sender's RobotClient) shared by every browser tab, so it is one holder: whoever clicks
# Enable or jogs first claims control for all of them, and it frees itself after 30 s idle.
# That is display-level only — the executor enforces regardless of what is greyed here.
#
# Written by _refresh_control_state() on a timer, read by handlers. Stale by up to the timer
# period, which is fine: a wrong guess here costs a refused command and a toast, never motion.
_ctl_view = {"holder": None, "mine": False, "other": False}


async def handle_take_control():
    """Take control from whoever holds it — a deliberate act, so it asks first.

    Taking control does NOT stop what the previous holder already queued: ownership gates
    NEW commands only. The dialog says so, because "take control" reads as "and stop them".
    """
    holder = _ctl_view["holder"] or "another client"
    gate = bool(float(shared_data.rti["force_confirm_enabled"][0]) >= 0.5)
    with ui.dialog() as dlg, ui.card().classes("max-w-xl"):
        ui.label("Take control?").classes("text-lg font-bold")
        ui.markdown(
            f"**{holder}** holds control and may be running a program.\n\n"
            "Taking control does **not** stop anything it already queued — press Stop "
            "first if that matters."
            + ("\n\nThe **control box will ask for confirmation**: hold B2 to allow, "
               "tap B1 to deny. This waits up to 20 s." if gate else
               "\n\nTakeover approval is OFF, so this takes effect immediately.")
        ).classes("text-sm")
        with ui.row().classes("gap-2 justify-end w-full mt-2"):
            ui.button("Cancel", on_click=lambda: dlg.submit(False)).props("outline")
            ui.button("Take control", on_click=lambda: dlg.submit(True)).props("color=orange")
    if not await dlg:
        return
    ui.notify("Waiting for approval at the control box…" if gate else "Taking control…",
              type="info")

    def _take(r):
        r.take_control(force=True)
        _gui_last_cmd[0] = time.monotonic()      # a takeover that waited 20 s is not "5 s idle"
        _gui_release_sent[0] = False
        _gui_toasts.put(("This dashboard now holds control.", "positive"))
    # Own thread, NOT the ordered queue: this can block for the whole confirmation window,
    # and Disable / jog-stop must never sit behind it.
    _exec_sender.submit_async(_take, "take_control")


def handle_release_control():
    _exec(lambda r: r.release_control(), "release_control")


def handle_enable():       _exec(lambda r: r.enable(), "enable")
def handle_disable():      _exec(lambda r: r.disable(), "disable")
def handle_home():         _exec(lambda r: r.home(), "home")
def handle_enter_exec():   _exec(lambda r: r.enter_exec_mode(), "enter_exec_mode")
def handle_exit_exec():    _exec(lambda r: r.enter_idle_mode(), "enter_idle_mode")

def handle_clear_errors():
    """Signal RTI to wipe the error latch. Any still-active errors re-latch next tick.

    Also sent through command_executor, whose clear_errors is the only thing that clears a
    MOTION ABORT (a failed queued move stops the program until cleared). The shared-memory
    write stays: it works even when the executor is down, which is the point of it. RTI may
    therefore see two clears in a row — harmless, the second only restarts the settle wait.
    """
    shared_data.clear_error_signal.set("1")
    _exec_sender.submit(lambda r: r.clear_errors(wait=False), "clear_errors")

def handle_reset_loop_stats():
    """Signal RTI to zero the since-start loop-timing peaks (max period/exec/jitter,
    overrun count) and the rolling p95/p99 window. The p95/p99 values themselves
    don't need this to stay meaningful (they're always a rolling last-500-tick
    window) — this is for the "since start" peaks, which otherwise persist across
    RTI restarts (same shared-memory segment, re-attached not recreated)."""
    shared_data.loop_stats_reset_signal.set("1")


def handle_reload_filters():
    """Rebuild the signal filters from config/filters.xml — hot reload, no restart, no re-home. Edit the tau/type
    in the XML, click this, see it live. Since 2026-10-04 this goes through command_executor (`reload_filters`):
    the executor parses the file and hands RTI the spec table, so RTI reads no file in its loop; each filter's
    internal state resets, so a brief (~one filter window) transient is expected right after. A malformed file is
    refused and the filters in force stay."""
    _exec(lambda r: r.reload_filters(), "reload_filters")
    ui.notify("Reloading signal filters from config/filters.xml", type="info")

_sw_estop_btn = None  # set after UI is built

async def handle_limp_toggle():
    """SAFETY_STOP: drive every motor with NOTHING (Position=None, Speed=None,
    Current=0), or release back to IDLE.

    Distinct from SW E-STOP and from disable(): both of those leave a velocity-0
    hold that still resists motion. This removes all authority, so the arm hangs
    on its own mechanics and WILL fall. Hence the dialog — a button this
    destructive must not be a single click.

    Its purpose is diagnostic: with every motor commanded by nothing, anything
    still moving or buzzing is the driver's own firmware rather than the runtime.
    """
    mode = shared_data.robot_mode.get()
    if mode == "SAFETY_STOP":
        _exec(lambda r: r.release_safety_stop(), "release_safety_stop")
        return

    with ui.dialog() as dlg, ui.card().classes("max-w-xl"):
        ui.label("Cut all motor authority?").classes("text-lg font-bold text-red-700")
        ui.markdown(
            "Every joint will be commanded with **Position=None, Speed=None, "
            "Current=0** — no position loop, no velocity loop, no holding torque.\n\n"
            "**THE ARM WILL GO LIMP AND FALL UNDER ITS OWN WEIGHT.**\n\n"
            "This is not the E-Stop and not Disable — both of those keep a "
            "velocity-0 hold that resists motion. This removes it entirely.\n\n"
            "Support the arm, or make sure it is resting somewhere it can drop "
            "safely, before continuing."
        ).classes("text-sm")
        with ui.row().classes("gap-2 justify-end w-full mt-2"):
            ui.button("Cancel", on_click=lambda: dlg.submit(False)).props("outline")
            ui.button("THE ARM IS SUPPORTED — CUT POWER",
                      on_click=lambda: dlg.submit(True)).props("color=red")
    if not await dlg:
        return
    _exec(lambda r: r.safety_stop(), "safety_stop")


def handle_sw_estop_toggle():
    """Toggle software e-stop. Assert if inactive, release if active."""
    current = int(shared_data.rti["software_estop"][0])
    shared_data.rti["software_estop"][0] = 0 if current else 1
    if _sw_estop_btn is not None:
        active = bool(shared_data.rti["software_estop"][0])
        _sw_estop_btn.props("color=orange" if active else "color=red size=sm")
        _sw_estop_btn.text = "Release E-Stop" if active else "SW E-STOP"

# ─────────────────────────────────────────────────────────────
# Column definitions
# ─────────────────────────────────────────────────────────────
motor_columns = [
    {'name': 'joint', 'label': 'Joint',    'field': 'joint'},
    {'name': 'pos',   'label': 'Position', 'field': 'pos'},
    {'name': 'spd',   'label': 'Speed',    'field': 'spd'},
    {'name': 'cur',   'label': 'Current',  'field': 'cur'},
    {'name': 'err',   'label': 'Error',    'field': 'err'},
    {'name': 'HW',    'label': 'HW Ver',   'field': 'HW'},
    {'name': 'SW',    'label': 'SW Ver',   'field': 'SW'},
    {'name': 'SN',    'label': 'Serial',   'field': 'SN'},
]

# What RTI actually PUT ON THE WIRE this tick, in motor space — the literal ints handed to
# Send_data_pack_1 / Send_data_pack_PD. Distinct from motor_columns above, which is what the
# driver reports BACK. "—" means the channel was not commanded (data_pack_1 omits it), which
# is not the same as a commanded 0.
sent_columns = [
    {'name': 'joint', 'label': 'Joint',            'field': 'joint'},
    {'name': 'spos',  'label': 'Position (ticks)', 'field': 'spos'},
    {'name': 'sspd',  'label': 'Speed (ticks/s)',  'field': 'sspd'},
    {'name': 'scur',  'label': 'Current (mA)',     'field': 'scur'},
]

info_columns = [
    {'name': 'joint', 'label': 'Joint',     'field': 'joint'},
    {'name': 'temp',  'label': 'Temp (°C)', 'field': 'temp'},
    {'name': 'volt',  'label': 'Voltage',   'field': 'volt'},
    {'name': 'kt',    'label': 'Kt (Nm/A)', 'field': 'kt'},
    {'name': 'kt_src', 'label': 'Kt from',  'field': 'kt_src'},
]


def _fmt_kt_src(value):
    """Where a Kt value actually came from: read off the motor over CAN, or the XML fallback
    (motor didn't answer, or kt_source=xml). Resolved once at startup by utility/kt_init.py."""
    return "CAN" if float(value) else "XML"

error_columns = [
    {'name': 'joint', 'label': 'Joint',      'field': 'joint'},
    {'name': 'temp',  'label': 'Temp',       'field': 'temp'},
    {'name': 'enc',   'label': 'Encoder',    'field': 'enc'},
    {'name': 'vbus',  'label': 'VBUS',       'field': 'vbus'},
    {'name': 'drv',   'label': 'Driver',     'field': 'drv'},
    {'name': 'vel',   'label': 'Velocity',   'field': 'vel'},
    {'name': 'cur',   'label': 'Current',    'field': 'cur'},
    {'name': 'estop', 'label': 'EStop',      'field': 'estop'},
    {'name': 'wd',    'label': 'Watchdog',   'field': 'wd'},
    {'name': 'cal',   'label': 'Calibrated', 'field': 'cal'},
]

gripper_columns = [
    {'name': 'pos',      'label': 'Position',   'field': 'pos'},
    {'name': 'cur',      'label': 'Current',    'field': 'cur'},
    {'name': 'act',      'label': 'Active',     'field': 'act'},
    {'name': 'action',   'label': 'Action',     'field': 'action'},
    {'name': 'obj',      'label': 'Object',     'field': 'obj'},
    # The GRIPPER MECHANISM's calibration (stroke/endstop sweep — what the Calibrate button runs).
    # Distinct from the motor driver's own 'Motor Calibrated' flag in the gripper motor table.
    {'name': 'cal',      'label': 'Gripper Calibrated', 'field': 'cal'},
    {'name': 'temp_err', 'label': 'Temp Err',   'field': 'temp_err'},
    {'name': 'timeout',  'label': 'Timeout',    'field': 'timeout'},
    {'name': 'err',      'label': 'Error',      'field': 'err'},
    {'name': 'estop',    'label': 'EStop',      'field': 'estop'},
]

joint_columns = [
    {'name': 'j',        'label': 'Joint',          'field': 'j'},
    {'name': 'homed',    'label': 'Homed',           'field': 'homed'},
    {'name': 'pos',      'label': 'Position (rad)',  'field': 'pos'},
    {'name': 'spd',      'label': 'Speed (rad/s)',   'field': 'spd'},
    {'name': 'trq',      'label': 'Torque (Nm)',     'field': 'trq'},
    {'name': 'acc',      'label': 'Accel (rad/s²)',  'field': 'acc'},
    {'name': 'jrk',      'label': 'Jerk (rad/s³)',   'field': 'jrk'},
    {'name': 'pos_cmd',  'label': 'Pos Cmd (rad)',   'field': 'pos_cmd'},
    {'name': 'spd_cmd',  'label': 'Spd Cmd (rad/s)', 'field': 'spd_cmd'},
    {'name': 'trq_cmd',  'label': 'Trq Cmd',         'field': 'trq_cmd'},
    {'name': 'trq_ext',  'label': 'Ext Torque',      'field': 'trq_ext'},
]

tcp_columns = [
    {'name': 'axis',      'label': 'Axis',       'field': 'axis'},
    {'name': 'pose',      'label': 'Pose (m / rad)',        'field': 'pose'},
    {'name': 'spd',       'label': 'Speed (m/s / rad/s)',   'field': 'spd'},
    {'name': 'force_ext', 'label': 'F External',  'field': 'force_ext'},
    {'name': 'pose_cmd',  'label': 'Pose Cmd (m / rad)',    'field': 'pose_cmd'},
    {'name': 'spd_cmd',   'label': 'Spd Cmd (m/s / rad/s)', 'field': 'spd_cmd'},
]

rti_columns = [
    {'name': 'node',     'label': 'Node',     'field': 'node'},
    {'name': 'scan',     'label': 'Scan',     'field': 'scan'},
    {'name': 'err_cnt',  'label': 'Errors',   'field': 'err_cnt'},
    {'name': 'old_data', 'label': 'Old Data', 'field': 'old_data'},
    {'name': 'conn_err', 'label': 'Conn Err', 'field': 'conn_err'},
]

# Processes panel (System Info tab): gui_processes.py (processes, threads, the core plan; 2026-10-13).
process_columns = PROCESS_COLUMNS

pid_columns = [
    {'name': 'joint',    'label': 'Joint',          'field': 'joint'},
    {'name': 'KPP',      'label': 'KPP (Pos P)',    'field': 'KPP'},
    {'name': 'KPV',      'label': 'KPV (Vel P)',    'field': 'KPV'},
    {'name': 'KIV',      'label': 'KIV (Vel I)',    'field': 'KIV'},
    {'name': 'KPIQ',     'label': 'KPIQ (Cur P)',   'field': 'KPIQ'},
    {'name': 'KIIQ',     'label': 'KIIQ (Cur I)',   'field': 'KIIQ'},
    {'name': 'KP',       'label': 'KP (Imp P)',     'field': 'KP'},
    {'name': 'KD',       'label': 'KD (Imp D)',     'field': 'KD'},
    {'name': 'Ilim',     'label': 'Iq Limit (mA)',  'field': 'Ilim'},
    {'name': 'vel_lim',  'label': 'Vel Limit (t/s)','field': 'vel_lim'},
    {'name': 'vlim',     'label': 'V Limit (mV)',   'field': 'vlim'},
]


def _fmt_vlim(value):
    """Voltage limit for the config tables. 0 is the driver's "use VBUS" mode, not a 0 mV limit —
    show it as such rather than a bare zero."""
    return "VBUS" if float(value) == 0.0 else f"{float(value):.0f}"

# ─────────────────────────────────────────────────────────────
# STATUS BAR  (always visible, top of page)
# ─────────────────────────────────────────────────────────────
with ui.card().classes("w-full mb-2"):
    with ui.row().classes("items-center gap-3 flex-wrap px-3 py-2"):
        ui.label("Status").classes("font-bold text-sm text-gray-500 uppercase tracking-wide")
        robot_mode_badge  = ui.badge("MODE: ---")
        robot_state_badge = ui.badge("STATE: ---").props("color=blue outline")
        homed_badge       = ui.badge("NOT HOMED").props("color=orange outline")
        # Shown only while RTI runs the SIMULATED robot (tools/sim_run.py; RTI writes sim_mode at start, 2026-10-13).
        sim_badge         = ui.badge("SIMULATION").props("color=purple").tooltip(
            "RTI runs the SIMULATED robot (tools/sim_run.py): simulated drives on a virtual CAN bus — no arm moves, "
            "and the I/O is simulated too")
        sim_badge.visible = False
        error_badge       = ui.badge("ERR: 0").props("color=grey outline")
        safety_badge      = ui.badge("SAFETY: OK").props("color=positive outline")
        rti_process_badge = ui.badge("RTI: ---").props("color=grey outline")
        robot_can_badge   = ui.badge("ROBOT: ---").props("color=grey outline")
        # Self-collision: is the worker watching, and how close is the arm to where a jog stops (gui_self_collision.py)
        try:
            from gui_self_collision import add_self_collision_badge
            add_self_collision_badge(shared_data, _sys_cfg["self_collision"], int(_sys_cfg["joint_num"]))
        except Exception:
            logger.exception("self-collision badge unavailable")
        # Who holds control. Shown on every tab: the failure it prevents is a colleague's
        # script and this dashboard both believing they are driving.
        control_badge     = ui.badge("CONTROL: ---").props("color=grey outline")
        _take_btn = ui.button("Take control", icon="pan_tool", on_click=handle_take_control) \
            .props("color=orange size=sm")
        _take_btn.set_visibility(False)
        _release_btn = ui.button("Release", icon="lock_open", on_click=handle_release_control) \
            .props("color=grey outline size=sm")
        _release_btn.set_visibility(False)
        _release_btn.tooltip("Free the robot now instead of waiting for the 30 s idle timeout.")
        # A running block program (blockly/), on every tab: its name (click: the Blocks tab) and a Stop for it.
        try:
            from blockly.nicegui_tab import add_program_badge
            add_program_badge(open_blocks=lambda: tabs.set_value('Blocks'))
        except Exception:
            logger.exception("block program badge unavailable")
        ui.separator().props("vertical")
        # FIXED widths (2026-10-17): the text changes with every command (a - jog's is one character longer, jog_stop
        # much shorter) — a label that grew and shrank re-wrapped this row, the page jumped and a held jog button slid
        # out from under the pointer (its mouseleave stops the jog). A long command is cut with '…'.
        robot_cmd_label = ui.label("CMD: ---").classes("font-mono text-sm text-gray-500 w-64 truncate")
        recipe_label    = ui.label("Recipe: ---").classes("font-mono text-sm text-gray-500 w-44 truncate")
        ui.separator().props("vertical")
        _sw_estop_btn = ui.button("SW E-STOP", icon="emergency", on_click=handle_sw_estop_toggle).props("color=red size=sm")
        ui.separator().props("vertical")
        _limp_btn = ui.button("MOTORS OFF", icon="power_settings_new",
                              on_click=lambda: handle_limp_toggle()).props("color=deep-orange size=sm")
        ui.separator().props("vertical")

        _grav_comp_state = [True]
        def _handle_grav_comp_toggle():
            _grav_comp_state[0] = not _grav_comp_state[0]
            enabled = _grav_comp_state[0]
            _exec(lambda r: r.set_gravity_comp(enabled), "gravity_comp")
            _grav_comp_btn.props(f"color={'positive' if enabled else 'grey'}")
            _grav_comp_btn.set_text("Grav Comp ON" if enabled else "Grav Comp OFF")
        _grav_comp_btn = ui.button("Grav Comp ON", icon="downhill_skiing",
                                   on_click=_handle_grav_comp_toggle).props("color=positive outline size=sm")

        # Inertia / Coriolis feedforward on the commanded motion (2026-10-10, utility/mode_dispatch._add_dynamics_ff):
        # two switches, starting as system.xml <dynamics_feedforward> says (ON as shipped). The buttons read the ROBOT's
        # state (synced in update_tables), so they stay right when a script toggles them.
        def _dyn_toggle(key, cmd, call):
            enabled = not (shared_data.rti[key][0] > 0.5)
            _exec(lambda r: call(r, enabled), cmd)
        _inertia_btn = ui.button("Inertia OFF", icon="speed",
                                 on_click=lambda: _dyn_toggle("inertia_comp_enabled", "inertia_comp",
                                                              lambda r, e: r.set_inertia_comp(e))).props("color=grey outline size=sm")
        _inertia_btn.tooltip("Inertia feedforward M(q)·q̈ on the commanded motion — EXEC, JOG, RTI position/velocity. "
                             "Not in freedrive, holds, homing or RTI torque commands. Start state: system.xml <dynamics_feedforward>.")
        _coriolis_btn = ui.button("Coriolis OFF", icon="sync_alt",
                                  on_click=lambda: _dyn_toggle("coriolis_comp_enabled", "coriolis_comp",
                                                               lambda r, e: r.set_coriolis_comp(e))).props("color=grey outline size=sm")
        _coriolis_btn.tooltip("Coriolis / centripetal feedforward C(q, q̇)·q̇ on the commanded motion — same modes as "
                              "Inertia. Start state: system.xml <dynamics_feedforward>.")

        # Friction (utility/friction_model.py): NO buttons (2026-10-17, user, after measuring both on the arm — the
        # feedforward does not improve the moves (api_tests/66), freedrive compensation is ~0.04-0.10 Nm with its 0.7
        # share). The model is used by the external-torque estimate, always; the robot's friction_ff / friction_comp
        # commands still switch the two (OFF at every start). docs/friction_compensation.md.
        _owned_btns = [_grav_comp_btn, _inertia_btn, _coriolis_btn]   # greyed while someone else holds control (see below)

        ui.separator().props("vertical")
        # Runtime payload (utility/payload.py, 2026-10-04): the part the gripper holds — mass + centre of mass in
        # the TOOL frame (mm here, metres on the wire). An OWNED command: the gravity model and the freedrive
        # walls' dynamics change with it. The live label shows what RTI has (shared memory), not the fields.
        with ui.row().classes("items-center gap-1"):
            _pl_mass = ui.number("Payload kg", value=0.0, min=0.0, step=0.05, format="%.3f").props("dense outlined").classes("w-28")
            _pl_x = ui.number("COM x mm", value=0.0, step=1, format="%.0f").props("dense outlined").classes("w-24")
            _pl_y = ui.number("y mm", value=0.0, step=1, format="%.0f").props("dense outlined").classes("w-20")
            _pl_z = ui.number("z mm", value=0.0, step=1, format="%.0f").props("dense outlined").classes("w-20")

            def _apply_payload():
                m = float(_pl_mass.value or 0.0)
                com = tuple(float(v.value or 0.0) / 1000.0 for v in (_pl_x, _pl_y, _pl_z))
                _exec(lambda r: r.set_payload(m, com), "set_payload")
                ui.notify(f"Payload {m:.3f} kg at ({com[0]*1000:.0f}, {com[1]*1000:.0f}, {com[2]*1000:.0f}) mm requested", type="info")

            _pl_btn = ui.button("Apply", icon="fitness_center", on_click=_apply_payload).props("size=sm outline") \
                .tooltip("Declare the part the gripper holds (tool frame: +z out of the flange). 0 = gripper alone. "
                         "Refused above the robot XML's <max_payload_kg>. Not persisted — 0 after every RTI start.")
            _pl_live = ui.label("RTI: 0.000 kg").classes("text-xs text-gray-500 font-mono") \
                .tooltip("What RTI's gravity model carries right now (shared memory payload_mass / payload_com_*)")
        _owned_btns.append(_pl_btn)

        ui.separator().props("vertical")

        # Physical confirmation for a FORCED control takeover (arbitration plan 4.4).
        # Reads its initial state from shared memory rather than assuming, so the button
        # never shows OFF while the robot is actually enforcing it.
        _fc_state = [bool(float(shared_data.rti["force_confirm_enabled"][0]) >= 0.5)]

        def _fc_label(on):
            return "Takeover approval ON" if on else "Takeover approval OFF"

        def _handle_force_confirm_toggle():
            _fc_state[0] = not _fc_state[0]
            on = _fc_state[0]
            _exec(lambda r: r.set_force_confirm(on), "set_force_confirm")
            _fc_btn.props(f"color={'positive' if on else 'grey'}")
            _fc_btn.set_text(_fc_label(on))
            ui.notify(
                "Takeover approval ON: taking control away from a program that holds it now needs "
                "B2 held on the control box."
                if on else
                "Takeover approval OFF: any program or PC on the network can take control away "
                "from another at once, without anyone at the robot agreeing.",
                type="positive" if on else "warning")

        _fc_btn = ui.button(_fc_label(_fc_state[0]), icon="touch_app",
                            on_click=_handle_force_confirm_toggle) \
            .props(f"color={'positive' if _fc_state[0] else 'grey'} outline size=sm")
        _fc_btn.tooltip("Click to switch. The ? next to it explains what it does.")
        with ui.icon("help_outline").classes("text-gray-400 cursor-help text-sm"):
            with ui.tooltip().classes("max-w-sm"), ui.column().classes("gap-1 py-1"):
                ui.label("Takeover approval: can someone take the robot away from a program "
                         "that is using it?").classes("font-semibold")
                ui.label("Only one program or PC controls the robot at a time (this dashboard, "
                         "a script, a remote PC). Another one can take control away from it — "
                         "\"Take control\", e.g. to rescue a stuck script.")
                ui.label("ON: someone AT THE ROBOT must agree. The control box shows who is "
                         "asking; HOLD B2 for 1 s to allow, PRESS B1 to refuse. No answer in "
                         "20 s = refused.")
                ui.label("OFF: any program or PC on the network takes control at once — "
                         "nobody at the robot is asked.")
                ui.label("Never affected: taking a robot nobody controls, and Stop, Disable, "
                         "Clear Errors, E-stop — those always work for everyone.") \
                    .classes("italic")
                ui.label("Anyone may switch it ON; only whoever controls the robot (or anyone, "
                         "when nobody does) may switch it OFF.").classes("italic")

        ui.separator().props("vertical")
        _owned_btns.append(ui.button("Enable",      icon="power",       on_click=handle_enable).props("color=positive outline size=sm"))
        # Disable is NEVER greyed: it is ALWAYS_OPEN in the executor, and "lock the queue,
        # never lock the stop" applies to the display too. Same for Clear Errors, SW E-STOP
        # and MOTORS OFF — none of them is in _owned_btns.
        ui.button("Disable",     icon="power_off",      on_click=handle_disable).props("color=grey outline size=sm")
        _owned_btns.append(ui.button("Home",        icon="home",        on_click=handle_home).props("color=primary outline size=sm"))
        _owned_btns.append(ui.button("Enter EXEC",  icon="play_circle", on_click=handle_enter_exec).props("color=secondary outline size=sm"))
        _owned_btns.append(ui.button("Exit EXEC",   icon="stop_circle", on_click=handle_exit_exec).props("color=grey outline size=sm"))
        ui.button("Clear Errors", icon="cancel",        on_click=handle_clear_errors).props("color=negative outline size=sm")
        ui.separator().props("vertical")
        # Display only (_ANGLE_UNIT): not an owned command, so never greyed.
        ui.label("Angles").classes("text-sm text-gray-500")
        ui.toggle({'rad': 'rad', 'deg': 'deg'}, value=_ANGLE_UNIT[0],
                  on_change=lambda e: _set_angle_unit(e.value)).props("dense") \
            .tooltip("Show angles, angular speeds, accelerations and jerks in radians or degrees, on every tab "
                     "(tables, plots, their CSV, Plan Preview, jog speeds). Display only: nothing sent to the robot "
                     "changes. Back to radians when the page reloads.")

# ─────────────────────────────────────────────────────────────
# JOG CONTROLS
# ─────────────────────────────────────────────────────────────
# Read live jog config from shared memory so a fresh GUI tab reflects whatever
# RTI currently has — XML defaults at boot, or whatever the user last set.
# RTI writes these via JogHandler._mirror_all_to_shared_memory + set_* methods.
_PROFILE_IDX_TO_NAME = {0: 'trapezoidal', 1: 'scurve'}
_CONTROL_IDX_TO_NAME = {0: 'pid',         1: 'pd'}

_initial_speed_pct    = float(shared_data.rti['jog_speed_pct'][0])
_initial_accel_time   = float(shared_data.rti['jog_accel_time'][0])
_initial_jerk_factor  = float(shared_data.rti['jog_jerk_factor'][0])
_initial_profile      = _PROFILE_IDX_TO_NAME.get(int(shared_data.rti['jog_profile_idx'][0]),  'scurve')
_initial_control_mode = _CONTROL_IDX_TO_NAME.get(int(shared_data.rti['jog_control_mode'][0]), 'pid')

# Safety fallback: if RTI hasn't started yet, shared memory reads as 0 — use
# the XML-equivalent defaults so the GUI is usable in isolation.
if _initial_speed_pct   <= 0.0: _initial_speed_pct   = 0.20
if _initial_accel_time  <= 0.0: _initial_accel_time  = 0.55
if _initial_jerk_factor <= 0.0: _initial_jerk_factor = 3.0

_jog_state = {
    'speed_pct':    _initial_speed_pct,
    'accel_time':   _initial_accel_time,
    'profile':      _initial_profile,
    'jerk_factor':  _initial_jerk_factor,
    'control_mode': _initial_control_mode,
}

def _jog_start(joint_idx, direction):
    # Gate the START only. The release path (_jog_stop) must ALWAYS go through, and this is
    # why the card is dimmed with opacity and not pointer-events:none — a card that stopped
    # receiving the mouse-up because control changed hands mid-press would leave the arm
    # jogging with no way for its own button to end it.
    if _ctl_view["other"]:
        return
    speed = _jog_state['speed_pct'] * 100.0          # the slider's fraction; jog() takes percent (protocol 2)
    _exec(lambda r: r.jog(joint_idx, direction, speed), "jog_set")

def _jog_stop():
    _exec(lambda r: r.jog_stop(), "jog_stop")

with ui.card().classes("w-full mb-2") as _jog_card:
    with ui.row().classes("items-center gap-3 flex-wrap px-3 py-2"):
        ui.label("Jog").classes("font-bold text-sm text-gray-500 uppercase tracking-wide")
        ui.button("Enter JOG", icon="open_with",
                  on_click=lambda: _exec(lambda r: r.enter_jog_mode(), "enter_jog_mode")
                  ).props("color=primary outline size=sm")
        ui.button("Exit JOG", icon="close",
                  on_click=lambda: _exec(lambda r: r.enter_idle_mode(), "enter_idle_mode")
                  ).props("color=grey outline size=sm")
        ui.separator().props("vertical")
        ui.label("Speed:").classes("text-sm text-gray-500")
        _speed_init = int(round(_initial_speed_pct * 100))
        _jog_speed_slider = ui.slider(min=1, max=100, value=_speed_init).classes("w-36")
        _jog_speed_label  = ui.label(f"{_speed_init}%").classes("text-sm font-mono w-8")

        def _on_jog_speed(e):
            _jog_state['speed_pct'] = e.args / 100.0
            _jog_speed_label.set_text(f"{e.args}%")
            _refresh_jog_speed_labels()
        _jog_speed_slider.on('update:model-value', _on_jog_speed)

        ui.separator().props("vertical")
        ui.label("Accel:").classes("text-sm text-gray-500")
        _accel_init = int(round(_initial_accel_time * 100))
        _jog_accel_slider = ui.slider(min=5, max=400, value=_accel_init).classes("w-36")
        _jog_accel_label  = ui.label(f"{_initial_accel_time:.2f}s").classes("text-sm font-mono w-12")

        def _on_jog_accel(e):
            t = e.args / 100.0
            _jog_state['accel_time'] = t
            _jog_accel_label.set_text(f"{t:.2f}s")
            _exec(lambda r: r.set_jog_accel(t), "jog_accel")
        _jog_accel_slider.on('update:model-value', _on_jog_accel)

        ui.separator().props("vertical")
        ui.label("Profile:").classes("text-sm text-gray-500")

        def _on_jog_profile(e):
            _jog_state['profile'] = e.value
            _exec(lambda r: r.set_jog_profile(e.value), "jog_profile")
        _jog_profile_toggle = ui.toggle({'trapezoidal': 'TRAPEZOIDAL', 'scurve': 'S-CURVE'},
                                        value=_initial_profile, on_change=_on_jog_profile)

        with ui.row().classes("items-center gap-2") as _jog_jerk_row:
            ui.separator().props("vertical")
            ui.label("Jerk:").classes("text-sm text-gray-500")
            _jerk_init = int(round(_initial_jerk_factor * 10))
            _jog_jerk_slider = ui.slider(min=5, max=200, value=_jerk_init).classes("w-28")
            _jog_jerk_label  = ui.label(f"{_initial_jerk_factor:.1f}×").classes("text-sm font-mono w-8")
        _jog_jerk_row.bind_visibility_from(_jog_profile_toggle, 'value', value='scurve')

        def _on_jog_jerk(e):
            factor = e.args / 10.0
            _jog_state['jerk_factor'] = factor
            _jog_jerk_label.set_text(f"{factor:.1f}×")
            _exec(lambda r: r.set_jog_jerk_factor(factor), "jog_jerk_factor")
        _jog_jerk_slider.on('update:model-value', _on_jog_jerk)

        ui.separator().props("vertical")
        ui.label("Control:").classes("text-sm text-gray-500")

        def _on_jog_control_mode(e):
            _jog_state['control_mode'] = e.value
            _exec(lambda r: r.set_jog_control_mode(e.value), "jog_control_mode")
        ui.toggle(['pid', 'pd'], value=_initial_control_mode, on_change=_on_jog_control_mode)

    _jog_buttons = []  # list of {'neg': btn, 'pos': btn} — used by update_tables() for block indicators

    # Tracks whether a jog button is currently held. Lets mouseleave act as a
    # drag-off safety only when a button was actually pressed, instead of
    # firing jog_stop every time the cursor passes over a button.
    _jog_btn_pressed = {'active': False}

    def _jog_button_down(joint_idx, direction):
        _jog_btn_pressed['active'] = True
        _jog_start(joint_idx, direction)

    def _jog_button_up():
        if _jog_btn_pressed['active']:
            _jog_btn_pressed['active'] = False
            _jog_stop()

    _jog_max_vel_rad   = [float(v) for v in _xml_cfg['joint_velocity_limit']]
    _jog_ticks_per_rad = [
        (2 ** int(_xml_cfg['motor_encoder'][i])) * float(_xml_cfg['gear_ratio'][i]) / (2.0 * math.pi)
        for i in range(JOINTS)
    ]
    _jog_speed_labels = []

    def _refresh_jog_speed_labels():
        pct = _jog_state['speed_pct']
        for i, lbl in enumerate(_jog_speed_labels):
            rad_s  = pct * _jog_max_vel_rad[i]
            tick_s = rad_s * _jog_ticks_per_rad[i]
            lbl.set_text(f"{_ang(rad_s):.2f} {_ang_label('rad/s')} · {tick_s:.0f} t/s")

    with ui.row().classes("gap-6 px-3 pb-3 flex-wrap"):
        for _ji in range(6):
            with ui.column().classes("items-start gap-0"):
                with ui.row().classes("items-center gap-1"):
                    ui.label(f"J{_ji + 1}").classes("text-sm font-mono text-gray-500 w-5")
                    _neg = ui.button("◄").props("size=sm flat dense")
                    _pos = ui.button("►").props("size=sm flat dense")
                    _neg.on('mousedown',  lambda e, j=_ji: _jog_button_down(j, -1))
                    _neg.on('mouseup',    lambda e: _jog_button_up())
                    _neg.on('mouseleave', lambda e: _jog_button_up())
                    _pos.on('mousedown',  lambda e, j=_ji: _jog_button_down(j, +1))
                    _pos.on('mouseup',    lambda e: _jog_button_up())
                    _pos.on('mouseleave', lambda e: _jog_button_up())
                    _jog_buttons.append({'neg': _neg, 'pos': _pos})
                _jog_speed_labels.append(
                    ui.label("").classes("text-xs font-mono text-gray-400 pl-6")
                )
    _refresh_jog_speed_labels()

    _JOYSTICK_DEADZONE = 0.04  # ignore x within this fraction of full range

    # A joystick held at a FIXED deflection sends no move events but the joint keeps jogging, so
    # "no recent command" is not "idle" for it: the idle release must see it as active.
    _joystick_active = [False]

    def _joystick_end(e):
        _joystick_active[0] = False
        _exec(lambda r: r.jog_stop(), "jog_stop")

    def _on_joystick_move(e, joint_idx):
        x = e.x  # nipplejs vector component: -1.0 (full left) … +1.0 (full right)
        if abs(x) < _JOYSTICK_DEADZONE:
            _joystick_active[0] = False
            _exec(lambda r: r.jog_stop(), "jog_stop")
            return
        _joystick_active[0] = True
        if _ctl_view["other"]:
            return          # start/continue is gated; the stop above and on_end never are
        direction = 1 if x > 0 else -1
        speed_pct = min(abs(x), 1.0) * 100.0          # jog() takes percent (command protocol 2)
        _exec(lambda r: r.jog(joint_idx, direction, speed_pct), "jog_set")

    ui.separator()
    with ui.row().classes("gap-5 px-3 pt-1 pb-3 flex-wrap"):
        for _ji in range(6):
            with ui.column().classes("items-center gap-1"):
                ui.label(f"J{_ji + 1}").classes("text-xs font-mono text-gray-500")
                ui.joystick(
                    color='#1976D2',
                    size=46,
                    mode='static',
                    lockX=True,
                    on_move=lambda e, j=_ji: _on_joystick_move(e, j),
                    on_end=_joystick_end,
                    throttle=0.05,
                ).style("width:90px; height:90px;")
                ui.label("← speed →").classes("text-xs text-gray-400")

# ─────────────────────────────────────────────────────────────
# TABS
# ─────────────────────────────────────────────────────────────
with ui.tabs().classes("w-full") as tabs:
    tab_joints  = ui.tab('Joints & TCP',   icon='timeline')
    tab_motor   = ui.tab('Motor',          icon='electric_bolt')
    tab_gripper = ui.tab('Gripper',        icon='back_hand')
    tab_diag    = ui.tab('RT Diagnostics', icon='speed')
    tab_power   = ui.tab('Power & CB',     icon='bolt')
    tab_io      = ui.tab('I/O',            icon='device_hub')
    tab_sysinfo = ui.tab('System Info',    icon='computer')
    tab_errors  = ui.tab('Errors',         icon='error')
    tab_activity = ui.tab('Activity Log',  icon='history')
    tab_logs    = ui.tab('Logs',           icon='article')     # the log files + support bundle (gui_logs_tab.py)
    tab_incidents = ui.tab('Recordings',   icon='video_library')  # captures + incident dumps (gui_incidents_tab.py)
    tab_pid     = ui.tab('PID Config',     icon='tune')
    tab_filters = ui.tab('Tune Filters',   icon='graphic_eq')
    tab_preview = ui.tab('Plan Preview',   icon='play_circle')
    tab_plots   = ui.tab('Plots',          icon='show_chart')
    tab_firmware = ui.tab('Firmware',      icon='memory')
    tab_walls   = ui.tab('Walls',          icon='fence')     # TEMPORARY freedrive wall tuner (gui_walls_tab.py)
    tab_blocks  = ui.tab('Blocks',         icon='extension')  # block programming (blockly/); no panel, see below
tabs.set_value('Joints & TCP')

with ui.tab_panels(tabs, value=tab_joints).classes("w-full"):

    # ─── Joints & TCP ───────────────────────────────────────
    with ui.tab_panel(tab_joints):
        ui.label("Joint Data").classes("text-sm font-semibold mt-2 mb-1 text-gray-600 uppercase tracking-wide")
        joint_table = ui.table(columns=joint_columns, rows=[]).classes("w-full")

        ui.separator().classes("my-3")
        ui.label("TCP Pose & Forces").classes("text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
        tcp_table = ui.table(columns=tcp_columns, rows=[]).classes("w-full")

    # ─── Motor ──────────────────────────────────────────────
    with ui.tab_panel(tab_motor):
        with ui.row().classes("gap-4 mb-3 flex-wrap"):
            motor_conn_badges = []
            motor_err_badges  = []
            for _i in range(JOINTS):
                with ui.column().classes("items-center gap-1"):
                    ui.label(f"J{_i+1}").classes("text-xs font-bold text-gray-500")
                    _cb = ui.badge("CONNECTED").props("color=positive")
                    _eb = ui.badge("NO ERRORS").props("color=positive outline")
                    motor_conn_badges.append(_cb)
                    motor_err_badges.append(_eb)

        ui.label("Motor Telemetry").classes("text-sm font-semibold mt-2 mb-1 text-gray-600 uppercase tracking-wide")
        motor_table = ui.table(columns=motor_columns, rows=[]).classes("w-full")

        ui.separator().classes("my-3")
        with ui.row().classes("items-center gap-3 mb-1"):
            ui.label("CAN Commands Sent").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
            sent_mode_badge = ui.badge("—").props("color=grey outline")
        ui.label("Exactly what RTI put on the wire this tick, in motor space (after rad→ticks / "
                 "Nm→mA and int truncation). '—' = channel not commanded, which is different "
                 "from a commanded 0. Arm joints on the dispatch path only — homing does its "
                 "own sends and the gripper has its own path.").classes("text-xs text-gray-400 mb-1")
        sent_table = ui.table(columns=sent_columns, rows=[]).classes("w-full")

        ui.separator().classes("my-3")
        ui.label("Temperature / Voltage / Kt").classes("text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
        info_table = ui.table(columns=info_columns, rows=[]).classes("w-full")

        ui.separator().classes("my-3")
        ui.label("Error Flags").classes("text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
        error_table = ui.table(columns=error_columns, rows=[]).classes("w-full")

    # ─── Gripper ────────────────────────────────────────────
    with ui.tab_panel(tab_gripper):
        with ui.row().classes("items-center gap-3 mb-3"):
            gripper_conn_badge = ui.badge("GRIPPER: UNKNOWN").props("color=grey")
            gripper_mode_badge = ui.badge("MODE: ---").props("color=grey outline")
            gripper_err_badge  = ui.badge("NO ERRORS").props("color=positive outline")
            gripper_err_badge.set_visibility(False)

        # ─── Gripper Command (firmware mode) ────────────────────
        # In firmware mode, RTI sends Send_gripper_data_pack every tick with
        # activate forced to 1. To take the firmware path offline, flip the
        # mode toggle to MOTOR. The action flag below toggles movement vs hold.
        _gripper_cmd_state = {'position': 0, 'speed': 100, 'current': 200,
                              'action': 0,
                              'last_dir': 0}  # +1 closing, -1 opening (set on each move)

        def _send_gripper_cmd(position):
            s = _gripper_cmd_state
            # Snapshot the fields NOW — the worker runs the call later and the shared state
            # dict is mutated by _gr_move/_gr_stop, so a captured dict could send stale values.
            pos, spd, cur, act = int(position), s['speed'], s['current'], s['action']
            _exec(lambda r: r.set_gripper(pos, spd, cur, act), "gripper_set")

        def _gr_move(pos):
            # Open/Close pass 1/254; Goto passes the slider's tracked value.
            # We do NOT write `position` into _gripper_cmd_state — that field is
            # owned by the slider so Goto always reflects the slider, not the
            # last hard-coded button target.
            target = int(pos)
            current = int(shared_data.gripper['gripper_position'][0])
            _gripper_cmd_state['last_dir'] = 1 if target > current else (-1 if target < current else 0)
            _gripper_cmd_state['action']   = 1
            _send_gripper_cmd(target)

        def _gr_idle():
            # RELEASE. action=0 drops the firmware into its "Idle or disabled" branch:
            # velocity setpoint 0, and on a Spectral BLDC board the output stage is
            # switched off as well (SLEEP/RESET LOW), so the jaws go limp and can be
            # moved by hand. A STEPFOC board has no such lines and settles into a
            # velocity-0 hold instead — no active command either way, but only the
            # Spectral one is genuinely draggable. RTI announces this to the firmware
            # and then holds the bus quiet with empty polls.
            _gripper_cmd_state['last_dir'] = 0
            _gripper_cmd_state['action'] = 0
            _send_gripper_cmd(_gripper_cmd_state['position'])

        def _gr_stop():
            # HALT IN PLACE — hold the jaws exactly where they are.
            #
            # The firmware has no graceful stop primitive. Its only stop is the ESTOP
            # bit, and that LATCHES A FAULT (motor_control.cpp sets estop_error = 1 and
            # controller.Error = 1), which would then need clearing before the gripper
            # worked again. Not what a Stop button should do.
            #
            # So stop is done host-side: re-target the position the jaws are at RIGHT
            # NOW, keeping action=1. The firmware is already within tolerance of that
            # target, so it holds there instead of continuing to the old one.
            here = int(shared_data.gripper['gripper_position'][0])
            if not 1 <= here <= 254:
                # No trustworthy reading (an uncalibrated gripper reports 0). Releasing
                # is the safe fallback — commanding 0 would be read as FULLY OPEN and
                # drive the jaws instead of stopping them.
                _gr_idle()
                return
            _gripper_cmd_state['last_dir'] = 0
            _gripper_cmd_state['action'] = 1
            _send_gripper_cmd(here)

        def _on_gripper_ctrl_mode(e):
            mode = 2 if e.value == 'firmware' else 1
            _exec(lambda r: r.set_gripper_ctrl_mode(mode), "gripper_ctrl_mode")

        with ui.card().classes("w-full mb-3"):
            with ui.row().classes("items-center gap-3 flex-wrap px-3 py-2"):
                ui.label("Gripper Command").classes("font-bold text-sm text-gray-500 uppercase tracking-wide")
                ui.label("Mode:").classes("text-sm text-gray-500")
                gripper_ctrl_mode_toggle = ui.toggle(['motor', 'firmware'], value='motor',
                                                    on_change=_on_gripper_ctrl_mode)
                ui.separator().props("vertical")
                ui.button("Open",  icon="open_in_full",  on_click=lambda: _gr_move(1)  ).props("color=primary outline size=sm")
                ui.button("Close", icon="close_fullscreen",on_click=lambda: _gr_move(254)).props("color=primary outline size=sm")
                ui.button("Goto",  icon="play_arrow",    on_click=lambda: _gr_move(_gripper_cmd_state['position'])).props("color=primary outline size=sm")
                ui.button("Stop",  icon="stop",          on_click=_gr_stop      ).props("color=warning outline size=sm") \
                    .tooltip("Halt in place — holds the jaws where they are (still gripping)")
                ui.button("Idle",  icon="power_settings_new", on_click=_gr_idle ).props("color=warning outline size=sm") \
                    .tooltip("Release — no active command. Limp on SSG48, velocity-0 hold on MSG")
                ui.separator().props("vertical")
                ui.button("Calibrate", icon="straighten", on_click=lambda: _exec(lambda r: r.gripper_calibrate(), "gripper_calibrate")).props("color=secondary outline size=sm")

            with ui.row().classes("items-center gap-3 flex-wrap px-3 pb-1"):
                ui.label("Status:").classes("text-sm text-gray-500")
                gripper_status_label = ui.label("---").classes("text-sm font-mono font-bold")

            with ui.row().classes("items-center gap-3 flex-wrap px-3 pb-3"):
                ui.label("Pos:").classes("text-sm text-gray-500")
                _gr_pos_slider = ui.slider(min=0, max=255, value=0).classes("w-40")
                _gr_pos_label  = ui.label("0").classes("text-sm font-mono w-10")
                def _on_gr_pos(e):
                    _gripper_cmd_state['position'] = int(e.args)
                    _gr_pos_label.set_text(str(int(e.args)))
                _gr_pos_slider.on('update:model-value', _on_gr_pos)

                ui.separator().props("vertical")
                ui.label("Spd:").classes("text-sm text-gray-500")
                _gr_spd_slider = ui.slider(min=0, max=255, value=100).classes("w-40")
                _gr_spd_label  = ui.label("100").classes("text-sm font-mono w-10")
                def _on_gr_spd(e):
                    _gripper_cmd_state['speed'] = int(e.args)
                    _gr_spd_label.set_text(str(int(e.args)))
                _gr_spd_slider.on('update:model-value', _on_gr_spd)

                ui.separator().props("vertical")
                ui.label("Cur (mA):").classes("text-sm text-gray-500")
                _gr_cur_slider = ui.slider(min=1, max=1500, value=200).classes("w-40")   # set_gripper: 1 mA up (2026-10-24)
                _gr_cur_label  = ui.label("200").classes("text-sm font-mono w-12")
                def _on_gr_cur(e):
                    _gripper_cmd_state['current'] = int(e.args)
                    _gr_cur_label.set_text(str(int(e.args)))
                _gr_cur_slider.on('update:model-value', _on_gr_cur)


        ui.label("Gripper Status").classes("text-sm font-semibold mt-2 mb-1 text-gray-600 uppercase tracking-wide")
        gripper_table = ui.table(columns=gripper_columns, rows=[]).classes("w-full")

        ui.separator().classes("my-3")
        ui.label("Gripper Motor Health").classes("text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
        with ui.grid(columns=2).classes("gap-x-8 gap-y-1 font-mono text-sm mt-1 mb-2"):
            ui.label("Position (ticks)"); grp_motor_pos_lbl    = ui.label("---")
            ui.label("Speed (ticks/s)"); grp_motor_spd_lbl    = ui.label("---")
            ui.label("Current (mA)");    grp_motor_cur_lbl    = ui.label("---")
            ui.label("Temperature (°C)"); grp_motor_temp_lbl  = ui.label("---")
            ui.label("Voltage");          grp_motor_volt_lbl  = ui.label("---")
            ui.label("Kt (Nm/A)");        grp_motor_kt_lbl    = ui.label("---")
            ui.label("HW Version");       grp_motor_hw_lbl    = ui.label("---")
            ui.label("SW Version");       grp_motor_sw_lbl    = ui.label("---")
            ui.label("Serial");           grp_motor_sn_lbl    = ui.label("---")

        ui.label("Gripper Motor Error Flags").classes("text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
        grp_motor_err_columns = [
            {'name': 'temp',  'label': 'Temp',     'field': 'temp'},
            {'name': 'enc',   'label': 'Encoder',  'field': 'enc'},
            {'name': 'vbus',  'label': 'VBUS',     'field': 'vbus'},
            {'name': 'drv',   'label': 'Driver',   'field': 'drv'},
            {'name': 'vel',   'label': 'Velocity', 'field': 'vel'},
            {'name': 'cur',   'label': 'Current',  'field': 'cur'},
            {'name': 'estop', 'label': 'EStop',    'field': 'estop'},
            {'name': 'wd',    'label': 'Watchdog', 'field': 'wd'},
            # The MOTOR DRIVER's own calibration (commutation/encoder, factory-level) — not the
            # gripper mechanism's stroke calibration shown as 'Gripper Calibrated' in the status table.
            {'name': 'cal',   'label': 'Motor Calibrated','field': 'cal'},
        ]
        grp_motor_err_table = ui.table(columns=grp_motor_err_columns, rows=[]).classes("w-full mb-3")

        ui.separator().classes("my-3")
        ui.label("Gripper Joint (SI units)").classes("text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
        with ui.grid(columns=4).classes("gap-3 font-mono text-sm mt-1"):
            ui.label("").classes("font-bold")
            ui.label("Position (m)").classes("font-bold")
            ui.label("Speed (m/s)").classes("font-bold")
            ui.label("Force (N)").classes("font-bold")

            ui.label("Actual")
            grp_pos_actual_lbl = ui.label("---")
            grp_spd_actual_lbl = ui.label("---")
            grp_frc_actual_lbl = ui.label("---")

            ui.label("Target")
            grp_pos_target_lbl = ui.label("---")
            grp_spd_target_lbl = ui.label("---")
            grp_frc_target_lbl = ui.label("---")

            ui.label("Commanded")
            grp_pos_cmd_lbl = ui.label("---")
            grp_spd_cmd_lbl = ui.label("---")
            grp_frc_cmd_lbl = ui.label("---")

        # The jaws where the collision check puts them: the CAD, live from the gripper's report (gui_jaw_view.py)
        try:
            from gui_jaw_view import add_jaw_view
            add_jaw_view(shared_data, _xml_cfg, int(_sys_cfg["joint_num"]))
        except Exception:
            logger.exception("jaw view unavailable")

    # ─── RT Diagnostics ─────────────────────────────────────
    with ui.tab_panel(tab_diag):
        with ui.card().classes("w-full mb-3"):
            ui.label("Loop Health").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
            with ui.row().classes("gap-6 font-mono text-sm flex-wrap"):
                ts_label                = ui.label("Timestamp: ---")
                tick_label              = ui.label("Tick: ---")
                gripper_connected_label = ui.label("Gripper: ---")
                hall_index_label        = ui.label("Hall: ---")
                additional_pin2_label   = ui.label("Pin2: ---")
                edge_detect_label       = ui.label("Edge: ---")

            with ui.row().classes("gap-6 font-mono text-sm flex-wrap mt-1"):
                safety_violation_label = ui.label("Safety violation: NO")
                error_active_label     = ui.label("Error active: 0")
                can_error_nodes_label  = ui.label("CAN nodes in error: 0")
                can_old_data_label     = ui.label("CAN old-data flags: 0")

            ui.separator().classes("my-2")
            ui.label("Timing").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
            # p95/p99 are a rolling window over the last ~500 ticks (always fresh,
            # never need a manual reset). "Peak" is since-start and can be zeroed
            # with the button below — see handle_reset_loop_stats.
            with ui.grid(columns=6).classes("gap-x-6 gap-y-1 font-mono text-sm mt-1"):
                ui.label("").classes("font-bold")
                ui.label("Current").classes("font-bold")
                ui.label("EMA avg").classes("font-bold")
                ui.label("p95").classes("font-bold")
                ui.label("p99").classes("font-bold")
                ui.label("Peak").classes("font-bold")

                ui.label("Period (ms)")
                period_cur_label  = ui.label("—")
                period_avg_label  = ui.label("—")
                period_p95_label  = ui.label("—")
                period_p99_label  = ui.label("—")
                period_peak_label = ui.label("—")

                # Informational mirror of the Period row above — the ms figures stay the
                # authority. Only Current/EMA are inverted: 1/period flips a percentile's
                # tail (p99 period is the SLOW tail, so its Hz is the 1st percentile), and
                # 1/peak-period is the LOWEST rate, not a peak. Showing those under p95/p99/
                # Peak headers would read backwards, so they stay blank — same choice the
                # GUI Loop panel below already makes.
                ui.label("Rate (Hz)")
                rate_cur_label    = ui.label("—")
                rate_avg_label    = ui.label("—")
                ui.label("")
                ui.label("")
                ui.label("")

                ui.label("Exec (ms)")
                exec_cur_label    = ui.label("—")
                exec_avg_label    = ui.label("—")
                exec_p95_label    = ui.label("—")
                exec_p99_label    = ui.label("—")
                exec_peak_label   = ui.label("—")

                ui.label("Jitter (µs)")
                jitter_cur_label  = ui.label("—")
                jitter_avg_label  = ui.label("—")
                ui.label("")
                ui.label("")
                jitter_peak_label = ui.label("—")

                ui.label("Overruns")
                overrun_label     = ui.label("0")
                ui.label("")
                ui.label("")
                ui.label("")
                ui.label("")

                # How far behind the CAN bus this loop is running. Healthy = a few ms; 100 ms+
                # means RTI is controlling on stale measurements (no PID tuning fixes that).
                with ui.row().classes("items-center gap-1 no-wrap"):
                    ui.label("CAN age (ms)")
                    with ui.icon("help_outline").classes("text-gray-400 cursor-help text-sm"):
                        # Plain labels rather than ui.html/ui.markdown: their signatures have
                        # changed across NiceGUI versions (3.3 made ui.html's `sanitize` a
                        # required kwarg), and a tooltip is not worth a version dependency.
                        with ui.tooltip().classes("max-w-sm"), ui.column().classes("gap-1 py-1"):
                            ui.label("How long the CAN frames waited for RTI's read.") \
                                .classes("font-semibold")
                            ui.label(
                                "With the read first (the default since 2026-10-12) the commands leave early, the "
                                "replies come back early and wait longer for the next read: ~0.3 ms more here than "
                                "in the old order is MARGIN for the last joints, not older data. How old the data "
                                "is when it reaches the motors is the Loop delay row (3.8 vs 7.8 ms).")
                            ui.label(
                                "Each tick RTI drains the CAN socket. Every frame carries a "
                                "kernel arrival timestamp, so this is how long the oldest frame "
                                "sat in the buffer before RTI read it — how far behind the bus "
                                "the loop is running.")
                            ui.label("Current = worst frame this tick · min = best this tick · "
                                     "Peak = worst since reset").classes("italic")
                            ui.label("HEALTHY: under one tick (<4 ms).").classes("font-semibold")
                            ui.label(
                                "Draining once per tick means a frame arriving just after a "
                                "drain waits for the next one, so 0–4 ms is the floor, not a "
                                "fault. A spread between min and Current is normal and good.")
                            ui.label("BAD: min ≈ Current, both tens of ms.").classes("font-semibold")
                            ui.label(
                                "That is uniform staleness = a receive backlog. RTI is acting "
                                "on the past: commanded-vs-actual looks phase-shifted, errors "
                                "latch on stale state, and no amount of PID tuning fixes it.")
                            ui.label(
                                "Reads 0 on a silent bus (no frames to age) — CAN_LOST and "
                                "CAN_STALE cover that case instead.").classes("italic")
                # 6 cells to match the grid: name | Current | (min in the EMA col) | p95 | p99 | Peak
                can_age_label      = ui.label("—")
                can_age_min_label  = ui.label("—")
                ui.label("")
                ui.label("")
                can_age_peak_label = ui.label("—")

            # The control loop's delay per joint, ON REQUEST (2026-10-25): the button runs tools/loop_delay.py, which
            # records the bus with candump and RTI's read times from its trace — RTI does nothing for it. Until then RTI
            # measured it live (utility/loop_delay_meter.py) and its socket-filter switches stalled the tick now and then.
            _LOOP_DELAY_TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools", "loop_delay.py")
            _LOOP_DELAY_S = 3

            def _loop_delay_text(res):
                if not res.get("ok"):
                    return f"cannot measure: {res.get('error', 'no answer')}"
                parts = [f"{name} {s['med']:.1f}/{s['max']:.1f}" if s.get("n") else f"{name} —"
                         for name, s in res["joints"].items()]
                return ("   ".join(parts) + f"   (median / max, {res['ticks']} ticks, "
                        f"{res['measured_utc'][11:19]} UTC, order: {res['order']})")

            async def _measure_loop_delay():
                loop_delay_button.disable()
                loop_delay_label.text = f"measuring for {_LOOP_DELAY_S} s ..."
                try:
                    proc = await asyncio.create_subprocess_exec(
                        sys.executable, _LOOP_DELAY_TOOL, "--seconds", str(_LOOP_DELAY_S), "--json",
                        stdin=asyncio.subprocess.DEVNULL,
                        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                    out, err = await asyncio.wait_for(proc.communicate(), timeout=_LOOP_DELAY_S + 20)
                    text = out.decode(errors="replace").strip()
                    res = json.loads(text.splitlines()[-1]) if text else {
                        "ok": False, "error": err.decode(errors="replace").strip()[-200:] or "no output"}
                except Exception as exc:
                    res = {"ok": False, "error": str(exc)}
                finally:
                    loop_delay_button.enable()
                loop_delay_label.text = _loop_delay_text(res)

            with ui.row().classes("items-center gap-1 no-wrap mt-2 font-mono text-sm"):
                ui.label("Loop delay (ms)")
                with ui.icon("help_outline").classes("text-gray-400 cursor-help text-sm"):
                    with ui.tooltip().classes("max-w-md"), ui.column().classes("gap-1 py-1"):
                        ui.label("How old a joint's measurement is when the command computed from it reaches "
                                 "the drive.").classes("font-semibold")
                        ui.label(
                            "Per joint: from the moment the joint's CAN reply (its position, speed, current) "
                            "arrives to the moment the CAN command RTI computed from that reply leaves. Measured "
                            f"when you press Measure, never on its own: for {_LOOP_DELAY_S} s a separate process "
                            "records the CAN bus (candump: the replies and our commands with the kernel's times) "
                            "and RTI's CAN read times from its trace; median / max over those ticks. RTI does "
                            "nothing for it (tools/loop_delay.py; the same from a terminal).")
                        ui.label("Until 2026-10-25 RTI measured this live and switched a CAN socket filter "
                                 "inside its tick; that call waits for the kernel's network lock, so a network "
                                 "change elsewhere (a Wi-Fi reconnect) stalled the loop 8-44 ms now and then "
                                 "(LOOP_CRITICAL).").classes("italic")
                        ui.label("Normal tick order: ~7.8 ms. The reply arrives while RTI sleeps, is read in "
                                 "the next tick, used by the tick after that, and that command leaves only when "
                                 "RTI sleeps again — about two ticks.")
                        ui.label("Option B (read first, send early — the default since 2026-10-12, system.xml "
                                 "<can_read_first>): ~3.8 ms, about one tick: the reply is read at the start of "
                                 "the next tick and the command computed from it leaves right away.")
                        ui.label(
                            "What it matters for: loops OUR software closes on the measured state — the "
                            "haptic walls in freedrive, a PC's own force / impedance control in hard RTI. "
                            "The drives close their position loops themselves, so path tracking does not "
                            "depend on it (measured: 66 / 68 unchanged with option B).").classes("italic")
                        ui.label("A max one tick (~4 ms) above the median = a reply that came after RTI's read "
                                 "now and then; often = load on the Pi.")
                loop_delay_button = ui.button(f"Measure ({_LOOP_DELAY_S} s)", icon="timer",
                                              on_click=_measure_loop_delay).props("color=grey outline size=sm")
                loop_delay_order_label = ui.label("—").classes("text-gray-500 ml-2")
            loop_delay_label = ui.label("not measured — press Measure").classes("font-mono text-sm")

            with ui.row().classes("mt-2 gap-2"):
                ui.button("Reset Loop Stats (peaks)", icon="restart_alt",
                          on_click=handle_reset_loop_stats).props("color=grey outline size=sm")
                ui.button("Reload Filters", icon="tune",
                          on_click=handle_reload_filters).props("color=grey outline size=sm")

        # ── RTI mode (external PC streaming) ───────────────────
        # Reads rti_substate / control_success_rate / rti_consecutive_misses, all written
        # every tick by motion/rti_handler.py. Inactive most of the time — the whole card
        # dims to grey when no PC is paired so it is not permanent noise.
        with ui.card().classes("w-full mt-3"):
            with ui.row().classes("items-center gap-3 mb-2"):
                ui.label("RTI Mode (external control)") \
                    .classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
                rti_substate_badge = ui.badge("NO PC PAIRED").props("color=grey outline")
                with ui.icon("help_outline").classes("text-gray-400 cursor-help text-sm"):
                    with ui.tooltip().classes("max-w-sm"), ui.column().classes("gap-1 py-1"):
                        ui.label("Live health of the UDP link to the controlling PC.") \
                            .classes("font-semibold")
                        ui.label("Two numbers, deliberately measuring different things:")
                        ui.label("Misses = ticks since the last command. Reacts on the NEXT "
                                 "tick. This is what actually stops the robot when it exceeds "
                                 "the budget. Watch this one.").classes("font-semibold")
                        ui.label(
                            f"Success = fraction of the last {SUCCESS_WINDOW_TICKS} ticks that "
                            "carried a command. A deliberately damped gauge — a shorter window "
                            "could not carry its own thresholds (at 25 samples, two dropped "
                            "packets would read as 'unreliable'). It reports; it never stops "
                            "anything.")
                        ui.label("Only meaningful in robot-clocked mode.").classes("italic")
                        ui.label(
                            "In PC-clocked mode the PC sends at its own rate, so a healthy "
                            "100 Hz stream against a 250 Hz loop reads ~40% by design. Read "
                            "Misses instead.").classes("italic")
            with ui.row().classes("gap-6 font-mono text-sm flex-wrap"):
                rti_success_label      = ui.label("Success: —")
                rti_misses_label       = ui.label("Misses: —")
                rti_worst_misses_label = ui.label("Worst run: —")
            with ui.row().classes("gap-6 font-mono text-sm flex-wrap mt-1"):
                rti_bw_tx_label    = ui.label("TX: —")
                rti_bw_rx_label    = ui.label("RX: —")
                rti_bw_total_label = ui.label("Total: —")
                rti_discard_label  = ui.label("Discarded: —")
                with ui.icon("help_outline").classes("text-gray-400 cursor-help text-sm"):
                    with ui.tooltip().classes("max-w-sm"), ui.column().classes("gap-1 py-1"):
                        ui.label("UDP link bandwidth, averaged over 1 second.") \
                            .classes("font-semibold")
                        ui.label("TX = robot → PC state packets. Scales directly with the "
                                 "active data recipe — switching from 'standard' to 'full' "
                                 "multiplies it several times over. This is the number to "
                                 "check before putting RTI on WiFi.")
                        ui.label("RX = PC → robot commands. Fixed 101 bytes per packet.")
                        ui.label("Discarded = command packets thrown away because a newer "
                                 "one had already arrived in the same tick.").classes("font-semibold")
                        ui.label(
                            "Only the newest setpoint per tick can be applied. A PC sending "
                            "faster than the loop ticks loses the rest — and NOTHING else "
                            "shows it: success rate still reads 100% (a command did arrive) "
                            "and misses still read 0. A 500 Hz sender against this 250 Hz "
                            "loop reads ~50% here and perfect everywhere else.")
                        ui.label("Nonzero in robot-clocked mode means the PC is "
                                 "double-sending.").classes("italic")

        with ui.card().classes("w-full"):
            ui.label("CAN Node Status").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
            rti_table = ui.table(columns=rti_columns, rows=[]).classes("w-full")
            ui.separator().classes("my-2")
            received_ids_label       = ui.label("Received IDs: []").classes("font-mono text-sm")
            latest_commands_id_label = ui.label("Latest Commands: []").classes("font-mono text-sm")

        with ui.card().classes("w-full mt-3"):
            ui.label("GUI Loop").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
            with ui.grid(columns=4).classes("gap-x-6 gap-y-1 font-mono text-sm mt-1"):
                ui.label("").classes("font-bold")
                ui.label("Current").classes("font-bold")
                ui.label("EMA avg").classes("font-bold")
                ui.label("Peak").classes("font-bold")

                ui.label("Period (ms)")
                gui_period_cur_label  = ui.label("—")
                gui_period_avg_label  = ui.label("—")
                gui_period_peak_label = ui.label("—")

                ui.label("Exec (ms)")
                gui_exec_cur_label    = ui.label("—")
                gui_exec_avg_label    = ui.label("—")
                gui_exec_peak_label   = ui.label("—")

                ui.label("Rate (Hz)")
                gui_fps_cur_label     = ui.label("—")
                gui_fps_avg_label     = ui.label("—")
                ui.label("")

    # ─── Power & Control Board ──────────────────────────────
    with ui.tab_panel(tab_power):
        with ui.row().classes("gap-4 items-start flex-wrap"):
            with ui.card().classes("flex-1"):
                ui.label("Electrical").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                    ui.label("VBUS 1");    cb_vbus1_label   = ui.label("--- mV")
                    ui.label("VBUS 2");    cb_vbus2_label   = ui.label("--- mV")
                    ui.label("Current");   cb_current_label = ui.label("--- mA")
                    ui.label("Power");     cb_power_label   = ui.label("--- W")

            with ui.card().classes("flex-1"):
                ui.label("States").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                    ui.label("Relay");        cb_relay_label     = ui.label("---")
                    ui.label("Precharge");    cb_precharge_label = ui.label("---")
                    ui.label("Brake Res.");   cb_brake_label     = ui.label("---")
                    ui.label("CB Error");     cb_error_label     = ui.label("---")

            with ui.card().classes("flex-1"):
                ui.label("Versions").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                    ui.label("HW Version");  cb_hw_label    = ui.label("---")
                    ui.label("SW Version");  cb_sw_label    = ui.label("---")
                    ui.label("Batch Date");  cb_batch_label = ui.label("---")
                    ui.label("Serial No.");  cb_sn_label    = ui.label("---")
                    # Declared in config/system.xml (the board does not report it): what the E-stop policy assumes.
                    from hardware.control_box import describe as _describe_pcb
                    ui.label("PCB (config)"); ui.label(f"v{_sys_cfg.get('pcb_version', 1)}").tooltip(_describe_pcb(_sys_cfg.get('pcb_version', 1)))

    # ─── I/O ────────────────────────────────────────────────
    with ui.tab_panel(tab_io):
        with ui.row().classes("gap-4 items-start flex-wrap"):

            with ui.card().classes("flex-1 min-w-48"):
                ui.label("Inputs").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-4 gap-y-2 font-mono text-sm items-center"):
                    ui.label("Isolated In 1"); io_in1_badge  = ui.badge("LOW").props("color=grey outline")
                    ui.label("Isolated In 2"); io_in2_badge  = ui.badge("LOW").props("color=grey outline")
                    ui.label("Isolated In 3"); io_in3_badge  = ui.badge("LOW").props("color=grey outline")
                    ui.label("E-Stop 1");      io_estop1_badge = ui.badge("OK").props("color=positive outline")
                    ui.label("E-Stop 2");      io_estop2_badge = ui.badge("OK").props("color=positive outline")

            with ui.card().classes("flex-1 min-w-48"):
                ui.label("Outputs").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-4 gap-y-2 font-mono text-sm items-center"):
                    ui.label("Isolated Out 1"); io_out1_badge = ui.badge("LOW").props("color=grey outline")
                    ui.label("Isolated Out 2"); io_out2_badge = ui.badge("LOW").props("color=grey outline")
                    ui.label("Isolated Out 3"); io_out3_badge = ui.badge("LOW").props("color=grey outline")

            with ui.card().classes("flex-1 min-w-48"):
                ui.label("General GPIO").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-4 gap-y-2 font-mono text-sm items-center"):
                    ui.label("GPIO 1"); io_gpio1_badge = ui.badge("LOW").props("color=grey outline")
                    ui.label("GPIO 2"); io_gpio2_badge = ui.badge("LOW").props("color=grey outline")
                    ui.label("GPIO 3"); io_gpio3_badge = ui.badge("LOW").props("color=grey outline")
                    ui.label("GPIO 4"); io_gpio4_badge = ui.badge("LOW").props("color=grey outline")

            with ui.card().classes("flex-1 min-w-48"):
                ui.label("Buttons & LEDs").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-4 gap-y-2 font-mono text-sm items-center"):
                    ui.label("Button 1"); io_btn1_badge = ui.badge("OFF").props("color=grey outline")
                    ui.label("Button 2"); io_btn2_badge = ui.badge("OFF").props("color=grey outline")
                    ui.label("LED 1");    io_led1_badge = ui.badge("OFF").props("color=grey outline")
                    ui.label("LED 2");    io_led2_badge = ui.badge("OFF").props("color=grey outline")

            # The Hall sensor check (gui_hall_check.py, robotics/hall_check.py, 2026-10-13)
            refresh_hall_card = None
            try:
                from gui_hall_check import add_hall_check_card
                from robotics.hall_check import hall_joint_index
                refresh_hall_card = add_hall_check_card(
                    shared_data, _exec_sender.submit,
                    hall_joint_index(_xml_cfg["has_hall_sensor"], int(_sys_cfg["joint_num"])))
            except Exception:
                logger.exception("Hall sensor check card unavailable")

    # ─── System Info ────────────────────────────────────────
    with ui.tab_panel(tab_sysinfo):
        # The health summary first (utility/health.py, gui_health_card.py, 2026-10-15): one level per component
        try:
            from gui_health_card import add_health_card
            from utility.health import assess as _assess_health
            add_health_card(assess_fn=lambda: _assess_health(shared_data, int(_sys_cfg["joint_num"]),
                                                             float(_xml_cfg.get('delta_t') or 0.004),
                                                             has_gripper=_gui_has_can_gripper()),
                            is_visible=lambda: _active_tab_name(tabs.value) == 'System Info')
        except Exception:
            logger.exception("health card unavailable")
        with ui.row().classes("gap-4 items-start flex-wrap mb-3"):
            with ui.card().classes("flex-1 min-w-64"):
                ui.label("Platform").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                    ui.label("Model");        sysinfo_pi_model_lbl   = ui.label("---")
                    ui.label("Release");      sysinfo_release_lbl    = ui.label("---")
                    ui.label("Kernel");       sysinfo_kernel_lbl     = ui.label("---")
                    ui.label("Distro");       sysinfo_distro_lbl     = ui.label("---")
                    ui.label("Version");      sysinfo_distro_ver_lbl = ui.label("---")

            with ui.card().classes("flex-1 min-w-48"):
                ui.label("CPU").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                    ui.label("Processor");    sysinfo_cpu_proc_lbl = ui.label("---")
                    ui.label("Architecture"); sysinfo_cpu_arch_lbl = ui.label("---")
                    ui.label("Core Count");   sysinfo_cpu_cnt_lbl  = ui.label("---")

            with ui.card().classes("flex-1 min-w-48"):
                ui.label("Memory").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                    ui.label("Total");        sysinfo_mem_size_lbl  = ui.label("---")
                    ui.label("Used");         sysinfo_mem_used_lbl  = ui.label("---")
                    ui.label("Usage");        sysinfo_mem_usage_lbl = ui.label("---")

            # The zone times are shown in, and whether the robot's clock is right (gui_time_card.py, 2026-10-09)
            try:
                from gui_time_card import add_time_card
                add_time_card(str(SYSTEM_XML))
            except Exception:
                logger.exception("time card unavailable")

            with ui.card().classes("flex-1 min-w-48"):
                ui.label("Disk").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                    ui.label("Device");       sysinfo_disk_device_lbl = ui.label("---")
                    ui.label("Total");        sysinfo_disk_size_lbl   = ui.label("---")
                    ui.label("Used");         sysinfo_disk_used_lbl   = ui.label("---")
                    ui.label("Usage");        sysinfo_disk_usage_lbl  = ui.label("---")

        with ui.card().classes("w-full"):
            ui.label("Control Board Strings").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
            with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                ui.label("CB Command");       sysinfo_cb_cmd_lbl  = ui.label("---")
                ui.label("CB Status");        sysinfo_cb_stat_lbl = ui.label("---")
                ui.label("CB Current Error"); sysinfo_cb_err_lbl  = ui.label("---")

        # ─── Processes, threads and the core plan (gui_processes.py, 2026-10-13) ──────
        # Reads LIVE kernel state; refreshed at 1 Hz only while this tab is active (refresh_processes()).
        refresh_process_card = add_processes_card()

        # Which arm homing recognised, the learned arms, rename / forget / accept (gui_homing_arms.py, 2026-10-09)
        try:
            from gui_homing_arms import add_homing_arms_card
            add_homing_arms_card(shared_data, _xml_cfg, int(_sys_cfg["joint_num"]))
        except Exception:
            logger.exception("homing arms card unavailable")

        # Every part's version: the runtime, the OS, the description package, the box, each drive
        # (utility/versions.py, gui_versions_card.py, 2026-10-21). Built in a try: a failure must not break the page.
        try:
            from gui_versions_card import add_versions_card
            from utility.versions import report as _versions_report
            # The visibility check is guarded: _active_tab_name is defined further down this script, and a card module
            # cached by a running dashboard may still call it while the page is being built.
            add_versions_card(lambda: _versions_report(shared_data, int(_sys_cfg["joint_num"]), _gui_has_can_gripper(),
                                                       _sys_cfg.get('pcb_version', 1)),
                              is_visible=lambda: ('_active_tab_name' in globals()
                                                  and _active_tab_name(tabs.value) == 'System Info'))
        except Exception:
            logger.exception("versions card unavailable")

    # ─── Errors ───────────────────────────────────────────────
    with ui.tab_panel(tab_errors):
        with ui.card().classes("w-full"):
            ui.label("Active Errors").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
            errors_container = ui.column().classes("w-full gap-1 font-mono text-sm")
            with errors_container:
                errors_no_errors_label = ui.label("No active errors.").classes("text-positive")

    # ─── Activity Log (gui_activity_tab.py, 2026-10-15) ─────────
    # The event history (logs/activity_rti.log + logs/activity_exec.log), merged by time; everything shown, filters
    # narrow it. Separate from the Errors tab, which is the live error latch. docs/logging.md "The activity log".
    with ui.tab_panel(tab_activity):
        with ui.card().classes("w-full"):
            ui.label("Activity Log").classes("text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
            try:
                from gui_activity_tab import add_activity_tab
                add_activity_tab(is_visible=lambda: _active_tab_name(tabs.value) == 'Activity Log',
                                 sources=_ACTIVITY_LOG_SOURCES, fmt_time=display_time.fmt_precise)
            except Exception:
                logger.exception("Activity Log tab unavailable")
                ui.label("Activity Log tab unavailable — see logs/gui.log").classes("text-negative")

    # ─── Logs (the log files, the support bundle; gui_logs_tab.py, 2026-10-15) ───
    with ui.tab_panel(tab_logs):
        try:
            from gui_logs_tab import add_logs_tab
            add_logs_tab(is_visible=lambda: _active_tab_name(tabs.value) == 'Logs',
                         robot_fn=lambda: _robot_snapshot(),        # defined further down
                         fmt_time=display_time.fmt_precise,      # one time format in every tab
                         zone_name=display_time.zone_name,
                         # runtime log levels (2026-10-15): RTI + the executor through set_log_level
                         level_fn=lambda name, level: _exec_sender.submit(
                             lambda r, n=name, lv=level: r.set_log_level(n, lv), label="set_log_level"))
        except Exception:
            logger.exception("Logs tab unavailable")
            ui.label("Logs tab unavailable — see logs/gui.log").classes("text-negative")

    # ─── Recordings: sub-tabs Captures (on request) and Incidents (around a fault); gui_incidents_tab.py ───
    with ui.tab_panel(tab_incidents):
        try:
            from gui_incidents_tab import add_incidents_tab
            add_incidents_tab(is_visible=lambda: _active_tab_name(tabs.value) == 'Recordings',
                              fmt_time=display_time.fmt_precise,
                              capture_fn=lambda seconds, name: _exec_sender.submit(
                                  lambda r, s=seconds, n=name: r.capture_telemetry(s, n), label="capture_telemetry"),
                              stop_fn=lambda: _exec_sender.submit(lambda r: r.stop_capture(), label="stop_capture"),
                              joint_num=int(_sys_cfg["joint_num"]))
        except Exception:
            logger.exception("Recordings tab unavailable")
            ui.label("Recordings tab unavailable — see logs/gui.log").classes("text-negative")

    # ─── PID Config ───────────────────────────────────────────
    with ui.tab_panel(tab_pid):
        with ui.card().classes("w-full mb-3"):
            ui.label("Joint Motors").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
            ui.label("Updated at startup and on motor reconnect.").classes("text-xs text-gray-400 mb-2")
            pid_table = ui.table(columns=pid_columns, rows=[]).classes("w-full")

        with ui.card().classes("w-full"):
            ui.label("Gripper").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
            ui.label("Updated at startup and on gripper reconnect.").classes("text-xs text-gray-400 mb-2")
            gripper_pid_table = ui.table(columns=pid_columns, rows=[]).classes("w-full")

        # ─── Interactive PID Tuner ────────────────────────────
        import types as _types

        _GAIN_DEFS = [
            # (key,       label,            shm_key,               min,       max,     step,    fmt)
            ("KPP",    "KPP (Pos P)",     "cfg_KPP",             0.0,   1000.0,    1.0,   "%.1f"),
            ("KPV",    "KPV (Vel P)",     "cfg_KPV",             0.0,      2.0,  0.001,   "%.4f"),
            # 6 decimals: KIV is an integral gain and useful values sit well below the old
            # 0.001 step, which quantised anything finer away (the bound slider shares this
            # step). The Motor tab already displayed KIV to 6 dp, so this also makes the
            # tuner and the table agree.
            ("KIV",    "KIV (Vel I)",     "cfg_KIV",             0.0,      2.0, 0.000001, "%.6f"),
            # Same treatment as KIV above, and for the same reason: the shipped values are
            # 0.012, which step=1.0 could not reach at all and "%.1f" rendered as "0.0" —
            # the field could neither show nor set what the driver was actually running.
            ("KPIQ",   "KPIQ (Cur P)",    "cfg_KPIQ",            0.0,    500.0, 0.000001, "%.6f"),
            ("KIIQ",   "KIIQ (Cur I)",    "cfg_KIIQ",            0.0,    500.0, 0.000001, "%.6f"),
            ("KP",     "KP (Imp P)",      "cfg_KP",              0.0,    500.0,  0.001,   "%.4f"),
            ("KD",     "KD (Imp D)",      "cfg_KD",              0.0,     10.0, 0.0001,   "%.6f"),
            ("Ilim",   "Ilim (mA)",       "cfg_Ilim",            0.0,   2800.0,  100.0,   "%.0f"),
            ("vel_lim","Vel Lim (t/s)",   "cfg_velocity_limit",  0.0, 120000.0, 1000.0,   "%.0f"),
            # 0 = use VBUS (a mode, not a limit) — min must be 0 so a VBUS-configured joint
            # displays honestly. Anything between 0 and 6000 is floored to 6000 on apply.
            ("vlim",   "V Lim (mV, 0=VBUS)", "cfg_voltage_limit", 0.0, 12000.0,  100.0,   "%.0f"),
        ]

        _pts = _types.SimpleNamespace(**{k: 0.0 for k, *_ in _GAIN_DEFS})

        with ui.card().classes("w-full mt-3"):
            ui.label("Tune Joint").classes("text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
            ui.label("Changes are sent live to the motor driver over CAN.").classes("text-xs text-gray-400 mb-3")

            _pid_joint_options = {i: f"J{i+1}" for i in range(JOINTS)}
            _pid_joint_options[JOINTS] = "Gripper"
            _pid_joint_sel = ui.select(
                options=_pid_joint_options,
                value=0,
                label="Joint",
            ).classes("w-40 mb-3")

            with ui.grid(columns=4).classes("gap-x-6 gap-y-4 w-full"):
                for _key, _lbl, _shm, _min, _max, _step, _fmt in _GAIN_DEFS:
                    with ui.column().classes("gap-1"):
                        ui.label(_lbl).classes("text-xs text-gray-500 font-mono")
                        ui.number(
                            min=_min, max=_max, step=_step, format=_fmt,
                        ).classes("w-full font-mono text-sm").bind_value(_pts, _key)
                        ui.slider(
                            min=_min, max=_max, step=_step,
                        ).classes("w-full").bind_value(_pts, _key)

            def _load_pid_from_robot():
                ji = _pid_joint_sel.value
                for _key, _lbl, _shm, *_ in _GAIN_DEFS:
                    if ji == JOINTS:  # gripper
                        shm_key = _shm.replace("cfg_", "cfg_gripper_", 1)
                        setattr(_pts, _key, float(shared_data.motor_config[shm_key][0]))
                    else:
                        setattr(_pts, _key, float(shared_data.motor_config[_shm][ji]))

            ui.timer(0, _load_pid_from_robot, once=True)

            def _load_default_pid():
                ji = _pid_joint_sel.value
                for _key, _lbl, _shm, *_ in _GAIN_DEFS:
                    if ji == JOINTS:  # gripper
                        xml_key = _shm.replace('cfg_', 'gripper_', 1)
                        setattr(_pts, _key, float(_xml_cfg[xml_key]))
                    else:
                        xml_key = _shm.replace('cfg_', 'motor_', 1)
                        setattr(_pts, _key, float(_xml_cfg[xml_key][ji]))

            def _apply_pid_to_joint():
                ji = _pid_joint_sel.value
                gains = {_key: getattr(_pts, _key) for _key, *_ in _GAIN_DEFS}
                _exec(lambda r: r.set_pid_gains(ji, **gains), "set_pid_gains")

            with ui.row().classes("gap-3 mt-4"):
                ui.button("Load from Robot", icon="download", on_click=_load_pid_from_robot).props("color=primary outline size=sm")
                ui.button("Load Default", icon="settings_backup_restore", on_click=_load_default_pid).props("color=secondary outline size=sm")
                ui.button("Apply to Joint", icon="send", on_click=_apply_pid_to_joint).props("color=positive size=sm")

    # ─── Tune Filters ─────────────────────────────────────────
    with ui.tab_panel(tab_filters):
        import xml.etree.ElementTree as _FET

        _FILT_PATH = CONFIG_DIR / 'filters.xml'

        # What each signal feeds, and how many filter POLES it needs. The rule the whole
        # design rests on: a signal needs as many poles as it has DIFFERENTIATIONS.
        _SIG_INFO = {
            'speed':         ('joint_speed_filtered',        'measured on CAN (0 diffs) — 1 pole is plenty'),
            'accel':         ('joint_acceleration_filtered', '1 diff, but its filtered-speed input already supplies the pole — "none" is usually right'),
            'jerk':          ('joint_jerk_filtered',         '2 diffs — NEEDS 2 poles (ema2/butter2/savgol). A single ema cannot fix jerk at any tau'),
            'torque':        ('joint_torque_filtered',       'measured (0 diffs) — 1 pole. (torque_external has its own smoothing: the momentum observer)'),
            'tcp_accel':     ('TCP_acceleration_filtered',   'Cartesian accel; its J̇·q̇ term is quadratic in velocity so it needs its own low-pass'),
            'gripper_speed': ('gripper_joint_speed_filtered', 'measured (0 diffs) — 1 pole'),
        }
        _KIND_INFO = {
            'none':    'passthrough (0 poles)',
            'ema':     '1-pole low-pass. Cheapest. needs tau',
            'ema2':    '2-pole (two ema stages). The jerk fix. needs tau',
            'dema':    'lag-REDUCED ema — NOT 2-pole, will not clean jerk. needs tau',
            'sma':     'moving average, linear phase. needs window',
            'butter2': '2-pole Butterworth, flat passband. needs tau',
            'savgol':  'polynomial fit — built for noisy derivatives. needs window + poly',
            'median':  'non-linear: kills lone spikes instead of smearing. needs window',
        }
        _TAU_KINDS_UI = ('ema', 'ema2', 'dema', 'butter2')
        _WIN_KINDS_UI = ('sma', 'savgol', 'median')

        _filt_ctl = {}          # signal -> dict of controls
        _filt_overrides = {}    # signal -> list of raw <joint> elements (preserved on save)
        _filt_status = None     # set after the UI is built

        def _filt_read_file():
            """Signal-level attrs from config/filters.xml, plus any <joint> overrides.
            Falls back to the built-in defaults when the file is missing/unreadable."""
            vals, overrides = {}, {}
            defaults = default_filter_config()
            root = None
            try:
                if _FILT_PATH.exists():
                    root = _FET.parse(_FILT_PATH).getroot()
            except Exception:
                root = None
            for sig in ALL_SIGNALS:
                el = root.find(sig) if root is not None else None
                if el is None:
                    d = defaults.get(sig, {'kind': 'none'})
                    vals[sig] = {'type': d.get('kind', 'none'), 'tau': d.get('tau'),
                                 'window': d.get('window'), 'poly': d.get('poly')}
                    overrides[sig] = []
                else:
                    vals[sig] = {'type': el.get('type') or 'none',
                                 'tau': el.get('tau'), 'window': el.get('window'),
                                 'poly': el.get('poly')}
                    overrides[sig] = list(el.findall('joint'))
            return vals, overrides

        def _filt_sync_visibility(sig):
            """Show only the params the chosen filter type actually uses."""
            c = _filt_ctl[sig]
            kind = c['type'].value
            c['tau'].set_visibility(kind in _TAU_KINDS_UI)
            c['window'].set_visibility(kind in _WIN_KINDS_UI)
            c['poly'].set_visibility(kind == 'savgol')
            c['hint'].text = _KIND_INFO.get(kind, '')

        def _filt_load_form():
            vals, overrides = _filt_read_file()
            _filt_overrides.clear()
            _filt_overrides.update(overrides)
            for sig, v in vals.items():
                if sig not in _filt_ctl:
                    continue
                c = _filt_ctl[sig]
                c['type'].value = v['type'] if v['type'] in FILTER_KINDS else 'none'
                c['tau'].value = float(v['tau']) if v['tau'] not in (None, '') else 0.0246
                c['window'].value = int(v['window']) if v['window'] not in (None, '') else 5
                c['poly'].value = int(v['poly']) if v['poly'] not in (None, '') else 2
                n = len(overrides.get(sig, []))
                c['ovr'].text = f'{n} per-joint override(s) — preserved on save' if n else ''
                _filt_sync_visibility(sig)
            if _filt_status:
                _filt_status.text = (f'Loaded {_FILT_PATH}' if _FILT_PATH.exists()
                                     else 'No filters.xml — showing built-in defaults')

        # Short reminder re-emitted next to each signal, so a GUI save does not leave the block
        # bare. The block is REGENERATED on save (that is how per-signal edits are applied), so
        # any hand-written inline comment inside <filters> is lost — these put the essential
        # "how many poles does this need" note back. The long tutorial above <filters> is
        # preserved untouched by the save, and remains the real documentation.
        _SIG_XML_HINT = {
            'speed':         'measured (0 diffs): 1 pole is plenty',
            'accel':         'input already carries speed\'s pole; "none" measured BEST',
            'jerk':          '2 diffs: needs 2 POLES (ema2/butter2/savgol) - 1-pole ema cannot fix it at any tau',
            'torque':        'measured (0 diffs): 1 pole. torque_external is smoothed by its observer, not by this',
            'tcp_accel':     'Cartesian accel; its Jdot*qdot term is quadratic in velocity',
            'gripper_speed': 'measured (0 diffs): 1 pole',
        }

        def _filt_build_xml():
            """Emit a new <filters> block from the form, re-emitting any <joint> overrides
            verbatim so editing a signal default never silently drops them."""
            lines = ['<filters>']
            for sig in ALL_SIGNALS:
                c = _filt_ctl[sig]
                kind = c['type'].value
                attrs = [f'type="{kind}"']
                if kind in _TAU_KINDS_UI:
                    attrs.append(f'tau="{float(c["tau"].value):g}"')
                if kind in _WIN_KINDS_UI:
                    attrs.append(f'window="{int(c["window"].value)}"')
                if kind == 'savgol':
                    attrs.append(f'poly="{int(c["poly"].value)}"')
                # "--" is illegal inside an XML comment, so a hint containing it would produce a
                # file that cannot be re-read. Guard rather than trust the table.
                hint = _SIG_XML_HINT.get(sig, '').replace('--', '-')
                kids = _filt_overrides.get(sig, [])
                if kids:
                    lines.append(f'    <{sig} {" ".join(attrs)}>')
                    for k in kids:
                        kattr = ' '.join(f'{a}="{b}"' for a, b in k.attrib.items())
                        lines.append(f'        <joint {kattr}/>')
                    lines.append(f'    </{sig}>' + (f'   <!-- {hint} -->' if hint else ''))
                else:
                    lines.append(f'    <{sig} {" ".join(attrs)}/>'
                                 + (f'   <!-- {hint} -->' if hint else ''))
            lines.append('</filters>')
            return '\n'.join(lines)

        def _filt_apply():
            """Write the <filters> block back (keeping the file's doc header) and ask RTI
            to hot-reload. No restart, no re-home."""
            try:
                new_block = _filt_build_xml()
                old = _FILT_PATH.read_text(encoding='utf-8') if _FILT_PATH.exists() else ''
                # Locate the REAL <filters> element. The file's doc comment quotes "<filters>"
                # in its tutorial, so a plain find() would match inside the comment and shred
                # it. Search backwards from the closing tag instead.
                end = old.rfind('</filters>')
                start = old.rfind('<filters>', 0, end) if end != -1 else -1
                if start != -1 and end != -1:
                    # Preserve everything around the block (XML decl + the big doc comment).
                    text = old[:start] + new_block + old[end + len('</filters>'):]
                else:
                    text = '<?xml version="1.0" encoding="UTF-8"?>\n' + new_block + '\n'
                # Never write something we cannot read back — a corrupt filters.xml would
                # silently drop RTI to default filters on its next reload.
                _FET.fromstring(text)
                _FILT_PATH.write_text(text, encoding='utf-8')
                _exec(lambda r: r.reload_filters(), "reload_filters")   # the executor parses it, RTI applies it
                _filt_status.text = 'Saved and asked RTI to reload — no restart needed'
                ui.notify('Filters saved & reload requested', type='positive')
            except Exception as e:
                _filt_status.text = f'Save FAILED: {e}'
                ui.notify(f'Filter save failed: {e}', type='negative')

        ui.label('Signal Filters').classes('text-lg font-semibold mb-1')
        ui.label('Edit and apply live — RTI rebuilds its filters on the next tick. '
                 'No restart and no re-home. Filter state resets on reload, so expect a '
                 'brief transient.').classes('text-sm text-gray-500 mb-3')

        with ui.card().classes('w-full mb-3'):
            ui.label('Rule that matters').classes('text-sm font-semibold')
            ui.label('A signal needs as many filter POLES as it has DIFFERENTIATIONS. Each '
                     'derivative amplifies noise; each 1-pole stage removes one octave of it. '
                     'No amount of tau tuning replaces a missing pole — that is why jerk '
                     '(2 differentiations) needs a 2-pole filter and a single ema can never '
                     'clean it.').classes('text-xs text-gray-500')
            with ui.grid(columns=2).classes('gap-x-6 gap-y-0 mt-2'):
                for _k, _d in _KIND_INFO.items():
                    ui.label(_k).classes('text-xs font-mono font-semibold')
                    ui.label(_d).classes('text-xs text-gray-500')

        for _sig in ALL_SIGNALS:
            _field, _why = _SIG_INFO.get(_sig, ('', ''))
            with ui.card().classes('w-full mb-2'):
                with ui.row().classes('items-center gap-3 w-full flex-wrap'):
                    with ui.column().classes('gap-0'):
                        ui.label(_sig).classes('text-sm font-mono font-semibold')
                        ui.label(_field).classes('text-xs text-gray-500')
                    _t = ui.select(list(FILTER_KINDS), value='none', label='type') \
                        .props('dense outlined').classes('w-36') \
                        .on_value_change(lambda _e, s=_sig: _filt_sync_visibility(s))
                    _tau = ui.number(label='tau (s)', value=0.0246, format='%.4f', step=0.005,
                                     min=0.0001).props('dense outlined').classes('w-32')
                    _win = ui.number(label='window (odd)', value=5, step=2, min=3) \
                        .props('dense outlined').classes('w-32')
                    _poly = ui.number(label='poly', value=2, step=1, min=1) \
                        .props('dense outlined').classes('w-24')
                    with ui.column().classes('gap-0'):
                        _hint = ui.label('').classes('text-xs text-gray-500')
                        _ovr = ui.label('').classes('text-xs text-orange-500')
                ui.label(_why).classes('text-xs text-gray-400')
                _filt_ctl[_sig] = {'type': _t, 'tau': _tau, 'window': _win,
                                   'poly': _poly, 'hint': _hint, 'ovr': _ovr}

        with ui.row().classes('items-center gap-3 mt-2'):
            ui.button('Apply & Reload', icon='send', on_click=_filt_apply) \
                .props('color=positive size=sm')
            ui.button('Reload from file', icon='refresh', on_click=_filt_load_form) \
                .props('color=secondary outline size=sm')
            _filt_status = ui.label('').classes('text-xs text-gray-500')

        ui.label('Per-joint overrides are edited in config/filters.xml directly (see the '
                 'tutorial in that file). They are shown above and preserved when you save '
                 'from here.').classes('text-xs text-gray-400 mt-2')
        _filt_load_form()

    # ─── Offline Plan Preview ─────────────────────────────────
    with ui.tab_panel(tab_preview):
        from robotclient.dry_run_preview_ui import add_dry_run_preview

        # Follows the Angles toggle and returns its redraw (_set_angle_unit). NiceGUI 3's script mode runs this FILE
        # again at every page load but keeps the modules it imported as they were first imported, so until the GUI
        # process restarts a page load can meet the Plan Preview from before angle_unit existed: then it is built as
        # before (radians, returns None) instead of the whole page failing on an unexpected keyword.
        import inspect as _inspect
        _preview_kw = ({'angle_unit': lambda: _ANGLE_UNIT[0]}
                       if 'angle_unit' in _inspect.signature(add_dry_run_preview).parameters else {})
        _preview_set_units = add_dry_run_preview(
            is_visible=lambda: _active_tab_name(tabs.value) == 'Plan Preview', **_preview_kw) or (lambda: None)

    # ─── Walls (TEMPORARY freedrive joint-wall tuner, gui_walls_tab.py) ───
    with ui.tab_panel(tab_walls):
        from gui_walls_tab import add_walls_tab
        add_walls_tab(_exec, shared_data, JOINTS, is_visible=lambda: _active_tab_name(tabs.value) == 'Walls',
                      poll_fn=lambda fn, label: _exec_sender.submit(fn, label=label))   # not "activity"

    # ─── Plots ────────────────────────────────────────────────
    with ui.tab_panel(tab_plots):
        from collections import deque as _deque

        # Two independent rates are in play:
        #   - RTI_loop_time (delta_t): how fast loop_tick advances -> the X-AXIS
        #     scale (x is in ticks). seconds -> ticks = seconds / delta_t.
        #   - GUI timer (10 Hz): how often update_tables() samples shared memory
        #     into the buffer -> the BUFFER fill rate.
        # Windows are defined in SECONDS so the labels are honest for any loop
        # time; ticks are derived from the live delta_t read from the robot XML.
        _GUI_FPS    = PLOT_REFRESH_HZ                      # plot timer rate (samples/s)
        _DELTA_T    = _xml_cfg.get('delta_t') or 0.005     # RTI_loop_time (s); guard /0
        _PLOT_WINDOWS_S = [5, 15, 30]                      # seconds
        _PLOT_WINDOWS = {f'{s} s': max(1, round(s / _DELTA_T)) for s in _PLOT_WINDOWS_S}
        # Buffer only needs window_seconds * plot fps samples (+50% margin).
        _PLOT_BUF   = int(max(_PLOT_WINDOWS_S) * _GUI_FPS * 1.5)

        _DEFAULT_WIN_S = 15
        _plot_win     = [_PLOT_WINDOWS[f'{_DEFAULT_WIN_S} s']]  # current window (ticks)
        _plot_paused  = [False]       # freeze on-screen plots (keeps buffering)
        # jerk/etrq are joint-space extras; fext/ftool are the Cartesian external wrench
        # (world and tool frame). etrq/fext/ftool have no commanded/target counterpart, and
        # only jerk has a filtered twin — the push guards on None for the missing ones.
        _PLOT_KEYS    = ('pos', 'spd', 'acc', 'cur', 'jerk', 'etrq', 'fext', 'ftool')
        _plot_ylock   = {k: [False] for k in _PLOT_KEYS}

        # Units: which shared-memory arrays feed the 3 charts. 'motor' = raw
        # encoder/driver values, 'joint' = real joint-space (rad / rad·s / Nm).
        # (segment attr on shared_data, field, y-axis label, chart title).
        _UNIT_META = {
            'motor': {
                'pos': ('motor', 'motor_position', 'ticks',   'Motor Position'),
                'spd': ('motor', 'motor_speed',    'ticks/s', 'Motor Speed'),
                'cur': ('motor', 'motor_current',  'mA',      'Motor Current'),
            },
            'joint': {
                'pos': ('joint', 'joint_position',     'rad',      'Joint Position'),
                'spd': ('joint', 'joint_speed',        'rad/s',    'Joint Speed'),
                'acc': ('joint', 'joint_acceleration', 'rad/s^2',  'Joint Acceleration'),
                'cur': ('joint', 'joint_torque',       'Nm',       'Joint Torque'),
                'jerk': ('joint', 'joint_jerk',        'rad/s^3',  'Joint Jerk'),
                # External joint torque: per-joint (Nm) but stored in the tcp segment.
                'etrq': ('tcp',  'torque_external',    'Nm',       'External Joint Torque (valid while moving)'),
            },
            'tcp': {
                'pos': ('tcp', 'robot_pose', 'm / rad', 'TCP Pose'),
                'spd': ('tcp', 'TCP_speed',  'm/s / rad/s', 'TCP Speed'),
                # Actual acceleration is RTI-owned; commanded/target are GUI-derived.
                'acc': (None, None, 'm/s^2 / rad/s^2', 'TCP Acceleration'),
                # External wrench [Fx,Fy,Fz,Mx,My,Mz] in each frame.
                'fext':  ('tcp', 'force_external',      'N / Nm', 'TCP External Wrench (World)'),
                'ftool': ('tcp', 'force_external_tool', 'N / Nm', 'TCP External Wrench (Tool)'),
            },
        }
        # Fallback (ylabel, title) for a key that is not part of the CURRENT unit — its chart
        # is hidden then, but it still needs sane labels for when the unit switches back.
        _KEY_HOME_LABEL = {
            'pos':   ('rad', 'Joint Position'),
            'spd':   ('rad/s', 'Joint Speed'),
            'acc':   ('rad/s^2', 'Joint Acceleration'),
            'cur':   ('mA', 'Motor Current'),
            'jerk':  ('rad/s^3', 'Joint Jerk'),
            'etrq':  ('Nm', 'External Joint Torque'),
            'fext':  ('N / Nm', 'TCP External Wrench (World)'),
            'ftool': ('N / Nm', 'TCP External Wrench (Tool)'),
        }
        _plot_unit = ['motor']        # current units ('motor' | 'joint')

        # Per-chart, per-joint SELECTION. This gates what gets PUSHED to the
        # browser each frame (load), not just what's visible: unselected joints
        # send an empty series (~0 bytes). Sampling still fills every buffer
        # (cheap), so re-selecting a joint instantly shows its history.
        _plot_sel = {k: [True] * JOINTS for k in _PLOT_KEYS}

        _plot_buf_tick  = _deque(maxlen=_PLOT_BUF)
        _plot_pos     = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_spd     = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_acc     = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_cur     = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_jerk    = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_etrq    = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_fext    = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_ftool   = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_prev_visible  = [False]  # tracks tab visibility (kept for symmetry)

        # Commanded/target buffers - joint-space only. These overlays only make
        # sense in JOINT units (measured is then joint too, same units); in motor
        # units there is no stored commanded/target equivalent.
        _CMD_FIELD = {
            # In MOTOR units the "commanded" overlay is what RTI actually put on the CAN wire
            # (motor_*_sent), which is in the same ticks/ticks-per-s/mA as the measured traces.
            # NaN there means the channel was not commanded that tick — dropped from the plot.
            'motor': {'pos': 'motor_position_sent',
                      'spd': 'motor_speed_sent',
                      'cur': 'motor_current_sent'},
            'joint': {'pos': 'joint_position_commanded',
                      'spd': 'joint_speed_commanded',
                      'acc': 'joint_acceleration_commanded',
                      'cur': 'joint_torque_commanded'},
            'tcp':   {'pos': 'TCP_pose_commanded',
                      'spd': 'TCP_speed_commanded'},
        }
        _TGT_FIELD = {
            'joint': {'pos': 'joint_position_target',
                      'spd': 'joint_speed_target',
                      'acc': 'joint_acceleration_target',
                      'cur': 'joint_torque_target'},
            'tcp':   {'pos': 'TCP_pose_target',
                      'spd': 'TCP_speed_target'},
        }
        # Filtered overlay (dash-dot). Only where a _filtered twin exists — position is
        # never filtered, and motor units have no filtered arrays, so no 'pos'/'motor' here.
        # 'cur' maps to torque (the plot's current/torque chart) since current is not filtered.
        _FILT_FIELD = {
            'joint': {'spd': 'joint_speed_filtered',
                      'acc': 'joint_acceleration_filtered',
                      'cur': 'joint_torque_filtered',
                      'jerk': 'joint_jerk_filtered'},
            'tcp':   {'spd': 'TCP_speed_filtered',
                      'acc': 'TCP_acceleration_filtered'},
        }
        _plot_pos_cmd = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_spd_cmd = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_acc_cmd = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_cur_cmd = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_pos_tgt = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_spd_tgt = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_acc_tgt = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_cur_tgt = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        # Filtered buffers (spd/acc/cur only — no filtered position).
        _plot_spd_filt = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_acc_filt = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_cur_filt = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_jerk_filt = [_deque(maxlen=_PLOT_BUF) for _ in range(JOINTS)]
        _plot_show_actual = [True]     # measured/actual solid traces
        _plot_show_cmd    = [False]    # overlay commanded (dashed) - joint units only
        _plot_show_tgt    = [False]    # overlay target (dotted) - joint units only
        _plot_show_filt   = [False]    # overlay filtered (dash-dot) - joint/tcp only
        _plot_show_events = [True]     # vertical markers for mode/error/E-stop

        _tcp_prev_tick = [None]
        _tcp_prev_valid = [False]
        _tcp_prev_speed = {
            'actual': [0.0] * JOINTS,
            'cmd': [0.0] * JOINTS,
            'tgt': [0.0] * JOINTS,
        }
        _path_mode = ['xy']

        # Event log for vertical markers: (tick, short_label, color). Bounded.
        _plot_events  = _deque(maxlen=200)
        _evt_prev     = {'mode': None, 'err': None, 'estop': None}

        _KEY_DATA = {'pos': _plot_pos, 'spd': _plot_spd, 'acc': _plot_acc, 'cur': _plot_cur,
                     'jerk': _plot_jerk, 'etrq': _plot_etrq,
                     'fext': _plot_fext, 'ftool': _plot_ftool}
        _CMD_DATA = {'pos': _plot_pos_cmd, 'spd': _plot_spd_cmd, 'acc': _plot_acc_cmd, 'cur': _plot_cur_cmd}
        _TGT_DATA = {'pos': _plot_pos_tgt, 'spd': _plot_spd_tgt, 'acc': _plot_acc_tgt, 'cur': _plot_cur_tgt}
        _FILT_DATA = {'spd': _plot_spd_filt, 'acc': _plot_acc_filt, 'cur': _plot_cur_filt,
                      'jerk': _plot_jerk_filt}
        _plot_ylock_btns = {}          # key -> the 'Y: Auto/Lock' button (label updates)

        # The angular charts per unit, for the Angles toggle: in Joint units position / speed / acceleration / jerk
        # of every joint, in TCP units the rotation components (rx, ry, rz) of pose / speed / acceleration. Torque,
        # current, the external wrench, metres and motor ticks never convert.
        _ANGULAR_KEYS = {'joint': ('pos', 'spd', 'acc', 'jerk'), 'tcp': ('pos', 'spd', 'acc')}

        def _plot_ang(key, i, v):
            """Sample v of chart `key`, joint/axis `i`, in the display angle unit (unchanged if not an angle)."""
            unit = _plot_unit[0]
            if key not in _ANGULAR_KEYS.get(unit, ()):
                return v
            return _ang_tcp(v, i) if unit == 'tcp' else _ang(v)

        def _plot_read(key, i):
            """Read one sample for chart `key`, joint `i`, in the current units."""
            seg, field, _, _ = _UNIT_META[_plot_unit[0]][key]
            return round(_plot_ang(key, i, float(getattr(shared_data, seg)[field][i])), 4)

        def _plot_read_cmd(key, i):
            """Read the commanded sample for chart `key`, axis/joint `i`.

            In motor units this is the value actually sent over CAN; NaN (channel not
            commanded that tick) comes back as None so the push drops the point rather
            than feeding NaN to the chart.
            """
            unit = _plot_unit[0]
            if unit == 'motor':
                v = float(shared_data.motor[_CMD_FIELD['motor'][key]][i])
                return None if v != v else round(v, 4)      # v != v is the NaN test
            if unit == 'tcp':
                return round(_plot_ang(key, i, float(shared_data.tcp[_CMD_FIELD['tcp'][key]][i])), 4)
            return round(_plot_ang(key, i, float(shared_data.joint[_CMD_FIELD['joint'][key]][i])), 4)

        def _plot_read_tgt(key, i):
            """Read the target sample for chart `key`, axis/joint `i`."""
            unit = _plot_unit[0]
            if unit == 'tcp':
                return round(_plot_ang(key, i, float(shared_data.tcp[_TGT_FIELD['tcp'][key]][i])), 4)
            return round(_plot_ang(key, i, float(shared_data.joint[_TGT_FIELD['joint'][key]][i])), 4)

        def _plot_read_filt(key, i):
            """Read the filtered sample for chart `key`, axis/joint `i`, or None if the
            current unit/key has no filtered twin (position, or motor units)."""
            field = _FILT_FIELD.get(_plot_unit[0], {}).get(key)
            if field is None:
                return None
            seg = shared_data.tcp if _plot_unit[0] == 'tcp' else shared_data.joint
            return round(_plot_ang(key, i, float(seg[field][i])), 4)

        # Modern, colourblind-friendlier joint palette (tuned to read well on
        # both dark and light backgrounds).
        _JCOLORS = ['#4f9dff', '#39d98a', '#ffd166', '#ff6b6b', '#21d4d4', '#c77dff']

        def _shade(hex_color, amount, lighten):
            """Blend a #rrggbb toward white (lighten) or black by `amount` (0..1).

            Used for the filtered overlay: it keeps each joint's hue (so you still know
            WHICH joint a trace belongs to) while separating it from the raw line, which
            plain same-colour dash-dot did not do well enough. Direction is theme-aware —
            lightening on a light background would wash the line out instead of standing
            it apart.
            """
            r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
            t = 255 if lighten else 0
            r, g, b = (round(c + (t - c) * amount) for c in (r, g, b))
            return f'#{r:02x}{g:02x}{b:02x}'
        _TCP_LABELS = ('X', 'Y', 'Z', 'RX', 'RY', 'RZ')

        def _keys_for_unit(unit=None):
            unit = unit or _plot_unit[0]
            if unit == 'motor':
                return ('pos', 'spd', 'cur')
            if unit == 'tcp':
                return ('pos', 'spd', 'acc', 'fext', 'ftool')
            return ('pos', 'spd', 'acc', 'cur', 'jerk', 'etrq')   # joint units

        def _trace_label(i):
            return _TCP_LABELS[i] if _plot_unit[0] == 'tcp' else f'J{i+1}'

        def _refresh_chip_labels():
            for _chips in _plot_cbs.values():
                for _i, _chip in enumerate(_chips):
                    _chip.text = _trace_label(_i)

        # Two chart themes. The toolbar toggles between them; everything visual
        # is data here (no hardcoded dark) so adding/tweaking a theme is trivial.
        _PLOT_THEMES = {
            'dark':  {'bg': '#11151c', 'text': '#c8d0da', 'grid': '#2a3340',
                      'axis': '#3a4554', 'card': 'background-color:#11151c'},
            'light': {'bg': '#ffffff', 'text': '#4b5563', 'grid': '#eef1f5',
                      'axis': '#cbd5e1', 'card': 'background-color:#ffffff'},
        }
        _plot_theme = ['light']       # current theme key
        _plot_cards = []              # chart card wrappers (tinted on theme switch)

        # Collapse/expand per chart. Collapsing hides a chart's body (chart +
        # chips) down to just its header bar; the still-open charts grow to use
        # the freed vertical space.
        _plot_collapsed = {k: [False] for k in _PLOT_KEYS}
        _chart_cards    = {}          # key -> card wrapper (hidden for joint-only charts in motor mode)
        _chart_bodies   = {}          # key -> body column (hidden when collapsed)
        _chart_els      = {}          # key -> the ui.echart element
        _collapse_btns  = {}          # key -> collapse/expand icon button
        _card_title_lbls = {}         # key -> header title label (theme-tinted)
        _cmd_note_lbls   = {}         # key -> "channel not commanded" note beside the title
        # Chart heights by number of open charts (1 big / 2 medium / 3 default).
        _CHART_H = {0: '256px', 1: '560px', 2: '384px', 3: '256px', 4: '220px',
                    5: '200px', 6: '190px'}

        def _echart_opts(title, ylabel):
            _th = _PLOT_THEMES[_plot_theme[0]]
            return {
                'animation': False,
                'backgroundColor': _th['bg'],
                # Title is rendered in the CARD HEADER (so it stays visible when
                # the chart is collapsed), not inside the chart.
                'tooltip': {
                    'trigger': 'axis',
                    'axisPointer': {'type': 'cross', 'animation': False,
                                    'label': {'precision': 4}},
                },
                # Title lives in the card header now; legend top-centre so it
                # doesn't collide with the toolbox icons in the top-right.
                'legend': {
                    'data': [_trace_label(i) for i in range(JOINTS)],
                    'top': 6, 'left': 'center',
                    'textStyle': {'color': _th['text']},
                    'inactiveColor': _th['axis'],
                },
                'grid': {'left': '8%', 'right': '3%', 'top': 34, 'bottom': 42},
                # Zoom/pan. Active only while paused — the live loop drives
                # xAxis.min/max each frame and overrides it while running.
                #   X inside: drag to PAN, wheel to ZOOM (the natural combo).
                #   X slider: draggable scrollbar at the bottom.
                'dataZoom': [
                    {'type': 'inside', 'xAxisIndex': 0, 'filterMode': 'none',
                     'zoomOnMouseWheel': True, 'moveOnMouseMove': True,
                     'moveOnMouseWheel': False},
                    {'type': 'slider', 'xAxisIndex': 0, 'height': 14, 'bottom': 4,
                     'filterMode': 'none', 'borderColor': _th['axis'],
                     'textStyle': {'color': _th['text']}},
                ],
                # Toolbox: drag a rectangle to box-zoom a region (both axes),
                # plus a one-click reset. The icons sit top-right of the chart.
                'toolbox': {
                    'right': 8, 'top': 2, 'itemSize': 13,
                    'iconStyle': {'borderColor': _th['text']},
                    'feature': {
                        'dataZoom': {'yAxisIndex': 'none',
                                     'title': {'zoom': 'Box zoom', 'back': 'Undo zoom'}},
                        'restore': {'title': 'Reset'},
                    },
                },
                'xAxis': {
                    'type': 'value',
                    'name': 'tick', 'nameLocation': 'end', 'nameGap': 8,
                    'nameTextStyle': {'color': _th['text']},
                    'axisLabel': {'formatter': '{value}', 'interval': 'auto',
                                  'color': _th['text']},
                    'axisLine': {'lineStyle': {'color': _th['axis']}},
                    'splitLine': {'lineStyle': {'color': _th['grid']}},
                    'splitNumber': 10,
                    'min': 0, 'max': _plot_win[0],
                },
                'yAxis': {'type': 'value', 'name': ylabel, 'nameLocation': 'end',
                          'nameGap': 12,   # lift the unit label above the axis top
                          'nameTextStyle': {'color': _th['text'], 'align': 'left'},
                          'axisLabel': {'color': _th['text']},
                          'axisLine': {'lineStyle': {'color': _th['axis']}},
                          'splitLine': {'lineStyle': {'color': _th['grid']}}},
                # Series layout (fixed count so ECharts never re-inits on merge):
                #   [0 .. JOINTS-1]          measured  J1..J6  (solid)
                #   [JOINTS .. 2*JOINTS-1]   commanded J1..J6  (dashed, same colors)
                #   [2*JOINTS .. 3*JOINTS-1] target    J1..J6  (dotted, same colors)
                #   [3*JOINTS .. 4*JOINTS-1] filtered  J1..J6  (dash-dot, same colors)
                #   [4*JOINTS]               invisible carrier that holds event markLines
                'series': [
                    {
                        'name': _trace_label(i),
                        'type': 'line', 'data': [],
                        # symbol hidden normally, but shown as a dot at the hovered
                        # point (synced across all 3 charts via connect()).
                        'smooth': False, 'symbol': 'circle', 'symbolSize': 7,
                        'showSymbol': False,
                        'itemStyle': {'color': _JCOLORS[i]},
                        'lineStyle': {'width': 1.5},
                    }
                    for i in range(JOINTS)
                ] + [
                    {
                        'name': f'{_trace_label(i)} cmd',
                        'type': 'line', 'data': [],
                        'smooth': False, 'symbol': 'circle', 'symbolSize': 6,
                        'showSymbol': False,
                        'itemStyle': {'color': _JCOLORS[i]},
                        'lineStyle': {'width': 1.2, 'type': 'dashed', 'opacity': 0.9},
                    }
                    for i in range(JOINTS)
                ] + [
                    {
                        'name': f'{_trace_label(i)} target',
                        'type': 'line', 'data': [],
                        'smooth': False, 'symbol': 'circle', 'symbolSize': 6,
                        'showSymbol': False,
                        'itemStyle': {'color': _JCOLORS[i]},
                        'lineStyle': {'width': 1.2, 'type': 'dotted', 'opacity': 0.95},
                    }
                    for i in range(JOINTS)
                ] + [
                    {
                        'name': f'{_trace_label(i)} filt',
                        'type': 'line', 'data': [],
                        'smooth': False, 'symbol': 'circle', 'symbolSize': 6,
                        'showSymbol': False,
                        # Shaded variant of the joint colour: same hue (so the joint is still
                        # identifiable) but clearly separated from its raw trace.
                        'itemStyle': {'color': _shade(_JCOLORS[i], 0.45,
                                                      _plot_theme[0] == 'dark')},
                        # dash-dot ("line dot line dot"): dash 9, gap 4, dot 2, gap 4.
                        'lineStyle': {'width': 1.8, 'type': [9, 4, 2, 4], 'opacity': 1.0},
                    }
                    for i in range(JOINTS)
                ] + [
                    {
                        'name': '_events', 'type': 'line', 'data': [],
                        'silent': True,
                        'markLine': {'symbol': 'none', 'data': []},
                    }
                ],
            }

        _PATH_PROJECTIONS = {
            'xy':  ('X', 'Y', 0, 1),
            'xz':  ('X', 'Z', 0, 2),
            'yz':  ('Y', 'Z', 1, 2),
            'iso': ('X-Y', 'Z + 0.5(X+Y)', None, None),
        }

        def _path_project(x, y, z):
            mode = _path_mode[0]
            if mode == 'xy':
                return x, y
            if mode == 'xz':
                return x, z
            if mode == 'yz':
                return y, z
            # Simple isometric-style projection for a compact XYZ view.
            return x - y, z + 0.5 * (x + y)

        def _path_axis_labels():
            xlbl, ylbl, _, _ = _PATH_PROJECTIONS[_path_mode[0]]
            return xlbl, ylbl

        def _path_echart_opts():
            _th = _PLOT_THEMES[_plot_theme[0]]
            _xlbl, _ylbl = _path_axis_labels()
            return {
                'animation': False,
                'backgroundColor': _th['bg'],
                'tooltip': {'trigger': 'axis', 'axisPointer': {'type': 'cross', 'animation': False}},
                'legend': {
                    'data': ['Actual', 'Commanded', 'Target'],
                    'top': 6, 'left': 'center',
                    'textStyle': {'color': _th['text']},
                    'inactiveColor': _th['axis'],
                },
                'grid': {'left': '9%', 'right': '5%', 'top': 38, 'bottom': 52},
                'dataZoom': [
                    {'type': 'inside', 'filterMode': 'none', 'zoomOnMouseWheel': True,
                     'moveOnMouseMove': True, 'moveOnMouseWheel': False},
                    {'type': 'slider', 'height': 14, 'bottom': 8, 'filterMode': 'none',
                     'borderColor': _th['axis'], 'textStyle': {'color': _th['text']}},
                ],
                'toolbox': {
                    'right': 8, 'top': 2, 'itemSize': 13,
                    'iconStyle': {'borderColor': _th['text']},
                    'feature': {
                        'dataZoom': {'yAxisIndex': 'none',
                                     'title': {'zoom': 'Box zoom', 'back': 'Undo zoom'}},
                        'restore': {'title': 'Reset'},
                    },
                },
                'xAxis': {
                    'type': 'value', 'name': f'{_xlbl} (m)', 'scale': True,
                    'nameLocation': 'end', 'nameGap': 8,
                    'nameTextStyle': {'color': _th['text']},
                    'axisLabel': {'color': _th['text']},
                    'axisLine': {'lineStyle': {'color': _th['axis']}},
                    'splitLine': {'lineStyle': {'color': _th['grid']}},
                },
                'yAxis': {
                    'type': 'value', 'name': f'{_ylbl} (m)', 'scale': True,
                    'nameLocation': 'end', 'nameGap': 12,
                    'nameTextStyle': {'color': _th['text']},
                    'axisLabel': {'color': _th['text']},
                    'axisLine': {'lineStyle': {'color': _th['axis']}},
                    'splitLine': {'lineStyle': {'color': _th['grid']}},
                },
                'series': [
                    {'name': 'Actual', 'type': 'line', 'data': [], 'showSymbol': False,
                     'symbol': 'circle', 'symbolSize': 5,
                     'itemStyle': {'color': _JCOLORS[0]},
                     'lineStyle': {'width': 1.8}},
                    {'name': 'Commanded', 'type': 'line', 'data': [], 'showSymbol': False,
                     'symbol': 'circle', 'symbolSize': 5,
                     'itemStyle': {'color': _JCOLORS[3]},
                     'lineStyle': {'width': 1.4, 'type': 'dashed', 'opacity': 0.9}},
                    {'name': 'Target', 'type': 'line', 'data': [], 'showSymbol': False,
                     'symbol': 'circle', 'symbolSize': 5,
                     'itemStyle': {'color': _JCOLORS[1]},
                     'lineStyle': {'width': 1.4, 'type': 'dotted', 'opacity': 0.95}},
                ],
            }

        def _path_data(pos_buffers, win_min):
            x_vals, y_vals, z_vals = [list(pos_buffers[i]) for i in range(3)]
            n = min(len(x_vals), len(y_vals), len(z_vals), len(_plot_buf_tick))
            if n <= 0:
                return []
            ticks = list(_plot_buf_tick)[-n:]
            x_vals, y_vals, z_vals = x_vals[-n:], y_vals[-n:], z_vals[-n:]
            out = []
            for tick, x, y, z in zip(ticks, x_vals, y_vals, z_vals):
                if tick < win_min:
                    continue
                px, py = _path_project(float(x), float(y), float(z))
                out.append([round(px, 5), round(py, 5)])
            return out

        def _path_axis_bounds(series_list):
            xs, ys = [], []
            for series in series_list:
                for point in series:
                    xs.append(point[0]); ys.append(point[1])
            if not xs or not ys:
                return None, None, None, None
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
            span = max(xmax - xmin, ymax - ymin, 0.001)
            pad = span * 0.08
            half = span / 2.0 + pad
            return cx - half, cx + half, cy - half, cy + half

        def _set_path_mode(mode):
            _path_mode[0] = mode
            _path_chart.options.clear()
            _path_chart.options.update(_path_echart_opts())
            _path_chart.update()

        _plot_cbs = {k: [] for k in _PLOT_KEYS}

        def _pause_toggle():
            _plot_paused[0] = not _plot_paused[0]
            _pause_btn.text = 'Resume' if _plot_paused[0] else 'Pause'
            _pause_btn.props(f"color={'positive' if _plot_paused[0] else 'orange'} outline size=sm")
            if _plot_paused[0]:
                # Freeze: hint that the charts are now zoomable/pannable.
                _pause_hint.set_visibility(True)
            else:
                # Resume: clear any zoom so the live loop's window isn't constrained.
                _pause_hint.set_visibility(False)
                for _c in (pos_chart, spd_chart, acc_chart, cur_chart):
                    _c.run_chart_method('dispatchAction',
                                        {'type': 'dataZoom', 'start': 0, 'end': 100})

        def _set_window(label):
            _plot_win[0] = _PLOT_WINDOWS[label]
            # Next frame already redraws the whole window at the new span.

        def _relabel_charts(relayout=True):
            """Re-title charts + Y axes for the current units, and clear buffers
            so old-unit samples don't mix with new-unit ones.

            relayout=False (the Angles toggle, clicked on any tab): the chart sizes stay as they are —
            resizing a chart the browser has not mounted yet wedges the Plots tab (the NOTE under the
            chart cards), and an angle switch changes no chart's visibility."""
            _plot_buf_tick.clear()
            for _d in (_plot_pos, _plot_spd, _plot_acc, _plot_cur,
                       _plot_jerk, _plot_etrq, _plot_fext, _plot_ftool,
                       _plot_pos_cmd, _plot_spd_cmd, _plot_acc_cmd, _plot_cur_cmd,
                       _plot_pos_tgt, _plot_spd_tgt, _plot_acc_tgt, _plot_cur_tgt,
                       _plot_spd_filt, _plot_acc_filt, _plot_cur_filt, _plot_jerk_filt):
                for _q in _d:
                    _q.clear()
            _tcp_prev_valid[0] = False
            _refresh_chip_labels()
            for _key, _chart in _charts.items():
                _meta = _UNIT_META.get(_plot_unit[0], {})
                if _key in _meta:
                    _, _, _ylabel, _title = _meta[_key]
                else:
                    # Not part of the current unit (chart hidden) — use its home labels so it
                    # re-appears correctly titled when the unit switches back.
                    _ylabel, _title = _KEY_HOME_LABEL[_key]
                _ylabel = _ang_label(_ylabel)                 # the Angles toggle
                _card_title_lbls[_key].text = _title
                _chart.options.clear()
                _chart.options.update(_echart_opts(_title, _ylabel))
                _chart.update()
            if relayout:
                _relayout_charts()

        def _set_theme(theme):
            """Switch chart theme (dark/light). Rebuilds each chart's options from
            _echart_opts so all colours update, preserving current titles/labels,
            and tints the surrounding cards to match."""
            _plot_theme[0] = theme
            _th = _PLOT_THEMES[theme]
            for _key, _chart in _charts.items():
                _meta = _UNIT_META.get(_plot_unit[0], {})
                if _key in _meta:
                    _, _, _ylabel, _title = _meta[_key]
                else:
                    # Not part of the current unit (chart hidden) — use its home labels so it
                    # re-appears correctly titled when the unit switches back.
                    _ylabel, _title = _KEY_HOME_LABEL[_key]
                _ylabel = _ang_label(_ylabel)                 # the Angles toggle
                # EChart.options is read-only - mutate the existing dict in place
                # (clear + repopulate) rather than reassigning it.
                _chart.options.clear()
                _chart.options.update(_echart_opts(_title, _ylabel))
                _chart.update()
                _card_title_lbls[_key].style(f"color:{_th['text']}")   # re-tint header
            for _card in _plot_cards:
                _card.style(replace=_th['card'])   # replace, not append
            _path_chart.options.clear()
            _path_chart.options.update(_path_echart_opts())
            _path_chart.update()

        def _set_units(unit):
            _plot_unit[0] = unit
            # Commanded overlay only exists in joint space — disable the switch
            # (and the overlay) in motor units so it can't show stale joint data.
            if unit == 'motor':
                # Target/filtered have no motor-space equivalent, but Commanded does now:
                # it shows what RTI actually sent over CAN (motor_*_sent).
                _plot_show_tgt[0] = False
                _plot_show_filt[0] = False
                _tgt_switch.value = False
                _filt_switch.value = False
            # The overlay means different things per unit, so say which: in motor units it is
            # the literal CAN payload, everywhere else it is the joint/TCP commanded value.
            _cmd_switch.text = 'CAN Sent' if unit == 'motor' else 'Commanded'
            _cmd_switch.set_enabled(True)
            _tgt_switch.set_enabled(unit in ('joint', 'tcp'))
            _filt_switch.set_enabled(unit in ('joint', 'tcp'))
            _relabel_charts()

        with ui.row().classes("items-center gap-4 mb-2"):
            _pause_btn = ui.button('Pause', icon='pause',
                                   on_click=_pause_toggle).props('color=orange outline size=sm')
            ui.separator().props('vertical')
            ui.label('Units:').classes('text-sm text-gray-500')
            ui.toggle({'motor': 'Motor', 'joint': 'Joint', 'tcp': 'TCP'}, value='motor',
                      on_change=lambda e: _set_units(e.value)).props('dense')
            ui.separator().props('vertical')
            ui.label('Window:').classes('text-sm text-gray-500')
            ui.toggle(list(_PLOT_WINDOWS), value=f'{_DEFAULT_WIN_S} s',
                      on_change=lambda e: _set_window(e.value)).props('dense')
            ui.separator().props('vertical')
            ui.label('Theme:').classes('text-sm text-gray-500')
            ui.toggle({'dark': 'Dark', 'light': 'Light'}, value='light',
                      on_change=lambda e: _set_theme(e.value)).props('dense')
            ui.separator().props('vertical')
            ui.switch('Actual', value=True,
                      on_change=lambda e: _plot_show_actual.__setitem__(0, bool(e.value))) \
                .props('dense').tooltip('Show actual/measured solid traces')
            # Commanded/target overlays - only meaningful in Joint/TCP units, so
            # they start disabled (default unit is motor) and _set_units enables them.
            # Label/enabled state must match the DEFAULT unit here: _set_units() only runs on a
            # unit CHANGE, so anything set up wrong at build time stays wrong until the user
            # toggles units and back. This overlay is valid in every unit (motor = the CAN
            # payload, joint/TCP = the commanded value), so it starts enabled.
            _cmd_switch = ui.switch('CAN Sent' if _plot_unit[0] == 'motor' else 'Commanded',
                                    value=False,
                                    on_change=lambda e: _plot_show_cmd.__setitem__(0, bool(e.value))) \
                .props('dense').tooltip('Overlay commanded (dashed). In Motor units this is '
                                        'what was actually sent over CAN (motor_*_sent)')
            _tgt_switch = ui.switch('Target', value=False,
                                    on_change=lambda e: _plot_show_tgt.__setitem__(0, bool(e.value))) \
                .props('dense').tooltip('Overlay target (dotted) - Joint/TCP only')
            _tgt_switch.set_enabled(False)
            _filt_switch = ui.switch('Filtered', value=False,
                                     on_change=lambda e: _plot_show_filt.__setitem__(0, bool(e.value))) \
                .props('dense').tooltip('Overlay filtered signals (dash-dot) - Joint/TCP only')
            _filt_switch.set_enabled(False)
            ui.switch('Events', value=True,
                      on_change=lambda e: _plot_show_events.__setitem__(0, bool(e.value))) \
                .props('dense').tooltip('Vertical markers on mode / error / E-stop changes')
            ui.separator().props('vertical')
            ui.button('Snapshot', icon='photo_camera', on_click=lambda: ui.timer(0, _take_snapshot, once=True)).props('color=primary outline size=sm')
            ui.button('CSV', icon='download', on_click=lambda: ui.timer(0, _export_csv, once=True)).props('color=primary outline size=sm')
            ui.separator().props('vertical')
            # Live plot refresh rate (measured, not the nominal PLOT_REFRESH_HZ).
            plot_fps_label = ui.label('FPS: —').classes('text-sm font-mono text-gray-500')
            _pause_hint = ui.label('⏸ paused — scroll to zoom, drag to pan').classes('text-xs text-orange-500')
            _pause_hint.set_visibility(False)

        def _set_all_joints(key, on):
            for _i, _chip in enumerate(_plot_cbs[key]):
                _plot_sel[key][_i] = on
                _chip.style(replace=_chip_style(_JCOLORS[_i], on))

        def _toggle_ylock(key, chart):
            """Lock the Y axis to the current view (stops autoscale jitter)."""
            locked = not _plot_ylock[key][0]
            _plot_ylock[key][0] = locked
            _plot_ylock_btns[key].text = 'Y: Lock' if locked else 'Y: Auto'

        def _chip_style(color, on):
            """Inline style for a joint chip: filled in the joint's colour when
            selected, faint outline when off."""
            if on:
                return (f'background-color:{color};color:#0b0e13;'
                        'border:1px solid transparent;font-weight:600;'
                        'border-radius:9999px;padding:1px 10px;cursor:pointer;'
                        'font-size:12px;line-height:18px;')
            return (f'background-color:transparent;color:{color};'
                    f'border:1px solid {color};opacity:0.55;'
                    'border-radius:9999px;padding:1px 10px;cursor:pointer;'
                    'font-size:12px;line-height:18px;')

        def _mk_chip_toggle(key, i, chip):
            def _h():
                _plot_sel[key][i] = not _plot_sel[key][i]
                chip.style(replace=_chip_style(_JCOLORS[i], _plot_sel[key][i]))
            return _h

        def _checkbox_row(label_text, chart, key):
            with ui.row().classes('items-center gap-2 mb-1 mt-1'):
                ui.label(label_text).classes('text-xs font-semibold text-gray-500 w-14')
                for _i in range(JOINTS):
                    _chip = ui.label(f'J{_i+1}').style(_chip_style(_JCOLORS[_i], True))
                    _chip.on('click', _mk_chip_toggle(key, _i, _chip))
                    _plot_cbs[key].append(_chip)
                ui.separator().props('vertical')
                ui.button('All', on_click=lambda k=key: _set_all_joints(k, True)).props('flat dense size=sm')
                ui.button('None', on_click=lambda k=key: _set_all_joints(k, False)).props('flat dense size=sm')
                _plot_ylock_btns[key] = ui.button('Y: Auto', on_click=lambda k=key, c=chart: _toggle_ylock(k, c)).props('flat dense size=sm')

        def _relayout_charts():
            """Resize open charts to share the vertical space: fewer open ->
            taller each. Collapsed charts show only their header bar."""
            _visible_keys = _keys_for_unit()
            _open = [k for k in _visible_keys if not _plot_collapsed[k][0]]
            _h = _CHART_H.get(len(_open), '256px')
            for _k in _PLOT_KEYS:
                if _k not in _visible_keys:
                    _chart_cards[_k].set_visibility(False)
                    _chart_bodies[_k].set_visibility(False)
                    _collapse_btns[_k].set_visibility(False)
                    continue
                _chart_cards[_k].set_visibility(True)
                _collapse_btns[_k].set_visibility(True)
                _collapsed = _plot_collapsed[_k][0]
                _chart_bodies[_k].set_visibility(not _collapsed)
                _collapse_btns[_k].props(f"icon={'add' if _collapsed else 'remove'}")
                if not _collapsed:
                    _chart_els[_k].style(replace=f'width:100%;height:{_h};')
            _path_card.set_visibility(_plot_unit[0] == 'tcp')
            # ECharts must be told to re-fit after its container resizes.
            for _k in _open:
                try:
                    _chart_els[_k].run_chart_method('resize')
                except Exception:
                    pass   # chart not mounted yet — harmless, it sizes on render

        def _toggle_collapse(key):
            _plot_collapsed[key][0] = not _plot_collapsed[key][0]
            _relayout_charts()

        def _chart_card(title, ylabel, key):
            """Chart wrapped in a rounded card with a header (title + collapse
            button). Collapsing hides the body; open charts grow via _relayout."""
            _th = _PLOT_THEMES[_plot_theme[0]]
            _card = ui.card().classes('w-full mb-3 rounded-xl shadow-md')
            _card.style(replace=_th['card'])
            _plot_cards.append(_card)
            _chart_cards[key] = _card
            with _card:
                with ui.row().classes('items-center justify-between w-full'):
                    with ui.row().classes('items-center gap-2 no-wrap'):
                        _card_title_lbls[key] = ui.label(title) \
                            .classes('text-sm font-semibold').style(f"color:{_th['text']}")
                        # Shown when an overlay is ON but that channel carries nothing — an
                        # empty trace otherwise looks identical to "overlay off" or "no data".
                        _cmd_note_lbls[key] = ui.label('') \
                            .classes('text-xs text-orange-500 italic') \
                            .tooltip('This channel carried no value on the CAN wire, so there '
                                     'is nothing to draw — not the same as a commanded 0. '
                                     'data_pack_1 omits unset channels: holding under gravity '
                                     'compensation, for example, sends Current only and leaves '
                                     'Position and Speed unset. Enable and jog (or run EXEC) '
                                     'and all three appear.')
                    _collapse_btns[key] = ui.button(
                        on_click=lambda k=key: _toggle_collapse(k)) \
                        .props('icon=remove flat dense round size=sm') \
                        .tooltip('Collapse / expand this chart')
                _body = ui.column().classes('w-full gap-0')
                _chart_bodies[key] = _body
                with _body:
                    _chart = ui.echart(_echart_opts(title, ylabel)).classes('w-full').style('height:256px;')
                    _chart_els[key] = _chart
                    _checkbox_row(f'{key.upper()}:', _chart, key)
            return _chart

        pos_chart = _chart_card('Motor Position', 'ticks', 'pos')
        spd_chart = _chart_card('Motor Speed', 'ticks/s', 'spd')
        acc_chart = _chart_card('Joint Acceleration', 'rad/s^2', 'acc')
        cur_chart = _chart_card('Motor Current', 'mA', 'cur')
        jerk_chart  = _chart_card('Joint Jerk', 'rad/s^3', 'jerk')
        etrq_chart  = _chart_card('External Joint Torque', 'Nm', 'etrq')
        fext_chart  = _chart_card('TCP External Wrench (World)', 'N / Nm', 'fext')
        ftool_chart = _chart_card('TCP External Wrench (Tool)', 'N / Nm', 'ftool')
        _charts = {'pos': pos_chart, 'spd': spd_chart, 'acc': acc_chart, 'cur': cur_chart,
                   'jerk': jerk_chart, 'etrq': etrq_chart,
                   'fext': fext_chart, 'ftool': ftool_chart}
        # Charts that don't exist in the default (motor) unit start hidden; _relayout_charts
        # shows/hides them per _keys_for_unit on every unit switch.
        for _k0 in ('acc', 'jerk', 'etrq', 'fext', 'ftool'):
            _chart_cards[_k0].set_visibility(False)
            _chart_bodies[_k0].set_visibility(False)
            _collapse_btns[_k0].set_visibility(False)

        _path_card = ui.card().classes('w-full mb-3 rounded-xl shadow-md')
        _path_card.style(replace=_PLOT_THEMES[_plot_theme[0]]['card'])
        _plot_cards.append(_path_card)
        with _path_card:
            with ui.row().classes('items-center justify-between w-full'):
                ui.label('TCP Path').classes('text-sm font-semibold').style(f"color:{_PLOT_THEMES[_plot_theme[0]]['text']}")
                ui.toggle({'xy': 'XY', 'xz': 'XZ', 'yz': 'YZ', 'iso': 'XYZ'},
                          value='xy', on_change=lambda e: _set_path_mode(e.value)).props('dense')
            _path_chart = ui.echart(_path_echart_opts()).classes('w-full').style('height:520px;')
        _path_card.set_visibility(False)
        # NOTE: do NOT call _relayout_charts() here — the charts aren't mounted in
        # the browser yet, and run_chart_method('resize') on an unmounted chart
        # wedges the render (blank Plots tab). Charts are created at the correct
        # The motor view starts with 3 visible charts at 256px; acceleration is
        # shown for Joint/TCP modes, while torque/current is hidden for TCP.
        # _relayout_charts only runs on a user collapse/expand, when mounted.

        # ── TEMP diagnostics: live Processes table under the plots, so the GUI
        # core's CPU% is visible while toggling joints. Remove when done tuning.
        with ui.card().classes('w-full mt-3'):
            with ui.row().classes('items-center gap-2'):
                ui.label('Processes (diagnostic)').classes('text-sm font-semibold text-gray-600 uppercase tracking-wide')
                ui.label('TEMP — watch GUI CPU% vs selected joints').classes('text-xs text-gray-400')
            process_table_diag = ui.table(columns=process_columns, rows=[], row_key='name') \
                .classes('w-full').props('dense flat')

        async def _take_snapshot():
            # Export with the current theme's background so dark charts don't
            # come out on a white page.
            _opts = {'type': 'png', 'pixelRatio': 2,
                     'backgroundColor': _PLOT_THEMES[_plot_theme[0]]['bg']}
            _snapshot_titles = {'pos': 'Position', 'spd': 'Speed', 'acc': 'Acceleration', 'cur': 'Torque/Current'}
            _shots = []
            for _key in _keys_for_unit():
                _url = await _charts[_key].run_chart_method('getDataURL', _opts)
                _shots.append((_snapshot_titles[_key], _url))
            with ui.dialog().props('maximized') as _dlg, ui.card().classes('w-full h-full overflow-auto'):
                with ui.row().classes('items-center justify-between w-full mb-2'):
                    ui.label('Plot Snapshot').classes('text-lg font-semibold')
                    ui.button('Close', icon='close', on_click=_dlg.close).props('flat color=grey size=sm')
                for _title, _url in _shots:
                    with ui.row().classes('items-center justify-between w-full mt-2'):
                        ui.label(_title).classes('text-sm font-semibold text-gray-600 uppercase tracking-wide')
                        ui.button('Download', icon='download', on_click=lambda u=_url, t=_title: ui.download(u, f'snapshot_{t.lower()}.png')).props('flat size=sm color=primary')
                    ui.image(_url).classes('w-full')
            _dlg.open()

        def _export_csv():
            """Dump the current plot buffer to a CSV the browser downloads.
            One row per sample: tick + all visible metrics for all joints, in the
            currently-selected units. Commanded/target columns included when the
            overlay buffers hold data."""
            import io
            _u = _plot_unit[0]
            _keys = list(_keys_for_unit(_u))
            # Header and rows are driven by the SAME key lists, so they can never drift out of
            # alignment when a chart is added (they used to be hardcoded per-unit tuples).
            # A key only gets cmd/target/filtered columns if that channel exists for it.
            _cmd_has = any(len(q) for q in _plot_pos_cmd)     # overlay was active
            _tgt_has = any(len(q) for q in _plot_pos_tgt)     # overlay was active
            _cmd_keys = [k for k in _keys if k in _CMD_DATA] if _cmd_has else []
            _tgt_keys = [k for k in _keys if k in _TGT_DATA] if _tgt_has else []
            _filt_keys = [k for k in _keys if k in _FILT_DATA
                          and any(len(q) for q in _FILT_DATA[k])]

            _hdr = ['tick']
            for _k in _keys:
                _lbl = _ang_label(_UNIT_META[_u][_k][2])      # unit label (rad or deg/Nm/…)
                for i in range(JOINTS):
                    _hdr.append(f'J{i+1}_{_k}[{_lbl}]')
            for _k in _cmd_keys:
                for i in range(JOINTS):
                    _hdr.append(f'J{i+1}_{_k}_cmd')
            for _k in _tgt_keys:
                for i in range(JOINTS):
                    _hdr.append(f'J{i+1}_{_k}_target')
            for _k in _filt_keys:
                for i in range(JOINTS):
                    _hdr.append(f'J{i+1}_{_k}_filtered')

            _ticks = list(_plot_buf_tick)
            _snap      = {k: [list(q) for q in _KEY_DATA[k]] for k in _keys}
            _snap_cmd  = {k: [list(q) for q in _CMD_DATA[k]] for k in _cmd_keys}
            _snap_tgt  = {k: [list(q) for q in _TGT_DATA[k]] for k in _tgt_keys}
            _snap_filt = {k: [list(q) for q in _FILT_DATA[k]] for k in _filt_keys}

            buf = io.StringIO()
            buf.write(','.join(_hdr) + '\n')
            for r in range(len(_ticks)):
                row = [str(_ticks[r])]
                for _k in _keys:
                    _d = _snap[_k]
                    row += [str(_d[i][r]) if r < len(_d[i]) else '' for i in range(JOINTS)]
                # overlay buffers align to the TAIL of the tick axis, not the head.
                for _grp, _ks in ((_snap_cmd, _cmd_keys), (_snap_tgt, _tgt_keys),
                                  (_snap_filt, _filt_keys)):
                    for _k in _ks:
                        _d = _grp[_k]
                        _off = len(_ticks) - len(_d[0]) if _d and _d[0] else len(_ticks)
                        for i in range(JOINTS):
                            _ri = r - _off
                            row.append(str(_d[i][_ri]) if 0 <= _ri < len(_d[i]) else '')
                buf.write(','.join(row) + '\n')

            ui.download(buf.getvalue().encode('utf-8'),
                        f'plot_{_u}_{int(time.time())}.csv')

    # ─── Firmware ───────────────────────────────────────────
    # Motor-driver firmware updates over CAN. See docs/firmware_flashing.md.
    #
    # This tab NEVER opens a CAN socket. It drives tools/flash_motor_firmware.py
    # as a subprocess with --progress-json and renders the stream, which keeps
    # "only RTI transmits during normal operation" intact and keeps ONE flashing
    # implementation — the thing this tab runs is the thing you test from a
    # terminal. The handover itself is RTI's FLASHING mode: RTI stays alive and
    # keeps ticking (E-stop, IO, telemetry) but stops transmitting.
    # These two run while the tab is being BUILT (this file executes top-to-bottom
    # at import), so unlike the rest of the firmware helpers they cannot live
    # further down next to the handlers.
    _FW_LATEST = "__latest__"       # sentinel option: "whatever GitHub calls latest"
    _fw_gh_module = None

    def _fw_github_module():
        """tools/firmware_github.py, imported lazily and cached.

        Lazy and never imported at module scope on purpose: the dashboard has to
        keep starting on a box where tools/ is missing or broken. A firmware
        convenience is not worth a dead GUI.
        """
        global _fw_gh_module
        if _fw_gh_module is None:
            tools_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools")
            if tools_dir not in sys.path:
                sys.path.insert(0, tools_dir)
            import firmware_github
            _fw_gh_module = firmware_github
        return _fw_gh_module

    def _fw_product_options():
        """{product_id: label} from tools/firmware_github.py, or {} if unreadable.

        Not hardcoded: the registry there is the single source of truth for which
        repo a product maps to. Empty on failure rather than a guess — an empty
        picker is honest, a stale one flashes the wrong image.
        """
        try:
            return {pid: spec["label"]
                    for pid, spec in _fw_github_module().FIRMWARE_PRODUCTS.items()}
        except Exception:
            return {}

    with ui.tab_panel(tab_firmware):
        with ui.row().classes("gap-4 items-start flex-wrap w-full"):

            with ui.card().classes("flex-1 min-w-96"):
                ui.label("1 · Prepare").classes(
                    "text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                ui.label("Flashing reboots a driver and it LOSES TORQUE while it "
                         "happens. Park the arm so it can go limp safely, confirm "
                         "it, then hand over the bus.").classes("text-xs text-orange-700 mb-2")
                with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
                    ui.label("Mode");          fw_mode_label    = ui.label("---")
                    ui.label("Homed");         fw_homed_label   = ui.label("---")
                    ui.label("Gravity load");  fw_gravity_label = ui.label("--- Nm")
                    ui.label("Park pose");     fw_parkpose_label = ui.label("---")

                # Gravity is shown, NOT enforced. It says how hard the motors are
                # working, not whether the arm is resting on something — and PAR6's
                # own standby pose reads 1.80 Nm, so the old 1.0 Nm gate refused
                # the correct answer. A human decides now.
                ui.label("Gravity load is information, not a gate — a parked arm "
                         "can still read a real torque.").classes("text-xs text-gray-500")

                with ui.row().classes("gap-2 mt-2 items-center"):
                    fw_park_btn = ui.button("Move to park pose",
                                            on_click=lambda: handle_fw_move_to_park()) \
                        .props("outline color=primary")
                    fw_park_hint = ui.label("").classes("text-xs text-gray-500")

                fw_parked_confirm = ui.checkbox(
                    "I confirm the arm is parked and can go limp safely") \
                    .classes("text-xs")
                fw_force = ui.checkbox(
                    "Override — arm is NOT parked but is physically supported") \
                    .classes("text-xs")
                ui.label("Override exists for a driver that lost its firmware and "
                         "cannot be commanded to move at all. It asserts you have "
                         "supported the arm by other means.") \
                    .classes("text-xs text-gray-500")

                with ui.row().classes("gap-2 mt-2"):
                    fw_enter_btn = ui.button("Hand over bus (enter FLASHING)",
                                             on_click=lambda: handle_fw_enter()) \
                        .props("color=orange")
                    fw_exit_btn = ui.button("Take bus back (exit to IDLE)",
                                            on_click=lambda: handle_fw_exit()).props("outline")

                ui.separator().classes("my-3")
                ui.label("Who is on the bus?").classes(
                    "text-xs font-semibold text-gray-600 uppercase tracking-wide")
                with ui.row().classes("gap-2 mt-1 items-center"):
                    fw_scan_passive_btn = ui.button(
                        "Passive scan", on_click=lambda: handle_fw_scan(False)) \
                        .props("outline dense")
                    fw_scan_active_btn = ui.button(
                        "Active scan", on_click=lambda: handle_fw_scan(True)) \
                        .props("outline dense color=orange")
                fw_scan_chips = ui.row().classes("gap-1 flex-wrap mt-2 items-center")
                with fw_scan_chips:
                    ui.label("not scanned").classes("text-xs text-gray-500")
                ui.label("Passive finds boards already sitting in a bootloader and "
                         "touches nothing else — that is how you find a board left "
                         "behind by an interrupted flash. Active REBOOTS every motor "
                         "on the bus to ask, and takes ~20 s because a driver needs "
                         "~8 s to reach its bootloader.").classes("text-xs text-gray-500")

            with ui.card().classes("flex-1 min-w-96"):
                ui.label("2 · Firmware source").classes(
                    "text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                fw_source = ui.radio({"file": "Local path",
                                      "upload": "Upload .bin",
                                      "github": "GitHub release"},
                                     value="file",
                                     on_change=lambda: _fw_sync_source()) \
                    .props("inline dense").classes("text-sm")

                with ui.column().classes("w-full gap-1") as fw_file_box:
                    fw_file = ui.input("Firmware .bin (path on the Pi)") \
                        .classes("w-full").props("dense")

                with ui.column().classes("w-full gap-1") as fw_upload_box:
                    ui.upload(on_upload=lambda e: handle_fw_upload(e),
                              auto_upload=True, max_files=1) \
                        .props("accept=.bin flat dense").classes("w-full")
                    fw_upload_label = ui.label("no file uploaded yet") \
                        .classes("text-xs font-mono text-gray-500")
                    ui.label("Saved under temp_data/firmware/uploads/ so the exact "
                             "image can be re-flashed from the CLI.") \
                        .classes("text-xs text-gray-500")

                with ui.column().classes("w-full gap-1") as fw_github_box:
                    _fw_products = _fw_product_options()
                    fw_product = ui.select(_fw_products, label="Driver",
                                           value=next(iter(_fw_products), None)) \
                        .props("dense").classes("w-full")
                    with ui.row().classes("gap-2 items-end w-full"):
                        fw_version = ui.select({_FW_LATEST: "Latest release"},
                                               value=_FW_LATEST, label="Version") \
                            .props("dense").classes("flex-1")
                        fw_fetch_btn = ui.button(
                            "Fetch", on_click=lambda: handle_fw_fetch_releases()) \
                            .props("outline dense")
                    fw_github_note = ui.label(
                        "Fetch lists published releases. The image is downloaded, "
                        "sha256-verified and cached by the flasher itself.") \
                        .classes("text-xs text-gray-500")

                # The radio defaults to "file", so the other two start hidden.
                # Set inline rather than by calling _fw_sync_source(): this block
                # runs at import, and that handler is defined further down.
                fw_upload_box.set_visibility(False)
                fw_github_box.set_visibility(False)

                with ui.row().classes("gap-2 mt-2 items-center"):
                    fw_erase = ui.checkbox("Erase first", value=True).classes("text-xs")
                    fw_validate_btn = ui.button(
                        "Validate", on_click=lambda: handle_fw_validate()) \
                        .props("outline dense")

            with ui.card().classes("flex-1 min-w-96"):
                ui.label("3 · Flash").classes(
                    "text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
                with ui.row().classes("gap-2 items-center"):
                    fw_node = ui.number("Bootloader ID", value=0, min=0, max=13, step=1) \
                        .classes("w-36")
                    fw_node_role = ui.label("").classes("text-xs text-gray-500")
                fw_app_node = ui.number("App node ID (blank = same)",
                                        value=None, min=0, max=13, step=1) \
                    .classes("w-full").props("dense")
                ui.label("Two IDs because they address different things: the app node "
                         "receives the reboot-to-bootloader command, the bootloader ID "
                         "receives the flash. They match unless a board came back on a "
                         "different id.").classes("text-xs text-gray-500")
                with ui.row().classes("gap-2 mt-2"):
                    fw_flash_btn = ui.button("FLASH THIS NODE",
                                             on_click=lambda: handle_fw_flash()) \
                        .props("color=red")
                    fw_cancel_btn = ui.button("Cancel",
                                              on_click=lambda: handle_fw_cancel()) \
                        .props("outline color=red")
                fw_progress = ui.linear_progress(value=0.0, show_value=False) \
                    .classes("w-full mt-2")
                fw_stage_label = ui.label("idle").classes("text-xs font-mono")

        # ─── 4 · Update whole robot ──────────────────────────────────────
        # Card 3 flashes ONE node with ONE file. That cannot express a real
        # robot: PAR6 is six stepfoc joints plus a spectral-bldc gripper, so
        # updating "the robot" is never one image. The unit here is a VERSION —
        # each node's product comes from its <driver_type>, and every node gets
        # the image that belongs to it.
        with ui.card().classes("w-full mt-2"):
            with ui.row().classes("items-center gap-3 w-full"):
                ui.label("4 · Update whole robot").classes(
                    "text-sm font-semibold text-gray-600 uppercase tracking-wide")
                ui.label("every declared node, each with its own driver's image") \
                    .classes("text-xs text-gray-500")

            with ui.row().classes("gap-4 items-end flex-wrap w-full mt-1"):
                fwr_source = ui.radio({"github": "GitHub release",
                                       "dir": "Local folder"},
                                      value="github",
                                      on_change=lambda: _fwr_sync_source()) \
                    .props("inline dense").classes("text-sm")
                with ui.row().classes("gap-2 items-end") as fwr_github_box:
                    fwr_version = ui.select({_FW_LATEST: "Latest release"},
                                            value=_FW_LATEST, label="Target version") \
                        .props("dense").classes("w-56")
                    ui.button("Fetch versions",
                              on_click=lambda: handle_fwr_fetch()).props("outline dense")
                with ui.row().classes("gap-2 items-end") as fwr_dir_box:
                    fwr_dir = ui.input("Folder with one .bin per driver",
                                       value="/home/robot/Desktop") \
                        .props("dense").classes("w-96")
                # Radio defaults to github; set inline because this block runs at
                # import and _fwr_sync_source is defined further down.
                fwr_dir_box.set_visibility(False)
                ui.button("Check", on_click=lambda: handle_fwr_preflight()) \
                    .props("outline dense")

            fwr_table = ui.column().classes("w-full mt-2 font-mono text-xs gap-0")
            with fwr_table:
                ui.label("Press Check to resolve images and read what each node "
                         "is running.").classes("text-gray-500")

            fwr_preflight = ui.column().classes("w-full mt-1 text-xs gap-0")

            with ui.row().classes("gap-3 items-center mt-2"):
                fwr_go_btn = ui.button("UPDATE ALL NODES",
                                       on_click=lambda: handle_fwr_update()) \
                    .props("color=red")
                fwr_from = ui.number("Resume from node", value=0, min=0, max=13,
                                     step=1).classes("w-40").props("dense")
                fwr_est = ui.label("").classes("text-xs text-gray-500")
            # Nothing has been resolved yet, so there is nothing safe to run.
            # Check must pass before this becomes clickable.
            fwr_go_btn.set_enabled(False)
            ui.label("Resume-from exists because a failure part-way leaves MIXED "
                     "firmware. Re-flashing the nodes that already succeeded "
                     "burns erase cycles for nothing.") \
                .classes("text-xs text-gray-500")

        with ui.card().classes("w-full mt-2"):
            ui.label("Log").classes(
                "text-sm font-semibold mb-1 text-gray-600 uppercase tracking-wide")
            fw_log = ui.log(max_lines=400).classes("w-full h-56 text-xs font-mono")
            ui.markdown(
                "**After flashing:** take the bus back, confirm the Motor tab shows "
                "the new SW version, then **re-home** if the flashed joint lost its "
                "position reference. A flash that fails part-way is not a brick — "
                "the board waits in its bootloader; flash it again."
            ).classes("text-xs text-gray-500")

# ─── Blocks (block programming, blockly/nicegui_tab.py) ───
# An iframe AFTER the panels, not inside one: the panels' keep-alive would reload it on every tab switch.
# Guarded: the dashboard must start even if the block editor cannot.
try:
    from blockly.nicegui_tab import add_blockly_tab
    add_blockly_tab(app, tabs, 'Blocks')
    try:
        from blockly.nicegui_tab import add_runs_card
        add_runs_card(tabs, 'Blocks', display_time.fmt_precise)     # the program run history (2026-10-15)
    except Exception:
        logger.exception("program runs card unavailable")
except Exception:
    logger.exception("Blocks tab unavailable")


# ─────────────────────────────────────────────────────────────
# FIRMWARE TAB — subprocess driver
# ─────────────────────────────────────────────────────────────
# The tab owns no CAN socket and no flashing logic; it spawns the CLI and
# renders its JSON lines. Deliberately NOT start_new_session: if this GUI is
# restarted mid-flash the child must die with it. An orphaned flasher still
# transmitting after RTI has left FLASHING mode is the two-transmitter failure
# the whole design exists to prevent, whereas a killed one merely leaves the
# board in its bootloader, which is recoverable.
_FLASHER_CLI = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "tools", "flash_motor_firmware.py")
_fw_proc = None          # asyncio subprocess while a flash is running
_fw_starting = False     # claimed synchronously — covers the gap before _fw_proc
                         # is assigned. See _fw_run_cli for why that gap matters.


def _fw_busy():
    """Is a firmware operation running OR about to be? Both halves are needed:
    `_fw_proc` misses the window between the guard and the subprocess spawn."""
    return _fw_proc is not None or _fw_starting


def _fw_log(line):
    fw_log.push(f"{display_time.fmt(time.time(), '%H:%M:%S')}  {line}")


def _fw_park_state():
    """(max_gravity_nm, advisory_limit_nm, under_advisory) — DISPLAY ONLY.

    RTI no longer gates on this. Gravity torque says how hard the motors are
    working, not whether the arm is resting on something, and PAR6's own standby
    pose reads 1.80 Nm against the shipped 1.0 Nm limit — the gate refused the
    correct answer. Entry now needs an explicit human claim instead. The number
    is still worth showing: it is a useful sanity check, just not a verdict.
    """
    g = shared_data.joint['joint_gravity_torque'][:JOINTS]
    worst = float(max(abs(float(v)) for v in g)) if len(g) else 0.0
    limit = float(_sys_cfg.get('maintenance_defaults', {})
                  .get('flash_park_max_gravity_nm', 1.0))
    return worst, limit, worst <= limit


def _fw_bus_ours():
    """(ok, reason) — mirrors flash_motor_firmware.bus_available().

    The bus is ours in exactly two cases: RTI granted FLASHING, or RTI is not
    running at all (the recovery path). Checking only the mode string would
    disable every button on a box where the runtime is down — which is when the
    flasher matters most.
    """
    if not _fw_alive_cached:
        return True, ""
    mode = shared_data.robot_mode.get() or "---"
    if mode == "FLASHING":
        return True, ""
    return False, (f"RTI is in {mode} mode and still owns the bus. "
                   "Hand it over first (step 1).")


def _fw_sync_source():
    """Show only the source box the radio selects."""
    source = fw_source.value
    fw_file_box.set_visibility(source == "file")
    fw_upload_box.set_visibility(source == "upload")
    fw_github_box.set_visibility(source == "github")
    # `validate` takes a path, so it cannot check a release that has not been
    # downloaded yet. The flash itself still validates before erasing anything.
    fw_validate_btn.set_enabled(source != "github")


def handle_fw_upload(event):
    """Persist an uploaded .bin and point the flasher at it."""
    try:
        data = event.content.read()
        path = _fw_github_module().save_uploaded_firmware(event.name, data)
    except Exception as exc:
        _fw_log(f"ERROR: could not save the upload: {exc}")
        ui.notify(f"Could not save the upload: {exc}", type="negative", timeout=10000)
        return
    fw_file.value = str(path)
    fw_upload_label.set_text(f"{path}  ({len(data)} bytes)")
    _fw_log(f"Uploaded {event.name} -> {path} ({len(data)} bytes)")
    ui.notify("Uploaded. Validate it before flashing.", type="positive")


async def handle_fw_fetch_releases():
    """Populate the version picker from GitHub, falling back to the local cache."""
    product = fw_product.value
    if not product:
        ui.notify("No driver selected", type="warning")
        return
    fw_fetch_btn.disable()
    fw_github_note.set_text("Fetching release list ...")
    try:
        github = _fw_github_module()
        try:
            releases = await asyncio.to_thread(github.list_releases, product)
            options = {_FW_LATEST: "Latest release"}
            for release in releases:
                label = release["tag"]
                if release.get("prerelease"):
                    label += "  (pre-release)"
                if not (release.get("has_manifest") and release.get("has_bin")):
                    label += "  — no firmware asset"
                options[release["tag"]] = label
            fw_version.set_options(options, value=_FW_LATEST)
            fw_github_note.set_text(f"{len(releases)} release(s) on GitHub.")
            _fw_log(f"GitHub: {len(releases)} release(s) for {product}")
        except github.FirmwareFetchError as exc:
            # Egress is firewalled on some boxes (see block_internet.md), so an
            # offline Pi is expected rather than exceptional — show what has
            # already been downloaded instead of just failing.
            cached = [entry for entry in github.list_cached_firmware()
                      if entry["product_id"] == product]
            options = {entry["tag"]: f"{entry['tag']}  (cached)" for entry in cached}
            fw_version.set_options(options, value=next(iter(options), None))
            fw_github_note.set_text(
                f"GitHub unreachable. {len(cached)} cached release(s) available."
                if cached else "GitHub unreachable and nothing is cached.")
            _fw_log(f"GitHub fetch failed: {exc}")
    except Exception as exc:
        fw_github_note.set_text(f"Could not list releases: {exc}")
        _fw_log(f"ERROR: {exc}")
    finally:
        fw_fetch_btn.enable()


def _fw_source_args():
    """(cli_args, description) for the selected source, or (None, why-not)."""
    source = fw_source.value
    if source == "github":
        if not fw_product.value:
            return None, "Pick a driver first."
        tag = fw_version.value
        if tag and tag != _FW_LATEST:
            return (["--product", str(fw_product.value), "--version", str(tag)],
                    f"{fw_product.value} {tag}")
        return (["--product", str(fw_product.value), "--latest"],
                f"{fw_product.value} latest")
    path = str(fw_file.value or "").strip()
    if not path:
        return None, ("Upload a .bin first." if source == "upload"
                      else "Enter the path to a .bin first.")
    return ["--file", path], os.path.basename(path)


async def _fw_confirm(title, body, ok_text="Continue", color="orange"):
    with ui.dialog() as dialog, ui.card().classes("max-w-lg"):
        ui.label(title).classes("font-bold text-base")
        ui.label(body).classes("text-sm whitespace-pre-wrap")
        with ui.row().classes("gap-2 justify-end w-full mt-2"):
            ui.button("Cancel", on_click=lambda: dialog.submit(False)).props("outline")
            ui.button(ok_text, on_click=lambda: dialog.submit(True)) \
                .props(f"color={color}")
    return bool(await dialog)


def _fw_show_boards(boards):
    fw_scan_chips.clear()
    role = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6", 6: "gripper"}
    with fw_scan_chips:
        if not boards:
            ui.label("none in a bootloader").classes("text-xs text-gray-500")
            return
        for board in boards:
            label = f"{board} · {role[board]}" if board in role else str(board)
            ui.chip(label).props("dense color=orange text-color=white")


async def handle_fw_scan(active):
    ok, reason = _fw_bus_ours()
    if not ok:
        ui.notify(reason, type="negative", timeout=10000)
        return
    if active and not await _fw_confirm(
            "Active scan reboots every motor",
            "Every driver on the bus is commanded into its bootloader to answer, "
            "so all of them LOSE TORQUE and reboot. They return to their "
            "application ~3 s after the bus goes quiet.\n\n"
            "Takes about 20 s. Use the passive scan if you only want to find a "
            "board left behind by an interrupted flash."):
        return
    await _fw_run_cli(["scan"] + ([] if active else ["--no-reset"]),
                      "active scan" if active else "passive scan")


def _fw_park_pose_deg():
    """Park pose from the robot XML, in degrees, or None if the robot has none."""
    pose = (_xml_cfg or {}).get('park_pose')
    if not pose:
        return None
    return [math.degrees(v) for v in pose]


async def handle_fw_move_to_park():
    """Send the arm to its park pose so it can safely lose torque.

    Goes through move_joints (EXEC), which means the robot must be ENABLED and
    HOMED. That is not a limitation to work around — an unhomed robot has no
    trustworthy idea where it is, so commanding it to an absolute pose is exactly
    what you must not do. When it cannot be used, the override is the answer.
    """
    pose = _fw_park_pose_deg()
    if pose is None:
        ui.notify("This robot declares no <park_pose> in its XML — park it by "
                  "hand, then confirm.", type="warning", timeout=10000)
        return
    if shared_data.homed_state.get() != "HOMED":
        ui.notify("Robot is NOT HOMED, so it cannot be moved to an absolute pose. "
                  "Home it first, or park by hand and use the override.",
                  type="negative", timeout=12000)
        return
    pretty = ", ".join(f"{v:.0f}" for v in pose)
    if not await _fw_confirm(
            "Move the arm to its park pose?",
            f"Target (degrees): {pretty}\n\n"
            "The arm will move. Make sure its path is clear.",
            ok_text="MOVE", color="primary"):
        return
    _fw_log(f"Moving to park pose [{pretty}] deg")
    _exec(lambda r: r.move_joints(*pose), "move_joints(park)")
    ui.notify("Park move commanded — wait for it to finish, then confirm it is "
              "parked.", type="info", timeout=8000)


async def handle_fw_enter():
    """Enter FLASHING. Requires an explicit claim about the arm — see
    RobotClient.enter_flashing_mode(); there is no automatic park test."""
    worst, _limit, _ok = _fw_park_state()
    force = bool(fw_force.value)
    parked = bool(fw_parked_confirm.value)
    if not (parked or force):
        ui.notify("Confirm the arm is parked, or tick the override if it is "
                  "supported some other way. Motors go limp in FLASHING.",
                  type="negative", timeout=10000)
        return
    # The override is the one path with no machine-checkable safety net, so it
    # gets a dialog naming the consequence rather than a quiet checkbox.
    if force and not await _fw_confirm(
            "Override the park requirement?",
            # The drives' watchdog from system.xml (<drive_watchdog_ms>, 100 ms since 2026-10-13; the text said ~5 s, the
            # old bring-up value, until 2026-10-24).
            f"Every motor will LOSE TORQUE {_sys_cfg.get('drive_watchdog_ms', 100) / 1000:g} s after the robot falls "
            "silent on the bus: each drive's watchdog lets go, the joints brake into a slow sag (shorted-phase "
            "brake), the gripper coasts. They stay that way for the whole session — about 25 s per joint.\n\n"
            "If the arm is not parked or physically supported, IT WILL FALL. "
            "Anything it lands on, including a person, is in the way.\n\n"
            "Only continue if you have supported it by other means.",
            ok_text="I HAVE SUPPORTED THE ARM", color="red"):
        return
    _fw_log(f"Handing bus to the flasher — "
            + ("OVERRIDE, arm NOT parked" if force else "operator confirmed PARKED")
            + f" (gravity {worst:.2f} Nm)")
    _exec(lambda r: r.enter_flashing_mode(parked_confirmed=parked, force=force),
          "enter_flashing_mode")


def handle_fw_exit():
    _fw_log("Taking the bus back — RTI resumes normal traffic")
    _exec(lambda r: r.exit_flashing_mode(), "exit_flashing_mode")


async def _fw_run_cli(args, label):
    """Spawn the flasher CLI and pump its JSON lines into the UI."""
    global _fw_proc, _fw_starting
    # `_fw_proc` alone is NOT a sufficient guard: it is only assigned after the
    # `await` below, so two click events queued close together both saw None and
    # both spawned a flasher. Disabling the buttons does not help — an event
    # already in flight is still delivered. This dashboard is a single shared UI
    # (no @ui.page), so the two clicks can even come from different browsers.
    # `_fw_starting` is set SYNCHRONOUSLY, before any await, closing the window.
    if _fw_busy():
        ui.notify("A firmware operation is already running", type="warning")
        return
    _fw_starting = True
    _fw_set_busy(True)
    fw_progress.set_value(0.0)
    fw_stage_label.set_text(f"{label}: starting")
    _fw_log(f"$ {os.path.basename(_FLASHER_CLI)} {' '.join(args)}")
    try:
        _fw_proc = await asyncio.create_subprocess_exec(
            sys.executable, _FLASHER_CLI, *args, "--progress-json",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        async for raw in _fw_proc.stdout:
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                _fw_log(line)          # not JSON: show it rather than swallow it
                continue
            kind = msg.get("type")
            if kind == "log":
                _fw_log(msg.get("message", ""))
            elif kind == "boards":
                _fw_show_boards(sorted(msg.get("boards") or []))
            elif kind == "progress":
                fw_progress.set_value(float(msg.get("fraction") or 0.0))
                page, total = msg.get("page"), msg.get("total_pages")
                detail = f"  page {page + 1}/{total}" if page is not None and total else ""
                fw_stage_label.set_text(
                    f"{msg.get('stage', '')}: {msg.get('message', '')}{detail}")
            elif kind == "result":
                ok = bool(msg.get("ok"))
                _fw_log(("OK: " if ok else "FAILED: ") + str(msg.get("message", "")))
                fw_stage_label.set_text("done" if ok else "failed")
                ui.notify(str(msg.get("message", "")),
                          type="positive" if ok else "negative", timeout=10000)
        await _fw_proc.wait()
        _stderr = (await _fw_proc.stderr.read()).decode(errors="replace").strip()
        if _fw_proc.returncode not in (0, None) and _stderr:
            # Plain-text output is redirected to stderr in JSON mode, so this is
            # where a crash (as opposed to a clean refusal) shows up.
            _fw_log(_stderr.splitlines()[-1])
    except Exception as exc:
        _fw_log(f"ERROR: could not run the flasher: {exc}")
        ui.notify(f"Could not run the flasher: {exc}", type="negative", timeout=10000)
    finally:
        _fw_proc = None
        _fw_starting = False
        _fw_set_busy(False)


async def handle_fw_validate():
    if not fw_file.value:
        ui.notify("Enter the path to a .bin first", type="warning")
        return
    await _fw_run_cli(["validate", "--file", str(fw_file.value)], "validate")


async def handle_fw_flash():
    ok, reason = _fw_bus_ours()
    if not ok:
        ui.notify(reason, type="negative", timeout=10000)
        return
    source_args, described = _fw_source_args()
    if source_args is None:
        ui.notify(described, type="warning")
        return

    node = int(fw_node.value or 0)
    args = ["flash", "--node", str(node)] + source_args
    app_node = fw_app_node.value
    if app_node is not None and str(app_node) != "" and int(app_node) != node:
        args += ["--app-node", str(int(app_node))]
    if not fw_erase.value:
        args.append("--no-erase")

    role = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6", 6: "gripper"}
    if not await _fw_confirm(
            f"Flash node {node} ({role.get(node, 'unexpected node')})?",
            f"Image: {described}\n"
            f"{'Erases the application first. ' if fw_erase.value else ''}"
            "The driver reboots and is limp for ~25 s.\n\n"
            "An interrupted flash is NOT a brick — the board waits in its "
            "bootloader and can be flashed again.",
            ok_text="FLASH", color="red"):
        return
    await _fw_run_cli(args, f"flash node {node}")


def _fwr_sync_source():
    fwr_github_box.set_visibility(fwr_source.value == "github")
    fwr_dir_box.set_visibility(fwr_source.value == "dir")


def _fwr_source_args():
    """CLI source flags for update-robot, or (None, why-not)."""
    if fwr_source.value == "dir":
        path = str(fwr_dir.value or "").strip()
        if not path:
            return None, "Enter the folder holding the .bin files."
        return ["--dir", path], f"folder {path}"
    tag = fwr_version.value
    if tag and tag != _FW_LATEST:
        return ["--version", str(tag)], str(tag)
    return ["--latest"], "latest release"


async def handle_fwr_fetch():
    """Version list for a whole-robot update: only tags EVERY product has.

    A robot cannot be updated to a version one of its drivers never shipped, so
    offering such a tag would just produce a preflight failure later.
    """
    try:
        github = _fw_github_module()
        per_product = {}
        for product in github.FIRMWARE_PRODUCTS:
            try:
                releases = await asyncio.to_thread(github.list_releases, product)
                per_product[product] = {
                    r["tag"] for r in releases
                    if r.get("has_manifest") and r.get("has_bin")}
            except github.FirmwareFetchError:
                per_product[product] = {
                    e["tag"] for e in github.list_cached_firmware()
                    if e["product_id"] == product}
        common = set.intersection(*per_product.values()) if per_product else set()
        options = {_FW_LATEST: "Latest release"}
        for tag in sorted(common, reverse=True):
            options[tag] = tag
        fwr_version.set_options(options, value=_FW_LATEST)
        _fw_log(f"Whole-robot versions available for ALL drivers: "
                f"{sorted(common, reverse=True) or 'none'}")
        if not common:
            ui.notify("No single version exists for every driver — update them "
                      "individually, or use a local folder.",
                      type="warning", timeout=12000)
    except Exception as exc:
        _fw_log(f"ERROR listing versions: {exc}")
        ui.notify(f"Could not list versions: {exc}", type="negative", timeout=10000)


def _fwr_render(rows, preflight_lines, ok):
    fwr_table.clear()
    with fwr_table:
        ui.label(f"{'node':<6}{'role':<10}{'driver':<16}{'current':<10}target") \
            .classes("text-gray-500")
        for line in rows:
            ui.label(line)
    fwr_preflight.clear()
    with fwr_preflight:
        for line in preflight_lines:
            ui.label(line).classes(
                "text-green-700" if line.startswith("OK") else "text-red-600 font-bold")
    fwr_go_btn.set_enabled(bool(ok) and not _fw_busy())


async def handle_fwr_preflight():
    """Resolve images and read the inventory WITHOUT flashing.

    Runs the same CLI with an empty stdin so its confirmation prompt aborts:
    the preflight the operator sees is literally the preflight the update runs,
    rather than a second implementation that could disagree with it.
    """
    source_args, described = _fwr_source_args()
    if source_args is None:
        ui.notify(described, type="warning")
        return
    ok, reason = _fw_bus_ours()
    if not ok:
        _fwr_render([], [f"BLOCKED: {reason}"], False)
        return
    _fw_log(f"Preflight for a whole-robot update -> {described}")
    rows, preflight, resolved_ok = [], [], False
    try:
        proc = await asyncio.create_subprocess_exec(
            sys.executable, _FLASHER_CLI, "update-robot", *source_args,
            "--from-node", str(int(fwr_from.value or 0)),
            stdin=asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        out, err = await proc.communicate()
        text = out.decode(errors="replace") + err.decode(errors="replace")
        for line in text.splitlines():
            stripped = line.strip()
            if stripped[:1].isdigit() and len(stripped.split()) >= 4:
                rows.append(line.rstrip())
            elif "page(s), validation" in line:
                preflight.append("OK  " + stripped)
            elif stripped.startswith("ERROR"):
                preflight.append(stripped)
        resolved_ok = bool(rows) and not any(
            p.startswith("ERROR") for p in preflight)
        if resolved_ok:
            preflight.append(f"OK  {len(rows)} node(s) resolved — "
                             f"est. {len(rows) * 28 // 60 + 1} min limp")
            fwr_est.set_text(f"~{len(rows) * 28 // 60 + 1} min, arm limp throughout")
        _fw_log(text.strip().splitlines()[-1] if text.strip() else "(no output)")
    except Exception as exc:
        preflight.append(f"ERROR {exc}")
    _fwr_render(rows, preflight or ["ERROR could not resolve anything"], resolved_ok)


async def handle_fwr_update():
    ok, reason = _fw_bus_ours()
    if not ok:
        ui.notify(reason, type="negative", timeout=10000)
        return
    source_args, described = _fwr_source_args()
    if source_args is None:
        ui.notify(described, type="warning")
        return
    start = int(fwr_from.value or 0)
    if not await _fw_confirm(
            "Update every declared node?",
            f"Target: {described}\n"
            + (f"Resuming from node {start}.\n" if start else "")
            + "\nEach driver reboots in turn and the arm is LIMP for the whole "
              "run — several minutes, not seconds.\n\n"
              "A failure part-way leaves MIXED firmware; the log names the node "
              "to resume from. Homing is invalidated either way.",
            ok_text="UPDATE ALL", color="red"):
        return
    args = ["update-robot"] + source_args + ["--from-node", str(start)]
    await _fw_run_cli(args, f"update robot -> {described}")


def handle_fw_cancel():
    """SIGINT the child — the CLI handles it and explains the recoverable state."""
    if _fw_proc is None:
        ui.notify("Nothing is running", type="info")
        return
    _fw_log("Cancelling — sending SIGINT to the flasher")
    try:
        _fw_proc.send_signal(signal.SIGINT)
    except Exception as exc:
        _fw_log(f"Could not signal the flasher: {exc}")


def _fw_set_busy(busy):
    """One child at a time owns the bus, so every action locks out the others."""
    for button in (fw_flash_btn, fw_scan_passive_btn, fw_scan_active_btn,
                   fw_validate_btn, fw_enter_btn, fw_exit_btn, fw_park_btn):
        button.set_enabled(not busy)
    if busy:
        # The whole-robot button is re-armed only by a fresh preflight, never by
        # simply going idle — the inventory it was enabled against is stale once
        # anything else has touched the bus.
        fwr_go_btn.set_enabled(False)
    if not busy:
        # refresh_firmware_tab re-applies the real gating (mode, bus ownership)
        # on its next pass; this only lifts the blanket lock.
        fw_validate_btn.set_enabled(fw_source.value != "github")


_fw_prev_tick = -1
_fw_stale_cnt = 0
_fw_alive_cached = True   # assume RTI is up until a refresh proves otherwise:
                          # the safe default demands an explicit FLASHING grant


def _fw_rti_alive():
    """Own liveness check — update_tables keeps `_rti_alive` as a local, and
    reaching into it would couple two refreshers running at different rates.
    Two stale reads at 0.5 s = 1 s without loop_tick advancing."""
    global _fw_prev_tick, _fw_stale_cnt
    tick = int(shared_data.rti["loop_tick"][0])
    if tick != _fw_prev_tick:
        _fw_prev_tick, _fw_stale_cnt = tick, 0
        return True
    _fw_stale_cnt += 1
    return _fw_stale_cnt < 2


def refresh_firmware_tab():
    """Live park verdict + mode badge. Cheap, but only while the tab is open."""
    global _fw_alive_cached
    if _active_tab_name(tabs.value) != 'Firmware':
        return
    _alive = _fw_rti_alive()
    _fw_alive_cached = _alive       # _fw_bus_ours reads this; _fw_rti_alive is
                                    # stateful and must be called once per pass
    mode = shared_data.robot_mode.get() or "---"
    fw_mode_label.set_text(mode)
    fw_mode_label.classes(replace="font-mono " + (
        "text-orange-600 font-bold" if mode == "FLASHING" else "text-gray-700"))

    worst, _limit, _ok = _fw_park_state()
    _homed = shared_data.homed_state.get() or "---"
    _pose = _fw_park_pose_deg()
    fw_parkpose_label.set_text(
        ", ".join(f"{v:.0f}" for v in _pose) + " deg" if _pose else "not declared")

    # Shared memory OUTLIVES the RTI process, so with RTI stopped these are
    # whatever the last run left behind. Saying so is the difference between a
    # readout and a lie; this project has already been bitten once by stale shm
    # looking live (see currently_doing.md).
    if not _alive:
        fw_gravity_label.set_text("stale (RTI stopped)")
        fw_homed_label.set_text("stale (RTI stopped)")
        fw_homed_label.classes(replace="font-mono text-gray-500")
    else:
        fw_gravity_label.set_text(f"{worst:.2f} Nm")
        fw_homed_label.set_text(_homed)
        fw_homed_label.classes(replace="font-mono " + (
            "text-green-700" if _homed == "HOMED" else "text-orange-600 font-bold"))

    # "Move to park" goes through EXEC, which requires ENABLED + HOMED. Say WHY
    # it is unavailable rather than just greying it out — the reason determines
    # what the operator should do instead (home it, or use the override).
    _can_park = bool(_pose) and _alive and _homed == "HOMED" and mode != "FLASHING"
    fw_park_btn.set_enabled(_can_park and not _fw_busy())
    if not _pose:
        fw_park_hint.set_text("no <park_pose> in the robot XML")
    elif not _alive:
        fw_park_hint.set_text("RTI is stopped")
    elif mode == "FLASHING":
        fw_park_hint.set_text("already in FLASHING")
    elif _homed != "HOMED":
        fw_park_hint.set_text("robot is not homed — park by hand, then override")
    else:
        fw_park_hint.set_text("")

    # The bus is ours on a FLASHING grant OR with RTI stopped — same rule the
    # CLI enforces in bus_available(), so the buttons cannot promise something
    # the flasher would then refuse.
    bus_ours, _ = _fw_bus_ours()
    idle = not _fw_busy()
    fw_flash_btn.set_enabled(bus_ours and idle)
    fw_scan_passive_btn.set_enabled(bus_ours and idle)
    fw_scan_active_btn.set_enabled(bus_ours and idle)
    fw_validate_btn.set_enabled(idle and fw_source.value != "github")
    fw_enter_btn.set_enabled(_alive and mode != "FLASHING" and idle)
    fw_exit_btn.set_enabled(_alive and mode == "FLASHING" and idle)
    fw_cancel_btn.set_visibility(not idle)

    role = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6", 6: "gripper"}
    fw_node_role.set_text(role.get(int(fw_node.value or 0), "unexpected node"))


# Leaving the page must not leave a flasher transmitting on the bus.
app.on_shutdown(lambda: _fw_proc.kill() if _fw_proc is not None else None)


# ─────────────────────────────────────────────────────────────
# UPDATE FUNCTION
# ─────────────────────────────────────────────────────────────
def _active_tab_name(value):
    """Normalise tabs.value (None | str | ui.tab) to the tab's name string."""
    if value is None:
        return 'Joints & TCP'
    if isinstance(value, str):
        return value
    return getattr(value, '_props', {}).get('name', 'Joints & TCP')


def _gui_has_can_gripper() -> bool:
    raw = _xml_cfg.get('gripper_CAN_gripper', 0)
    return not (isinstance(raw, float) and math.isnan(raw)) and int(raw) == 1


def _robot_snapshot() -> dict:
    """The robot's state for the support bundle's info.txt (gui_logs_tab.py); None for a block not set up."""
    def read(block, how="get"):
        return getattr(block, how)() if block is not None else None
    snap = {"robot_state": read(shared_data.robot_state), "robot_mode": read(shared_data.robot_mode),
            "homed": read(shared_data.homed_state), "errors": read(shared_data.error_state, "get_json")}
    try:            # every part's version, the drives' included (utility/versions.py, 2026-10-21)
        from utility.versions import report as _versions_report
        snap["versions"] = _versions_report(shared_data, int(_sys_cfg["joint_num"]), _gui_has_can_gripper(),
                                            _sys_cfg.get('pcb_version', 1))
    except Exception:
        logger.exception("versions for the support bundle unavailable")
    return snap


def _set_angle_unit(unit):
    """The status bar's Angles toggle: every angle readout re-labelled and shown in `unit` ('rad' | 'deg') from now
    on. The plots start afresh (_relabel_charts clears their buffers: the two units never share a chart)."""
    _ANGLE_UNIT[0] = 'deg' if unit == 'deg' else 'rad'
    joint_table.columns = [{**c, 'label': _ang_label(c['label'])} for c in joint_columns]
    joint_table.update()
    tcp_table.columns = [{**c, 'label': _ang_label(c['label'])} for c in tcp_columns]
    tcp_table.update()
    _refresh_jog_speed_labels()
    _relabel_charts(relayout=False)
    _preview_set_units()


def update_tables():
    global _prev_rti_tick, _rti_stale_cnt
    global _gui_last_t, _gui_exec_last, _gui_period_ema, _gui_exec_ema, _gui_period_peak, _gui_exec_peak

    _t_start = time.monotonic()
    _gui_period = (_t_start - _gui_last_t) * 1e3 if _gui_last_t else 0.0
    _gui_last_t = _t_start
    if _gui_period > 0:
        _gui_period_ema  = _GUI_EMA_ALPHA * _gui_period  + (1 - _GUI_EMA_ALPHA) * _gui_period_ema
        _gui_period_peak = max(_gui_period_peak, _gui_period)

    # ── RTI process liveness ──────────────────────────────────
    # If loop_tick has not advanced for 3 consecutive 100 ms refreshes (~300 ms),
    # RTI is considered stopped or frozen.
    _cur_tick = int(shared_data.rti["loop_tick"][0])
    if _cur_tick != _prev_rti_tick:
        _rti_stale_cnt = 0
        _rti_alive = True
    else:
        _rti_stale_cnt += 1
        _rti_alive = _rti_stale_cnt < 3
    _prev_rti_tick = _cur_tick

    rti_process_badge.text = "RTI: RUNNING" if _rti_alive else "RTI: STOPPED"
    rti_process_badge.props(f"color={'positive' if _rti_alive else 'negative'} outline")

    # MOTORS OFF is a toggle, so it has to say which way it will go. Reading the
    # mode (rather than tracking a local flag) keeps it honest when something
    # else — an error, another browser — changes the mode underneath us.
    _limped = shared_data.robot_mode.get() == "SAFETY_STOP"
    _limp_btn.text = "RESTORE MOTORS" if _limped else "MOTORS OFF"
    _limp_btn.props(f"color={'positive' if _limped else 'deep-orange'} size=sm")
    # Inertia / Coriolis feedforward buttons: the robot's real state (RTI starts both OFF; a script may toggle them).
    for _dbtn, _dkey, _dname in ((_inertia_btn, "inertia_comp_enabled", "Inertia"),
                                 (_coriolis_btn, "coriolis_comp_enabled", "Coriolis")):
        _don = shared_data.rti[_dkey][0] > 0.5
        _dtext = f"{_dname} {'ON' if _don else 'OFF'}"
        if _dbtn.text != _dtext:
            _dbtn.set_text(_dtext)
            _dbtn.props(f"color={'positive' if _don else 'grey'} outline size=sm")

    # ── Robot CAN connection ──────────────────────────────────
    _robot_can_ok = bool(shared_data.rti["robot_can_connected"][0])
    robot_can_badge.text = "ROBOT: CONNECTED" if _robot_can_ok else "ROBOT: NO CAN"
    robot_can_badge.props(f"color={'positive' if _robot_can_ok else 'negative'} outline")

    # ── Status bar ───────────────────────────────────────────
    mode = shared_data.robot_mode.get()
    robot_mode_badge.text = f"MODE: {mode}"
    _pl_live.text = (f"RTI: {shared_data.rti['payload_mass'][0]:.3f} kg @ ({shared_data.rti['payload_com_x'][0]*1000:.0f}, "
                     f"{shared_data.rti['payload_com_y'][0]*1000:.0f}, {shared_data.rti['payload_com_z'][0]*1000:.0f}) mm")
    robot_mode_badge.props(
        f"color={'positive' if mode in ('running', 'rti', 'active') else 'negative' if 'error' in mode.lower() else 'grey'}"
    )
    robot_state_badge.text = f"STATE: {shared_data.robot_state.get()}"
    homed = shared_data.homed_state.get()
    homed_badge.text = homed
    homed_badge.props(f"color={'positive' if homed == 'HOMED' else 'orange'} outline")
    sim_badge.visible = shared_data.sim_mode.get() == "ON"
    err_val = int(shared_data.rti['error_active'][0])
    error_badge.text = f"ERR: {err_val}"
    error_badge.props(f"color={'negative' if err_val else 'positive'} outline")
    safety_violation = int(shared_data.rti['safety_violation'][0])
    safety_badge.text = "SAFETY: STOP" if safety_violation else "SAFETY: OK"
    safety_badge.props(f"color={'negative' if safety_violation else 'positive'} outline")
    robot_cmd_label.text = f"CMD: {shared_data.robot_command.get()}"
    recipe_label.text    = f"Recipe: {shared_data.active_recipe.get()}"

    # ── Jog button blocked-direction indicators ───────────────────────────────
    # Turn a direction button yellow when the JogHandler has blocked it at a soft limit.
    # Only meaningful in JOG mode — reset to default in all other modes.
    # props(remove="color") is required to actually clear a previously set color=warning;
    # omitting color from the props string leaves the old value in place in NiceGUI.
    _in_jog = (mode == "JOG")
    for _ji in range(6):
        _blocked = int(shared_data.joint['jog_blocked'][_ji]) if _in_jog else 0
        if _blocked == -1:
            _jog_buttons[_ji]['neg'].props("color=warning")
            _jog_buttons[_ji]['pos'].props(remove="color")
        elif _blocked == 1:
            _jog_buttons[_ji]['neg'].props(remove="color")
            _jog_buttons[_ji]['pos'].props("color=warning")
        else:
            _jog_buttons[_ji]['neg'].props(remove="color")
            _jog_buttons[_ji]['pos'].props(remove="color")

    # Only refresh elements for the tab the user is actually viewing.
    # tabs.value may be None, a string, or a ui.tab object (when initialised via
    # `ui.tab_panels(tabs, value=tab_joints)`). Normalise to the tab's name string
    # so the `== 'Joints & TCP'` comparisons below actually match on first render.
    _tv = tabs.value
    if _tv is None:
        _tab = 'Joints & TCP'
    elif isinstance(_tv, str):
        _tab = _tv
    else:
        _tab = getattr(_tv, '_props', {}).get('name', 'Joints & TCP')

    # ── Joints & TCP tab ──────────────────────────────────────
    if _tab == 'Joints & TCP':
        joint_table.rows = [{
            'j':       f'J{i+1}',
            'homed':   'YES' if shared_data.rti['joint_homed'][i] else 'NO',
            'pos':     fmt(_ang(shared_data.joint['joint_position'][i])),
            'spd':     fmt(_ang(shared_data.joint['joint_speed'][i])),
            'trq':     fmt(shared_data.joint['joint_torque'][i]),
            'acc':     fmt(_ang(shared_data.joint['joint_acceleration'][i])),
            'jrk':     fmt(_ang(shared_data.joint['joint_jerk'][i])),
            'pos_cmd': fmt(_ang(shared_data.joint['joint_position_commanded'][i])),
            'spd_cmd': fmt(_ang(shared_data.joint['joint_speed_commanded'][i])),
            'trq_cmd': fmt(shared_data.joint['joint_torque_commanded'][i]),
            'trq_ext': fmt(shared_data.tcp['torque_external'][i]),
        } for i in range(JOINTS)]
        joint_table.update()

        tcp_table.rows = [{
            'axis':      lbl,
            'pose':      fmt(_ang_tcp(shared_data.tcp['robot_pose'][i], i)),
            'spd':       fmt(_ang_tcp(shared_data.tcp['TCP_speed'][i], i)),
            'force_ext': fmt(shared_data.tcp['force_external'][i]),
            'pose_cmd':  fmt(_ang_tcp(shared_data.tcp['TCP_pose_commanded'][i], i)),
            'spd_cmd':   fmt(_ang_tcp(shared_data.tcp['TCP_speed_commanded'][i], i)),
        } for i, lbl in enumerate(["X", "Y", "Z", "RX", "RY", "RZ"])]
        tcp_table.update()

    # ── Motor tab ─────────────────────────────────────────────
    if _tab == 'Motor':
        motor_table.rows = [{
            'joint': f'J{i+1}',
            'pos':   int(shared_data.motor['motor_position'][i]),
            'spd':   int(shared_data.motor['motor_speed'][i]),
            'cur':   int(shared_data.motor['motor_current'][i]),
            'err':   int(shared_data.motor['global_motor_error'][i]),
            'HW':    int(shared_data.motor['HARDWARE_VERSION'][i]),
            'SW':    int(shared_data.motor['SOFTWARE_VERSION'][i]),
            'SN':    int(shared_data.motor['SERIAL_NUMBER'][i]),
        } for i in range(JOINTS)]
        motor_table.update()

        # What was actually sent this tick. NaN = channel not commanded -> render as '—'.
        def _sent_fmt(v):
            return '—' if v != v else str(int(v))       # v != v is the NaN test
        sent_table.rows = [{
            'joint': f'J{i+1}',
            'spos':  _sent_fmt(shared_data.motor['motor_position_sent'][i]),
            'sspd':  _sent_fmt(shared_data.motor['motor_speed_sent'][i]),
            'scur':  _sent_fmt(shared_data.motor['motor_current_sent'][i]),
        } for i in range(JOINTS)]
        sent_table.update()
        _pack = shared_data.rti['motor_pack_mode_sent'][0]
        if _pack != _pack:      # NaN -> dispatch sent nothing (homing owns its sends)
            sent_mode_badge.text = 'not dispatched'
            sent_mode_badge.props('color=grey outline')
        elif _pack >= 0.5:
            sent_mode_badge.text = 'data_pack_PD (firmware KP/KD impedance)'
            sent_mode_badge.props('color=purple outline')
        else:
            sent_mode_badge.text = 'data_pack_1 (cascade PID, Current = feedforward)'
            sent_mode_badge.props('color=primary outline')

        info_table.rows = [{
            'joint': f'J{i+1}',
            'temp':  int(shared_data.motor['motor_temperature'][i]),
            'volt':  int(shared_data.motor['motor_voltage'][i]),
            'kt':    f"{shared_data.motor['motor_Kt'][i]:.4f}",
            'kt_src': _fmt_kt_src(shared_data.motor['motor_Kt_source'][i]),
        } for i in range(JOINTS)]
        info_table.update()

        error_table.rows = [{
            'joint': f'J{i+1}',
            'temp':  int(shared_data.motor['motor_temperature_error'][i]),
            'enc':   int(shared_data.motor['motor_encoder_error'][i]),
            'vbus':  int(shared_data.motor['motor_vbus_error'][i]),
            'drv':   int(shared_data.motor['motor_driver_error'][i]),
            'vel':   int(shared_data.motor['motor_velocity_error'][i]),
            'cur':   int(shared_data.motor['motor_current_error'][i]),
            'estop': int(shared_data.motor['motor_estop_error'][i]),
            'wd':    int(shared_data.motor['motor_watchdog_error'][i]),
            'cal':   int(shared_data.motor['motor_calibrated'][i]),
        } for i in range(JOINTS)]
        error_table.update()

        for _i in range(JOINTS):
            _m_connected = int(shared_data.rti['CAN_node_connection_error'][_i]) == 0
            motor_conn_badges[_i].text = "CONNECTED" if _m_connected else "DISCONNECTED"
            motor_conn_badges[_i].props(f"color={'positive' if _m_connected else 'negative'}")
            if _m_connected:
                _errs = []
                if shared_data.motor['motor_temperature_error'][_i]: _errs.append("TEMP")
                if shared_data.motor['motor_encoder_error'][_i]:     _errs.append("ENC")
                if shared_data.motor['motor_vbus_error'][_i]:        _errs.append("VBUS")
                if shared_data.motor['motor_driver_error'][_i]:      _errs.append("DRV")
                if shared_data.motor['motor_velocity_error'][_i]:    _errs.append("VEL")
                if shared_data.motor['motor_current_error'][_i]:     _errs.append("CUR")
                if shared_data.motor['motor_estop_error'][_i]:       _errs.append("ESTOP")
                if shared_data.motor['motor_watchdog_error'][_i]:    _errs.append("WD")
                if _errs:
                    motor_err_badges[_i].text = "ERR: " + ", ".join(_errs)
                    motor_err_badges[_i].props("color=negative")
                else:
                    motor_err_badges[_i].text = "NO ERRORS"
                    motor_err_badges[_i].props("color=positive outline")
                motor_err_badges[_i].set_visibility(True)
            else:
                motor_err_badges[_i].set_visibility(False)

    # ── Gripper tab ───────────────────────────────────────────
    if _tab == 'Gripper':
        gripper_table.rows = [{
            'pos':      int(shared_data.gripper['gripper_position'][0]),
            'cur':      int(shared_data.gripper['gripper_current'][0]),
            'act':      int(shared_data.gripper['gripper_activated'][0]),
            'action':   int(shared_data.gripper['gripper_action'][0]),
            'obj':      int(shared_data.gripper['gripper_object_detection'][0]),
            'cal':      int(shared_data.gripper['gripper_calibrated'][0]),
            'temp_err': int(shared_data.gripper['gripper_temperature_error'][0]),
            'timeout':  int(shared_data.gripper['gripper_timeout_error'][0]),
            'err':      int(shared_data.gripper['gripper_error'][0]),
            'estop':    int(shared_data.gripper['gripper_estop_error'][0]),
        }]
        gripper_table.update()

        grp_motor_pos_lbl.text  = str(int(shared_data.gripper['gripper_motor_position'][0]))
        grp_motor_spd_lbl.text  = str(int(shared_data.gripper['gripper_motor_speed'][0]))
        grp_motor_cur_lbl.text  = str(int(shared_data.gripper['gripper_motor_current'][0]))
        grp_motor_temp_lbl.text = str(int(shared_data.gripper['gripper_motor_temperature'][0]))
        grp_motor_volt_lbl.text = str(int(shared_data.gripper['gripper_motor_voltage'][0]))
        grp_motor_kt_lbl.text   = (f"{shared_data.gripper['gripper_motor_kt'][0]:.4f} "
                                   f"({_fmt_kt_src(shared_data.gripper['gripper_motor_kt_source'][0])})")
        grp_motor_hw_lbl.text   = str(int(shared_data.gripper['gripper_motor_hw_version'][0]))
        grp_motor_sw_lbl.text   = str(int(shared_data.gripper['gripper_motor_sw_version'][0]))
        grp_motor_sn_lbl.text   = str(int(shared_data.gripper['gripper_motor_serial'][0]))

        grp_motor_err_table.rows = [{
            'temp':  int(shared_data.gripper['gripper_motor_temp_error'][0]),
            'enc':   int(shared_data.gripper['gripper_motor_enc_error'][0]),
            'vbus':  int(shared_data.gripper['gripper_motor_vbus_error'][0]),
            'drv':   int(shared_data.gripper['gripper_motor_driver_error'][0]),
            'vel':   int(shared_data.gripper['gripper_motor_velocity_error'][0]),
            'cur':   int(shared_data.gripper['gripper_motor_current_error'][0]),
            'estop': int(shared_data.gripper['gripper_motor_estop_error'][0]),
            'wd':    int(shared_data.gripper['gripper_motor_watchdog_error'][0]),
            'cal':   int(shared_data.gripper['gripper_motor_calibrated'][0]),
        }]
        grp_motor_err_table.update()
        grp_pos_actual_lbl.text = fmt(shared_data.gripper['gripper_joint_position'][0], 4)
        grp_spd_actual_lbl.text = fmt(shared_data.gripper['gripper_joint_speed'][0], 4)
        grp_frc_actual_lbl.text = fmt(shared_data.gripper['gripper_joint_force'][0])
        grp_pos_target_lbl.text = fmt(shared_data.gripper['gripper_joint_position_target'][0], 4)
        grp_spd_target_lbl.text = fmt(shared_data.gripper['gripper_joint_speed_target'][0], 4)
        grp_frc_target_lbl.text = fmt(shared_data.gripper['gripper_joint_force_target'][0])
        grp_pos_cmd_lbl.text    = fmt(shared_data.gripper['gripper_joint_position_commanded'][0], 4)
        grp_spd_cmd_lbl.text    = fmt(shared_data.gripper['gripper_joint_speed_commanded'][0], 4)
        grp_frc_cmd_lbl.text    = fmt(shared_data.gripper['gripper_joint_force_commanded'][0])

        grp_connected = bool(shared_data.rti['CAN_gripper_connected'][0])
        gripper_conn_badge.text = f"GRIPPER: {'CONNECTED' if grp_connected else 'DISCONNECTED'}"
        gripper_conn_badge.props(f"color={'positive' if grp_connected else 'negative'}")

        _grp_ctrl = int(shared_data.rti['gripper_ctrl_mode'][0])
        if _grp_ctrl == 1:
            gripper_mode_badge.text = "MOTOR MODE"
            gripper_mode_badge.props("color=blue outline")
        elif _grp_ctrl == 2:
            gripper_mode_badge.text = "FIRMWARE MODE"
            gripper_mode_badge.props("color=purple outline")
        else:
            gripper_mode_badge.text = "MODE: ---"
            gripper_mode_badge.props("color=grey outline")

        # Reflect current ctrl_mode in the toggle without re-firing on_change
        _expected_toggle = 'firmware' if _grp_ctrl == 2 else 'motor'
        if gripper_ctrl_mode_toggle.value != _expected_toggle:
            gripper_ctrl_mode_toggle.set_value(_expected_toggle)

        # Gripper command status text — derives motion / object detection from
        # gripper_object_detection (the gripper's codes 0–3), gripper_action, and the
        # last commanded direction. ctrl_mode != 2 means firmware path is off.
        _obj    = int(shared_data.gripper['gripper_object_detection'][0])
        _action = int(shared_data.gripper['gripper_action'][0])
        _last_dir = _gripper_cmd_state['last_dir']  # +1 closing, -1 opening
        if _grp_ctrl != 2:
            gripper_status_label.text = "FIRMWARE PATH OFF"
            gripper_status_label.classes(replace="text-sm font-mono font-bold text-gray-500")
        elif _obj == 1:
            gripper_status_label.text = "OBJECT DETECTED (opening)"
            gripper_status_label.classes(replace="text-sm font-mono font-bold text-orange-600")
        elif _obj == 2:
            gripper_status_label.text = "OBJECT DETECTED (closing)"
            gripper_status_label.classes(replace="text-sm font-mono font-bold text-orange-600")
        elif _obj == 3:
            gripper_status_label.text = "AT POSITION"
            gripper_status_label.classes(replace="text-sm font-mono font-bold text-green-600")
        elif _action:
            if _last_dir > 0:
                gripper_status_label.text = "GRIPPER CLOSING"
            elif _last_dir < 0:
                gripper_status_label.text = "GRIPPER OPENING"
            else:
                gripper_status_label.text = "GRIPPER MOVING"
            gripper_status_label.classes(replace="text-sm font-mono font-bold text-blue-600")
        else:
            gripper_status_label.text = "IDLE"
            gripper_status_label.classes(replace="text-sm font-mono font-bold text-gray-700")

        if grp_connected:
            grp_err_list = []
            if shared_data.gripper['gripper_timeout_error'][0]:  grp_err_list.append("TIMEOUT")
            if shared_data.gripper['gripper_estop_error'][0]:    grp_err_list.append("ESTOP")
            if shared_data.gripper['gripper_error'][0]:          grp_err_list.append("FAULT")
            if grp_err_list:
                gripper_err_badge.text = "ERR: " + ", ".join(grp_err_list)
                gripper_err_badge.props("color=negative")
            else:
                gripper_err_badge.text = "NO ERRORS"
                gripper_err_badge.props("color=positive outline")
            gripper_err_badge.set_visibility(True)
        else:
            gripper_err_badge.set_visibility(False)

    # ── RT Diagnostics tab ────────────────────────────────────
    if _tab == 'RT Diagnostics':
        ts_label.text                = f"Timestamp: {shared_data.rti['timestamp'][0]:12.6f}"
        tick_label.text              = f"Tick: {int(shared_data.rti['loop_tick'][0])}"
        gripper_connected_label.text = f"Gripper: {'YES' if shared_data.rti['CAN_gripper_connected'][0] else 'NO'}"
        hall_index_label.text        = f"Hall: {int(shared_data.motor['HALL_INDEX'][0])}"
        additional_pin2_label.text   = f"Pin2: {int(shared_data.motor['ADDITIONAL_PIN2'][0])}"
        edge_detect_label.text       = f"Edge: {int(shared_data.motor['HALL_EDGE_DETECT'][0])}"
        safety_violation_label.text  = f"Safety violation: {'YES' if safety_violation else 'NO'}"
        error_active_label.text      = f"Error active: {err_val}"

        can_error_nodes = int((shared_data.rti['CAN_node_connection_error'][:] > 0).sum())
        can_old_data_flags = int((shared_data.rti['old_data_warning'][:] > 0).sum())
        can_error_nodes_label.text = f"CAN nodes in error: {can_error_nodes}"
        can_old_data_label.text    = f"CAN old-data flags: {can_old_data_flags}"

        period_cur_label.text   = f"{shared_data.rti['loop_period'][0]*1e3:.3f}"
        period_avg_label.text   = f"{shared_data.rti['avg_loop_duration'][0]*1e3:.3f}"
        period_p95_label.text   = f"{shared_data.rti['p95_loop_period'][0]*1e3:.3f}"
        period_p99_label.text   = f"{shared_data.rti['p99_loop_period'][0]*1e3:.3f}"
        period_peak_label.text  = f"{shared_data.rti['max_loop_period'][0]*1e3:.3f}"
        _rti_period     = shared_data.rti['loop_period'][0]
        _rti_period_ema = shared_data.rti['avg_loop_duration'][0]
        rate_cur_label.text     = f"{1.0 / _rti_period:.1f}" if _rti_period > 0 else "—"
        rate_avg_label.text     = f"{1.0 / _rti_period_ema:.1f}" if _rti_period_ema > 0 else "—"
        exec_cur_label.text     = f"{shared_data.rti['loop_execution_time'][0]*1e3:.3f}"
        exec_avg_label.text     = f"{shared_data.rti['avg_loop_execution_time'][0]*1e3:.3f}"
        exec_p95_label.text     = f"{shared_data.rti['p95_loop_execution_time'][0]*1e3:.3f}"
        exec_p99_label.text     = f"{shared_data.rti['p99_loop_execution_time'][0]*1e3:.3f}"
        exec_peak_label.text    = f"{shared_data.rti['max_loop_execution_time'][0]*1e3:.3f}"
        jitter_cur_label.text   = f"{shared_data.rti['jitter'][0]*1e6:.1f}"
        jitter_avg_label.text   = f"{shared_data.rti['avg_jitter'][0]*1e6:.1f}"
        jitter_peak_label.text  = f"{shared_data.rti['max_jitter'][0]*1e6:.1f}"
        overrun_label.text      = f"{int(shared_data.rti['overrun_count'][0])}"
        _cage = shared_data.rti['can_frame_age'][0] * 1e3
        _cage_min = shared_data.rti['can_frame_age_min'][0] * 1e3
        can_age_label.text      = f"{_cage:.2f}"
        # "EMA avg" column is reused as the per-tick MINIMUM here: min ~= max means every
        # frame is stale (a real backlog); min small with max large means one frame class
        # is old and the bus view is otherwise current.
        can_age_min_label.text  = f"min {_cage_min:.2f}"
        can_age_peak_label.text = f"{shared_data.rti['can_frame_age_peak'][0]*1e3:.2f}"
        loop_delay_order_label.text = ("order: option B (read first)" if shared_data.rti['can_read_first'][0] > 0.5
                                       else "order: normal")
        # Colour the live value: green = current, amber = drifting, red = acting on the past.
        can_age_label.classes(replace='font-mono ' + (
            'text-red-500' if _cage > 50 else 'text-orange-500' if _cage > 15 else 'text-green-500'))

        # ── RTI mode panel ────────────────────────────────────
        # substate: 0 unpaired · 1 connected · 2 control_active · 3 stopping_clean
        #           4 stopping_error   (motion/rti_handler.py SUBSTATE_CODE)
        _rti_sub = int(shared_data.rti['rti_substate'][0])
        _sub_text, _sub_colour = _RTI_SUBSTATE_UI.get(_rti_sub, ("UNKNOWN", "grey"))
        rti_substate_badge.text = _sub_text
        rti_substate_badge.props(f"color={_sub_colour} outline")

        # Show the miss counters whenever they can mean something: while control is claimed
        # (live) and after an error stop (which is exactly when you want to see how the link
        # died). They are meaningless only before the first claim.
        _claimed = _rti_sub == 2
        _show_misses = _rti_sub in (2, 3, 4)
        _misses  = int(shared_data.rti['rti_consecutive_misses'][0])
        _worst   = int(shared_data.rti['rti_worst_consecutive_misses'][0])
        # Budget in ticks. Prefer the MEASURED loop period so it stays honest if the robot
        # is running off-nominal; fall back to the configured delta_t when RTI is stopped
        # (measured period is 0 then, and "5/0 ticks" would read as a zero budget).
        # COMMAND_TIMEOUT_S is imported, never copied — see the import comment.
        _tick_s = _rti_period if _rti_period > 0 else (_xml_cfg.get('delta_t') or 0.0)
        _budget_ticks = max(1, int(round(COMMAND_TIMEOUT_S / _tick_s))) if _tick_s > 0 else 0

        # Bandwidth. Meaningful from pairing onward — state streams before control is
        # claimed — so it is gated on "paired", not on "claimed".
        _tx = shared_data.rti['rti_tx_bytes_per_s'][0]
        _rx = shared_data.rti['rti_rx_bytes_per_s'][0]
        _disc = shared_data.rti['rti_discard_pct'][0]
        if _rti_sub == 0:
            rti_bw_tx_label.text    = "TX: —"
            rti_bw_rx_label.text    = "RX: —"
            rti_bw_total_label.text = "Total: —"
            rti_discard_label.text  = "Discarded: —"
        else:
            rti_bw_tx_label.text    = f"TX: {_tx/1024:.1f} kB/s"
            rti_bw_rx_label.text    = f"RX: {_rx/1024:.1f} kB/s"
            # Mbit/s alongside kB/s because link capacity is quoted in bits, and that is
            # the comparison that matters the moment WiFi is involved.
            rti_bw_total_label.text = (f"Total: {(_tx+_rx)/1024:.1f} kB/s "
                                       f"({(_tx+_rx)*8/1e6:.2f} Mbit/s)")
            rti_discard_label.text  = f"Discarded: {_disc:.1f}% of commands"
        for _l in (rti_bw_tx_label, rti_bw_rx_label, rti_bw_total_label):
            _l.classes(replace='font-mono text-sm '
                       + ('text-gray-400' if _rti_sub == 0 else 'text-gray-600'))
        # Any discard at all is worth seeing — it is silent in every other metric.
        rti_discard_label.classes(replace='font-mono text-sm ' + (
            'text-gray-400' if _rti_sub == 0 else
            'text-red-500' if _disc > 25 else
            'text-orange-500' if _disc > 1 else 'text-gray-600'))

        if _rti_sub == 0:
            # Nothing paired: blank the numbers rather than showing a stale last session.
            rti_success_label.text      = "Success: —"
            rti_misses_label.text       = "Misses: —"
            rti_worst_misses_label.text = "Worst run: —"
            for _l in (rti_success_label, rti_misses_label, rti_worst_misses_label):
                _l.classes(replace='font-mono text-sm text-gray-400')
        else:
            _sr = shared_data.rti['control_success_rate'][0]
            rti_success_label.text = f"Success: {_sr*100:.1f}% (last {SUCCESS_WINDOW_TICKS} ticks)"
            # Only colour the success rate while control is actually claimed — it is
            # meaningless before the first command, and it is a duty cycle rather than a
            # loss rate in PC-clocked mode.
            rti_success_label.classes(replace='font-mono text-sm ' + (
                ('text-red-500' if _sr < SUCCESS_BAD else
                 'text-orange-500' if _sr < SUCCESS_WARN else 'text-green-500')
                if _claimed else 'text-gray-400'))

            # Misses is the signal with teeth: show it against the budget that stops control.
            # Omit the "/budget" rather than print a fake 0 when the tick rate is unknown.
            if not _show_misses:
                rti_misses_label.text = "Misses: — (control not claimed)"
            elif _budget_ticks:
                rti_misses_label.text = f"Misses: {_misses}/{_budget_ticks} ticks"
            else:
                rti_misses_label.text = f"Misses: {_misses} ticks"
            rti_misses_label.classes(replace='font-mono text-sm ' + (
                ('text-red-500' if _budget_ticks and _misses >= _budget_ticks else
                 'text-orange-500' if _budget_ticks and _misses >= _budget_ticks // 2 else
                 'text-green-500')
                if _show_misses else 'text-gray-400'))

            # Peak of this claim — how close the link came to stopping. Amber once it has
            # been past half the budget even if it is fine now, because that is the thing
            # a live reading hides.
            rti_worst_misses_label.text = f"Worst run: {_worst} ticks"
            rti_worst_misses_label.classes(replace='font-mono text-sm ' + (
                'text-orange-500' if _budget_ticks and _worst >= _budget_ticks // 2
                else 'text-gray-500'))

        rti_table.rows = [{
            'node':     f'N{i+1}',
            'scan':     int(shared_data.rti["scan_received_ids"][i]),
            'err_cnt':  int(shared_data.rti["Error_counter"][i]),
            'old_data': int(shared_data.rti["old_data_warning"][i]),
            'conn_err': int(shared_data.rti["CAN_node_connection_error"][i]),
        } for i in range(16)]
        rti_table.update()
        received_ids_label.text       = f"Received IDs:    {shared_data.rti['received_ids'][:].tolist()}"
        latest_commands_id_label.text = f"Latest Commands: {shared_data.rti['latest_commands_id'][:].tolist()}"

        gui_period_cur_label.text  = f"{_gui_period:.2f}"
        gui_period_avg_label.text  = f"{_gui_period_ema:.2f}"
        gui_period_peak_label.text = f"{_gui_period_peak:.2f}"
        gui_exec_cur_label.text    = f"{_gui_exec_last:.2f}"
        gui_exec_avg_label.text    = f"{_gui_exec_ema:.2f}"
        gui_exec_peak_label.text   = f"{_gui_exec_peak:.2f}"
        gui_fps_cur_label.text     = f"{1000.0 / _gui_period:.1f}" if _gui_period > 0 else "—"
        gui_fps_avg_label.text     = f"{1000.0 / _gui_period_ema:.1f}" if _gui_period_ema > 0 else "—"

    # ── Power & CB tab ────────────────────────────────────────
    if _tab == 'Power & CB':
        cb_vbus1_label.text     = f"{int(shared_data.control_board['cb_filtered_VBUS1_mV'][0])} mV"
        cb_vbus2_label.text     = f"{int(shared_data.control_board['cb_filtered_VBUS2_mV'][0])} mV"
        cb_current_label.text   = f"{int(shared_data.control_board['cb_filtered_Current_mA'][0])} mA"
        cb_power_label.text     = f"{shared_data.control_board['cb_power_usage'][0]:.1f} W"
        cb_relay_label.text     = str(int(shared_data.control_board['cb_relayState'][0]))
        cb_precharge_label.text = str(int(shared_data.control_board['cb_PrechargeState'][0]))
        cb_brake_label.text     = str(int(shared_data.control_board['cb_BrakeResistorState'][0]))
        cb_error_label.text     = str(int(shared_data.control_board['cb_error_state'][0]))
        cb_hw_label.text        = str(int(shared_data.control_board['cb_HARDWARE_VERSION'][0]))
        cb_sw_label.text        = str(int(shared_data.control_board['cb_SOFTWARE_VERSION'][0]))
        cb_batch_label.text     = str(int(shared_data.control_board['cb_BATCH_DATE'][0]))
        cb_sn_label.text        = str(int(shared_data.control_board['cb_SERIAL_NUMBER'][0]))

    # ── I/O tab ───────────────────────────────────────────────
    if _tab == 'I/O':
        from data.shared_memory_index import (
            ISOLATED_INPUT_1, ISOLATED_INPUT_2, ISOLATED_INPUT_3,
            ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3,
            ESTOP_1, ESTOP_2, GPIO_1, GPIO_2, GPIO_3, GPIO_4,
            LED1, LED2, BUTTON1, BUTTON2,
        )

        def _io_badge(badge, value, high_label="HIGH", low_label="LOW",
                      high_color="positive", low_color="grey"):
            badge.text = high_label if value else low_label
            badge.props(f"color={high_color if value else low_color} outline")

        _io_badge(io_in1_badge,   shared_data.gpio_states[ISOLATED_INPUT_1])
        _io_badge(io_in2_badge,   shared_data.gpio_states[ISOLATED_INPUT_2])
        _io_badge(io_in3_badge,   shared_data.gpio_states[ISOLATED_INPUT_3])

        # E-stops: active LOW — value=0 means TRIGGERED
        _io_badge(io_estop1_badge, shared_data.gpio_states[ESTOP_1],
                  high_label="OK", low_label="TRIGGERED",
                  high_color="positive", low_color="negative")
        _io_badge(io_estop2_badge, shared_data.gpio_states[ESTOP_2],
                  high_label="OK", low_label="TRIGGERED",
                  high_color="positive", low_color="negative")

        _io_badge(io_out1_badge,  shared_data.gpio_states[ISOLATED_OUTPUT_1])
        _io_badge(io_out2_badge,  shared_data.gpio_states[ISOLATED_OUTPUT_2])
        _io_badge(io_out3_badge,  shared_data.gpio_states[ISOLATED_OUTPUT_3])

        _io_badge(io_gpio1_badge, shared_data.gpio_states[GPIO_1])
        _io_badge(io_gpio2_badge, shared_data.gpio_states[GPIO_2])
        _io_badge(io_gpio3_badge, shared_data.gpio_states[GPIO_3])
        _io_badge(io_gpio4_badge, shared_data.gpio_states[GPIO_4])

        # Buttons: active LOW (pull-up), value=0 means PRESSED
        _io_badge(io_btn1_badge, not shared_data.gpio_states[BUTTON1],
                  high_label="PRESSED", low_label="OFF",
                  high_color="positive", low_color="grey")
        _io_badge(io_btn2_badge, not shared_data.gpio_states[BUTTON2],
                  high_label="PRESSED", low_label="OFF",
                  high_color="positive", low_color="grey")

        if refresh_hall_card is not None:
            refresh_hall_card()

        _io_badge(io_led1_badge, shared_data.gpio_states[LED1],
                  high_label="ON", low_label="OFF",
                  high_color="warning", low_color="grey")
        _io_badge(io_led2_badge, shared_data.gpio_states[LED2],
                  high_label="ON", low_label="OFF",
                  high_color="warning", low_color="grey")

    # ── System Info tab ───────────────────────────────────────
    if _tab == 'System Info':
        for name, lbl in [
            ("pi_model",         sysinfo_pi_model_lbl),
            ("platform_release", sysinfo_release_lbl),
            ("kernel_version",   sysinfo_kernel_lbl),
            ("distro_name",      sysinfo_distro_lbl),
            ("distro_version",   sysinfo_distro_ver_lbl),
            ("cpu_processor",    sysinfo_cpu_proc_lbl),
            ("cpu_architecture", sysinfo_cpu_arch_lbl),
            ("cpu_count",        sysinfo_cpu_cnt_lbl),
            ("memory_size",      sysinfo_mem_size_lbl),
            ("memory_used",      sysinfo_mem_used_lbl),
            ("memory_usage",     sysinfo_mem_usage_lbl),
            ("disk_device",      sysinfo_disk_device_lbl),
            ("disk_size",        sysinfo_disk_size_lbl),
            ("disk_used",        sysinfo_disk_used_lbl),
            ("disk_usage",       sysinfo_disk_usage_lbl),
        ]:
            lbl.text = shared_data.system_info_shms[name].get() or "---"

        sysinfo_cb_cmd_lbl.text  = shared_data.cb_command_.get() or "---"
        sysinfo_cb_stat_lbl.text = shared_data.cb_status.get() or "---"
        sysinfo_cb_err_lbl.text  = shared_data.cb_current_error.get() or "---"

    # ── PID Config tab ────────────────────────────────────────
    if _tab == 'PID Config':
        pid_table.rows = [{
            'joint':   f'J{i+1}',
            'KPP':     f"{shared_data.motor_config['cfg_KPP'][i]:.4f}",
            'KPV':     f"{shared_data.motor_config['cfg_KPV'][i]:.6f}",
            'KIV':     f"{shared_data.motor_config['cfg_KIV'][i]:.6f}",
            'KPIQ':    f"{shared_data.motor_config['cfg_KPIQ'][i]:.6f}",
            'KIIQ':    f"{shared_data.motor_config['cfg_KIIQ'][i]:.6f}",
            'KP':      f"{shared_data.motor_config['cfg_KP'][i]:.4f}",
            'KD':      f"{shared_data.motor_config['cfg_KD'][i]:.6f}",
            'Ilim':    f"{shared_data.motor_config['cfg_Ilim'][i]:.0f}",
            'vel_lim': f"{shared_data.motor_config['cfg_velocity_limit'][i]:.0f}",
            'vlim':    _fmt_vlim(shared_data.motor_config['cfg_voltage_limit'][i]),
        } for i in range(JOINTS)]
        pid_table.update()

        gripper_pid_table.rows = [{
            'joint':   'Gripper',
            'KPP':     f"{shared_data.motor_config['cfg_gripper_KPP'][0]:.4f}",
            'KPV':     f"{shared_data.motor_config['cfg_gripper_KPV'][0]:.6f}",
            'KIV':     f"{shared_data.motor_config['cfg_gripper_KIV'][0]:.6f}",
            'KPIQ':    f"{shared_data.motor_config['cfg_gripper_KPIQ'][0]:.6f}",
            'KIIQ':    f"{shared_data.motor_config['cfg_gripper_KIIQ'][0]:.6f}",
            'KP':      f"{shared_data.motor_config['cfg_gripper_KP'][0]:.4f}",
            'KD':      f"{shared_data.motor_config['cfg_gripper_KD'][0]:.6f}",
            'Ilim':    f"{shared_data.motor_config['cfg_gripper_Ilim'][0]:.0f}",
            'vel_lim': f"{shared_data.motor_config['cfg_gripper_velocity_limit'][0]:.0f}",
            'vlim':    _fmt_vlim(shared_data.motor_config['cfg_gripper_voltage_limit'][0]),
        }]
        gripper_pid_table.update()

    # ── Errors tab ────────────────────────────────────────────
    if _tab == 'Errors':
        error_data = shared_data.error_state.get_json()
        errors_container.clear()
        with errors_container:
            if error_data in (None, "NONE", []):
                ui.label("No active errors.").classes("text-positive")
            else:
                # One block per latched key (2026-10-15, the message standard): number, code and title; where; when;
                # then the catalog's cause / effect / fix (gui_activity_tab.error_view; utility/error_catalog.py).
                from gui_activity_tab import error_view
                for err in error_data:
                    v = error_view(err, display_time.fmt_precise)
                    color = "text-warning" if v["level"] == "warning" else "text-negative"
                    with ui.column().classes("gap-0 mb-2"):
                        ui.label(f"{v['level'].upper()}  {v['code']}  {v['title']}"
                                 + (f"  ({v['where']})" if v['where'] else "")).classes(f"{color} font-semibold")
                        ui.label(f"{v['time']}  tick {v['tick']}  {v['description']}").classes("text-xs text-gray-500")
                        for name in ("cause", "effect", "fix"):
                            if v[name]:
                                ui.label(f"{name.capitalize()}: {v[name]}").classes("text-xs whitespace-normal")

    # ── GUI exec time (measured after all tab updates) ────────
    _gui_exec      = (time.monotonic() - _t_start) * 1e3
    _gui_exec_last = _gui_exec
    _gui_exec_ema  = _GUI_EMA_ALPHA * _gui_exec  + (1 - _GUI_EMA_ALPHA) * _gui_exec_ema
    _gui_exec_peak = max(_gui_exec_peak, _gui_exec)


# ─────────────────────────────────────────────────────────────
# LIVE PLOTS  (own timer, PLOT_REFRESH_HZ — independent of the tables)
# ─────────────────────────────────────────────────────────────
def _plot_yaxis(key, datasets, win_min):
    """Y-lock: scale to visible selected traces in the current window.
    Auto: return (None, None) so ECharts autoscales."""
    if not _plot_ylock[key][0] or not datasets:
        return None, None
    _sel = _plot_sel[key]
    _ticks = list(_plot_buf_tick)
    lo, hi = None, None
    for data in datasets:
        for i in range(JOINTS):
            if not _sel[i]:
                continue
            _series_len = len(data[i])
            _off = len(_ticks) - _series_len
            for j, v in enumerate(data[i]):
                _tick_idx = _off + j
                if _tick_idx < 0 or _tick_idx >= len(_ticks) or _ticks[_tick_idx] < win_min:
                    continue
                lo = v if lo is None else min(lo, v)
                hi = v if hi is None else max(hi, v)
    if lo is None:
        return None, None
    pad = max(1, (hi - lo) * 0.05)
    return lo - pad, hi + pad


def refresh_plots():
    """Sample data into rolling buffers and update the live charts.
    Runs on its own PLOT_REFRESH_HZ timer so plot smoothness is decoupled from
    the (slower) table refresh."""
    global _plot_last_t, _plot_period_ema

    # Measure the real plot-loop period for the FPS readout.
    _t_now = time.monotonic()
    if _plot_last_t:
        _per = (_t_now - _plot_last_t) * 1e3
        _plot_period_ema = _GUI_EMA_ALPHA * _per + (1 - _GUI_EMA_ALPHA) * _plot_period_ema
    _plot_last_t = _t_now

    # Always buffer — even when the tab is hidden — so history stays continuous.
    # Sampled in the currently-selected units (motor ticks/mA or joint rad/Nm).
    _cur_plot_tick = int(shared_data.rti["loop_tick"][0])
    _plot_buf_tick.append(_cur_plot_tick)
    _unit = _plot_unit[0]
    if _unit == 'motor':
        for i in range(JOINTS):
            _plot_pos[i].append(_plot_read('pos', i))
            _plot_spd[i].append(_plot_read('spd', i))
            _plot_cur[i].append(_plot_read('cur', i))
            # "commanded" in motor units = what actually went on the CAN wire.
            _plot_pos_cmd[i].append(_plot_read_cmd('pos', i))
            _plot_spd_cmd[i].append(_plot_read_cmd('spd', i))
            _plot_cur_cmd[i].append(_plot_read_cmd('cur', i))
    elif _unit == 'joint':
        for i in range(JOINTS):
            _plot_pos[i].append(_plot_read('pos', i))
            _plot_spd[i].append(_plot_read('spd', i))
            _plot_acc[i].append(_plot_read('acc', i))
            _plot_cur[i].append(_plot_read('cur', i))
            _plot_pos_cmd[i].append(_plot_read_cmd('pos', i))
            _plot_spd_cmd[i].append(_plot_read_cmd('spd', i))
            _plot_acc_cmd[i].append(_plot_read_cmd('acc', i))
            _plot_cur_cmd[i].append(_plot_read_cmd('cur', i))
            _plot_pos_tgt[i].append(_plot_read_tgt('pos', i))
            _plot_spd_tgt[i].append(_plot_read_tgt('spd', i))
            _plot_acc_tgt[i].append(_plot_read_tgt('acc', i))
            _plot_cur_tgt[i].append(_plot_read_tgt('cur', i))
            _plot_spd_filt[i].append(_plot_read_filt('spd', i))
            _plot_acc_filt[i].append(_plot_read_filt('acc', i))
            _plot_cur_filt[i].append(_plot_read_filt('cur', i))
            _plot_jerk[i].append(_plot_read('jerk', i))
            _plot_jerk_filt[i].append(_plot_read_filt('jerk', i))
            _plot_etrq[i].append(_plot_read('etrq', i))
    else:
        # Actual acceleration comes from RTI; commanded/target stay GUI-derived.
        _prev_tick = _tcp_prev_tick[0]
        _dt = ((_cur_plot_tick - _prev_tick) * _DELTA_T
               if _prev_tick is not None and _cur_plot_tick > _prev_tick
               else PLOT_REFRESH_S)
        if _dt <= 0:
            _dt = PLOT_REFRESH_S
        for i in range(JOINTS):
            _spd = _plot_read('spd', i)
            _spd_cmd = _plot_read_cmd('spd', i)
            _spd_tgt = _plot_read_tgt('spd', i)
            # 'actual' TCP accel = the RAW Cartesian accel, so the filtered dash-dot overlay
            # adds information (raw vs filtered). filtered comes from _plot_read_filt below.
            _acc = _plot_ang('acc', i, float(shared_data.tcp['TCP_acceleration_raw'][i]))
            if _tcp_prev_valid[0]:
                _acc_cmd = (_spd_cmd - _tcp_prev_speed['cmd'][i]) / _dt
                _acc_tgt = (_spd_tgt - _tcp_prev_speed['tgt'][i]) / _dt
            else:
                _acc_cmd = _acc_tgt = 0.0
            _plot_pos[i].append(_plot_read('pos', i))
            _plot_spd[i].append(_spd)
            _plot_acc[i].append(round(float(_acc), 4))
            _plot_pos_cmd[i].append(_plot_read_cmd('pos', i))
            _plot_spd_cmd[i].append(_spd_cmd)
            _plot_acc_cmd[i].append(round(float(_acc_cmd), 4))
            _plot_pos_tgt[i].append(_plot_read_tgt('pos', i))
            _plot_spd_tgt[i].append(_spd_tgt)
            _plot_acc_tgt[i].append(round(float(_acc_tgt), 4))
            _plot_spd_filt[i].append(_plot_read_filt('spd', i))
            _plot_acc_filt[i].append(_plot_read_filt('acc', i))
            _plot_fext[i].append(_plot_read('fext', i))
            _plot_ftool[i].append(_plot_read('ftool', i))
            _tcp_prev_speed['actual'][i] = _spd
            _tcp_prev_speed['cmd'][i] = _spd_cmd
            _tcp_prev_speed['tgt'][i] = _spd_tgt
        _tcp_prev_tick[0] = _cur_plot_tick
        _tcp_prev_valid[0] = True

    # Detect mode / error / E-stop transitions and log them for vertical markers.
    _m = shared_data.robot_mode.get()
    _e = int(shared_data.rti['error_active'][0])
    _es = int(shared_data.gpio_states[_ESTOP_1]) == 0 or bool(shared_data.rti['software_estop'][0])
    if _evt_prev['mode'] is not None and _m != _evt_prev['mode']:
        _plot_events.append((_cur_plot_tick, _m, '#5470c6'))           # mode -> blue
    if _evt_prev['err'] is not None and _e and not _evt_prev['err']:
        _plot_events.append((_cur_plot_tick, 'ERROR', '#ee6666'))      # error -> red
    if _evt_prev['estop'] is not None and _es and not _evt_prev['estop']:
        _plot_events.append((_cur_plot_tick, 'E-STOP', '#d48806'))     # estop -> amber
    _evt_prev['mode'], _evt_prev['err'], _evt_prev['estop'] = _m, _e, _es

    # Only push to the browser when the Plots tab is visible and not paused.
    if _active_tab_name(tabs.value) != 'Plots':
        _plot_prev_visible[0] = False
        return
    _plot_prev_visible[0] = True

    plot_fps_label.text = (f"FPS: {1000.0 / _plot_period_ema:.1f}"
                           if _plot_period_ema > 0 else "FPS: —")
    if _plot_paused[0]:
        return

    _win_min = max(0, _cur_plot_tick - _plot_win[0])
    _win_max = _cur_plot_tick

    # Push the WHOLE windowed slice each frame via a single setOption merge.
    # We deliberately do NOT use appendData: appendData streams into a fixed
    # coordinate system, and changing xAxis min/max on the same frame forces
    # ECharts to re-render the canvas → the flicker that scaled with frame rate.
    # Replacing series.data on a merge scrolls smoothly (animation is off) and
    # keeps the browser-side series naturally bounded — no separate trim needed.
    _times = list(_plot_buf_tick)
    # index of the first sample still inside the window (drop older ones)
    _start = 0
    for _start in range(len(_times)):
        if _times[_start] >= _win_min:
            break

    _actual_on = _plot_show_actual[0]
    # Commanded is meaningful in motor units too now: there it is the CAN-sent value.
    _cmd_on = _plot_show_cmd[0] and _plot_unit[0] in ('motor', 'joint', 'tcp')
    _tgt_on = _plot_show_tgt[0] and _plot_unit[0] in ('joint', 'tcp')
    _filt_on = _plot_show_filt[0] and _plot_unit[0] in ('joint', 'tcp')

    # Event markLines for the carrier series: only those inside the window.
    if _plot_show_events[0]:
        _ml = [
            {'xAxis': t, 'lineStyle': {'color': c, 'type': 'dashed', 'width': 1},
             'label': {'formatter': lbl, 'fontSize': 9, 'color': c, 'rotate': 90,
                       'position': 'insideEndTop'}}
            for (t, lbl, c) in _plot_events if t >= _win_min
        ]
    else:
        _ml = []

    # (chart, measured, commanded, target, filtered, key). None where a channel does not
    # exist for that signal: jerk has no commanded/target; etrq/fext/ftool have neither those
    # nor a filtered twin (they are already derived from filtered torque).
    for _chart, _data, _cmd, _tgt, _filt, _key in [
        (pos_chart,   _plot_pos,   _plot_pos_cmd, _plot_pos_tgt, None,            'pos'),
        (spd_chart,   _plot_spd,   _plot_spd_cmd, _plot_spd_tgt, _plot_spd_filt,  'spd'),
        (acc_chart,   _plot_acc,   _plot_acc_cmd, _plot_acc_tgt, _plot_acc_filt,  'acc'),
        (cur_chart,   _plot_cur,   _plot_cur_cmd, _plot_cur_tgt, _plot_cur_filt,  'cur'),
        (jerk_chart,  _plot_jerk,  None,          None,          _plot_jerk_filt, 'jerk'),
        (etrq_chart,  _plot_etrq,  None,          None,          None,            'etrq'),
        (fext_chart,  _plot_fext,  None,          None,          None,            'fext'),
        (ftool_chart, _plot_ftool, None,          None,          None,            'ftool'),
    ]:
        if _key not in _keys_for_unit():
            continue
        if _plot_collapsed[_key][0]:
            continue   # collapsed chart is hidden — skip its push entirely
        # Gate the PUSH by selection: unselected joints send an empty series
        # (~0 bytes) instead of the full window. This is what makes plotting
        # fewer joints actually cut CPU/network load — not just hide them.
        _sel = _plot_sel[_key]
        # measured (solid) [0..JOINTS-1]
        _series = [
            {'data': [[_times[j], _data[i][j]] for j in range(_start, len(_times))]
                     if _actual_on and _sel[i] else []}
            for i in range(JOINTS)
        ]
        # commanded (dashed) [JOINTS..2*JOINTS-1] - only when overlay on; empty
        # otherwise. Buffers may be shorter than _times, so align from the tail.
        # An overlay that is ON but carries only None (channel not transmitted) draws nothing,
        # which is indistinguishable from the overlay being off. Say so explicitly instead.
        if _cmd_on and _cmd is not None and all(len(_cmd[i]) and _cmd[i][-1] is None
                                                for i in range(JOINTS)):
            _cmd_note_lbls[_key].text = ('· CAN Sent: channel not commanded'
                                         if _plot_unit[0] == 'motor' else '· no commanded data')
        else:
            _cmd_note_lbls[_key].text = ''

        for i in range(JOINTS):
            if _cmd_on and _cmd is not None and _sel[i] and len(_cmd[i]):
                _cd = list(_cmd[i])
                _ct = _times[len(_times) - len(_cd):]
                # None = channel not commanded that tick (motor units, unset CAN channel).
                _series.append({'data': [[_ct[j], _cd[j]] for j in range(len(_cd))
                                         if _ct[j] >= _win_min and _cd[j] is not None]})
            else:
                _series.append({'data': []})
        # target (dotted) [2*JOINTS..3*JOINTS-1] - same tail alignment.
        for i in range(JOINTS):
            if _tgt_on and _tgt is not None and _sel[i] and len(_tgt[i]):
                _td = list(_tgt[i])
                _tt = _times[len(_times) - len(_td):]
                _series.append({'data': [[_tt[j], _td[j]] for j in range(len(_td))
                                         if _tt[j] >= _win_min]})
            else:
                _series.append({'data': []})
        # filtered (dash-dot) [3*JOINTS..4*JOINTS-1] - same tail alignment. _filt is None for
        # the position chart (no filtered position); those series stay empty. Filtered samples
        # can be None where a unit/key has no filtered twin — drop those points.
        for i in range(JOINTS):
            if _filt_on and _filt is not None and _sel[i] and len(_filt[i]):
                _fd = list(_filt[i])
                _ft = _times[len(_times) - len(_fd):]
                _series.append({'data': [[_ft[j], _fd[j]] for j in range(len(_fd))
                                         if _ft[j] >= _win_min and _fd[j] is not None]})
            else:
                _series.append({'data': []})
        # carrier series [4*JOINTS] holds the event markLines
        _series.append({'data': [], 'markLine': {'symbol': 'none', 'data': _ml}})

        _visible_ydata = []
        if _actual_on:
            _visible_ydata.append(_data)
        if _cmd_on:
            _visible_ydata.append(_cmd)
        if _tgt_on:
            _visible_ydata.append(_tgt)
        _ymin, _ymax = _plot_yaxis(_key, _visible_ydata, _win_min)
        _chart.run_chart_method('setOption', {
            'series': _series,
            'xAxis': {'min': _win_min, 'max': _win_max},
            'yAxis': {'min': _ymin, 'max': _ymax},
        })

    if _plot_unit[0] == 'tcp':
        _actual_path = _path_data(_plot_pos, _win_min) if _actual_on else []
        _cmd_path = _path_data(_plot_pos_cmd, _win_min) if _cmd_on else []
        _tgt_path = _path_data(_plot_pos_tgt, _win_min) if _tgt_on else []
        _xmin, _xmax, _ymin, _ymax = _path_axis_bounds([_actual_path, _cmd_path, _tgt_path])
        _path_chart.run_chart_method('setOption', {
            'series': [
                {'data': _actual_path},
                {'data': _cmd_path},
                {'data': _tgt_path},
            ],
            'xAxis': {'min': _xmin, 'max': _xmax},
            'yAxis': {'min': _ymin, 'max': _ymax},
        })


# ─────────────────────────────────────────────────────────────
# AUTO REFRESH
# ─────────────────────────────────────────────────────────────
def refresh_processes():
    """1 Hz refresh of the processes / threads / core-plan card (gui_processes.py). Reads only while a tab that shows
    it is active (System Info, or the Plots tab's TEMP diagnostic copy) — cpu_percent() wants ~1 s between samples."""
    _tab = _active_tab_name(tabs.value)
    if _tab == 'System Info':
        refresh_process_card()
    elif _tab == 'Plots':  # TEMP diagnostic copy
        process_table_diag.rows = process_rows()
        process_table_diag.update()


_plot_connected = [False]   # one-shot guard: connect the charts only once

async def _connect_plot_crosshair():
    """Link the 3 plot charts so the crosshair + tooltip sync across all of them
    (hover one -> a shared vertical line + each chart's own tooltip on all three
    at the same tick). Uses ECharts' native connect(): tag each instance with a
    shared `group` and connect it.

    MUST run while the Plots tab is active — ui.tab_panels only mounts the
    visible tab, so the chart DOM/instances don't exist until then. Called from
    the tab-change handler; the guard makes it a one-shot."""
    if _plot_connected[0]:
        return
    ok = await ui.run_javascript('''
        try {
            const mod = await import("nicegui-echart");
            const ec = mod.echarts || mod.default || mod;
            if (!ec || !ec.connect || !ec.getInstanceByDom) return false;
            // Find every mounted ECharts instance on the page (the Plots tab is
            // active now, so its 3 charts exist), group + connect them.
            const insts = Array.from(document.querySelectorAll("div"))
                .map(d => ec.getInstanceByDom(d)).filter(Boolean);
            if (insts.length < 2) return false;
            insts.forEach(c => { c.group = "plots"; });
            ec.connect("plots");
            return true;
        } catch (e) { return false; }
    ''', timeout=5.0)
    if ok:
        _plot_connected[0] = True


async def _on_connect():
    await asyncio.sleep(0.3)   # let the browser finish mounting Vue components
    update_tables()

async def _on_tab_change():
    # The Plots charts only mount when their tab is shown, so link the synced
    # crosshair the first time the user opens Plots (one-shot, guarded inside).
    if _active_tab_name(tabs.value) == 'Plots':
        await asyncio.sleep(0.25)   # let the tab's charts mount
        await _connect_plot_crosshair()

_ctl_last = [None]


def _refresh_control_state():
    """Mirror the executor's control holder into the status bar. Only touches the UI when
    something changed — a 0.5 s timer that rewrote every prop every tick would flood the
    websocket for no visible effect."""
    st = _exec_sender.control_state()
    _ctl_view.update(holder=st["holder"], mine=st["mine"], other=st["other"])
    # Hand control back once the dashboard has been idle. Never mid-jog, mid-programme or during a
    # live RTI session (auto_release_due), and only what THIS connection actually holds.
    if not st["mine"]:
        _gui_release_sent[0] = False
    elif not _gui_release_sent[0]:
        try:
            # Sub-state is only meaningful while the robot is IN an RTI mode: it is published only
            # while the RTI handler steps, so after a link-loss stop it stays at 4 (stopping_error)
            # for minutes. Reading it alone locked the dashboard's claim in place.
            rti_active = (shared_data.robot_mode.get() in ("RTI", "SOFT_RTI")
                          and float(shared_data.rti["rti_substate"][0]) >= 2.0)
        except Exception:
            rti_active = True          # cannot tell -> do not release on a guess
        if auto_release_due(st, time.monotonic() - _gui_last_cmd[0],
                            jog_active=bool(_jog_btn_pressed['active'] or _joystick_active[0]),
                            rti_active=rti_active):
            _gui_release_sent[0] = True
            _exec_sender.submit(lambda r: r.release_control(), label="release_control")
    key = (st["connected"], st["holder"], st["mine"])
    if key == _ctl_last[0]:
        return
    _ctl_last[0] = key
    if not st["connected"]:
        control_badge.text = "CONTROL: —"
        control_badge.props("color=grey outline")
    elif st["holder"] is None:
        control_badge.text = "CONTROL: free"
        control_badge.props("color=grey outline")
    elif st["mine"]:
        control_badge.text = "CONTROL: YOU"
        control_badge.props("color=positive")
    else:
        control_badge.text = f"CONTROL: {st['holder']}"
        control_badge.props("color=orange")
    _take_btn.set_visibility(st["other"])
    _release_btn.set_visibility(st["mine"])
    for b in _owned_btns:
        b.props(add="disable") if st["other"] else b.props(remove="disable")
    if st["other"]:
        _jog_card.classes(add="opacity-50")
    else:
        _jog_card.classes(remove="opacity-50")


def _drain_exec_toasts():
    """Surface command failures the background sender queued (runs in the UI context, so
    ui.notify is legal here — the sender thread cannot call it directly)."""
    while True:
        try:
            message, kind = _gui_toasts.get_nowait()
        except queue.Empty:
            break
        ui.notify(message, type=kind)


tabs.on_value_change(_on_tab_change)
app.on_connect(_on_connect)
ui.timer(TABLE_REFRESH_S, update_tables)   # tables/status — calm, readable
ui.timer(PLOT_REFRESH_S, refresh_plots)    # plots — smooth, independent rate
ui.timer(1.0, refresh_processes)           # slow panel (System Info tab)
ui.timer(0.5, refresh_firmware_tab)        # park verdict + mode badge (Firmware tab)
ui.timer(0.3, _drain_exec_toasts)          # command failures from the sender thread
ui.timer(0.5, _refresh_control_state)       # who holds control (status bar, greyed controls)
# Memory and disk move, but slowly — 5 s is plenty, and it keeps the shared strings fresh for any
# other consumer too. Platform/CPU/disk-device are published once at startup (they cannot change
# without a reboot). Both calls are cheap: one virtual_memory() and one statvfs.
def _publish_slow_system_info():
    publish_memory_info(shared_data)
    publish_disk_info(shared_data)


ui.timer(5.0, _publish_slow_system_info)

# Populate rows/labels synchronously so the very first page render already
# contains data — otherwise the default Joints & TCP tab shows as empty until
# the first websocket tick or a tab switch.
update_tables()

# Move everything allocated so far (UI tree, charts, config, shared-memory
# wrappers) into the GC's permanent generation so it never re-scans them. The
# per-frame plot garbage is then the only thing the collector has to consider.
gc.collect()
gc.freeze()

ui.run(reload=False, port=8081)
