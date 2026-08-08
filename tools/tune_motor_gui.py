"""Browser front-end for tune_motor_loops.py — set a gain, click Run, inspect the trace.

    python3 tools/tune_motor_gui.py          # then open http://<pi>:8082

  ONE motor on the bus, CAN id 0, SECURELY MOUNTED, SHAFT FREE TO SPIN.

Deliberately NOT part of nicegui_test.py. That dashboard reads shared memory and sends
commands through RobotClient -> command_executor -> RTI, so it needs the runtime UP. This
needs the runtime DOWN, because it takes the CAN bus for itself. The two are mutually
exclusive by construction, so folding this in would produce a dashboard where half the
tabs only work when the other half cannot.

All measurement and analysis is imported from tune_motor_loops; this file only owns the
widgets, interactive rendering, and bus lifecycle. Nothing here re-derives a metric, so
the CLI and the GUI cannot disagree about what a capture means.

Runs accumulate on one plot until you press Clear, which is the whole point: seeing the
previous gain next to the new one is what makes a tuning session converge.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from nicegui import app, run, ui

import Spectral_BLDC as Spectral
import numpy as np
import tune_motor_loops as T

BUS = {"comm": None, "motor": None, "node": 0}
RUNS = []                       # [(label, run_dict, metrics)]
MOTOR_RL_PRESETS = {
    "custom": ("Custom (manual R/L)", None),
    "big": ("Big motor — R 1.6 ohm, L 0.003 H", (1.6, 0.003)),
    "mid": ("Mid motor — R 1.1 ohm, L 0.0026 H", (1.1, 0.0026)),
    "small": ("Small motor — R 3.7 ohm, L 0.0045 H", (3.7, 0.0045)),
}


# ------------------------------------------------------------------- bus lifecycle

def bus_open(node):
    """Lazily open, then keep it. Reopening per run costs ~100 ms and a fresh socket for
    no benefit; Release Bus exists for when the runtime needs it back."""
    if BUS["comm"] is not None and BUS["node"] == node:
        return None
    bus_release()
    if T._rti_is_running():
        return "RTI is running - it owns the CAN bus. Stop the supervisor first."
    comm = Spectral.CanCommunication(bustype="socketcan", channel="can0", bitrate=1000000)
    motor = Spectral.SpectralCAN(node_id=node, communication=comm)
    state = T.read_state(comm, motor, node)
    if state is None:
        try:
            comm.bus.shutdown()
        except Exception:
            pass
        return f"Node {node} did not answer - check power, wiring and CAN id."
    BUS.update(comm=comm, motor=motor, node=node)
    return None


def bus_release():
    if BUS["motor"] is not None:
        try:
            BUS["motor"].Send_Idle()
        except Exception:
            pass
    if BUS["comm"] is not None:
        try:
            BUS["comm"].bus.shutdown()
        except Exception:
            pass
    BUS.update(comm=None, motor=None)


def scan_can_nodes():
    """Find motor nodes using the same zero-current state probe as ``read_state``.

    The library's Remote-CAN ping request is not answered by the deployed motor
    firmware, so it cannot identify a motor. A two-byte ``Current=0`` data packet
    is the known working status request: the driver replies with DATA_PACK_1 while
    commanding no torque, speed, or position. Each detected driver is immediately
    sent Idle before the temporary CAN socket is closed.
    """
    if BUS["comm"] is not None:
        return None, "Release Bus before scanning CAN IDs."
    if T._rti_is_running():
        return None, "RTI is running - it owns the CAN bus. Stop the supervisor first."
    try:
        comm = Spectral.CanCommunication(bustype="socketcan", channel="can0", bitrate=1000000)
    except Exception as exc:
        return None, f"Could not open can0 for scan: {exc}"

    # Nodes 13--15 are reserved for the timing device, host, and bootloader.
    # Physical motors and tools use the remaining four-bit node IDs.
    node_ids = range(13)
    found = set()
    try:
        while comm.receive_can_messages(timeout=0)[0] is not None:
            pass  # do not mistake stale traffic for a response to this scan

        # Probe twice, one node at a time. This paces transmission and gives a
        # responding driver a complete receive window before the next node.
        for _ in range(2):
            for node_id in node_ids:
                probe = Spectral.SpectralCAN(node_id=node_id, communication=comm)
                probe.Send_data_pack_1(Position=None, Speed=None, Current=0)
                deadline = time.monotonic() + 0.04
                while time.monotonic() < deadline:
                    message, msg_id = comm.receive_can_messages(timeout=0.01)
                    if (message is not None and msg_id is not None
                            and msg_id.node_id == node_id
                            and msg_id.command_id == T.RESPOND_DATA_PACK_1_ID
                            and message.dlc == 8):
                        found.add(node_id)

        for node_id in found:
            Spectral.SpectralCAN(node_id=node_id, communication=comm).Send_Idle()
        return sorted(found), None
    finally:
        try:
            comm.bus.shutdown()
        except Exception:
            pass

# Leaving the page must not leave a motor turning. Every capture is time-bounded and
# followed by hold_at_rest, so this is a backstop rather than the primary guarantee.
app.on_shutdown(bus_release)


# --------------------------------------------------------------------- measurement

def do_run(cfg):
    """Blocking. Called via run.io_bound so the event loop keeps serving the page."""
    pre, amp = cfg["pre"], cfg["amplitude"]
    frequency, duty = cfg["frequency"], cfg["duty"]
    if cfg["excite"] in ("sine", "pwm") and frequency <= 0:
        return None, "Sine/PWM frequency must be greater than zero."
    if cfg["excite"] == "pwm" and not 0.0 <= duty <= 100.0:
        return None, "PWM duty must be between 0 and 100 percent."
    if cfg["mode"] == "current" and abs(amp) > cfg["cur_limit"]:
        return None, "Current-test amplitude must not exceed the configured current limit."
    if cfg.get("write_voltage_limit", False):
        voltage_limit = int(cfg["voltage_limit"])
        if voltage_limit != 0 and not 6000 <= voltage_limit <= 10000:
            return None, "Voltage vlim must be 0 (use VBUS) or between 6000 and 10000 mV."

    motor, comm, node = BUS["motor"], BUS["comm"], BUS["node"]

    motor.Send_Clear_Error()
    time.sleep(0.05)
    # Keep the watchdog disabled on every run, including if it was armed previously.
    motor.Send_Watchdog(0, "Idle")
    motor.Send_Limits(cfg["vel_limit"], cfg["cur_limit"])
    if cfg.get("write_voltage_limit", False):
        # This is a separate driver limit in millivolts; 0 means use the bus voltage.
        motor.Send_Voltage_Limit(voltage_limit)
    if cfg["write_current_gains"]:
        # This is the inner Iq PI loop. It must be configured before the velocity or
        # position loop that depends on it.
        motor.Send_Current_Gains(cfg["kpiq"], cfg["kiiq"])
    motor.Send_Velocity_Gains(cfg["kpv"], cfg["kiv"])
    motor.Send_Position_Gains(cfg["kpp"])
    time.sleep(0.1)

    pos_base = 0.0
    if cfg["mode"] == "position":
        state = T.read_state(comm, motor, node)
        if state is None:
            return None, "Could not read shaft position - refusing to guess (see docstring)."
        pos_base = state[0]

    if cfg["excite"] == "step":
        fn = lambda t: T._step(t, amp, pre)
    elif cfg["excite"] == "sine":
        fn = lambda t: T._sine(t, amp, pre, frequency)
    elif cfg["excite"] == "pwm":
        fn = lambda t: T._pwm(t, amp, pre, frequency, duty)
    else:
        sweep_time = max(cfg["duration"] - pre, 0.1)
        fn = lambda t: T._chirp(t, amp, pre, cfg["f0"], cfg["f1"], sweep_time)

    run_data = T.capture(comm, motor, node, fn, cfg["duration"], cfg["rate"],
                         cfg["mode"], pos_base, cfg["ff_current"])
    T.hold_at_rest(comm, motor, 0.4, cfg["rate"])

    run_data = T._to_relative(run_data)
    if run_data is None or run_data["t"].size < 20:
        return None, "No data captured - is the driver powered and enabled?"

    if cfg["excite"] == "step":
        metrics = T.analyze_step(run_data, cfg["mode"], pre, amp)
    elif cfg["excite"] == "chirp":
        b = T.analyze_chirp(run_data, cfg["mode"], pre, cfg["rate"])
        metrics = {"bandwidth_hz": b["bandwidth_hz"]} if b else {
            "error": "chirp analysis failed",
        }
    else:
        metrics = {"note": "visual excitation - inspect the trace; no step/chirp metrics"}
    return (run_data, metrics), None


def _points(x_values, y_values):
    """Convert NumPy values to JSON-safe ECharts point pairs."""
    return [[float(x), float(y)] for x, y in zip(x_values, y_values, strict=True)]


def _x_values(run_data, x_axis):
    """Return the selected x coordinate and its bottom-axis label.

    A capture tick is deliberately the default: during a short limit cycle it makes
    individual CAN samples easy to count even when the achieved sample rate differs
    from the requested rate. Time remains available when comparing against a bandwidth
    or frequency measurement.
    """
    if x_axis == "time":
        return run_data["t"] * 1e3, "time [ms]"
    return range(run_data["t"].size), "capture tick [sample]"


def build_plot_options(runs, mode, excite, x_axis):
    """Build a linked, interactive ECharts plot for accumulated captures."""
    if mode == "velocity":
        unit, response_name = "ticks/s", "measured speed"
        secondary_name, secondary_unit = "current", "mA"
    elif mode == "current":
        unit, response_name = "mA", "measured current"
        secondary_name, secondary_unit = "measured speed", "ticks/s"
    else:
        unit, response_name = "ticks", "measured position"
        secondary_name, secondary_unit = "current", "mA"
    series = []

    for label, run_data in runs:
        x_values, _ = _x_values(run_data, x_axis)
        if mode == "velocity":
            response = run_data["spd"]
            secondary = run_data["cur"]
        elif mode == "current":
            response = run_data["cur"]
            secondary = run_data["spd"]
        else:
            response = run_data["pos"] - run_data["pos"][0]
            secondary = run_data["cur"]
        series.extend((
            {
                "name": f"{response_name}: {label}",
                "type": "line", "xAxisIndex": 0, "yAxisIndex": 0,
                "showSymbol": False, "data": _points(x_values, response),
            },
            {
                "name": f"{secondary_name}: {label}",
                "type": "line", "xAxisIndex": 1, "yAxisIndex": 1,
                "showSymbol": False, "data": _points(x_values, secondary),
            },
            {
                "name": f"encoder position: {label}",
                "type": "line", "xAxisIndex": 2, "yAxisIndex": 2,
                "showSymbol": False,
                "data": _points(x_values, run_data["pos"] - run_data["pos"][0]),
            },
        ))

    # The reference remains the first capture when overlaying runs. Clear captures before
    # changing the waveform so all traces share the intended command reference.
    reference = runs[0][1]
    if x_axis == "time":
        setpoint_x, setpoint_y = reference["cmd_t"] * 1e3, reference["cmd_u"]
    else:
        setpoint_x, _ = _x_values(reference, x_axis)
        setpoint_y = np.interp(reference["t"], reference["cmd_t"], reference["cmd_u"])
    series.insert(0, {
        "name": "setpoint (first run)", "type": "line", "xAxisIndex": 0,
        "yAxisIndex": 0, "showSymbol": False,
        "step": "end" if excite in ("step", "pwm") else False,
        "lineStyle": {"type": "dashed", "width": 1},
        "data": _points(setpoint_x, setpoint_y), "z": 1,
    })

    _, x_label = _x_values(reference, x_axis)
    return {
        "animation": False,
        "title": {"text": f"{mode} loop — {excite} response", "left": "center"},
        "legend": {"type": "scroll", "top": 28, "left": 10, "right": 10},
        "tooltip": {"trigger": "axis", "axisPointer": {"type": "cross"}},
        "axisPointer": {"link": [{"xAxisIndex": "all"}]},
        "toolbox": {"right": 14, "feature": {
            "dataZoom": {"yAxisIndex": "none"}, "restore": {}, "saveAsImage": {},
        }},
        "grid": [
            {"left": 78, "right": 28, "top": 88, "height": "30%"},
            {"left": 78, "right": 28, "top": "50%", "height": "18%"},
            {"left": 78, "right": 28, "top": "76%", "bottom": 82},
        ],
        "xAxis": [
            {"type": "value", "gridIndex": 0, "axisLabel": {"show": False}},
            {"type": "value", "gridIndex": 1, "axisLabel": {"show": False}},
            {"type": "value", "gridIndex": 2, "name": x_label, "nameLocation": "middle", "nameGap": 42},
        ],
        "yAxis": [
            {"type": "value", "gridIndex": 0, "name": f"{mode} [{unit}]", "scale": True},
            {"type": "value", "gridIndex": 1, "name": f"{secondary_name} [{secondary_unit}]", "scale": True},
            {"type": "value", "gridIndex": 2, "name": "encoder position [ticks]", "scale": True},
        ],
        "dataZoom": [
            {"type": "inside", "xAxisIndex": [0, 1, 2], "zoomOnMouseWheel": True,
             "moveOnMouseMove": True, "moveOnMouseWheel": False},
            {"type": "slider", "xAxisIndex": [0, 1, 2], "left": 78, "right": 28,
             "bottom": 16, "height": 22},
        ],
        "series": series,
    }


# --------------------------------------------------------------------------- page

@ui.page("/")
def page():
    def section_title(number, title, subtitle, icon):
        with ui.row().classes("w-full items-start gap-3"):
            ui.icon(icon).classes(
                "mt-0.5 rounded-lg bg-slate-100 p-2 text-slate-700 text-lg"
            )
            with ui.column().classes("gap-0"):
                ui.label(f"{number} · {title}").classes("text-sm font-bold text-slate-800")
                ui.label(subtitle).classes("text-xs text-slate-500")


    with ui.dialog() as guide_dialog:
        with ui.card().classes("w-[960px] max-w-[96vw] max-h-[88vh] overflow-y-auto rounded-2xl p-5"):
            with ui.row().classes("w-full items-start justify-between gap-4"):
                with ui.column().classes("gap-0"):
                    ui.label("Motor tuner guide").classes("text-xl font-bold text-slate-800")
                    ui.label("What RUN CAPTURE sends, plus current-, velocity-, and position-loop test guidance.").classes("text-sm text-slate-500")
                ui.button(icon="close", on_click=guide_dialog.close).props("flat round color=grey")
            ui.separator().classes("my-4")
            with ui.tabs().classes("w-full") as guide_tabs:
                capture_tab = ui.tab("Run capture", icon="send")
                current_tab = ui.tab("Current loop", icon="bolt")
                position_tab = ui.tab("Position loop", icon="my_location")
                signals_tab = ui.tab("Signals & plots", icon="show_chart")
            with ui.tab_panels(guide_tabs, value=capture_tab).classes("w-full bg-transparent"):
                with ui.tab_panel(capture_tab).classes("px-0"):
                    ui.markdown("""
### What happens when you press RUN CAPTURE

The tuner first sends one-time setup commands, waits 100 ms, and then begins the requested-rate command-and-measurement loop.

1. Clear driver errors.
2. Disable the driver watchdog with timeout `0`.
3. Send the velocity and current limits.
4. Send voltage `vlim` only when **Write voltage vlim on RUN** is enabled.
5. Send current-loop `KPIQ` / `KIIQ` only when **Write current PI gains on RUN** is enabled.
6. Send velocity-loop `KPV` / `KIV` and position-loop `KPP` on **every** capture.
7. In position mode, read the live encoder position once and use it as the relative target base.

During capture, only the command packet is repeated at the requested rate. Current mode sends `Current = waveform setpoint` with no position or speed field; velocity mode sends `Speed = waveform setpoint`; position mode sends `Position = starting position + waveform setpoint`. The driver telemetry packets are drained and recorded after each command. The achieved sample rate can be lower than requested when CAN traffic or processing is the limit.

After the duration ends, the tuner sends a zero/rest command for 0.4 seconds.
                    """).classes("text-sm leading-6")
                with ui.tab_panel(current_tab).classes("px-0"):
                    ui.markdown("""
### Direct Iq / current-loop test

Selecting **current** sends a pure `Current = waveform` packet. It does not send a speed or position command, so it exposes the inner current response as seen through CAN telemetry. The upper plot becomes measured current; the middle plot shows any shaft speed caused by the torque.

1. Start with a **step**, 250–500 mA amplitude, and keep it below the configured current limit.
2. Tick **Write current PI gains on RUN** only when you intend to test the displayed `KPIQ` / `KIIQ`; otherwise the motor's existing current gains are preserved.
3. Increase `KPIQ` / `KIIQ` only after a clean response. Excessive current-loop gain can create noise or oscillation.

A current-only command produces torque: the shaft can move if it is free or loaded. Keep the mechanism clear. CAN sampling observes roughly up to 1 kHz, while the firmware current loop runs at 6.25 kHz, so this tool cannot resolve every inner-loop detail.
                    """).classes("text-sm leading-6")
                with ui.tab_panel(position_tab).classes("px-0"):
                    ui.markdown("""
### Tune velocity first

The position loop sits outside the velocity loop. A ringing velocity loop will make position tuning misleading.

### Position-step target

Start with a small, safe position step. A good trace moves to the target once, has little overshoot, settles to a flat encoder position, and has measured speed return to zero. Current should spike while accelerating, then fall toward the current needed to hold the load.

- **Ringing or repeated overshoot:** reduce `KPP`.
- **Slow but stable:** increase `KPP` gradually.
- **Small persistent error:** can come from encoder resolution, backlash, compliance, or load; sustained hunting is not normal.

Position tests are relative: the target is the shaft position at the start of the capture plus the displayed waveform, never an absolute move to tick zero.
                    """).classes("text-sm leading-6")
                with ui.tab_panel(signals_tab).classes("px-0"):
                    ui.markdown("""
### Excitation choices

- **Step:** the primary settling and overshoot test.
- **Sine:** at low frequency, position should follow a smooth sine with similar amplitude and little phase lag. Rising frequency brings lag and amplitude loss; strong growth, distortion, or ringing suggests resonance or too much `KPP`.
- **PWM:** this is a setpoint that alternates between zero and the amplitude, **not** FOC/phase PWM. Start slowly enough that the shaft can settle at each end. At higher frequency it becomes a bandwidth test.
- **Chirp:** sweeps frequency to estimate bandwidth.

**capture tick** is the sequential received-sample number; it is not assumed to equal milliseconds. **time [ms]** uses the kernel timestamp on each received motor telemetry frame. Use the achieved-rate line in the run summary when relating the two.
                    """).classes("text-sm leading-6")
            ui.button("Close guide", on_click=guide_dialog.close).classes("mt-4").props("outline color=primary")
    with ui.column().classes("w-full max-w-[2560px] mx-auto gap-4 p-3 lg:p-5"):

        # Desktop workspace: one tabbed control column and a permanently visible analysis area.
        # Narrow screens wrap gracefully; wide/4K displays keep both side by side for recording.
        with ui.row().classes("w-full items-start gap-4 flex-wrap xl:flex-nowrap"):
            with ui.card().classes(
                "w-full xl:w-[480px] xl:min-w-[480px] xl:max-w-[480px] rounded-xl border border-slate-200 shadow-sm"
            ):
                with ui.row().classes("w-full items-center justify-between gap-3"):
                    with ui.column().classes("gap-0"):
                        ui.label("Tuning console").classes("text-lg font-bold text-slate-800")
                        ui.label("Select one control group; the capture stays visible beside it.").classes("text-xs text-slate-500")
                    ui.icon("tune").classes("rounded-lg bg-blue-50 p-2 text-xl text-blue-700")
                ui.separator().classes("my-3")
                with ui.tabs().classes("w-full").props("dense align=justify") as control_tabs:
                    setup_tab = ui.tab("Setup", icon="settings")
                    loops_tab = ui.tab("Loops", icon="speed")
                    capture_tab = ui.tab("Capture", icon="timeline")
                    safety_tab = ui.tab("Safety", icon="shield")
                with ui.tab_panels(control_tabs, value=setup_tab).classes("w-full bg-transparent"):
                    with ui.tab_panel(setup_tab).classes("px-0"):
                        section_title("SETUP", "Test configuration", "Select the loop, signal, and plot coordinate.", "settings")
                        ui.separator().classes("my-3")
                        status = ui.label("bus: closed").classes(
                            "w-full rounded-lg bg-slate-100 px-3 py-2 font-mono text-xs text-slate-600"
                        )
                        with ui.row().classes("mt-3 w-full gap-3"):
                            node = ui.number("CAN node", value=0, format="%.0f").classes("flex-1")
                            mode = ui.select(["current", "velocity", "position"], value="velocity", label="loop under test").classes("flex-1")
                        with ui.row().classes("mt-2 w-full items-center gap-2"):
                            scan_btn = ui.button("SCAN CAN IDs").classes("flex-1 font-bold").props("outline color=primary icon=search")
                            ui.button("GUIDE", on_click=guide_dialog.open).props("outline color=primary icon=help_outline")
                        scan_result = ui.label(
                            "Probes motor IDs 0–12 with zero current, then idles every detected motor."
                        ).classes("mt-1 text-xs text-slate-500")
                        with ui.row().classes("mt-2 w-full gap-3"):
                            excite = ui.select(["step", "sine", "pwm", "chirp"], value="step", label="excitation").classes("flex-1")
                            x_axis = ui.select({"sample": "capture tick", "time": "time [ms]"}, value="sample", label="plot x-axis").classes("flex-1")
                        mode_hint = ui.label("Velocity mode: amplitude is ticks/s; tune KPV/KIV after current PI is stable.").classes("mt-2 text-xs text-slate-600")
                        ui.separator().classes("my-3")
                        section_title("1", "Current loop · Iq", "Optional driver write plus the firmware R/L calculator.", "bolt")
                        ui.separator().classes("my-3")
                        current_apply = ui.checkbox("Write current PI gains on RUN", value=False).classes("w-full font-medium text-slate-800")
                        with ui.row().classes("mt-2 w-full gap-3"):
                            kpiq = ui.number("KPIQ · P", value=0.012, format="%.6f", step=0.001).classes("flex-1")
                            kiiq = ui.number("KIIQ · I", value=0.012, format="%.6f", step=0.001).classes("flex-1")
                        with ui.row().classes("mt-2 w-full gap-3"):
                            current_bandwidth = ui.number("current bandwidth (Hz)", value=800, format="%.0f", step=50).classes("flex-1")
                            motor_preset = ui.select(
                                {key: label for key, (label, _) in MOTOR_RL_PRESETS.items()},
                                value="custom", label="motor R/L preset",
                            ).classes("flex-1")
                        with ui.row().classes("mt-2 w-full gap-3"):
                            phase_resistance = ui.number("controller.R (ohm)", value=None, format="%.6f", step=0.001).classes("flex-1")
                            controller_inductance = ui.number("controller.L (H)", value=None, format="%.9f", step=0.000001).classes("flex-1")
                        current_calc = ui.label("Choose a preset or enter R/L, then calculate.").classes(
                            "mt-2 w-full rounded-lg bg-slate-50 px-3 py-2 font-mono text-xs text-slate-600"
                        )

                        def load_motor_preset(event):
                            label, values = MOTOR_RL_PRESETS[event.value]
                            if values is None:
                                current_calc.set_text("Custom R/L values; enter both, then calculate.")
                                return
                            resistance, inductance = values
                            phase_resistance.set_value(resistance)
                            controller_inductance.set_value(inductance)
                            current_calc.set_text(f"{label} loaded; choose bandwidth, then calculate.")

                        motor_preset.on_value_change(load_motor_preset)

                        def fill_current_pi_from_rl():
                            try:
                                gains = T.calculate_current_pi_gains(
                                    float(phase_resistance.value),
                                    float(controller_inductance.value),
                                    float(current_bandwidth.value),
                                )
                            except (TypeError, ValueError) as exc:
                                ui.notify(f"Current PI calculator: {exc}", type="negative")
                                return
                            kpiq.set_value(gains["kpiq"])
                            kiiq.set_value(gains["kiiq"])
                            current_calc.set_text(
                                f"KPIQ={gains['kpiq']:.6g}, KIIQ={gains['kiiq']:.6g}; "
                                f"firmware Ki={gains['discrete_ki']:.6g}"
                            )

                        ui.button("Calculate and fill current PI", on_click=fill_current_pi_from_rl).classes("mt-3 w-full").props("outline color=primary")

                    with ui.tab_panel(loops_tab).classes("px-0"):
                        section_title("2", "Velocity & position loops", "Tune outer-loop gains after the current loop is stable.", "speed")
                        ui.separator().classes("my-3")
                        with ui.row().classes("w-full gap-3"):
                            kpv = ui.number("KPV · velocity P", value=0.007, format="%.6f", step=0.001).classes("flex-1")
                            kiv = ui.number("KIV · velocity I", value=0.0005, format="%.6f", step=0.0001).classes("flex-1")
                        kpp = ui.number("KPP · position P", value=15.0, format="%.3f", step=1.0).classes("mt-3 w-full")
                        ui.label("Stabilize current first, velocity second, and position last. The active loop is selected in Setup.").classes("mt-3 text-xs text-slate-600")

                    with ui.tab_panel(capture_tab).classes("px-0"):
                        section_title("3", "Command & capture", "Keep the test repeatable while comparing gain changes.", "timeline")
                        ui.separator().classes("my-3")
                        with ui.row().classes("w-full gap-3"):
                            amplitude = ui.number("amplitude", value=4000, format="%.0f").classes("flex-1")
                            duration = ui.number("capture duration (s)", value=1.2, format="%.2f", step=0.1).classes("flex-1")
                        pre = ui.number("quiet before (s)", value=0.2, format="%.2f", step=0.05).classes("mt-2 w-full")
                        with ui.row().classes("mt-2 w-full gap-3"):
                            rate = ui.number("requested rate (Hz)", value=1000, format="%.0f").classes("flex-1")
                            f0 = ui.number("chirp start (Hz)", value=1.0, format="%.1f").classes("flex-1")
                            f1 = ui.number("chirp end (Hz)", value=200.0, format="%.1f").classes("flex-1")
                        with ui.row().classes("mt-2 w-full gap-3"):
                            frequency = ui.number("sine / PWM (Hz)", value=5.0, format="%.2f", step=1.0).classes("flex-1")
                            duty = ui.number("PWM duty (%)", value=50.0, format="%.1f", step=5.0).classes("flex-1")
                        amplitude_hint = ui.label("Amplitude: ticks/s in velocity mode, ticks in position mode, mA in current mode. "
                                                  "Sine reverses; PWM is a setpoint, not FOC PWM.").classes("mt-3 text-xs text-amber-700")

                    with ui.tab_panel(safety_tab).classes("px-0"):
                        section_title("4", "Safety envelope", "Limits are sent to the driver before every capture.", "shield")
                        ui.separator().classes("my-3")
                        with ui.row().classes("w-full gap-3"):
                            cur_limit = ui.number("current limit (mA)", value=2500, format="%.0f").classes("flex-1")
                            vel_limit = ui.number("velocity limit (ticks/s)", value=40000, format="%.0f").classes("flex-1")
                        with ui.row().classes("mt-2 w-full gap-3"):
                            ff_current = ui.number("feedforward current (mA)", value=0, format="%.0f").classes("flex-1")
                            ui.label("watchdog: disabled on every run").classes("flex-1 self-center text-sm font-medium text-slate-700")
                        voltage_apply = ui.checkbox("Write voltage vlim on RUN", value=False).classes("mt-3 font-medium text-slate-800")
                        voltage_limit = ui.number("vlim (voltage limit, mV; 0 = VBUS)", value=6000,
                                                  min=0, max=10000, format="%.0f", step=100).classes("mt-2 w-full")
                        ui.label("When enabled, the driver receives this separate voltage limit. Use 0 for VBUS, or choose 6000–10000 mV.").classes("mt-1 text-xs text-amber-800")
                        ui.label("Watchdog is always disabled; a stopped process can leave the last command active.").classes("mt-2 text-xs text-amber-800")
                        ui.separator().classes("my-3")
                        ui.label("Run controls").classes("text-sm font-bold text-slate-800")
                        ui.label("Every run is time-bounded and ends by commanding rest.").classes("mt-1 text-xs text-slate-600")
                        run_btn = ui.button("RUN CAPTURE").classes("mt-3 w-full font-bold").props("color=primary unelevated")
                        with ui.row().classes("mt-2 w-full gap-2"):
                            ui.button("Clear captures", on_click=lambda: clear_runs()).classes("flex-1").props("outline color=primary")
                            ui.button("Release bus", on_click=lambda: do_release()).classes("flex-1").props("outline color=primary")
                        ui.button("STOP / IDLE MOTOR", on_click=lambda: do_idle()).classes("mt-2 w-full font-bold").props("color=red outline")

            with ui.column().classes("flex-1 min-w-[720px] gap-4 analysis-anchor"):
                with ui.card().classes("w-full rounded-xl border border-slate-200 shadow-sm"):
                    with ui.row().classes("w-full items-center justify-between gap-4"):
                        with ui.column().classes("gap-0"):
                            ui.label("Capture analysis").classes("text-lg font-bold text-slate-800")
                            ui.label("The plot remains visible while you change control tabs.").classes("text-sm text-slate-500")
                        ui.label("ZOOM · PAN · EXPORT").classes(
                            "rounded-full bg-blue-50 px-3 py-1 text-xs font-bold tracking-wide text-blue-700"
                        )
                    ui.separator().classes("my-3")
                    empty_state = ui.column().classes(
                        "w-full items-center justify-center rounded-xl border border-dashed border-slate-300 bg-slate-50 py-10 text-center"
                    )
                    with empty_state:
                        ui.icon("insights").classes("text-5xl text-slate-300")
                        ui.label("No capture yet").classes("mt-3 text-lg font-bold text-slate-600")
                        ui.label("Configure a safe run in the console, then compare its traces here.").classes("text-sm text-slate-500")
                    plot = ui.echart({"animation": False}).classes("w-full rounded-lg").style("height: min(780px, calc(100vh - 280px))")
                    plot.set_visibility(False)
                    ui.label("Mouse wheel or pinch zooms; drag pans. The slider, legend, reset, and image export are available above the traces.").classes("mt-3 text-xs text-slate-500")

                with ui.card().classes("w-full rounded-xl border border-slate-200 shadow-sm"):
                    with ui.row().classes("items-center gap-2"):
                        ui.icon("fact_check").classes("text-slate-600")
                        ui.label("Run summary").classes("text-sm font-bold text-slate-800")
                    metrics_box = ui.column().classes("mt-3 w-full gap-0 rounded-lg bg-slate-50 p-3 font-mono text-sm")
                    with metrics_box:
                        ui.label("No runs yet — press RUN CAPTURE.").classes("text-slate-500")

    async def scroll_to_analysis():
        await ui.run_javascript(
            'document.querySelector(".analysis-anchor")?.scrollIntoView({behavior: "smooth", block: "start"});'
        )

    async def scroll_to_controls():
        await ui.run_javascript('window.scrollTo({top: 0, behavior: "smooth"});')


    def update_mode_controls(_=None):
        """Keep command units safe and unambiguous when the tested loop changes."""
        if mode.value == "current":
            amplitude.set_value(500)
            ff_current.set_value(0)
            ff_current.disable()
            mode_hint.set_text(
                "Current mode: direct Iq command in mA; start at 500 mA or less. "
                "It produces torque, so the shaft may move. Enable current PI writing to test new gains."
            )
            amplitude_hint.set_text(
                "Amplitude: mA in current mode. The command is pure Current (no position/speed loop); "
                "it cannot exceed the current limit."
            )
        elif mode.value == "velocity":
            amplitude.set_value(4000)
            ff_current.enable()
            mode_hint.set_text("Velocity mode: amplitude is ticks/s; tune KPV/KIV after current PI is stable.")
            amplitude_hint.set_text("Amplitude: ticks/s in velocity mode. Sine reverses; PWM is a setpoint, not FOC PWM.")
        else:
            amplitude.set_value(500)
            ff_current.enable()
            mode_hint.set_text("Position mode: amplitude is relative encoder ticks; tune KPP only after velocity is stable.")
            amplitude_hint.set_text("Amplitude: ticks in position mode. Sine reverses; PWM is a setpoint, not FOC PWM.")
        if RUNS:
            clear_runs()
            ui.notify("Capture history cleared because the loop under test changed", type="info")

    mode.on_value_change(update_mode_controls)
    # ------------------------------------------------------------------ actions

    def refresh_plot():
        if not RUNS:
            return
        options = build_plot_options(
            [(label, run_data) for label, run_data, _ in RUNS],
            mode.value, excite.value, x_axis.value,
        )
        plot.options.clear()
        plot.options.update(options)
        plot.update()

    def clear_runs():
        RUNS.clear()
        plot.set_visibility(False)
        empty_state.set_visibility(True)
        metrics_box.clear()
        with metrics_box:
            ui.label("cleared").classes("text-gray-500")

    def do_release():
        bus_release()
        status.set_text("bus: closed")
        ui.notify("Bus released - the runtime can start again", type="positive")

    def do_idle():
        if BUS["motor"] is not None:
            BUS["motor"].Send_Idle()
            ui.notify("Motor idled", type="warning")
        else:
            ui.notify("Bus is closed — this tuner has no motor to idle", type="info")

    async def on_scan():
        scan_btn.disable()
        try:
            found, err = await run.io_bound(scan_can_nodes)
            if err:
                scan_result.set_text(err)
                ui.notify(err, type="negative", timeout=8000)
            elif len(found) == 1:
                node.set_value(found[0])
                scan_result.set_text(f"Found one Spectral motor: CAN node {found[0]}")
                ui.notify(f"CAN node {found[0]} selected", type="positive")
            elif found:
                values = ", ".join(str(value) for value in found)
                scan_result.set_text(f"Found multiple nodes: {values}. Select the motor ID manually.")
                ui.notify("Multiple Spectral nodes found; choose one manually", type="warning")
            else:
                scan_result.set_text("No zero-current status replies. Check power, can0 wiring/bitrate, and RTI.")
                ui.notify("No motor answered the CAN-ID scan", type="negative", timeout=8000)
        finally:
            scan_btn.enable()

    scan_btn.on_click(on_scan)
    async def on_run():
        run_btn.disable()
        try:
            err = await run.io_bound(bus_open, int(node.value))
            if err:
                ui.notify(err, type="negative", timeout=8000)
                return
            status.set_text(f"bus: open, node {BUS['node']}")

            cfg = dict(mode=mode.value, excite=excite.value,
                       write_current_gains=bool(current_apply.value),
                       write_voltage_limit=bool(voltage_apply.value),
                       voltage_limit=float(voltage_limit.value),
                       kpiq=float(kpiq.value), kiiq=float(kiiq.value),
                       kpv=float(kpv.value), kiv=float(kiv.value), kpp=float(kpp.value),
                       amplitude=float(amplitude.value), duration=float(duration.value),
                       pre=float(pre.value), rate=float(rate.value),
                       f0=float(f0.value), f1=float(f1.value),
                       frequency=float(frequency.value), duty=float(duty.value),
                       cur_limit=float(cur_limit.value), vel_limit=float(vel_limit.value),
                       ff_current=float(ff_current.value))

            result, err = await run.io_bound(do_run, cfg)
            if err:
                ui.notify(err, type="negative", timeout=8000)
                return

            run_data, metrics = result
            current_label = (f"KPIQ={kpiq.value:g} KIIQ={kiiq.value:g}"
                             if current_apply.value else "current PI unchanged")
            voltage_label = (f"vlim={voltage_limit.value:g} mV"
                             if voltage_apply.value else "vlim unchanged")
            if mode.value == "velocity":
                gain_label = f"{voltage_label} | {current_label} | KPV={kpv.value:g} KIV={kiv.value:g}"
                amplitude_unit = "ticks/s"
            elif mode.value == "current":
                gain_label = f"{voltage_label} | {current_label} | direct Iq test"
                amplitude_unit = "mA"
            else:
                gain_label = f"{voltage_label} | {current_label} | KPP={kpp.value:g}"
                amplitude_unit = "ticks"
            if excite.value == "sine":
                signal_label = f"sine +-{amplitude.value:g} {amplitude_unit} @ {frequency.value:g} Hz"
            elif excite.value == "pwm":
                signal_label = (f"PWM 0-{amplitude.value:g} {amplitude_unit} @ {frequency.value:g} Hz "
                                f"({duty.value:g}% duty)")
            else:
                signal_label = f"{excite.value} {amplitude.value:g} {amplitude_unit}"
            label = f"{gain_label} | {signal_label}"
            RUNS.append((label, run_data, metrics))
            refresh_plot()
            plot.set_visibility(True)
            empty_state.set_visibility(False)

            metrics_box.clear()
            with metrics_box:
                ui.label(f"--- {label} ---").classes("font-bold mt-2")
                achieved = run_data["t"].size / float(duration.value)
                ui.label(f"{run_data['t'].size} samples, {achieved:.0f} Hz achieved").classes("text-gray-500")
                if achieved < 0.6 * float(rate.value):
                    ui.label("! sample rate well under target - results are coarse").classes("text-orange-600")
                if excite.value == "chirp":
                    bw = metrics.get("bandwidth_hz")
                    ui.label(f"-3 dB bandwidth {bw:.1f} Hz" if bw else metrics.get("error", "no result"))
                elif excite.value == "step":
                    for line in T.format_metrics(metrics, mode.value):
                        cls = "text-red-600" if line.startswith("!") else ""
                        cls = "text-orange-700 font-bold" if "SUSTAINED" in line else cls
                        ui.label(line).classes(cls)
                else:
                    ui.label(metrics["note"]).classes("text-gray-500")
            await scroll_to_analysis()
        finally:
            run_btn.enable()
    run_btn.on_click(on_run)
    x_axis.on_value_change(lambda _: refresh_plot())


if __name__ in {"__main__", "__mp_main__"}:
    ui.run(host="0.0.0.0", port=8082, title="Motor loop tuner", reload=False, show=False)
