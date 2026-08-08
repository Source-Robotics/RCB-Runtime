"""Web front-end for the motor-driver firmware flasher, on its own port (8083).

Deliberately NOT part of nicegui_test.py, for the same reason tune_motor_gui.py
is not: that dashboard reads shared memory and sends commands through
RobotClient -> command_executor -> RTI, so it needs the runtime UP. This needs
the runtime DOWN, because it takes the CAN bus for itself. The two are mutually
exclusive by construction, so folding this in would produce a dashboard where
half the tabs only work when the other half cannot.

    sudo systemctl stop autorun.service     # or rcb-runtime.service
    python3 tools/flash_motor_gui.py        # then open http://<pi>:8083

All protocol work lives in flash_motor_firmware.py; this file owns widgets and
bus lifecycle only. The panel flow: pick a source, optionally scan, flash one
node with a live progress bar and log, then verify.

Threading: flash_firmware blocks for tens of seconds, so it runs via
run.io_bound. Its on_progress/on_log callbacks fire on that worker thread and
only ever write into STATE; a ui.timer drains STATE into the widgets on the
event loop. Cancel is a threading.Event the worker polls between chunks.
"""

import sys
import threading
import time
from collections import deque
from pathlib import Path

from nicegui import app, run, ui

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import flash_motor_firmware as F
import firmware_github as G

BUS = {"bus": None}
CANCEL = threading.Event()

STATE = {
    "stage": "", "fraction": 0.0, "page": None, "total_pages": None,
    "message": "", "log": deque(maxlen=500), "running": False, "dirty": True,
}

# Firmware currently selected: bytes + where it came from + declared link address.
FIRMWARE = {"bytes": None, "description": "", "app_start": F.APP_BASE_ADDRESS,
            "validation": None}


def log_line(line):
    STATE["log"].append(f"{time.strftime('%H:%M:%S')}  {line}")
    STATE["dirty"] = True


def on_progress(update):
    STATE.update(update)
    STATE["dirty"] = True


# ------------------------------------------------------------------- bus lifecycle
def bus_open():
    """Lazily open and keep it. Returns an error string, or None on success."""
    if BUS["bus"] is not None:
        return None
    if F.rti_is_running():
        return ("RTI is running - it owns the CAN bus. Stop the runtime first:  "
                f"sudo systemctl stop {F.runtime_service_name()}")
    try:
        BUS["bus"] = F.open_bus()
    except Exception as error:
        return f"Could not open can0: {error}"
    return None


def bus_release():
    if BUS["bus"] is not None:
        F.close_bus(BUS["bus"])
        BUS["bus"] = None


# Leaving the page must not leave the bus held — the runtime cannot restart
# while this process owns the socket.
app.on_shutdown(bus_release)


# ------------------------------------------------------------------- worker bodies
def do_scan(reset_apps):
    if (error := bus_open()) is not None:
        return None, error
    try:
        return F.scan_bootloader_boards(BUS["bus"], reset_apps=reset_apps,
                                        on_log=log_line), None
    except Exception as error:
        return None, str(error)


def do_flash(board_id, app_node_id, erase_first, chunk_frames):
    if (error := bus_open()) is not None:
        return error
    try:
        F.flash_firmware(BUS["bus"], board_id, FIRMWARE["bytes"],
                         erase_first=erase_first,
                         app_reset_node_id=app_node_id,
                         stream_chunk_frames=chunk_frames,
                         on_progress=on_progress, on_log=log_line,
                         cancel=CANCEL)
        return None
    except F.FlashCancelled:
        return ("Cancelled. The board is still in its bootloader — it will stay "
                "there until you flash it again (Passive scan will find it).")
    except Exception as error:
        return str(error)


def do_read_info(node_id):
    if (error := bus_open()) is not None:
        return None, error
    try:
        return F.read_device_info(BUS["bus"], node_id), None
    except Exception as error:
        return None, str(error)


def do_list_releases(product_id):
    try:
        return G.list_releases(product_id), None
    except G.FirmwareFetchError as error:
        return None, str(error)


def do_download(product_id, tag):
    try:
        return G.download_firmware(product_id, tag=tag, on_log=log_line), None
    except G.FirmwareFetchError as error:
        return None, str(error)


def set_firmware(data, description, app_start=F.APP_BASE_ADDRESS):
    FIRMWARE.update(bytes=data, description=description, app_start=app_start,
                    validation=F.validate_firmware_bytes(data, app_start=app_start))
    log_line(f"Firmware selected: {description} ({len(data)} bytes)")


# =================================================================== page
@ui.page("/")
def main_page():
    ui.label("Motor firmware flasher").classes("text-2xl font-bold")
    ui.label("Flashes Spectral / STEPFOC motor drivers over CAN using the "
             "STM32F103 bootloader. The runtime must be stopped.") \
        .classes("text-gray-500")

    # ---------------------------------------------------------- bus and safety
    with ui.card().classes("w-full"):
        ui.label("1. Bus and safety").classes("text-lg font-bold")
        ui.label("Flashing REBOOTS motor drivers and they lose torque while it "
                 "happens. Park the arm in a safe pose first — it will go limp. "
                 "After flashing, restart the runtime and RE-HOME the robot.") \
            .classes("text-orange-700")
        with ui.row().classes("items-center gap-4"):
            rti_badge = ui.badge("checking...").props("color=grey")
            bus_badge = ui.badge("bus: closed").props("color=grey")
            ui.button("Open bus", on_click=lambda: handle_open_bus())
            ui.button("Release bus", on_click=lambda: handle_release_bus()) \
                .props("outline")

    # ---------------------------------------------------------- scan
    with ui.card().classes("w-full"):
        ui.label("2. Find boards (optional)").classes("text-lg font-bold")
        ui.label("Active scan reboots EVERY motor on the bus into its "
                 "bootloader; they return to normal ~3 s after the bus goes "
                 "quiet. Passive scan only finds boards already stuck in a "
                 "bootloader, e.g. after an interrupted flash.") \
            .classes("text-gray-500")
        with ui.row().classes("items-center gap-2"):
            ui.button("Active scan (reboots all motors)",
                      on_click=lambda: handle_scan(True)).props("color=orange")
            ui.button("Passive scan", on_click=lambda: handle_scan(False)).props("outline")
        scan_result = ui.row().classes("items-center gap-2 flex-wrap")
        ui.label("Node 0 is Joint 1 on this robot — a legitimate device, not a "
                 "factory-fresh board. But a REPLACEMENT board also ships as ID 0: "
                 "if you just installed one, make sure only one ID-0 device is on "
                 "the bus, then use the CLI's set-id.").classes("text-xs text-gray-500")

    # ---------------------------------------------------------- firmware source
    with ui.card().classes("w-full"):
        ui.label("3. Firmware image").classes("text-lg font-bold")
        source = ui.radio({"upload": "Upload .bin", "github": "GitHub release",
                           "cache": "Previously downloaded"},
                          value="upload").props("inline")

        with ui.column().classes("w-full") as upload_box:
            ui.upload(label="Drop or choose a .bin (max 116 KB)",
                      auto_upload=True, max_file_size=F.MAX_APP_PAGES * F.PAGE_SIZE,
                      on_upload=lambda e: handle_upload(e)).classes("w-full")
            ui.label("Must be built with FLASH origin 0x08003000 — an image "
                     "linked at 0x08000000 needs an SWD probe to undo, so it is "
                     "rejected below.").classes("text-xs text-gray-500")

        with ui.column().classes("w-full") as github_box:
            with ui.row().classes("items-center gap-2"):
                product = ui.select(
                    {pid: meta["label"] for pid, meta in G.FIRMWARE_PRODUCTS.items()},
                    value="spectral-bldc", label="Product").classes("w-64")
                ui.button("List releases",
                          on_click=lambda: handle_list_releases()).props("outline")
            release = ui.select({}, label="Release").classes("w-full")
            ui.button("Download selected", on_click=lambda: handle_download())
            ui.label("Needs internet. If this robot has outgoing traffic "
                     "blocked, use an upload or a previously downloaded image.") \
                .classes("text-xs text-gray-500")

        with ui.column().classes("w-full") as cache_box:
            cached = ui.select({}, label="Cached image").classes("w-full")
            with ui.row().classes("gap-2"):
                ui.button("Refresh list", on_click=lambda: refresh_cache()).props("outline")
                ui.button("Use selected", on_click=lambda: handle_use_cached())

        firmware_label = ui.label("No firmware selected.").classes("font-bold mt-2")
        validation_box = ui.column().classes("gap-0")

    # ---------------------------------------------------------- flash
    with ui.card().classes("w-full"):
        ui.label("4. Flash").classes("text-lg font-bold")
        with ui.row().classes("items-center gap-4"):
            node = ui.number("Bootloader board ID", value=0, min=0,
                             max=F.BL_BOARD_ID_MAX, step=1).classes("w-48")
            link_ids = ui.checkbox("App node ID is the same", value=True)
            app_node = ui.number("App node ID", value=0, min=0,
                                 max=F.APP_NODE_ID_MAX, step=1).classes("w-48")
            node_role = ui.label("").classes("text-gray-500")
        with ui.row().classes("items-center gap-4"):
            erase = ui.checkbox("Erase app region first (recommended)", value=True)
            chunk = ui.number("Stream chunk frames", value=F.DEFAULT_STREAM_CHUNK_FRAMES,
                              min=1, max=F.CAN_STREAM_FRAME_COUNT, step=1).classes("w-48")
        override = ui.checkbox("Flash even though validation FAILED "
                               "(can brick the board)", value=False)
        with ui.row().classes("items-center gap-2"):
            flash_btn = ui.button("Flash this node",
                                  on_click=lambda: handle_flash()).props("color=red")
            cancel_btn = ui.button("Cancel", on_click=lambda: handle_cancel()) \
                .props("outline color=red")
        progress = ui.linear_progress(value=0.0, show_value=False).classes("w-full")
        stage_label = ui.label("").classes("text-sm")

    # ---------------------------------------------------------- after / log
    with ui.card().classes("w-full"):
        ui.label("5. After flashing").classes("text-lg font-bold")
        with ui.row().classes("items-center gap-2"):
            ui.button("Read device info (verify version)",
                      on_click=lambda: handle_verify()).props("outline")
            info_label = ui.label("").classes("text-sm")
        ui.markdown(
            "1. Release the bus (or close this tool).\n"
            f"2. `sudo systemctl start {F.runtime_service_name()}`\n"
            "3. Check the dashboard shows **ROBOT: CONNECTED** and the new "
            "software version on the Motor tab.\n"
            "4. **Re-home the robot** — restarting RTI clears homing.\n\n"
            "If a flash failed part-way the board stays in its bootloader: run "
            "**Passive scan** and flash it again. Nothing is lost."
        ).classes("text-sm")

    log_view = ui.log(max_lines=500).classes("w-full h-64")

    # ---------------------------------------------------------- handlers
    def refresh_source_boxes():
        upload_box.set_visibility(source.value == "upload")
        github_box.set_visibility(source.value == "github")
        cache_box.set_visibility(source.value == "cache")

    source.on_value_change(lambda _: refresh_source_boxes())
    refresh_source_boxes()

    def refresh_node_role():
        role = F.NODE_ROLES.get(int(node.value or 0))
        node_role.set_text(f"= {role}" if role else "= unexpected node")
        if link_ids.value:
            app_node.set_value(node.value)
        app_node.set_enabled(not link_ids.value)

    node.on_value_change(lambda _: refresh_node_role())
    link_ids.on_value_change(lambda _: refresh_node_role())
    refresh_node_role()

    def show_validation():
        validation_box.clear()
        result = FIRMWARE["validation"]
        if result is None:
            firmware_label.set_text("No firmware selected.")
            return
        firmware_label.set_text(f"Selected: {FIRMWARE['description']}")
        with validation_box:
            ui.label(F.format_validation(result).replace("\n", " | ")) \
                .classes("text-xs text-gray-600")
            for warning in result.warnings:
                ui.label(f"WARNING: {warning}").classes("text-orange-700 text-sm")
            for error in result.errors:
                ui.label(f"ERROR: {error}").classes("text-red-600 text-sm font-bold")
            ui.label("PASS — safe to flash" if result.ok
                     else "FAIL — flashing is blocked unless you tick the override")\
                .classes("font-bold " + ("text-green-700" if result.ok else "text-red-600"))

    def handle_open_bus():
        if (error := bus_open()) is not None:
            ui.notify(error, type="negative", timeout=10000)
            log_line(f"ERROR: {error}")
            return
        ui.notify("CAN bus open", type="positive")
        log_line("CAN bus opened.")

    def handle_release_bus():
        bus_release()
        ui.notify("Bus released", type="info")
        log_line("CAN bus released.")

    async def handle_scan(reset_apps):
        if reset_apps and not await confirm(
                "Active scan reboots EVERY motor on the bus. The arm will go "
                "limp. Continue?"):
            return
        found, error = await run.io_bound(do_scan, reset_apps)
        if error:
            ui.notify(error, type="negative", timeout=10000)
            log_line(f"ERROR: {error}")
            return
        scan_result.clear()
        with scan_result:
            if not found:
                ui.label("No bootloaders answered." if not reset_apps else
                         "No bootloaders answered — are the boards powered?") \
                    .classes("text-gray-500")
            for board_id in found:
                ui.button(F._node_label(board_id),
                          on_click=lambda _, b=board_id: select_node(b)).props("outline")
        log_line(f"Scan found: {found or 'nothing'}")

    def select_node(board_id):
        node.set_value(board_id)
        refresh_node_role()
        ui.notify(f"Selected board {board_id}", type="positive")

    def handle_upload(event):
        data = event.content.read()
        if not event.name.lower().endswith(".bin"):
            ui.notify(f"'{event.name}' is not a .bin — check this is the raw "
                      "image, not a .hex or .elf", type="warning", timeout=8000)
        path = G.save_uploaded_firmware(event.name, data)
        set_firmware(data, f"upload {event.name}")
        log_line(f"Saved upload to {path}")
        show_validation()

    async def handle_list_releases():
        releases, error = await run.io_bound(do_list_releases, product.value)
        if error:
            ui.notify(error, type="negative", timeout=12000)
            log_line(f"ERROR: {error}")
            return
        options = {}
        for entry in releases:
            suffix = "" if entry["has_manifest"] and entry["has_bin"] else "  (no image)"
            prerelease = "  [pre-release]" if entry["prerelease"] else ""
            options[entry["tag"]] = f"{entry['tag']}{prerelease}{suffix}"
        release.set_options(options)
        if options:
            release.set_value(next(iter(options)))
        ui.notify(f"{len(options)} release(s) listed", type="positive")

    async def handle_download():
        if not release.value:
            ui.notify("Pick a release first", type="warning")
            return
        downloaded, error = await run.io_bound(do_download, product.value, release.value)
        if error:
            ui.notify(error, type="negative", timeout=12000)
            log_line(f"ERROR: {error}")
            return
        base = G.manifest_app_base_address(downloaded["manifest"]) or F.APP_BASE_ADDRESS
        set_firmware(downloaded["path"].read_bytes(),
                     f"{product.value} {downloaded['tag']}", app_start=base)
        show_validation()
        ui.notify(f"Downloaded {downloaded['tag']}", type="positive")

    def refresh_cache():
        entries = G.list_cached_firmware()
        options = {str(i): f"{e['product_id']} {e['tag']} ({e['size']} bytes)"
                   for i, e in enumerate(entries)}
        cached.set_options(options)
        cached._entries = entries
        if options:
            cached.set_value(next(iter(options)))
        else:
            ui.notify("Nothing downloaded yet", type="info")

    def handle_use_cached():
        entries = getattr(cached, "_entries", [])
        if cached.value is None or not entries:
            ui.notify("Refresh the list and pick an image", type="warning")
            return
        entry = entries[int(cached.value)]
        base = G.manifest_app_base_address(entry["manifest"]) or F.APP_BASE_ADDRESS
        set_firmware(entry["path"].read_bytes(),
                     f"{entry['product_id']} {entry['tag']} (cached)", app_start=base)
        show_validation()

    async def confirm(question):
        with ui.dialog() as dialog, ui.card():
            ui.label(question).classes("text-lg")
            with ui.row():
                ui.button("Cancel", on_click=lambda: dialog.submit(False)).props("outline")
                ui.button("Continue", on_click=lambda: dialog.submit(True)).props("color=red")
        return await dialog

    async def handle_flash():
        if FIRMWARE["bytes"] is None:
            ui.notify("Select a firmware image first", type="warning")
            return
        result = FIRMWARE["validation"]
        if not result.ok and not override.value:
            ui.notify("This image failed validation — flashing is blocked. "
                      "Tick the override only if you are certain.",
                      type="negative", timeout=12000)
            return
        board_id = int(node.value)
        app_node_id = int(app_node.value) if not link_ids.value else board_id
        role = F.NODE_ROLES.get(board_id, "unexpected node")
        if not await confirm(f"Flash node {board_id} ({role}) with "
                             f"{FIRMWARE['description']}? The motor reboots and "
                             "loses torque."):
            return

        CANCEL.clear()
        STATE.update(running=True, fraction=0.0, stage="starting", message="")
        flash_btn.disable()
        try:
            error = await run.io_bound(do_flash, board_id, app_node_id,
                                       bool(erase.value), int(chunk.value))
        finally:
            STATE["running"] = False
            flash_btn.enable()
        if error:
            ui.notify(error, type="negative", timeout=15000)
            log_line(f"ERROR: {error}")
            return
        ui.notify(f"Node {board_id} flashed. Restart the runtime and re-home.",
                  type="positive", timeout=12000)

    def handle_cancel():
        if not STATE["running"]:
            ui.notify("Nothing is running", type="info")
            return
        CANCEL.set()
        log_line("Cancel requested — stopping after the current chunk.")
        ui.notify("Cancelling...", type="warning")

    async def handle_verify():
        info, error = await run.io_bound(do_read_info, int(node.value))
        if error:
            ui.notify(error, type="negative", timeout=10000)
            return
        if info is None:
            info_label.set_text("No answer — the board may still be in its "
                                "bootloader, or is powered off.")
            return
        text = (f"software {info['software_version']}, "
                f"hardware {info['hardware_version']}, "
                f"serial {info['serial_number']}")
        info_label.set_text(text)
        log_line(f"Node {int(node.value)}: {text}")

    # ---------------------------------------------------------- pumps
    def refresh_status():
        running = F.rti_is_running()
        rti_badge.set_text("RTI: RUNNING" if running else "RTI: stopped")
        rti_badge.props(f"color={'red' if running else 'green'}")
        open_now = BUS["bus"] is not None
        bus_badge.set_text("bus: OPEN" if open_now else "bus: closed")
        bus_badge.props(f"color={'green' if open_now else 'grey'}")
        cancel_btn.set_visibility(STATE["running"])

    def drain_state():
        if not STATE["dirty"]:
            return
        STATE["dirty"] = False
        progress.set_value(STATE["fraction"])
        page, total = STATE["page"], STATE["total_pages"]
        detail = f"  (page {page + 1}/{total})" if page is not None and total else ""
        stage_label.set_text(f"{STATE['stage']}: {STATE['message']}{detail}"
                             if STATE["stage"] else "")
        while STATE["log"]:
            log_view.push(STATE["log"].popleft())

    ui.timer(1.0, refresh_status)
    ui.timer(0.15, drain_state)
    refresh_cache()
    show_validation()


if __name__ in {"__main__", "__mp_main__"}:
    ui.run(host="0.0.0.0", port=8083, title="Motor firmware flasher",
           reload=False, show=False)
