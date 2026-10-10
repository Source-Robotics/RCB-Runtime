"""The Blocks tab of the NiceGUI dashboard — the only file of the block editor that knows NiceGUI.

The editor is an iframe of /blockly/ (blockly/server.py, mounted on the dashboard's own app, so same port and
origin). The iframe sits OUTSIDE `ui.tab_panels`, below it: the panels use Vue keep-alive, which takes a hidden
panel's DOM out of the document, and an iframe taken out of the document is unloaded — it would reload on every tab
switch and lose its undo history (later: the run view). NiceGUI hides an element with the `hidden` class
(display: none), which keeps the iframe loaded; the editor resizes itself when it is shown again.

Remove: delete the add_blockly_tab lines and the 'Blocks' tab in nicegui_test.py.
"""
from nicegui import run, ui

from blockly import runs
from blockly.server import mount


def add_blockly_tab(app, tabs, tab_name, height="calc(100vh - 170px)"):
    """Mount the editor on `app` and add its iframe, shown only while the tab `tab_name` is selected.

    Call it AFTER the `with ui.tab_panels(...)` block, at the same level, so the iframe is not inside a panel.
    """
    mount(app)
    frame = ui.element("iframe").props('src="/blockly/" title="Block editor"').classes("w-full").style(
        f"height: {height}; min-height: 480px; border: 0;")
    frame.set_visibility(_tab_name(tabs.value) == tab_name)
    tabs.on_value_change(lambda e: frame.set_visibility(_tab_name(e.value) == tab_name))
    return frame


def _tab_name(value):
    """`tabs.value` is None, a tab name, or a ui.tab."""
    if value is None or isinstance(value, str):
        return value
    return getattr(value, "_props", {}).get("name")


def add_program_badge(open_blocks=None):
    """For the dashboard's status bar: while a block program runs, its name and state and a Stop for it — so a
    running program is visible, and stoppable, from every tab. A click on the name opens the Blocks tab
    (`open_blocks`). Hidden when nothing runs. Returns (the row, its refresh function) — the latter for the nets.

    The Stop is runs.stop(): the robot first, on the server's own connection, then the program (never refused)."""
    async def stop():
        await run.io_bound(runs.stop)

    with ui.row().classes("items-center gap-1") as row:
        badge = ui.badge("").props("color=primary").classes("cursor-pointer")
        ui.button("Stop program", icon="stop", on_click=stop).props("color=red size=sm")
    badge.tooltip("A block program is running — click to open the Blocks tab")
    if open_blocks is not None:
        badge.on("click", lambda: open_blocks())
    row.set_visibility(False)
    shown = {"key": None}

    def refresh():
        snap = runs.snapshot()
        key = (snap["program"], snap["state"]) if snap["running"] else None
        if key == shown["key"]:          # send the browser something only when it changed
            return
        shown["key"] = key
        row.set_visibility(key is not None)
        if key is not None:
            badge.text = f"▶ PROGRAM: {snap['program']} — {snap['state']}"
            badge.props(f"color={'orange' if snap['state'] == 'paused' else 'primary'}")

    ui.timer(0.5, refresh)
    return row, refresh
