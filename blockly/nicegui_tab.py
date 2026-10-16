"""The Blocks tab of the NiceGUI dashboard — the only file of the block editor that knows NiceGUI.

The editor is an iframe of /blockly/ (blockly/server.py, mounted on the dashboard's own app, so same port and
origin). The iframe sits OUTSIDE `ui.tab_panels`, below it: the panels use Vue keep-alive, which takes a hidden
panel's DOM out of the document, and an iframe taken out of the document is unloaded — it would reload on every tab
switch and lose its undo history (later: the run view). NiceGUI hides an element with the `hidden` class
(display: none), which keeps the iframe loaded; the editor resizes itself when it is shown again.

Remove: delete the add_blockly_tab lines and the 'Blocks' tab in nicegui_test.py.
"""
from nicegui import run, ui

from blockly import run_records, runs
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


def add_runs_card(tabs, tab_name, fmt_time, limit=20):
    """The program run history under the editor (blockly/run_records.py, 2026-10-15): the newest `limit` runs —
    program, start, duration, result, blocks, the error — shown only while the tab `tab_name` is selected, re-read
    every 5 s then. Call it right after add_blockly_tab. Returns (the expansion, its refresh) for the nets."""
    columns = [{"name": "program", "label": "Program", "field": "program", "align": "left"},
               {"name": "started", "label": "Started", "field": "started", "align": "left"},
               {"name": "duration", "label": "Duration", "field": "duration", "align": "right"},
               {"name": "result", "label": "Result", "field": "result", "align": "left"},
               {"name": "blocks", "label": "Blocks", "field": "blocks", "align": "right"},
               {"name": "error", "label": "Error", "field": "error", "align": "left"}]
    with ui.expansion("Program runs (the newest 20; every run is a file in logs/block_runs/)", icon="history")             .classes("w-full") as box:
        table = ui.table(columns=columns, rows=[], row_key="file").classes("w-full").props("dense flat")
    box.set_visibility(_tab_name(tabs.value) == tab_name)
    tabs.on_value_change(lambda e: box.set_visibility(_tab_name(e.value) == tab_name))
    shown = {"rows": None}

    def refresh():
        if not box.visible:
            return
        rows = run_rows(run_records.history(limit), fmt_time)
        if rows == shown["rows"]:
            return
        shown["rows"] = rows
        table.rows = rows
        table.update()

    ui.timer(5.0, refresh)
    refresh()
    return box, refresh


def run_rows(history, fmt_time):
    """Table rows from run_records.history()."""
    out = []
    for h in history:
        err = h.get("error") or {}
        out.append({"file": h["file"], "program": h.get("program"),
                    "started": fmt_time(h["started"]) if h.get("started") else "",
                    "duration": f"{h['duration_s']:.1f} s" if h.get("duration_s") is not None else "",
                    "result": h.get("result"), "blocks": h.get("blocks") if h.get("blocks") is not None else "",
                    "error": (f"{err.get('code')}: {err.get('message')}" if isinstance(err, dict) and err.get("code") else "")})
    return out


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
