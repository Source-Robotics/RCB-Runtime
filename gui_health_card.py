"""Health card — the robot's health summary at the top of the System Info tab (utility/health.py, 2026-10-15).

One badge for the whole robot (OK / WARN / ERROR / STALE) and one row per component with its level and one line
saying why; the values behind it in a tooltip. Refreshed every 2 s while the tab is shown, in a worker thread (the
assessment waits ~50 ms for the counters to advance). Nothing here commands the robot.
"""
from __future__ import annotations

REFRESH_S = 2.0
COLORS = {"OK": "green", "WARN": "orange", "ERROR": "red", "STALE": "grey"}


def rows_of(summary):
    """Table rows from assess()'s result."""
    return [{"name": c["name"], "level": c["level"], "summary": c["summary"],
             "values": ", ".join(f"{k}={v}" for k, v in c.get("values", {}).items())} for c in summary["components"]]


def add_health_card(assess_fn, is_visible):
    """Build the card. `assess_fn()` -> utility.health.assess's dict; `is_visible()` -> whether the tab is shown.
    Returns (refresh coroutine function, handles) for the nets."""
    from nicegui import run, ui

    columns = [{"name": "name", "label": "Component", "field": "name", "align": "left"},
               {"name": "level", "label": "Level", "field": "level", "align": "left"},
               {"name": "summary", "label": "Status", "field": "summary", "align": "left"}]
    with ui.card().classes("w-full mb-3"):
        with ui.row().classes("items-center gap-3"):
            ui.label("Health").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
            badge = ui.badge("—").props("color=grey")
            stamp = ui.label("").classes("text-xs text-gray-500")
        table = ui.table(columns=columns, rows=[], row_key="name").classes("w-full").props("dense flat")
        table.add_slot("body-cell-level", '''
            <q-td :props="props">
              <q-badge :color="{OK: 'green', WARN: 'orange', ERROR: 'red', STALE: 'grey'}[props.value] || 'grey'">
                {{ props.value }}
              </q-badge>
            </q-td>''')
        table.add_slot("body-cell-summary", '''
            <q-td :props="props">{{ props.value }}<q-tooltip>{{ props.row.values }}</q-tooltip></q-td>''')
    state = {"busy": False, "last": None}

    async def refresh():
        if state["busy"] or not is_visible():
            return
        state["busy"] = True
        try:
            summary = await run.io_bound(assess_fn)
        except Exception as exc:          # the card must never take the tab down
            badge.text, stamp.text = "?", f"health unavailable: {exc}"
            badge.props("color=grey")
            return
        finally:
            state["busy"] = False
        state["last"] = summary
        badge.text = summary["level"]
        badge.props(f"color={COLORS.get(summary['level'], 'grey')}")
        import time
        stamp.text = "checked " + time.strftime("%H:%M:%S")
        table.rows = rows_of(summary)
        table.update()

    ui.timer(REFRESH_S, refresh)
    return refresh, {"badge": badge, "table": table, "state": state}
