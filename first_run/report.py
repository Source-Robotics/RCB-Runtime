"""The first-run check's report (2026-10-18): temp_data/first_run/<UTC>/report.json + report.txt — what a builder
keeps, and sends when asking for help (the support bundle joins it in P4 of TODO/first_run_plan.md).

    write(results, meta, root=REPORTS_DIR) -> folder      the newest KEEP_REPORTS folders are kept
    text(results, meta) -> str                           the readable report
"""
from __future__ import annotations

import json
import os
import time

from first_run.core import FAIL, STAGE_TITLES, WARN, counts, worst
from project_paths import TEMP_DATA_DIR
from utility.housekeeping import prune_folders

REPORTS_DIR = TEMP_DATA_DIR / "first_run"
KEEP_REPORTS = 50          # a report is a few KB; housekeeping.KEPT_DIRS names the folder


def stamp(t) -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.gmtime(t))


def text(results, meta) -> str:
    c = counts(results)
    lines = [f"RCB first-run check — {time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime(meta.get('time', 0)))}",
             f"Robot {meta.get('robot', '?')} ({meta.get('joint_num', '?')} joints), gripper {meta.get('gripper', '?')}, "
             f"PCB v{meta.get('pcb_version', '?')}; code {meta.get('code', '?')}; host {meta.get('host', '?')}",
             f"Result: {worst(results)} — " + ", ".join(f"{c[v]} {v}" for v in c if c[v])
             + ("" if meta.get("completed", True) else " (stopped before the end)"), ""]
    stage = None
    for r in results:
        if r["stage"] != stage:
            stage = r["stage"]
            lines.append(f"Stage {stage} — {STAGE_TITLES.get(stage, '')}")
        lines.append(f"  {r['verdict']:<4}  {r['title']} — {r['summary']}")
        if r["verdict"] in (FAIL, WARN):
            if r.get("expected") is not None:
                lines.append(f"        expected: {r['expected']}")
            lines += [f"        check: {h}" for h in r.get("hints", [])]
    lines += ["", "Asking for help? Send this folder (report.json + report.txt)."]
    return "\n".join(lines) + "\n"


def write(results, meta, root=REPORTS_DIR) -> str:
    folder = os.path.join(str(root), stamp(meta.get("time", time.time())))
    os.makedirs(folder, exist_ok=True)
    doc = {"meta": meta, "verdict": worst(results), "counts": counts(results), "results": results}
    with open(os.path.join(folder, "report.json"), "w") as f:
        json.dump(doc, f, indent=1, default=str)
    with open(os.path.join(folder, "report.txt"), "w") as f:
        f.write(text(results, meta))
    prune_folders(str(root), KEEP_REPORTS)
    return folder
