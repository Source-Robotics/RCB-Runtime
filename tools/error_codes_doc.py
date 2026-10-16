#!/usr/bin/env python3
"""Write docs/error_codes.md — every error, warning and event code the robot reports — from utility/error_catalog.py.

The operator's directory, like an industrial controller's error-code manual: by group, each code's number, kind,
title, cause, effect and fix. Generated, never edited by hand; dummy_test_scripts/test_error_catalog.py fails when
it is out of date.

    .venv/bin/python tools/error_codes_doc.py            # write it
    .venv/bin/python tools/error_codes_doc.py --check    # exit 1 if it is out of date
"""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
from utility.error_catalog import CATALOG, GROUPS  # noqa: E402

DOC = os.path.join(_ROOT, "docs", "error_codes.md")


def render():
    by_group = {}
    for code, e in sorted(CATALOG.items(), key=lambda kv: kv[1]["number"]):
        by_group.setdefault(e["number"] // 1000, []).append((code, e))
    out = ["# Error and event codes", "",
           "Generated from `utility/error_catalog.py` by `tools/error_codes_doc.py`. Do not edit by hand.", "",
           "Every code has a stable number. The number and the code appear together in the dashboard's Errors and "
           "Activity Log tabs, in RTI.log, and in a client's error replies (`\"number\"` next to `\"code\"`). The "
           "writing rules are in `docs/logging.md`, section 14.", "",
           "Kind: **stop** the robot stops and is disabled; **fails** the command, move or program fails or is "
           "refused, and the robot is not affected; **warning** the robot keeps running; **event** information "
           "only.", ""]
    for g in sorted(by_group):
        out += [f"## {g}xxx {GROUPS[g]}", "", "| No. | Code | Kind | Title |", "|---|---|---|---|"]
        out += [f"| {e['number']} | `{code}` | {e['kind']} | {e['title']} |" for code, e in by_group[g]]
        out.append("")
        for code, e in by_group[g]:
            if e["cause"] is None:
                continue
            out += [f"### {e['number']} {code}: {e['title']}", "",
                    f"- **Cause:** {e['cause']}", f"- **Effect:** {e['effect']}", f"- **Fix:** {e['fix']}", ""]
    return "\n".join(out).rstrip("\n") + "\n"


def main():
    text = render()
    if "--check" in sys.argv:
        current = open(DOC, encoding="utf-8").read() if os.path.exists(DOC) else ""
        if current != text:
            print("docs/error_codes.md is out of date: run .venv/bin/python tools/error_codes_doc.py")
            return 1
        print("docs/error_codes.md is up to date")
        return 0
    with open(DOC, "w", encoding="utf-8") as fh:
        fh.write(text)
    print(f"wrote {os.path.relpath(DOC, _ROOT)}: {len(CATALOG)} codes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
