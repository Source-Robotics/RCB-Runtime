#!/usr/bin/env python3
"""Print one version's section of CHANGELOG.md: the GitHub release notes of its tag (2026-10-21, versioning V5).

    tools/release_notes.py            the section of the version in the VERSION file
    tools/release_notes.py 0.9.0      the section of 0.9.0
    tools/release_notes.py --list     every version in the changelog, newest first

The notes of a release are exactly its changelog section, so the two cannot disagree. For example, with the GitHub
command-line tool:

    tools/release_notes.py 0.9.0 > /tmp/notes.md
    gh release create v0.9.0 --title "0.9.0" --notes-file /tmp/notes.md

Exit 1 when the changelog has no section for that version.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CHANGELOG = ROOT / "CHANGELOG.md"
# "## [0.9.0] - 2026-10-21" -> version, date; "## [Unreleased]" -> "Unreleased", None.
HEADING = re.compile(r"^## \[(?P<version>[^\]]+)\](?: - (?P<date>\d{4}-\d{2}-\d{2}))?\s*$")


def sections(text: str) -> list[dict]:
    """Every "## [version]" section of a changelog, in file order: {"version", "date", "body"} (the body without its
    heading, surrounding blank lines removed)."""
    out, current = [], None
    for line in text.splitlines():
        m = HEADING.match(line)
        if m:
            current = {"version": m.group("version"), "date": m.group("date"), "lines": []}
            out.append(current)
        elif line.startswith("## "):
            current = None                       # some other second-level heading ends the section
        elif current is not None:
            current["lines"].append(line)
    for s in out:
        s["body"] = "\n".join(s.pop("lines")).strip("\n")
    return out


def notes_for(version: str, text: str) -> str | None:
    """The body of `version`'s section, or None when there is none."""
    for s in sections(text):
        if s["version"] == version:
            return s["body"]
    return None


def main(argv: list[str]) -> int:
    text = CHANGELOG.read_text(encoding="utf-8")
    if argv and argv[0] == "--list":
        for s in sections(text):
            print(s["version"], s["date"] or "")
        return 0
    version = argv[0] if argv else (ROOT / "VERSION").read_text(encoding="utf-8").strip()
    body = notes_for(version, text)
    if body is None:
        print(f"CHANGELOG.md has no section for {version}", file=sys.stderr)
        return 1
    print(body)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
