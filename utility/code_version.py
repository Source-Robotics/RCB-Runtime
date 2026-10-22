"""Which commit of this repository a process is running (2026-10-13): RTI and the executor log it at start, and
tools/arm_smoke.py records it with every smoke run.

Read straight from the .git files — no `git` subprocess: RTI calls this after it has taken SCHED_FIFO 99 on its core,
and a child process would inherit that priority and that core.
"""
from __future__ import annotations

import os


def git_commit(repo_root):
    """(branch or None, full commit hash or None) of the checkout at repo_root, from .git/HEAD and the refs. Never
    raises: anything unreadable gives None."""
    git = os.path.join(repo_root, ".git")
    try:
        with open(os.path.join(git, "HEAD")) as fh:
            head = fh.read().strip()
    except OSError:
        return None, None
    if not head.startswith("ref: "):
        return None, head or None                        # a detached HEAD holds the hash itself
    ref = head[5:].strip()
    branch = ref[len("refs/heads/"):] if ref.startswith("refs/heads/") else ref
    try:
        with open(os.path.join(git, ref)) as fh:
            return branch, fh.read().strip() or None
    except OSError:
        pass
    try:                                                 # a ref git has packed
        with open(os.path.join(git, "packed-refs")) as fh:
            for line in fh:
                parts = line.split()
                if len(parts) == 2 and parts[1] == ref:
                    return branch, parts[0]
    except OSError:
        pass
    return branch, None


def describe(repo_root):
    """'v0.9.0, feat/x @ 80d9701c9a2b' — for a log line. The runtime's version (the VERSION file, 2026-10-21) first;
    the commit stays LAST: tools/arm_smoke.py compares the end of RTI's `code:` line with HEAD."""
    from utility.versions import runtime_version         # here, not at the top: utility.versions imports this module
    ver = runtime_version(repo_root)
    head = f"v{ver}" if ver != "unknown" else "version unknown"
    branch, sha = git_commit(repo_root)
    if sha is None:
        return f"{head}, unknown (no readable .git)"
    return f"{head}, {branch or 'detached'} @ {sha[:12]}"
