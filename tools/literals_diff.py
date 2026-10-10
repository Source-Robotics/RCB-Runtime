#!/usr/bin/env python3
"""Proof helper for a function split: the NEW module must contain exactly the OLD module's string literals
(every log message, every error key, every shared-memory key — the things a retype can corrupt), and every
module-level function of the old module must still exist with an identical signature.

    python literals_diff.py OLD.py NEW.py [--allow-removed "lit1" "lit2" ...]

Docstrings (the first string statement of a module / class / function body) are excluded, so new helpers may
have docstrings and old docstrings may be edited. Comments are not in the AST. f-string pieces ARE counted.
Exit 1 on any difference.
"""
import ast
import collections
import sys


def literals(path):
    tree = ast.parse(open(path).read())
    doc_ids = set()
    for n in ast.walk(tree):
        if isinstance(n, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and n.body \
                and isinstance(n.body[0], ast.Expr) and isinstance(n.body[0].value, ast.Constant) \
                and isinstance(n.body[0].value.value, str):
            doc_ids.add(id(n.body[0].value))
    c = collections.Counter()
    for n in ast.walk(tree):
        if isinstance(n, ast.Constant) and isinstance(n.value, str) and id(n) not in doc_ids:
            c[n.value] += 1
    sigs = {n.name: ast.dump(n.args) + "|" + ast.dump(n.returns) if n.returns else ast.dump(n.args)
            for n in tree.body if isinstance(n, ast.FunctionDef)}
    return c, sigs


def main():
    argv = sys.argv[1:]
    allow = set()
    if "--allow-removed" in argv:
        i = argv.index("--allow-removed")
        allow = set(argv[i + 1:])
        argv = argv[:i]
    old, new = argv
    lo, so = literals(old)
    ln, sn = literals(new)
    removed = {k: v for k, v in (lo - ln).items() if k not in allow}
    added = ln - lo
    bad = False
    if removed:
        bad = True
        print(f"REMOVED/CHANGED literals ({len(removed)}):")
        for k, v in sorted(removed.items()):
            print(f"   -{v} {k!r}")
    if added:
        print(f"ADDED literals ({len(added)}) — each must be a NEW helper's own text, never a retyped old one:")
        for k, v in sorted(added.items()):
            print(f"   +{v} {k!r}")
    for name, sig in so.items():
        if name not in sn:
            bad = True
            print(f"MISSING function: {name}")
        elif sn[name] != sig:
            bad = True
            print(f"SIGNATURE CHANGED: {name}")
    print(("FAIL" if bad else "OK") + f": {len(lo)} distinct old literals, {len(so)} module-level functions checked")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
