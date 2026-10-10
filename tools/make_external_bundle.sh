#!/usr/bin/env bash
# Package the off-robot SOFT_RTI test suite for a PC that is not this robot.
#
# The bundle is deliberately tiny and dependency-free: robotclient is stdlib-only, so the
# target machine needs Python 3 and nothing else -- no pip, no virtualenv, no build.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/temp_data/rcb_external_tests.tar.gz}"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
PKG="$STAGE/rcb_external_tests"

mkdir -p "$PKG/api_tests/external" "$PKG/robotclient" "$PKG/robot_side"

# The PC-side client. Stdlib only -- verified below, not assumed.
# *.py only: __pycache__ would ship this robot's 3.11 bytecode to a PC that may run a
# different Python, where it is at best ignored and at worst confusing.
cp "$ROOT"/robotclient/*.py "$PKG/robotclient/"
# The suite, plus the __init__ files its import path needs.
cp "$ROOT"/api_tests/external/*.py "$PKG/api_tests/external/"
# README.md is optional: the repo's .gitignore keeps markdown local (public-repo cleanup), so a fresh
# clone does not have it and `set -e` would otherwise abort the whole bundle.
[ -f "$ROOT/api_tests/external/README.md" ] && cp "$ROOT/api_tests/external/README.md" "$PKG/api_tests/external/"
touch "$PKG/api_tests/__init__.py" "$PKG/api_tests/external/__init__.py"
# Goes on the ROBOT, not the PC -- no shipped recipe carries j_p + j_t + rti_seq together.
cp "$ROOT/data_recipe/external_test.xml" "$PKG/robot_side/"
[ -f "$ROOT/api_tests/external/README.md" ] && cp "$ROOT/api_tests/external/README.md" "$PKG/README.md"

cat > "$PKG/robot_side/INSTALL.txt" <<'TXT'
external_test.xml goes in the ROBOT's data_recipe/ directory.

    scp external_test.xml robot@<robot-ip>:~/Desktop/RCB-Runtime/data_recipe/

RecipeManager scans that directory once at startup, so a file copied afterwards is invisible
until a reload. e1_link.py calls reload_recipes for you and then VERIFIES the recipe took by
looking at what the packets contain -- set_recipe refuses an unknown name with only a log
line the caller never sees, so "accepted" does not mean "applied".
TXT

# Refuse to ship a bundle that would need pip on the target.
THIRD="$(cd "$PKG" && python3 -c '
import sys, json
before = set(sys.modules)
import robotclient, robotclient.rti_client
new = {m.split(".")[0] for m in set(sys.modules) - before}
print(json.dumps(sorted(new - set(sys.stdlib_module_names) - {"robotclient"})))' 2>/dev/null || echo '["<check failed>"]')"
if [ "$THIRD" != "[]" ]; then
  echo "REFUSING: the bundle would need third-party modules on the target: $THIRD" >&2
  echo "robotclient is supposed to be stdlib-only; something has regressed." >&2
  exit 1
fi

mkdir -p "$(dirname "$OUT")"
find "$STAGE" -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null || true
tar -czf "$OUT" -C "$STAGE" rcb_external_tests
echo "wrote $OUT  ($(du -h "$OUT" | cut -f1))"
echo
echo "On the PC:"
echo "    tar -xzf $(basename "$OUT") && cd rcb_external_tests"
echo "    python3 api_tests/external/e1_link.py --host <robot-ip>"
echo
echo "On the ROBOT (once): copy robot_side/external_test.xml into data_recipe/"
