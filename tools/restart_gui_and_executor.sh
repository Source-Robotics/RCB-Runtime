#!/usr/bin/env bash
# Restart command_executor.py and nicegui_test.py TOGETHER — and ONLY those two.
#
#   tools/restart_gui_and_executor.sh             # restart both, verify, report
#   tools/restart_gui_and_executor.sh --dry-run   # show what it WOULD signal, change nothing
#
# No sudo: the supervisor (autorun.service, User=robot) runs them as you.
#
# HOW IT WORKS. supervisor.py starts every managed script with start_new_session=True, so each
# has its own process group, and its monitor (every 2 s) respawns any script that has died. This
# sends SIGTERM to the whole GROUP of just those two — the same thing the OLED menu's "Restart
# GUI" / "Restart HL" do — and lets the supervisor bring them back. Killing the group, not just
# the python PID, matters for the GUI: a leftover child holding port 8081 makes the respawn fail
# with "Address already in use" and the supervisor spirals.
#
# WHAT IT NEVER TOUCHES. RTI.py. Restarting RTI clears homing, so this proves it did not: RTI's
# PID and start time are compared before and after, and the script says so.
#
# It also refuses to signal any group that is the supervisor's own or RTI's.
set -uo pipefail

DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS=(command_executor nicegui_test)
GUI_PORT=8081
EXEC_PORT=29000

pid_of()  { pgrep -f "^(\S*/)?python3 $ROOT/$1.py" | head -1; }   # "python3" or ".venv/bin/python3" (2026-10-16)
pgid_of() { ps -o pgid= -p "$1" 2>/dev/null | tr -d ' '; }
lstart()  { ps -o lstart= -p "$1" 2>/dev/null | sed 's/  */ /g'; }

SUP_PID="$(pgrep -f '^(\S*/)?python3 supervisor.py' | head -1)"
if [ -z "$SUP_PID" ]; then
  echo "supervisor.py is not running — nothing would respawn the scripts. Aborting." >&2
  exit 1
fi
SUP_PGID="$(pgid_of "$SUP_PID")"
RTI_PID="$(pid_of RTI)"
RTI_START="$(lstart "$RTI_PID")"
echo "supervisor pid $SUP_PID (group $SUP_PGID)   RTI pid ${RTI_PID:-none} started: ${RTI_START:-n/a}"

declare -A OLD_PID
for s in "${SCRIPTS[@]}"; do
  pid="$(pid_of "$s")"
  if [ -z "$pid" ]; then
    echo "  $s: not running (the supervisor will start it if it is missing)"
    continue
  fi
  pgid="$(pgid_of "$pid")"
  if [ -z "$pgid" ] || [ "$pgid" = "$SUP_PGID" ] || [ "$pgid" = "$(pgid_of "${RTI_PID:-0}")" ]; then
    echo "  $s: pid $pid is in group '$pgid' which is the supervisor's or RTI's — REFUSING to signal it" >&2
    exit 1
  fi
  OLD_PID[$s]="$pid"
  echo "  $s: pid $pid, group $pgid -> SIGTERM the group"
  [ "$DRY" = 1 ] || kill -TERM -- "-$pgid"
done

if [ "$DRY" = 1 ]; then echo "(dry run — nothing signalled)"; exit 0; fi

echo "waiting for the supervisor to respawn them (checks every 2 s)..."
ok=1
for s in "${SCRIPTS[@]}"; do
  new=""
  for _ in $(seq 1 40); do                        # up to ~20 s
    p="$(pid_of "$s")"
    if [ -n "$p" ] && [ "$p" != "${OLD_PID[$s]:-}" ]; then new="$p"; break; fi
    sleep 0.5
  done
  if [ -n "$new" ]; then echo "  $s: back as pid $new"; else echo "  $s: did NOT come back" >&2; ok=0; fi
done

# the executor and the GUI take a few seconds to bind their ports
port_up() { ss -tln 2>/dev/null | grep -q ":$1 "; }
for spec in "executor:$EXEC_PORT" "dashboard:$GUI_PORT"; do
  name="${spec%%:*}"; port="${spec##*:}"; up=0
  for _ in $(seq 1 40); do port_up "$port" && { up=1; break; }; sleep 0.5; done
  if [ "$up" = 1 ]; then echo "  $name listening on :$port"; else echo "  $name NOT listening on :$port yet" >&2; ok=0; fi
done

NEW_RTI="$(pid_of RTI)"; NEW_START="$(lstart "$NEW_RTI")"
if [ "$NEW_RTI" = "$RTI_PID" ] && [ "$NEW_START" = "$RTI_START" ]; then
  echo "RTI untouched (same pid $RTI_PID, same start time) -> homing is preserved, no re-home needed."
else
  echo "WARNING: RTI's pid/start time changed ($RTI_PID -> $NEW_RTI). Homing is probably cleared: re-home." >&2
  ok=0
fi

[ "$ok" = 1 ] && echo "DONE: executor and dashboard restarted." || { echo "Something above did not check out." >&2; exit 1; }
