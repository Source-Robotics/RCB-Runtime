#!/bin/bash
# The system's own logs on the SD card, capped (2026-10-15):
#   1. the journal at 200 MB         /etc/systemd/journald.conf.d/rcb.conf: SystemMaxUse=200M
#   2. syslog also rotated at 50 MB  "maxsize 50M" in the block of /etc/logrotate.d/rsyslog (weekly, 4 kept, as before)
#   3. logrotate run hourly          /etc/systemd/system/logrotate.timer.d/hourly.conf (daily otherwise: a log flood
#                                    would grow syslog for up to a day before the size rule is looked at)
# Rotation by time ("daily", "weekly") is unchanged by the hourly run: logrotate rotates those only when their
# time has passed. The original rsyslog file is kept as /var/backups/rsyslog.logrotate.rcb-orig (never inside
# /etc/logrotate.d/, where logrotate would read a copy as a second, duplicate configuration).
#
# Run once:   sudo bash tools/system_log_limits.sh
# Idempotent: a second run changes nothing. RCB_SYS_ROOT=<folder> runs it against a copy of the files (the test
# does), without sudo and without systemctl.
set -euo pipefail

ROOT="${RCB_SYS_ROOT:-}"
if [ -z "$ROOT" ] && [ "$(id -u)" != "0" ]; then
    echo "run it with sudo: sudo bash $0" >&2
    exit 1
fi

JOURNAL_DIR="$ROOT/etc/systemd/journald.conf.d"
RSYSLOG="$ROOT/etc/logrotate.d/rsyslog"
TIMER_DIR="$ROOT/etc/systemd/system/logrotate.timer.d"
BACKUP="$ROOT/var/backups/rsyslog.logrotate.rcb-orig"

# 1. the journal
mkdir -p "$JOURNAL_DIR"
printf '[Journal]\nSystemMaxUse=200M\n' > "$JOURNAL_DIR/rcb.conf"
echo "journal: SystemMaxUse=200M ($JOURNAL_DIR/rcb.conf)"

# 2. syslog's size rule
if [ ! -f "$RSYSLOG" ]; then
    echo "syslog: $RSYSLOG not found — skipped (no rsyslog on this system?)"
elif grep -q 'maxsize' "$RSYSLOG"; then
    echo "syslog: $RSYSLOG already has a maxsize rule — unchanged"
else
    mkdir -p "$(dirname "$BACKUP")"
    [ -f "$BACKUP" ] || cp -p "$RSYSLOG" "$BACKUP"
    sed -i 's/^\([[:space:]]*\)weekly[[:space:]]*$/&\n\1maxsize 50M/' "$RSYSLOG"
    if [ "$(grep -c 'maxsize 50M' "$RSYSLOG")" != "1" ]; then
        cp -p "$BACKUP" "$RSYSLOG"
        echo "syslog: the edit did not land exactly once — the original was put back; nothing changed" >&2
        exit 1
    fi
    echo "syslog: maxsize 50M added to $RSYSLOG (original: $BACKUP)"
fi

# 3. logrotate hourly
mkdir -p "$TIMER_DIR"
printf '[Timer]\nOnCalendar=\nOnCalendar=hourly\nAccuracySec=1m\n' > "$TIMER_DIR/hourly.conf"
echo "logrotate: hourly ($TIMER_DIR/hourly.conf)"

if [ -z "$ROOT" ]; then
    logrotate -d /etc/logrotate.conf > /dev/null 2>&1 || {
        echo "logrotate's configuration check FAILED — restoring the original rsyslog rules" >&2
        [ -f "$BACKUP" ] && cp -p "$BACKUP" "$RSYSLOG"
        exit 1
    }
    systemctl daemon-reload
    systemctl restart logrotate.timer
    systemctl restart systemd-journald
    journalctl --vacuum-size=200M > /dev/null 2>&1 || true
    echo
    journalctl --disk-usage
    systemctl list-timers logrotate.timer --no-pager | head -3
fi
