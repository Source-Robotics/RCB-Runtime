#!/bin/bash
# The system-level setup of a robot Pi: everything the repository cannot carry by itself (2026-10-16).
# README.md "Set up a new Pi" gives the order; this script checks every item and applies the ones that are files and
# system settings:
#
#   checked only (it prints how to fix them):
#     kernel     a real-time kernel (/sys/kernel/realtime = 1)
#     boot       the CAN chip, SPI, UART and I2C in /boot/firmware/config.txt; no serial console in cmdline.txt
#     python     the repo's .venv on Python 3.11 with the runtime's packages: the robot runs it (autorun.sh starts the
#                supervisor with .venv/bin/python3, which starts the rest; 2026-10-16) and so do the tests and tools
#   applied (with sudo):
#     groups     the user in dialout, i2c and gpio (the UART, the OLED's I2C, the GPIO header)
#     sudo       the commands the runtime runs with sudo, password-free: /etc/sudoers.d/10-rcb-runtime — only when the
#                user cannot already run them (the reference robot has NOPASSWD: ALL)
#     priority   cap_sys_nice on the python3 binary: RTI's SCHED_FIFO 99 on its core (without it RTI refuses to start)
#     service    autorun.service + autorun.sh next to the repo folder (systemd/*.in), enabled (not started); the
#                drop-ins memlock.conf (LimitMEMLOCK=infinity: RTI locks its memory) and, with --allowed-ips,
#                allowed-ips.conf (RCB_ALLOWED_IPS); an existing unit or autorun.sh is checked, never overwritten
#     logs       tools/system_log_limits.sh: the journal 200 MB, syslog 50 MB, logrotate hourly
#
#   bash scripts/setup_system.sh --check          # report only, no sudo; exit 1 when anything is missing
#   sudo bash scripts/setup_system.sh [--user NAME] [--allowed-ips LIST]
#
# The package check starts a python3 that imports the runtime's packages (~3 s of CPU): run it with the arm DISABLED
# (another process's start-up slows RTI's ticks on this Pi). Idempotent: a second run changes nothing.
# RCB_SYS_ROOT=<folder>: test mode — every file is read and written under that folder, no system command runs, the
# items that need the real system (python, groups, sudo, the capability, systemctl) are skipped
# (dummy_test_scripts/test_setup_system.py).
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="${RCB_SYS_ROOT:-}"
MODE="apply"
RUN_USER="${SUDO_USER:-${USER:-$(id -un)}}"
ALLOWED_IPS=""
while [ $# -gt 0 ]; do
    case "$1" in
        --check) MODE="check" ;;
        --user) RUN_USER="$2"; shift ;;
        --allowed-ips) ALLOWED_IPS="$2"; shift ;;
        -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
        *) echo "unknown option: $1 (--check, --user NAME, --allowed-ips LIST)" >&2; exit 2 ;;
    esac
    shift
done
if [ "$MODE" = "apply" ] && [ -z "$ROOT" ] && [ "$(id -u)" != "0" ]; then
    echo "apply mode needs sudo: sudo bash $0   (or: bash $0 --check)" >&2
    exit 1
fi

MISSING=0
CHANGED=0
ok()   { printf '  OK       %s\n' "$*"; }
miss() { printf '  MISSING  %s\n' "$*"; MISSING=$((MISSING + 1)); }
did()  { printf '  DONE     %s\n' "$*"; CHANGED=$((CHANGED + 1)); }
note() { printf '  NOTE     %s\n' "$*"; }
fix()  { printf '           -> %s\n' "$*"; }
skip() { printf '  SKIPPED  %s (test mode)\n' "$*"; }
real() { [ -z "$ROOT" ]; }
apply() { [ "$MODE" = "apply" ]; }
# write FILE CONTENT MODE: write when different; prints nothing, returns 0 when it wrote
write_if_changed() {
    local file="$1" content="$2" perm="$3"
    if [ -f "$file" ] && [ "$(cat "$file")" = "$content" ]; then
        return 1
    fi
    mkdir -p "$(dirname "$file")"
    printf '%s\n' "$content" > "$file.tmp" && chmod "$perm" "$file.tmp" && mv "$file.tmp" "$file"
}
render() {   # render TEMPLATE: the {{...}} placeholders filled in
    sed -e "s|{{USER}}|$RUN_USER|g" -e "s|{{REPO}}|$REPO|g" -e "s|{{AUTORUN_SH}}|$AUTORUN_SH|g" "$1"
}

echo "RCB-Runtime system setup — repo $REPO, user $RUN_USER, mode $MODE${ROOT:+ (test mode under $ROOT)}"

# ------------------------------------------------------------------ kernel
echo "kernel"
if [ "$(cat "$ROOT/sys/kernel/realtime" 2>/dev/null)" = "1" ]; then
    ok "real-time kernel"
else
    miss "a real-time kernel: /sys/kernel/realtime is not 1"
    fix "flash the real-time image — README 'OS image'"
fi

# ------------------------------------------------------------------ boot
echo "boot"
CONFIG_TXT="$ROOT/boot/firmware/config.txt"
CMDLINE_TXT="$ROOT/boot/firmware/cmdline.txt"
for want in "dtoverlay=mcp2515-can0" "dtoverlay=spi0-1cs" "enable_uart=1" "dtparam=i2c_arm=on"; do
    if grep -Eq "^[[:space:]]*${want}" "$CONFIG_TXT" 2>/dev/null; then
        ok "config.txt: $want"
    else
        miss "config.txt: $want"
        fix "README 'Boot configuration' (then reboot)"
    fi
done
if grep -Eq 'console=(serial0|ttyAMA0)' "$CMDLINE_TXT" 2>/dev/null; then
    miss "cmdline.txt still puts a serial console on the UART the control board uses"
    fix "README 'Free the UART from the console'"
elif [ -f "$CMDLINE_TXT" ]; then
    ok "cmdline.txt: no serial console"
else
    miss "cannot read $CMDLINE_TXT"
fi
if real; then
    GETTY="$(systemctl is-enabled serial-getty@ttyAMA0.service 2>/dev/null)"
    if [ "$GETTY" = "enabled" ]; then
        miss "a login prompt runs on the control board's UART (serial-getty@ttyAMA0)"
        fix "sudo systemctl disable --now serial-getty@ttyAMA0.service"
    else
        ok "no login prompt on the UART (serial-getty@ttyAMA0: ${GETTY:-not installed})"
    fi
fi

# ------------------------------------------------------------------ python
echo "python"
if real; then
    ROBOT_PY="$REPO/.venv/bin/python3"         # the robot's interpreter (2026-10-16), the tests' too
    if [ -x "$ROBOT_PY" ]; then
        ok ".venv: the robot's and the tests' Python"
    else
        miss "no .venv: the robot would start on the system python3 and ~/.local's packages, which the tests do not run"
        fix "as $RUN_USER: ./scripts/install.sh"
        ROBOT_PY="python3"
    fi
    PY_BIN="$(readlink -f "$(command -v "$ROBOT_PY")" 2>/dev/null)"
    PY_VER="$("$ROBOT_PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null)"
    if [ "$PY_VER" = "3.11" ]; then
        ok "it is Python 3.11 ($PY_BIN)"
    else
        miss "it is Python ${PY_VER:-?}, not 3.11 (every pin was verified on 3.11)"
        fix "README 'Python 3.11, not 3.12' (the deadsnakes PPA), then ./scripts/install.sh"
    fi
    MODULES="numpy scipy can zmq nicegui psutil roboticstoolbox spatialmath pinocchio coal ruckig toppra Spectral_BLDC SourceRoboticsToolbox lgpio gpiozero distro serial PIL adafruit_ssd1306 board busio"
    AS_USER=()
    [ "$(id -u)" = "0" ] && AS_USER=(sudo -u "$RUN_USER" -H)
    BAD="$("${AS_USER[@]}" "$ROBOT_PY" - "$MODULES" 2>/dev/null <<'PY'
import importlib, sys
bad = []
for m in sys.argv[1].split():
    try:
        importlib.import_module(m)
    except Exception as e:                      # noqa: BLE001
        bad.append(f"{m} ({type(e).__name__})")
print(", ".join(bad))
PY
)" || BAD="(the check itself failed)"
    if [ -z "$BAD" ]; then
        ok "the runtime's packages import"
    else
        miss "the robot's Python cannot import: $BAD"
        fix "as $RUN_USER: ./scripts/install.sh  (requirements.txt into .venv; README 'Python packages')"
    fi
else
    skip "the robot's Python (.venv), its packages"
fi

# ------------------------------------------------------------------ groups
echo "groups"
if real; then
    for g in dialout i2c gpio; do
        if id -nG "$RUN_USER" 2>/dev/null | tr ' ' '\n' | grep -qx "$g"; then
            ok "$RUN_USER is in $g"
        elif apply; then
            getent group "$g" > /dev/null || groupadd "$g"
            usermod -aG "$g" "$RUN_USER" && did "$RUN_USER added to $g (log out and in, or reboot)"
        else
            miss "$RUN_USER is not in $g"
            fix "sudo usermod -aG $g $RUN_USER"
        fi
    done
else
    skip "group membership"
fi

# ------------------------------------------------------------------ sudo
echo "sudo"
SUDOERS="$ROOT/etc/sudoers.d/10-rcb-runtime"
CAN_DEV="$(readlink -f /sys/class/net/can0/device 2>/dev/null)"
POWER="${CAN_DEV%%/spi_master/*}/power/control"
[ -n "$CAN_DEV" ] || POWER="/sys/devices/platform/axi/1000120000.pcie/1f00050000.spi/power/control"
NEEDED=(
    "/usr/sbin/ip link set can0 down"
    "/usr/sbin/ifconfig can0 txqueuelen 1000"
    "/usr/bin/taskset -pc 2 1"
    "/usr/bin/tee $POWER"
    "/usr/bin/systemctl set-property --runtime user.slice AllowedCPUs=0,1"
    "/usr/sbin/reboot"
    "/usr/sbin/poweroff"
)
sudo_missing() {     # the NEEDED commands RUN_USER cannot run without a password
    local c
    for c in "${NEEDED[@]}"; do
        # shellcheck disable=SC2086
        if [ "$(id -u)" = "0" ]; then
            sudo -u "$RUN_USER" sudo -n -l $c > /dev/null 2>&1 || echo "$c"
        else
            sudo -n -l $c > /dev/null 2>&1 || echo "$c"
        fi
    done
}
RULE="# RCB-Runtime (scripts/setup_system.sh, 2026-10-16): what the runtime runs with sudo — nothing else.
# start_can_interface (ip, ifconfig), place_can_path (taskset, tee, systemctl: the CAN path's own core), the OLED
# menu's reboot / power off (supervisor.py). The arguments of taskset, tee and systemctl are regular expressions.
$RUN_USER ALL=(root) NOPASSWD: /usr/sbin/ip, /usr/sbin/ifconfig, /usr/sbin/reboot, /usr/sbin/poweroff
$RUN_USER ALL=(root) NOPASSWD: /usr/bin/taskset ^-pc [0-9\\,-]+ [0-9]+\$
$RUN_USER ALL=(root) NOPASSWD: /usr/bin/tee ^/sys/devices/[A-Za-z0-9._\\:/-]+/power/control\$
$RUN_USER ALL=(root) NOPASSWD: /usr/bin/systemctl ^set-property --runtime user[.]slice AllowedCPUs\\=[0-9\\,-]+\$"
if real; then
    LACK="$(sudo_missing)"
    if [ -z "$LACK" ]; then
        ok "$RUN_USER runs the runtime's sudo commands without a password"
    elif apply; then
        write_if_changed "$SUDOERS" "$RULE" 0440
        if visudo -c -f "$SUDOERS" > /dev/null; then
            LACK="$(sudo_missing)"
            if [ -z "$LACK" ]; then
                did "sudo rule $SUDOERS"
            else
                miss "after $SUDOERS these still ask for a password: $(echo "$LACK" | tr '\n' ';')"
            fi
        else
            rm -f "$SUDOERS"
            miss "the sudo rule did not parse (visudo) — removed, nothing changed"
        fi
    else
        miss "these ask $RUN_USER for a password: $(echo "$LACK" | tr '\n' ';')"
        fix "sudo bash $0   (writes $SUDOERS)"
    fi
else
    if [ -f "$SUDOERS" ] && [ "$(cat "$SUDOERS")" = "$RULE" ]; then
        ok "sudo rule $SUDOERS"
    elif apply; then
        write_if_changed "$SUDOERS" "$RULE" 0440 && did "sudo rule $SUDOERS"
    else
        miss "no sudo rule $SUDOERS"
    fi
    if [ -f "$SUDOERS" ] && ! visudo -c -f "$SUDOERS" > /dev/null 2>&1; then
        miss "the sudo rule does not parse (visudo -c)"
    fi
fi

# ------------------------------------------------------------------ priority
echo "priority"
if real; then
    PY_BIN="${PY_BIN:-$(readlink -f "$(command -v python3)")}"      # .venv's python3 resolves to this binary
    if getcap "$PY_BIN" 2>/dev/null | grep -q cap_sys_nice; then
        ok "$PY_BIN has cap_sys_nice (RTI's real-time priority)"
    elif apply; then
        setcap cap_sys_nice=eip "$PY_BIN" && did "cap_sys_nice on $PY_BIN"
    else
        miss "$PY_BIN has no cap_sys_nice: RTI cannot get SCHED_FIFO 99 and refuses to start"
        fix "sudo setcap cap_sys_nice=eip $PY_BIN"
    fi
else
    skip "cap_sys_nice"
fi

# ------------------------------------------------------------------ service
echo "service"
AUTORUN_SH="$(dirname "$REPO")/autorun.sh"
UNIT="$ROOT/etc/systemd/system/autorun.service"
DROPIN="$ROOT/etc/systemd/system/autorun.service.d"
RELOAD=0
if [ -f "$ROOT$AUTORUN_SH" ]; then
    if ! grep -q "supervisor.py" "$ROOT$AUTORUN_SH"; then
        miss "$AUTORUN_SH does not start supervisor.py"
        fix "compare it with systemd/autorun.sh.in"
    elif grep -q ".venv/bin/python3" "$ROOT$AUTORUN_SH"; then
        ok "$AUTORUN_SH starts the supervisor with .venv's python3 (left as it is)"
    else
        miss "$AUTORUN_SH starts the supervisor with the system python3: the robot then runs ~/.local's packages, not the tested ones"
        fix "replace its 'python3 supervisor.py' line with the three lines at the end of systemd/autorun.sh.in"
    fi
elif apply; then
    write_if_changed "$ROOT$AUTORUN_SH" "$(render "$REPO/systemd/autorun.sh.in")" 0755 && did "$AUTORUN_SH"
    real && chown "$RUN_USER": "$AUTORUN_SH"
else
    miss "no $AUTORUN_SH"
fi
if [ -f "$UNIT" ]; then
    if grep -q "^ExecStart=.*$AUTORUN_SH" "$UNIT" && grep -q "^User=$RUN_USER\$" "$UNIT"; then
        ok "autorun.service runs $AUTORUN_SH as $RUN_USER (left as it is)"
    else
        miss "autorun.service exists but does not run $AUTORUN_SH as $RUN_USER"
        fix "compare /etc/systemd/system/autorun.service with systemd/autorun.service.in"
    fi
elif apply; then
    write_if_changed "$UNIT" "$(render "$REPO/systemd/autorun.service.in")" 0644 && did "autorun.service" && RELOAD=1
else
    miss "no autorun.service"
fi
MEMLOCK="[Service]
LimitMEMLOCK=infinity"
if [ -f "$DROPIN/memlock.conf" ] && [ "$(cat "$DROPIN/memlock.conf")" = "$MEMLOCK" ]; then
    ok "LimitMEMLOCK=infinity (autorun.service.d/memlock.conf: RTI locks its memory)"
elif apply; then
    write_if_changed "$DROPIN/memlock.conf" "$MEMLOCK" 0644 && did "autorun.service.d/memlock.conf" && RELOAD=1
else
    miss "no autorun.service.d/memlock.conf: RTI cannot lock its memory"
fi
# RCB_ALLOWED_IPS may be set by any drop-in (this robot's is allow-any-ip.conf); systemd reads them in name order and
# the last one wins
read_ips() {       # NOW_IPS / IPS_FILE: the value in force — the last drop-in (by name) that sets it
    local f
    NOW_IPS=""
    IPS_FILE=""
    for f in "$DROPIN"/*.conf; do
        [ -f "$f" ] || continue
        if grep -q "RCB_ALLOWED_IPS=" "$f"; then
            NOW_IPS="$(sed -n 's/.*RCB_ALLOWED_IPS=\([^"]*\)".*/\1/p' "$f" | tail -1)"
            IPS_FILE="$(basename "$f")"
        fi
    done
}
ips_files() {      # every drop-in that sets it
    local f
    for f in "$DROPIN"/*.conf; do
        [ -f "$f" ] && grep -q "RCB_ALLOWED_IPS=" "$f" && basename "$f"
    done
}
read_ips
if [ -n "$ALLOWED_IPS" ]; then
    IPS="[Service]
Environment=\"RCB_ALLOWED_IPS=$ALLOWED_IPS\""
    if [ "$NOW_IPS" = "$ALLOWED_IPS" ]; then
        ok "RCB_ALLOWED_IPS=$ALLOWED_IPS ($IPS_FILE)"
    elif apply; then
        write_if_changed "$DROPIN/allowed-ips.conf" "$IPS" 0644 && RELOAD=1
        read_ips
        if [ "$NOW_IPS" = "$ALLOWED_IPS" ]; then
            did "RCB_ALLOWED_IPS=$ALLOWED_IPS (allowed-ips.conf)"
        else
            miss "RCB_ALLOWED_IPS is still $NOW_IPS: $IPS_FILE comes after allowed-ips.conf and sets it — remove it"
        fi
    else
        miss "RCB_ALLOWED_IPS is ${NOW_IPS:-not set}, not $ALLOWED_IPS"
    fi
elif [ -n "$NOW_IPS" ]; then
    note "RCB_ALLOWED_IPS=$NOW_IPS ($IPS_FILE; '*' = any machine — isolated networks only)"
else
    note "no RCB_ALLOWED_IPS: the executor accepts only its built-in list (command_executor.py); --allowed-ips to add"
fi
if [ "$(ips_files | wc -l)" -gt 1 ]; then
    note "RCB_ALLOWED_IPS is set in $(ips_files | tr '\n' ' ')— the last one wins; keep one"
fi
if real; then
    if [ "$RELOAD" = "1" ]; then
        systemctl daemon-reload
    fi
    if [ "$(systemctl is-enabled autorun.service 2>/dev/null)" = "enabled" ]; then
        ok "autorun.service enabled (starts at boot)"
    elif apply; then
        systemctl enable autorun.service > /dev/null 2>&1 && did "autorun.service enabled (not started: reboot, or sudo systemctl start autorun.service)"
    else
        miss "autorun.service is not enabled"
        fix "sudo systemctl enable autorun.service"
    fi
    if [ "$(systemctl is-enabled rcb-runtime.service 2>/dev/null)" = "enabled" ]; then
        miss "rcb-runtime.service is ALSO enabled: two supervisors would start"
        fix "sudo systemctl disable rcb-runtime.service"
    fi
    if [ "$RELOAD" = "1" ] && [ "$(systemctl is-active autorun.service 2>/dev/null)" = "active" ]; then
        note "autorun.service is running: the changes apply at its next restart (sudo systemctl restart autorun.service, then home the arm)"
    fi
else
    skip "systemctl (daemon-reload, enable)"
fi

# ------------------------------------------------------------------ logs
echo "logs"
JOURNAL="$ROOT/etc/systemd/journald.conf.d/rcb.conf"
RSYSLOG="$ROOT/etc/logrotate.d/rsyslog"
TIMER="$ROOT/etc/systemd/system/logrotate.timer.d/hourly.conf"
if grep -q "SystemMaxUse=200M" "$JOURNAL" 2>/dev/null && grep -q "OnCalendar=hourly" "$TIMER" 2>/dev/null \
        && { [ ! -f "$RSYSLOG" ] || grep -q "maxsize" "$RSYSLOG"; }; then
    ok "journal 200 MB, syslog 50 MB, logrotate hourly"
elif apply; then
    if RCB_SYS_ROOT="$ROOT" bash "$REPO/tools/system_log_limits.sh" > /dev/null; then
        did "journal / syslog limits (tools/system_log_limits.sh)"
    else
        miss "tools/system_log_limits.sh failed — run it alone to see why"
    fi
else
    miss "the journal / syslog limits"
    fix "sudo bash tools/system_log_limits.sh"
fi

# ------------------------------------------------------------------ not in git
echo "local material"
for d in docs TODO dummy_test_scripts; do
    if [ -d "$REPO/$d" ]; then
        ok "$d/ is here"
    else
        note "$d/ is not here — it is not in git (only README.md and docs/pin_map.md are): copy it from the reference Pi"
    fi
done
if real && ! modinfo vcan > /dev/null 2>&1; then
    note "no vcan kernel module: the simulator (tools/sim_run.py) cannot run here"
fi

echo
if [ "$MODE" = "check" ]; then
    echo "$MISSING missing"
    [ "$MISSING" = "0" ]
    exit $?
fi
echo "$CHANGED changed, $MISSING still missing"
[ "$MISSING" = "0" ]
