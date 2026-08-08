#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON:-python3}"
VENV_DIR="${ROOT_DIR}/.venv"
REQUIREMENTS="${ROOT_DIR}/requirements.txt"

say() { printf '
[rcb-runtime] %s
' "$*"; }
warn() { printf '
[rcb-runtime][warn] %s
' "$*" >&2; }

confirm() {
    local prompt="$1"
    read -r -p "${prompt} [y/N] " answer
    case "${answer}" in
        y|Y|yes|YES) return 0 ;;
        *) return 1 ;;
    esac
}

say "Installing from ${ROOT_DIR}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    printf 'python3 was not found. Install Python 3.11+ first.
' >&2
    exit 1
fi

"${PYTHON_BIN}" - <<'PY_CHECK'
import sys
if sys.version_info < (3, 11):
    raise SystemExit(f"Python 3.11+ is required, found {sys.version.split()[0]}")
print(f"Python OK: {sys.version.split()[0]}")
PY_CHECK

if [[ -f /etc/os-release ]]; then
    . /etc/os-release
    say "Detected OS: ${PRETTY_NAME:-unknown}"
fi

if [[ -r /proc/device-tree/model ]]; then
    MODEL="$(tr -d '\\000' < /proc/device-tree/model)"
    say "Detected board: ${MODEL}"
    if [[ "${MODEL}" != *"Raspberry Pi 5"* ]]; then
        warn "First public runtime target is Raspberry Pi 5; continuing anyway."
    fi
else
    warn "Could not read Raspberry Pi model; continuing for development/non-Pi install."
fi

FREE_KB="$(df -Pk "${ROOT_DIR}" | awk 'NR==2 {print $4}')"
if [[ "${FREE_KB}" -lt 1048576 ]]; then
    warn "Less than 1GB free on the target filesystem. Dependency install may fail."
fi

if confirm "Install/update recommended apt packages?"; then
    sudo apt-get update
    sudo apt-get install -y         python3-venv python3-pip python3-dev build-essential         iproute2 net-tools can-utils i2c-tools
else
    warn "Skipping apt package install. Make sure Python venv, CAN, and I2C tools are present."
fi

say "Creating virtual environment at ${VENV_DIR}"
"${PYTHON_BIN}" -m venv "${VENV_DIR}"
# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

say "Installing Python tooling"
python -m pip install --upgrade pip wheel

say "Verifying required binary wheels are available"
RTB_REQUIREMENT="$(grep '^roboticstoolbox-python-source-robotics==' "${REQUIREMENTS}")"
python -m pip install --only-binary=:all: --no-deps "${RTB_REQUIREMENT}"

say "Installing Python dependencies from requirements.txt"
python -m pip install -r "${REQUIREMENTS}"

say "Creating runtime directories"
mkdir -p "${ROOT_DIR}/logs" "${ROOT_DIR}/temp_data"
touch "${ROOT_DIR}/logs/.gitkeep" "${ROOT_DIR}/temp_data/.gitkeep"

say "Running system check"
if ! "${VENV_DIR}/bin/python" "${ROOT_DIR}/scripts/check_system.py"; then
    warn "System check reported issues. Review the messages above before running the robot."
fi

say "Install finished"
printf '
Next steps:
'
printf '  1. Review config/system.xml and robots/<name>.xml
'
printf '  2. Run: ./scripts/check_system.py
'
printf '  3. Install service: ./scripts/install_service.sh
'
printf '  4. Start manually first: .venv/bin/python supervisor.py
'
