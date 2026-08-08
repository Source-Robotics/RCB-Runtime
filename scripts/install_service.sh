#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE="${ROOT_DIR}/systemd/rcb-runtime.service.in"
SERVICE_NAME="rcb-runtime.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

say() { printf '
[rcb-runtime] %s
' "$*"; }
confirm() {
    local prompt="$1"
    read -r -p "${prompt} [y/N] " answer
    case "${answer}" in
        y|Y|yes|YES) return 0 ;;
        *) return 1 ;;
    esac
}

if [[ ! -f "${TEMPLATE}" ]]; then
    printf 'Missing service template: %s
' "${TEMPLATE}" >&2
    exit 1
fi

if [[ ! -x "${ROOT_DIR}/.venv/bin/python" ]]; then
    printf 'Missing virtualenv python at %s/.venv/bin/python. Run ./scripts/install.sh first.
' "${ROOT_DIR}" >&2
    exit 1
fi

RUN_USER="${RCB_RUNTIME_USER:-${SUDO_USER:-${USER}}}"
RUN_GROUP="$(id -gn "${RUN_USER}")"
TMP_FILE="$(mktemp)"
trap 'rm -f "${TMP_FILE}"' EXIT

python3 - <<PY_RENDER
from pathlib import Path
text = Path(${TEMPLATE@Q}).read_text(encoding='utf-8')
text = text.replace('{{REPO_ROOT}}', ${ROOT_DIR@Q})
text = text.replace('{{USER}}', ${RUN_USER@Q})
text = text.replace('{{GROUP}}', ${RUN_GROUP@Q})
Path(${TMP_FILE@Q}).write_text(text, encoding='utf-8')
PY_RENDER

say "Installing ${SERVICE_NAME} for user ${RUN_USER} from ${ROOT_DIR}"
sudo install -m 0644 "${TMP_FILE}" "${SERVICE_PATH}"
sudo systemctl daemon-reload

say "Installed ${SERVICE_PATH}"

if confirm "Enable ${SERVICE_NAME} at boot?"; then
    sudo systemctl enable "${SERVICE_NAME}"
fi

if confirm "Start ${SERVICE_NAME} now?"; then
    sudo systemctl start "${SERVICE_NAME}"
    sudo systemctl status --no-pager "${SERVICE_NAME}" || true
else
    printf '
Start later with:
  sudo systemctl start %s
' "${SERVICE_NAME}"
fi
