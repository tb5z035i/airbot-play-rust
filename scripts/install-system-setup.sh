#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Install AIRBOT host setup files from a source checkout.

Usage:
  sudo ./scripts/install-system-setup.sh [--prefix /usr/local] [--skip-modprobe] [--skip-reload]

Options:
  --prefix PATH     Install helper scripts under PATH/bin. Default: /usr/local
  --skip-modprobe   Do not attempt to load CAN kernel modules
  --skip-reload     Do not reload udev rules or systemd units after install
  -h, --help        Show this help text
EOF
}

PREFIX="/usr/local"
SKIP_MODPROBE=0
SKIP_RELOAD=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --prefix)
            PREFIX="${2:?missing value for --prefix}"
            shift 2
            ;;
        --skip-modprobe)
            SKIP_MODPROBE=1
            shift
            ;;
        --skip-reload)
            SKIP_RELOAD=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [[ "${EUID}" -ne 0 ]]; then
    echo "This script must be run as root. Use sudo to run it." >&2
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

BIN_DIR="${PREFIX%/}/bin"
UDEV_DIR="/etc/udev/rules.d"
SYSTEMD_DIR="/etc/systemd/system"

install_file() {
    local src="$1"
    local dest="$2"
    local mode="$3"

    install -d "$(dirname "$dest")"
    install -m "$mode" "$src" "$dest"
}

render_file() {
    local src="$1"
    local dest="$2"
    local mode="$3"
    local tmp

    install -d "$(dirname "$dest")"
    tmp="$(mktemp)"
    sed \
        -e "s|/usr/bin/env can_add.sh|${BIN_DIR}/can_add.sh|g" \
        -e "s|ExecStart=/usr/bin/env slcan_add.sh %I|ExecStart=${BIN_DIR}/slcan_add.sh %I|g" \
        -e "s|/usr/local/bin|${BIN_DIR}|g" \
        "$src" >"$tmp"
    install -m "$mode" "$tmp" "$dest"
    rm -f "$tmp"
}

maybe_load_module() {
    local module="$1"
    if [[ -r /proc/modules ]] && grep -q "^${module} " /proc/modules; then
        return 0
    fi

    if command -v modprobe >/dev/null 2>&1; then
        echo "Trying to load kernel module ${module}"
        modprobe "$module" || echo "Warning: failed to load module ${module}" >&2
    fi
}

install_file "${REPO_ROOT}/root/bin/can_add.sh" "${BIN_DIR}/can_add.sh" 0755
install_file "${REPO_ROOT}/root/bin/slcan_add.sh" "${BIN_DIR}/slcan_add.sh" 0755
render_file "${REPO_ROOT}/root/bin/bind_airbot_device" "${BIN_DIR}/bind_airbot_device" 0755
render_file "${REPO_ROOT}/root/lib/udev/rules.d/90-usb-can.rules" "${UDEV_DIR}/90-usb-can.rules" 0644
install_file "${REPO_ROOT}/root/lib/udev/rules.d/90-usb-slcan.rules" "${UDEV_DIR}/90-usb-slcan.rules" 0644
render_file "${REPO_ROOT}/root/lib/systemd/system/slcan@.service" "${SYSTEMD_DIR}/slcan@.service" 0644

if [[ "${SKIP_MODPROBE}" -eq 0 ]]; then
    maybe_load_module can
    maybe_load_module can_raw
    maybe_load_module slcan
    maybe_load_module can_dev
fi

if [[ "${SKIP_RELOAD}" -eq 0 ]]; then
    if command -v udevadm >/dev/null 2>&1; then
        udevadm control --reload-rules || true
        udevadm trigger || true
    fi

    if command -v systemctl >/dev/null 2>&1; then
        systemctl daemon-reload || true
        systemctl restart systemd-udevd.service || systemctl restart udev || true
    fi
fi

cat <<EOF
Installed AIRBOT source-checkout host setup:
  helpers: ${BIN_DIR}
  udev rules: ${UDEV_DIR}
  systemd units: ${SYSTEMD_DIR}

The helper scripts are installed outside the .deb payload so crate users can
set up the host directly from a checkout.
EOF
