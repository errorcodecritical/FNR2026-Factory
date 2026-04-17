#!/usr/bin/env bash
# install-usb-serial-rules.sh
# Installs udev rules to bind USB serial devices to stable symlinks:
#   /dev/ttyACM_ARDUINO  ← Arduino Micro        (2341:8037)
#   /dev/ttyACM_PICO     ← Raspberry Pi Pico 2  (2e8a:000f)
#   /dev/ttyUSB_CP210X   ← CP210x UART Bridge   (10c4:ea60)

set -euo pipefail

RULES_FILE="99-usb-serial.rules"
RULES_DIR="/etc/udev/rules.d"
RULES_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/${RULES_FILE}"

# ── Helpers ────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
ok()      { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*" >&2; }
die()     { error "$*"; exit 1; }

print_header() {
    echo -e "${BOLD}"
    echo "╔══════════════════════════════════════════════════╗"
    echo "║       USB Serial Device Rules Installer          ║"
    echo "╚══════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

# ── Preflight ──────────────────────────────────────────────────────────────
print_header

[[ $EUID -ne 0 ]] && die "Run as root: sudo $0"
[[ -f "$RULES_SRC" ]] || die "Rules file not found: $RULES_SRC"
command -v udevadm &>/dev/null || die "udevadm not found — is udev/systemd installed?"

# ── Detect currently connected devices ────────────────────────────────────
info "Scanning for target devices..."
echo ""

declare -A DEVICE_STATUS
TARGETS=(
    "2341:8037:Arduino Micro:ttyACM_ARDUINO"
    "2e8a:000f:Raspberry Pi Pico 2:ttyACM_PICO"
    "10c4:ea60:CP210x UART Bridge:ttyUSB_CP210X"
)

for entry in "${TARGETS[@]}"; do
    IFS=: read -r vendor product label symlink <<< "$entry"
    node=$(find /sys/bus/usb/devices -name "idVendor" -exec grep -li "^${vendor}$" {} \; 2>/dev/null | \
           while read -r f; do
               dir=$(dirname "$f")
               prod=$(cat "$dir/idProduct" 2>/dev/null)
               [[ "$prod" == "$product" ]] && echo "$dir"
           done | head -1)

    if [[ -n "$node" ]]; then
        # Find associated tty
        tty=$(find "$node" -name "tty*" -maxdepth 4 2>/dev/null | grep -oP 'tty\w+' | head -1 || true)
        if [[ -n "$tty" ]]; then
            printf "  ${GREEN}✔${RESET}  %-28s → /dev/%-14s  (currently /dev/%s)\n" \
                "$label" "$symlink" "$tty"
            DEVICE_STATUS[$label]="found:/dev/$tty"
        else
            printf "  ${YELLOW}~${RESET}  %-28s → /dev/%-14s  (connected, no tty yet)\n" \
                "$label" "$symlink"
            DEVICE_STATUS[$label]="connected"
        fi
    else
        printf "  ${RED}✘${RESET}  %-28s → /dev/%-14s  (not connected)\n" \
            "$label" "$symlink"
        DEVICE_STATUS[$label]="absent"
    fi
done

echo ""

# ── Check for multiple same-VID:PID devices ───────────────────────────────
# If you have two identical devices you'd need serial-number-based rules.
# Warn if duplicates are found.
for entry in "${TARGETS[@]}"; do
    IFS=: read -r vendor product label symlink <<< "$entry"
    count=$(find /sys/bus/usb/devices -name "idVendor" -exec grep -li "^${vendor}$" {} \; 2>/dev/null | \
            while read -r f; do
                dir=$(dirname "$f")
                prod=$(cat "$dir/idProduct" 2>/dev/null)
                [[ "$prod" == "$product" ]] && echo "$dir"
            done | wc -l)
    if [[ $count -gt 1 ]]; then
        warn "Multiple devices found for ${label} (${vendor}:${product})."
        warn "VID:PID rules alone cannot distinguish them — consider adding ATTRS{serial}."
    fi
done

# ── Backup existing rules if present ──────────────────────────────────────
DEST="${RULES_DIR}/${RULES_FILE}"
if [[ -f "$DEST" ]]; then
    BACKUP="${DEST}.bak.$(date +%Y%m%d_%H%M%S)"
    info "Backing up existing rules to ${BACKUP}"
    cp "$DEST" "$BACKUP"
fi

# ── Install rules ──────────────────────────────────────────────────────────
info "Installing ${RULES_FILE} → ${RULES_DIR}/"
cp "$RULES_SRC" "$DEST"
chmod 644 "$DEST"
ok "Rules file installed."

# ── Reload udev ───────────────────────────────────────────────────────────
info "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --action=add
ok "udev rules reloaded and triggered."

# ── Verify symlinks ───────────────────────────────────────────────────────
echo ""
info "Waiting 1 s for symlinks to settle..."
sleep 1

echo ""
echo -e "${BOLD}  Symlink Status:${RESET}"
ALL_OK=true
for entry in "${TARGETS[@]}"; do
    IFS=: read -r vendor product label symlink <<< "$entry"
    target="/dev/${symlink}"
    if [[ -L "$target" ]]; then
        real=$(readlink -f "$target")
        printf "  ${GREEN}✔${RESET}  /dev/%-18s → %s\n" "$symlink" "$real"
    else
        printf "  ${YELLOW}—${RESET}  /dev/%-18s   (device not currently connected)\n" "$symlink"
        # Only flag as broken if device was detected earlier
        if [[ "${DEVICE_STATUS[$label]:-}" == found:* ]]; then
            ALL_OK=false
        fi
    fi
done

echo ""
if $ALL_OK; then
    ok "Installation complete. Symlinks are active for all connected devices."
else
    warn "Some connected devices did not get symlinks. Try unplugging and replugging."
fi

# ── Print usage summary ───────────────────────────────────────────────────
echo ""
echo -e "${BOLD}  Usage in your applications:${RESET}"
echo "    Arduino Micro   →  /dev/ttyACM_ARDUINO"
echo "    Pico 2          →  /dev/ttyACM_PICO"
echo "    CP210x Bridge   →  /dev/ttyUSB_CP210X"
echo ""
echo -e "  Symlinks survive reboots and replugs regardless of enumeration order."
echo ""

# ── Optional: user group check ────────────────────────────────────────────
INVOKING_USER="${SUDO_USER:-${USER}}"
if [[ -n "$INVOKING_USER" && "$INVOKING_USER" != "root" ]]; then
    if ! groups "$INVOKING_USER" | grep -qw dialout; then
        warn "User '${INVOKING_USER}' is not in the 'dialout' group."
        warn "Add them with:  sudo usermod -aG dialout ${INVOKING_USER}"
        warn "Then log out and back in (or run: newgrp dialout)"
    else
        ok "User '${INVOKING_USER}' is already in the 'dialout' group."
    fi
fi

exit 0