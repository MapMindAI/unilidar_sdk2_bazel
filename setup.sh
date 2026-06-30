#!/usr/bin/env bash
# UniLidar one-shot setup: sudo rules, CPU max freq, web service boot.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Colour helpers ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()    { echo -e "${GREEN}[setup]${NC} $*"; }
warn()    { echo -e "${YELLOW}[setup]${NC} $*"; }
error()   { echo -e "${RED}[setup]${NC} $*" >&2; }
section() { echo -e "\n${GREEN}━━━ $* ━━━${NC}"; }

# ── Root check ────────────────────────────────────────────────────────────────
if [[ "$(id -u)" -ne 0 ]]; then
  error "Please run as root:  sudo bash $0"
  exit 1
fi


# ─────────────────────────────────────────────────────────────────────────────
section "1 / 3  Sudo rules + dialout group"
# ─────────────────────────────────────────────────────────────────────────────

bash "${SCRIPT_DIR}/tools/setup_unilidar_sudo.sh"

# ─────────────────────────────────────────────────────────────────────────────
section "2 / 3  CPU frequency — set to max"
# ─────────────────────────────────────────────────────────────────────────────

SET_SCRIPT="${SCRIPT_DIR}/tools/set_cpu_freq_max.sh"
CHECK_SCRIPT="${SCRIPT_DIR}/tools/check_current_cpu_freq.sh"

if [[ ! -x "${SET_SCRIPT}" ]]; then
  warn "${SET_SCRIPT} not found or not executable — skipping."
else
  info "Setting all CPU policies to max frequency..."
  bash "${SET_SCRIPT}"

  if [[ -x "${CHECK_SCRIPT}" ]]; then
    info "Current CPU frequency after change:"
    bash "${CHECK_SCRIPT}"
  fi
fi

# ─────────────────────────────────────────────────────────────────────────────
section "3 / 3  Enable UniLidar web service on boot"
# ─────────────────────────────────────────────────────────────────────────────

BOOT_SCRIPT="${SCRIPT_DIR}/docker_compose/boot_app/enable_unilidar_web_boot.sh"

if [[ ! -f "${BOOT_SCRIPT}" ]]; then
  error "Boot script not found: ${BOOT_SCRIPT}"
  exit 1
fi

bash "${BOOT_SCRIPT}"

# ─────────────────────────────────────────────────────────────────────────────
section "Setup complete"
# ─────────────────────────────────────────────────────────────────────────────

info "All steps finished."
warn "If the user was just added to dialout, they must log out and back in"
warn "(or reboot) before the new group membership takes effect."
echo ""
info "Web service status:  systemctl status unilidar-web.service"
info "Web service logs:    journalctl -u unilidar-web.service -f"
