#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SERVICE_SRC="${REPO_ROOT}/docker_compose/boot_app/unilidar-web.service"
SERVICE_DST="/etc/systemd/system/unilidar-web.service"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Please run as root, for example: sudo bash $0" >&2
  exit 1
fi

if [[ ! -f "${SERVICE_SRC}" ]]; then
  echo "Service file not found: ${SERVICE_SRC}" >&2
  exit 1
fi

install -m 0644 "${SERVICE_SRC}" "${SERVICE_DST}"
systemctl daemon-reload
systemctl enable unilidar-web.service
systemctl restart unilidar-web.service

echo "Installed ${SERVICE_DST}"
echo "Enabled and started unilidar-web.service"
echo "Check status with: systemctl status unilidar-web.service"
echo "Follow logs with: journalctl -u unilidar-web.service -f"
