#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SERVICE_DST="/etc/systemd/system/unilidar-web.service"
SERVICE_USER="${SUDO_USER:-$(id -un)}"
ENV_DIR="/etc/unilidar"
ENV_FILE="${ENV_DIR}/rtk.env"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Please run as root, for example: sudo bash $0" >&2
  exit 1
fi

if [[ -z "${SUDO_USER:-}" ]]; then
  SERVICE_USER="$(id -un)"
fi

if [[ ! -d "${REPO_ROOT}" ]]; then
  echo "Repo root not found: ${REPO_ROOT}" >&2
  exit 1
fi

if [[ ! -f "${REPO_ROOT}/docker_compose/unilidar_mapping/webserver.py" ]]; then
  echo "webserver.py not found under repo root: ${REPO_ROOT}" >&2
  exit 1
fi

mkdir -p "${ENV_DIR}"
if [[ ! -f "${ENV_FILE}" ]]; then
  cat > "${ENV_FILE}" <<'EOF'
# UniLidar RTK publisher settings.
# This file is loaded by unilidar-web.service and by arm64_start_unilidar.sh.
# Use KEY=value lines, without "export".

RTK_SERIAL_PORT=/dev/ttyUSB0
RTK_BAUDRATE=115200
RTK_FRAME_ID=rtk
RTK_FIX_TOPIC=/rtk/fix

# Leave NTRIP_HOST empty to disable NTRIP corrections.
NTRIP_HOST=
NTRIP_PORT=2101
NTRIP_MOUNTPOINT=
NTRIP_USER=
NTRIP_PASSWORD=
NTRIP_TLS=false
EOF
  chown root:"${SERVICE_USER}" "${ENV_FILE}" 2>/dev/null || chown root:root "${ENV_FILE}"
  chmod 0640 "${ENV_FILE}"
fi

cat > "${SERVICE_DST}" <<EOF
[Unit]
Description=UniLidar Remote Web Server
After=network-online.target docker.service
Wants=network-online.target docker.service

[Service]
Type=simple
User=${SERVICE_USER}
WorkingDirectory=${REPO_ROOT}
Environment=UNILIDAR_WEB_HOST=0.0.0.0
Environment=UNILIDAR_WEB_PORT=8080
EnvironmentFile=-${ENV_FILE}
ExecStart=/usr/bin/python3 ${REPO_ROOT}/docker_compose/unilidar_mapping/webserver.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

chmod 0644 "${SERVICE_DST}"
systemctl daemon-reload
systemctl enable unilidar-web.service
systemctl restart unilidar-web.service

echo "Installed ${SERVICE_DST}"
echo "RTK env file: ${ENV_FILE}"
echo "Service user: ${SERVICE_USER}"
echo "Working directory: ${REPO_ROOT}"
echo "Enabled and started unilidar-web.service"
echo "Check status with: systemctl status unilidar-web.service"
echo "Follow logs with: journalctl -u unilidar-web.service -f"
