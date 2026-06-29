#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

USB_DEVICE="${USB_DEVICE:-/dev/sda1}"
MOUNT_POINT="${MOUNT_POINT:-/mnt/usb}"
SOURCE_ROOT="${SOURCE_ROOT:-${REPO_ROOT}/data/rosbags}"
DEST_DIR_NAME="${DEST_DIR_NAME:-unilidar}"

usage() {
  cat <<EOF
Usage:
  $0

Behavior:
  - Mounts USB device ${USB_DEVICE} to ${MOUNT_POINT}
  - Copies all rosbag directories under ${SOURCE_ROOT} to ${MOUNT_POINT}/${DEST_DIR_NAME}
  - Removes each rosbag directory from the host after a successful copy
  - Syncs and unmounts

Environment overrides:
  USB_DEVICE
  MOUNT_POINT
  SOURCE_ROOT
  DEST_DIR_NAME
EOF
}

list_source_dirs() {
  find "${SOURCE_ROOT}" -maxdepth 1 -mindepth 1 -type d | sort
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

DEST_ROOT="${MOUNT_POINT}/${DEST_DIR_NAME}"

if [[ $# -gt 0 ]]; then
  echo "unexpected arguments: $*" >&2
  usage >&2
  exit 1
fi

cleanup() {
  local code=$?
  if mountpoint -q "${MOUNT_POINT}"; then
    sync || true
    sudo umount "${MOUNT_POINT}" || true
  fi
  exit "${code}"
}
trap cleanup EXIT

if [[ ! -b "${USB_DEVICE}" ]]; then
  echo "USB device not found: ${USB_DEVICE}" >&2
  exit 1
fi

if [[ ! -d "${SOURCE_ROOT}" ]]; then
  echo "source root not found: ${SOURCE_ROOT}" >&2
  exit 1
fi

mapfile -t SOURCE_DIRS < <(list_source_dirs)
if [[ ${#SOURCE_DIRS[@]} -eq 0 ]]; then
  echo "no rosbag directories found under ${SOURCE_ROOT}" >&2
  exit 1
fi

sudo mkdir -p "${MOUNT_POINT}"

if ! mountpoint -q "${MOUNT_POINT}"; then
  sudo mount -t exfat "${USB_DEVICE}" "${MOUNT_POINT}"
fi

sudo mkdir -p "${DEST_ROOT}"

echo "Found ${#SOURCE_DIRS[@]} rosbag directories under ${SOURCE_ROOT}"
for SOURCE_DIR in "${SOURCE_DIRS[@]}"; do
  BAG_BASENAME="$(basename "${SOURCE_DIR}")"
  DEST_PATH="${DEST_ROOT}/${BAG_BASENAME}"

  echo "Copying:"
  echo "  source: ${SOURCE_DIR}"
  echo "  dest:   ${DEST_PATH}"

  sudo rm -rf "${DEST_PATH}"
  sudo mkdir -p "${DEST_PATH}"

  if command -v rsync >/dev/null 2>&1; then
    sudo rsync -r --info=progress2 "${SOURCE_DIR}/" "${DEST_PATH}/"
  else
    echo "rsync not found, falling back to per-file verbose copy." >&2
    sudo cp -rv "${SOURCE_DIR}/." "${DEST_PATH}/"
  fi

  sudo rm -rf "${SOURCE_DIR}"
  echo "Removed source: ${SOURCE_DIR}"
done

sync
sudo umount "${MOUNT_POINT}"
echo "Copy complete: ${DEST_ROOT}"
