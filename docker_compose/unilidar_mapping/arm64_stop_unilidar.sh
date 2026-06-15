#!/usr/bin/env bash
set -euo pipefail

if [ $# -ne 1 ]; then
    COMPOSE_NAME="unilidar_collection"
else
    COMPOSE_NAME=$1
fi

COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-unilidar}"
COMPOSE_FILE_PATH="${COMPOSE_FILE_PATH:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/docker_compose/unilidar_mapping/${COMPOSE_NAME}.compose.yml}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker not found" >&2
  exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
  echo "docker compose not available" >&2
  exit 1
fi

if [[ ! -f "${COMPOSE_FILE_PATH}" ]]; then
  echo "compose file not found: ${COMPOSE_FILE_PATH}" >&2
  exit 1
fi

docker compose \
  -p "${COMPOSE_PROJECT_NAME}" \
  -f "${COMPOSE_FILE_PATH}" \
  down
