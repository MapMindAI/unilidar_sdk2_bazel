#!/usr/bin/env bash
set -euo pipefail

write_value() {
  local value="$1"
  local path="$2"

  if [[ -w "${path}" ]]; then
    printf '%s\n' "${value}" > "${path}"
  elif command -v sudo >/dev/null 2>&1; then
    printf '%s\n' "${value}" | sudo tee "${path}" >/dev/null
  else
    echo "cannot write ${path}; rerun as root or install sudo" >&2
    exit 1
  fi
}

set_policy_max() {
  local policy_dir="$1"
  local policy_name cpu_list cpuinfo_max governor_path

  policy_name="$(basename "${policy_dir}")"
  cpu_list="$(<"${policy_dir}/affected_cpus")"
  cpuinfo_max="$(<"${policy_dir}/cpuinfo_max_freq")"
  governor_path="${policy_dir}/scaling_governor"

  if [[ -f "${governor_path}" ]]; then
    write_value "performance" "${governor_path}"
  fi

  write_value "${cpuinfo_max}" "${policy_dir}/scaling_max_freq"
  write_value "${cpuinfo_max}" "${policy_dir}/scaling_min_freq"

  echo "${policy_name}: cpus=${cpu_list} set_to_khz=${cpuinfo_max}"
}

if [[ $# -gt 0 ]]; then
  echo "unexpected arguments: $*" >&2
  echo "usage: $0" >&2
  exit 1
fi

shopt -s nullglob
POLICY_DIRS=(/sys/devices/system/cpu/cpufreq/policy*)
shopt -u nullglob

if [[ ${#POLICY_DIRS[@]} -eq 0 ]]; then
  echo "no cpufreq policy directories found under /sys/devices/system/cpu/cpufreq" >&2
  exit 1
fi

for policy_dir in "${POLICY_DIRS[@]}"; do
  set_policy_max "${policy_dir}"
done
