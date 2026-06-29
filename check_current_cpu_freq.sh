#!/usr/bin/env bash
set -euo pipefail

show_policy() {
  local policy_dir="$1"
  local policy_name cpu_list cur_freq min_freq max_freq governor

  policy_name="$(basename "${policy_dir}")"
  cpu_list="$(<"${policy_dir}/affected_cpus")"
  cur_freq="$(<"${policy_dir}/scaling_cur_freq")"
  min_freq="$(<"${policy_dir}/scaling_min_freq")"
  max_freq="$(<"${policy_dir}/scaling_max_freq")"
  governor="$(<"${policy_dir}/scaling_governor")"

  echo "${policy_name}: cpus=${cpu_list} current_khz=${cur_freq} min_khz=${min_freq} max_khz=${max_freq} governor=${governor}"
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
  show_policy "${policy_dir}"
done
