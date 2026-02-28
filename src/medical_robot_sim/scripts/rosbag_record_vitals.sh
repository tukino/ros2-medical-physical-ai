#!/usr/bin/env bash
set -euo pipefail

patients_csv="${1:-patient_01,patient_02,patient_03,patient_04,patient_05}"
vitals_topic="${2:-patient_vitals}"
out_base="${3:-bags}"

ts="$(date +%Y%m%d_%H%M%S)"
out_dir="${out_base}/vitals_${ts}"

IFS=',' read -r -a patients <<< "${patients_csv}"

topics=()
for raw_pid in "${patients[@]}"; do
  pid="$(echo "${raw_pid}" | xargs)"
  pid="${pid#/}"
  if [[ -z "${pid}" ]]; then
    continue
  fi
  topic="/${pid}/${vitals_topic#/}"
  topics+=("${topic}")
done

if [[ ${#topics[@]} -eq 0 ]]; then
  echo "patients が空です" >&2
  exit 2
fi

mkdir -p "${out_base}"

echo "[rosbag] record: ${out_dir}"
echo "[rosbag] topics: ${topics[*]}"

exec ros2 bag record -o "${out_dir}" "${topics[@]}"
