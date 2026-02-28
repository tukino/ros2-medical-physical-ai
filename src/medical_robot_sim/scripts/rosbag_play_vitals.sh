#!/usr/bin/env bash
set -euo pipefail

bag_path="${1:-}"
rate="${2:-}"

if [[ -z "${bag_path}" ]]; then
  if [[ ! -d "bags" ]]; then
    echo "bag_path 未指定かつ bags が見つかりません" >&2
    exit 2
  fi
  bag_path="$(ls -dt bags/* 2>/dev/null | head -n 1 || true)"
fi

if [[ -z "${bag_path}" ]]; then
  echo "再生する bag が見つかりません" >&2
  exit 2
fi

echo "[rosbag] play: ${bag_path}"

if [[ -n "${rate}" ]]; then
  exec ros2 bag play "${bag_path}" -r "${rate}"
fi

exec ros2 bag play "${bag_path}"
