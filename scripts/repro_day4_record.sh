#!/usr/bin/env bash
set -euo pipefail

# Day4 再現用: patients=... を受け取り、対象トピックを組み立てて ros2 bag record し、
# Ctrl+C 1回で停止 → ros2 bag info を表示する。

usage() {
  cat <<'USAGE'
使い方:
  ./scripts/repro_day4_record.sh [patients=patient_01,patient_02,...]

説明:
  - 引数なし: patient_01〜patient_05 を対象に記録
  - Ctrl+C を1回押すと記録を停止し、そのまま bag info を表示

前提:
  - 別途ターミナル等で ROS 2 環境を source 済み（/opt/ros/humble, install/setup.bash など）
USAGE
}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 コマンドが見つかりません。ROS 2 環境を source してください。" >&2
  echo "例: source /opt/ros/humble/setup.bash && source ./install/setup.bash" >&2
  exit 1
fi

patients_csv="patient_01,patient_02,patient_03,patient_04,patient_05"

if [[ $# -ge 2 ]]; then
  usage >&2
  exit 2
fi

if [[ $# -eq 1 ]]; then
  case "$1" in
    patients=*)
      patients_csv="${1#patients=}"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: 引数は patients=... の形式で指定してください: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

if [[ -z "$patients_csv" ]]; then
  echo "ERROR: patients が空です" >&2
  exit 2
fi

IFS=',' read -r -a patients <<<"$patients_csv"
if [[ ${#patients[@]} -eq 0 ]]; then
  echo "ERROR: patients の解析に失敗しました: $patients_csv" >&2
  exit 2
fi

mkdir -p bags
stamp="$(date +%Y%m%d_%H%M%S)"
out_dir="bags/day4_vitals_${stamp}"

# 記録対象トピックを組み立て
# 例: /patient_01/patient_vitals
record_topics=()
for pid in "${patients[@]}"; do
  # 念のため空要素を除外
  if [[ -z "$pid" ]]; then
    continue
  fi
  record_topics+=("/${pid}/patient_vitals")
done

if [[ ${#record_topics[@]} -eq 0 ]]; then
  echo "ERROR: 記録対象トピックが空です（patients=$patients_csv）" >&2
  exit 2
fi

echo "Recording topics:" >&2
printf '  %s\n' "${record_topics[@]}" >&2

echo "Output: ${out_dir}" >&2
echo "停止するには Ctrl+C を1回押してください。" >&2

record_pid=""

on_int() {
  # Ctrl+C でスクリプト自体が落ちないようにしつつ、子プロセス(ros2 bag record)に SIGINT を渡す
  echo "" >&2
  echo "Stopping ros2 bag record..." >&2
  if [[ -n "${record_pid}" ]] && kill -0 "${record_pid}" 2>/dev/null; then
    kill -INT "${record_pid}" 2>/dev/null || true
  fi
}
trap on_int INT

# ros2 bag record はフォアグラウンドだと SIGINT でスクリプトごと止まりやすいので、
# 背景実行 + trap + wait で「Ctrl+C 1回」での停止後処理を実現する。
ros2 bag record -o "${out_dir}" "${record_topics[@]}" &
record_pid=$!

set +e
wait "${record_pid}"
record_rc=$?
set -e

# Ctrl+C による終了(130)は正常扱い
if [[ ${record_rc} -ne 0 && ${record_rc} -ne 130 ]]; then
  echo "WARN: ros2 bag record が非0で終了しました (rc=${record_rc})" >&2
fi

echo "" >&2
echo "Bag info:" >&2
ros2 bag info "${out_dir}"
