#!/usr/bin/env bash
# Day6 再現スクリプト: flatline 検知の確認
#
# 目的:
#   - scenario='flatline' の vital_sensor を起動し、HR/SpO2 を固定値で送り続ける
#   - rule_alert_engine が flatline.hr / flatline.spo2 を検知して
#     /patient_01/alerts に publish することを確認する
#
# 使い方:
#   ./scripts/repro_day6_flatline.sh
#
# ROS 2 Humble 環境が source 済みであること:
#   source /opt/ros/humble/setup.bash
#   source ~/ros2_ws/install/setup.bash

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_SETUP="/opt/ros/humble/setup.bash"
OVERLAY_SETUP="${WS_DIR}/install/setup.bash"

if [[ -f "${ROS_SETUP}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
else
  echo "ERROR: ROS setup not found: ${ROS_SETUP}" >&2
  echo "Hint: install ROS 2 Humble" >&2
  exit 1
fi

if [[ -f "${OVERLAY_SETUP}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${OVERLAY_SETUP}"
  set -u
else
  echo "ERROR: Overlay setup not found: ${OVERLAY_SETUP}" >&2
  echo "Hint: run 'colcon build --symlink-install' in ${WS_DIR}" >&2
  exit 1
fi

PATIENT="${1:-patient_01}"
FLATLINE_HISTORY_SIZE="${FLATLINE_HISTORY_SIZE:-8}"
WAIT_SEC="${WAIT_SEC:-15}"

echo "=============================="
echo " Day6 flatline 再現スクリプト"
echo "=============================="
echo "patient          : ${PATIENT}"
echo "flatline_history_size: ${FLATLINE_HISTORY_SIZE}"
echo "待機時間         : ${WAIT_SEC}s (flatline 検知には ${FLATLINE_HISTORY_SIZE} サンプル必要)"
echo ""

cd "${WS_DIR}"

mkdir -p bags
stamp="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="bags/day6_flatline_${stamp}"

# ---- 子プロセス管理 ----
PIDS=()

cleanup() {
  echo ""
  echo "--- クリーンアップ中 ---"
  for pid in "${PIDS[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 0.5
  for pid in "${PIDS[@]}"; do
    kill -KILL "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

# ---- (1) vital_sensor（flatline シナリオ）起動 ----
echo "[1/4] vital_sensor (scenario=flatline) を起動..."
ros2 run medical_robot_sim vital_sensor \
  --ros-args \
  --remap __ns:="/${PATIENT}" \
  -p patient_id:="${PATIENT}" \
  -p scenario:=flatline \
  -p flatline_hr_value:=72 \
  -p flatline_spo2_value:=98 \
  -p publish_rate_hz:=1.0 \
  &
PIDS+=($!)
sleep 1

# ---- (2) rule_alert_engine 起動 ----
echo "[2/4] rule_alert_engine を起動..."
ros2 run medical_robot_sim rule_alert_engine \
  --ros-args \
  -p "patients:=[${PATIENT}]" \
  -p vitals_topic:=patient_vitals \
  -p alert_topic:=alerts \
  -p flatline_history_size:="${FLATLINE_HISTORY_SIZE}" \
  -p flatline_hr_epsilon:=1.0 \
  -p flatline_spo2_epsilon:=1.0 \
  &
PIDS+=($!)
sleep 1

# ---- (3) bag record 開始 ----
echo "[3/4] bag 記録開始: ${BAG_DIR}"
ros2 bag record -o "${BAG_DIR}" \
  "/${PATIENT}/patient_vitals" \
  "/${PATIENT}/alerts" \
  &
PIDS+=($!)

# ---- (4) WAIT_SEC 秒待機して alerts を確認 ----
echo "[4/4] ${WAIT_SEC}s 待機中..."
echo "      (別ターミナルで以下を実行するとリアルタイム確認可)"
echo "      ros2 topic echo /${PATIENT}/alerts"
echo ""
sleep "${WAIT_SEC}"

# ---- 後片付け（trap cleanup が呼ばれる） ----
# ここで trap EXIT が実行される
echo ""
echo "=============================="
echo " bag 情報"
echo "=============================="
sleep 0.5
ros2 bag info "${BAG_DIR}" 2>/dev/null || echo "(bag info 取得失敗 - 記録されていない可能性があります)"

echo ""
echo "=============================="
echo " 合格条件チェック"
echo "=============================="
echo "以下を確認してください:"
echo "  1) /${PATIENT}/patient_vitals の Message Count > 0"
echo "  2) /${PATIENT}/alerts          の Message Count > 0"
echo "     (rule_id: flatline.hr または flatline.spo2 が含まれているはず)"
echo ""
echo "手動確認コマンド:"
echo "  ros2 bag play ${BAG_DIR} --read-ahead-queue-size 100"
echo "  ros2 topic echo /${PATIENT}/alerts"
