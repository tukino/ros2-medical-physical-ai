#!/usr/bin/env bash
# CI smoke test: launch the ROS graph and verify representative topics.
#
# This intentionally exercises runtime wiring that unit tests do not cover:
# - ros2 launch can start the multi-patient system
# - topic discovery works in the CI container
# - vitals, alerts, and control actions can be observed with ros2 topic echo

set -eo pipefail

PATIENT_ID="${PATIENT_ID:-patient_01}"
LOG_FILE="${LOG_FILE:-/tmp/ros2_ci_launch_topic_smoke.log}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-30}"
ECHO_WAIT_SEC="${ECHO_WAIT_SEC:-35}"

LAUNCH_PID=""

print_launch_log() {
  if [ -f "${LOG_FILE}" ]; then
    echo "----- launch log (${LOG_FILE}) -----"
    sed -n '1,240p' "${LOG_FILE}" || true
    echo "----- end launch log -----"
  fi
}

cleanup() {
  local rc=$?
  if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  if [ "${rc}" -ne 0 ]; then
    print_launch_log
  fi
  return "${rc}"
}
trap cleanup EXIT

topic_exists() {
  local topic="$1"
  local topics_file="/tmp/ros2_ci_topics.txt"
  ros2 topic list > "${topics_file}"
  python3 - "$topic" "${topics_file}" <<'PY'
import sys

expected = sys.argv[1]
path = sys.argv[2]
with open(path, encoding="utf-8") as f:
    topics = {line.strip() for line in f if line.strip()}
raise SystemExit(0 if expected in topics else 1)
PY
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + TOPIC_WAIT_SEC))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if topic_exists "${topic}"; then
      echo "topic available: ${topic}"
      return 0
    fi
    sleep 1
  done

  echo "ERROR: topic did not appear within ${TOPIC_WAIT_SEC}s: ${topic}" >&2
  ros2 topic list || true
  return 1
}

echo_once() {
  local topic="$1"
  local msg_type="$2"
  echo "echo once: ${topic} (${msg_type})"
  timeout "${ECHO_WAIT_SEC}s" ros2 topic echo "${topic}" "${msg_type}" --once
}

main() {
  rm -f "${LOG_FILE}"

  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true

  ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    patients:="${PATIENT_ID}" \
    enable_alerts:=true \
    enable_closed_loop:=true \
    scenario:=flatline \
    flatline_history_size:=3 \
    control_cooldown_sec:=1.0 \
    > "${LOG_FILE}" 2>&1 &
  LAUNCH_PID=$!

  wait_for_topic "/${PATIENT_ID}/patient_vitals"
  wait_for_topic "/${PATIENT_ID}/alerts"
  wait_for_topic "/${PATIENT_ID}/control_actions"

  echo_once "/${PATIENT_ID}/patient_vitals" "medical_interfaces/msg/VitalSigns"
  echo_once "/${PATIENT_ID}/alerts" "medical_interfaces/msg/Alert"
  echo_once "/${PATIENT_ID}/control_actions" "medical_interfaces/msg/Alert"

  if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    echo "ERROR: launch process exited before cleanup" >&2
    print_launch_log
    return 1
  fi

  echo "CI launch/topic smoke test passed"
}

main "$@"
