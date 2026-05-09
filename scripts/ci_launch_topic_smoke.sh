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
SCENARIO="${SCENARIO:-flatline}"
FLATLINE_HISTORY_SIZE="${FLATLINE_HISTORY_SIZE:-3}"
CONTROL_COOLDOWN_SEC="${CONTROL_COOLDOWN_SEC:-1.0}"
CONTROL_LOW_SPO2="${CONTROL_LOW_SPO2:-92.0}"
CONTROL_CRITICAL_SPO2="${CONTROL_CRITICAL_SPO2:-88.0}"
EXPECTED_CONTROL_RULE_ID="${EXPECTED_CONTROL_RULE_ID:-}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-30}"
ECHO_WAIT_SEC="${ECHO_WAIT_SEC:-35}"
CONTROL_ONCE_LOG="${CONTROL_ONCE_LOG:-/tmp/ros2_ci_control_once.log}"

LAUNCH_PID=""

# Avoid the ros2 CLI daemon in headless CI; acceptance docs use the same
# mitigation when daemon state makes topic discovery flaky.
export ROS2CLI_NO_DAEMON="${ROS2CLI_NO_DAEMON:-1}"

stop_launch() {
  if [ -z "${LAUNCH_PID}" ] || ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    return 0
  fi

  echo "Stopping launch process: ${LAUNCH_PID}"
  kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
      wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
      return 0
    fi
    sleep 1
  done

  echo "Launch process did not stop after SIGINT; sending SIGTERM"
  kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
  for _ in 1 2 3 4 5; do
    if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
      wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
      return 0
    fi
    sleep 1
  done

  echo "Launch process did not stop after SIGTERM; sending SIGKILL"
  kill -KILL "${LAUNCH_PID}" >/dev/null 2>&1 || true
  wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
}

print_launch_log() {
  if [ -f "${LOG_FILE}" ]; then
    echo "----- launch log (${LOG_FILE}) -----"
    sed -n '1,240p' "${LOG_FILE}" || true
    echo "----- end launch log -----"
  fi
}

cleanup() {
  local rc=$?
  stop_launch
  if [ "${rc}" -ne 0 ]; then
    print_launch_log
  fi
  return "${rc}"
}
trap cleanup EXIT

topic_exists() {
  local topic="$1"
  local topics_file="/tmp/ros2_ci_topics.txt"
  if ! ros2 topic list > "${topics_file}" 2>/tmp/ros2_ci_topic_list.err; then
    echo "WARN: ros2 topic list failed while waiting for ${topic}" >&2
    sed -n '1,40p' /tmp/ros2_ci_topic_list.err >&2 || true
    return 1
  fi
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

echo_control_once() {
  local topic="/${PATIENT_ID}/control_actions"
  echo "echo once: ${topic} (medical_interfaces/msg/Alert)"
  timeout "${ECHO_WAIT_SEC}s" ros2 topic echo \
    "${topic}" medical_interfaces/msg/Alert --once | tee "${CONTROL_ONCE_LOG}"
}

main() {
  rm -f "${LOG_FILE}"

  ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    patients:="${PATIENT_ID}" \
    enable_alerts:=true \
    enable_closed_loop:=true \
    scenario:="${SCENARIO}" \
    flatline_history_size:="${FLATLINE_HISTORY_SIZE}" \
    control_cooldown_sec:="${CONTROL_COOLDOWN_SEC}" \
    control_low_spo2:="${CONTROL_LOW_SPO2}" \
    control_critical_spo2:="${CONTROL_CRITICAL_SPO2}" \
    sigterm_timeout:=2 \
    sigkill_timeout:=2 \
    > "${LOG_FILE}" 2>&1 &
  LAUNCH_PID=$!

  wait_for_topic "/${PATIENT_ID}/patient_vitals"
  wait_for_topic "/${PATIENT_ID}/alerts"
  wait_for_topic "/${PATIENT_ID}/control_actions"

  echo_once "/${PATIENT_ID}/patient_vitals" "medical_interfaces/msg/VitalSigns"
  echo_once "/${PATIENT_ID}/alerts" "medical_interfaces/msg/Alert"
  echo_control_once

  if [ -n "${EXPECTED_CONTROL_RULE_ID}" ]; then
    grep -n "rule_id: ${EXPECTED_CONTROL_RULE_ID}" "${CONTROL_ONCE_LOG}"
  fi

  if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    echo "ERROR: launch process exited before cleanup" >&2
    print_launch_log
    return 1
  fi

  echo "CI launch/topic smoke test passed (scenario=${SCENARIO})"
}

main "$@"
