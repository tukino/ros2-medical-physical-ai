#!/usr/bin/env bash
# Day19 CI smoke test: record vitals to rosbag, replay through icu_replay.launch.py
# with enable_closed_loop=true, and verify control_actions are emitted.
#
# This extends the Day12 rosbag reproducibility loop (ci_rosbag_replay_smoke.sh)
# to cover the Day18/Day19 closed-loop control path:
#   live vitals -> rosbag record -> replay pipeline -> alerts -> control_actions.

set -euo pipefail

PATIENT_ID="${PATIENT_ID:-patient_01}"
WORK_DIR="${WORK_DIR:-$(mktemp -d /tmp/ros2_ci_day19_replay_control.XXXXXX)}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-30}"
ECHO_WAIT_SEC="${ECHO_WAIT_SEC:-35}"
CONTROL_LOW_SPO2="${CONTROL_LOW_SPO2:-99.0}"
CONTROL_CRITICAL_SPO2="${CONTROL_CRITICAL_SPO2:-95.0}"
CONTROL_COOLDOWN_SEC="${CONTROL_COOLDOWN_SEC:-1.0}"
# Default to control.call_staff: the injected SpO2=85 sample is below
# CONTROL_CRITICAL_SPO2=95.0, so the replay must produce a call_staff action
# rather than an arbitrary control.hold that could be captured before replay.
EXPECTED_CONTROL_RULE_ID="${EXPECTED_CONTROL_RULE_ID:-control.call_staff}"

BAG_DIR="${WORK_DIR}/vitals_bag"
LIVE_LOG="${WORK_DIR}/live_launch.log"
RECORD_LOG="${WORK_DIR}/bag_record.log"
BAG_INFO_LOG="${WORK_DIR}/bag_info.log"
MANUAL_PUB_LOG="${WORK_DIR}/manual_pub.log"
REPLAY_LOG="${WORK_DIR}/replay_launch.log"
PLAY_LOG="${WORK_DIR}/bag_play.log"
ALERT_ONCE_LOG="${WORK_DIR}/alert_once.log"
CONTROL_ONCE_LOG="${WORK_DIR}/control_once.log"

LIVE_PID=""
RECORD_PID=""
REPLAY_PID=""
PLAY_PID=""
ECHO_ALERT_PID=""
ECHO_CONTROL_PID=""

# Avoid the ros2 CLI daemon in headless CI.
export ROS2CLI_NO_DAEMON="${ROS2CLI_NO_DAEMON:-1}"

stop_pid() {
  local label="$1"
  local pid="$2"
  if [ -z "${pid}" ] || ! kill -0 "${pid}" >/dev/null 2>&1; then
    return 0
  fi

  echo "Stopping ${label}: ${pid}"
  kill -INT "${pid}" >/dev/null 2>&1 || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      wait "${pid}" >/dev/null 2>&1 || true
      return 0
    fi
    sleep 1
  done

  echo "${label} did not stop after SIGINT; sending SIGTERM"
  kill -TERM "${pid}" >/dev/null 2>&1 || true
  for _ in 1 2 3 4 5; do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      wait "${pid}" >/dev/null 2>&1 || true
      return 0
    fi
    sleep 1
  done

  echo "${label} did not stop after SIGTERM; sending SIGKILL"
  kill -KILL "${pid}" >/dev/null 2>&1 || true
  wait "${pid}" >/dev/null 2>&1 || true
}

print_log() {
  local label="$1"
  local path="$2"
  if [ -f "${path}" ]; then
    echo "----- ${label} (${path}) -----"
    sed -n '1,220p' "${path}" || true
    echo "----- end ${label} -----"
  fi
}

print_debug_logs() {
  print_log "live launch log"    "${LIVE_LOG}"
  print_log "bag record log"     "${RECORD_LOG}"
  print_log "manual publish log" "${MANUAL_PUB_LOG}"
  print_log "bag info log"       "${BAG_INFO_LOG}"
  print_log "replay launch log"  "${REPLAY_LOG}"
  print_log "bag play log"       "${PLAY_LOG}"
  print_log "alert echo log"     "${ALERT_ONCE_LOG}"
  print_log "control echo log"   "${CONTROL_ONCE_LOG}"
}

cleanup() {
  local rc=$?
  stop_pid "alert echo"   "${ECHO_ALERT_PID}"
  stop_pid "control echo" "${ECHO_CONTROL_PID}"
  stop_pid "bag play"     "${PLAY_PID}"
  stop_pid "bag record"   "${RECORD_PID}"
  stop_pid "live launch"  "${LIVE_PID}"
  stop_pid "replay launch" "${REPLAY_PID}"
  if [ "${rc}" -ne 0 ]; then
    print_debug_logs
  fi
  return "${rc}"
}
trap cleanup EXIT

topic_exists() {
  local topic="$1"
  local topics_file="${WORK_DIR}/topics.txt"
  if ! ros2 topic list > "${topics_file}" 2>"${WORK_DIR}/topic_list.err"; then
    echo "WARN: ros2 topic list failed while waiting for ${topic}" >&2
    sed -n '1,40p' "${WORK_DIR}/topic_list.err" >&2 || true
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

publish_abnormal_vitals_once() {
  # Publish vitals with SpO2=85 (below CONTROL_CRITICAL_SPO2=95 and CONTROL_LOW_SPO2=99)
  # This deterministic sample ensures the replay triggers a control action.
  timeout 15s ros2 topic pub --once \
    --qos-reliability reliable \
    --qos-durability transient_local \
    "/${PATIENT_ID}/patient_vitals" medical_interfaces/msg/VitalSigns \
    "{patient_id: '${PATIENT_ID}', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}" \
    > "${MANUAL_PUB_LOG}" 2>&1
}

record_bag() {
  echo "=== Phase 1: Record vitals bag to ${BAG_DIR} ==="
  rm -rf "${BAG_DIR}"

  ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    patients:="${PATIENT_ID}" \
    enable_alerts:=false \
    scenario:=normal \
    sigterm_timeout:=2 \
    sigkill_timeout:=2 \
    > "${LIVE_LOG}" 2>&1 &
  LIVE_PID=$!

  wait_for_topic "/${PATIENT_ID}/patient_vitals"

  ros2 bag record -o "${BAG_DIR}" "/${PATIENT_ID}/patient_vitals" \
    </dev/null > "${RECORD_LOG}" 2>&1 &
  RECORD_PID=$!

  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if [ -d "${BAG_DIR}" ]; then
      break
    fi
    sleep 0.5
  done
  if [ ! -d "${BAG_DIR}" ]; then
    echo "ERROR: bag directory was not created: ${BAG_DIR}" >&2
    return 1
  fi

  # Capture normal samples first, then inject a deterministic abnormal sample.
  sleep 2
  publish_abnormal_vitals_once
  sleep 3

  stop_pid "bag record"  "${RECORD_PID}"
  RECORD_PID=""
  stop_pid "live launch" "${LIVE_PID}"
  LIVE_PID=""

  ros2 bag info "${BAG_DIR}" | tee "${BAG_INFO_LOG}"
  grep -n "Topic: /${PATIENT_ID}/patient_vitals" "${BAG_INFO_LOG}"
  grep -n "Topic: /${PATIENT_ID}/patient_vitals" "${BAG_INFO_LOG}" | grep -E "Count: [1-9]"
}

replay_bag_and_expect_control() {
  echo "=== Phase 2: Replay bag through icu_replay.launch.py with enable_closed_loop=true ==="

  ros2 launch medical_robot_sim icu_replay.launch.py \
    patients:="${PATIENT_ID}" \
    enable_alerts:=true \
    enable_closed_loop:=true \
    control_low_spo2:="${CONTROL_LOW_SPO2}" \
    control_critical_spo2:="${CONTROL_CRITICAL_SPO2}" \
    control_cooldown_sec:="${CONTROL_COOLDOWN_SEC}" \
    sigterm_timeout:=2 \
    sigkill_timeout:=2 \
    > "${REPLAY_LOG}" 2>&1 &
  REPLAY_PID=$!

  # Wait for both alerts and control_actions topics to appear.
  wait_for_topic "/${PATIENT_ID}/alerts"
  wait_for_topic "/${PATIENT_ID}/control_actions"

  # Start echo listeners before replaying.
  timeout "${ECHO_WAIT_SEC}s" ros2 topic echo \
    "/${PATIENT_ID}/alerts" medical_interfaces/msg/Alert --once \
    > "${ALERT_ONCE_LOG}" 2>&1 &
  ECHO_ALERT_PID=$!

  timeout "${ECHO_WAIT_SEC}s" ros2 topic echo \
    "/${PATIENT_ID}/control_actions" medical_interfaces/msg/Alert --once \
    > "${CONTROL_ONCE_LOG}" 2>&1 &
  ECHO_CONTROL_PID=$!

  sleep 1

  ros2 bag play "${BAG_DIR}" --loop </dev/null > "${PLAY_LOG}" 2>&1 &
  PLAY_PID=$!

  # Wait for the alert echo to receive at least one message.
  if ! wait "${ECHO_ALERT_PID}"; then
    ECHO_ALERT_PID=""
    echo "ERROR: did not receive replay-generated alert" >&2
    return 1
  fi
  ECHO_ALERT_PID=""
  echo "Alert received from replay."

  # Wait for control echo (control action from closed-loop controller).
  if ! wait "${ECHO_CONTROL_PID}"; then
    ECHO_CONTROL_PID=""
    echo "ERROR: did not receive control action during replay" >&2
    return 1
  fi
  ECHO_CONTROL_PID=""
  echo "Control action received from replay."

  # Verify alert content.
  grep -n "patient_id: ${PATIENT_ID}" "${ALERT_ONCE_LOG}"
  grep -n "rule_id:" "${ALERT_ONCE_LOG}"

  # Verify control action content.
  grep -n "kind: control_action" "${CONTROL_ONCE_LOG}"
  grep -n "rule_id: control\." "${CONTROL_ONCE_LOG}"

  # If EXPECTED_CONTROL_RULE_ID is set, verify exact rule was produced.
  if [ -n "${EXPECTED_CONTROL_RULE_ID}" ]; then
    if ! grep -n "rule_id: ${EXPECTED_CONTROL_RULE_ID}" "${CONTROL_ONCE_LOG}" \
      && ! grep -n "event=control.publish.*rule_id=${EXPECTED_CONTROL_RULE_ID}" "${REPLAY_LOG}"; then
      echo "ERROR: expected control rule not observed: ${EXPECTED_CONTROL_RULE_ID}" >&2
      return 1
    fi
  fi

  # Verify control events in launch log.
  grep -n "event=control\.config" "${REPLAY_LOG}"
  grep -n "event=control\.publish" "${REPLAY_LOG}"

  stop_pid "bag play"      "${PLAY_PID}"
  PLAY_PID=""
  stop_pid "replay launch" "${REPLAY_PID}"
  REPLAY_PID=""

  # Clean shutdown must not leave stack traces.
  if grep -n "Traceback\|KeyboardInterrupt" "${REPLAY_LOG}"; then
    echo "ERROR: stack trace found on shutdown of replay launch" >&2
    return 1
  fi
}

main() {
  echo "Using work directory: ${WORK_DIR}"
  mkdir -p "${WORK_DIR}"

  record_bag
  replay_bag_and_expect_control

  echo "CI Day19 rosbag replay + closed-loop control smoke test passed"
}

main "$@"
