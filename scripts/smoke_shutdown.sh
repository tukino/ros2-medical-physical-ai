#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_SETUP="/opt/ros/humble/setup.bash"
OVERLAY_SETUP="$WS_DIR/install/setup.bash"

if [[ -f "$ROS_SETUP" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
else
  echo "ERROR: ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

if [[ -f "$OVERLAY_SETUP" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$OVERLAY_SETUP"
  set -u
else
  echo "ERROR: Overlay setup not found: $OVERLAY_SETUP" >&2
  echo "Hint: run 'colcon build --symlink-install' in $WS_DIR" >&2
  exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "ERROR: 'timeout' command not found (coreutils)." >&2
  exit 1
fi

LAUNCH_PKG="${LAUNCH_PKG:-medical_robot_sim}"
LAUNCH_FILE="${LAUNCH_FILE:-icu_multi_patient.launch.py}"
PATIENTS="${PATIENTS:-patient_01,patient_02}"

STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-3}"
SHUTDOWN_TIMEOUT_SEC="${SHUTDOWN_TIMEOUT_SEC:-20}"

SIGTERM_TIMEOUT="${SIGTERM_TIMEOUT:-}"
SIGKILL_TIMEOUT="${SIGKILL_TIMEOUT:-}"

launch_args=(
  "$LAUNCH_PKG" "$LAUNCH_FILE"
  "patients:=$PATIENTS"
)

if [[ -n "$SIGTERM_TIMEOUT" ]]; then
  launch_args+=("sigterm_timeout:=$SIGTERM_TIMEOUT")
fi
if [[ -n "$SIGKILL_TIMEOUT" ]]; then
  launch_args+=("sigkill_timeout:=$SIGKILL_TIMEOUT")
fi

LOG_FILE="${LOG_FILE:-/tmp/ros2_smoke_launch.log}"

LAUNCH_PID=""
CHILD_PIDS=()

collect_child_pids() {
  local -A seen
  local -a pids
  local pid

  pids=()

  if [[ -f "$LOG_FILE" ]]; then
    while IFS= read -r pid; do
      [[ -n "$pid" ]] && pids+=("$pid")
    done < <(
      grep -Eo 'process started with pid \[[0-9]+\]' "$LOG_FILE" \
        | grep -Eo '[0-9]+'
    )
  fi

  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
    while IFS= read -r pid; do
      [[ -n "$pid" ]] && pids+=("$pid")
    done < <(pgrep -P "$LAUNCH_PID" 2>/dev/null || true)
  fi

  CHILD_PIDS=()
  for pid in "${pids[@]}"; do
    if [[ -z "${seen[$pid]+x}" ]]; then
      seen[$pid]=1
      CHILD_PIDS+=("$pid")
    fi
  done
}

any_pid_alive() {
  local pid
  for pid in "${CHILD_PIDS[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      return 0
    fi
  done
  return 1
}

count_patients() {
  local -a raw
  local item
  local count=0
  IFS=',' read -r -a raw <<< "$PATIENTS"
  for item in "${raw[@]}"; do
    item="${item//[[:space:]]/}"
    if [[ -n "$item" ]]; then
      count=$((count + 1))
    fi
  done
  echo "$count"
}

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    collect_child_pids || true

    kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
    sleep 0.5
    kill -TERM "$LAUNCH_PID" >/dev/null 2>&1 || true

    local pid
    for pid in "${CHILD_PIDS[@]}"; do
      kill -INT "$pid" >/dev/null 2>&1 || true
    done
    sleep 0.5
    for pid in "${CHILD_PIDS[@]}"; do
      kill -TERM "$pid" >/dev/null 2>&1 || true
    done
    sleep 0.5
    for pid in "${CHILD_PIDS[@]}"; do
      kill -KILL "$pid" >/dev/null 2>&1 || true
    done
  fi
}
trap cleanup EXIT

cd "$WS_DIR"

echo "Starting: ros2 launch ${launch_args[*]}"
rm -f "$LOG_FILE" || true
# Run via bash wrapper and unbuffered Python so PID lines appear promptly.
bash -c 'trap - INT; trap - TERM; export PYTHONUNBUFFERED=1; exec ros2 launch "$@"' _ "${launch_args[@]}" \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID="$!"

echo "Launched (pid=${LAUNCH_PID}), waiting ${STARTUP_WAIT_SEC}s..."
expected_children=$(( $(count_patients) + 1 ))
if [[ "$expected_children" -le 1 ]]; then
  echo "ERROR: PATIENTS resulted in empty list: '$PATIENTS'" >&2
  exit 1
fi

deadline_start=$((SECONDS + STARTUP_WAIT_SEC))
while (( SECONDS < deadline_start )); do
  collect_child_pids || true
  if (( ${#CHILD_PIDS[@]} >= expected_children )); then
    break
  fi
  sleep 0.2
done

collect_child_pids || true
echo "Detected child pids (${#CHILD_PIDS[@]}/${expected_children}): ${CHILD_PIDS[*]:-}" 

echo "Sending SIGINT to launch pid ${LAUNCH_PID}"
kill -INT "$LAUNCH_PID" || true
for pid in "${CHILD_PIDS[@]}"; do
  kill -INT "$pid" >/dev/null 2>&1 || true
done

deadline_shutdown=$((SECONDS + SHUTDOWN_TIMEOUT_SEC))
while (( SECONDS < deadline_shutdown )); do
  collect_child_pids || true
  if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1 && ! any_pid_alive; then
    echo "OK: launch and child processes exited"
    exit 0
  fi
  sleep 0.2
done

echo "FAIL: processes still running after ${SHUTDOWN_TIMEOUT_SEC}s" >&2
if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  ps -o pid,ppid,pgid,stat,cmd -p "$LAUNCH_PID" >&2 || true
fi
for pid in "${CHILD_PIDS[@]}"; do
  if kill -0 "$pid" >/dev/null 2>&1; then
    ps -o pid,ppid,pgid,stat,cmd -p "$pid" >&2 || true
  fi
done

echo "--- tail of ${LOG_FILE} ---" >&2
tail -n 200 "$LOG_FILE" >&2 || true

exit 1

