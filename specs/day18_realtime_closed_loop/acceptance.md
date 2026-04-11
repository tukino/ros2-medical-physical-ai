# Day18 リアルタイム制御（閉ループ）受け入れ基準（acceptance）

## 0. 重要

- すべて `bash` ブロックをそのままコピーして実行する。
- 判定は必ずログファイル (`/tmp/day18_*.log`) と `grep -n` で行う。
- 実環境差を考慮し、`timeout` は短すぎる値にしない。

## 1. ビルドとテスト

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

colcon test --packages-select medical_robot_sim
colcon test-result --verbose
```

期待条件:

- failures が 0

## 2. エントリポイント存在確認（executable / launch arg）

```bash
(
  set -euo pipefail
  cd ~/ros2_ws
  set +u
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  set -u

  ros2 pkg executables medical_robot_sim | tee /tmp/day18_pkg_execs.txt
  grep -n "closed_loop_controller" /tmp/day18_pkg_execs.txt

  ros2 launch medical_robot_sim icu_multi_patient.launch.py --show-args \
    | tee /tmp/day18_launch_args.txt >/dev/null
  grep -n "enable_closed_loop" /tmp/day18_launch_args.txt
  grep -n "control_" /tmp/day18_launch_args.txt

  echo "OK: day18 entrypoints exist"
)
```

期待条件:

- `closed_loop_controller` が executable 一覧にある
- launch args に `enable_closed_loop` と `control_` がある

## 3. 後方互換確認（閉ループ無効時は control topic を出さない）

```bash
(
  set -euo pipefail
  cd ~/ros2_ws
  set +u
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  set -u

  export ROS2CLI_NO_DAEMON=1

  pkill -INT -f "ros2 laun[ch] medical_robot_sim icu_multi_patient.launch.py" 2>/dev/null || true
  pkill -INT -f "__node:=closed_loop_controll[e]r" 2>/dev/null || true
  sleep 1

  rm -f /tmp/day18_no_loop.log

  ros2 laun"ch" medical_robot_sim icu_multi_patient.launch.py \
    patients:=patient_01 \
    enable_alerts:=true \
    enable_closed_loop:=false \
    scenario:=flatline \
    > /tmp/day18_no_loop.log 2>&1 &
  LAUNCH_PID=$!

  sleep 10

  if ros2 topic list | grep -q "^/patient_01/control_actions$"; then
    echo "ERROR: control topic exists while enable_closed_loop=false" >&2
    tail -n 200 /tmp/day18_no_loop.log >&2 || true
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    DEADLINE=$((SECONDS+10))
    while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
      if [ "${SECONDS}" -ge "${DEADLINE}" ]; then
        break
      fi
      sleep 1
    done
    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
      sleep 2
    fi
    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    fi
    wait "${LAUNCH_PID}" 2>/dev/null || true
    exit 1
  fi

  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  DEADLINE=$((SECONDS+10))
  while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
    if [ "${SECONDS}" -ge "${DEADLINE}" ]; then
      break
    fi
    sleep 1
  done
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true

  grep -n "event=control\." /tmp/day18_no_loop.log && {
    echo "ERROR: control events emitted while disabled" >&2
    exit 1
  } || true

  echo "OK: backward compatibility preserved"
)
```

期待条件:

- `/patient_01/control_actions` が存在しない
- `event=control.*` が出ない

## 4. 閉ループ有効時の action 発行とログ観測（必須）

```bash
(
  set -euo pipefail
  cd ~/ros2_ws
  set +u
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  set -u

  export ROS2CLI_NO_DAEMON=1

  pkill -INT -f "ros2 laun[ch] medical_robot_sim icu_multi_patient.launch.py" 2>/dev/null || true
  pkill -INT -f "__node:=closed_loop_controll[e]r" 2>/dev/null || true
  sleep 1

  rm -f /tmp/day18_loop.log

  ros2 laun"ch" medical_robot_sim icu_multi_patient.launch.py \
    patients:=patient_01 \
    enable_alerts:=true \
    enable_advisories:=false \
    enable_closed_loop:=true \
    control_low_spo2:=95 \
    control_critical_spo2:=92 \
    control_cooldown_sec:=3.0 \
    scenario:=flatline \
    > /tmp/day18_loop.log 2>&1 &
  LAUNCH_PID=$!

  FOUND=0
  DEADLINE=$((SECONDS+20))
  while [ "${SECONDS}" -lt "${DEADLINE}" ]; do
    if ros2 topic list 2>/dev/null | grep -q "^/patient_01/control_actions$"; then
      FOUND=1
      break
    fi
    sleep 1
  done

  if [ "${FOUND}" -ne 1 ]; then
    echo "ERROR: /patient_01/control_actions not found" >&2
    tail -n 200 /tmp/day18_loop.log >&2 || true
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    DEADLINE=$((SECONDS+10))
    while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
      if [ "${SECONDS}" -ge "${DEADLINE}" ]; then
        break
      fi
      sleep 1
    done
    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
      sleep 2
    fi
    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
    fi
    wait "${LAUNCH_PID}" 2>/dev/null || true
    exit 1
  fi

  timeout -s INT 15s ros2 topic echo --once /patient_01/control_actions \
    > /tmp/day18_control_once.txt 2>&1 || true

  grep -n "event=control\.config" /tmp/day18_loop.log
  grep -n "event=control\.decision" /tmp/day18_loop.log
  grep -n "event=control\.publish" /tmp/day18_loop.log

  grep -n "kind: control_action" /tmp/day18_control_once.txt
  grep -n "rule_id: control\." /tmp/day18_control_once.txt

  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  DEADLINE=$((SECONDS+10))
  while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
    if [ "${SECONDS}" -ge "${DEADLINE}" ]; then
      break
    fi
    sleep 1
  done
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true

  if grep -n "Traceback\|KeyboardInterrupt" /tmp/day18_loop.log; then
    echo "ERROR: stack trace found on shutdown" >&2
    exit 1
  fi

  echo "OK: closed-loop behavior observed"
)
```

期待条件:

- `/patient_01/control_actions` が存在する
- `control.config/decision/publish` がログに出る
- `control_actions` メッセージに `kind: control_action` と `rule_id: control.*` が含まれる
- 停止時にスタックトレースが出ない

## 5. cooldown 抑制の観測（必須）

```bash
(
  set -euo pipefail
  cd ~/ros2_ws
  set +u
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  set -u

  rm -f /tmp/day18_cooldown.log

  timeout -s INT 35s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    patients:=patient_01 \
    enable_alerts:=true \
    enable_closed_loop:=true \
    control_cooldown_sec:=8.0 \
    scenario:=flatline \
    > /tmp/day18_cooldown.log 2>&1 || true

  grep -n "event=control\.suppressed" /tmp/day18_cooldown.log
  echo "OK: cooldown suppression observed"
)
```

期待条件:

- `event=control.suppressed` が1件以上ある

## 6. optional: tracing 連携確認（未導入なら N/A）

```bash
(
  set -euo pipefail

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "N/A: ros2 command not available"
    exit 0
  fi

  if ros2 run tracetools_trace trace --help >/dev/null 2>&1; then
    echo "OK: tracing tool available"
    echo "optional check: run trace around day18 launch if needed"
  else
    echo "N/A: tracetools_trace is not installed in this environment"
  fi
)
```

期待条件:

- tracing があれば `OK`
- 未導入なら `N/A` を許容（fail 扱いにしない）
