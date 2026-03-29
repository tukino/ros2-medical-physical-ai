# Day15 EEG / BCI入力 受け入れ基準（acceptance）

## 0. 重要（コマンドのコピペ方法）

- ターミナルには **このドキュメント内の ```bash``` ブロックの中身だけ** をコピーして実行する。
- `[...] ( ... )` のような **Markdown リンク表記**はコマンドではない。

## 0.1 共通セットアップ（毎ターミナル）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 付録: `ros2` CLI が不安定な場合（daemon再起動）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 daemon stop || true
ros2 daemon start
sleep 1
```

### 付録: 前回のプロセスが残っていそうな場合（掃除）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ps -eo pid,stat,cmd | egrep 'medical_robot_sim|ros2 launch' | grep -v egrep || true

pkill -INT -f "/install/medical_robot_sim/lib/medical_robot_sim/" 2>/dev/null || true
pkill -INT -f "ros2 launch medical_robot_sim" 2>/dev/null || true
sleep 1

pkill -TERM -f "/install/medical_robot_sim/lib/medical_robot_sim/" 2>/dev/null || true
pkill -TERM -f "ros2 launch medical_robot_sim" 2>/dev/null || true
sleep 1

ps -eo pid,stat,cmd | egrep 'medical_robot_sim|ros2 launch' | grep -v egrep || true
```

## 1. ビルドとテストが通る

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

## 2. 前提: Day15 の interface / executable / launch が存在する

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  set -euo pipefail

  # interface
  ros2 interface show medical_interfaces/msg/BCIFeatures >/dev/null

  # executables
  ros2 pkg executables medical_robot_sim | tee /tmp/day15_pkg_execs.txt
  grep -n "bci_sensor" /tmp/day15_pkg_execs.txt
  grep -n "bci_monitor" /tmp/day15_pkg_execs.txt

  # launch
  ros2 launch medical_robot_sim icu_bci.launch.py --show-args >/dev/null

  echo "OK: day15 entrypoints exist"
)
```

期待条件:
- `ros2 interface show ...` が 0 で終了する
- `ros2 pkg executables ...` に `bci_sensor` と `bci_monitor` が含まれる
- `icu_bci.launch.py --show-args` が 0 で終了する

## 3. mock driver で `/patient_01/patient_bci` が publish される（必須）

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export ROS2CLI_NO_DAEMON=1

  rm -f /tmp/day15_bci_pipeline.log
  rm -f /tmp/day15_bci_once.txt

  echo "INFO: launching day15 bci pipeline (mock)"
  ros2 launch medical_robot_sim icu_bci.launch.py \
    patient:=patient_01 \
    driver:=mock \
    mock_scenario:=drowsy \
    publish_rate_hz:=10.0 \
    > /tmp/day15_bci_pipeline.log 2>&1 &
  LAUNCH_PID=$!

  sleep 3

  echo "INFO: waiting for one BCI message (timeout=10s)"
  timeout 10s ros2 topic echo --once \
    --qos-history keep_last \
    --qos-depth 10 \
    --qos-reliability reliable \
    --qos-durability volatile \
    /patient_01/patient_bci medical_interfaces/msg/BCIFeatures \
    > /tmp/day15_bci_once.txt 2>&1

  echo "INFO: stopping launch (pid=${LAUNCH_PID})"
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  DEADLINE=$((SECONDS+8))
  while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
    if [ "${SECONDS}" -ge "${DEADLINE}" ]; then
      break
    fi
    sleep 1
  done
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "WARN: launch still running; sending SIGTERM" >&2
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "WARN: launch still running; sending SIGKILL" >&2
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true

  # 受信内容の最小確認
  if grep -n "patient_id" /tmp/day15_bci_once.txt; then
    echo "OK: bci published"
  else
    echo "ERROR: bci output missing patient_id" >&2
    echo "--- /tmp/day15_bci_once.txt ---" >&2
    cat /tmp/day15_bci_once.txt >&2 || true
    echo "--- /tmp/day15_bci_pipeline.log (tail) ---" >&2
    tail -n 160 /tmp/day15_bci_pipeline.log >&2 || true
    exit 1
  fi
)
```

期待条件:
- `ros2 topic echo /patient_01/patient_bci --once` が 10 秒以内に 1 件受信して終了する
- `/tmp/day15_bci_once.txt` に `patient_id` が含まれる

## 4. 観測ログ（イベント）が出ている（必須）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 起動設定
grep -n "event=bci\.device_config" /tmp/day15_bci_pipeline.log

# 状態遷移（実装により行の厳密表現は変わって良いが、event があること）
grep -n "event=bci\.patient_state" /tmp/day15_bci_pipeline.log
```

期待条件:
- `bci.device_config` が 1 回以上出る
- `bci.patient_state` が 1 回以上出る

## 5. Ctrl+C 停止でスタックトレースが出ない（最小）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

grep -n "Traceback" /tmp/day15_bci_pipeline.log || true
grep -n "KeyboardInterrupt" /tmp/day15_bci_pipeline.log || true
```

期待条件:
- `Traceback` が出ない
- `KeyboardInterrupt` が出ない

## 6. （任意 / 実機がある場合）serial driver で publish できる

このステップは環境依存のため、未導入・デバイス無しなら N/A とする。

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  # optional dependency check
  python3 -c 'import serial; print("OK: pyserial")' 2>/dev/null || {
    echo "N/A: pyserial not installed" >&2
    exit 0
  }

  # NOTE: デバイスパスは環境依存
  # 例: /dev/ttyUSB0 が存在しない場合は N/A
  if [ ! -e /dev/ttyUSB0 ]; then
    echo "N/A: /dev/ttyUSB0 not found" >&2
    exit 0
  fi

  rm -f /tmp/day15_serial_pipeline.log
  ros2 launch medical_robot_sim icu_bci.launch.py \
    patient:=patient_01 \
    driver:=serial \
    serial_port:=/dev/ttyUSB0 \
    publish_rate_hz:=20.0 \
    > /tmp/day15_serial_pipeline.log 2>&1 &
  PID=$!

  sleep 3

  timeout 10s ros2 topic echo --once \
    --qos-history keep_last \
    --qos-depth 10 \
    --qos-reliability reliable \
    --qos-durability volatile \
    /patient_01/patient_bci medical_interfaces/msg/BCIFeatures \
    >/dev/null

  kill -INT "$PID" 2>/dev/null || true
  wait "$PID" 2>/dev/null || true

  echo "OK: serial bci published"
)
```

期待条件:
- 実機がある環境では 1 件以上受信できる
- 実機が無い/未導入の環境では N/A で終了 0
