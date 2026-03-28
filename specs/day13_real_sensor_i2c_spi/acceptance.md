# Day13 実センサー接続（I2C/SPI） 受け入れ基準（acceptance）

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

## 2. 前提: icu_hw_sensor.launch.py と hw_vital_sensor が存在する

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# NOTE: `set -euo pipefail` はターミナルに残るので、サブシェルで閉じ込める
(
  set -euo pipefail

  ros2 launch medical_robot_sim icu_hw_sensor.launch.py --show-args >/dev/null
  ros2 pkg executables medical_robot_sim | tee /tmp/day13_pkg_execs.txt

  grep -n "hw_vital_sensor" /tmp/day13_pkg_execs.txt
) && echo "OK: launch and executable exist"
```

期待条件:
- `--show-args` が 0 で終了する
- `ros2 pkg executables medical_robot_sim` の出力に `hw_vital_sensor` が含まれる

## 3. mock driver で vitals が publish される（必須）

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  # ros2 daemon が不安定な環境では、topic 系コマンドが失敗することがある
  export ROS2CLI_NO_DAEMON=1

  rm -f /tmp/day13_mock_pipeline.log
  echo "INFO: launching mock pipeline (normal)"
  ros2 launch medical_robot_sim icu_hw_sensor.launch.py \
    patient:=patient_01 \
    driver:=mock \
    mock_scenario:=normal \
    enable_alerts:=false \
    > /tmp/day13_mock_pipeline.log 2>&1 &
  LAUNCH_PID=$!

  # 起動待ち
  sleep 3

  # vitals を 1 件受信できる
  rm -f /tmp/day13_vitals_once.txt
  echo "INFO: waiting for one vitals message (timeout=8s)"
  timeout 8s ros2 topic echo /patient_01/patient_vitals --once \
    --qos-history keep_last --qos-depth 10 \
    --qos-reliability reliable --qos-durability volatile \
    > /tmp/day13_vitals_once.txt 2>&1

  # 停止
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
  if grep -n "patient_id" /tmp/day13_vitals_once.txt; then
    echo "OK: mock vitals published"
  else
    echo "ERROR: vitals output missing patient_id" >&2
    echo "--- /tmp/day13_vitals_once.txt ---" >&2
    cat /tmp/day13_vitals_once.txt >&2 || true
    echo "--- /tmp/day13_mock_pipeline.log (tail) ---" >&2
    tail -n 120 /tmp/day13_mock_pipeline.log >&2 || true
    exit 1
  fi
) || echo "FAILED: mock vitals publish check (see logs under /tmp)" >&2
```

期待条件:
- `ros2 topic echo /patient_01/patient_vitals --once` が 8 秒以内に 1 件受信して終了する
- `/tmp/day13_vitals_once.txt` に `patient_id` が含まれる

## 4. mock の異常シナリオで alerts が生成される（必須）

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export ROS2CLI_NO_DAEMON=1

  rm -f /tmp/day13_mock_alerts_pipeline.log
  echo "INFO: launching mock pipeline (spo2_drop)"
  ros2 launch medical_robot_sim icu_hw_sensor.launch.py \
    patient:=patient_01 \
    driver:=mock \
    mock_scenario:=spo2_drop \
    enable_alerts:=true \
    > /tmp/day13_mock_alerts_pipeline.log 2>&1 &
  LAUNCH_PID=$!

  # 起動待ち
  sleep 3

  rm -f /tmp/day13_alerts_once.txt
  echo "INFO: waiting for one alert message (timeout=20s)"
  timeout 20s ros2 topic echo /patient_01/alerts --once \
    --qos-history keep_last --qos-depth 10 \
    --qos-reliability reliable --qos-durability volatile \
    > /tmp/day13_alerts_once.txt 2>&1

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
  if grep -n "rule_id" /tmp/day13_alerts_once.txt; then
    echo "OK: mock alerts generated"
  else
    echo "ERROR: alerts output missing rule_id" >&2
    echo "--- /tmp/day13_alerts_once.txt ---" >&2
    cat /tmp/day13_alerts_once.txt >&2 || true
    echo "--- /tmp/day13_mock_alerts_pipeline.log (tail) ---" >&2
    tail -n 160 /tmp/day13_mock_alerts_pipeline.log >&2 || true
    exit 1
  fi
) || echo "FAILED: mock alerts generation check (see logs under /tmp)" >&2
```

期待条件:
- `ros2 topic echo /patient_01/alerts --once` が 20 秒以内に 1 件受信して終了する
- `/tmp/day13_alerts_once.txt` に `rule_id` が含まれる

## 5. Ctrl+C でスタックトレースが出ない（最小）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

grep -n "Traceback" /tmp/day13_mock_pipeline.log || true
grep -n "KeyboardInterrupt" /tmp/day13_mock_pipeline.log || true

grep -n "Traceback" /tmp/day13_mock_alerts_pipeline.log || true
grep -n "KeyboardInterrupt" /tmp/day13_mock_alerts_pipeline.log || true
```

期待条件:
- `Traceback` が出ない
- `KeyboardInterrupt` が出ない

## 6. （任意 / 実機がある場合）I2C/SPI driver で publish できる

このステップは環境依存のため、未導入・デバイス無しなら N/A とする。

### 6.0 任意依存（Pythonライブラリ）を入れる（I2C/SPI を試す場合）

I2C/SPI は「optional driver」のため、OS への導入状況によってはここで失敗する。

```bash
# I2C 用（smbus2）: 無ければ入れる
python3 -c 'import smbus2; print("OK: smbus2")' 2>/dev/null || {
  echo "INFO: smbus2 not found; installing (optional)" >&2
  sudo apt-get update && sudo apt-get install -y python3-smbus2 || true
  python3 -m pip install --user smbus2 || true
  python3 -c 'import smbus2; print("OK: smbus2")'
}

# SPI 用（spidev）: 無ければ入れる
python3 -c 'import spidev; print("OK: spidev")' 2>/dev/null || {
  echo "INFO: spidev not found; installing (optional)" >&2
  sudo apt-get update && sudo apt-get install -y python3-spidev || true
  python3 -m pip install --user spidev || true
  python3 -c 'import spidev; print("OK: spidev")'
}
```

### 6.1 I2C デバイスが見える（N/A 判定あり）

```bash
# I2C デバイスが無い環境では N/A
ls -1 /dev/i2c-* >/tmp/day13_i2c_devs.txt 2>&1 || true
cat /tmp/day13_i2c_devs.txt

if grep -q "No such file" /tmp/day13_i2c_devs.txt; then
  echo "N/A: no /dev/i2c-* devices"
  echo "SKIP: optional i2c steps"
elif grep -qi "Permission denied" /tmp/day13_i2c_devs.txt; then
  echo "N/A: permission denied listing /dev/i2c-*" >&2
  echo "HINT: add user to i2c group or adjust udev permissions" >&2
else
  echo "OK: i2c devices exist (optional path continues)"
fi

true
```

### 6.2 I2C driver 起動（手動で bus/address を合わせる）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# NOTE: まずデバイスノードがあるか確認（無ければここは N/A）
if [ ! -e /dev/i2c-1 ]; then
  echo "N/A: /dev/i2c-1 does not exist (check 6.1)." >&2
  echo "HINT: on Linux you may need: sudo modprobe i2c-dev" >&2
  echo "HINT: on WSL, /dev/i2c-* is typically unavailable." >&2
  true
  exit 0
fi

# NOTE: ここは実機構成に依存するため、bus/address は現場に合わせて上書きする
rm -f /tmp/day13_i2c_pipeline.log
timeout -s INT 10s ros2 launch medical_robot_sim icu_hw_sensor.launch.py \
  patient:=patient_01 \
  driver:=i2c \
  i2c_bus:=1 \
  i2c_addr:=0x57 \
  enable_alerts:=false \
  > /tmp/day13_i2c_pipeline.log 2>&1 || true

grep -n "event=vitals\\.hw_device_open" /tmp/day13_i2c_pipeline.log || true

echo "OK: i2c optional run finished"
```

期待条件（任意）:
- `event=vitals.hw_device_open_ok` が出る
- NOTE: 現状の I2C/SPI は「open できるところまで」の最小実装で、`read_sample()` は未実装。
  実センサーのレジスタ仕様に合わせて `read_sample()` を実装した場合に限り、vitals publish を `ros2 topic echo ... --once` で確認できる。

補足:
- `event=vitals.hw_device_open_fail ... error=I2C_driver_requires_smbus2_...` の場合は、上の「6.0 任意依存」を実行してから再試行する
- `/dev/i2c-*` が無い環境（例: WSL 等）では I2C 経路は N/A になり得る（6.1 を優先して確認）
