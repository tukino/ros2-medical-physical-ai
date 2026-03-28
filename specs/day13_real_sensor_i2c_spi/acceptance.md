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

ros2 launch medical_robot_sim icu_hw_sensor.launch.py --show-args >/dev/null
ros2 pkg executables medical_robot_sim | tee /tmp/day13_pkg_execs.txt

grep -n "hw_vital_sensor" /tmp/day13_pkg_execs.txt

echo "OK: launch and executable exist"
```

期待条件:
- `--show-args` が 0 で終了する
- `ros2 pkg executables medical_robot_sim` の出力に `hw_vital_sensor` が含まれる

## 3. mock driver で vitals が publish される（必須）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# ros2 daemon が不安定な環境では、topic 系コマンドが失敗することがある
export ROS2CLI_NO_DAEMON=1

rm -f /tmp/day13_mock_pipeline.log
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
timeout 8s ros2 topic echo /patient_01/patient_vitals --once \
  > /tmp/day13_vitals_once.txt 2>&1

# 停止
kill -INT "${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true

# 受信内容の最小確認
grep -n "patient_id" /tmp/day13_vitals_once.txt

echo "OK: mock vitals published"
```

期待条件:
- `ros2 topic echo /patient_01/patient_vitals --once` が 8 秒以内に 1 件受信して終了する
- `/tmp/day13_vitals_once.txt` に `patient_id` が含まれる

## 4. mock の異常シナリオで alerts が生成される（必須）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS2CLI_NO_DAEMON=1

rm -f /tmp/day13_mock_alerts_pipeline.log
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
timeout 20s ros2 topic echo /patient_01/alerts --once \
  > /tmp/day13_alerts_once.txt 2>&1

kill -INT "${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true

# 受信内容の最小確認
grep -n "rule_id" /tmp/day13_alerts_once.txt

echo "OK: mock alerts generated"
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

### 6.1 I2C デバイスが見える（N/A 判定あり）

```bash
# I2C デバイスが無い環境では N/A
ls -1 /dev/i2c-* >/tmp/day13_i2c_devs.txt 2>&1 || true
cat /tmp/day13_i2c_devs.txt

if grep -q "No such file" /tmp/day13_i2c_devs.txt; then
  echo "N/A: no /dev/i2c-* devices"
  echo "SKIP: optional i2c steps"
else
  echo "OK: i2c devices exist (optional path continues)"
fi

true
```

### 6.2 I2C driver 起動（手動で bus/address を合わせる）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

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
- vitals が publish される（同様に `ros2 topic echo ... --once` で確認）
