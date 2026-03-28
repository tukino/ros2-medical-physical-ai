# Day14 Edgeデバイス（Jetson） 受け入れ基準（acceptance）

## 0. 重要（コマンドのコピペ方法）

- ターミナルには **このドキュメント内の ```bash``` ブロックの中身だけ** をコピーして実行する。
- `[...] ( ... )` のような **Markdown リンク表記**はコマンドではない。

## 0.1 共通セットアップ（毎ターミナル）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

（ビルド済みなら）:

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

## 2. 前提: Day14 の運用入口（script/launch）が存在する

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  set -euo pipefail

  test -f src/medical_robot_sim/scripts/jetson_env_report.sh
  ros2 launch medical_robot_sim icu_edge.launch.py --show-args >/dev/null

  echo "OK: day14 entrypoints exist"
)
```

期待条件:
- `jetson_env_report.sh` が存在する
- `icu_edge.launch.py --show-args` が 0 で終了する

## 3. 環境レポートが取得できる（Jetson 以外は N/A で良い）

```bash
(
  set -euo pipefail

  rm -f /tmp/day14_env_report.txt
  bash src/medical_robot_sim/scripts/jetson_env_report.sh | tee /tmp/day14_env_report.txt

  # 必ず arch は出る
  grep -n "arch=" /tmp/day14_env_report.txt

  # Jetson 以外は N/A でもOK（exit 0 が必須）
  echo "OK: env report captured"
)
```

期待条件:
- `/tmp/day14_env_report.txt` に `arch=` が含まれる

## 4. Edge 起動で vitals が publish される（必須）

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export ROS2CLI_NO_DAEMON=1

  rm -f /tmp/day14_edge_pipeline.log
  echo "INFO: launching icu_edge (mock normal)"
  ros2 launch medical_robot_sim icu_edge.launch.py \
    patient:=patient_01 \
    driver:=mock \
    mock_scenario:=normal \
    enable_alerts:=false \
    > /tmp/day14_edge_pipeline.log 2>&1 &
  LAUNCH_PID=$!

  sleep 3

  rm -f /tmp/day14_vitals_once.txt
  echo "INFO: waiting for one vitals message (timeout=10s)"
  timeout 10s ros2 topic echo /patient_01/patient_vitals --once \
    --qos-history keep_last --qos-depth 10 \
    --qos-reliability reliable --qos-durability volatile \
    > /tmp/day14_vitals_once.txt 2>&1

  echo "INFO: stopping launch (pid=${LAUNCH_PID})"
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true

  grep -n "patient_id" /tmp/day14_vitals_once.txt
  echo "OK: vitals published"
)
```

期待条件:
- `ros2 topic echo /patient_01/patient_vitals --once` が 10 秒以内に 1 件受信して終了する
- `/tmp/day14_vitals_once.txt` に `patient_id` が含まれる

## 5. Edge 起動で alerts が生成される（必須）

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export ROS2CLI_NO_DAEMON=1

  rm -f /tmp/day14_edge_alerts_pipeline.log
  echo "INFO: launching icu_edge (mock spo2_drop, alerts enabled)"
  ros2 launch medical_robot_sim icu_edge.launch.py \
    patient:=patient_01 \
    driver:=mock \
    mock_scenario:=spo2_drop \
    enable_alerts:=true \
    > /tmp/day14_edge_alerts_pipeline.log 2>&1 &
  LAUNCH_PID=$!

  sleep 3

  rm -f /tmp/day14_alerts_once.txt
  echo "INFO: waiting for one alert message (timeout=25s)"
  timeout 25s ros2 topic echo /patient_01/alerts --once \
    --qos-history keep_last --qos-depth 10 \
    --qos-reliability reliable --qos-durability volatile \
    > /tmp/day14_alerts_once.txt 2>&1

  echo "INFO: stopping launch (pid=${LAUNCH_PID})"
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true

  grep -n "rule_id" /tmp/day14_alerts_once.txt
  echo "OK: alerts generated"
)
```

期待条件:
- `ros2 topic echo /patient_01/alerts --once` が 25 秒以内に 1 件受信して終了する
- `/tmp/day14_alerts_once.txt` に `rule_id` が含まれる

## 6. Ctrl+C でスタックトレースが出ない（最小）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

grep -n "Traceback" /tmp/day14_edge_pipeline.log || true
grep -n "KeyboardInterrupt" /tmp/day14_edge_pipeline.log || true

grep -n "Traceback" /tmp/day14_edge_alerts_pipeline.log || true
grep -n "KeyboardInterrupt" /tmp/day14_edge_alerts_pipeline.log || true
```

期待条件:
- `Traceback` が出ない
- `KeyboardInterrupt` が出ない

## 7. （任意 / Jetson の場合）`tegrastats` で負荷を記録できる

このステップは環境依存のため、未導入なら N/A とする。

```bash
(
  set -euo pipefail

  if command -v tegrastats >/dev/null 2>&1; then
    echo "INFO: tegrastats exists; capturing 3 samples"
    (timeout 3s tegrastats) > /tmp/day14_tegrastats.txt 2>&1 || true
    test -s /tmp/day14_tegrastats.txt
    head -n 3 /tmp/day14_tegrastats.txt
    echo "OK: tegrastats captured"
  else
    echo "N/A: tegrastats not installed"
  fi
)
```

期待条件（任意）:
- `tegrastats` がある環境では `/tmp/day14_tegrastats.txt` に 1 行以上出力される
