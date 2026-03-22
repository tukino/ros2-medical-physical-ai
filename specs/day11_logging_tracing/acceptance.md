# Day11 Logging / Tracing 受け入れ基準（acceptance）

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

## 2. baseline 起動で `vitals.fault_config` が出る

起動（ログをファイルへ）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_baseline.log
timeout -s INT 5s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  > /tmp/day11_baseline.log 2>&1

# fault config が出ていること
# （event形式は実装で確定。最低限 event=vitals.fault_config を含むこと）
grep -n "vitals\.fault_config" /tmp/day11_baseline.log
```

期待条件:
- `vitals.fault_config` が 1回以上出力される

## 3. pause 注入で enter/exit が出る

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_pause.log
# すぐ pause に入って、少しで復帰する設定
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_pause_after_sec:=1.0 \
  vitals_fault_pause_duration_sec:=2.0 \
  > /tmp/day11_pause.log 2>&1 &
PID=$!

sleep 6
kill -INT "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true

grep -n "vitals\.pause_enter" /tmp/day11_pause.log
grep -n "vitals\.pause_exit" /tmp/day11_pause.log
```

期待条件:
- `vitals.pause_enter` と `vitals.pause_exit` がそれぞれ 1回以上出力される

## 4. stop 注入で trigger が出る

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_stop.log
timeout 8s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_stop_after_sec:=2.0 \
  > /tmp/day11_stop.log 2>&1

grep -n "vitals\.stop_trigger" /tmp/day11_stop.log
```

期待条件:
- `vitals.stop_trigger` が 1回以上出力される

## 4.1 （任意 / 学習用）drop/delay の「理由」を verbose イベントで追う

`observability_verbose:=true` のときのみ、頻度が高いイベント（drop/delay）をログ化する。

### 4.1.1 drop（`vitals.drop`）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_verbose_drop.log
timeout -s INT 6s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  observability_verbose:=true \
  vitals_fault_drop_rate:=0.5 \
  > /tmp/day11_verbose_drop.log 2>&1

grep -n "event=vitals\.drop" /tmp/day11_verbose_drop.log
```

期待条件（任意）:
- `event=vitals.drop` が 1回以上出力される

### 4.1.2 delay（`vitals.enqueue_delayed`）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_verbose_delay.log
timeout -s INT 6s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  observability_verbose:=true \
  vitals_fault_delay_ms:=500 \
  vitals_fault_jitter_ms:=0 \
  > /tmp/day11_verbose_delay.log 2>&1

grep -n "event=vitals\.enqueue_delayed" /tmp/day11_verbose_delay.log
```

期待条件（任意）:
- `event=vitals.enqueue_delayed` が 1回以上出力される

## 5. `icu_monitor` が状態遷移（FRESH/STALE/NO_DATA）をログ化する

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_monitor_state.log
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_pause_after_sec:=1.0 \
  vitals_fault_pause_duration_sec:=4.0 \
  > /tmp/day11_monitor_state.log 2>&1 &
PID=$!

sleep 8
kill -INT "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true

# 例: event=monitor.patient_state state=STALE など
# 実装により厳密表現は変わって良いが、state を含む行が出ること
grep -n "monitor\.patient_state" /tmp/day11_monitor_state.log
```

期待条件:
- `monitor.patient_state` が複数回出力される
- pause 窓の前後で state が変化した痕跡がログから読み取れる

## 6. `rule_alert_engine` が alert 根拠をログ化する

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_alerts_emit.log
# アラート有効（既定 true）
# scenario は既存の spo2_drop を使ってアラートが出やすい状態にする
timeout -s INT 20s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  scenario:=spo2_drop \
  > /tmp/day11_alerts_emit.log 2>&1

# NOTE: scenario=spo2_drop は開始数秒後から段階低下するため、
# launch オーバーヘッド込みで短時間だと alerts.emit まで到達しないことがある。
# 必要に応じて timeout を延長する。

grep -n "alerts\.emit" /tmp/day11_alerts_emit.log
```

期待条件:
- `alerts.emit` が 1回以上出力される

## 7. Ctrl+C でスタックトレースが出ない

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_ctrlc_clean.log

# NOTE: 起動が遅い環境で `timeout -s INT 3s ros2 ...` を使うと、
# `ros2 launch` が import 中に SIGINT を受けて KeyboardInterrupt の Traceback が出ることがある。
# ここでは「起動後に Ctrl+C（SIGINT）を入れる」形で確認する。
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  > /tmp/day11_ctrlc_clean.log 2>&1 &
PID=$!

sleep 5
kill -INT "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true

grep -n "Traceback" /tmp/day11_ctrlc_clean.log || true
grep -n "KeyboardInterrupt" /tmp/day11_ctrlc_clean.log || true
```

期待条件:
- `Traceback` が出ない
- `KeyboardInterrupt` が出ない

## 8. （任意）Tracing が可能な環境では trace を取れる

環境に `ros2 trace` がある場合のみ実施する。

```bash
# `ros2 trace` サブコマンドが存在するか確認（未インストールならスキップ）
command -v ros2 >/dev/null
if ros2 trace --help >/dev/null 2>&1; then
  echo "ros2 trace: available"
else
  echo "ros2 trace: NOT available (optional step: skipped)"
fi
```

期待条件（任意）:
- `ros2 trace` が利用可能なら help が表示できる
- 利用不可なら「NOT available」と表示され、このセクションは N/A
