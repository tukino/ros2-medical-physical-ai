# Day10 Fault Injection 受け入れ基準（acceptance）

## 0. 重要（コマンドのコピペ方法）

- ターミナルには **このドキュメント内の ```bash``` ブロックの中身だけ** をコピーして実行する。
- `[...] ( ... )` のような **Markdown リンク表記**はコマンドではない。

## 0.1 共通セットアップ（毎ターミナル）

次の2段階で環境を有効化する。

### A) ROS 2 本体（必須）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

### B) ワークスペース（ビルド後に有効）

初回や `install/` が無い場合はビルドしてから:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

すでにビルド済みなら:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### C) 付録: `ros2` CLI が `!rclpy.ok()` で落ちる場合（daemon再起動）

`ros2 topic echo` / `ros2 topic hz` が次のような例外で落ちることがある:
- `xmlrpc.client.Fault: ... !rclpy.ok()`

その場合は daemon を再起動してから再実行する:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 daemon stop || true
ros2 daemon start
sleep 1
```

## 0.2 重要: 多重起動の排除（Validation 前チェック）

Validation（drop/pause/stop）は **その患者 namespace の `vital_sensor` が 1 つだけ**動いている前提。
過去に起動した launch が残っていると、別プロセスから受信できてしまい、drop/pause/stop の確認が成立しない。

確認（Publisher count が 1 であること）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic info -v /patient_01/patient_vitals
```

Publisher count が 2 以上なら、起動中プロセスを止める。
- まずは launch を動かしているターミナルで `Ctrl+C`
- それでも残る場合は PID を確認して停止（例）:

```bash
pgrep -af "ros2 launch medical_robot_sim icu_multi_patient.launch.py" || true
pgrep -af "medical_robot_sim.*vital_sensor" || true
```

## 1. ビルドとテストが通る

次がすべて成功すること:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

colcon test --packages-select medical_robot_sim
colcon test-result --verbose
```

期待条件:
- `colcon test-result --verbose` に failures が 0

## 2. 後方互換: fault 未指定で vitals が流れる

起動:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false
```

別ターミナルで確認:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

timeout 5s ros2 topic echo /patient_01/patient_vitals medical_interfaces/msg/VitalSigns --once
```

期待条件:
- 5秒以内に 1 件受信して終了する

## 3. Drop 注入で観測レートが下がる

起動:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_drop_rate:=0.8 \
  vitals_fault_seed:=42
```

別ターミナルで確認（10秒観測）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

timeout 10s ros2 topic hz /patient_01/patient_vitals
```

期待条件:
- 出力に rate（Hz）が表示される
- 既定（faultなし）よりも rate が明確に低い（目安: おおむね 1/5 程度）

## 4. Pause 注入で「途切れて再開」する

起動（約 5 秒後から約 6 秒間 pause）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_pause_after_sec:=5.0 \
  vitals_fault_pause_duration_sec:=6.0
```

別ターミナルで確認（2回 --once でタイミング差を観測）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 起動直後は受信できる
timeout 3s ros2 topic echo /patient_01/patient_vitals --once

# pause 区間に入るまで待つ（目安）
sleep 6

# pause 区間中は受信できず timeout になる（exit code=124）
set +e
timeout 2s ros2 topic echo /patient_01/patient_vitals --once
RC=$?
echo "echo exit code=$RC"

test $RC -eq 124

# さらに待つと再開し、再び受信できる
sleep 6
timeout 3s ros2 topic echo /patient_01/patient_vitals --once
```

より確実な確認（exit code を連続表示して判定）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 20回ぶん、1回ごとに「受信できたか/timeoutしたか」を表示する
# - rc=0   : 受信できた
# - rc=124 : timeout（受信できない）
for i in $(seq 1 20); do
  TS=$(date +%H:%M:%S)
  set +e
  timeout 1.5s ros2 topic echo /patient_01/patient_vitals --once > /dev/null
  RC=$?
  echo "$TS rc=$RC"
done
```

期待条件（目安）:
- 起動直後は rc=0 が続く
- pause 窓（この例では 5〜11 秒）付近で rc=124 が数回続く
- pause 明けに rc=0 に戻る

期待条件:
- 1回目の `--once` は受信できる
- pause 区間では `timeout 2s ...` が受信できず exit code 124 になる
- pause 後は再び受信できる

## 5. Stop 注入で `vital_sensor` が擬似停止する

起動（約 5 秒後に停止）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_stop_after_sec:=5.0
```

別ターミナルで確認:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 起動直後は受信できる
timeout 3s ros2 topic echo /patient_01/patient_vitals --once

# 停止後は受信できず timeout になる
sleep 7
set +e
timeout 2s ros2 topic echo /patient_01/patient_vitals --once
RC=$?
echo "echo exit code=$RC"

test $RC -eq 124
```

期待条件:
- 停止前は受信できる
- 停止後は受信できず timeout になる
- `Ctrl+C` による停止でも Python の例外スタックトレースが出ない

## 6. Ctrl+C でクリーン停止する（スタックトレース無し）

`Ctrl+C` は手動でも良いが、再現性のため `timeout -s INT`（SIGINT送信）で検証できる。

fault なし（3秒でSIGINT）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day10_ctrlc_clean_default.log
set +e
timeout -s INT 3s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  > /tmp/day10_ctrlc_clean_default.log 2>&1
RC=$?
set -e

echo "launch rc=$RC"
grep -n "Traceback" /tmp/day10_ctrlc_clean_default.log || true
```

fault あり（例: drop）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day10_ctrlc_clean_drop.log
set +e
timeout -s INT 3s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_drop_rate:=0.8 \
  vitals_fault_seed:=42 \
  > /tmp/day10_ctrlc_clean_drop.log 2>&1
RC=$?
set -e

echo "launch rc=$RC"
grep -n "Traceback" /tmp/day10_ctrlc_clean_drop.log || true
```

期待条件:
- どちらのログにも `Traceback` が出ない

## 5.1 Delay 注入（到達遅延）の観測メモ

`delay_ms` は「サンプル遅延（due_tick キュー）」で実装している。
`publish_rate_hz=1.0` だと 1tick=1秒のため、`delay_ms:=500` は 1tick に丸められ、観測が難しい場合がある。

確実に観測したい場合は、`publish_rate_hz` を上げて（例: 10Hz）delay を入れる。

### 補足: `ros2 topic echo` の「型が確定できない」エラー回避

起動直後に `ros2 topic echo <topic> --once` を実行すると、DDS の発見（discovery）が間に合わず
`Could not determine the type for the passed topic` になることがある。

その場合は、メッセージ型を明示して実行する（このプロジェクトの vitals は `medical_interfaces/msg/VitalSigns`）。

### A) baseline（10Hz、delayなし）

ターミナルA:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# launch ではなく vital_sensor 単体で確認（/patient_01 に publish）
ros2 run medical_robot_sim vital_sensor --ros-args \
  -r __ns:=/patient_01 \
  -p patient_id:=patient_01 \
  -p publish_rate_hz:=10.0
```

ターミナルB:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

/usr/bin/time -f "baseline real=%e sec" timeout 2s \
  ros2 topic echo /patient_01/patient_vitals medical_interfaces/msg/VitalSigns --once > /dev/null
```

### B) delay=500ms（10Hz）

ターミナルA（Ctrl+Cで止めてから起動し直す）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run medical_robot_sim vital_sensor --ros-args \
  -r __ns:=/patient_01 \
  -p patient_id:=patient_01 \
  -p publish_rate_hz:=10.0 \
  -p vitals_fault_delay_ms:=500
```

ターミナルB:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

/usr/bin/time -f "delay500 real=%e sec" timeout 2s \
  ros2 topic echo /patient_01/patient_vitals medical_interfaces/msg/VitalSigns --once > /dev/null

```

### C) 1ターミナルで測る（起動直後の 1 件目到達まで）

ターミナル（コピペでOK）:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ns=/delay_probe
topic=$ns/patient_vitals
type=medical_interfaces/msg/VitalSigns

measure_once() {
  label=$1
  delay_ms=$2

  # 先に subscriber を起動して待ち受けてから publisher を起動する。
  # discovery の揺らぎを最小化しつつ、delay_ms が「最初の1件」に効いていることを見やすくする。
  ( /usr/bin/time -f "$label real=%e sec" timeout 8 \
    ros2 topic echo "$topic" "$type" --once > /dev/null ) & echo_pid=$!

  sleep 0.1

  ( ros2 run medical_robot_sim vital_sensor --ros-args \
    --log-level WARN \
    -r __ns:=$ns -r __node:=${label}_delay_probe_vital_sensor \
    -p publish_rate_hz:=10.0 \
    -p vitals_fault_delay_ms:=$delay_ms -p vitals_fault_jitter_ms:=0 -p vitals_fault_drop_rate:=0.0 \
    -p vitals_fault_seed:=1 ) & pub_pid=$!

  wait $echo_pid
  rc=$?

  kill $pub_pid 2>/dev/null
  wait $pub_pid 2>/dev/null || true

  return $rc
}

# warm-up（結果は捨ててOK）
measure_once warmup 0

# 本計測（必要なら複数回繰り返して傾向を見る）
measure_once baseline 0
measure_once delay500 500

# 例: 3回ぶん
# for i in 1 2 3; do
#   measure_once baseline_$i 0
#   measure_once delay500_$i 500
# done
```

期待条件（目安）:
- baseline の `real` より delay500 の `real` が明確に大きい
  - この測定は「最初の1件」なので、publish の初回タイミング（0〜0.1s）や discovery の揺らぎで差が縮むことがある
  - 目安: `delay500 - baseline >= +0.25s` が安定して出る（複数回のうち大半で満たす）
- `timeout` で落ちない（落ちる場合は discovery の揺らぎが大きいので、もう一度実行する）




補足:
- 最初の 1 回目だけ DDS discovery のコールドスタートで極端に遅くなることがある。
  その場合は、`measure_once warmup 0` を 1 回流してから、`baseline` と `delay500` を取り直して比較する。
## 6. 不正パラメータは明確に失敗する

次のような不正値（例: drop_rate > 1）で起動する:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_drop_rate:=1.5
```

期待条件:
- 起動失敗し、ログに「許容される範囲（0.0〜1.0）」が分かるエラーが出る
- vitals topic が安定して生成/更新されない
