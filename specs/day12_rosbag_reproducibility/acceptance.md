# Day12 rosbagによる再現性検証 受け入れ基準（acceptance）

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

`ros2 launch` / `ros2 bag` を途中で中断した場合、`vital_sensor` や `rule_alert_engine` が残留して
手順の出力が混線することがある。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ps -eo pid,stat,cmd | egrep 'medical_robot_sim|ros2 launch|ros2 bag (record|play)' | grep -v egrep || true

pkill -INT -f "/install/medical_robot_sim/lib/medical_robot_sim/" 2>/dev/null || true
pkill -INT -f "ros2 launch medical_robot_sim" 2>/dev/null || true
pkill -INT -f "ros2 bag (record|play)" 2>/dev/null || true
sleep 1

pkill -TERM -f "/install/medical_robot_sim/lib/medical_robot_sim/" 2>/dev/null || true
pkill -TERM -f "ros2 launch medical_robot_sim" 2>/dev/null || true
pkill -TERM -f "ros2 bag (record|play)" 2>/dev/null || true
sleep 1

ps -eo pid,stat,cmd | egrep 'medical_robot_sim|ros2 launch|ros2 bag (record|play)' | grep -v egrep || true
```

## 1. 前提: rosbag2 が利用できる

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag --help >/dev/null
echo "ros2 bag --help rc=$? (期待: 0)"
```

期待条件:
- `ros2 bag --help` が 0 で終了する

（参考）もしコマンドが存在しない場合:
- 環境依存のため、この acceptance は **FAIL（前提未達）** とする
- 典型例（Ubuntu）: `sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins`

## 2. ビルドとテストが通る

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

## 3. live 入力を bag に記録し、`ros2 bag info` で Message Count > 0 を確認できる

このセクションでは「入力（vitals）が bag に入る」ことだけを確認する。
alerts は後続の replay で “生成できること” を確認するため、同一 bag への同時記録は必須にしない。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day12_bag_dir.txt
rm -f /tmp/day12_record_live.log
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  > /tmp/day12_record_live.log 2>&1 &
LAUNCH_PID=$!

# 起動待ち
sleep 3

# bag 記録（短時間）
BAG_DIR="/tmp/day12_vitals_$(date +%Y%m%d_%H%M%S)"
rm -rf "${BAG_DIR}"
rm -f /tmp/day12_record_bag.log
rm -f /tmp/day12_bag_info.txt
rm -f /tmp/day12_manual_pub.log

# ros2 bag record をバックグラウンド起動し、記録中に 1 回だけ異常サンプルを投入する
# NOTE: バックグラウンドプロセスが stdin を読もうとして "Stopped" になるのを防ぐため </dev/null を付ける
ros2 bag record -o "${BAG_DIR}" /patient_01/patient_vitals </dev/null > /tmp/day12_record_bag.log 2>&1 &
BAG_PID=$!

# bag ディレクトリが作られるまで少し待つ（作られない場合は record 失敗の可能性が高い）
for i in $(seq 1 10); do
  if [[ -d "${BAG_DIR}" ]]; then
    break
  fi
  sleep 0.3
done
test -d "${BAG_DIR}" || (echo "ERROR: bag dir not created: ${BAG_DIR}" >&2; tail -n 50 /tmp/day12_record_bag.log >&2; exit 1)

sleep 2

# 途中で 1 回だけ異常サンプルを手動投入（bag に入っていれば replay でアラートが出せる）
# NOTE: このリポジトリの /patient_01/patient_vitals は TRANSIENT_LOCAL の QoS で publish されることがある。
#       ros2 bag record に確実に記録させるため、手動 publish 側も QoS を合わせる。
ros2 topic pub --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /patient_01/patient_vitals medical_interfaces/msg/VitalSigns \
  "{patient_id: 'patient_01', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}" \
  > /tmp/day12_manual_pub.log 2>&1 || true

sleep 3

kill -INT "${BAG_PID}" 2>/dev/null || true
wait "${BAG_PID}" 2>/dev/null || true

# 後続ステップ向けに bag パスを保存（別シェルでも復元できるように）
echo "${BAG_DIR}" > /tmp/day12_bag_dir.txt

# 停止
kill -INT "${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true

# bag info で topic と count を確認
ros2 bag info "${BAG_DIR}" | tee /tmp/day12_bag_info.txt

grep -n "Topic: /patient_01/patient_vitals" /tmp/day12_bag_info.txt

# Count が 0 でないこと（最小チェック）
# 実際の出力は rosbag2 のバージョン差があるため、まずは行が存在することを確認し、
# 次に数値が 1 以上らしいことを grep で見る。
grep -n "Topic: /patient_01/patient_vitals" /tmp/day12_bag_info.txt | grep -E "Count: [1-9]" 

echo "OK: record+info passed (BAG_DIR=${BAG_DIR})"
```

期待条件:
- `ros2 bag info` に `/patient_01/patient_vitals` が含まれる
- `Count` が 1 以上である（Message Count > 0）

## 4. replay パイプライン（sensor 無し）で bag を再生し、alerts を生成できる

このセクションは、次が実装されている前提:
- `medical_robot_sim` に replay 用 launch `icu_replay.launch.py` が存在する（sensor を起動しない）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# NOTE: ros2 daemon が不安定な環境では、topic 系コマンドが
#       xmlrpc.client.Fault: ... !rclpy.ok() で失敗することがある。
#       受け入れ手順は daemon を使わない。
export ROS2CLI_NO_DAEMON=1

# Step 3 の bag_dir を復元（同一シェルで実行している場合は既に入っていても良い）
if [[ -z "${BAG_DIR:-}" ]]; then
  BAG_DIR="$(cat /tmp/day12_bag_dir.txt)"
fi
test -d "${BAG_DIR}" || (echo "ERROR: bag dir not found: ${BAG_DIR}" >&2; exit 1)

# replay パイプライン起動（sensor なし）
rm -f /tmp/day12_replay_pipeline.log
rm -f /tmp/day12_bag_play.log
rm -f /tmp/day12_alerts_once.txt
ros2 launch medical_robot_sim icu_replay.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  > /tmp/day12_replay_pipeline.log 2>&1 &
REPLAY_LAUNCH_PID=$!

# 起動待ち（subscriber が揃うまで）
sleep 3

# bag 再生（loop で取りこぼしを減らす）
# NOTE: バックグラウンドプロセスが stdin を読もうとして "Stopped" になるのを防ぐため </dev/null を付ける
ros2 bag play "${BAG_DIR}" --loop </dev/null \
  > /tmp/day12_bag_play.log 2>&1 &
PLAY_PID=$!

# bag play が生きていること（即死していたら bag パス不正など）
sleep 0.5
if ! kill -0 "${PLAY_PID}" 2>/dev/null; then
  echo "ERROR: ros2 bag play exited early" >&2
  tail -n 50 /tmp/day12_bag_play.log >&2
  exit 1
fi

# replay 中に alerts が生成されていること（1件受信できる）
timeout 8s ros2 topic echo /patient_01/alerts --once \
  > /tmp/day12_alerts_once.txt 2>&1

# echo が成功したら、bag play を停止
kill -INT "${PLAY_PID}" 2>/dev/null || true
wait "${PLAY_PID}" 2>/dev/null || true

# 停止
kill -INT "${REPLAY_LAUNCH_PID}" 2>/dev/null || true
wait "${REPLAY_LAUNCH_PID}" 2>/dev/null || true

# 受信内容を軽く確認（patient_id が含まれる）
grep -n "patient_id" /tmp/day12_alerts_once.txt

echo "OK: replay passed (BAG_DIR=${BAG_DIR})"
```

期待条件:
- `ros2 topic echo /patient_01/alerts --once` が 8 秒以内に 1 件受信して終了する
- `/tmp/day12_alerts_once.txt` に `patient_id` が含まれる

## 5. Ctrl+C でスタックトレースが出ない（最小）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

grep -n "Traceback" /tmp/day12_record_live.log || true
grep -n "KeyboardInterrupt" /tmp/day12_record_live.log || true

grep -n "Traceback" /tmp/day12_replay_pipeline.log || true
grep -n "KeyboardInterrupt" /tmp/day12_replay_pipeline.log || true
```

期待条件:
- `Traceback` が出ない
- `KeyboardInterrupt` が出ない

## 6. （任意 / 学習用）alerts も同一 bag に記録できる

このステップは “bag に複数 topic を入れられる” ことの確認であり、replay の合否には使わない。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

BAG_DIR2="/tmp/day12_vitals_alerts_$(date +%Y%m%d_%H%M%S)"
rm -rf "${BAG_DIR2}"

rm -f /tmp/day12_record_both.log
(timeout -s INT 8s ros2 bag record -o "${BAG_DIR2}" \
  /patient_01/patient_vitals /patient_01/alerts) \
  > /tmp/day12_record_both.log 2>&1 || true

ros2 bag info "${BAG_DIR2}" | tee /tmp/day12_bag_info2.txt

grep -n "Topic: /patient_01/patient_vitals" /tmp/day12_bag_info2.txt
grep -n "Topic: /patient_01/alerts" /tmp/day12_bag_info2.txt
```

期待条件（任意）:
- bag に `/patient_01/patient_vitals` と `/patient_01/alerts` が含まれる
