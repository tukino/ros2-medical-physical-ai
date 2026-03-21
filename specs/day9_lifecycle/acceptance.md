# Day9 Lifecycle Node導入 受け入れ基準（acceptance）

## 0. 重要（コマンドのコピペ方法）

- ターミナルには **このドキュメント内の ```bash``` ブロックの中身だけ** をコピーして実行する。
- `[...] ( ... )` のような **Markdown リンク表記**はコマンドではない（貼ると `bash: syntax error near unexpected token '('` になる）。

## 0.1 共通セットアップ（毎ターミナル）

次の2段階で環境を有効化する（`source setup.bash` はワークスペース直下には通常存在しないため使わない）:

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

## 2. 後方互換: classic 起動で従来通り alerts が流れる

起動:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# NOTE: acceptance を安定させるため、発火対象ルールを単一に絞る
ros2 launch medical_robot_sim icu_multi_patient.launch.py patients:=patient_01 enable_alerts:=true alerts_node_kind:=classic enabled_rule_ids:=single.spo2_lt_90
```

別ターミナルで手動 publish → 受信:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# alerts は Durability=volatile が既定のため、先に待ち受けて取り逃がしを防ぐ
timeout 10s ros2 topic echo /patient_01/alerts --once &
sleep 0.2

# edge-trigger のため、まず正常値→異常値の順に投入して「False→True」を作る
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns "{patient_id: 'patient_01', measurement_id: 998, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 95, status: 'monitoring'}"
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns "{patient_id: 'patient_01', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"

wait
```

期待条件:
- `ros2 topic echo /patient_01/alerts --once` が 1 件受信して終了する
- 受信した `rule_id` が `single.spo2_lt_90` である（SpO2 < 90 の単発ルール）

## 3. Lifecycle モード: 手動状態遷移で alerts を制御できる

注意:
- `ros2 lifecycle get/set /rule_alert_engine` が `Node not found` の場合、Lifecycle Node として起動できていない。
  - `ros2 node list | grep rule_alert_engine` でノード自体が存在するか確認する
  - ノードが存在するのに lifecycle が使えない場合、classic 版（`rclpy.node.Node`）が動いている
  - この Day の実装が完了している場合は `ros2 lifecycle nodes` に `/rule_alert_engine` が表示される

### 3.0 確定診断（Node not found の切り分け）

同じターミナルで次を実行し、出力を分けて確認する:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

echo '--- ros2 node list (graph上の存在確認)'
ros2 node list | grep rule_alert_engine || echo '(not found in node list)'

echo '--- ros2 lifecycle nodes (Lifecycleサービス提供ノードのみ)'
ros2 lifecycle nodes | grep rule_alert_engine || echo '(not found in lifecycle nodes)'

echo '--- lifecycle services (サービスが無ければclassicの可能性が高い)'
ros2 service list | grep -E '^/rule_alert_engine/(change_state|get_state|get_available_states|get_available_transitions)$' || echo '(no lifecycle services)'
```

期待される判断:
- `node list` には出るが `lifecycle nodes` と `lifecycle services` が空 → classic が起動中（Day9未実装 or 起動切替できていない）
- `lifecycle nodes` と `lifecycle services` が出る → Lifecycle Node として起動できている

### 3.1 起動直後（autostart=false）では unconfigured で待機する

起動:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# NOTE: acceptance を安定させるため、発火対象ルールを単一に絞る
ros2 launch medical_robot_sim icu_multi_patient.launch.py patients:=patient_01 enable_alerts:=true alerts_node_kind:=lifecycle lifecycle_autostart:=false enabled_rule_ids:=single.spo2_lt_90
```

状態確認:

```bash
ros2 lifecycle get /rule_alert_engine
```

期待条件:
- `ros2 lifecycle get` の出力が `unconfigured`（または同等の未設定状態）を示す

### 3.2 configure -> activate すると alerts が流れる

状態遷移:

```bash
ros2 lifecycle set /rule_alert_engine configure
ros2 lifecycle set /rule_alert_engine activate
ros2 lifecycle get /rule_alert_engine
```

手動 publish → 受信:

```bash
# alerts は Durability=volatile が既定のため、先に待ち受けて取り逃がしを防ぐ
timeout 10s ros2 topic echo /patient_01/alerts --once &
sleep 0.2

ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns "{patient_id: 'patient_01', measurement_id: 998, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 95, status: 'monitoring'}"
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns "{patient_id: 'patient_01', measurement_id: 1000, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"

wait
```

期待条件:
- `ros2 lifecycle set ... configure` の後に `ros2 lifecycle get` が `inactive`（または同等）を示す
- `ros2 lifecycle set ... activate` の後に `ros2 lifecycle get` が `active`（または同等の稼働状態）を示す
- `ros2 topic echo /patient_01/alerts --once` が 1 件受信して終了する
- 受信した `rule_id` が `single.spo2_lt_90` である

### 3.3 deactivate すると alerts が止まる

deactivate:

```bash
ros2 lifecycle set /rule_alert_engine deactivate
ros2 lifecycle get /rule_alert_engine
```

手動 publish 後に、alerts が来ないこと:

```bash
set +e

# もし alerts が出てしまう実装なら、ここで 5 秒以内に 1 件受信して exit code=0 になる
timeout 5s ros2 topic echo /patient_01/alerts --once &
ECHO_PID=$!
sleep 0.2

ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns "{patient_id: 'patient_01', measurement_id: 998, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 95, status: 'monitoring'}"
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns "{patient_id: 'patient_01', measurement_id: 1001, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"

wait $ECHO_PID
RC=$?
echo "echo exit code=$RC"

# timeout の exit code は 124
test $RC -eq 124
```

期待条件:
- `ros2 lifecycle get` が `inactive`（または同等）を示す
- `timeout 5s ...` が 5 秒以内に 1 件も受信せず終了し、alerts が表示されない

## 4. Clean shutdown（Ctrl+C）でスタックトレースが出ない

`alerts_node_kind` が classic/lifecycle のどちらでも、launch 実行中に `Ctrl+C` で停止して次を満たす:

期待条件:
- Python の例外スタックトレースが出ない
- 終了後にノードが残留しない（任意確認）:

```bash
ros2 node list
pgrep -af medical_robot_sim
```
