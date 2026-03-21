# Day8 QoS設計 受け入れ基準（acceptance）

## 1. ビルドとテストが通る

次がすべて成功すること:

```bash
cd ~/ros2_ws

# ROS 2 Humble の環境を先に読み込む
source /opt/ros/humble/setup.bash

colcon build --symlink-install

# build 後に workspace を読み込む
source ~/ros2_ws/install/setup.bash

colcon test --packages-select medical_robot_sim
colcon test-result --verbose
```

期待条件:
- `colcon test-result --verbose` に failures が 0

## 2. 既定起動で topic が成立する（後方互換）

起動:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true
```

期待条件:
- `/patient_01/patient_vitals` が存在する
- `/patient_01/alerts` が存在する

確認コマンド:

```bash
ros2 topic list | grep -E "/patient_01/(patient_vitals|alerts)$"
```

## 3. `ros2 topic info --verbose` で QoS が観測できる

起動中に別ターミナルで実行:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic info --verbose /patient_01/patient_vitals
ros2 topic info --verbose /patient_01/alerts
```

期待条件（両 topic で満たす）:
- 出力に `Reliability:` が含まれる
- 出力に `Durability:` が含まれる
- 出力に `History (Depth):`（または同等の深さ表示）が含まれる

## 4. launch 引数で QoS を切り替えると、観測結果が変わる

### 4.1 alerts durability を transient_local に切り替え

起動:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  alerts_qos_durability:=transient_local
```

確認:

```bash
ros2 topic info --verbose /patient_01/alerts
```

期待条件:
- 出力に `Durability: TRANSIENT_LOCAL`（または同等の transient local 表示）が含まれる

### 4.2 vitals reliability を best_effort に切り替え

起動:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_qos_reliability:=best_effort
```

確認:

```bash
ros2 topic info --verbose /patient_01/patient_vitals
```

期待条件:
- 出力に `Reliability: BEST_EFFORT`（または同等の best effort 表示）が含まれる

## 5. 不正な QoS 値は明確に失敗する

次のような不正値を渡して起動する:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_qos_reliability:=invalid_value
```

期待条件:
- ノードが起動失敗し、ログに「許容される reliability 値」が分かるエラーが出る
- topic は生成されない
