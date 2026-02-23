# ICU 複数患者 VitalSigns 監視（ROS 2 Humble / colcon workspace）

複数患者（例: `patient_01`, `patient_02`）のバイタルを **namespace で分離**して publish し、集約ノードが患者ごとのトピックを subscribe して一覧表示します。

- `vital_sensor` が相対トピック `patient_vitals` に publish（例: `/patient_01/patient_vitals`）
- `icu_monitor` が `patients` 引数から購読先 `/{pid}/patient_vitals` を決定

Day3 の shutdown / clean exit の学びは [docs/day3_shutdown_clean_exit.md](docs/day3_shutdown_clean_exit.md) にまとめています。

## 前提（ROS 2 Humble）
- ROS 2 Humble がインストール済み
- `colcon` が利用できる

例（Ubuntu 想定）:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

## 依存（rosdep）
この workspace には 2 パッケージがあります。

- `medical_interfaces`（`VitalSigns.msg`）
- `medical_robot_sim`（`vital_sensor`, `icu_monitor`）

依存解決（初回のみ `rosdep update`）:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## ビルド
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 起動
### 環境読み込み
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### マルチ患者起動（デフォルト: `patient_01`〜`patient_05`）
```bash
cd ~/ros2_ws
ros2 launch medical_robot_sim icu_multi_patient.launch.py
```

### launch 引数 `patients:=...` の説明
`patients` は **CSV（カンマ区切り）**で、各要素が患者 namespace になります。

- 例: `patients:=patient_01,patient_02`
  - `vital_sensor` が 2 個起動し、それぞれ `/patient_01/patient_vitals`, `/patient_02/patient_vitals` に publish
  - `icu_monitor` は 1 個だけ起動し、両方を subscribe

患者を 2 人に絞る例:
```bash
cd ~/ros2_ws
ros2 launch medical_robot_sim icu_multi_patient.launch.py patients:=patient_01,patient_02
```

引数一覧の確認:
```bash
cd ~/ros2_ws
ros2 launch medical_robot_sim icu_multi_patient.launch.py --show-args
```

## 停止
通常は `Ctrl+C` で停止します。

停止まわりの学び（shutdown / clean exit）: [docs/day3_shutdown_clean_exit.md](docs/day3_shutdown_clean_exit.md)

もし特定患者の `vital_sensor` だけ止めたい場合（Fault injection / 動作確認用）:
```bash
# __ns:=/patient_02 が目印
pgrep -af vital_sensor | grep "__ns:=/patient_02"

# 先頭の数値(PID)を kill
kill <PID>
```

## 確認コマンド（topic echo / hz）
（別ターミナルの場合は、先に環境を読み込みます）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### トピック一覧
```bash
ros2 topic list | grep patient_vitals
```

### 1 件だけ表示
```bash
ros2 topic echo /patient_01/patient_vitals --once
```

### publish レート確認
```bash
ros2 topic hz /patient_01/patient_vitals
```

（任意）接続関係確認:
```bash
sudo apt update
sudo apt install -y ros-humble-rqt-graph
rqt_graph
```

## 想定出力例（1段）
`icu_monitor` は 1 秒ごとにダッシュボード表示を更新します（launch 経由だとクリア上書きできず、ブロック表示になる場合があります）。

```text
ICU DASHBOARD  (patients=2)
----------------------------------------------------------------------------------------------------
pid        | alert   |  HR | SpO2 | BP      | Temp | age
----------------------------------------------------------------------------------------------------
patient_01 | OK      |  72 |   96 | 129/83  | 37.0 |  0s
patient_02 | OK      |  73 |   96 | 118/74  | 37.3 |  0s
----------------------------------------------------------------------------------------------------
```
