# Day8: QoS設計（day8_qos）

## Purpose

ROS2 の通信品質（QoS: Quality of Service）を設計し、このリポジトリの主要 topic に対して **暗黙のデフォルト QoS をやめ、明示的・再現可能なQoS設定**を導入する。

対象は最小限として、既存の topic 契約（/patient_XX/patient_vitals, /patient_XX/alerts）を崩さずに、次を達成する:

- ノード間の QoS 不一致による「購読できない/たまに途切れる」を予防する
- `ros2 topic info --verbose` で意図した QoS が観測できる
- launch 引数（= ROS params）で QoS を切り替えられる（後方互換のまま）

## Background

現状の実装では、`create_publisher(..., 10)` / `create_subscription(..., 10)` のように **depth だけを指定**しており、Reliability / Durability / History などは rclpy の暗黙デフォルトに委ねられている。

- 動作はするが、設計意図がコード・docs のどこにも残らない
- QoS 不一致が発生したときの原因特定が難しい
- rosbag 記録や CLI 検証（`ros2 topic echo`）で期待とズレやすい

`docs/PLAN.md` の Phase2（システム設計）で Day8 が「QoS設計」とされているため、このタイミングで QoS を仕様化する。

## Why this day matters in the roadmap

今後の Day9（Lifecycle Node）や Day10（Fault Injection）では、
- ノードが停止/再起動する
- 遅延/ドロップ/帯域制限など通信異常を注入する

といった条件下で「監視・アラートが正しく成立する」ことを扱う。
QoS が暗黙だと、通信異常と QoS 不一致が混ざって再現性が崩れる。

Day8 で QoS を **設計として固定し、かつ切替可能**にしておくことで、以降の Day の検証が安定する。

## Target topics/components

### 対象 topic（契約は変更しない）

- vitals: `/patient_XX/patient_vitals`（型: `medical_interfaces/msg/VitalSigns`）
- alerts: `/patient_XX/alerts`（型: `medical_interfaces/msg/Alert`）

### 対象ノード

- `vital_sensor`（vitals publish）
- `icu_monitor`（vitals subscribe）
- `rule_alert_engine`（vitals subscribe / alerts publish）

※ `medical_monitor` は単体デモ用途のため、Day8 の必須対象からは外す（ただし QoS を揃えるなら同じ方針で追加できる）。

## Design policy

- 後方互換優先: 既定値は「現状と同等の挙動」を維持する
- 小さく増分: QoS の対象は vitals/alerts のみ
- 明示的: `QoSProfile(...)` を生成して `create_*` に渡し、設計意図をコードに残す
- 再現性: launch 引数で切替でき、CLI で観測可能にする
- テスト可能: 文字列→ポリシー変換や QoSProfile 生成を純粋関数化して pytest で検証する

## Recommended presets（使い分けの指針）

Day8 の狙いである「落としていいデータ」と「絶対に落とせないデータ」を、topic ごとに QoS で区別できるようにする。

- vitals（`/<patient>/patient_vitals`）: 最新値を継続監視するデータ
   - 例: `vitals_qos_reliability:=best_effort`（多少の欠落より、滞留しにくさ/軽さを優先したい場合）
   - 既定は後方互換のため `reliable` のまま

- alerts（`/<patient>/alerts`）: 取りこぼしが望ましくないイベント通知
   - 例: `alerts_qos_reliability:=reliable`（既定）
   - 例: `alerts_qos_durability:=transient_local`（late-joiner に直近の alert を渡したい場合）

※ Day8 ではまず reliability/durability/depth の明示と切替を優先し、deadline/lifespan 等の遅延・期限系 QoS は今後の拡張ポイントとする。

## Implementation requirements

1. QoSProfile を純粋関数で生成できるユーティリティを追加する
   - 入力: depth, reliability, durability（必要最小限）
   - 出力: `rclpy.qos.QoSProfile`
   - 入力値が不正な場合は `ValueError` で明確に失敗

2. 各ノードに QoS を切り替える ROS params を追加する
   - `vitals_qos_depth`（int）
   - `vitals_qos_reliability`（string: `reliable`/`best_effort`）
   - `vitals_qos_durability`（string: `volatile`/`transient_local`）
   - `alerts_qos_depth`（int）
   - `alerts_qos_reliability`（string）
   - `alerts_qos_durability`（string）

3. 既定値（後方互換）
   - vitals: `KEEP_LAST depth=10`, reliability=`reliable`, durability=`volatile`
   - alerts: `KEEP_LAST depth=10`, reliability=`reliable`, durability=`volatile`

4. launch に引数を追加して、`icu_multi_patient.launch.py` から各ノードへ params を渡す

5. 自動テスト
   - QoSProfile 生成ユーティリティのユニットテストを追加
   - `colcon test --packages-select medical_robot_sim` で 0 failures

6. Docs/README 更新（最小）
   - Day8 doc への再現手順
   - CLI で QoS を確認・切替するコマンド例

## Files expected to change

- `src/medical_robot_sim/medical_robot_sim/vital_sensor.py`
- `src/medical_robot_sim/medical_robot_sim/icu_monitor.py`
- `src/medical_robot_sim/medical_robot_sim/rule_alert_engine.py`
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`
- `src/medical_robot_sim/medical_robot_sim/qos_profiles.py`（新規）
- `src/medical_robot_sim/test/test_qos_profiles.py`（新規）
- `README.md`（必要なら）
- `docs/day8_qos.md`（本ファイル）

## Reproduction / validation steps

### 1) 既定 QoS の観測（後方互換確認）

```bash
cd ~/ros2_ws

# NOTE:
# - この workspace 直下に `setup.bash` は存在しません。
# - まず ROS 2 Humble の環境を source してから build/test/launch します。
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true
```

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic info --verbose /patient_01/patient_vitals
ros2 topic info --verbose /patient_01/alerts
```

期待:
- どちらも Publisher/Subscription が成立している
- QoS の Reliability/Durability/Depth が意図した既定値で表示される

### 2) QoS パラメータ切替（例）

alerts を transient_local にする例（Late-joiner が最後の alerts を受け取れる設定）:

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  alerts_qos_durability:=transient_local
```

### 3) 自動テスト

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

colcon test --packages-select medical_robot_sim
colcon test-result --verbose
```

## Success criteria

- QoS がコード上で明示され、設計意図が docs と spec に残っている
- launch 引数で vitals/alerts の QoS を切替できる
- `ros2 topic info --verbose` で QoS を観測できる
- `colcon test` が成功し、少なくとも QoSProfile 生成のユニットテストがある
- topic 契約（名前/型）と既存 Day5-7 の再現手順が破壊されない

## Relevance to medical / BCI / Physical AI context

- 医療監視では「最新値を確実に受ける」ことと「遅延を増やさない」ことのバランスが重要
- BCI/EEG のような高頻度センサでは BEST_EFFORT が妥当な場合がある一方、アラートは取りこぼしが許されない
- QoS を設計・切替可能にしておくことで、将来の実センサー接続（Day13）や edge（Day14）で通信条件が変わっても同じ設計枠で評価できる

## Connection to the next day

Day9（Lifecycle Node導入）では、ノードの状態遷移（configure/activate/deactivate）により publisher/subscription が動的に切り替わる。

Day8 で QoS を仕様として確定しておくことで、Day9 では「Lifecycle による停止/再開」と「QoS 不一致」を切り分けて検証できる。
