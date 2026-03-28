# Day13: 実センサー接続（I2C/SPI）（day13_real_sensor_i2c_spi）

## Purpose

本Dayの目的は、これまでの「仮想バイタル生成（`vital_sensor`）」に加えて、**実センサー由来の入力をROS2 topicへ流し込める最小構成**を設計・実装できる状態にすること。

- 実機入力（I2C/SPI）を **既存のtopic契約**（`/patient_XX/patient_vitals`）へ変換して publish できる
- 実センサーが無い環境でも、**mockドライバ**で同等のパイプラインを検証できる
- Day10（Fault Injection）/ Day11（Logging）/ Day12（rosbag）で整えた「再現性・観測可能性」を、実機入力にも拡張できる

## Background

医療×Physical AI の入力系では、最終的に「現実世界の信号」を扱う必要がある。
一方で、いきなり実機依存の実装に寄せると次が起きやすい:

- 開発環境差（デバイス有無、権限、ドライバ）が原因で再現性が落ちる
- 監視/アラートの上流が変わり、既存の検証（Day10-12）が壊れる

Day13では、**実機依存を隔離しつつ、ROS2上の契約を維持**する設計にする。

## Why this day matters in the roadmap

- Day12（rosbag）: 実機入力を bag に落とし、オフライン再生で切り分けできる
- Day14（Jetson）: 実機入力 + Edge実行に向けて、依存/権限/デバイスパスの扱いを整理する必要がある
- Day16（異常検知AI）: 実機のノイズや欠測が混ざる前提で、同じtopic契約のまま評価を進めたい

## Target topics/components

### 対象 topic（契約は変更しない）

- vitals: `/patient_XX/patient_vitals`（`medical_interfaces/msg/VitalSigns`）
- alerts: `/patient_XX/alerts`（`medical_interfaces/msg/Alert`）

本Dayは「入力ソースの追加」を扱うため、topic名・message定義の変更は行わない。

### 対象コンポーネント

- 新規: **実センサー入力ノード**（例: `hw_vital_sensor`）
- 既存: `icu_monitor`（監視）、`rule_alert_engine`（ルール評価）、[src/medical_robot_sim/medical_robot_sim/qos_profiles.py](src/medical_robot_sim/medical_robot_sim/qos_profiles.py)（Day8 QoS）
- 既存（残す）: `vital_sensor`（仮想入力。後方互換として維持）

## Design policy

- **後方互換**: 既存の [src/medical_robot_sim/launch/icu_multi_patient.launch.py](src/medical_robot_sim/launch/icu_multi_patient.launch.py)（仮想センサー）はそのまま動く
- **小さく増分**: Day13で必須にするのは「実入力ノード（＋mock）で既存パイプラインに流せる」こと
- **実機依存を隔離**: I2C/SPIの具体実装は driver 層に閉じ込め、ROS Node 側は薄くする
- **観測可能**: Day11のイベントログ形式（`event=... key=value ...`）で、device open/read の成否を追える
- **クリーン停止維持**: Ctrl+Cでスタックトレースを出さない（Day3方針の維持）

## Implementation requirements

### 1) 入力ノード（`hw_vital_sensor`）の最小要件

- `medical_interfaces/msg/VitalSigns` を publish できる
- 相対topic `patient_vitals` に publish し、namespaceで `/patient_XX/patient_vitals` に解決される（既存と同一）
- 既存 `vital_sensor` と同等に次を parameter で外部化する
  - `patient_id`（例: `patient_01`）
  - `publish_rate_hz`
  - Day8 QoS: `vitals_qos_depth`, `vitals_qos_reliability`, `vitals_qos_durability`

### 2) driver 抽象（I2C/SPI / mock）

実センサー対応は環境依存が大きいので、**必須の受け入れは mock ドライバで成立**させる。

- `driver` parameter（例: `mock` / `i2c` / `spi`）で切替
- `mock` は追加依存なし（pure python）で動作
- `i2c` / `spi` は optional（未導入なら N/A として受け入れに影響しない）

設計の最小形（例）:

- hardware_io.py（新規）
  - `class VitalDevice`: `read_sample() -> dict` などの最小I/F
  - `MockVitalDevice`: deterministic に値を返す（受け入れの再現性のため）
  - `I2CVitalDevice` / `SPIVitalDevice`: optional 実装（例外/権限/デバイス不在を明確に）

### 3) mock の「異常シナリオ」

実機が無い環境でも alerts 生成まで確認できるように、mock には **異常を確実に作れるモード**を持たせる。

- `mock_scenario`（例: `normal` / `spo2_drop` / `flatline`）
- `spo2_drop` は Day6/Day5 のルールが発火しやすい値を作る（例: SpO2を段階的に低下）

### 4) Observability（Day11）

少なくとも次のイベントを 1行ログで出し、grep で追えるようにする。

- `vitals.hw_device_config`（起動時）: driver種別、パラメータ（bus/addressなど）、publish_rate_hz
- `vitals.hw_device_open_ok` / `vitals.hw_device_open_fail`
- （任意/verbose）`vitals.hw_read_ok` / `vitals.hw_read_fail`

### 5) Launch（既存を壊さない）

- [src/medical_robot_sim/launch/icu_multi_patient.launch.py](src/medical_robot_sim/launch/icu_multi_patient.launch.py) は変更最小（または無変更）で後方互換を維持
- 新規に icu_hw_sensor.launch.py を追加し、単一患者（まず `patient_01`）で
  - `hw_vital_sensor`（namespace=patient_01）
  - `icu_monitor`
  - `rule_alert_engine`（任意）
  を起動できる

### 6) README方針への反映（設計時の前提）

READMEには詳細を重複させず、次のリンク導線だけを追記候補とする:

- [docs/day13_real_sensor_i2c_spi.md](docs/day13_real_sensor_i2c_spi.md)
- [specs/day13_real_sensor_i2c_spi/acceptance.md](specs/day13_real_sensor_i2c_spi/acceptance.md)

## Files expected to change

- src/medical_robot_sim/medical_robot_sim/hardware_io.py（新規: driver 抽象 + mock + optional i2c/spi）
- src/medical_robot_sim/medical_robot_sim/hw_vital_sensor.py（新規: ROS2 node）
- src/medical_robot_sim/launch/icu_hw_sensor.launch.py（新規: hw sensor 用パイプライン）
- [src/medical_robot_sim/setup.py](src/medical_robot_sim/setup.py)（entry_points 追加）
- src/medical_robot_sim/test/test_hardware_io_mock.py（新規: mockの再現性/境界値）
- src/medical_robot_sim/test/test_hw_vital_sensor_params.py（新規: param validation の純粋関数テスト）
- [docs/day13_real_sensor_i2c_spi.md](docs/day13_real_sensor_i2c_spi.md)（本ファイル）
- [specs/day13_real_sensor_i2c_spi/spec.md](specs/day13_real_sensor_i2c_spi/spec.md)
- [specs/day13_real_sensor_i2c_spi/tasks.md](specs/day13_real_sensor_i2c_spi/tasks.md)
- [specs/day13_real_sensor_i2c_spi/acceptance.md](specs/day13_real_sensor_i2c_spi/acceptance.md)

## Reproduction / validation steps

- 受け入れ手順（コピペ）: [specs/day13_real_sensor_i2c_spi/acceptance.md](specs/day13_real_sensor_i2c_spi/acceptance.md)

### 最短コピペ（Quickstart: 5分で観測）

このQuickstartは、**実機が無い環境でも mock で動作確認できる**ことを最優先にする。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

rm -f /tmp/day13_quick.log
timeout -s INT 10s ros2 launch medical_robot_sim icu_hw_sensor.launch.py \
  patient:=patient_01 \
  driver:=mock \
  mock_scenario:=spo2_drop \
  enable_alerts:=true \
  > /tmp/day13_quick.log 2>&1 || true

# vitals/alerts のイベント痕跡（どれかが出る）
grep -n "event=vitals\\." /tmp/day13_quick.log || true

# 期待: デバイス設定と open 成功ログがある
grep -n "event=vitals\\.hw_device_config" /tmp/day13_quick.log

echo "OK: day13 quickstart ran"
```

詳細な合否判定は acceptance に従う（topic echo での観測が必須）。

## Success criteria

- mock driver で `/patient_01/patient_vitals` が publish される
- mock の異常シナリオで `/patient_01/alerts` が生成される（ルールエンジン起動時）
- 実機がある場合、I2C/SPI driver でも同一topic契約で publish できる（optional）
- Ctrl+C / SIGINT でスタックトレース無しに停止できる

## Learning path

### Quickstart（最短で動かす）

- まずは本ページの Quickstart で mock 起動し、ログで device open 成否を掴む
- 次に acceptance で topic echo による観測（vitals/alerts）を機械判定する

### 読み方（原因追跡の順番）

1. `/tmp/day13_*.log` の `event=vitals.hw_device_*` を grep
   - device 設定（driver/bus/address）と open 成否
2. `ros2 topic list | grep patient_vitals`
   - publish されている topic が期待どおりか
3. `ros2 topic echo /patient_01/patient_vitals --once`
   - 1件受信できるか（QoS不一致/未接続の切り分け）
4. alerts が出ない場合
   - `rule_alert_engine` 起動の有無、ルール（Day7 YAML）と enabled_rule_ids を確認

### 必須（受け入れ）と任意（学習）

- 必須: mock で vitals publish と alerts 生成が観測できる
- 任意: 実機接続（I2C/SPI）で同一パイプラインを動かす

## Relevance to medical / BCI / Physical AI context

- 医療監視の入力は、実機由来の欠測/ノイズ/遅延が本質的な難しさ
- BCI/EEG 同様、入力系を「契約（topic/message）」に落とし込むことで下流の評価を安定化できる
- Physical AI では入力→判断の遅延が挙動に直結するため、driver層の観測（open/read/latency）が重要

## Connection to the next day

- Day14（Jetson）: 実機入力ノードをEdgeで動かすための依存（I2C/SPI権限、udev、パフォーマンス）へ接続する
- Day12（rosbag）: 実機入力を bag 化し、オフライン再生でルール/監視の検証を繰り返す
