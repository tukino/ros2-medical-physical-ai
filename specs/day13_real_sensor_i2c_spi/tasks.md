# Day13 実センサー接続（I2C/SPI） タスク一覧（tasks）

## Implementation tasks

- [x] hardware_io.py を追加し、`VitalDevice` I/F と `MockVitalDevice` を実装する
- [x] mock に `mock_scenario`（`normal/spo2_drop/flatline`）を実装する（deterministic）
- [x] hw_vital_sensor.py を追加し、driver から読み取った値を `medical_interfaces/msg/VitalSigns` に詰めて publish する
- [x] `hw_vital_sensor` に parameters を追加する
  - [x] `patient_id`
  - [x] `publish_rate_hz`
  - [x] `driver`（mock/i2c/spi）
  - [x] `mock_scenario`（driver=mock のときのみ）
  - [x] Day8 QoS: `vitals_qos_depth`, `vitals_qos_reliability`, `vitals_qos_durability`
  - [x] Day11 Observability: `observability_verbose`
- [x] Day11 形式のイベントログを追加する（`vitals.hw_device_config`, `vitals.hw_device_open_ok/fail`）
- [x] icu_hw_sensor.launch.py を追加し、単一患者で `hw_vital_sensor` + `icu_monitor` + `rule_alert_engine` を起動できるようにする
- [x] src/medical_robot_sim/setup.py の `console_scripts` に `hw_vital_sensor` を追加する

## Validation tasks

- [x] `pytest`（`colcon test --packages-select medical_robot_sim`）が通る
- [x] mock driver で `/patient_01/patient_vitals` が publish される（acceptance セクション3）
- [x] mock の異常シナリオで `/patient_01/alerts` が生成される（acceptance セクション4）
- [x] Ctrl+C でスタックトレースが出ない（acceptance セクション5）

## Automated tests

- [x] test_hardware_io_mock.py
  - [x] `mock_scenario=normal` が範囲内の値を返す
  - [x] `mock_scenario=spo2_drop` が単調に低下し、最小値で下げ止まる
  - [x] `mock_scenario=flatline` が一定値を返す
- [x] test_hw_vital_sensor_params.py
  - [x] params のバリデーション（publish_rate_hz>0、driver の許容値、mock_scenario の許容値）

## Docs update tasks

- [x] docs/day13_real_sensor_i2c_spi.md を完成させる（Quickstart / 読み方 / 次Dayとの接続）
- [x] specs/day13_real_sensor_i2c_spi/acceptance.md を copy-paste 可能な手順として整備する
- [x] README に Day13 の最小リンク（docs→acceptance）を追記する
