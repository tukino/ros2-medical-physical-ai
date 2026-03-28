# Day13 実センサー接続（I2C/SPI） タスク一覧（tasks）

## Implementation tasks

- [ ] hardware_io.py を追加し、`VitalDevice` I/F と `MockVitalDevice` を実装する
- [ ] mock に `mock_scenario`（`normal/spo2_drop/flatline`）を実装する（deterministic）
- [ ] hw_vital_sensor.py を追加し、driver から読み取った値を `medical_interfaces/msg/VitalSigns` に詰めて publish する
- [ ] `hw_vital_sensor` に parameters を追加する
  - [ ] `patient_id`
  - [ ] `publish_rate_hz`
  - [ ] `driver`（mock/i2c/spi）
  - [ ] `mock_scenario`（driver=mock のときのみ）
  - [ ] Day8 QoS: `vitals_qos_depth`, `vitals_qos_reliability`, `vitals_qos_durability`
  - [ ] Day11 Observability: `observability_verbose`
- [ ] Day11 形式のイベントログを追加する（`vitals.hw_device_config`, `vitals.hw_device_open_ok/fail`）
- [ ] icu_hw_sensor.launch.py を追加し、単一患者で `hw_vital_sensor` + `icu_monitor` + `rule_alert_engine` を起動できるようにする
- [ ] src/medical_robot_sim/setup.py の `console_scripts` に `hw_vital_sensor` を追加する

## Validation tasks

- [ ] `pytest`（`colcon test --packages-select medical_robot_sim`）が通る
- [ ] mock driver で `/patient_01/patient_vitals` が publish される（acceptance セクション3）
- [ ] mock の異常シナリオで `/patient_01/alerts` が生成される（acceptance セクション4）
- [ ] Ctrl+C でスタックトレースが出ない（acceptance セクション5）

## Automated tests

- [ ] test_hardware_io_mock.py
  - [ ] `mock_scenario=normal` が範囲内の値を返す
  - [ ] `mock_scenario=spo2_drop` が単調に低下し、最小値で下げ止まる
  - [ ] `mock_scenario=flatline` が一定値を返す
- [ ] test_hw_vital_sensor_params.py
  - [ ] params のバリデーション（publish_rate_hz>0、driver の許容値、mock_scenario の許容値）

## Docs update tasks

- [ ] docs/day13_real_sensor_i2c_spi.md を完成させる（Quickstart / 読み方 / 次Dayとの接続）
- [ ] specs/day13_real_sensor_i2c_spi/acceptance.md を copy-paste 可能な手順として整備する
- [ ] README に追記すべき最小リンク（docs→acceptance）を docs 側に明記する（README自体は変更しない）
