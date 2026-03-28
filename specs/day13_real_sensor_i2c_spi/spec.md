# Day13 実センサー接続（I2C/SPI） 実装仕様（spec）

## Goal

- 実センサー（I2C/SPI）または mock から値を取得し、既存契約の `/patient_XX/patient_vitals` に publish できる
- 実機が無い環境でも **mock driver で受け入れが完走**する（CI で再現できる）
- publish された vitals を `icu_monitor` / `rule_alert_engine` がそのまま利用できる（下流変更を最小化）

## Design

### 概要

新しい入力ノード `hw_vital_sensor` を追加し、`vital_sensor`（仮想生成）とは並存させる。

- `vital_sensor`: 既存の仮想入力。後方互換のため維持
- `hw_vital_sensor`: 実センサー入力。mock/i2c/spi を切替

### topic と namespace

- `hw_vital_sensor` は相対topic `patient_vitals` に publish
- launch で namespace を患者ID（例: `patient_01`）にし、topic は `/patient_01/patient_vitals` へ解決
- message 型は `medical_interfaces/msg/VitalSigns` を維持

### driver 分離（環境差を吸収）

I2C/SPI の環境依存を ROS Node から分離するため、driver 抽象を作る。

- hardware_io.py
  - `VitalDevice`（I/F）: `open()` / `read_sample()` / `close()` 程度
  - `MockVitalDevice`: deterministic な系列を返す（受け入れの再現性）
  - `I2CVitalDevice` / `SPIVitalDevice`: optional 実装

`hw_vital_sensor` は driver を生成し、取得した値を `VitalSigns` に詰めて publish する。

### mock シナリオ

mock で alerts 生成まで確認できるよう、異常パターンを用意する。

- `mock_scenario=normal`: ほぼ正常値
- `mock_scenario=spo2_drop`: SpO2 を段階的に低下（`roc.spo2_drop` 等を誘発しやすい）
- `mock_scenario=flatline`: HR/SpO2 を固定（flatline ルールを誘発しやすい）

### QoS（Day8）

`hw_vital_sensor` も `vital_sensor` と同様に QoS を parameter で制御し、`build_qos_profile()` を再利用する。

### Observability（Day11）

最低限、次の 1行イベントログを出す。

- `vitals.hw_device_config`
- `vitals.hw_device_open_ok` / `vitals.hw_device_open_fail`

## Affected files

- src/medical_robot_sim/medical_robot_sim/hardware_io.py（新規）
- src/medical_robot_sim/medical_robot_sim/hw_vital_sensor.py（新規）
- src/medical_robot_sim/launch/icu_hw_sensor.launch.py（新規）
- src/medical_robot_sim/setup.py（entry point 追加）
- src/medical_robot_sim/test/test_hardware_io_mock.py（新規）
- src/medical_robot_sim/test/test_hw_vital_sensor_params.py（新規）
- docs/day13_real_sensor_i2c_spi.md
- specs/day13_real_sensor_i2c_spi/spec.md
- specs/day13_real_sensor_i2c_spi/tasks.md
- specs/day13_real_sensor_i2c_spi/acceptance.md

## Constraints

- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）を変更しない
- message 定義（`medical_interfaces`）を変更しない
- 既存 launch / node の後方互換を維持する（icu_multi_patient.launch.py が従来どおり動く）
- Ctrl+C clean shutdown を維持する
- 実機依存（i2c/spi）の導入は optional とし、mock で受け入れ完走できることを必須にする

## Non-goals

- 特定センサー（例: MAX3010x等）の完全な信号処理（PPG→HR/SpO2推定）
- 自動デバイス検出やホットプラグ対応
- 複数実センサーの同時運用（まずは単一患者/単一デバイス）
- 医療機器レベルの安全規格・監査ログ
