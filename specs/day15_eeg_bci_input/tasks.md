# Day15 EEG / BCI入力 タスク（tasks）

## Implementation tasks

- [x] `medical_interfaces/msg/BCIFeatures.msg` を追加する（features 最小セット）
- [x] `medical_interfaces/CMakeLists.txt` の `rosidl_generate_interfaces()` に `BCIFeatures.msg` を追加する

- [x] `medical_robot_sim/bci_io.py` を追加する
  - [x] `BCIDevice` I/F（open/read/close）
  - [x] `MockBCIDevice`（seed で deterministic）
  - [x] （任意）`SerialBCIDevice` の雛形（依存未導入なら N/A で良い）

- [x] `medical_robot_sim/bci_sensor.py` を追加する
  - [x] publish topic は相対名 `patient_bci`
  - [x] params: `patient_id`, `publish_rate_hz`, `driver`, `mock_scenario`, `seed`, `bci_qos_*`
  - [x] （任意/driver=serial）params: `serial_port`
  - [x] `event=bci.device_config` と open 成否イベントを出す
  - [x] Ctrl+C で clean shutdown

- [x] `medical_robot_sim/bci_monitor.py` を追加する
  - [x] params: `patients`, `bci_topic`, `bci_qos_*`
  - [x] 患者ごとに last_seen と state を保持し、状態遷移のみ `event=bci.patient_state` を出す
  - [x] 状態分類は pure function に切り出す

- [x] `launch/icu_bci.launch.py` を追加する
  - [x] `patient`（単一）、`driver`、`mock_scenario`、`publish_rate_hz`、`seed`、`bci_qos_*` を launch arg 化
  - [x] （任意/driver=serial）`serial_port` を launch arg 化
  - [x] `bci_sensor`（namespace=patient）と `bci_monitor`（root）を起動する

- [x] `setup.py` の `console_scripts` に `bci_sensor` と `bci_monitor` を追加する

## Validation tasks

- [x] `pytest`（`src/medical_robot_sim/test/`）を追加する
  - [x] `test_bci_io_mock.py`: seed 固定で deterministic になる
  - [x] `test_bci_patient_state.py`: FRESH/STALE/NO DATA の境界値

- [x] `colcon build --symlink-install` が通る
- [x] `colcon test --packages-select medical_robot_sim` が通る

## Acceptance tasks

- [x] `specs/day15_eeg_bci_input/acceptance.md` の手順で以下が観測できる
  - [x] `ros2 interface show medical_interfaces/msg/BCIFeatures` が成功する
  - [x] `/patient_01/patient_bci` を `--once` で 1 件受信できる（mock）
  - [x] `event=bci.device_config` と `event=bci.patient_state` がログに残る
  - [x] Ctrl+C 停止で `Traceback` / `KeyboardInterrupt` が出ない

## Docs update tasks

- [x] `docs/day15_eeg_bci_input.md` に Quickstart と acceptance 導線がある
- [x] `specs/day15_eeg_bci_input/spec.md` / `tasks.md` / `acceptance.md` が揃っている
- [x] README へ追記する場合は「最短導線リンクのみ」に留める（詳細は docs/specs に集約）
