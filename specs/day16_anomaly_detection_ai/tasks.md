# Day16 異常検知AI（advisory layer）タスク（tasks）

## Implementation tasks

- [ ] `medical_robot_sim/advisory_publisher.py` を追加する
  - [ ] subscribe: 相対 `patient_vitals`
  - [ ] publish: 相対 `advisories`
  - [ ] publish 型は `medical_interfaces/msg/Alert`（kind=advisory）
  - [ ] `FlatlineDetector` インスタンスを保持し、患者ごとに状態が隔離される
  - [ ] `event=advisory.config` / `event=advisory.publish` を Day11 形式で出す
  - [ ] Ctrl+C で clean shutdown

- [ ] `setup.py` の `console_scripts` に `advisory_publisher` を追加する

- [ ] `launch/icu_multi_patient.launch.py` に `enable_advisories` を追加する（default `false`）
  - [ ] `enable_advisories:=true` のときだけ患者ごとに `advisory_publisher` を起動
  - [ ] `advisories_qos_*` を launch arg 化して publish QoS を外部化
  - [ ] （任意）`advisory_window_sec/window_size` 等を launch arg 化

- [ ] （必要なら）`anomaly_detector.py` を node から使いやすいように拡張する
  - [ ] window/threshold を外部から注入できる（インスタンスで完結）
  - [ ] テストのために module-level state を避ける/明示 reset できる

## Validation tasks

- [ ] `enable_advisories:=false` で従来通り起動できる（後方互換）
- [ ] `enable_advisories:=true scenario:=flatline` で `/patient_01/advisories` を `--once` で受信できる
- [ ] 受信した `Alert.msg` の `kind` が `advisory` である
- [ ] ログに `event=advisory.config` と `event=advisory.publish` が残る
- [ ] Ctrl+C 停止で `Traceback` / `KeyboardInterrupt` が出ない

## Automated tests

- [ ] `src/medical_robot_sim/test/test_anomaly_detector.py` を追加する
  - [ ] window 未満でイベント無し
  - [ ] window 到達で初回のみイベントが出る（edge-trigger）
  - [ ] `spo2_drop_threshold` / `hr_jump_threshold` の境界値

- [ ] `colcon test --packages-select medical_robot_sim` が成功する

## Docs update tasks

- [ ] `docs/day16_anomaly_detection_ai.md` に Quickstart と acceptance 導線がある
- [ ] `specs/day16_anomaly_detection_ai/spec.md` / `tasks.md` / `acceptance.md` が揃っている
- [ ] README へ追記する場合は「最短導線リンクのみ」に留める（詳細は docs/specs に集約）
