# Day16 異常検知AI（advisory layer）タスク（tasks）

## Implementation tasks

- [x] `medical_robot_sim/advisory_publisher.py` を追加する
  - [x] subscribe: 相対 `patient_vitals`
  - [x] publish: 相対 `advisories`
  - [x] publish 型は `medical_interfaces/msg/Alert`（kind=advisory）
  - [x] `FlatlineDetector` インスタンスを保持し、患者ごとに状態が隔離される
  - [x] `event=advisory.config` / `event=advisory.publish` を Day11 形式で出す
  - [x] Ctrl+C で clean shutdown

- [x] `setup.py` の `console_scripts` に `advisory_publisher` を追加する

- [x] `medical_robot_sim/advisory_alerts.py` を追加する（AnomalyEvent→Alert 変換）
  - [x] `kind=advisory` / `rule_id=ai.*` の規約を満たす

- [x] `launch/icu_multi_patient.launch.py` に `enable_advisories` を追加する（default `false`）
  - [x] `enable_advisories:=true` のときだけ患者ごとに `advisory_publisher` を起動
  - [x] `advisories_qos_*` を launch arg 化して publish QoS を外部化
  - [x] `advisories_window_sec/window_size` 等を launch arg 化

- [x] （必要なら）`anomaly_detector.py` を node から使いやすいように拡張する
  - [x] window/threshold を外部から注入できる（インスタンスで完結）
  - [x] テストのために module-level state を避ける/明示 reset できる

## Validation tasks

- [x] `enable_advisories:=false` で従来通り起動できる（後方互換）
- [x] `enable_advisories:=true scenario:=flatline` で `/patient_01/advisories` を `--once` で受信できる
- [x] 受信した `Alert.msg` の `kind` が `advisory` である
- [x] ログに `event=advisory.config` と `event=advisory.publish` が残る
- [x] Ctrl+C 停止で `Traceback` / `KeyboardInterrupt` が出ない

## Automated tests

- [x] `src/medical_robot_sim/test/test_day16_advisory_alerts.py` を追加する
  - [x] `anomaly_event_to_advisory_alert()` が `Alert.msg` を生成できる
  - [x] `rule_id` が安定ID（`ai.*`）へ変換される

- [x] `colcon test --packages-select medical_robot_sim` が成功する

## Docs update tasks

- [x] `docs/day16_anomaly_detection_ai.md` に Quickstart と acceptance 導線がある
- [x] `specs/day16_anomaly_detection_ai/spec.md` / `tasks.md` / `acceptance.md` が揃っている
- [x] README へ追記する場合は「最短導線リンクのみ」に留める（詳細は docs/specs に集約）
