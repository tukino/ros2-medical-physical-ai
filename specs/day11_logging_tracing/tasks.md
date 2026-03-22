# Day11 Logging / Tracing タスク（tasks）

## Implementation tasks

- [x] `medical_robot_sim/observability.py` を新規追加し、`format_event` / `sanitize_value` を実装する（rclpy 非依存）
- [x] `vital_sensor` に Day11 params（`observability_verbose` など）を declare し、起動時に `vitals.fault_config` を出す
- [x] `vital_sensor` に `vitals.pause_enter/exit` と `vitals.stop_trigger` のイベントログを追加する
- [x] （verbose時のみ）`vitals.drop` / `vitals.enqueue_delayed` をイベントログ化する（ログ量を抑制）
- [x] `icu_monitor` に患者ごとの状態遷移イベント `monitor.patient_state` を追加する（FRESH/STALE/NO_DATA）
- [x] `rule_alert_engine`（classic/lifecycle）に `alerts.emit` を追加する（rule_id/priority を含む）

## Automated tests

- [x] `src/medical_robot_sim/test/test_observability.py` を追加し、`format_event` の安定性（key順/改行無し）をテストする
- [x] `colcon test --packages-select medical_robot_sim` が成功する

## Validation tasks（手動）

- [x] baseline 起動で `vitals.fault_config` がログに出る
- [x] pause 注入で `vitals.pause_enter` と `vitals.pause_exit` がログに出る
- [x] stop 注入で `vitals.stop_trigger` がログに出る
- [x] monitor 側で `monitor.patient_state` が出力され、pause/stop の前後で state が遷移する
- [x] alerts 有効時に `alerts.emit` がログに出る
- [x] どの設定でも Ctrl+C でスタックトレースが出ない

## Docs update tasks

- [x] `docs/day11_logging_tracing.md` を更新し、ログイベント一覧と確認コマンドを載せる
- [x] `specs/day11_logging_tracing/acceptance.md` のコピペ手順を最新化する
