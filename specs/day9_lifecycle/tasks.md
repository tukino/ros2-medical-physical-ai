# Day9 Lifecycle Node導入 タスク（tasks）

## Implementation tasks

- [x] [src/medical_robot_sim/medical_robot_sim/rule_alert_engine_lifecycle.py](src/medical_robot_sim/medical_robot_sim/rule_alert_engine_lifecycle.py) を新規追加し、LifecycleNode で `rule_alert_engine` を実装する
- [x] `lifecycle_autostart` parameter を追加し、true なら自動で `configure -> activate` する
- [x] `on_configure/on_activate/on_deactivate` で pub/sub の作成・破棄（または処理ゲート）を実装し、active でのみ alerts が出るようにする
- [x] 既存 `rule_alert_engine` の parameter 群（Day7/Day8/Day6）を Lifecycle 版でも受け取れるようにする
- [x] [src/medical_robot_sim/setup.py](src/medical_robot_sim/setup.py) に `rule_alert_engine_lifecycle` の console_scripts を追加する
- [x] [src/medical_robot_sim/launch/icu_multi_patient.launch.py](src/medical_robot_sim/launch/icu_multi_patient.launch.py) に `alerts_node_kind` と `lifecycle_autostart` を追加し、classic/lifecycle を切替できるようにする

## Validation tasks

- [x] `alerts_node_kind:=classic`（既定）で従来通り alerts が流れることを確認する
- [x] `alerts_node_kind:=lifecycle lifecycle_autostart:=false` で起動し、`ros2 lifecycle get/set` で状態遷移できることを確認する
- [x] inactive/deactivated 状態で `ros2 topic echo /patient_01/alerts --once` が受信できないこと（timeout で終了すること）を確認する
- [x] active 状態で手動 publish（SpO2=85）により alerts を 1 件受信できることを確認する
- [x] `Ctrl+C` で停止してスタックトレースが出ないことを確認する

## Automated tests

- [x] [src/medical_robot_sim/test/test_rule_alert_engine_lifecycle.py](src/medical_robot_sim/test/test_rule_alert_engine_lifecycle.py) を新規追加する
- [x] `lifecycle_autostart=false` で未 activate のときに publish しないことをテストする（内部フラグでも可）
- [x] `configure -> activate -> deactivate` 相当の遷移が例外なく通ることをテストする
- [x] `colcon test --packages-select medical_robot_sim` が成功する

## Docs update tasks

- [x] [docs/day9_lifecycle.md](docs/day9_lifecycle.md) に再現手順（classic/lifecycle 両方）を載せる
- [x] [specs/day9_lifecycle/acceptance.md](specs/day9_lifecycle/acceptance.md) を観測可能なコマンド中心で整備する
