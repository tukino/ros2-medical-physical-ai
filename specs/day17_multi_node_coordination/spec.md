# Day17 マルチノード協調 実装仕様（spec）

## Goal

Day9 で導入した Lifecycle Node（`rule_alert_engine_lifecycle`）を、
別ノード（Coordinator）が **自動で制御**できるようにし、システムとしての協調動作を成立させる。

- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）は維持
- coordinator は既定で無効（後方互換）
- `enable_coordination:=true` のとき、
  vitals の準備完了（ready）に応じて `rule_alert_engine` を `configure -> activate` する
- （任意・既定ON）全患者 NO_DATA で `deactivate` し、データ復帰で再 `activate`
- Day11形式ログ + `ros2 lifecycle` で機械的に観測可能
- pytest による自動テストを最低 1 つ追加

## Design

### Coordination model

- 制御対象: `rule_alert_engine_lifecycle`（node name は `rule_alert_engine`）
- 制御手段: lifecycle service（`lifecycle_msgs/srv/ChangeState`, `lifecycle_msgs/srv/GetState`）

coordinator は次の状態遷移を扱う。

- `configure`（unconfigured -> inactive）
- `activate`（inactive -> active）
- `deactivate`（active -> inactive）

coordinator はサービス未提供/遷移失敗を許容し、ログを残して safe に待機する。

### Node: `icu_coordinator`

- executable: `icu_coordinator`
- node name: `icu_coordinator`（root namespace）
- subscribe:
  - `/patient_XX/patient_vitals`（`medical_interfaces/msg/VitalSigns`）
- service client:
  - `/rule_alert_engine/get_state`
  - `/rule_alert_engine/change_state`

ready 判定:
- 患者ごとに vitals を `min_messages_per_patient` 件受信したら ready
- `ready_timeout_sec` を超過したら ready 失敗として `event=coord.ready_timeout` を出し、以後は安全側（待機）

NO_DATA 判定:
- 患者ごとの最終受信から `no_data_after_sec` 超で NO_DATA
- 全患者 NO_DATA なら `deactivate`（`deactivate_on_no_data=true` の場合）

### Launch integration

[src/medical_robot_sim/launch/icu_multi_patient.launch.py](src/medical_robot_sim/launch/icu_multi_patient.launch.py) に以下を追加する。

- `enable_coordination`（default `false`）
- `coord_min_messages_per_patient`（int）
- `coord_ready_timeout_sec`（float）
- `coord_check_period_sec`（float）
- `coord_stale_after_sec`（float）
- `coord_no_data_after_sec`（float）
- `coord_deactivate_on_no_data`（bool）

起動推奨:
- `alerts_node_kind:=lifecycle lifecycle_autostart:=false enable_coordination:=true`

### Observability

Day11形式ログ（1行=1イベント）。最低限:

- `event=coord.config`
- `event=coord.ready` / `event=coord.ready_timeout`
- `event=coord.lifecycle_set`
- `event=coord.deactivate_on_no_data`

### Testing

pytest（`src/medical_robot_sim/test/`）で pure function をテストする。

- `is_ready(counts, min_messages_per_patient)`
- `all_patients_no_data(last_seen_age, no_data_after_sec)`

（ROS service の統合テストは必須外。必要なら後続 Day で強化する）

## Affected files

- `src/medical_robot_sim/medical_robot_sim/icu_coordinator.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/coordination_policy.py`（新規）
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`
- `src/medical_robot_sim/setup.py`
- `src/medical_robot_sim/test/test_day17_coordination_policy.py`（新規）

- `docs/day17_multi_node_coordination.md`
- `specs/day17_multi_node_coordination/spec.md`
- `specs/day17_multi_node_coordination/tasks.md`
- `specs/day17_multi_node_coordination/acceptance.md`

## Constraints

- 既存 topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）を壊さない
- coordinator は既定無効で後方互換を維持する
- ルールベース alerting を主にする（coordinator は “判断” の置換をしない）
- Ctrl+C 停止でスタックトレースを出さない（Day3方針）
- 協調の判断ロジックは可能な限り pure function 化し、pytest でテスト可能にする

## Non-goals

- `vital_sensor` / `icu_monitor` の lifecycle 化
- 自動復旧の作り込み（再起動/再接続のフル実装）
- 新しい message 定義の導入（本Dayは service + 既存topic観測で成立させる）
- launch_testing 等の重い統合テスト基盤の導入
