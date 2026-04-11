# Day18 リアルタイム制御（閉ループ）実装仕様（spec）

## Goal

既存の multi-patient 監視系に、
**ルールベース判定による閉ループ制御（vitals/alerts -> control_actions）** を追加する。

達成条件:

- 既存 topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）は維持
- `enable_closed_loop:=true` のときのみ制御ノードが動作
- 制御判断が `event=control.*` ログと `/patient_XX/control_actions` で観測できる
- pure function の policy テストが追加される

## Design

### 1) Control policy（pure function）

`closed_loop_policy.py` に、ROS依存のない判定関数を実装する。

想定インタフェース（例）:

- 入力: `hr`, `spo2`, `latest_alert_priority`, `latest_alert_rule_id`, `age_sec`, `last_action`, `last_publish_age_sec`, 設定閾値
- 出力: `action`, `reason`, `severity`, `should_publish`

最小 action セット:

- `HOLD`: 介入なし（安全側/通常時）
- `OXYGEN_BOOST`: 低SpO2時の軽介入
- `CALL_STAFF`: 重症/RED時の強介入

判定優先順:

1. NO_DATA 安全ガード（`age_sec > no_data_after_sec`）
2. 重症判定（`spo2 <= critical_spo2` または latest RED alert）
3. 警戒判定（`spo2 <= low_spo2` または `outlier.spo2` 相当）
4. 通常時 HOLD

抑制条件:

- cooldown 未経過なら `should_publish=false`
- action が前回と同じ場合は重複 publish を抑制（ログは decision/suppressed で記録）

### 2) ROS node: `closed_loop_controller`

- namespace: 患者 namespace（`/patient_XX`）
- subscribe:
  - 相対 `patient_vitals` (`medical_interfaces/msg/VitalSigns`)
  - 相対 `alerts` (`medical_interfaces/msg/Alert`)
  - 任意で相対 `advisories` (`medical_interfaces/msg/Alert`)
- publish:
  - 相対 `control_actions` (`medical_interfaces/msg/Alert` 再利用)

`control_actions` payload 方針:

- `kind=control_action`
- `rule_id=control.<action_name>`
- `priority` は action に対応して固定
- 判定に使った主因（field/value/score）を埋める

### 3) Launch integration

`icu_multi_patient.launch.py` に以下を追加。

- `enable_closed_loop`（bool, default `false`）
- `control_topic`（string, default `control_actions`）
- `control_cooldown_sec`（float, default `5.0`）
- `control_no_data_after_sec`（float, default `10.0`）
- `control_low_spo2`（float, default `92.0`）
- `control_critical_spo2`（float, default `88.0`）
- `enable_control_from_advisories`（bool, default `false`）

`enable_closed_loop=true` のとき、患者ごとに `closed_loop_controller` を起動する。

### 4) Observability

Day11形式ログ（1行 key=value）で最低限を出力:

- `event=control.config`
- `event=control.decision`
- `event=control.publish`
- `event=control.suppressed`
- `event=control.safe_hold`

## Affected files

- `src/medical_robot_sim/medical_robot_sim/closed_loop_policy.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/closed_loop_controller.py`（新規）
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`
- `src/medical_robot_sim/setup.py`
- `src/medical_robot_sim/test/test_day18_closed_loop_policy.py`（新規）

- `docs/day18_realtime_closed_loop.md`
- `specs/day18_realtime_closed_loop/spec.md`
- `specs/day18_realtime_closed_loop/tasks.md`
- `specs/day18_realtime_closed_loop/acceptance.md`

## Constraints

- topic 契約（vitals/alerts）を壊さない
- 閉ループは既定で無効（後方互換）
- ルールベース主導を維持（LLM判断を導入しない）
- 判定ロジックは pure function でテスト可能にする
- Ctrl+C で clean shutdown（stack trace無し）
- 患者ごとの状態（last_seen, cooldown, last_action）を分離する

## Non-goals

- 高度な制御理論（PID/MPC）導入
- 医療機器としての治療制御保証
- 新しい外部クラウドAPI連携
- 大規模UI追加
- 既存ルールエンジンの全面置換
