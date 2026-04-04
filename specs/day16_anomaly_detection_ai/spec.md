# Day16 異常検知AI（advisory layer）実装仕様（spec）

## Goal

異常検知（AI/統計的検知）を **助言（advisory）として追加**し、次を満たす。

- 既存契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）は変更しない
- 異常検知の出力を `/patient_XX/advisories` として publish できる
- rule-based alerts を置き換えない（AIは意思決定しない）
- 再現性（シナリオ/固定パラメータ）と観測可能性（ログ/echo）がある
- pytest により検知ロジックが自動テストされる

## Design

### Topic / message

- input: `/patient_XX/patient_vitals`（`medical_interfaces/msg/VitalSigns`）
- output: `/patient_XX/advisories`（`medical_interfaces/msg/Alert`）

`Alert.msg` を advisory に転用する際の規約:
- `kind`: `advisory`
- `rule_id`: `ai.*` の安定ID（例: `ai.flatline_hr`, `ai.spo2_drop`, `ai.hr_jump`）
- `priority`: `INFO|YELLOW|RED` 等（最小マップでよい）
- 未設定値は `Alert.msg` の方針通り、空文字/NaN で表現する

### Node

新規ノード `advisory_publisher` を追加する。

- namespace=患者（例: `patient_01`）で起動
- subscribe（相対）: `patient_vitals`
- publish（相対）: `advisories`

検知は node 内で `FlatlineDetector` インスタンスを保持して行う。

### Parameters

最低限:
- `patient_id`（string）
- `window_sec`（int）
- `window_size`（int）
- `spo2_drop_threshold`（float）
- `hr_jump_threshold`（float）
- （任意）`field_epsilon`（flatline 用）

QoS（Day8 流儀で文字列指定）:
- subscribe: `vitals_qos_depth`, `vitals_qos_reliability`, `vitals_qos_durability`
- publish: `advisories_qos_depth`, `advisories_qos_reliability`, `advisories_qos_durability`

### Observability

Day11 形式の 1 行ログを必須にする。

- `event=advisory.config`（起動時: window/threshold/QoS）
- `event=advisory.publish`（publish 時: pid/rule_id/priority/type/score）

### Launch integration

`launch/icu_multi_patient.launch.py` に次を追加する。

- `enable_advisories`（default `false`）
- `advisories_qos_*`
- （任意）`advisory_window_sec`, `advisory_window_size` 等

`enable_advisories:=true` の場合のみ、患者ごとに `advisory_publisher` を起動する。

### Testing

pytest（`src/medical_robot_sim/test/`）で `anomaly_detector` のユニットテストを追加する。

- window 未満はイベント無し
- window 到達で edge-trigger（初回のみ）
- しきい値境界（drop/jump）

## Affected files

- `src/medical_robot_sim/medical_robot_sim/anomaly_detector.py`
- `src/medical_robot_sim/medical_robot_sim/advisory_publisher.py`（新規）
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`
- `src/medical_robot_sim/setup.py`
- `src/medical_robot_sim/test/test_anomaly_detector.py`（新規）

- `docs/day16_anomaly_detection_ai.md`
- `specs/day16_anomaly_detection_ai/spec.md`
- `specs/day16_anomaly_detection_ai/tasks.md`
- `specs/day16_anomaly_detection_ai/acceptance.md`

## Constraints

- rule-based ロジックを主にする（AIは助言。`/alerts` を置換しない）
- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）を壊さない
- 状態（窓バッファ）は患者ごとに隔離
- Ctrl+C による clean shutdown を維持（スタックトレース無し）
- 外部依存（重いMLフレームワーク）を必須にしない

## Non-goals

- LLM による最終判断（本リポジトリの方針外）
- `/patient_XX/alerts` の生成方式の置換
- 学習（training）パイプラインの導入（データ収集/学習は後続で別途）
- launch_testing 等の重い統合テスト基盤の導入
