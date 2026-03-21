# Day8 QoS設計 実装仕様（spec）

## Goal

`/patient_XX/patient_vitals` と `/patient_XX/alerts` の QoS を明示化し、launch 引数（ROS params）で切替できるようにする。

- 既定値は現状と同等（後方互換）
- QoS の設計意図をコード・docs に残す
- QoS の不一致を減らし、検証/再現性を上げる

## Design

### 対象 topic

- vitals: `/patient_XX/patient_vitals`
- alerts: `/patient_XX/alerts`

topic 名・型は変更しない。

### QoS の設計（既定値）

後方互換のため、既定値は rclpy の「depth 指定だけ」の挙動に揃える。

- History: KEEP_LAST
- Depth: 10
- Reliability: RELIABLE
- Durability: VOLATILE

### QoS の切替（ROS params）

各ノードで同一のパラメータ名を使い、launch から一括で渡せるようにする。

- `vitals_qos_depth` (int, default 10)
- `vitals_qos_reliability` (string, default `reliable`)
- `vitals_qos_durability` (string, default `volatile`)

- `alerts_qos_depth` (int, default 10)
- `alerts_qos_reliability` (string, default `reliable`)
- `alerts_qos_durability` (string, default `volatile`)

許容値:
- reliability: `reliable` / `best_effort`（大文字小文字は不問として正規化）
- durability: `volatile` / `transient_local`

不正値は `ValueError` で明確に失敗させる（曖昧な黙殺をしない）。

### 実装方針

1. QoSProfile 生成を純粋関数化

- 新規ファイル `medical_robot_sim/qos_profiles.py` を追加し、以下を提供する:
  - `build_qos_profile(depth: int, reliability: str, durability: str) -> QoSProfile`
  - `normalize_*` のような補助（必要なら）

2. ノード側で QoSProfile を組み立てて `create_publisher` / `create_subscription` に渡す

- `vital_sensor`:
  - vitals publish に `vitals_qos_*` を適用

- `icu_monitor`:
  - vitals subscribe に `vitals_qos_*` を適用

- `rule_alert_engine`:
  - vitals subscribe に `vitals_qos_*` を適用
  - alerts publish に `alerts_qos_*` を適用

3. launch 引数を追加し、全ノードへ同じ値を渡す

- `icu_multi_patient.launch.py` に launch arg を追加
- `vital_sensor` / `icu_monitor` / `rule_alert_engine` の parameters に伝播

### テスト方針

- `test_qos_profiles.py` を追加し、少なくとも以下を検証する:
  - 既定値相当の入力で `QoSProfile` が生成できる
  - reliability/durability の文字列正規化（例: `RELIABLE`, `Reliable`）
  - 不正値で `ValueError`

（rclpy の enum 値に依存しすぎないよう、プロパティの一致で検証する）

## Affected files

- `src/medical_robot_sim/medical_robot_sim/vital_sensor.py`
- `src/medical_robot_sim/medical_robot_sim/icu_monitor.py`
- `src/medical_robot_sim/medical_robot_sim/rule_alert_engine.py`
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`
- `src/medical_robot_sim/medical_robot_sim/qos_profiles.py`（新規）
- `src/medical_robot_sim/test/test_qos_profiles.py`（新規）
- `docs/day8_qos.md`

## Constraints

- topic 契約（/patient_XX/patient_vitals, /patient_XX/alerts）とメッセージ定義は変更しない
- Day5-7 の再現手順（alerts の生成、rule 外部化、flatline 検知）を壊さない
- Clean shutdown を維持する（Ctrl+C でスタックトレースを出さない）
- 変更は QoS 設計・明示化に限定し、ルール評価ロジックや alert の意味は変更しない

## Non-goals

- 新しい topic の追加、topic 名の変更
- message 定義の変更（`medical_interfaces` の改変）
- ルールセット追加や閾値設計の変更
- 複雑な QoS（deadline/lifespan/liveliness）を全面導入すること
- QoS を YAML 外部化すること（Day8 では ROS params までに留める）
