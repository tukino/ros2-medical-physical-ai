# Day9 Lifecycle Node導入 実装仕様（spec）

## Goal

`rule_alert_engine` を Lifecycle Node として起動可能にし、状態遷移（configure/activate/deactivate）で alerts の発行を制御できるようにする。

- topic 契約（/patient_XX/patient_vitals, /patient_XX/alerts）を維持
- 既定動作は後方互換（classic で従来通り、または lifecycle autostart で同等）
- `ros2 lifecycle` コマンドで観測・操作できる

## Design

### 対象

- 対象ノード: `rule_alert_engine`（Lifecycle 版）
- 対象 topic:
  - subscribe: `/{pid}/{vitals_topic}`（既定 `patient_vitals`）
  - publish: `/{pid}/{alert_topic}`（既定 `alerts`）

### 追加する executable / node

- 新規 executable: `rule_alert_engine_lifecycle`
  - node name: `rule_alert_engine`（既存名を維持）
  - class: `rclpy.lifecycle.LifecycleNode`

既存 `rule_alert_engine`（classic）は維持し、launch 引数で切替する。

### Lifecycle と動作

- `on_configure`:
  - 既存 `rule_alert_engine` と同等の parameter 解釈（Day7 YAML / Day8 QoS 含む）
  - QoSProfile 構築（`build_qos_profile` を利用）
  - 内部状態（患者別バッファ、cooldown状態等）を初期化

- `on_activate`:
  - subscription/publisher を作成し、メッセージ処理・publish を開始

- `on_deactivate`:
  - subscription/publisher を停止（destroy）し、publish が止まることを担保
  - 必要なら内部バッファもクリアして再開時の再現性を上げる

- `on_cleanup`（任意・最小）:
  - configure 前相当まで戻す（内部状態の破棄）

状態差が観測できること（active でのみ alerts が流れること）を最優先する。

### 起動互換（autostart）

Lifecycle 版 node に次の parameter を追加:

- `lifecycle_autostart`（bool, default `true`）
  - true: 起動後に `configure -> activate` を自動実行
  - false: unconfigured で待機し、外部から `ros2 lifecycle set` で `configure -> activate` する

### launch の切替

[src/medical_robot_sim/launch/icu_multi_patient.launch.py](src/medical_robot_sim/launch/icu_multi_patient.launch.py) に追加:

- `alerts_node_kind`（string, default `classic`）
  - `classic`: executable=`rule_alert_engine`
  - `lifecycle`: executable=`rule_alert_engine_lifecycle`

- `lifecycle_autostart`（bool, default `true`）
  - lifecycle 版への parameter として渡す

### テスト方針

pytest（[src/medical_robot_sim/test/](src/medical_robot_sim/test/)）で最小限の状態遷移を確認する。

- `lifecycle_autostart=false` でノード生成後、未 activate では publish しないこと
  - 実装上、`_is_active` のような内部フラグに依存してもよい（黒箱で topic 実験までやらない）
- `trigger_configure/activate/deactivate`（または同等の内部メソッド）で例外が出ないこと

## Affected files

- [src/medical_robot_sim/medical_robot_sim/rule_alert_engine_lifecycle.py](src/medical_robot_sim/medical_robot_sim/rule_alert_engine_lifecycle.py)（新規）
- [src/medical_robot_sim/setup.py](src/medical_robot_sim/setup.py)（console_scripts 追加）
- [src/medical_robot_sim/launch/icu_multi_patient.launch.py](src/medical_robot_sim/launch/icu_multi_patient.launch.py)（alerts_node_kind 追加）
- [src/medical_robot_sim/test/test_rule_alert_engine_lifecycle.py](src/medical_robot_sim/test/test_rule_alert_engine_lifecycle.py)（新規）
- [docs/day9_lifecycle.md](docs/day9_lifecycle.md)

## Constraints

- topic 契約（/patient_XX/patient_vitals, /patient_XX/alerts）と message 定義は変更しない
- Day3 の clean shutdown を維持（Ctrl+C でスタックトレースを出さない）
- Day7（ルール外部化）と Day8（QoS切替）の挙動を壊さない
- 変更範囲は lifecycle 導入に限定し、ルール内容・閾値・評価ロジックは変更しない

## Non-goals

- すべてのノード（vital_sensor/icu_monitor）を lifecycle 化すること
- lifecycle manager や自動復旧のオーケストレーションを作り込むこと
- message/ topic の追加・改名
- launch_testing 等の重い統合テスト基盤の導入
