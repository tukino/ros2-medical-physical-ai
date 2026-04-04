# Day17: マルチノード協調（day17_multi_node_coordination）

## Purpose

本Dayの目的は、既存の multi-patient ICU シミュレーションを「ノードが個別に動いている状態」から一段進め、
**複数ノードが“状態”を共有し、順序・安全条件を満たすように協調して動作する**設計・実装手順を確立すること。

このリポジトリの設計原則に従い、協調の対象はまず **ルールベース alerting を中心**に据える。

- 既存 topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）は変更しない
- Day9 の Lifecycle Node（`rule_alert_engine_lifecycle`）を活用し、
  **外部ノード（Coordinator）が状態遷移（configure/activate/deactivate）を制御**できるようにする
- 協調機能は **既定で無効**（後方互換）とし、`enable_coordination:=true` のときのみ有効化する
- 観測可能性（Day11形式ログ + `ros2 lifecycle`）と再現性（同じ起動手順で同じ状態遷移）が必須

## Background

医療監視や Physical AI の実システムでは、単一ノードの正しさだけでなく、
**システム全体の起動順・状態・依存関係（データが来ているか/来ていないか）** が安全性と運用性を左右する。

- データが届いていない（NO_DATA）状態で alerting を有効にすると、誤アラートやノイズを生みやすい
- センサー側が停止/遅延（Day10）したとき、監視・アラートの“望ましい状態”は一意でない
  - 例: データ無しならアラート生成を止める、データ復帰時に再開する、など

Day9 で Lifecycle により「止められるアラート」を獲得した。
Day17 はそれを **手動操作（CLI）から自動協調（別ノードが判断して遷移）へ**進め、
次の Day18（閉ループ制御）に必要な土台を作る。

## Why this day matters in the roadmap

- Day10/11 の“異常を作る/説明する”に対し、Day17 は **異常時にシステムとしてどう振る舞うか** を定義する
- Day9 Lifecycle を “運用で触れる機能” から **システムの安全ガード** に昇格できる
- Day18（閉ループ）では「状態に応じて制御を切り替える」必要があるため、
  Day17 の **協調・オーケストレーション**がその前提になる

## Target topics/components

### 既存（維持）

- vitals: `/patient_XX/patient_vitals`（`medical_interfaces/msg/VitalSigns`）
- alerts: `/patient_XX/alerts`（`medical_interfaces/msg/Alert`）

### 対象コンポーネント

- `vital_sensor`（入力源）
- `icu_monitor`（観測者）
- `rule_alert_engine_lifecycle`（制御対象）
- **新規**: `icu_coordinator`（協調/オーケストレーション）

## Design policy

- **後方互換**: 既定では coordinator を起動しない（従来の起動がそのまま動く）
- **疎結合**: coordinator は topic と lifecycle service を通してのみ干渉する
- **安全側**: 期待する前提が満たせない場合（lifecycle node が無い等）は “何もしない” を基本にする
- **観測可能**: Day11形式の `event=coord.*` ログと、`ros2 lifecycle get/set` で状態を確認できる
- **テスト可能**: 協調判定（ready/no_data の判定）は pure function として切り出し、pytest で自動テストする

## Implementation requirements

### 1) `icu_coordinator` node（必須）

新規ノード `icu_coordinator` を追加し、`rule_alert_engine_lifecycle` の状態を制御する。

- node name: `icu_coordinator`（root namespace）
- 役割:
  - `/patient_XX/patient_vitals` の受信状況を監視し、
    “アラートエンジンを稼働させる準備が整ったか” を判断する
  - 準備が整ったら、`/rule_alert_engine` の lifecycle service を呼び、
    `configure -> activate` を実行する
  - （任意・既定ON）全患者が一定時間 NO_DATA のとき `deactivate` する（ノイズ抑制）

### 2) Coordinator が使う判定（必須）

- “ready” 条件（最小）:
  - 患者ごとに `/patient_XX/patient_vitals` を `min_messages_per_patient` 件受信
  - `ready_timeout_sec` を超えた場合は ready 失敗としてログに残し、以後は safe に待機（または degrade）

- “no data” 判定（最小）:
  - 各患者の最終受信から `no_data_after_sec` を超えたら NO_DATA
  - **全患者が NO_DATA** の場合のみ `deactivate` する（患者が一人でも生きていれば動かす）

### 3) Parameters（必須）

`icu_coordinator` の最小パラメータ:

- `patients`（string array）: 対象患者（launch から渡す）
- `vitals_topic`（string, default `patient_vitals`）
- `min_messages_per_patient`（int, default 1）
- `ready_timeout_sec`（float, default 10.0）
- `check_period_sec`（float, default 1.0）
- `stale_after_sec`（float, default 3.0）
- `no_data_after_sec`（float, default 10.0）
- `deactivate_on_no_data`（bool, default true）

QoS（Day8流儀、vitals subscribe に使用）:
- `vitals_qos_depth`, `vitals_qos_reliability`, `vitals_qos_durability`

### 4) Observability（必須）

Day11形式（1行=1イベント、`key=value`）で次を出す。

- 起動時: `event=coord.config`（patients/閾値/QoS 等）
- ready 到達: `event=coord.ready`（pidごとの受信数、ready判定）
- lifecycle 操作: `event=coord.lifecycle_set`（target, transition, result）
- no_data で停止: `event=coord.deactivate_on_no_data`（理由、患者状態）

### 5) Launch 統合（必須）

`launch/icu_multi_patient.launch.py` に次を追加する。

- `enable_coordination`（default `false`）
- `coord_min_messages_per_patient`
- `coord_ready_timeout_sec`
- `coord_check_period_sec`
- `coord_stale_after_sec`
- `coord_no_data_after_sec`
- `coord_deactivate_on_no_data`

起動条件（推奨の組合せ）:
- `enable_alerts:=true`
- `alerts_node_kind:=lifecycle`
- `lifecycle_autostart:=false`（coordinator が activate するため）
- `enable_coordination:=true`

### 6) 自動テスト（最低1つ）（必須）

pytest（`src/medical_robot_sim/test/`）で coordinator の判定ロジックをテストする。

- ready 判定の境界値（min_messages_per_patient 未満では false、到達で true）
- NO_DATA 判定（last_seen age の境界）
- “全患者 NO_DATA のときだけ deactivate” の判定

## Files expected to change

- `src/medical_robot_sim/medical_robot_sim/icu_coordinator.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/coordination_policy.py`（新規: pure functions）
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`（enable_coordination 等の追加）
- `src/medical_robot_sim/setup.py`（console_scripts 追加）
- `src/medical_robot_sim/test/test_day17_coordination_policy.py`（新規）

- 本ファイル: `docs/day17_multi_node_coordination.md`
- `specs/day17_multi_node_coordination/spec.md`
- `specs/day17_multi_node_coordination/tasks.md`
- `specs/day17_multi_node_coordination/acceptance.md`

## Reproduction / validation steps

- 受け入れ手順（コピペ）: `specs/day17_multi_node_coordination/acceptance.md`

### 最短コピペ（Quickstart: 5分で観測）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

rm -f /tmp/day17_quick.log
timeout -s INT 25s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  alerts_node_kind:=lifecycle \
  lifecycle_autostart:=false \
  enable_coordination:=true \
  scenario:=flatline \
  > /tmp/day17_quick.log 2>&1 || true

# coordinator が起動して設定ログを出す
grep -n "event=coord\.config" /tmp/day17_quick.log || true

# lifecycle 制御を試みたログ（成功/失敗どちらでも痕跡が残ること）
grep -n "event=coord\.lifecycle_set" /tmp/day17_quick.log || true

echo "OK: day17 quickstart ran (see /tmp/day17_quick.log)"
```

合否の厳密判定は acceptance に従う（`ros2 lifecycle get` を含む）。

## Success criteria

- `enable_coordination:=true` で `icu_coordinator` が起動し、`event=coord.config` を出す
- lifecycle モード（Day9）で `lifecycle_autostart:=false` のとき、
  coordinator が `configure -> activate` を実行し、`ros2 lifecycle get /rule_alert_engine` が `active` を返す
- `enable_coordination:=false` では従来互換（coordinator が起動しない）
- `colcon test --packages-select medical_robot_sim` が成功し、Day17 追加ロジックの pytest が少なくとも 1 つある
- Ctrl+C 停止でスタックトレース無し（Day3方針を維持）

## Learning path

### Quickstart（最短で動かす）

- Quickstart を実行し、`event=coord.*` ログが残ることを確認
- acceptance で `ros2 lifecycle get` により状態（unconfigured/active 等）を機械判定

### 読み方（原因追跡の順番）

1. `/tmp/day17_*.log` の `event=coord.config` を grep
   - patients/QoS/閾値が期待通りか
2. `ros2 node list | grep icu_coordinator`
   - coordinator が起動しているか
3. `ros2 lifecycle get /rule_alert_engine`
   - lifecycle node が存在するか、状態は何か
4. `event=coord.lifecycle_set` の結果
   - サービス未提供（起動順/QoS）か、遷移失敗かを切り分け

### 必須（受け入れ）と任意（学習）

- 必須: coordinator により `rule_alert_engine` が active になる
- 任意: `vitals_fault_pause_after_sec` 等（Day10）で NO_DATA を作り、`deactivate_on_no_data` の挙動を観察

## Relevance to medical / BCI / Physical AI context

- 医療監視では「入力が無い」状態での誤作動を抑えることが重要で、
  lifecycle による **安全な停止/再開** は運用価値が高い
- BCI/EEG などノイズ/欠測の多い入力では、下流処理の **有効/無効を状態で切り替える**設計が必須になりやすい
- Physical AI の閉ループ（Day18）では、制御器が常に動いてよいとは限らないため、
  協調によるフェイルセーフが土台になる

## Connection to the next day

- Day18（閉ループ制御）では、状態に応じて制御を有効/無効にする必要がある。
  Day17 の coordinator は、その “状態遷移を外部から制御する” 最小実装として再利用できる。

## README へ追記する場合の最小導線（設計メモ）

README には詳細を重複させず、追記するならリンクだけに留める:
- `docs/day17_multi_node_coordination.md`
- `specs/day17_multi_node_coordination/acceptance.md`
