# Day16: 異常検知AI（advisory layer）（day16_anomaly_detection_ai）

## Purpose

本Dayの目的は、既存の rule-based alerting（`/patient_XX/alerts`）を「判断の主」から動かさずに、
**異常検知（AI/統計的検知）を“助言（advisory）レイヤ”として追加**できる設計・実装手順を確立すること。

- 既存 topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）は変更しない
- AI は **意思決定の権限を持たない**（alerts を置き換えない）
- 異常検知の出力は **任意の追加 topic**（例: `/patient_XX/advisories`）として外部化し、他ノードと疎結合にする
- 再現性（seed/シナリオ）と観測可能性（grep 可能ログ、topic echo）を優先する

## Background

医療監視や Physical AI では、単発しきい値（Day5）や temporal stability（Day6）だけでは拾いづらい異常がある。

- 「普段の分布」からの逸脱（外れ値/急変/組合せ）
- 既存ルールが無い/調整しづらい領域（患者差、デバイス差）
- BCI/EEG のようにノイズが多く、ルールだけで完全に記述しにくい入力（Day15）

一方で、このリポジトリの設計原則として **ルールベースが主**であり、LLM は “助言” に留める。
よって Day16 では、AI を「alerts の代替」ではなく、
**運用で観測できる追加ストリーム（advisories）として扱う**。

## Why this day matters in the roadmap

- Day12（rosbag）で記録・再生する際、advisories を含めることで「同じ入力に対し同じ助言が出るか」を検証しやすい
- Day17（マルチノード協調）で、advisories を別ノードが参照して“協調”に繋げられる（ただし最終判断は rule-based のまま）
- Day18（閉ループ制御）で、警報（alerts）とは別の“予兆”を入力として扱える

## Target topics/components

### 既存（維持）

- vitals: `/patient_XX/patient_vitals`（`medical_interfaces/msg/VitalSigns`）
- alerts: `/patient_XX/alerts`（`medical_interfaces/msg/Alert`）

### 追加（Day16）

- advisories: `/patient_XX/advisories`（型は `medical_interfaces/msg/Alert` を再利用）
  - topic 契約としては “future optional” の `/patient_XX/advisories` を採用
  - message は新規定義を避け、まずは既存 `Alert.msg` を **advisory 用途に転用**する
    - `kind=advisory` を固定
    - `rule_id` は `ai.*` の安定ID（例: `ai.flatline_hr`, `ai.spo2_drop`, `ai.hr_jump`）

### 対象コンポーネント（想定）

- 既存（拡張/利用）
  - `medical_robot_sim/anomaly_detector.py`（検知。純粋ロジック中心）
  - `medical_robot_sim/rule_evaluator.py`（イベント優先度付け）
  - `medical_robot_sim/advisory_engine.py`（表示用サマリ計算。必要なら拡張）

- 新規（追加）
  - `medical_robot_sim/advisory_publisher.py`（ROS2 node）
    - subscribe: 相対 `patient_vitals`
    - publish: 相対 `advisories`
    - anomaly events を `Alert.msg` に変換して publish

- launch
  - `launch/icu_multi_patient.launch.py` に `enable_advisories` を追加し、患者ごとに `advisory_publisher` を起動可能にする（既定は無効）

## Design policy

- **後方互換**: 既定では advisories を出さない（挙動に影響を与えない）
- **分離**: 検知（anomaly_detector）と publish（ROS node）を分離し、pytest で純粋ロジックをテストする
- **patient 分離**: 窓バッファ等の状態は患者ごとに隔離する（ノード分割 or pid別状態管理）
- **再現性**: Day6 の `scenario=flatline` 等を利用し、観測が安定する受け入れにする
- **観測可能**: Day11 形式の `event=... key=value` を出し、`grep -n` で機械判定できる
- **AIは助言**: `/patient_XX/alerts` を AI で置換しない。advisories は“補助情報”と明記する

## Implementation requirements

### 1) `advisory_publisher`（必須）

- executable 名: `advisory_publisher`
- namespace: 患者 namespace（例: `/patient_01`）で起動
- subscribe topic: 相対 `patient_vitals`（→ `/patient_01/patient_vitals`）
- publish topic: 相対 `advisories`（→ `/patient_01/advisories`）
- publish message: `medical_interfaces/msg/Alert`
  - `patient_id`: vitals 由来
  - `kind`: `advisory`
  - `rule_id`: `ai.*` の安定ID
  - `priority`: `INFO|YELLOW|RED` 等（設計上の最小マップでよい）
  - `ts/window_sec/field/value/delta/score`: AnomalyEvent から埋める（無いものは NaN/空文字）

params（最小）:
- `patient_id`（string）
- `window_sec` / `window_size`（int）: 検知窓（受け入れを短くできるよう外部化）
- `spo2_drop_threshold`（float）
- `hr_jump_threshold`（float）
- `field_epsilon.*`（float）: flatline 判定レンジ（必要なら）
- Day8 QoS（subscribe/publish）:
  - `vitals_qos_*`（subscribe）
  - `advisories_qos_*`（publish）

observability（必須）:
- `event=advisory.config`（起動時: thresholds/window/qos）
- `event=advisory.publish`（publish時: pid/rule_id/priority/type/score 等）

### 2) 検知ロジックの方針（必須）

- `anomaly_detector.FlatlineDetector` の **インスタンスを node 内に保持**し、患者ごとに状態を持つ
- `detect_anomalies()` の module-level detector は、Node では使わず（または reset 可能にして）テスト容易性を確保する
- まずは既存のイベント型（例: `flatline`, `spo2_drop`, `hr_jump`）を利用し、アルゴリズムは増やさない

### 3) Launch 統合（必須）

`icu_multi_patient.launch.py` に追加:
- `enable_advisories`（default `false`）
- `advisories_qos_depth/reliability/durability`
- （任意）`advisory_window_sec/window_size` 等

`enable_advisories:=true` のとき、患者ごとに `advisory_publisher` を namespace 付きで起動する。

### 4) 自動テスト（最低1つ）（必須）

pytest（`src/medical_robot_sim/test/`）で、少なくとも以下をカバーする。

- `FlatlineDetector` の境界値（window 未満ではイベント無し、window 到達で 1 回発火、連続サンプルで二重発火しない）
- `spo2_drop` / `hr_jump` のしきい値境界（ちょうど threshold）

（ROS node の統合テストは必須外。必要なら後続で追加。）

## Files expected to change

- `src/medical_robot_sim/medical_robot_sim/advisory_alerts.py`（新規: AnomalyEvent→Alert 変換の純粋関数）
- `src/medical_robot_sim/medical_robot_sim/advisory_publisher.py`（新規）
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`（enable_advisories 等の追加）
- `src/medical_robot_sim/setup.py`（console_scripts 追加）
- `src/medical_robot_sim/test/test_day16_advisory_alerts.py`（新規）

- 本ファイル: `docs/day16_anomaly_detection_ai.md`
- `specs/day16_anomaly_detection_ai/spec.md`
- `specs/day16_anomaly_detection_ai/tasks.md`
- `specs/day16_anomaly_detection_ai/acceptance.md`

## Reproduction / validation steps

- 受け入れ手順（コピペ）: `specs/day16_anomaly_detection_ai/acceptance.md`

### 最短コピペ（Quickstart: 5分で観測）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

rm -f /tmp/day16_quick.log
timeout -s INT 25s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  enable_advisories:=true \
  advisories_window_size:=4 \
  scenario:=flatline \
  > /tmp/day16_quick.log 2>&1 || true

# 助言ノードの設定ログ（実装要件）
grep -n "event=advisory\.config" /tmp/day16_quick.log || true

# publish痕跡（実装要件）
grep -n "event=advisory\.publish" /tmp/day16_quick.log || true

echo "OK: day16 quickstart ran (see /tmp/day16_quick.log)"
```

合否判定は acceptance（topic echo による観測）に従う。

## Success criteria

- `enable_advisories:=true` のとき `/patient_01/advisories` が成立し、`--once` で 1 件以上受信できる
- `enable_advisories:=false` では従来通り（後方互換）
- `colcon test --packages-select medical_robot_sim` が成功し、異常検知ロジックのユニットテストが少なくとも 1 つある
- Ctrl+C 停止でスタックトレース無し（既存要件を維持）

## Learning path

### Quickstart（最短で動かす）

- Quickstart で launch を起動し、`event=advisory.*` のログが残ることを確認
- acceptance で `ros2 topic echo --once` により `/patient_01/advisories` を機械判定

### 読み方（原因追跡の順番）

1. `/tmp/day16_*.log` の `event=advisory.config` を grep
   - window/threshold/QoS が期待通りか
2. `ros2 topic list | grep advisories`
   - topic が想定 namespace にあるか
3. `ros2 topic echo --once /patient_01/advisories ...`
   - QoS 不一致/未接続の切り分け
4. `anomaly_detector` の窓設定
   - window_size が大きすぎて発火まで時間がかかっていないか

### 必須（受け入れ）と任意（学習）

- 必須: vitals から advisories が 1 件以上 publish される
- 任意: threshold/window を調整し、発火タイミングと false positive を観察する

## Relevance to medical / BCI / Physical AI context

- 医療監視では「閾値を超えた後」だけでなく、**急変や予兆**を別系統で観測できると運用上の価値が高い
- BCI/EEG のようなノイズ入力では “ルールだけで完全に書き切らない” 方針が現実的
- Physical AI は入力品質（遅延/欠測/異常）に敏感で、advisories を別ストリームとして設計できると統合がしやすい

## Connection to the next day

- Day17（マルチノード協調）: advisories を参照するノードを追加しても、alerts（最終判断）とは分離したまま協調できる
- Day12（rosbag）: `/patient_XX/advisories` を記録・再生し、同条件で異常検知の挙動比較が可能になる

## README へ追記する場合の最小導線（設計メモ）

README には詳細を重複させず、追記するならリンクだけに留める:
- `docs/day16_anomaly_detection_ai.md`
- `specs/day16_anomaly_detection_ai/acceptance.md`
