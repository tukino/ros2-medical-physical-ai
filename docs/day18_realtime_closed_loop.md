# Day18: リアルタイム制御（閉ループ）（day18_realtime_closed_loop）

## Purpose

本Dayの目的は、既存の監視パイプライン（vitals/alerts/advisories/coordinator）を基盤として、
**入力（vitals/alerts）→判断（ルールベース policy）→制御（action publish）** を継続ループ化し、
「観測するだけ」から「安全に介入できる」システムへ拡張することである。

このDayでは、以下を必須とする。

- 既存 topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）を維持
- rule-based を主とし、LLM は意思決定に使わない
- 制御出力を観測可能な topic とログで外部化する
- 後方互換を維持（既定で閉ループ無効）
- Ctrl+C で clean shutdown を維持

## Background

Day16 で「助言（advisories）」、Day17 で「協調（coordinator）」が入ったことで、
システムは「異常を見つける」「状態を制御する」までは可能になった。
一方、Physical AI としては、異常検知後に何らかの介入（制御）を行い、
再び入力に反映される閉ループが必要になる。

閉ループ導入時は、次の失敗が起きやすい。

- 閾値付近で制御が頻繁に切替わる（chattering）
- 入力欠測時に誤って強い制御を出す
- 操作履歴が残らず、なぜその制御になったか説明できない

Day18 ではアルゴリズム高度化より先に、
**安全ガード付きの最小 policy と再現可能な観測手順**を確立する。

## Why this day matters in the roadmap

- Day17（マルチノード協調）で獲得した「状態制御」を、実際の介入判断に接続できる
- Day13/14（実センサー・Edge）で通信条件が変わっても、同じ制御 policy で評価できる
- BCI/EEG（Day15）や advisories（Day16）を将来の追加入力として統合しやすくなる
- ロードマップ最終段の「人を支援するシステム」の最小成立条件を満たす

## Target topics/components

### 既存（維持）

- vitals: `/patient_XX/patient_vitals`（`medical_interfaces/msg/VitalSigns`）
- alerts: `/patient_XX/alerts`（`medical_interfaces/msg/Alert`）
- advisories（任意）: `/patient_XX/advisories`（`medical_interfaces/msg/Alert`）

### Day18 で追加する topic/component

- control actions: `/patient_XX/control_actions`（`medical_interfaces/msg/Alert` を再利用）
  - `kind=control_action`
  - `rule_id=control.*`（例: `control.oxygen_boost`, `control.call_staff`, `control.hold`）

- 新規 node: `closed_loop_controller`
  - subscribe: `patient_vitals`, `alerts`（必要に応じて `advisories`）
  - publish: `control_actions`

- 新規 pure policy: `closed_loop_policy.py`
  - ループ判断を rclpy 非依存関数として分離し、pytest で境界テスト可能にする

## Design policy

- **安全側デフォルト**: データ欠測・不整合・初期化前は `HOLD` のみ
- **後方互換**: `enable_closed_loop:=false` を既定値にし、従来 launch へ影響を与えない
- **判定は純粋関数**: 制御判定を `closed_loop_policy.py` に集約
- **振動抑制**: cooldown（最短発行間隔）と hysteresis（復帰条件）を必須化
- **観測可能性**: Day11 形式の `event=control.*` ログで判定理由を可視化
- **説明可能性**: どの入力でどの action が出たかを `reason` として必ず残す

## Implementation requirements

### 1) `closed_loop_policy.py`（必須）

最小入力:

- `hr`, `spo2`
- 最新 alert（priority/rule_id）
- `age_sec`（最終 vitals 受信からの経過秒）
- 直前 action と直前発行時刻（cooldown 用）

最小出力:

- `action`（`HOLD` / `OXYGEN_BOOST` / `CALL_STAFF`）
- `reason`（機械判定しやすい文字列）
- `should_publish`（cooldown 等で抑制された場合 false）

最低限の判定例（初版）:

- `age_sec > no_data_after_sec` -> `HOLD`（安全側）
- `spo2 <= critical_spo2` または `priority=RED` -> `CALL_STAFF`
- `spo2 <= low_spo2` または `rule_id=outlier.spo2` -> `OXYGEN_BOOST`
- それ以外 -> `HOLD`

### 2) `closed_loop_controller` node（必須）

- 患者 namespace で1ノードずつ起動（`/patient_XX`）
- subscribe:
  - 相対 `patient_vitals`
  - 相対 `alerts`
  - （任意）相対 `advisories`
- publish:
  - 相対 `control_actions`（`medical_interfaces/msg/Alert`）

publish payload（`Alert`再利用）:

- `patient_id`: namespace の患者ID
- `kind`: `control_action`
- `rule_id`: `control.*`
- `priority`: `INFO/YELLOW/RED`（action に応じて固定マップ）
- `field`: `spo2` or `hr`（主因）
- `value`: 現在値
- `score`: 制御強度（0.0/0.5/1.0 など）

### 3) launch 統合（必須）

`src/medical_robot_sim/launch/icu_multi_patient.launch.py` に追加:

- `enable_closed_loop`（default `false`）
- `control_topic`（default `control_actions`）
- `control_cooldown_sec`（default `5.0`）
- `control_no_data_after_sec`（default `10.0`）
- `control_low_spo2`（default `92.0`）
- `control_critical_spo2`（default `88.0`）
- `enable_control_from_advisories`（default `false`）

起動推奨（最小）:

- `enable_alerts:=true`
- `enable_closed_loop:=true`
- `scenario:=flatline` または `scenario:=normal`

### 4) Observability（必須）

最低限のイベント:

- `event=control.config`（閾値/cooldown/topic）
- `event=control.decision`（pid/action/reason/input）
- `event=control.publish`（publishした action）
- `event=control.suppressed`（cooldown 等で抑制）
- `event=control.safe_hold`（NO_DATA で安全停止）

### 5) 自動テスト（必須）

`src/medical_robot_sim/test/test_day18_closed_loop_policy.py` を追加し、少なくとも以下を検証:

- 境界値:
  - `spo2 == low_spo2` で `OXYGEN_BOOST`
  - `spo2 == critical_spo2` で `CALL_STAFF`
- 安全側:
  - `age_sec > no_data_after_sec` は常に `HOLD`
- 振動抑制:
  - cooldown 内は `should_publish=false`
- 負ケース:
  - 正常値 + alertなしで `HOLD`

## Files expected to change

- `src/medical_robot_sim/medical_robot_sim/closed_loop_policy.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/closed_loop_controller.py`（新規）
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`
- `src/medical_robot_sim/setup.py`
- `src/medical_robot_sim/test/test_day18_closed_loop_policy.py`（新規）

- 本ファイル: `docs/day18_realtime_closed_loop.md`
- `specs/day18_realtime_closed_loop/spec.md`
- `specs/day18_realtime_closed_loop/tasks.md`
- `specs/day18_realtime_closed_loop/acceptance.md`

## Reproduction / validation steps

- 詳細な受け入れ手順は `specs/day18_realtime_closed_loop/acceptance.md` を参照

### Quickstart（5分で確認）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

rm -f /tmp/day18_quick.log
timeout -s INT 35s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  enable_closed_loop:=true \
  scenario:=flatline \
  > /tmp/day18_quick.log 2>&1 || true

grep -n "event=control\.config" /tmp/day18_quick.log || true
grep -n "event=control\.decision" /tmp/day18_quick.log || true
grep -n "event=control\.publish" /tmp/day18_quick.log || true

echo "OK: day18 quickstart ran (see /tmp/day18_quick.log)"
```

## Success criteria

- `enable_closed_loop:=true` で `control.*` ログが出る
- `/patient_01/control_actions` が観測できる（`ros2 topic echo --once`）
- `enable_closed_loop:=false` で従来動作（control topic未生成）
- policy の pytest が追加され `colcon test --packages-select medical_robot_sim` が成功
- Ctrl+C 停止で `Traceback`/`KeyboardInterrupt` を出さない

## Learning path

### 最短コピペ（Quickstart）

1. 上記 Quickstart を実行し `control.*` ログを確認
2. 受け入れ（acceptance）で topic の有無と action 発行を機械判定

### 読み方（トラブル時の順番）

1. `/tmp/day18_*.log` の `event=control.config` を grep
   - 閾値/cooldown/有効化フラグが期待どおりか
2. `ros2 node list | grep closed_loop_controller`
   - controller が起動しているか
3. `ros2 topic echo --once /patient_01/control_actions`
   - action publish が成立しているか
4. `event=control.suppressed` / `event=control.safe_hold` を確認
   - cooldown で抑制されたのか、NO_DATA で停止したのかを切り分け

### 必須（受け入れ）と任意（学習）

- 必須: control topic の生成・action 発行・後方互換
- 任意: advisories を入力に足し、`enable_control_from_advisories` の差分を比較

### 学習者が説明できるようになること（達成目標）

- 閉ループ制御の最小構成を、topic と policy の観点で説明できる
- 安全側ガード（NO_DATA/cooldown/hysteresis）が必要な理由を説明できる
- ログと topic 観測だけで、制御判断の根拠を追跡できる

## Relevance to medical / BCI / Physical AI context

- 医療文脈では「異常検知」だけでなく、「誤介入を避ける制御設計」が同等に重要
- BCI/EEG のような揺らぎ入力では、入力欠測時の安全停止が必須
- Physical AI の本質である「知覚→判断→行動」のループを最小実装で成立させる

## Connection to the next day

Day18 はロードマップ上の到達点だが、次の実践課題として以下に接続する:

- rosbag replay（Day12）で閉ループ判定の再現検証を自動化
- Edge（Day14）で遅延条件を変えたときの制御安定性比較
- BCI（Day15）+ advisory（Day16）入力を使ったマルチモーダル制御

## README へ追記する場合の最小導線（設計メモ）

README へは詳細を書かず、以下リンクのみ追記候補とする。

- `docs/day18_realtime_closed_loop.md`
- `specs/day18_realtime_closed_loop/acceptance.md`
