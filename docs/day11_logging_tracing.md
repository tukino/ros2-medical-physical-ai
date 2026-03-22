# Day11: Logging / Tracing（day11_logging_tracing）

## Purpose

PLAN.md での Day11 の狙いは「ログとトレースを整備して、**なぜ起きたかを説明できる**」ようにすること。
本Dayでは Day10（Fault Injection）で再現できる「drop/delay/pause/stop」に対して、
**原因（注入パラメータ）→ 発生イベント → 監視/アラート側の見え方** をログで接続できる状態を作る。

Day10（Fault Injection）で再現できる「drop/delay/pause/stop」に対して、
**なぜその現象が起きたのか** を第三者が追跡できるように、ログと（任意で）トレースを整備する。

この Day のゴールは「ログが綺麗」ではなく、次を満たすこと:
- 同じ手順で再現した障害について、原因（注入パラメータと発生イベント）をログから説明できる
- 監視側の見え方（STALE/NO DATA などの状態遷移）と、注入イベントの時系列が対応づけられる
- Ctrl+C による clean shutdown を維持する（ログ/トレース導入で壊さない）

## Background

医療監視や Physical AI の入力系では、正常時よりも異常時の切り分けが重要になる。
Day10 で障害（drop/delay/pause/stop）を「設計された条件として」再現できるようになったが、
次の問いに答えるには観測が不足する:

- 「データが来ない」のは drop/pause/stop のどれか？
- delay の影響で遅れているのか、単に publish 自体が止まっているのか？
- monitor/alert 側で stale/no_data になった瞬間はいつか？

Day11 は、Day10 の再現性に **原因追跡可能性** を加える。

## このDayで「説明できる」ようになること（達成イメージ）

Day11を完了すると、ログだけで次を説明できる:

- 何を注入したか（例: `vitals.fault_config` の `drop_rate/delay_ms/.../seed`）
- 何が起きたか（例: `vitals.pause_enter/exit` や `vitals.stop_trigger`）
- 下流がどう見えたか
  - monitor: `monitor.patient_state`（FRESH/STALE/NO_DATA の遷移）
  - alerts: `alerts.emit`（どの `rule_id` が、どの `priority` で出たか）

学習のコツ:
- Day10 は「異常を作る」、Day11 は「異常を説明できるように観測する」。
- まずは pause/stop で時系列を掴み、次に（任意で）drop/delay を verbose で追う。

## Why this day matters in the roadmap

- Day12（rosbag）で「障害の記録→再生」を行う際、
  *何を入れた結果どう見えたか* をログで説明できると実験の再現性が上がる。
- Day8（QoS）/ Day9（Lifecycle）と組み合わせて、
  QoS 不一致・inactive・真の欠落/停止を分離して評価するためには、
  状態遷移と fault イベントの記録が必要。

## Target topics/components

### 対象 topic（契約は変更しない）

- vitals: `/patient_XX/patient_vitals`
- alerts: `/patient_XX/alerts`

### 対象コンポーネント（最小スコープ）

- `vital_sensor`（Day10 fault 注入の主戦場）
- `icu_monitor`（観測者: stale/no_data の状態遷移をログ化）
- `rule_alert_engine` / `rule_alert_engine_lifecycle`（観測者: alert 発行の根拠をログ化）

## Design policy

- 後方互換: 既定ではログの量・形式が既存運用を壊さない
- 低コスト: rclpy 非依存の純粋関数を中心にし、pytest でテスト可能にする
- grep 可能: 1行=1イベント（キー=値）の形式で機械的に抽出できる
- 再現性: 乱数が関わる fault は seed とともに必ず記録する
- ノイズ抑制: 毎サンプルの詳細ログはデフォルトOFF（必要なイベントだけ）

## Implementation requirements

### 現状の実装ステータス（最小）

実装済みイベント（grep で確認可能）:
- `vitals.fault_config`
- `vitals.pause_enter`, `vitals.pause_exit`
- `vitals.stop_trigger`
- （verbose時のみ）`vitals.drop`, `vitals.enqueue_delayed`
- `monitor.patient_state`（FRESH/STALE/NO_DATA の状態遷移時のみ）
- `alerts.emit`（アラート publish 時のみ）

未実装（TODO / verbose 時のみ想定）:
（現時点なし）

## イベントログの読み方（最短）

ログは「1行=1イベント」なので、grep で必要な行だけ拾って因果を追う。

最初に見るべき順番:

1. `vitals.fault_config`
  - Day10 の注入条件がこの1行に出る（再現性の根拠）。
2. `vitals.pause_enter/exit` / `vitals.stop_trigger`
  - 「publish が止まった」のが設計された pause/stop なのかが分かる。
3. `monitor.patient_state`
  - pause などの影響で STALE/NO_DATA に落ちた瞬間が追える。
4. `alerts.emit`
  - “何が根拠でアラートになったか”が追える（rule_id/priority/message）。

（任意）`observability_verbose=true` のとき:
- `vitals.drop`: drop が実際に発生した tick を追える
- `vitals.enqueue_delayed`: delay/jitter による遅延キュー投入を追える（`due_tick/queue_len`）

### 1) 共通のイベントログ形式（最小スキーマ）

各ノードは「イベント」を 1行ログとして出力する。

- 形式: `event=<name> key=value key=value ...`（スペース区切り）
- キー順は安定（辞書順）にして、差分比較しやすくする
- 値は基本的に単純型（int/float/str/bool）に限定

必須フィールド（最低限）:
- `event`: イベント名（例: `vitals.drop`）
- `node`: ノード名（例: `vital_sensor`）
- `ns`: namespace（例: `/patient_01`）

推奨フィールド:
- `pid`: patient_id（message内 patient_id）
- `tick`: measurement_count（vital_sensorのサンプル番号）
- `elapsed_sec`: 起動からの経過（tick/publish_rate_hz で算出）

### 2) `observability.py`（純粋関数）を追加

- `format_event(event: str, fields: dict) -> str`
  - `event=...` + `key=value` の並びを生成
  - keys は辞書順で出力
- `sanitize_value(v) -> str`
  - 文字列の空白/改行などを安全に表現（最低限）

このモジュールは rclpy に依存しない。

### 3) `vital_sensor` のログ強化（Day10 fault の原因が追えること）

必須イベント:
- 起動時: `vitals.fault_config`
  - drop_rate/delay_ms/jitter_ms/pause_after_sec/pause_duration_sec/stop_after_sec/seed/publish_rate_hz
- drop: `vitals.drop`
  - 少なくとも「drop が発生した」ことがログで分かる（サンプリング間引き可）
- delay/jitter: `vitals.enqueue_delayed`
  - delay_ticks と queue_len を記録（サンプリング間引き可）
- pause: `vitals.pause_enter`, `vitals.pause_exit`
- stop: `vitals.stop_trigger`

注意: デフォルトで毎サンプルをログらない。
- param `observability_verbose`（bool, default false）で詳細ログON

（拡張予定 / 予約）
- `observability_stats_period_sec`（統計ログ周期）

### 4) `icu_monitor` の状態遷移ログ

Day10 の観測が「画面表示」だけだと機械的に追えないため、患者ごとの状態遷移をイベントとして出す。

- `monitor.patient_state`
  - `pid`, `state`（FRESH/STALE/NO_DATA）, `age_sec`, `last_measurement_id`

### 5) `rule_alert_engine` のアラート根拠ログ

- `alerts.emit`
  - `pid`, `rule_id`, `priority`, `message`, `measurement_id`（可能なら）

### 6) Tracing（任意）

システムに `ros2_tracing` が導入されている場合のみ、
追加の依存を増やさずに「実行の時間的な分解能」を上げる。

- 目的: callback/タイマ遅延など、ログでは見えないタイミング問題の把握
- 非必須: Tracing が無くても Day11 は合格できる

## Files expected to change

- `src/medical_robot_sim/medical_robot_sim/observability.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/vital_sensor.py`（ログ追加）
- `src/medical_robot_sim/medical_robot_sim/icu_monitor.py`（状態遷移ログ）
- `src/medical_robot_sim/medical_robot_sim/rule_alert_engine.py`（alert根拠ログ）
- `src/medical_robot_sim/medical_robot_sim/rule_alert_engine_lifecycle.py`（同上）
- `src/medical_robot_sim/test/test_observability.py`（新規）
- `docs/day11_logging_tracing.md`（本ファイル）
- `specs/day11_logging_tracing/spec.md`
- `specs/day11_logging_tracing/tasks.md`
- `specs/day11_logging_tracing/acceptance.md`

## Reproduction / validation steps

- [specs/day11_logging_tracing/acceptance.md](specs/day11_logging_tracing/acceptance.md) を参照

### 最短のコピペ検証（学習の入口）

以下は「Day11で何が見えるようになったか」を最短で掴むための入口。
フルの受け入れは acceptance を使う。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f /tmp/day11_quick.log
timeout -s INT 8s ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_pause_after_sec:=1.0 \
  vitals_fault_pause_duration_sec:=2.0 \
  > /tmp/day11_quick.log 2>&1

grep -n "event=vitals\.fault_config" /tmp/day11_quick.log
grep -n "event=vitals\.pause_enter" /tmp/day11_quick.log
grep -n "event=vitals\.pause_exit" /tmp/day11_quick.log
grep -n "event=monitor\.patient_state" /tmp/day11_quick.log
```

## Success criteria

- Day10 の代表ケース（drop/pause/stop/delay のいずれか）で、
  「何を注入し、どのイベントが発生し、monitor/alert がどう見えたか」をログで説明できる
- `colcon test --packages-select medical_robot_sim` が通り、ログ整形の自動テストが最低1つある
- Ctrl+C で clean shutdown を維持する

## Relevance to medical / BCI / Physical AI context

- 医療監視では、NO DATA や途切れはそれ自体が重大イベントになり得る。
  その根拠（センサ側停止/通信欠落/一時停止）を説明できることは運用上の価値が高い。
- Physical AI は入力遅延に弱く、遅延・欠落の「発生タイミング」が重要。
  Day11 のログ/トレースは、設計判断（バッファ/QoS/復旧）を支える観測基盤になる。

## Connection to the next day

- Day12（rosbag）: Fault 注入シナリオを rosbag に記録し、同じ現象を再生して比較する。
  Day11 のログは「その実験が同じ条件だったか」を保証するためのメタ情報になる。
