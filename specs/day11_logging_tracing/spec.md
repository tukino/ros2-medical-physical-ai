# Day11 Logging / Tracing 実装仕様（spec）

## Goal

Day10（Fault Injection）で発生させる異常（drop/delay/pause/stop）に対して、
**原因追跡できるログ**を最小の変更で整備する。

- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）は変更しない
- 既定値は後方互換（ログ量が増えすぎず、従来の動作を壊さない）
- 乱数が絡む挙動は seed とともに記録し、再現性を担保
- Ctrl+C clean shutdown を維持

## Design

### 対象

- `vital_sensor`（fault 注入の発生源）
- `icu_monitor`（観測者: stale/no_data 状態遷移）
- `rule_alert_engine` / `rule_alert_engine_lifecycle`（観測者: alerts の発行根拠）

### ログの基本方針

- 1行=1イベント
- grep できる安定形式
- rclpy 非依存の整形関数を用意し、pytest で担保する

#### イベント形式

```
event=<name> key=value key=value
```

- key は辞書順で出力する（diff が取りやすい）
- 値は単純型（int/float/bool/str）を基本とする
- str は空白/改行を最小限サニタイズする

### 新規モジュール: `observability.py`

rclpy に依存しない純粋関数として追加する。

- `format_event(event: str, **fields) -> str`
  - `event=...` を先頭にし、残りを辞書順 `key=value` で連結
- `sanitize_value(value) -> str`
  - `\n` などを安全に置換（最小限）

### `vital_sensor` のイベント

起動時（必須）:
- `vitals.fault_config`
  - `drop_rate`, `delay_ms`, `jitter_ms`, `pause_after_sec`, `pause_duration_sec`, `stop_after_sec`, `seed`, `publish_rate_hz`

注入イベント（必須）:
- `vitals.pause_enter`, `vitals.pause_exit`
- `vitals.stop_trigger`

注入イベント（推奨: 間引きログ可）:
- `vitals.drop`
- `vitals.enqueue_delayed`

共通フィールド（推奨）:
- `node=vital_sensor`, `ns=/patient_01`, `tick`, `elapsed_sec`

### `icu_monitor` のイベント

- `monitor.patient_state`
  - `pid`, `state`（FRESH/STALE/NO_DATA）, `age_sec`, `last_measurement_id`

### `rule_alert_engine` のイベント

- `alerts.emit`
  - `pid`, `rule_id`, `priority`, `message`

### パラメータ（最小）

- `observability_verbose`（bool, default `false`）
  - true のとき drop/delay など頻度が高いイベントも詳細に出す

（拡張予定 / 予約）:
- `observability_stats_period_sec`
  - 統計ログの周期（必要になった場合のみ追加する。Day11の必須にはしない）

### テスト方針

pytest で `observability.py` の整形を固定し、フォーマット破壊を防ぐ。

- keys が辞書順で出力される
- `event=` が必ず先頭
- サニタイズが最低限効いている（改行が入らない）

## Affected files

- `src/medical_robot_sim/medical_robot_sim/observability.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/vital_sensor.py`
- `src/medical_robot_sim/medical_robot_sim/icu_monitor.py`
- `src/medical_robot_sim/medical_robot_sim/rule_alert_engine.py`
- `src/medical_robot_sim/medical_robot_sim/rule_alert_engine_lifecycle.py`
- `src/medical_robot_sim/test/test_observability.py`（新規）
- `docs/day11_logging_tracing.md`
- `specs/day11_logging_tracing/spec.md`
- `specs/day11_logging_tracing/tasks.md`
- `specs/day11_logging_tracing/acceptance.md`

## Constraints

- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）と message 定義を変更しない
- fault injection の挙動（drop/delay/pause/stop）を変えない（観測だけ追加）
- Ctrl+C clean shutdown（スタックトレース無し）を維持
- 外部依存を安易に増やさない（重いトレーシング基盤の導入は非必須）

## Non-goals

- 新しい observability 用 topic の追加（Day11 の必須ではない）
- OpenTelemetry などの外部基盤の導入
- launch_testing 等の重い統合テスト導入
- 医療機器レベルの監査ログ/改ざん耐性
