# Day10 Fault Injection 実装仕様（spec）

## Goal

Fault Injection（障害注入）を導入し、vitals 入力が
- 欠落（drop）
- 遅延（delay + jitter）
- 一時停止（pause）
- 擬似停止（stop）

する状況を **ROS params / launch 引数だけで再現**できるようにする。

- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）は変更しない
- 既定値は後方互換（障害注入なし）
- 乱数が絡む挙動は seed により再現可能にする
- Ctrl+C で clean shutdown を維持する

## Design

### 対象

- 対象ノード: `vital_sensor`
- 対象 topic: `/<pid>/patient_vitals`（publish）

`icu_monitor` と `rule_alert_engine` は、fault 注入の影響を観測する側として利用する。

### 追加するモジュール（純粋関数 중심）

新規に `medical_robot_sim/fault_injection.py` を追加し、少なくとも次を提供する。

- `validate_fault_params(...) -> None`
  - 不正値（負数、範囲外）を `ValueError` で落とす

- `ms_to_ticks(ms: int, publish_rate_hz: float) -> int`
  - publish_rate_hz に基づき、ミリ秒を「サンプル遅延 tick」へ変換
  - 境界値（0ms、1tick相当未満、非常に大きいms）を定義する

- `compute_delay_ticks(delay_ms: int, jitter_ms: int, publish_rate_hz: float, rng: random.Random) -> int`
  - `delay_ms + U[0, jitter_ms]` を ticks に変換して返す

- `should_drop(drop_rate: float, rng: random.Random) -> bool`
  - drop_rate に基づき、drop するかを返す（0.0=never, 1.0=always）

※ `fault_injection.py` は rclpy に依存しない（pytest を軽くするため）。

### `vital_sensor` の実装方針

`vital_sensor` に Day10 params を declare し、`publish_vital_data()` の前後に注入を適用する。

#### パラメータ（すべて default は無効）

- `vitals_fault_drop_rate`（float, default `0.0`）
- `vitals_fault_delay_ms`（int, default `0`）
- `vitals_fault_jitter_ms`（int, default `0`）
- `vitals_fault_pause_after_sec`（float, default `0.0`）
- `vitals_fault_pause_duration_sec`（float, default `0.0`）
- `vitals_fault_stop_after_sec`（float, default `0.0`）
- `vitals_fault_seed`（int, default `0`）

#### drop

- `should_drop(drop_rate, rng)` が true の tick では publish しない

#### delay + jitter（ブロッキング禁止）

- 遅延は `sleep` ではなく「due_tick を持つキュー」で表現する
- 各 tick で msg を生成し、`delay_ticks` を計算して `(due_tick, msg)` を push
- due_tick 到達済みの msg を publish（複数件 publish される場合もある）

#### pause

- 起動から `pause_after_sec` 経過後、`pause_duration_sec` の間は publish を止める
- pause は drop より優先度が高い（pause 中は常に publish しない）

#### stop（擬似停止）

- 起動から `stop_after_sec` 経過後、例外スタックトレースを出さずにプロセスが終了する
- 実装候補:
  - `rclpy.shutdown()` を呼び、main loop を抜ける
  - または `SystemExit(0)` / `SystemExit(2)`（0 推奨）

### launch への追加

`icu_multi_patient.launch.py` に Day10 launch arg を追加し、患者ごとの `vital_sensor` params として渡す。

- `vitals_fault_drop_rate`
- `vitals_fault_delay_ms`
- `vitals_fault_jitter_ms`
- `vitals_fault_pause_after_sec`
- `vitals_fault_pause_duration_sec`
- `vitals_fault_stop_after_sec`
- `vitals_fault_seed`

### テスト方針

pytest（`src/medical_robot_sim/test/`）で `fault_injection.py` を中心にユニットテストを追加する。

- `ms_to_ticks` の境界値
- `validate_fault_params` が範囲外で `ValueError`
- `should_drop` の端（0.0, 1.0）と seed 固定の再現性
- `compute_delay_ticks` の jitter の範囲（0〜jitter のどこか）

（統合テストは Day10 の必須外）

## Affected files

- `src/medical_robot_sim/medical_robot_sim/vital_sensor.py`
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`
- `src/medical_robot_sim/medical_robot_sim/fault_injection.py`（新規）
- `src/medical_robot_sim/test/test_fault_injection.py`（新規）
- `docs/day10_fault_injection.md`

## Constraints

- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）と message 定義は変更しない
- 既存の Day5-9 の挙動（alert ルール、QoS、Lifecycle）を壊さない
- Ctrl+C による停止でスタックトレースを出さない（注入有無に関わらず）
- 乱数を使う場合は seed で再現可能にする

## Non-goals

- ネットワーク層（iptables/tc 等）による OS レベルの障害注入
- 新しい topic 追加や topic 構造の再設計
- launch_testing など重い統合テスト基盤の導入
- alerts 側の fault injection（Day10 の必須対象外。必要なら後続 Day で拡張）
- 医療デバイス準拠の故障分類やリスク管理（本リポジトリの非目標）
