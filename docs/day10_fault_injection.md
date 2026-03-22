# Day10: Fault Injection（day10_fault_injection）

## Purpose

Fault Injection（障害注入）を導入し、ROS2 多患者監視シミュレーションに対して **再現可能な通信/ノード障害** を意図的に起こせるようにする。

対象は最小限として、既存の topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）を崩さずに次を達成する:

- 通信遅延/ドロップ/一時停止/ノード停止（擬似）を **launch 引数（ROS params）だけで再現**できる
- 監視側（`icu_monitor`）で FRESH/STALE/NO DATA が観測でき、アラート側（`rule_alert_engine`）の挙動変化も切り分けられる
- Day8（QoS）/ Day9（Lifecycle）と組み合わせて、障害と QoS 不一致/状態遷移を分離して評価できる

## Background

これまでの Day で、システムの「通常系」を固めてきた:

- Day7: ルール外部化（YAML）により評価条件の再現性を確保
- Day8: QoS を明示化し、通信条件を設計として固定/切替可能に
- Day9: Lifecycle Node により稼働状態（inactive/active）の切替を導入

一方で、現実の医療監視（および Physical AI の入力系）では **正常に動いている時間よりも、異常が起きたときの挙動** が重要になる。

- Wi-Fi 混雑や CPU 負荷により、センサ更新が一時的に途切れる/遅れる
- ノードが落ちて再起動する（もしくは意図的に停止/復帰させる）
- 通信は生きているがデータが来ない、などの「曖昧な故障」が起こる

Day10 では、これらを「たまたま起きる現象」ではなく、**設計された条件として再現**できるようにする。

## Why this day matters in the roadmap

- Day11（Logging/Tracing）で原因追跡の材料を整備する前に、まず「再現できる障害」を用意する必要がある
- Day12（rosbag）で再現性検証をする際、障害パターンを固定できると実験が安定する

また Day8/Day9 によって、
- QoS 不一致
- Lifecycle の inactive
- 真の通信障害

を分離しやすい状態になった。Day10 はその上で「障害を注入→観測→復旧」を手順化する。

## Target topics/components

### 対象 topic（契約は変更しない）

- vitals: `/patient_XX/patient_vitals`（型: `medical_interfaces/msg/VitalSigns`）
- alerts: `/patient_XX/alerts`（型: `medical_interfaces/msg/Alert`）

### 対象ノード（最小スコープ）

- `vital_sensor`（vitals publish）
  - Fault Injection の主戦場（入力が欠落/遅延する状況を再現しやすい）

※ `rule_alert_engine`（classic/lifecycle）や `icu_monitor` は挙動観測の対象とするが、Day10 の必須変更対象は `vital_sensor` と launch 側に限定する。

## Design policy

- 後方互換優先: 既定値は「障害注入なし」で、従来と同じ挙動を維持
- 小さく増分: まずは vitals の drop/delay/pause/stop のみ（alerts 側の障害注入は非必須）
- 再現性: 乱数を使う場合は seed を param で固定可能にする
- 観測可能: `ros2 topic hz/echo/info --verbose` と `icu_monitor` の表示で影響が分かる
- Clean shutdown 維持: Ctrl+C でスタックトレースを出さない（注入の有無に関わらず）

## Implementation requirements

### 1) `vital_sensor` に fault injection params を追加

次の ROS params を `vital_sensor` に追加する（すべて既定は「無効」= 影響なし）:

- `vitals_fault_drop_rate`（float, default `0.0`）
  - 0.0〜1.0。確率で publish をスキップする

- `vitals_fault_delay_ms`（int, default `0`）
  - 生成したメッセージの publish を固定遅延させる（ミリ秒）

- `vitals_fault_jitter_ms`（int, default `0`）
  - `vitals_fault_delay_ms` に加算されるランダムな揺らぎ（0〜jitter）

- `vitals_fault_pause_after_sec`（float, default `0.0`）
  - 指定秒数経過後に、一時的に publish を止め始める（0.0 は無効）

- `vitals_fault_pause_duration_sec`（float, default `0.0`）
  - pause の継続秒数（pause_after が有効のときのみ使用）

- `vitals_fault_stop_after_sec`（float, default `0.0`）
  - 指定秒数経過後にノードを「擬似停止」させる（0.0 は無効）
  - 擬似停止は、例外スタックトレースを出さずにプロセスが終了する形を優先する

- `vitals_fault_seed`（int, default `0`）
  - drop/jitter の再現性用 seed（0 は「固定 seed=0」扱いでよい）

### 2) delay は「ブロッキング sleep」ではなく「サンプル遅延」で実装

Clean shutdown や executor の応答性を保つため、`time.sleep()` で callback をブロックするのではなく、
「publish 予定時刻（サンプル番号）」を持つキューで遅延を表現する。

- `publish_rate_hz` から period を導出し、`delay_ms/jitter_ms` を「何サンプル遅延させるか」に変換
- 各 tick で msg を生成し、(due_sample, msg) をキューへ push
- キュー先頭が due_sample を過ぎたら publish（0〜複数件）

この方式なら、delay/jitter の影響を再現しつつ executor を止めない。

### 3) launch 引数で切替可能にする

`icu_multi_patient.launch.py` に Day10 の launch arg を追加し、患者ごとの `vital_sensor` へ params を渡す。

- `vitals_fault_drop_rate`
- `vitals_fault_delay_ms`
- `vitals_fault_jitter_ms`
- `vitals_fault_pause_after_sec`
- `vitals_fault_pause_duration_sec`
- `vitals_fault_stop_after_sec`
- `vitals_fault_seed`

### 4) 自動テスト（最低1つ）

pytest（`src/medical_robot_sim/test/`）で、少なくとも「純粋関数」レベルのテストを追加する。

例:
- パラメータ検証（範囲外で `ValueError`）
- `delay_ms` + `publish_rate_hz` → delay_ticks の換算が境界値で期待通り
- seed 固定で drop 判定が再現される

（重い統合テストは Day10 の必須から外し、必要なら後続 Day で拡張する）

### 5) Docs/README 更新（最小）

- 本ドキュメントに「障害注入の再現手順（コマンド）」を追加する
- README は必要なら最小限で追記（省略可）

## Files expected to change

- `src/medical_robot_sim/medical_robot_sim/vital_sensor.py`
- `src/medical_robot_sim/launch/icu_multi_patient.launch.py`
- `src/medical_robot_sim/medical_robot_sim/fault_injection.py`（新規）
- `src/medical_robot_sim/test/test_fault_injection.py`（新規）
- `docs/day10_fault_injection.md`（本ファイル）
- `specs/day10_fault_injection/spec.md`
- `specs/day10_fault_injection/tasks.md`
- `specs/day10_fault_injection/acceptance.md`

## Reproduction / validation steps

### 0) 共通セットアップ

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### 1) baseline（障害注入なし）

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false
```

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /patient_01/patient_vitals
```

期待:
- publish_rate_hz 付近の頻度が観測できる

### 2) drop 注入（例: 80% drop）

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_drop_rate:=0.8 \
  vitals_fault_seed:=42
```

期待:
- `ros2 topic hz` の観測レートが下がる
- `icu_monitor` 側で STALE/NO DATA が増える（条件次第）

### 3) pause 注入（一定時間後に停止→復帰）

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_pause_after_sec:=5.0 \
  vitals_fault_pause_duration_sec:=6.0
```

期待:
- 起動から約 5 秒後に vitals が途切れ、約 6 秒後に再開する

### 4) stop 注入（擬似ノード停止）

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=false \
  vitals_fault_stop_after_sec:=5.0
```

期待:
- `vital_sensor` が約 5 秒で停止し、topic 更新が止まる

## Success criteria

- launch 引数だけで drop/delay/pause/stop の少なくとも 2 種類以上が再現できる
- 既定値では障害注入なし（後方互換）
- `ros2 topic hz` や `icu_monitor` 表示で影響が観測できる
- `colcon test --packages-select medical_robot_sim` が成功し、少なくとも fault injection のユニットテストが 1 つある
- Ctrl+C で clean shutdown を維持する

## Relevance to medical / BCI / Physical AI context

- 医療監視では「入力が途切れた」こと自体が重要なイベント（NO DATA）になり得る
- BCI/EEG のような高頻度入力ではドロップや遅延の影響が顕著で、監視ロジックの頑健性評価が必要
- Physical AI は入力遅延に弱いため、遅延/欠落条件下の設計判断（QoS/バッファ/復旧手順）を検証できる

## Connection to the next day

Day11（Logging / Tracing）では、Day10 の障害注入シナリオを「ログで追える」状態にする。

- どの fault params で注入したか
- いつ drop/pause/stop が発生したか
- 監視側がいつ STALE/NO DATA と判断したか

を観測可能にし、Day12（rosbag）で「障害の記録→再生」へつなげる。
