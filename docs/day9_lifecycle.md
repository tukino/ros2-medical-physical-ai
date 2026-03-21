# Day9: Lifecycle Node導入（day9_lifecycle）

## Purpose

ROS 2 の Lifecycle Node を導入し、ノードの状態（未設定/停止/稼働）を **明示的に制御**できるようにする。

この Day では対象を最小限に絞り、既存の topic 契約（/patient_XX/patient_vitals, /patient_XX/alerts）と、Day3 の clean shutdown（Ctrl+C でスタックトレースなし）を維持したまま、次を達成する。

- 監視/アラート系ノードを「起動したがまだ動かさない」「一時停止」「再開」できる
- `ros2 lifecycle` で状態遷移を観測・操作できる
- Lifecycle を使っても、従来の起動手順（launch 1発）を壊さない（後方互換）

## Background

現状のノードは `rclpy.node.Node` として起動し、起動直後から publisher/subscription/timer が動き始める。

- 再初期化や一時停止をしたい場合、いったんプロセスを落として再起動するしかない
- Day10（Fault Injection）以降で「ノード停止→復帰」「設定変更→再適用」を扱う際、プロセス再起動だけだと切り分けが難しい
- 安全性の観点でも、構成（configure）と稼働（activate）を分離できるのは重要

[docs/PLAN.md](docs/PLAN.md) の Phase2 で Day9 が Lifecycle Node とされているため、このタイミングで「状態管理できるノード」という設計要素を導入する。

## Why this day matters in the roadmap

- Day8（QoS設計）で通信品質を仕様として固定・切替可能にした
- Day9（Lifecycle）で「稼働状態」を制御できるようにする
- Day10（Fault Injection）で障害を注入し、**停止→復帰**や**復旧手順の再現性**を評価する

Lifecycle を先に入れておくことで、以降の Day で
- 「通信の問題（QoS）なのか」
- 「ノードが inactive なだけなのか」
- 「内部状態が壊れているのか」
を分離しやすくなる。

## Target topics/components

### 対象 topic（契約は変更しない）

- vitals: `/patient_XX/patient_vitals`（型: `medical_interfaces/msg/VitalSigns`）
- alerts: `/patient_XX/alerts`（型: `medical_interfaces/msg/Alert`）

### 対象ノード（最小スコープ）

- `rule_alert_engine` を Lifecycle 対応する
  - vitals subscribe / alerts publish を持ち、停止/再開の価値が最も大きい

※ `vital_sensor` と `icu_monitor` は Day9 の必須対象から外す（必要なら後続 Day で拡張）。

## Design policy

- 後方互換優先:
  - 従来通り `ros2 launch ... enable_alerts:=true` だけで alerts が流れる既定動作を維持
- 小さく増分:
  - Lifecycle 対応は `rule_alert_engine` のみに限定
- 観測可能:
  - `ros2 lifecycle get/set` と `ros2 topic echo/info` で状態差を確認できる
- 安全な停止/再開:
  - deactivate で publisher/subscription/timer を止め、activate で再開できる
- Clean shutdown 維持:
  - Ctrl+C で例外スタックトレースを出さない（Day3 の原則を維持）

## Implementation requirements

### 1) Lifecycle 版 alert engine を追加

- 新規 executable `rule_alert_engine_lifecycle` を追加し、ノード名は `rule_alert_engine` を維持する
- クラスは `rclpy.lifecycle.LifecycleNode` を使用
- 既存 `rule_alert_engine`（classic）は維持し、launch で切替できるようにする

### 2) 起動互換（autostart）

従来の launch 1発運用を壊さないため、Lifecycle 版は **既定で自動遷移**する。

- parameter: `lifecycle_autostart`（bool, default `true`）
  - true: 起動後に `configure -> activate` を自動で実行（従来と同等の体験）
  - false: `inactive` で待機し、オペレータが `ros2 lifecycle set` で遷移させる

### 3) 状態ごとの動作

- `unconfigured`:
  - YAML/params の解釈は行わない（ログは最小）
  - pub/sub は作らない

- `inactive`（configured）:
  - パラメータ解釈（Day7/Day8 含む）と内部状態の初期化を完了
  - pub/sub は作成してもよいが、メッセージ処理・publish は行わない（推奨は pub/sub を作らず active で作る）

- `active`:
  - vitals を購読し、ルール評価して alerts を publish

- `deactivate`:
  - pub/sub/timer を止め、必要なら内部バッファ（deque）もクリア

### 4) 既存パラメータ・topic の維持

Lifecycle 版でも、既存の `rule_alert_engine` が受け取る params をそのまま受け取れること。

- `patients`, `vitals_topic`, `alert_topic`
- Day7: `rules_path`, `enabled_rule_ids` 等
- Day8: `vitals_qos_*`, `alerts_qos_*`
- Day6/Day7: flatline 関連 params

### 5) launch 引数でノード種別を切替

[src/medical_robot_sim/launch/icu_multi_patient.launch.py](src/medical_robot_sim/launch/icu_multi_patient.launch.py) に次を追加する:

- `alerts_node_kind`（string, default `classic`）
  - `classic`: 既存 executable `rule_alert_engine`
  - `lifecycle`: 新規 executable `rule_alert_engine_lifecycle`

加えて lifecycle 側へ渡す:
- `lifecycle_autostart`（bool, default `true`）

### 6) 自動テスト（最低1つ）

[src/medical_robot_sim/test/](src/medical_robot_sim/test/) に pytest を追加し、少なくとも以下を検証する:

- `lifecycle_autostart=false` のとき、未 activate では alerts を publish しない（内部フラグでも可）
- `configure -> activate -> deactivate` の遷移で例外が出ない

（可能なら）
- `lifecycle_autostart=true` で `active` まで到達する

※ テストは「純粋関数」だけで完結しなくてよいが、既存のテスト構成（pytest）に合わせ、過度に重い launch_testing は Day9 では避ける。

## Files expected to change

- [src/medical_robot_sim/medical_robot_sim/rule_alert_engine_lifecycle.py](src/medical_robot_sim/medical_robot_sim/rule_alert_engine_lifecycle.py)（新規）
- [src/medical_robot_sim/setup.py](src/medical_robot_sim/setup.py)（console_scripts 追加）
- [src/medical_robot_sim/launch/icu_multi_patient.launch.py](src/medical_robot_sim/launch/icu_multi_patient.launch.py)（alerts_node_kind 追加）
- [src/medical_robot_sim/test/test_rule_alert_engine_lifecycle.py](src/medical_robot_sim/test/test_rule_alert_engine_lifecycle.py)（新規）
- [docs/day9_lifecycle.md](docs/day9_lifecycle.md)（本ファイル）
- [specs/day9_lifecycle/spec.md](specs/day9_lifecycle/spec.md) / [specs/day9_lifecycle/tasks.md](specs/day9_lifecycle/tasks.md) / [specs/day9_lifecycle/acceptance.md](specs/day9_lifecycle/acceptance.md)

## Reproduction / validation steps

### 0) 前提

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### 1) 後方互換（classic）の確認

従来通り起動し、alerts が流れること:

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  enabled_rule_ids:=single.spo2_lt_90
```

別ターミナルで、手動で 1 回だけ低SpO2を投入:

```bash
# alerts は Durability=volatile が既定のため、先に待ち受けて取り逃がしを防ぐ
timeout 10s ros2 topic echo /patient_01/alerts --once &
sleep 0.2

# edge-trigger のため、まず正常値→異常値の順に投入して「False→True」を作る
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns \
  "{patient_id: 'patient_01', measurement_id: 998, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 95, status: 'monitoring'}"
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns \
  "{patient_id: 'patient_01', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"

wait
```

### 2) Lifecycle モード（手動遷移）

起動（unconfigured で待機）:

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  alerts_node_kind:=lifecycle \
  lifecycle_autostart:=false \
  enabled_rule_ids:=single.spo2_lt_90
```

状態確認:

```bash
ros2 lifecycle get /rule_alert_engine
```

configure/activate:

```bash
ros2 lifecycle set /rule_alert_engine configure
ros2 lifecycle set /rule_alert_engine activate
```

alerts が流れること（上と同じ手動 publish を使う）:

```bash
timeout 10s ros2 topic echo /patient_01/alerts --once &
sleep 0.2

ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns \
  "{patient_id: 'patient_01', measurement_id: 998, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 95, status: 'monitoring'}"
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns \
  "{patient_id: 'patient_01', measurement_id: 1000, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"

wait
```

deactivate 後に止まること:

```bash
ros2 lifecycle set /rule_alert_engine deactivate

# 5秒待っても alerts が来ないこと（来れば失敗）
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns \
  "{patient_id: 'patient_01', measurement_id: 1001, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"

timeout 5s ros2 topic echo /patient_01/alerts --once
```

### 3) Clean shutdown

どのモードでも `Ctrl+C` で停止し、Python の例外スタックトレースが出ないこと。

## Success criteria

- `rule_alert_engine` を lifecycle として起動でき、`ros2 lifecycle get/set` で状態操作できる
- inactive/deactivated では alerts が publish されず、active でのみ publish される
- `alerts_node_kind:=classic`（既定）では従来通り動作する（後方互換）
- `colcon test --packages-select medical_robot_sim` が成功する（pytest 追加含む）
- topic 契約（名前/型）を破壊しない
- Ctrl+C で clean shutdown を維持する

## Relevance to medical / BCI / Physical AI context

- 医療監視では「動いている/止まっている/設定中」を曖昧にしないことが重要
- BCI/EEG のような高頻度入力でも、稼働状態を分離すると安全に再設定できる
- Physical AI はセンサ入力に依存するため、センサ/推論/通知の稼働状態を段階的に制御できると、現場での運用と障害対応が容易になる

## Connection to the next day

Day10（Fault Injection）では、
- ノード停止（deactivate/cleanup/shutdown 相当）
- 例外/通信断の注入
- そこからの復帰（configure/activate）

を「再現可能な手順」として評価する。

Day9 で Lifecycle を導入しておくことで、Fault Injection の復旧手順が launch 再起動に依存せず、より細かく検証できる。
