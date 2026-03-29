# Day15: EEG / BCI入力（day15_eeg_bci_input）

## Purpose

本Dayの目的は、脳波（EEG）/BCI のような「人間状態」由来の信号を、既存の multi-patient 監視パイプラインと同じ思想（topic契約、再現性、観測可能性、クリーン停止）で **ROS2入力として取り込める最小構成**を設計・実装できる状態にすること。

- 患者 namespace で `/patient_XX/patient_bci` に publish できる
- 実機が無い環境でも **mock driver** で受け入れが完走する（再現性）
- 高頻度・ノイズを前提に、QoS / publish_rate を設計として固定できる
- Ctrl+C で clean shutdown（スタックトレース無し）を維持する

本Dayでは **vitals/alerts の契約は変更しない**。BCI入力は「追加の入力ストリーム」として扱い、既存の `/patient_XX/patient_vitals` と `/patient_XX/alerts` はそのまま残す。

## Background

BCI/EEG は、vitals と違って次の特徴を持つ:

- 入力が高頻度（例: 100–250Hz）で帯域が大きい
- ノイズ（artifact）や欠測が常に混入し、デバイス差も大きい
- 実運用では Edge（Day14）側で取得し、rosbag（Day12）で再現・解析する流れになりやすい

このため Day15 では「生信号処理の高度化」よりも先に、
**入力契約（topic/message）と運用（mock/optional実機・ログ・停止）を固める**ことを優先する。

## Why this day matters in the roadmap

- Day14（Jetson）: Edge 側で新しい入力デバイスを扱う前提を作る
- Day12（rosbag）: BCI/EEG 入力を記録・再生し、オフラインで同じ条件を作れる
- Day16（異常検知AI）: ルールベースの外側に AI を載せる場合でも、まず入力が契約化されている必要がある

## Target topics/components

### 対象 topic（既存契約は維持）

- vitals: `/patient_XX/patient_vitals`（`medical_interfaces/msg/VitalSigns`）
- alerts: `/patient_XX/alerts`（`medical_interfaces/msg/Alert`）

### 追加する topic（Day15）

- bci: `/patient_XX/patient_bci`（`medical_interfaces/msg/BCIFeatures`）

本Dayで追加するのは BCI 入力 topic のみ。既存 topic の rename / restructure はしない。

### 対象コンポーネント

- 新規: `bci_sensor`（BCI入力ノード。mock/optional実機ドライバを切替）
- 新規: `bci_monitor`（BCI入力の鮮度・簡易状態を観測するノード。UIは作らずログ中心）
- 新規: `icu_bci.launch.py`（最短起動入口。受け入れが 1 コマンドで回ることを優先）
- 既存（参照のみ）:
  - Day8 QoSユーティリティ: `medical_robot_sim/qos_profiles.py`
  - Day11 イベントログ: `medical_robot_sim/observability.py`

## Design policy

- **後方互換**: vitals/alerts の topic契約と message定義は変更しない
- **増分**: Day15 の必須は「BCI入力 topic が流れ、観測でき、停止できる」まで
- **実機依存を隔離**: 実デバイスI/Oは driver 層へ閉じ込め、Node は薄くする
- **再現性**: mock は seed 固定で deterministic（受け入れ/CI を安定化）
- **観測可能**: Day11 形式の `event=... key=value` で、設定・読取・状態遷移を grep できる
- **クリーン停止**: Ctrl+C/SIGINT でスタックトレース無し

## Implementation requirements

### 1) Message 定義（必須）

`medical_interfaces/msg/BCIFeatures.msg` を追加する。

目的は「生EEG波形」ではなく、下流が使いやすい **特徴量（features）** を最小セットで運ぶこと。

設計指針:
- multi-patient と整合するため `patient_id` を含む
- 再現性のため `measurement_id`（単調増加）を含む
- 欠測や無効値は `float32` の NaN で表現する（Alert.msg の方針に合わせる）

（例）フィールド案:
- `string patient_id`
- `uint32 measurement_id`
- `float32 attention`（0..1 を想定）
- `float32 drowsiness`（0..1 を想定）
- `float32 signal_quality`（0..1 を想定）
- `string status`（ok/degraded/no_signal など）

### 2) `bci_sensor`（必須）

- publish: 相対 topic `patient_bci`（namespace により `/patient_XX/patient_bci` に解決）
- parameter（最低限）:
  - `patient_id`（例: `patient_01`）
  - `publish_rate_hz`（mock の publish 周期。既定は 10Hz 程度）
  - `driver`（`mock` / `serial` 等。必須受け入れは `mock`）
  - `mock_scenario`（例: `normal` / `drowsy` / `artifact_spike`）
  - `seed`（mock の再現性用）
  - Day8 QoS: `bci_qos_depth`, `bci_qos_reliability`, `bci_qos_durability`

実機 driver を入れる場合の例:
- `serial_port`（例: `/dev/ttyUSB0`）

実機 driver は optional とし、未導入・デバイス無しでも受け入れは落とさない。

### 3) `bci_monitor`（必須）

- subscribe: `/{pid}/{bci_topic}`（既定 `patient_bci`）
- parameter:
  - `patients`（string array）
  - `bci_topic`（string, default `patient_bci`）
  - Day8 QoS: `bci_qos_*`

最小の観測要件:
- 患者ごとに last_seen を保持し、FRESH/STALE/NO DATA を算出してイベントログ化する
  - vitals での `monitor.patient_state` と同様の設計（出力は `bci.patient_state`）

### 4) Observability（Day11 形式を踏襲）

最低限、次のイベントを 1行ログで出す（grep 可能）。

- `bci.device_config`（起動時）: driver / publish_rate / scenario / qos / seed
- `bci.device_open_ok` / `bci.device_open_fail`
- `bci.patient_state`（monitor 側の状態遷移）: pid / state / age_sec / last_measurement_id

### 5) Launch（必須）

`icu_bci.launch.py` を追加し、単一患者（既定 `patient_01`）で
- `bci_sensor`（namespace=patient）
- `bci_monitor`（root）
を起動できる。

既存 launch（例: `icu_multi_patient.launch.py`）は壊さない。

### 6) README方針への反映（設計時の前提）

README には詳細を重複させず、追記候補は最短導線だけに留める:
- 本ドキュメント
- 受け入れ手順（acceptance）

## Files expected to change

- `src/medical_interfaces/msg/BCIFeatures.msg`（新規）
- `src/medical_interfaces/CMakeLists.txt`（interfaces 追加）

- `src/medical_robot_sim/medical_robot_sim/bci_io.py`（新規: driver抽象 + mock + optional）
- `src/medical_robot_sim/medical_robot_sim/bci_patient_state.py`（新規: 状態分類 pure function）
- `src/medical_robot_sim/medical_robot_sim/bci_sensor.py`（新規: ROS2 node）
- `src/medical_robot_sim/medical_robot_sim/bci_monitor.py`（新規: 監視/状態ログ）
- `src/medical_robot_sim/launch/icu_bci.launch.py`（新規）
- `src/medical_robot_sim/setup.py`（entry_points 追加）

- `src/medical_robot_sim/test/test_bci_io_mock.py`（新規: deterministic / 境界値）
- `src/medical_robot_sim/test/test_bci_patient_state.py`（新規: 状態分類の純粋関数）

- 本ファイル: `docs/day15_eeg_bci_input.md`
- `specs/day15_eeg_bci_input/spec.md`
- `specs/day15_eeg_bci_input/tasks.md`
- `specs/day15_eeg_bci_input/acceptance.md`

## Reproduction / validation steps

- 受け入れ手順（コピペ）: `specs/day15_eeg_bci_input/acceptance.md`

### 最短コピペ（Quickstart: 5分で観測）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

rm -f /tmp/day15_quick.log
timeout -s INT 12s ros2 launch medical_robot_sim icu_bci.launch.py \
  patient:=patient_01 \
  driver:=mock \
  mock_scenario:=drowsy \
  publish_rate_hz:=10.0 \
  > /tmp/day15_quick.log 2>&1 || true

# 起動設定と状態遷移の痕跡（どれかが出る）
grep -n "event=bci\.device_config" /tmp/day15_quick.log || true
grep -n "event=bci\.patient_state" /tmp/day15_quick.log || true

echo "OK: day15 quickstart ran (see /tmp/day15_quick.log)"
```

合否判定は acceptance（topic echo による観測）に従う。

## Success criteria

- `medical_interfaces/msg/BCIFeatures` が生成され、`/patient_01/patient_bci` を 1 件以上観測できる
- mock driver で受け入れが完走する（実機無しで OK）
- `bci_monitor` が `bci.patient_state` をログに出せる
- `colcon test --packages-select medical_robot_sim` が成功し、BCI関連のユニットテストが少なくとも 1 つある
- Ctrl+C でスタックトレース無しに停止できる

## Learning path

### Quickstart（最短で動かす）

- Quickstart で `icu_bci.launch.py` を起動し、ログに `bci.device_config` / `bci.patient_state` が残ることを確認
- acceptance で `ros2 topic echo --once` により、topic が実際に流れていることを機械判定

### 読み方（原因追跡の順番）

1. `/tmp/day15_*.log` の `event=bci.device_*` を grep
   - driver / scenario / open 成否
2. `ros2 topic list | grep patient_bci`
   - topic が想定 namespace に生えているか
3. `ros2 topic echo /patient_01/patient_bci --once`
   - QoS 不一致/未接続の切り分け
4. `bci.patient_state` を確認
  - 受信できているのに state が NO DATA なら monitor 側の設定/患者IDが不一致

### 必須（受け入れ）と任意（学習）

- 必須: mock で publish + echo が成立する
- 任意: serial 等の実機 driver を接続し、同一 topic 契約で publish できる

## Relevance to medical / BCI / Physical AI context

- 医療・介護現場では、患者の「意識/眠気/負荷」などは行動・安全に直結し得る
- BCI/EEG は入力が不安定になりやすく、まず契約（topic/message）と観測・停止手順を固定化するのが重要
- Physical AI の入力は増え続けるため、vitals と同じ “運用可能な入力パターン（mock/optional実機）” を確立する価値がある

## Connection to the next day

- Day16（異常検知AI）: Day15 の BCI入力を bag 化し、オフラインで特徴量系列を学習/検知へ接続できる
- Day12（rosbag）: `/patient_XX/patient_bci` を記録・再生して、同条件での比較を可能にする
