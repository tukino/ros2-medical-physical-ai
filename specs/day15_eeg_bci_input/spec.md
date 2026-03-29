# Day15 EEG / BCI入力 実装仕様（spec）

## Goal

EEG/BCI の入力を multi-patient パイプラインへ追加し、次を満たす。

- `/patient_XX/patient_bci`（新規）の publish を実現する
- 実機が無い環境でも **mock driver で受け入れが完走**する（CI/再現性）
- 既存契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）は変更しない
- QoS / publish_rate / seed を parameter で外部化し、再現可能にする
- Ctrl+C の clean shutdown を維持する（スタックトレース無し）

## Design

### 追加する topic と message

- 新規 topic: `/patient_XX/patient_bci`
- 新規 message: `medical_interfaces/msg/BCIFeatures`

`BCIFeatures` は「生波形」ではなく特徴量（features）を運ぶ最小メッセージとする。

推奨フィールド（最小）:
- `string patient_id`
- `uint32 measurement_id`
- `float32 attention`（0..1）
- `float32 drowsiness`（0..1）
- `float32 signal_quality`（0..1）
- `string status`（ok/degraded/no_signal）

欠測は NaN で表現する（float32）。

### ノード構成

#### 1) `bci_sensor`（入力ノード）

- publish: 相対 topic `patient_bci`（namespace = 患者IDで `/patient_XX/patient_bci` に解決）
- パラメータ（必須）:
  - `patient_id`（string）
  - `publish_rate_hz`（float）
  - `driver`（string: `mock` を必須、他は optional）
  - `mock_scenario`（string: `normal` / `drowsy` / `artifact_spike`）
  - `seed`（int: mock の再現性）
  - Day8 形式の QoS:
    - `bci_qos_depth`（int, default 10）
    - `bci_qos_reliability`（string, default `reliable`）
    - `bci_qos_durability`（string, default `volatile`）

実機 driver は optional:
- 例: `serial`（pyserial 等）
- `driver=serial` の場合は `serial_port`（例: `/dev/ttyUSB0`）をパラメータで受け取れること
- 受け入れは mock だけで完走すること（実機無しで OK）

#### 2) `bci_monitor`（観測ノード）

- subscribe: `/{pid}/{bci_topic}`
- パラメータ:
  - `patients`（string array）
  - `bci_topic`（string, default `patient_bci`）
  - `bci_qos_*`（sensor と同名）

機能:
- 患者ごとに last_seen と last_measurement_id を保持する
- FRESH/STALE/NO DATA を算出し、状態遷移のみをイベントログで出す

状態分類は pure function を切り出して pytest でテスト可能にする（Day11 の `classify_patient_state` と同様）。

### driver 分離（環境差を吸収）

実デバイス I/O を Node から切り離す。

- `bci_io.py`
  - `BCIDevice` I/F（例: `open()` / `read_features()` / `close()`）
  - `MockBCIDevice`（deterministic）
  - `SerialBCIDevice`（optional。未導入/デバイス無しは明確なエラーにし、受け入れは N/A）

### Observability（Day11 形式）

最低限、次のイベントを `format_event()` で 1行ログ化する。

- `bci.device_config`（起動時）
- `bci.device_open_ok` / `bci.device_open_fail`
- `bci.patient_state`（monitor の状態遷移）

### Launch

新規: `icu_bci.launch.py`

- 目的: 受け入れが 1 コマンドで完走する入口
- 起動構成（最小）:
  - `bci_sensor`（namespace=patient）
  - `bci_monitor`（root）

既存 launch は変更最小（原則: 無変更）。

任意: `driver=serial` の場合に備え、`serial_port`（string）を launch arg として受け取れるようにして良い（実機が無い環境では使わない）。

### テスト方針

pytest（`src/medical_robot_sim/test/`）で純粋関数と mock を中心に最小テストを追加する。

- `MockBCIDevice` が seed 固定で deterministic に系列を返す
- state 分類（age_sec → FRESH/STALE/NO DATA）の境界値

## Affected files

- `src/medical_interfaces/msg/BCIFeatures.msg`（新規）
- `src/medical_interfaces/CMakeLists.txt`（interfaces 追加）

- `src/medical_robot_sim/medical_robot_sim/bci_io.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/bci_patient_state.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/bci_sensor.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/bci_monitor.py`（新規）
- `src/medical_robot_sim/launch/icu_bci.launch.py`（新規）
- `src/medical_robot_sim/setup.py`（entry_points 追加）

- `src/medical_robot_sim/test/test_bci_io_mock.py`（新規）
- `src/medical_robot_sim/test/test_bci_patient_state.py`（新規）

- `docs/day15_eeg_bci_input.md`
- `specs/day15_eeg_bci_input/spec.md`
- `specs/day15_eeg_bci_input/tasks.md`
- `specs/day15_eeg_bci_input/acceptance.md`

## Constraints

- 既存 topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）と message 定義は変更しない
- multi-patient の namespace 方式（`/patient_XX/...`）と整合する
- QoS は Day8 の文字列指定を踏襲し、不正値は `ValueError` / `SystemExit(2)` で明確に失敗
- 受け入れは mock driver で完走（実機無しで OK）
- Ctrl+C で clean shutdown（スタックトレース無し）

## Non-goals

- 生EEG波形の扱い（高周波サンプルをそのまま ROS2 message に流す）
- 高度な信号処理（フィルタ、ICA、周波数解析など）
- BCI入力から `/patient_XX/alerts` を直接発火するルール設計（本Day必須外）
- LLM ベース推論の導入（本リポジトリの現優先度外）
- launch_testing 等の重い統合テスト基盤の導入
