# Day5: Alert Rules（判定種別 / 優先度 / cooldown / 入出力 topic）

本ドキュメントは、複数患者のバイタル監視における「アラート（警告）ルール」の整理（判定種別・優先度・通知抑制）をまとめたものです。

## 入出力 topic（明記）

### 入力（vitals）

- `/{patient_ns}/patient_vitals`
  - 型: `medical_interfaces/msg/VitalSigns`
  - 例: `/patient_01/patient_vitals`
  - 補足: `vital_sensor` は namespace 配下で相対 topic `patient_vitals` に publish するため、実体は上記に解決されます。

### 出力（alerts）

- **現状（このリポジトリの実装）**
  - `/{patient_ns}/alerts`
    - 型: `medical_interfaces/msg/Alert`
    - 例: `/patient_01/alerts`
  - 補足: `rule_alert_engine` が publish します（`icu_multi_patient.launch.py` の `enable_alerts:=true` で起動）。

## Day5 再現手順（起動→echo→bag record→bag info）

以降のコマンド例は Linux / ROS 2 Humble を想定します。

### 1) 起動（alerts 有効）

別ターミナルで以下を実行します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# 例: 2患者で起動、alerts を有効化
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01,patient_02 \
  enable_alerts:=true
```

### 2) echo（alerts を確認）

ここでは **alerts が確実に 1 件以上流れる**ことを確認します。

まず、患者ごとに「しきい値を下回る vitals（SpO2<90）」を 1 回だけ手動 publish して、アラートを確実に発火させます。

```bash
# 患者1: 低SpO2を手動投入（single.spo2_lt_90 を誘発）
ros2 topic pub --once /patient_01/patient_vitals medical_interfaces/msg/VitalSigns \
  "{patient_id: 'patient_01', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"

# 患者2も同様（patients に含めた場合）
ros2 topic pub --once /patient_02/patient_vitals medical_interfaces/msg/VitalSigns \
  "{patient_id: 'patient_02', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"
```

次に `--once` で 1 件受信できることを確認します。

```bash
# 患者1のalertsを1件だけ受信
ros2 topic echo /patient_01/alerts --once
```

確認観点（例）:

- `ros2 topic echo` が 1 件受信して終了する（= alerts が実際に流れている）
- topic名が `/patient_01/alerts` である（namespaceが反映されている）
- 型が `medical_interfaces/msg/Alert` である

### 3) bag record（vitals + alerts を記録）

起動中に別ターミナルで記録します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash

OUT_DIR="bags/day5_alerts_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$OUT_DIR" \
  /patient_01/patient_vitals /patient_02/patient_vitals \
  /patient_01/alerts /patient_02/alerts
```

確認観点（記録中）:

- 少なくとも 10 秒以上記録する（vitalsのcount>0を確実にするため）
- 記録中に、上と同じ手動 publish を 1 回以上実行する（alertsのcount>0を確実にするため）

記録を止めるときは `Ctrl+C`。

### 4) bag info（記録内容の確認）

```bash
ros2 bag info "$OUT_DIR"
```

確認ポイント（例）:

- `/patient_XX/patient_vitals` と `/patient_XX/alerts` が存在する
- 型がそれぞれ `medical_interfaces/msg/VitalSigns` と `medical_interfaces/msg/Alert`
- topic 数が期待通り（patients が N 人なら **2×N topic**）
- 各 topic の `Message Count` が **0 より大きい**

## Day5 の再現の定義（合格条件）

最低限、以下を満たせば Day5 の再現として合格とします。

- 起動コマンド（`enable_alerts:=true`）で `rule_alert_engine` が起動し、`/patient_XX/alerts`（型: `medical_interfaces/msg/Alert`）が生成される
- `ros2 topic echo /patient_01/alerts --once` が 1 件以上受信できる
- `ros2 bag record` で以下を同一 bag に記録できる（patients が N 人なら **2×N topic**）
  - `/patient_XX/patient_vitals`（型: `medical_interfaces/msg/VitalSigns`）
  - `/patient_XX/alerts`（型: `medical_interfaces/msg/Alert`）
- `ros2 bag info` で上記 topic と型が確認でき、**各 topic の Message Count が 0 より大きい**

追加の確認（任意）:

- `enable_alerts:=false` で起動したとき、`rule_alert_engine` は起動せず、`/patient_XX/alerts` も生成されない

## 優先度（priority）

本 workspace では、最終的な表示/通知を次の優先度で決めます。

### 1) データ鮮度（最優先）

`icu_monitor` は各患者の最終受信からの経過で、以下を先に決めます。

- `NO DATA`（未受信、または最終受信から 10 秒超）
- `STALE`（最終受信から 3 秒超）
- `FRESH`（上記以外）

`NO DATA` / `STALE` のときは、vitals 由来のアラートより優先して表示します。

### 2) Alert ラベルの優先度

`FRESH` のとき、アラートは次の順で強いものを採用します。

`OK` < `YELLOW` < `ORANGE` < `RED`

- 単発しきい値（生値）
  - `SpO2 < 90` → `RED`
  - `HR > 120` → `ORANGE`
  - `SBP > 160` → `YELLOW`
- イベント（変化率/組合せ）のレベル（`rule_evaluator`）
  - `CRITICAL` → `RED`
  - `WARN` → `YELLOW`
  - `OK` → `OK`

最終決定は「イベント優先」しつつ「生値しきい値で出ている強いアラートは下げない」方針です。

## cooldown（通知抑制）

同一患者・同一ルールが短時間に連続発火するとノイズになるため、**通知（イベント publish）を抑制**するための `cooldown_sec` をルールに持たせます。

### 仕様

- 単位: 秒
- key: `(patient_id, rule_id)`
- 動作:
  - ルールが発火した時刻を `last_emit_ts` として保持
  - 次に同じルールが発火しても、`now - last_emit_ts < cooldown_sec` の間は publish しない
  - ただし **重大度が上がる場合**（例: `YELLOW`→`RED`）は cooldown をバイパスして即 publish して良い
  - ルールが解消（正常側）したら、次の発火は即時に publish して良い

### デフォルト（目安）

- `RED`: 5s
- `ORANGE`: 10s
- `YELLOW`: 30s
- `INFO`: 60s

補足: 現状の `anomaly_detector` は「状態が active に遷移した瞬間のみイベント化（エッジトリガ）」しており、結果として“自然な cooldown”になります。上記は topic 通知を追加した場合の明文化です。

## 判定種別（4カテゴリ）

ここでは、判定ロジックを **単発 / 特異値 / 組合せ / 変化率** の4つに分類し、各カテゴリにつき具体例を2件ずつ示します。

### 1) 単発（single-shot / しきい値）

- 定義: **1サンプル（単発の VitalSigns）だけ**で判定できるルール
- 特徴: 実装が簡単・説明しやすいが、個体差・ノイズに弱い

例1: 低酸素（即時危険）

```yaml
rule_id: single.spo2_lt_90
kind: single
when: oxygen_saturation < 90
priority: RED
cooldown_sec: 5
window_sec: 0
input_topic: /{patient_ns}/patient_vitals
output_topic: /{patient_ns}/alerts
message: "SpO2 が 90% 未満"
```

例2: 高血圧（注意）

```yaml
rule_id: single.sbp_gt_160
kind: single
when: blood_pressure_systolic > 160
priority: YELLOW
cooldown_sec: 30
window_sec: 0
input_topic: /{patient_ns}/patient_vitals
output_topic: /{patient_ns}/alerts
message: "収縮期血圧(SBP)が 160mmHg 超"
```

### 2) 特異値（sanity / outlier）

- 定義: 生理的にあり得ない・整合しない値、またはセンサ/入力の破綻を疑う値
- 特徴: “患者の危険”というより **データ品質の異常**を早期に検出できる

例1: 血圧の整合性崩れ（DBP >= SBP）

```yaml
rule_id: outlier.dbp_ge_sbp
kind: outlier
when: blood_pressure_diastolic >= blood_pressure_systolic
priority: ORANGE
cooldown_sec: 10
window_sec: 0
input_topic: /{patient_ns}/patient_vitals
output_topic: /{patient_ns}/alerts
message: "血圧の整合性異常（DBP>=SBP）"
```

例2: 体温が現実的レンジ外（センサ異常疑い）

```yaml
rule_id: outlier.temp_out_of_range
kind: outlier
when: body_temperature < 30.0 OR body_temperature > 42.0
priority: ORANGE
cooldown_sec: 10
window_sec: 0
input_topic: /{patient_ns}/patient_vitals
output_topic: /{patient_ns}/alerts
message: "体温が現実的レンジ外（<30°C または >42°C）"
```

### 3) 組合せ（combination）

- 定義: 複数条件（複数イベント/複数しきい値）が揃ったときに、重大度を引き上げるルール
- 特徴: 単発では弱いシグナルでも、組み合わせで強い根拠になる

例1: `spo2_drop` と `hr_jump` の同時発生（実装に対応）

```yaml
rule_id: combo.spo2_drop_and_hr_jump
kind: combination
when: event_types contains [spo2_drop, hr_jump] within 10s
priority: RED
cooldown_sec: 5
window_sec: 10
input_topic: /{patient_ns}/patient_vitals
output_topic: /{patient_ns}/alerts
message: "SpO2 低下 + HR 急上昇（10秒窓）"
```

例2: 低血圧 + 頻脈（ショック疑い）

```yaml
rule_id: combo.hypotension_and_tachycardia
kind: combination
when: (blood_pressure_systolic < 90) AND (heart_rate > 120) within 30s
priority: ORANGE
cooldown_sec: 10
window_sec: 30
input_topic: /{patient_ns}/patient_vitals
output_topic: /{patient_ns}/alerts
message: "低血圧 + 頻脈（30秒窓）"
```

### 4) 変化率（rate-of-change）

- 定義: 直近の履歴（window）と比較して、**増減幅（delta）**で判定するルール
- 特徴: “絶対値”が正常でも、急変を捉えやすい

例1: SpO2 drop（直近 window での最大値からの低下）

```yaml
rule_id: roc.spo2_drop
kind: rate_of_change
window_sec: 10
baseline: max(SpO2 in window)
when: (SpO2_now - baseline) <= -4
priority: YELLOW
cooldown_sec: 30
input_topic: /{patient_ns}/patient_vitals
output_topic: /{patient_ns}/alerts
details:
  event_type: spo2_drop
  score: abs(delta) / 4
message: "SpO2 が 10秒で 4%以上低下"
```

例2: HR jump（直近 window での最小値からの上昇）

```yaml
rule_id: roc.hr_jump
kind: rate_of_change
window_sec: 10
baseline: min(HR in window)
when: (HR_now - baseline) >= 20
priority: YELLOW
cooldown_sec: 30
input_topic: /{patient_ns}/patient_vitals
output_topic: /{patient_ns}/alerts
details:
  event_type: hr_jump
  score: delta / 20
message: "HR が 10秒で 20bpm 以上上昇"
```

## 実装対応メモ（現在のコードとの対応）

- `medical_robot_sim/advisory_engine.py`
  - 生値しきい値（単発）とイベント（組合せ/変化率）の最大優先度を合成
- `medical_robot_sim/anomaly_detector.py`
  - 変化率イベント: `spo2_drop`, `hr_jump`
  - 変化の停止（flatline）: `flatline`（直近 window 内の変動幅が極小）
  - エッジトリガで二重カウントを避ける
- `medical_robot_sim/rule_evaluator.py`
  - `spo2_drop` + `hr_jump` → `CRITICAL`（`RED`）
  - 片方のみ → `WARN`（`YELLOW`）
