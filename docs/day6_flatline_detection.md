# Day6: flatline 検知（temporal_stability）導入

## 目的

バイタル値が一定時間にわたりほぼ変化しない状態を **flatline** と定義し、ROS2 のアラート pipeline に組み込む。

想定される異常:
- センサの固着（値が物理的に動かない）
- 患者の重篤な安定化（自発的な生理的揺らぎの消失）

既存の単発しきい値ルール（`single.*`）や変化率ルール（`roc.*`）では検出できない「**時間的な停滞**」を捉えるため、`temporal_stability` という独立したカテゴリとして設計した。

---

## 実装内容

### センサ側: `scenario=flatline`

`vital_sensor` に `scenario` パラメータを追加し、`flatline` を指定すると HR・SpO2 を固定値で publish し続ける。

```bash
# scenario=flatline のとき: HR=72 固定、SpO2=98 固定
-p scenario:=flatline
-p flatline_hr_value:=72
-p flatline_spo2_value:=98
```

これにより、ルールエンジン側の flatline 検知を再現可能な形でテストできる。

### ルール定義: `flatline.hr` / `flatline.spo2`

`alert_rules.py` の `evaluate_alert_rules()` に以下を追加:

| rule_id        | 対象フィールド      | 判定条件                                       |
|----------------|---------------------|------------------------------------------------|
| `flatline.hr`  | `heart_rate`        | 末尾 `flatline_history_size` 件の max−min ≤ `flatline_hr_epsilon` |
| `flatline.spo2`| `oxygen_saturation` | 末尾 `flatline_history_size` 件の max−min ≤ `flatline_spo2_epsilon` |

判定は純粋関数として実装し、ノードの状態に依存しない。履歴が `flatline_history_size` 件に満たない場合は `active=False`（判定保留）。

### アラート発報フロー

`rule_alert_engine` では `temporal_stability` 種別に**レベルトリガ + cooldown** を採用している。

```
active=True が継続している間、_TEMPORAL_STABILITY_COOLDOWN_SEC (10秒) ごとに再発火
```

エッジトリガ（`False → True` の1回のみ）では継続中の flatline を見逃すため、意図的にレベルトリガを選択した。

---

## 判定仕様

| フィールド  | 値                         |
|-------------|----------------------------|
| `rule_id`   | `flatline.hr` / `flatline.spo2` |
| `kind`      | `temporal_stability`       |
| `priority`  | `YELLOW`                   |
| `window_sec`| `8`（`flatline_history_size` デフォルト）|
| `field`     | `heart_rate` / `oxygen_saturation` |
| `delta`     | 窓内の max−min（flatline 時は `0.0`）|
| `score`     | `1.0`（active=True 時）/ `0.0`（active=False 時）|

デフォルトパラメータ:

| パラメータ              | デフォルト | 意味                            |
|------------------------|------------|---------------------------------|
| `flatline_history_size` | `8`        | 判定に使う末尾サンプル数        |
| `flatline_hr_epsilon`   | `1.0`      | HR の許容変動幅（bpm）          |
| `flatline_spo2_epsilon` | `1.0`      | SpO2 の許容変動幅（%）          |

---

## 再現手順

### 起動

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01,patient_02 \
  enable_alerts:=true \
  scenario:=flatline \
  enabled_rule_ids:=flatline.hr,flatline.spo2
```

`enabled_rule_ids:=flatline.hr,flatline.spo2` を指定することで、他のルールを無効にして flatline のみを検証できる。

### アラート確認（別ターミナル）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

timeout 45s ros2 topic echo /patient_01/alerts
```

> **Note (WSL2)**: タイマー制約で発火まで最大 40 秒かかる場合は、起動コマンドに `flatline_history_size:=3` を追加してください（発火までのサンプル数を減らします）。

---

## 成功条件

| 確認項目                                              | 確認方法                              |
|-------------------------------------------------------|---------------------------------------|
| `/patient_01/alerts` に `flatline.hr` / `flatline.spo2` が出る | `ros2 topic echo /patient_01/alerts`  |
| `kind: temporal_stability`                            | 同上                                  |
| `window_sec: 8`                                       | 同上                                  |
| Publishers が 1 であること（二重起動がない）          | `ros2 topic info /patient_01/alerts`  |
| `colcon test` が 0 failures                           | `colcon test && colcon test-result`   |

期待出力例（抜粋）:

```text
rule_id: flatline.hr
kind: temporal_stability
priority: YELLOW
window_sec: 8
field: heart_rate
delta: 0.0
score: 1.0
```

---

## BCI 統合に向けた意義

flatline 検知の `temporal_stability` カテゴリは、将来の BCI（Brain-Computer Interface）パイプラインにおける以下のユースケースの原型となる。

- **信号消失の検知**: EEG・MEG などの脳波信号が完全に途絶えた状態の検出
- **電極接触不良の検知**: 特定チャンネルの値が固着（non-varying）になる場合のフォールト検出
- **低活動状態の検知**: 麻酔深度・睡眠段階・意識レベルの低下による信号変動抑制の検出

単発しきい値ルールでは「値が異常範囲に入った時」しか検知できないが、`temporal_stability` は「**値が動かなくなった時**」を検知する点で相補的な役割を果たす。

---

## 今後の課題

- **パラメータ外部化（Day7）**: `flatline_history_size` / `epsilon` は現在コードのデフォルト値か launch 引数でしか変更できない。YAML で管理することでコード変更なしにチューニングできるようにする
- **QoS 設計**: アラートトピックの QoS（`RELIABLE` / `BEST_EFFORT`、`TRANSIENT_LOCAL` など）を明示的に定義する
- **Lifecycle / Fail-safe 接続**: `vital_sensor` が停止した場合に `rule_alert_engine` 側でタイムアウト検知を行う仕組み（Lifecycle Node や Watchdog パターン）の検討
- **複数フィールドの連成 flatline**: HR と SpO2 が同時に固着している場合に優先度を上げる combo ルールへの発展

---

## 関連ファイル

| ファイル                                                              | 変更内容                                                  |
|-----------------------------------------------------------------------|-----------------------------------------------------------|
| `src/medical_robot_sim/medical_robot_sim/alert_rules.py`              | `RuleParams` に flatline パラメータ追加、`flatline.hr` / `flatline.spo2` ルール追加 |
| `src/medical_robot_sim/medical_robot_sim/rule_alert_engine.py`        | flatline パラメータ受け取り・`temporal_stability` 向けレベルトリガ実装 |
| `src/medical_robot_sim/medical_robot_sim/vital_sensor.py`             | `scenario=flatline`、`flatline_hr_value` / `flatline_spo2_value` パラメータ追加 |
| `src/medical_robot_sim/launch/icu_multi_patient.launch.py`            | `enable_alerts` デフォルト `true`、flatline 関連 launch 引数追加 |
| `src/medical_robot_sim/test/test_alert_rules.py`                      | flatline ルールのテスト 7 件追加（`window_sec=8` アサーション含む）|
| `scripts/repro_day6_flatline.sh`                                      | Day6 再現スクリプト（新規）                               |
| `docs/day6_flatline.md`                                               | 設計・仕様・再現手順（詳細版）                            |
