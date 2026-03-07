# Day6: 時間的安定性（flatline）検知の導入

## 設計意図

「通常のバイタルは適度に揺らぐ」という前提のもと、**一定時間にわたって値がほぼ変化しない状態**を異常（センサ固着 or 患者の重篤な状態）として検知します。これを **flatline 検知**と呼びます。

flatline は生値しきい値（単発）では検出できない「時間的な停滞」を捉えるため、既存の `roc.spo2_drop` / `roc.hr_jump` とは独立したカテゴリ `temporal_stability` として位置づけます。

---

## 追加ルール

### `flatline.hr`

| 項目         | 値                                        |
|--------------|-------------------------------------------|
| rule_id      | `flatline.hr`                             |
| kind         | `temporal_stability`                      |
| priority     | `YELLOW`                                  |
| window_sec   | `flatline_history_size`（パラメータ）      |
| 判定条件     | 末尾 `flatline_history_size` 件の HR の `max - min ≤ flatline_hr_epsilon` |
| field        | `heart_rate`                              |
| delta        | `max - min`（変動幅）                      |
| score        | 発火時 1.0 / 非発火時 0.0                 |

### `flatline.spo2`

| 項目         | 値                                        |
|--------------|-------------------------------------------|
| rule_id      | `flatline.spo2`                           |
| kind         | `temporal_stability`                      |
| priority     | `YELLOW`                                  |
| window_sec   | `flatline_history_size`（パラメータ）      |
| 判定条件     | 末尾 `flatline_history_size` 件の SpO2 の `max - min ≤ flatline_spo2_epsilon` |
| field        | `oxygen_saturation`                       |
| delta        | `max - min`（変動幅）                      |
| score        | 発火時 1.0 / 非発火時 0.0                 |

---

## パラメータ（`rule_alert_engine`）

| パラメータ名              | デフォルト | 説明                                      |
|--------------------------|------------|-------------------------------------------|
| `flatline_history_size`   | `8`        | flatline 判定に使う末尾サンプル数         |
| `flatline_hr_epsilon`     | `1.0`      | HR の許容変動幅（bpm）                    |
| `flatline_spo2_epsilon`   | `1.0`      | SpO2 の許容変動幅（%）                    |

`enabled_rule_ids` で `flatline.hr` / `flatline.spo2` を指定すれば、他ルールをオフにしつつ flatline だけを監視できます。

---

## 判定アルゴリズム

```
recent = history[末尾 flatline_history_size 件]
range  = max(recent) - min(recent)
active = (range <= epsilon)
```

- `range == 0.0` は最も典型的な flatline（完全固着）。
- `range ≤ epsilon` の境界値（ちょうど epsilon と等しい場合）も発火します。
- 履歴が `flatline_history_size` 件に満たない場合は `active=False`（判定保留）。

`temporal_stability` 種別は**レベルトリガ + cooldown** で制御します（`rule_alert_engine`）。

- `active=True` が継続している間、`_TEMPORAL_STABILITY_COOLDOWN_SEC`（10 秒）ごとに再発火します。
- エッジトリガ（`False → True` の瞬間のみ）では継続中の flatline が見逃されるため、意図的にレベルトリガを採用しています。

---

## `vital_sensor` への追加（`scenario='flatline'`）

```bash
ros2 run medical_robot_sim vital_sensor \
  --ros-args \
  --remap __ns:=/patient_01 \
  -p patient_id:=patient_01 \
  -p scenario:=flatline \
  -p flatline_hr_value:=72 \
  -p flatline_spo2_value:=98 \
  -p publish_rate_hz:=1.0
```

- `scenario:=flatline` を指定すると、HR と SpO2 を **固定値**（`flatline_hr_value` / `flatline_spo2_value`）で送り続けます。
- `flatline_history_size` 件（デフォルト 8 件）送れば `flatline.hr` / `flatline.spo2` が発火します。

---

## 再現手順（`repro_day6_flatline.sh`）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash

./scripts/repro_day6_flatline.sh
```

スクリプトは以下を自動実行します:

1. `vital_sensor`（`scenario:=flatline`）を起動
2. `rule_alert_engine`（`flatline_history_size=8`）を起動
3. `bags/day6_flatline_YYYYMMDD_HHMMSS/` に `patient_vitals` と `alerts` を記録
4. 30 秒待機したのち停止し、`ros2 bag info` を表示

> **Note (WSL2)**: WSL2 環境ではタイマー解像度の制約から `publish_rate_hz=1.0` でも実効レートが約 0.2Hz になることがあります。  
> デフォルト `flatline_history_size=8` のとき発火まで約 40 秒かかります。  
> 素早く確認したい場合は `publish_rate_hz:=5.0` を指定してください。

---

## 合格条件

| チェック項目                                         | 確認方法                                 |
|------------------------------------------------------|------------------------------------------|
| `flatline.hr` / `flatline.spo2` が `/patient_01/alerts` に publish される | `ros2 topic echo /patient_01/alerts`     |
| bag の `/patient_01/alerts` に Message Count > 0    | `ros2 bag info <bag_dir>`                |
| `kind: temporal_stability` が含まれる                | `ros2 topic echo` の出力確認             |
| テストが `colcon test` で全通する                    | `colcon test && colcon test-result`      |

---

## テスト（`test_alert_rules.py`）

追加されたテスト（抜粋）:

| テスト名                                        | 観点                                  |
|-------------------------------------------------|---------------------------------------|
| `test_flatline_hr_active_when_constant`         | HR 固定値で active=True               |
| `test_flatline_spo2_active_when_constant`       | SpO2 固定値で active=True             |
| `test_flatline_hr_not_active_when_variation...` | 変動幅超過で active=False              |
| `test_flatline_not_active_when_history_too_short` | 件数不足で active=False（判定保留）   |
| `test_flatline_hr_boundary_exactly_at_epsilon`  | ちょうど epsilon で active=True（境界値）|
| `test_flatline_uses_only_recent_window`         | 末尾8件のみを使っていることを確認 (`window_sec=8`) |

---

## 関連ファイル

| ファイル                                                        | 変更内容                                      |
|-----------------------------------------------------------------|-----------------------------------------------|
| `src/medical_robot_sim/medical_robot_sim/alert_rules.py`        | `RuleParams` に flatline パラメータ追加、`flatline.hr` / `flatline.spo2` ルール追加 |
| `src/medical_robot_sim/medical_robot_sim/rule_alert_engine.py`  | `flatline_*` パラメータを declare・RuleParams へ渡す；`temporal_stability` 向けレベルトリガ（`_select_level_fired_matches`）追加 |
| `src/medical_robot_sim/medical_robot_sim/vital_sensor.py`       | `scenario='flatline'`、`flatline_hr_value` / `flatline_spo2_value` パラメータ追加 |
| `src/medical_robot_sim/test/test_alert_rules.py`                | flatline ルールのテスト7件追加                |
| `scripts/repro_day6_flatline.sh`                                | Day6 再現スクリプト（新規）                   |
