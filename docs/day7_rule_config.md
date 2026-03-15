# Day7: アラートルール設定の外部化

## 概要

Day7 では `rule_alert_engine` が読み込むアラートルールのパラメータを、
コード外の YAML ファイルで管理できるようにしました。

- ルールの感度調整がコード変更なしで行えます
- コード変更なしで `enabled_rule_ids` によるルール有効/無効切り替えが可能になります
- 設定ファイル未指定時はコード内デフォルト値で動作し、**後方互換を維持**します

---

## 設定ファイルの場所

```
src/medical_robot_sim/config/alert_rules.yaml
```

`colcon build` 後は次の場所にもインストールされます:

```
install/medical_robot_sim/share/medical_robot_sim/config/alert_rules.yaml
```

---

## 設定ファイルの形式

```yaml
rule_config:
  # SpO2 変化率ルール: window 内の最大低下量がこの値を超えると roc.spo2_drop を発火
  spo2_drop_threshold: 4.0

  # 心拍数変化率ルール: window 内の最大上昇量がこの値を超えると roc.hr_jump を発火
  hr_jump_threshold: 20.0

  # flatline 判定に使う直近サンプル数
  flatline_history_size: 8

  # flatline 判定: 直近 N 件の HR の max-min がこの値以下なら flatline [bpm]
  flatline_hr_epsilon: 1.0

  # flatline 判定: 直近 N 件の SpO2 の max-min がこの値以下なら flatline [%]
  flatline_spo2_epsilon: 1.0

  # 有効にするルール ID のリスト（空リスト = 全ルール有効）
  enabled_rule_ids: []
```

各キーはすべて **省略可能**です。省略したキーはコード内デフォルト値が使われます。

---

## 起動方法

### 通常起動（YAML 未指定 / コード内デフォルト使用）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py
```

### YAML ファイルを指定して起動

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    rules_path:=$(ros2 pkg prefix medical_robot_sim)/share/medical_robot_sim/config/alert_rules.yaml
```

相対パスを指定した場合は、パッケージ共有ディレクトリ基準で解決されます:

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  rules_path:=config/alert_rules.yaml
```

カスタム設定ファイルを使う場合:

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    rules_path:=/home/user/my_alert_rules.yaml
```

### 特定ルールだけ有効化（YAML + launch arg の組み合わせ例）

launch arg は YAML より優先されます:

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    rules_path:=/home/user/my_rules.yaml \
    enabled_rule_ids:=flatline.hr,flatline.spo2
```

---

## 優先順位

```
ROS param (launch arg) > alert_rules.yaml > コード内デフォルト
```

1. launch arg で明示的に指定された値が最優先
2. `rules_path` で YAML を読み込んだ場合、YAML の値が次の優先
3. どちらも指定されていない場合、コード内デフォルト値を使用

---

## コードを変更せずにルールを追加する方法

現在のアーキテクチャでは **ルール評価ロジックは `alert_rules.py` 内の純粋関数** として実装されています。
新しいルールを追加するには最低限 `alert_rules.py` の変更が必要ですが、
**既存ルールのしきい値変更や有効/無効切り替えは YAML だけで完結**します。

例: flatline 検出の感度を上げる

```yaml
rule_config:
  flatline_history_size: 12   # より長い窓でフラットラインを判定
  flatline_hr_epsilon: 0.5    # より厳しい閾値（ほぼ変化なしでもflatline）
  flatline_spo2_epsilon: 0.5
```

例: 特定ルールのみ監視する

```yaml
rule_config:
  enabled_rule_ids:
    - flatline.hr
    - flatline.spo2
    - roc.spo2_drop
```

---

## テスト

```bash
cd ~/ros2_ws
colcon test --packages-select medical_robot_sim
colcon test-result --verbose
```

テスト対象ファイル: `src/medical_robot_sim/test/test_rule_config_loader.py`

カバーしているケース:

- 全キー記載の YAML を正常読み込み
- 一部キー省略時の後方互換
- `enabled_rule_ids` の空リスト・空白除去
- ファイル不存在・YAML パースエラーの異常系
- ヘルパー関数 `get_float` / `get_int` / `get_string_list` のデフォルト動作

---

## 関連ファイル

| ファイル | 役割 |
|---|---|
| `config/alert_rules.yaml` | デフォルト設定ファイル |
| `medical_robot_sim/rule_config_loader.py` | YAML ローダー（純粋関数） |
| `medical_robot_sim/rule_alert_engine.py` | ローダーを使って parameters を初期化 |
| `launch/icu_multi_patient.launch.py` | `rules_path` launch arg を追加 |
| `test/test_rule_config_loader.py` | ローダーのユニットテスト |
