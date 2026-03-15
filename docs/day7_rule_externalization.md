# Day7: ルール設定の外部化

## なぜ外部化したか

アラートルールの閾値や有効/無効の切り替えをコード変更なしで行えるようにするためです。
現場の検証では、同じロジックでも感度調整や一部ルールだけの有効化が頻繁に必要になります。
YAML で管理すれば再現性を保ったままパラメータ調整ができます。

## YAML フォーマット

```yaml
rule_config:
  spo2_drop_threshold: 4.0       # roc.spo2_drop 発火閾値
  hr_jump_threshold: 20.0        # roc.hr_jump 発火閾値
  flatline_history_size: 8       # flatline 判定サンプル数
  flatline_hr_epsilon: 1.0       # flatline HR 閾値 [bpm]
  flatline_spo2_epsilon: 1.0     # flatline SpO2 閾値 [%]
  enabled_rule_ids: []           # 空 = 全ルール有効
```

YAML は `src/medical_robot_sim/config/alert_rules.yaml` に置いてあります。
`colcon build` 後は package share にもインストールされます。

## 読み込み優先順位

```
ROS param (launch arg) > alert_rules.yaml > コード内デフォルト
```

- `rules_path` で YAML を読み込む
- `enabled_rule_ids` などは launch 引数が最優先
- YAML 未指定時はコード内デフォルトで動作

## 再現手順

環境読み込み:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

起動（YAML 既定パス）:

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  enable_alerts:=true \
  scenario:=flatline
```

`flatline.hr` だけに絞る例:

```bash
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  enable_alerts:=true \
  scenario:=flatline \
  enabled_rule_ids:=flatline.hr
```

確認:

```bash
timeout 15s ros2 topic echo /patient_01/alerts
```

期待:
- `rule_id: flatline.hr` または `flatline.spo2` が出力される
- Day6 の flatline 検知が維持される

## 今後の拡張

- ルール追加時の YAML テンプレート拡充
- 患者別のルールセット切り替え
- 複数 YAML の切り替えを scripts 化
