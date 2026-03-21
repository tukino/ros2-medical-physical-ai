# Day8 QoS設計 タスク（tasks）

## Implementation tasks

- [x] `medical_robot_sim/qos_profiles.py` を新規追加し、`build_qos_profile()` を実装する
- [x] `vital_sensor` に QoS params（`vitals_qos_*`）を追加し、publisher に適用する
- [x] `icu_monitor` に QoS params（`vitals_qos_*`）を追加し、subscription に適用する
- [x] `rule_alert_engine` に QoS params（`vitals_qos_*`, `alerts_qos_*`）を追加し、sub/pub に適用する
- [x] `icu_multi_patient.launch.py` に QoS の launch arg を追加し、各 node parameters に渡す
- [x] 不正な QoS パラメータが渡されたときに、エラーメッセージが分かる形で起動失敗することを確認する

## Validation tasks

- [x] 既定起動で topic が成立することを確認する（`ros2 topic list` / `ros2 topic echo`）
- [x] `ros2 topic info --verbose` で QoS が観測できることを確認する（vitals/alerts 両方）
- [x] `alerts_qos_durability:=transient_local` を指定して起動し、QoS 表示が切り替わることを確認する
- [x] `vitals_qos_reliability:=best_effort` を指定して起動し、QoS 表示が切り替わることを確認する

## Automated tests

- [x] `src/medical_robot_sim/test/test_qos_profiles.py` を新規追加する
- [x] `colcon test --packages-select medical_robot_sim` が成功する

## Docs update tasks

- [x] `docs/day8_qos.md` の再現手順が現状のコマンドで成立するよう確認する
- [x] 必要なら `README.md` に「QoS を切り替える launch 引数」セクションを最小追記する
