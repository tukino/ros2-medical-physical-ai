# Day12 rosbagによる再現性検証 タスク（tasks）

## Implementation tasks

- [x] replay 用 launch `icu_replay.launch.py` を追加する（sensor 無しで `icu_monitor` と `rule_alert_engine` のみ起動）
- [x] `icu_replay.launch.py` に `patients`/`enable_alerts`/Day7 rules_path/Day8 QoS/Day9 lifecycle 切替を実装する
- [x] `medical_robot_sim/rosbag_topics.py` を追加し、record 対象 topic の組み立て純粋関数を実装する

## Automated tests

- [x] `src/medical_robot_sim/test/test_rosbag_topics.py` を追加し、topics 組み立ての境界（空要素、空白、先頭/末尾スラッシュ）をテストする
- [x] `colcon test --packages-select medical_robot_sim` が成功する

## Validation tasks（手動）

- [ ] `ros2 bag record` で `/patient_01/patient_vitals` を 1 回以上記録できる
- [ ] `ros2 bag info` で `/patient_01/patient_vitals` の Message Count > 0 を確認できる
- [ ] `icu_replay.launch.py` + `ros2 bag play` で、replay 中に `/patient_01/alerts` を 1 件以上受信できる
- [ ] Ctrl+C / SIGINT でスタックトレース無しに停止できる

## Docs update tasks

- [x] `docs/day12_rosbag_reproducibility.md` を追加し、record→info→play→観測の最短手順と切り分け順序を明記する
- [x] `specs/day12_rosbag_reproducibility/acceptance.md` にコピペ受け入れ手順を追加する
