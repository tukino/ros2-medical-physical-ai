# Day17 マルチノード協調 タスク（tasks）

## Implementation tasks

- [ ] `medical_robot_sim/coordination_policy.py` を追加する（pure functions）
  - [ ] ready 判定（min_messages_per_patient）
  - [ ] NO_DATA 判定（no_data_after_sec）
  - [ ] 「全患者 NO_DATA のときだけ deactivate」判定

- [ ] `medical_robot_sim/icu_coordinator.py` を追加する
  - [ ] `patients`（string array）と `vitals_topic` を受け取れる
  - [ ] `/patient_XX/patient_vitals` を patients 分 subscribe できる（Day8 QoS params を使用）
  - [ ] vitals 受信回数を patient ごとにカウントし、ready を判定できる
  - [ ] lifecycle service client（`/rule_alert_engine/get_state`, `/rule_alert_engine/change_state`）を持つ
  - [ ] ready 到達で `configure -> activate` を実行する
  - [ ] （任意・既定ON）全患者 NO_DATA で `deactivate`、復帰で再 `activate`
  - [ ] Day11形式ログを出す（`event=coord.*`）
  - [ ] Ctrl+C で clean shutdown

- [ ] `setup.py` の `console_scripts` に `icu_coordinator` を追加する

- [ ] `launch/icu_multi_patient.launch.py` に coordinator 起動オプションを追加する
  - [ ] `enable_coordination`（default `false`）
  - [ ] `coord_*` 一式を launch arg 化し、`icu_coordinator` に渡す
  - [ ] `enable_coordination:=true` のときのみ root namespace で `icu_coordinator` を起動

## Validation tasks

- [ ] 後方互換: `enable_coordination:=false` のとき coordinator が起動しない
- [ ] `alerts_node_kind:=lifecycle lifecycle_autostart:=false enable_coordination:=true` で、
  `ros2 lifecycle get /rule_alert_engine` が `active` になる
- [ ] `event=coord.config` と `event=coord.lifecycle_set` がログに残る
- [ ] Ctrl+C 停止で `Traceback` / `KeyboardInterrupt` が出ない

## Automated tests

- [ ] `src/medical_robot_sim/test/test_day17_coordination_policy.py` を追加する
  - [ ] ready 判定の境界値
  - [ ] NO_DATA 判定の境界値
  - [ ] 全患者 NO_DATA の判定

- [ ] `colcon test --packages-select medical_robot_sim` が成功する

## Docs update tasks

- [ ] `docs/day17_multi_node_coordination.md` に Quickstart と acceptance 導線がある
- [ ] `specs/day17_multi_node_coordination/spec.md` / `tasks.md` / `acceptance.md` が揃っている
- [ ] README へ追記する場合は「最短導線リンクのみ」に留める（詳細は docs/specs に集約）
