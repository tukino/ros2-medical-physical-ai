# Day19 rosbag replay × 閉ループ制御統合タスク（tasks）

## Implementation tasks

- [x] `icu_replay.launch.py` に `enable_closed_loop` / `control_*` 引数を追加する
- [x] `enable_closed_loop=true` のとき患者ごとに `closed_loop_controller` を起動する
- [x] `enable_closed_loop=false` で従来の replay 動作（alerts まで）を維持する
- [x] `scripts/ci_day19_replay_control_smoke.sh` を新規作成する（Phase 1: 記録、Phase 2: replay+検証）
- [x] Phase 1 で SpO2=85 の決定論的な異常サンプルを注入する
- [x] Phase 2 で `control_actions` の `kind: control_action` と `rule_id: control.*` を確認する
- [x] Phase 2 でログの `event=control.config` と `event=control.publish` を確認する
- [x] clean shutdown（stack trace なし）を確認する

## Validation tasks

- [x] `enable_closed_loop:=false` で `/patient_01/control_actions` が生成されない
- [x] `enable_closed_loop:=true` で `event=control.config` がログに出る
- [x] replay + SpO2=85 サンプルで `event=control.publish` が出る
- [x] `ros2 topic echo --once /patient_01/control_actions` が1件以上取得できる
- [x] Ctrl+C 停止で `Traceback` / `KeyboardInterrupt` が出ない

## Automated test tasks

- [x] `src/medical_robot_sim/test/test_day19_replay_control_policy.py` を追加する
- [x] SpO2=85 < critical_spo2=95 → CALL_STAFF をテストする
- [x] SpO2=97 < low_spo2=99 → OXYGEN_BOOST をテストする
- [x] SpO2=100（正常）→ HOLD をテストする
- [x] age_sec > no_data_after_sec → HOLD（NO_DATA ガード）をテストする
- [x] age_sec=10.0 == no_data_after_sec=10.0 → ガード発動しないをテストする
- [x] cooldown 内の2回目 → should_publish=False をテストする
- [x] cooldown 後 → should_publish=True をテストする
- [x] RED alert + 正常 SpO2 → CALL_STAFF をテストする
- [x] age_sec=None → HOLD をテストする
- [x] severity マッピング（RED/YELLOW/INFO）をテストする
- [x] `colcon test --packages-select medical_robot_sim` を通す

## CI tasks

- [ ] `.github/workflows/ci.yml` に Day19 スモークテストステップを追加する
  （GitHub App の `workflows` 権限制限により本 PR 対象外。メンテナーフォローアップ）

## Docs update tasks

- [x] `docs/day19_rosbag_replay_closed_loop.md` を作成する（目的・設計・Quickstart・成功基準）
- [x] `specs/day19_rosbag_replay_closed_loop/spec.md` を作成する
- [x] `specs/day19_rosbag_replay_closed_loop/tasks.md` を作成する（本ファイル）
- [x] `specs/day19_rosbag_replay_closed_loop/acceptance.md` を作成する
- [x] `README.md` に Day19 の導線リンクを追加する
