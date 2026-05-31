# Day19 rosbag replay × 閉ループ制御統合仕様（spec）

## Goal

`icu_replay.launch.py` に `enable_closed_loop` オプションを追加し、
rosbag replay 時にも `closed_loop_controller` が動作して `control_actions` を publish できるようにする。

達成条件:

- `icu_replay.launch.py enable_closed_loop:=true` で `control_actions` が観測できる
- `enable_closed_loop:=false`（既定）で従来の replay 動作（alerts まで）を維持する
- CI スモークテストが `control_actions` まで機械判定する
- replay 文脈の pure function テストが追加される

## Design

### 1) `icu_replay.launch.py` 変更

`icu_multi_patient.launch.py` と同じ `enable_closed_loop` / `control_*` 引数セットを追加する。
`enable_closed_loop=true` のとき、各 patient namespace に `closed_loop_controller` を起動する。

追加する LaunchArgument:

- `enable_closed_loop` (bool, default `false`)
- `control_topic` (str, default `control_actions`)
- `control_cooldown_sec` (float, default `5.0`)
- `control_no_data_after_sec` (float, default `10.0`)
- `control_low_spo2` (float, default `92.0`)
- `control_critical_spo2` (float, default `88.0`)
- `enable_control_from_advisories` (bool, default `false`)

`enable_closed_loop=false` のとき `closed_loop_controller` を起動しない（後方互換）。

### 2) `scripts/ci_day19_replay_control_smoke.sh`

Phase 1（bag 記録）と Phase 2（replay + 閉ループ検証）の2フェーズ構成。

Phase 1:
- `icu_multi_patient.launch.py scenario:=normal` で live vitals を起動し bag 記録
- `ros2 topic pub --once` で SpO2=85 の異常サンプルを注入
- `ros2 bag info` で `patient_vitals` が記録されていることを確認

Phase 2:
- `icu_replay.launch.py enable_closed_loop:=true control_low_spo2:=99.0 control_critical_spo2:=95.0` を起動
- `alerts` と `control_actions` トピックが出現するまで待機
- `ros2 topic echo --once` で両トピックを受信
- `ros2 bag play --loop` で bag を再生し、control action を受信することを検証
- ログに `event=control.config` と `event=control.publish` があることを確認

### 3) `test_day19_replay_control_policy.py`

`closed_loop_policy.decide_control_action` をリプレイシナリオ文脈でテストする。
テスト閾値: `control_low_spo2=99.0`, `control_critical_spo2=95.0`（CI smoke test と同じ）。

主要テスト:
- SpO2=85 → CALL_STAFF
- SpO2=97 (low < x < normal=100) → OXYGEN_BOOST  
- SpO2=100 → HOLD
- age_sec > no_data_after_sec → HOLD（NO_DATA ガード）
- cooldown 内の2回目 → should_publish=False
- cooldown 後 → should_publish=True
- RED alert + 正常 SpO2 → CALL_STAFF
- age_sec=None → HOLD

## Affected files

- `src/medical_robot_sim/launch/icu_replay.launch.py`（変更）
- `src/medical_robot_sim/test/test_day19_replay_control_policy.py`（新規）
- `scripts/ci_day19_replay_control_smoke.sh`（新規）
- `docs/day19_rosbag_replay_closed_loop.md`（新規）
- `specs/day19_rosbag_replay_closed_loop/spec.md`（本ファイル）
- `specs/day19_rosbag_replay_closed_loop/tasks.md`（新規）
- `specs/day19_rosbag_replay_closed_loop/acceptance.md`（新規）

> Note: `.github/workflows/ci.yml` への Day19 スモークテスト追加は GitHub App の
> `workflows` 権限制限によりこの PR の対象外。メンテナーが別途対応すること。

## Constraints

- `icu_replay.launch.py` の既存引数・動作を壊さない（後方互換）
- `enable_closed_loop=false` 時は `control_actions` topic が存在しない
- `enable_closed_loop=true` 時、`closed_loop_controller` は `patient_vitals` の SpO2 を
  直接評価するため `enable_alerts=false` でも SpO2 閾値違反で制御アクションを発行できる。
  `enable_alerts=false` にすると RED alert による CALL_STAFF トリガーは無効になるが、
  vitals 経由の OXYGEN_BOOST / CALL_STAFF は引き続き動作する
- Ctrl+C で clean shutdown（stack trace 無し）

## Non-goals

- `icu_replay.launch.py` への BCI/advisory ノード追加
- replay 専用の新規 closed-loop policy の実装
- bag の自動整理・ローテーション機能
- マルチトピック bag（alerts も含む）の replay 対応
