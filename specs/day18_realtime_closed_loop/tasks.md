# Day18 リアルタイム制御（閉ループ）タスク（tasks）

## Implementation tasks

- [x] `closed_loop_policy.py` を追加する（pure functions）
- [x] action enum/定数（`HOLD`, `OXYGEN_BOOST`, `CALL_STAFF`）を定義する
- [x] NO_DATA ガード、SpO2閾値、RED alert 優先の判定を実装する
- [x] cooldown と重複抑制（同一action連打防止）を実装する
- [x] `closed_loop_controller.py` を追加する
- [x] 患者 namespace ごとに `patient_vitals` / `alerts` を購読できるようにする
- [x] `control_actions` publish（`medical_interfaces/msg/Alert` 再利用）を実装する
- [x] `event=control.config/decision/publish/suppressed/safe_hold` を実装する
- [x] `setup.py` に `closed_loop_controller` の entry point を追加する
- [x] `icu_multi_patient.launch.py` に `enable_closed_loop` と `control_*` 引数を追加する
- [x] `enable_closed_loop=true` のときのみ controller を患者ごとに起動する

## Validation tasks

- [x] `enable_closed_loop=false` で `/patient_01/control_actions` が生成されない
- [x] `enable_closed_loop=true` で `event=control.config` がログに出る
- [x] flatline など低SpO2シナリオで `event=control.publish` が出る
- [x] `ros2 topic echo --once /patient_01/control_actions` が1件以上取得できる
- [x] cooldown 中の重複 action が抑制され `event=control.suppressed` が出る
- [x] Ctrl+C停止で `Traceback` / `KeyboardInterrupt` が出ない

## Automated test tasks

- [x] `src/medical_robot_sim/test/test_day18_closed_loop_policy.py` を追加する
- [x] `spo2 == low_spo2` 境界をテストする
- [x] `spo2 == critical_spo2` 境界をテストする
- [x] `age_sec > no_data_after_sec` で常に `HOLD` をテストする
- [x] cooldown で `should_publish=false` をテストする
- [x] 正常値・alertなしで `HOLD` をテストする
- [x] `colcon test --packages-select medical_robot_sim` を通す

## Docs update tasks

- [x] `docs/day18_realtime_closed_loop.md` に Quickstart（5分）を維持する
- [x] `docs/day18_realtime_closed_loop.md` に受け入れ導線を明記する
- [x] `docs/day18_realtime_closed_loop.md` にトラブル時の読み方（ログ/トピック/順序）を明記する
- [x] `specs/day18_realtime_closed_loop/spec.md` と実装差分が一致していることを確認する
- [x] `specs/day18_realtime_closed_loop/acceptance.md` の期待条件が観測可能（grep/echo）であることを確認する
- [x] READMEは直接更新せず、必要な導線リンク候補だけ docs に記載する
