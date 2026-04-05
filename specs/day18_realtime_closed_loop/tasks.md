# Day18 リアルタイム制御（閉ループ）タスク（tasks）

## Implementation tasks

- [ ] `closed_loop_policy.py` を追加する（pure functions）
- [ ] action enum/定数（`HOLD`, `OXYGEN_BOOST`, `CALL_STAFF`）を定義する
- [ ] NO_DATA ガード、SpO2閾値、RED alert 優先の判定を実装する
- [ ] cooldown と重複抑制（同一action連打防止）を実装する
- [ ] `closed_loop_controller.py` を追加する
- [ ] 患者 namespace ごとに `patient_vitals` / `alerts` を購読できるようにする
- [ ] `control_actions` publish（`medical_interfaces/msg/Alert` 再利用）を実装する
- [ ] `event=control.config/decision/publish/suppressed/safe_hold` を実装する
- [ ] `setup.py` に `closed_loop_controller` の entry point を追加する
- [ ] `icu_multi_patient.launch.py` に `enable_closed_loop` と `control_*` 引数を追加する
- [ ] `enable_closed_loop=true` のときのみ controller を患者ごとに起動する

## Validation tasks

- [ ] `enable_closed_loop=false` で `/patient_01/control_actions` が生成されない
- [ ] `enable_closed_loop=true` で `event=control.config` がログに出る
- [ ] flatline など低SpO2シナリオで `event=control.publish` が出る
- [ ] `ros2 topic echo --once /patient_01/control_actions` が1件以上取得できる
- [ ] cooldown 中の重複 action が抑制され `event=control.suppressed` が出る
- [ ] Ctrl+C停止で `Traceback` / `KeyboardInterrupt` が出ない

## Automated test tasks

- [ ] `src/medical_robot_sim/test/test_day18_closed_loop_policy.py` を追加する
- [ ] `spo2 == low_spo2` 境界をテストする
- [ ] `spo2 == critical_spo2` 境界をテストする
- [ ] `age_sec > no_data_after_sec` で常に `HOLD` をテストする
- [ ] cooldown で `should_publish=false` をテストする
- [ ] 正常値・alertなしで `HOLD` をテストする
- [ ] `colcon test --packages-select medical_robot_sim` を通す

## Docs update tasks

- [ ] `docs/day18_realtime_closed_loop.md` に Quickstart（5分）を維持する
- [ ] `docs/day18_realtime_closed_loop.md` に受け入れ導線を明記する
- [ ] `docs/day18_realtime_closed_loop.md` にトラブル時の読み方（ログ/トピック/順序）を明記する
- [ ] `specs/day18_realtime_closed_loop/spec.md` と実装差分が一致していることを確認する
- [ ] `specs/day18_realtime_closed_loop/acceptance.md` の期待条件が観測可能（grep/echo）であることを確認する
- [ ] READMEは直接更新せず、必要な導線リンク候補だけ docs に記載する
