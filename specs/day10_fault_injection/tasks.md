# Day10 Fault Injection タスク（tasks）

## Implementation tasks

- [x] `medical_robot_sim/fault_injection.py` を新規追加し、パラメータ検証/換算/判定の純粋関数を実装する
- [x] `vital_sensor` に Day10 fault params を declare し、`fault_injection.py` を使って drop/delay/pause/stop を適用する
- [x] delay/jitter は `time.sleep()` を使わず、due_tick キューで実装する（executor をブロックしない）
- [x] `vitals_fault_stop_after_sec` により、例外スタックトレース無しで擬似停止できるようにする
- [x] `icu_multi_patient.launch.py` に Day10 の launch arg を追加し、各 `vital_sensor` に params を渡す
- [x] 不正な fault params の場合に、エラーメッセージが分かる形で起動失敗する（ValueError→ログ→終了コード）

## Validation tasks

- [x] 既定起動（fault params 未指定）で従来通り vitals が流れることを確認する（後方互換）
- [x] `vitals_fault_drop_rate:=0.8 vitals_fault_seed:=42` で `ros2 topic hz` の観測レートが下がることを確認する
- [x] `vitals_fault_pause_after_sec:=5.0 vitals_fault_pause_duration_sec:=6.0` で vitals が「途切れて再開」することを確認する
- [x] `vitals_fault_delay_ms:=500` で vitals の到達遅延が観測できることを確認する（`specs/day10_fault_injection/acceptance.md` の Delay 注入手順）
- [x] `vitals_fault_stop_after_sec:=5.0` で `vital_sensor` が停止し、以降 vitals 更新が止まることを確認する
- [x] どの fault 設定でも `Ctrl+C` で停止してスタックトレースが出ないことを確認する

## Automated tests

- [x] `src/medical_robot_sim/test/test_fault_injection.py` を新規追加する
- [x] `validate_fault_params` が範囲外で `ValueError` を投げること
- [x] `ms_to_ticks` の境界値（0ms、1tick未満、複数tick）をテストする
- [x] `should_drop` の端（0.0/1.0）と seed 固定の再現性をテストする
- [x] `colcon test --packages-select medical_robot_sim` が成功する

## Docs update tasks

- [x] `docs/day10_fault_injection.md` に再現手順（drop/pause/stop の最低2つ）と期待観測を載せる
- [x] （必要なら）README に Day10 の launch 引数を最小追記する
