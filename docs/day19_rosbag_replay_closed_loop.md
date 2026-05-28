# Day19: rosbag replay × 閉ループ制御の統合検証（day19_rosbag_replay_closed_loop）

## Purpose

Day12（rosbag再現性）と Day18（閉ループ制御）を**縦断的に統合**し、
「記録した vitals を replay → alerts 生成 → closed-loop control action 発行」を
**一本の自動化パイプラインで検証可能**にする。

このDayが実現すること:

- `icu_replay.launch.py` で `enable_closed_loop:=true` を指定すると、
  replay 再生時にも `closed_loop_controller` が起動し `control_actions` を publish する
- CI スモークテスト（`scripts/ci_day19_replay_control_smoke.sh`）で上記を機械判定できる
- pure function テスト（`test_day19_replay_control_policy.py`）で replay シナリオの境界を単体検証できる

## Background

Day18 で閉ループ制御を live pipeline に追加したが、次の課題が残っていた。

| 課題 | 内容 |
| --- | --- |
| 再現性 | 閉ループ制御の判定結果を、事後に同じ入力で再現検証できない |
| CI カバレッジ | `icu_replay.launch.py` は alerts まで検証するが、control_actions は未カバー |
| 境界テスト不足 | replay シナリオ固有の境界（stale age、閾値パラメータ変更）がテストされていない |

Day19 はこれらを解消し、「vitals → alerts → control_actions」の一貫した
**再現可能な閉ループ検証ループ**を確立する。

## Why this day matters

- Day12 の「rosbag 再現性」と Day18 の「閉ループ制御」の橋渡しになる
- CI が `control_actions` まで観測するようになり、回帰リスクを機械的に検出できる
- policy の境界テストが replay 文脈で追加され、閾値変更時の安全性を担保できる
- Physical AI の「知覚→判断→行動」ループが、replay環境でも再現できることを証明する

## Design

### 1) `icu_replay.launch.py` への閉ループ統合

既存の `icu_replay.launch.py` に、`icu_multi_patient.launch.py` と同じ
`enable_closed_loop` / `control_*` 引数セットを追加する。

追加引数（全て後方互換デフォルト）:

| 引数 | デフォルト | 説明 |
| --- | --- | --- |
| `enable_closed_loop` | `false` | true のとき患者ごとに `closed_loop_controller` を起動 |
| `control_topic` | `control_actions` | 制御出力トピック名 |
| `control_cooldown_sec` | `5.0` | 連続 publish の最短間隔 [秒] |
| `control_no_data_after_sec` | `10.0` | NO_DATA ガード判定秒数 |
| `control_low_spo2` | `92.0` | OXYGEN_BOOST 閾値 [%] |
| `control_critical_spo2` | `88.0` | CALL_STAFF 閾値 [%] |
| `enable_control_from_advisories` | `false` | advisory を制御入力に加えるか |

`enable_closed_loop=false` のとき、既存の replay 動作（alerts まで）は変わらない。

### 2) CI スモークテスト（`scripts/ci_day19_replay_control_smoke.sh`）

**Phase 1: bag 記録**
- `icu_multi_patient.launch.py scenario:=normal` で live vitals を起動
- `ros2 bag record` で `/patient_XX/patient_vitals` を記録
- `ros2 topic pub --once` で SpO2=85 の異常サンプルを注入（決定論的な制御トリガー）
- 記録停止・`ros2 bag info` で内容検証

**Phase 2: replay + 閉ループ検証**
- `icu_replay.launch.py enable_closed_loop:=true control_low_spo2:=99.0 control_critical_spo2:=95.0` を起動
- `ros2 topic echo --once` で `/patient_XX/alerts` と `/patient_XX/control_actions` を待受け
- `ros2 bag play --loop` でバッグを再生
- `control_actions` に `kind: control_action` と `rule_id: control.*` が含まれることを確認
- ログに `event=control.config` と `event=control.publish` が出ることを確認

### 3) pure function テスト（`test_day19_replay_control_policy.py`）

既存の `test_day18_closed_loop_policy.py` を replay 文脈で補完する追加テスト群:

| テスト | 検証内容 |
| --- | --- |
| `test_replay_spo2_drop_triggers_call_staff` | SpO2=85 < critical=95 → CALL_STAFF |
| `test_replay_spo2_below_low_triggers_oxygen_boost` | SpO2=97 < low=99, > critical=95 → OXYGEN_BOOST |
| `test_replay_normal_vitals_hold` | SpO2=100（正常）→ HOLD |
| `test_replay_no_data_guard_overrides_critical_spo2` | age_sec=15 > 10 → 常に HOLD |
| `test_replay_age_sec_at_boundary_is_ok` | age_sec=10.0 == no_data_after_sec=10.0 → ガード発動しない |
| `test_replay_cooldown_suppresses_second_publish` | cooldown 内の2回目 → should_publish=False |
| `test_replay_cooldown_allows_publish_after_expiry` | cooldown 後は再発行可 |
| `test_replay_advisory_flag_off_uses_alert_priority` | RED alert → CALL_STAFF（advisory 無効時） |
| `test_replay_age_sec_none_is_no_data` | age_sec=None → HOLD |
| `test_replay_call_staff_severity_red` | severity=RED |
| `test_replay_oxygen_boost_severity_yellow` | severity=YELLOW |
| `test_replay_hold_severity_info` | severity=INFO |

## Affected files

- `src/medical_robot_sim/launch/icu_replay.launch.py`（変更）
- `src/medical_robot_sim/test/test_day19_replay_control_policy.py`（新規）
- `scripts/ci_day19_replay_control_smoke.sh`（新規）
- `.github/workflows/ci.yml`（変更）
- `docs/day19_rosbag_replay_closed_loop.md`（本ファイル）
- `specs/day19_rosbag_replay_closed_loop/spec.md`（新規）
- `specs/day19_rosbag_replay_closed_loop/tasks.md`（新規）
- `specs/day19_rosbag_replay_closed_loop/acceptance.md`（新規）

## Reproduction / validation steps

詳細な受け入れ手順は `specs/day19_rosbag_replay_closed_loop/acceptance.md` を参照。

### Quickstart（5分で確認）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

# Phase 1: bag 記録（SpO2=85 の異常サンプルを含む）
WORK_DIR="$(mktemp -d /tmp/day19_quick.XXXXXX)"
BAG_DIR="${WORK_DIR}/vitals_bag"
PATIENT_ID="patient_01"

ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:="${PATIENT_ID}" enable_alerts:=false scenario:=normal \
  sigterm_timeout:=2 sigkill_timeout:=2 \
  > "${WORK_DIR}/live.log" 2>&1 &
LIVE_PID=$!
sleep 5

ros2 bag record -o "${BAG_DIR}" "/${PATIENT_ID}/patient_vitals" \
  </dev/null > "${WORK_DIR}/record.log" 2>&1 &
REC_PID=$!
sleep 2

ros2 topic pub --once \
  --qos-reliability reliable --qos-durability transient_local \
  "/${PATIENT_ID}/patient_vitals" medical_interfaces/msg/VitalSigns \
  "{patient_id: '${PATIENT_ID}', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}"

sleep 3
kill -INT $REC_PID 2>/dev/null || true; wait $REC_PID 2>/dev/null || true
kill -INT $LIVE_PID 2>/dev/null || true; wait $LIVE_PID 2>/dev/null || true

ros2 bag info "${BAG_DIR}"

# Phase 2: replay + 閉ループ検証
ros2 launch medical_robot_sim icu_replay.launch.py \
  patients:="${PATIENT_ID}" \
  enable_alerts:=true \
  enable_closed_loop:=true \
  control_low_spo2:=99.0 \
  control_critical_spo2:=95.0 \
  control_cooldown_sec:=1.0 \
  sigterm_timeout:=2 sigkill_timeout:=2 \
  > "${WORK_DIR}/replay.log" 2>&1 &
REPLAY_PID=$!

# Wait for topics and echo once
sleep 8
ros2 topic echo --once "/${PATIENT_ID}/control_actions" \
  medical_interfaces/msg/Alert | tee "${WORK_DIR}/control_once.txt" &
ECHO_PID=$!
ros2 bag play "${BAG_DIR}" --loop </dev/null > "${WORK_DIR}/play.log" 2>&1 &
PLAY_PID=$!

wait $ECHO_PID || true
kill -INT $PLAY_PID 2>/dev/null || true; wait $PLAY_PID 2>/dev/null || true
kill -INT $REPLAY_PID 2>/dev/null || true; wait $REPLAY_PID 2>/dev/null || true

grep "event=control\.config" "${WORK_DIR}/replay.log" || true
grep "event=control\.publish" "${WORK_DIR}/replay.log" || true
grep "kind: control_action" "${WORK_DIR}/control_once.txt" || true

echo "OK: day19 quickstart ran (see ${WORK_DIR})"
```

## Success criteria

- `enable_closed_loop:=true` で `icu_replay.launch.py` が `control_actions` を publish する
- `ros2 topic echo --once /patient_01/control_actions` に `kind: control_action` が含まれる
- `event=control.config` と `event=control.publish` がログに出る
- `enable_closed_loop:=false`（既定）で従来の replay 動作（alerts まで）を維持する
- `colcon test --packages-select medical_robot_sim` が成功する（新規 13 テスト含む）
- CI の Day19 スモークテストがパスする
- Ctrl+C 停止で `Traceback`/`KeyboardInterrupt` が出ない

## Learning path

### 最短コピペ（Quickstart）

上記 Quickstart を実行し `control_actions` の `kind: control_action` を確認する。

### 読み方（トラブル時の順番）

1. `replay.log` の `event=control.config` を grep
   - 閉ループが有効化されているか、閾値は期待値か
2. `ros2 node list | grep closed_loop_controller`
   - replay launch で controller が起動しているか
3. `ros2 topic echo --once /patient_01/control_actions`
   - bag play 後に action が publish されるか
4. `event=control.safe_hold` を確認
   - bag の SpO2 が閾値以上の場合 HOLD になる（異常サンプルが記録されているか確認）

### 学習者が説明できるようになること（達成目標）

- rosbag replay で閉ループ制御を再現検証できる理由（topic 契約を壊さない設計）
- `control_no_data_after_sec` が replay 時に重要な理由
  （bag play 開始直後は age_sec が大きくなりうる）
- CI でスモークテストを段階的に積み上げる意義

## Relevance to Medical / Physical AI

- 「閉ループ制御が正しく動いたか」を後から再現検証できることは、医療応用での説明責任に直結する
- bag replay による再現性は、同じシナリオで繰り返し比較評価を行う実験基盤になる
- rosbag + CI の組合せにより、アルゴリズム変更時の回帰を自動検出できる

## Connection to the roadmap

Day19 は Day12（rosbag再現性）と Day18（閉ループ制御）を統合する「横断的なDay」である。
次の実践課題として:

- Edge（Day14）で遅延条件が変わったときの制御安定性を、同じ bag で比較評価する
- BCI（Day15）+ advisory（Day16）を含む複合シナリオを bag に記録して replay 検証する
- 複数患者の bag を一括 replay して、協調制御（Day17）の再現性を評価する
