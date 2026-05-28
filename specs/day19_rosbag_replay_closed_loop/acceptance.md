# Day19 rosbag replay × 閉ループ制御統合 受け入れ基準（acceptance）

## 0. 重要

- すべて `bash` ブロックをそのままコピーして実行する。
- 判定は必ずログファイルと `grep -n` で行う。
- `WORK_DIR` を事前に設定しておくと、各ブロック間でファイルを共有できる。
- ROS 2 CLI daemon がトピック発見を阻害することがあるため `ROS2CLI_NO_DAEMON=1` を推奨。

## 1. ビルドとテスト

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash

colcon test --packages-select medical_robot_sim
colcon test-result --verbose
```

期待条件:

- failures が 0

## 2. エントリポイント存在確認

```bash
(
  set -euo pipefail
  cd ~/ros2_ws
  set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u

  ros2 launch medical_robot_sim icu_replay.launch.py --show-args \
    | tee /tmp/day19_replay_args.txt
  grep -n "enable_closed_loop"          /tmp/day19_replay_args.txt
  grep -n "control_low_spo2"            /tmp/day19_replay_args.txt
  grep -n "control_critical_spo2"       /tmp/day19_replay_args.txt
  grep -n "control_cooldown_sec"        /tmp/day19_replay_args.txt
  grep -n "control_no_data_after_sec"   /tmp/day19_replay_args.txt

  echo "OK: day19 icu_replay launch args exist"
)
```

期待条件:

- `enable_closed_loop` と `control_*` が `--show-args` に表示される

## 3. 後方互換確認（enable_closed_loop=false で control_actions が出ない）

```bash
(
  set -euo pipefail
  cd ~/ros2_ws
  set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
  export ROS2CLI_NO_DAEMON=1

  PATIENT_ID="patient_01"
  WORK_DIR="$(mktemp -d /tmp/day19_compat.XXXXXX)"
  BAG_DIR="${WORK_DIR}/vitals_bag"

  # --- Phase 1: bag 記録 ---
  ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    patients:="${PATIENT_ID}" enable_alerts:=false scenario:=normal \
    sigterm_timeout:=2 sigkill_timeout:=2 \
    > "${WORK_DIR}/live.log" 2>&1 &
  LIVE_PID=$!
  sleep 6

  ros2 bag record -o "${BAG_DIR}" "/${PATIENT_ID}/patient_vitals" \
    </dev/null > "${WORK_DIR}/record.log" 2>&1 &
  REC_PID=$!
  sleep 2

  ros2 topic pub --once \
    --qos-reliability reliable --qos-durability transient_local \
    "/${PATIENT_ID}/patient_vitals" medical_interfaces/msg/VitalSigns \
    "{patient_id: '${PATIENT_ID}', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}" || true
  sleep 3

  kill -INT "${REC_PID}"  2>/dev/null || true; wait "${REC_PID}"  2>/dev/null || true
  kill -INT "${LIVE_PID}" 2>/dev/null || true; wait "${LIVE_PID}" 2>/dev/null || true

  # --- Phase 2: replay（閉ループ無効）---
  ros2 launch medical_robot_sim icu_replay.launch.py \
    patients:="${PATIENT_ID}" \
    enable_alerts:=true \
    enable_closed_loop:=false \
    sigterm_timeout:=2 sigkill_timeout:=2 \
    > "${WORK_DIR}/replay.log" 2>&1 &
  REPLAY_PID=$!
  sleep 10

  if ros2 topic list 2>/dev/null | grep -q "^/${PATIENT_ID}/control_actions$"; then
    echo "ERROR: control_actions exists while enable_closed_loop=false" >&2
    kill -INT "${REPLAY_PID}" 2>/dev/null || true; wait "${REPLAY_PID}" 2>/dev/null || true
    rm -rf "${WORK_DIR}"
    exit 1
  fi

  kill -INT "${REPLAY_PID}" 2>/dev/null || true; wait "${REPLAY_PID}" 2>/dev/null || true
  rm -rf "${WORK_DIR}"
  echo "OK: backward compatibility preserved"
)
```

期待条件:

- `/patient_01/control_actions` が存在しない

## 4. 閉ループ有効時の control_actions 観測（必須）

```bash
(
  set -euo pipefail
  cd ~/ros2_ws
  set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
  export ROS2CLI_NO_DAEMON=1

  PATIENT_ID="patient_01"
  WORK_DIR="$(mktemp -d /tmp/day19_loop.XXXXXX)"
  BAG_DIR="${WORK_DIR}/vitals_bag"

  # --- Phase 1: bag 記録 ---
  ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    patients:="${PATIENT_ID}" enable_alerts:=false scenario:=normal \
    sigterm_timeout:=2 sigkill_timeout:=2 \
    > "${WORK_DIR}/live.log" 2>&1 &
  LIVE_PID=$!
  sleep 6

  ros2 bag record -o "${BAG_DIR}" "/${PATIENT_ID}/patient_vitals" \
    </dev/null > "${WORK_DIR}/record.log" 2>&1 &
  REC_PID=$!
  sleep 2

  ros2 topic pub --once \
    --qos-reliability reliable --qos-durability transient_local \
    "/${PATIENT_ID}/patient_vitals" medical_interfaces/msg/VitalSigns \
    "{patient_id: '${PATIENT_ID}', measurement_id: 999, heart_rate: 80, blood_pressure_systolic: 120, blood_pressure_diastolic: 80, body_temperature: 36.5, oxygen_saturation: 85, status: 'monitoring'}" || true
  sleep 3

  kill -INT "${REC_PID}"  2>/dev/null || true; wait "${REC_PID}"  2>/dev/null || true
  kill -INT "${LIVE_PID}" 2>/dev/null || true; wait "${LIVE_PID}" 2>/dev/null || true

  ros2 bag info "${BAG_DIR}" | tee "${WORK_DIR}/bag_info.txt"
  grep -n "Topic: /${PATIENT_ID}/patient_vitals" "${WORK_DIR}/bag_info.txt"

  # --- Phase 2: replay（閉ループ有効）---
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

  # wait for control_actions topic
  FOUND=0
  DEADLINE=$((SECONDS+30))
  while [ "${SECONDS}" -lt "${DEADLINE}" ]; do
    if ros2 topic list 2>/dev/null | grep -q "^/${PATIENT_ID}/control_actions$"; then
      FOUND=1; break
    fi
    sleep 1
  done
  if [ "${FOUND}" -ne 1 ]; then
    echo "ERROR: /patient_01/control_actions not found" >&2
    tail -n 100 "${WORK_DIR}/replay.log" >&2 || true
    kill -INT "${REPLAY_PID}" 2>/dev/null || true; wait "${REPLAY_PID}" 2>/dev/null || true
    rm -rf "${WORK_DIR}"
    exit 1
  fi

  timeout 35s ros2 topic echo --once \
    "/${PATIENT_ID}/control_actions" medical_interfaces/msg/Alert \
    > "${WORK_DIR}/control_once.txt" 2>&1 &
  ECHO_PID=$!

  sleep 1
  ros2 bag play "${BAG_DIR}" --loop </dev/null > "${WORK_DIR}/play.log" 2>&1 &
  PLAY_PID=$!

  wait "${ECHO_PID}" || true
  kill -INT "${PLAY_PID}"   2>/dev/null || true; wait "${PLAY_PID}"   2>/dev/null || true
  kill -INT "${REPLAY_PID}" 2>/dev/null || true; wait "${REPLAY_PID}" 2>/dev/null || true

  grep -n "kind: control_action"  "${WORK_DIR}/control_once.txt"
  grep -n "rule_id: control\."    "${WORK_DIR}/control_once.txt"
  grep -n "event=control\.config" "${WORK_DIR}/replay.log"
  grep -n "event=control\.publish" "${WORK_DIR}/replay.log"

  if grep -n "Traceback\|KeyboardInterrupt" "${WORK_DIR}/replay.log"; then
    echo "ERROR: stack trace found on shutdown" >&2
    rm -rf "${WORK_DIR}"
    exit 1
  fi

  rm -rf "${WORK_DIR}"
  echo "OK: replay + closed-loop control action observed"
)
```

期待条件:

- `/patient_01/control_actions` が存在する
- `control_actions` メッセージに `kind: control_action` と `rule_id: control.*` が含まれる
- ログに `event=control.config` と `event=control.publish` が出る
- 停止時にスタックトレースが出ない

## 5. CI スモークテスト（コピペ実行）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

bash scripts/ci_day19_replay_control_smoke.sh
```

期待条件:

- `CI Day19 rosbag replay + closed-loop control smoke test passed` と出力される
