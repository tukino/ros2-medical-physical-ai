# Day17 マルチノード協調 受け入れ基準（acceptance）

## 0. 重要（コマンドのコピペ方法）

- ターミナルには **このドキュメント内の ```bash``` ブロックの中身だけ** をコピーして実行する。
- `[...] ( ... )` のような **Markdown リンク表記**はコマンドではない。

## 0.1 共通セットアップ（毎ターミナル）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 付録: `ros2` CLI が不安定な場合（daemon再起動）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 daemon stop || true
ros2 daemon start
sleep 1
```

### 付録: 前回のプロセスが残っていそうな場合（掃除）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ps -eo pid,stat,cmd | egrep 'medical_robot_sim|ros2 launch' | grep -v egrep || true

pkill -INT -f "__ns:=/p[a]tient_0[1]" 2>/dev/null || true
# Day17 coordinator が残っていると Step3/4 の lifecycle が外部から遷移して誤判定になる
pkill -INT -f "__node:=icu_coordinat[o]r" 2>/dev/null || true
# rule_alert_engine が残っていると Step3/4 の `ros2 lifecycle get` が古い状態を拾う
pkill -INT -f "__node:=rule_alert_en[g]ine" 2>/dev/null || true
pkill -INT -f "rule_alert_engine_lifecy[c]le" 2>/dev/null || true
# NOTE: VS Code等の統合ターミナルでは `pkill -f` がこのスクリプト自身にマッチして
# subshell が途中で中断されることがあるため、自己マッチしないパターンにする。
pkill -INT -f "ros2 laun[ch] medical_robot_sim icu_multi_patient.launch.py" 2>/dev/null || true
sleep 1

pkill -TERM -f "__ns:=/p[a]tient_0[1]" 2>/dev/null || true
pkill -TERM -f "__node:=icu_coordinat[o]r" 2>/dev/null || true
pkill -TERM -f "__node:=rule_alert_en[g]ine" 2>/dev/null || true
pkill -TERM -f "rule_alert_engine_lifecy[c]le" 2>/dev/null || true
pkill -TERM -f "ros2 laun[ch] medical_robot_sim icu_multi_patient.launch.py" 2>/dev/null || true
sleep 1

ps -eo pid,stat,cmd | egrep 'medical_robot_sim|ros2 launch' | grep -v egrep || true
```

## 1. ビルドとテストが通る

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

## 2. 前提: Day17 の executable / launch 引数が存在する

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  set -euo pipefail

  # executables
  ros2 pkg executables medical_robot_sim | tee /tmp/day17_pkg_execs.txt
  grep -n "icu_coordinator" /tmp/day17_pkg_execs.txt

  # launch args
  ros2 launch medical_robot_sim icu_multi_patient.launch.py --show-args | tee /tmp/day17_launch_args.txt >/dev/null
  grep -n "enable_coordination" /tmp/day17_launch_args.txt
  grep -n "coord_" /tmp/day17_launch_args.txt

  echo "OK: day17 entrypoints exist"
)
```

期待条件:
- `ros2 pkg executables ...` に `icu_coordinator` が含まれる
- `--show-args` 出力に `enable_coordination` と `coord_` が含まれる

## 3. `enable_coordination:=false` では lifecycle engine は自動で active にならない（後方互換の確認）

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export ROS2CLI_NO_DAEMON=1

  pkill -INT -f "__ns:=/p[a]tient_0[1]" 2>/dev/null || true
  pkill -INT -f "__node:=icu_coordinat[o]r" 2>/dev/null || true
  # launch が途中で落ちた場合、rule_alert_engine が残っていることがある（service が複数応答して誤判定になる）
  pkill -INT -f "__node:=rule_alert_en[g]ine" 2>/dev/null || true
  pkill -INT -f "rule_alert_engine_lifecy[c]le" 2>/dev/null || true
  pkill -INT -f "ros2 laun[ch] medical_robot_sim icu_multi_patient.launch.py" 2>/dev/null || true
  sleep 1

  rm -f /tmp/day17_no_coord.log

  # NOTE: `pkill -f` が統合ターミナルの "bash -c ..." 自身にマッチする事故を避けるため、
  # 文字列として "ros2 launch" がコマンドラインに現れない形（laun"ch"）で実行する。
  ros2 laun"ch" medical_robot_sim icu_multi_patient.launch.py \
    patients:=patient_01 \
    enable_alerts:=true \
    alerts_node_kind:=lifecycle \
    lifecycle_autostart:=false \
    enable_coordination:=false \
    scenario:=flatline \
    > /tmp/day17_no_coord.log 2>&1 &
  LAUNCH_PID=$!

  # /rule_alert_engine が見えるまで待つ（Node not found の誤判定を避ける）
  FOUND=0
  DEADLINE0=$((SECONDS+8))
  while [ "${SECONDS}" -lt "${DEADLINE0}" ]; do
    if ros2 node list 2>/dev/null | grep -q -x "/rule_alert_engine"; then
      FOUND=1
      break
    fi
    sleep 1
  done
  if [ "${FOUND}" -ne 1 ]; then
    echo "ERROR: /rule_alert_engine not found" >&2
    echo "--- /tmp/day17_no_coord.log (tail) ---" >&2
    tail -n 200 /tmp/day17_no_coord.log >&2 || true
    # Stop launch
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
    exit 1
  fi

  # coordinator が無効なので、状態が active でないこと（unconfigured/inactive を許容）
  ros2 lifecycle get /rule_alert_engine | tee /tmp/day17_lc_state_no_coord.txt
  if grep -n -w "active" /tmp/day17_lc_state_no_coord.txt; then
    echo "ERROR: lifecycle state is active although enable_coordination=false and lifecycle_autostart=false" >&2
    RESULT=1
  else
    RESULT=0
  fi

  # Stop launch
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  DEADLINE=$((SECONDS+10))
  while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
    if [ "${SECONDS}" -ge "${DEADLINE}" ]; then
      break
    fi
    sleep 1
  done
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true

  if [ "${RESULT}" -ne 0 ]; then
    echo "--- /tmp/day17_no_coord.log (tail) ---" >&2
    tail -n 200 /tmp/day17_no_coord.log >&2 || true
    exit 1
  fi

  echo "OK: backward compatible (no auto-activate)"
)
```

期待条件:
- `ros2 lifecycle get /rule_alert_engine` が `active` を含まない

## 4. `enable_coordination:=true` で lifecycle engine が active になり、観測ログが出て、停止でスタックトレースが出ない（必須）

完了判断は **Step4 の 1 回の実行**で行う（= Step4 の中で Step5/6 相当も確認する）。
理由: `Step4 → Step5 → Step6` を同じ `/tmp/day17_coord.log`（同一起動）に対して行わないと、
過去のログや残プロセスの影響で誤判定しやすい。

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export ROS2CLI_NO_DAEMON=1

  pkill -INT -f "__ns:=/p[a]tient_0[1]" 2>/dev/null || true
  pkill -INT -f "__node:=icu_coordinat[o]r" 2>/dev/null || true
  pkill -INT -f "__node:=rule_alert_en[g]ine" 2>/dev/null || true
  pkill -INT -f "rule_alert_engine_lifecy[c]le" 2>/dev/null || true
  pkill -INT -f "ros2 laun[ch] medical_robot_sim icu_multi_patient.launch.py" 2>/dev/null || true
  sleep 1

  rm -f /tmp/day17_coord.log

  # NOTE: `pkill -f` が統合ターミナルの "bash -c ..." 自身にマッチする事故を避けるため、
  # 文字列として "ros2 launch" がコマンドラインに現れない形（laun"ch"）で実行する。
  ros2 laun"ch" medical_robot_sim icu_multi_patient.launch.py \
    patients:=patient_01 \
    enable_alerts:=true \
    alerts_node_kind:=lifecycle \
    lifecycle_autostart:=false \
    enable_coordination:=true \
    coord_min_messages_per_patient:=1 \
    coord_ready_timeout_sec:=10 \
    coord_check_period_sec:=1 \
    scenario:=flatline \
    > /tmp/day17_coord.log 2>&1 &
  LAUNCH_PID=$!

  # /rule_alert_engine が見えるまで待つ（Node not found の誤判定を避ける）
  FOUND=0
  DEADLINE0=$((SECONDS+8))
  while [ "${SECONDS}" -lt "${DEADLINE0}" ]; do
    if ros2 node list 2>/dev/null | grep -q -x "/rule_alert_engine"; then
      FOUND=1
      break
    fi
    sleep 1
  done
  if [ "${FOUND}" -ne 1 ]; then
    echo "ERROR: /rule_alert_engine not found" >&2
    echo "--- /tmp/day17_coord.log (tail) ---" >&2
    tail -n 200 /tmp/day17_coord.log >&2 || true
    # Stop launch
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
    exit 1
  fi

  # active になるまで最大 12 秒待つ（起動オーバーヘッドを考慮）
  RESULT=1
  DEADLINE=$((SECONDS+12))
  while [ "${SECONDS}" -lt "${DEADLINE}" ]; do
    if ros2 lifecycle get /rule_alert_engine | tee /tmp/day17_lc_state.txt | grep -q -w "active"; then
      RESULT=0
      break
    fi
    sleep 1
  done

  # Stop launch
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  DEADLINE2=$((SECONDS+10))
  while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
    if [ "${SECONDS}" -ge "${DEADLINE2}" ]; then
      break
    fi
    sleep 1
  done
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true

  if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR: lifecycle did not become active" >&2
    echo "--- /tmp/day17_coord.log (tail) ---" >&2
    tail -n 200 /tmp/day17_coord.log >&2 || true
    exit 1
  fi

  # Step5/6 相当: 観測ログ（イベント）と、停止時のスタックトレース無しを確認
  grep -n "event=coord\\.config" /tmp/day17_coord.log
  grep -n "event=coord\\.lifecycle_set" /tmp/day17_coord.log

  if grep -n "Traceback" /tmp/day17_coord.log; then
    echo "ERROR: Traceback found in /tmp/day17_coord.log" >&2
    exit 1
  fi
  if grep -n "KeyboardInterrupt" /tmp/day17_coord.log; then
    echo "ERROR: KeyboardInterrupt found in /tmp/day17_coord.log" >&2
    exit 1
  fi

  echo "OK: coordinator activated lifecycle engine (and logs validated)"
)
```

期待条件:
- 12 秒以内に `ros2 lifecycle get /rule_alert_engine` が `active` を返す
  - `/tmp/day17_coord.log` に `event=coord.config` が 1 回以上
  - `/tmp/day17_coord.log` に `event=coord.lifecycle_set` が 1 回以上
  - `/tmp/day17_coord.log` に `Traceback` が 0
  - `/tmp/day17_coord.log` に `KeyboardInterrupt` が 0

## 5. 観測ログ（イベント）が出ている（参考: Step4 を分割実行したい場合）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

grep -n "event=coord\.config" /tmp/day17_coord.log

grep -n "event=coord\.lifecycle_set" /tmp/day17_coord.log
```

期待条件:
- `coord.config` が 1 回以上
- `coord.lifecycle_set` が 1 回以上

## 6. Ctrl+C 停止でスタックトレースが出ない（参考: Step4 を分割実行したい場合）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

grep -n "Traceback" /tmp/day17_coord.log || true
grep -n "KeyboardInterrupt" /tmp/day17_coord.log || true
```

期待条件:
- `Traceback` が 0
- `KeyboardInterrupt` が 0
