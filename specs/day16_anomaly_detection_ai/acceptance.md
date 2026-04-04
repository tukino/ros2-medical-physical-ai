# Day16 異常検知AI（advisory layer）受け入れ基準（acceptance）

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

pkill -INT -f "/install/medical_robot_sim/lib/medical_robot_sim/" 2>/dev/null || true
pkill -INT -f "ros2 launch medical_robot_sim" 2>/dev/null || true
sleep 1

pkill -TERM -f "/install/medical_robot_sim/lib/medical_robot_sim/" 2>/dev/null || true
pkill -TERM -f "ros2 launch medical_robot_sim" 2>/dev/null || true
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

## 2. 前提: Day16 の executable / launch 引数が存在する

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  set -euo pipefail

  # executables
  ros2 pkg executables medical_robot_sim | tee /tmp/day16_pkg_execs.txt
  grep -n "advisory_publisher" /tmp/day16_pkg_execs.txt

  # launch args
  ros2 launch medical_robot_sim icu_multi_patient.launch.py --show-args | tee /tmp/day16_launch_args.txt >/dev/null
  grep -n "enable_advisories" /tmp/day16_launch_args.txt
  grep -n "advisories_window_size" /tmp/day16_launch_args.txt

  echo "OK: day16 entrypoints exist"
)
```

期待条件:
- `ros2 pkg executables ...` に `advisory_publisher` が含まれる
- `--show-args` 出力に `enable_advisories` が含まれる

## 3. `enable_advisories:=true` で `/patient_01/advisories` が publish される（必須）

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export ROS2CLI_NO_DAEMON=1

  # 前回の残プロセスがあると `--once` 取りこぼし/誤判定の原因になるため掃除
  pkill -INT -f "__ns:=/patient_01" 2>/dev/null || true
  pkill -INT -f "/install/medical_robot_sim/lib/medical_robot_sim/icu_monitor" 2>/dev/null || true
  pkill -INT -f "ros2 launch medical_robot_sim icu_multi_patient.launch.py" 2>/dev/null || true
  sleep 1

  rm -f /tmp/day16_pipeline.log
  rm -f /tmp/day16_advisory_once.txt

  # NOTE: flatline は edge-trigger で 1 回しか出ないため、subscriber は先に待ち受ける
  #       （遅れて subscribe すると取りこぼしうる）
  echo "INFO: starting subscriber (timeout=25s)"
  timeout 25s ros2 topic echo --once \
    --qos-history keep_last \
    --qos-depth 10 \
    --qos-reliability reliable \
    --qos-durability volatile \
    /patient_01/advisories medical_interfaces/msg/Alert \
    > /tmp/day16_advisory_once.txt 2>&1 &
  ECHO_PID=$!

  echo "INFO: launching day16 pipeline (flatline -> advisories)"
  ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    patients:=patient_01 \
    enable_alerts:=false \
    enable_advisories:=true \
    advisories_window_size:=4 \
    scenario:=flatline \
    > /tmp/day16_pipeline.log 2>&1 &
  LAUNCH_PID=$!

  echo "INFO: waiting for subscriber to receive one message"
  wait "${ECHO_PID}"
  ECHO_EXIT=$?
  if [ "${ECHO_EXIT}" -ne 0 ]; then
    echo "WARN: subscriber exited with code ${ECHO_EXIT}" >&2
  fi

  echo "INFO: stopping launch (pid=${LAUNCH_PID})"
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  DEADLINE=$((SECONDS+10))
  while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
    if [ "${SECONDS}" -ge "${DEADLINE}" ]; then
      break
    fi
    sleep 1
  done
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "WARN: launch still running; sending SIGTERM" >&2
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "WARN: launch still running; sending SIGKILL" >&2
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true

  # launch 停止が不完全な場合に備えて、namespace 配下の残ノードを掃除
  pkill -INT -f "__ns:=/patient_01" 2>/dev/null || true
  pkill -INT -f "/install/medical_robot_sim/lib/medical_robot_sim/icu_monitor" 2>/dev/null || true
  sleep 1

  # 受信内容の最小確認
  if grep -n "kind" /tmp/day16_advisory_once.txt | grep -n "advisory"; then
    echo "OK: advisories published"
  else
    echo "ERROR: advisory output missing kind=advisory" >&2
    echo "--- /tmp/day16_advisory_once.txt ---" >&2
    cat /tmp/day16_advisory_once.txt >&2 || true
    echo "--- /tmp/day16_pipeline.log (tail) ---" >&2
    tail -n 200 /tmp/day16_pipeline.log >&2 || true
    exit 1
  fi
)
```

期待条件:
- `/patient_01/advisories` を 25 秒以内に 1 件受信できる
- `/tmp/day16_advisory_once.txt` に `kind: advisory` が含まれる

## 4. 観測ログ（イベント）が出ている（必須）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

grep -n "event=advisory\.config" /tmp/day16_pipeline.log
grep -n "event=advisory\.publish" /tmp/day16_pipeline.log
```

期待条件:
- `advisory.config` が 1 回以上出る
- `advisory.publish` が 1 回以上出る

## 5. 後方互換: `enable_advisories:=false` で `/patient_01/advisories` が増えない（最小）

```bash
(
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  export ROS2CLI_NO_DAEMON=1

  # 前回の残プロセスがあると誤判定になるため掃除
  pkill -INT -f "__ns:=/patient_01" 2>/dev/null || true
  pkill -INT -f "/install/medical_robot_sim/lib/medical_robot_sim/icu_monitor" 2>/dev/null || true
  pkill -INT -f "ros2 launch medical_robot_sim icu_multi_patient.launch.py" 2>/dev/null || true
  sleep 1

  rm -f /tmp/day16_noadv.log

  ros2 launch medical_robot_sim icu_multi_patient.launch.py \
    patients:=patient_01 \
    enable_alerts:=false \
    enable_advisories:=false \
    > /tmp/day16_noadv.log 2>&1 &
  PID=$!

  sleep 3

  # 1) Node が起動していないこと（最小の確実判定）
  if timeout 5s ros2 node list | grep -E "^/patient_01/advisory_publisher$"; then
    echo "ERROR: advisory_publisher node exists although enable_advisories=false" >&2
    RESULT=1
  else
    RESULT=0
  fi

  # 2) Topic が見える場合でも、publisher count が 0 であること
  #    （topic list は subscriber だけでも見える/キャッシュされる可能性があるため）
  if timeout 5s ros2 topic info -v /patient_01/advisories >/tmp/day16_noadv_topicinfo.txt 2>&1; then
    PUB_COUNT=$(grep -E "^Publisher count:" /tmp/day16_noadv_topicinfo.txt | awk '{print $3}' | head -n 1)
    if [ "${PUB_COUNT:-0}" -ne 0 ]; then
      echo "ERROR: advisories publisher count != 0 although enable_advisories=false" >&2
      echo "--- /tmp/day16_noadv_topicinfo.txt ---" >&2
      cat /tmp/day16_noadv_topicinfo.txt >&2 || true
      RESULT=1
    fi
  fi

  # Stop launch reliably (avoid hanging on wait)
  kill -INT "$PID" 2>/dev/null || true
  DEADLINE=$((SECONDS+10))
  while kill -0 "$PID" 2>/dev/null; do
    if [ "${SECONDS}" -ge "${DEADLINE}" ]; then
      break
    fi
    sleep 1
  done
  if kill -0 "$PID" 2>/dev/null; then
    kill -TERM "$PID" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "$PID" 2>/dev/null; then
    kill -KILL "$PID" 2>/dev/null || true
  fi
  wait "$PID" 2>/dev/null || true

  if [ "${RESULT}" -ne 0 ]; then
    exit 1
  fi

  echo "OK: backward compatible"
)
```

期待条件:
- `enable_advisories:=false` の起動で `/patient_01/advisories` が見つからない

## 6. Ctrl+C 停止でスタックトレースが出ない（最小）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

grep -n "Traceback" /tmp/day16_pipeline.log || true
grep -n "KeyboardInterrupt" /tmp/day16_pipeline.log || true
```

期待条件:
- `Traceback` が出ない
- `KeyboardInterrupt` が出ない
