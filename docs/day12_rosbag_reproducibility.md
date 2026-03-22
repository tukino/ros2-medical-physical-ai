# Day12: rosbagによる再現性検証（day12_rosbag_reproducibility）

## Purpose

本Dayの目的は、ROS2 多患者バイタル監視シミュレーションに対して **rosbag による「記録→再生→再検証」** のループを確立し、
同じ入力（vitals）で同じ下流挙動（monitor/alerts）が観測できる状態を作ること。

- 実験や障害（Day10 Fault Injection）の入力条件を bag として保存できる
- Day11 のイベントログ（`alerts.emit` / `monitor.patient_state` / `vitals.fault_config` 等）と組み合わせて、
  「何が入力で、なぜそう見えたか」を第三者が追える
- ルール外部化（Day7 YAML）・QoS（Day8）・Lifecycle（Day9）を含む “システムとしての再現性” を底上げする

## Background

医療監視や Physical AI の入力系では、問題が起きたときに「その時の入力」を復元できるかが切り分け速度を左右する。

- Day10 で「落ちる/遅れる/止まる」を作れるようになった
- Day11 で「それが何だったか」をログで説明できるようになった
- しかし、ログだけでは “同じ入力で再検証する” ことができない

Day12 は rosbag を使い、**入力データを保存→オフライン再生→同じ評価ロジックで再実行** を可能にする。

## Why this day matters in the roadmap

- Day13（実センサー接続）以降は「実機由来の入力」が混ざり、偶然性が増える
  - bag があれば、実験を固定入力で反復できる
- Day8（QoS）や Day9（Lifecycle）で、挙動が「通信条件」「状態遷移」に依存するようになった
  - bag は “データ” を固定し、残る変数（QoS/状態/ルール）を切り分けできる
- Day16（異常検知AI）などの後半フェーズでも、学習データ・評価データの固定化に直結する

## Target topics/components

### 対象 topic（契約は変更しない）

- vitals: `/patient_XX/patient_vitals`
- alerts: `/patient_XX/alerts`

本Dayは「bag 記録/再生の運用」を扱うため、topic 名・型の変更は行わない。

### 対象コンポーネント

- `vital_sensor`（入力源。録る側で使用）
- `icu_monitor`（観測対象。再生側で使用）
- `rule_alert_engine` / `rule_alert_engine_lifecycle`（観測対象。再生側で使用）
- rosbag2（`ros2 bag record/info/play`）

## Design policy

- **後方互換**: 既存の起動（`icu_multi_patient.launch.py`）を壊さない
- **小さく増分**: 本Dayで必須にするのは「記録→再生→観測」ループの確立だけ
- **観測可能**: `ros2 bag info` と `ros2 topic echo`、ログの `grep` で合否判定できる
- **クリーン停止維持**: Ctrl+C（SIGINT）でスタックトレースを出さない
- **患者スケール対応**: まずは `patient_01` で成立させ、CSV で増やせる設計にする

## Implementation requirements

### 1) 記録の最小要件（record）

- `ros2 bag record` で `/patient_XX/patient_vitals` を記録できる
- `ros2 bag info <bag_dir>` で、
  - 対象 topic が列挙される
  - `Count`（Message Count）が **0 より大きい**

（任意）同じ bag に `/patient_XX/alerts` も含められるようにする。

### 2) 再生の最小要件（play）

- `ros2 bag play <bag_dir>` で vitals が publish される
- bag 再生だけでは alerts は「再生されるだけ」なので、
  **再生された vitals を購読して alerts を生成するパイプライン** を用意する

このために、次のいずれかを用意する（推奨はA）:

- A) **replay 用 launch**（推奨）
  - `icu_monitor` と `rule_alert_engine`（必要なら lifecycle）だけを起動
  - `vital_sensor` は起動しない（bag 入力と衝突させない）

- B) `ros2 run` で monitor/alerts を個別起動（最小だが手順が長くなりがち）

### 3) 再現性の最小要件（verification）

本Dayの「再現性検証」は次を満たせば合格とする:

- 記録 bag の `vitals` に Message Count > 0 がある
- その bag を再生し、下流（`rule_alert_engine`）が `/patient_01/alerts` を **生成できる**
  - 例: `ros2 topic echo /patient_01/alerts --once` が再生中に成功する

（発展）Day11 の `alerts.emit` ログを使い、
「録った入力で、同じ rule_id が発火した」ことを grep で観測できる。

### 4) QoS と再生の注意点（設計に含める）

- QoS（Day8）を変えると、record/play と subscriber の組合せで不一致が起き得る
- 本Dayでは “高度な QoS override” までを必須にはしない
  - ただし、トラブル時の切り分け手順（`ros2 topic info -v` で接続/Publisher/Subscriber を確認）は docs に明記する

## Files expected to change

- `src/medical_robot_sim/launch/icu_replay.launch.py`（新規: sensor 無しの replay パイプライン）
- `src/medical_robot_sim/scripts/rosbag_record_vitals.sh`（必要なら: docs 整合の微調整のみ）
- `src/medical_robot_sim/scripts/rosbag_play_vitals.sh`（同上）
- `src/medical_robot_sim/test/test_rosbag_topics.py`（新規: topic 組み立てユニットテスト）
- `docs/day12_rosbag_reproducibility.md`（本ファイル）
- `specs/day12_rosbag_reproducibility/spec.md`
- `specs/day12_rosbag_reproducibility/tasks.md`
- `specs/day12_rosbag_reproducibility/acceptance.md`

## Reproduction / validation steps

- 受け入れ手順（コピペ）: `specs/day12_rosbag_reproducibility/acceptance.md`

### 最短コピペ（Quickstart: 5分で観測）

次は「bag の記録→info」を最短で確認する入口。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1) 記録用にシステム起動（別プロセス）
rm -f /tmp/day12_live.log
ros2 launch medical_robot_sim icu_multi_patient.launch.py \
  patients:=patient_01 \
  enable_alerts:=true \
  > /tmp/day12_live.log 2>&1 &
LAUNCH_PID=$!

sleep 3

# 2) vitals を短時間だけ記録
BAG_DIR="/tmp/day12_vitals_$(date +%Y%m%d_%H%M%S)"
(timeout -s INT 6s ros2 bag record -o "${BAG_DIR}" /patient_01/patient_vitals) >/tmp/day12_bag_record.log 2>&1 || true

# 3) bag の中身を確認（Count > 0）
ros2 bag info "${BAG_DIR}" | tee /tmp/day12_bag_info.txt
grep -n "/patient_01/patient_vitals" /tmp/day12_bag_info.txt

# 4) 停止
kill -INT "${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true
```

次に replay は acceptance 手順に従う（sensor 無し launch + bag play）。

## Success criteria

- 1患者の vitals を rosbag に記録し、`ros2 bag info` で topic と count を確認できる
- 記録した bag を再生し、sensor を起動せずに monitor/alerts パイプラインが入力を受けられる
- replay 中に `/patient_01/alerts` が観測できる（生成された alert を 1 件以上受信）
- Ctrl+C / SIGINT でスタックトレース無しに停止できる

## Learning path

### Quickstart（最短で動かす）

- まずは本ページの Quickstart で「record→info」を確認
- 次に acceptance の replay 手順で「play→alerts 生成」を確認

### 読み方（原因追跡の順番）

1. `ros2 bag info <bag_dir>`
   - 期待 topic があるか、Count が 0 ではないか
2. `ros2 topic info -v /patient_01/patient_vitals`
   - play 中に Publisher が 1 以上になっているか
3. replay 側ノードのログ
   - Day11 の `alerts.emit` / `monitor.patient_state` が出るか
4. QoS が疑わしい場合
   - `vitals_qos_*` / `alerts_qos_*` を “既定値” に戻して再現する

### 必須（受け入れ）と任意（学習）

- 必須: 受け入れコマンドで record→info→play→alerts 生成が観測できる
- 任意: Day11 のイベントログ（`alerts.emit` 等）を使って、同一入力に対する発火内容を比較する

## Relevance to medical / BCI / Physical AI context

- 医療監視では「インシデント時の入力」を保存し、後から同条件で検証できることが重要
- BCI/EEG のようなノイズが多い入力は、固定入力を作れるとアルゴリズム改善が進む
- Physical AI は入力遅延・欠落が挙動に直結するため、
  “同じ入力を何度も流す” ことで制御やアラートの頑健性を評価しやすくなる

## Connection to the next day

- Day13（実センサー）: 実世界の入力を bag に落とし、シミュレーション環境で再生して切り分けする
- Day10/11 と合わせて、
  「障害条件（params）＋入力（bag）＋観測（ログ）」の三点セットで再現性を担保する
