# Day14: Edgeデバイス（Jetson）（day14_jetson）

## Purpose

本Dayの目的は、ROS2 多患者バイタル監視シミュレーション（センサー→monitor→ルールベース alert）を **Edge デバイス（Jetson）上で動作させるための最小構成**を設計・実装できる状態にすること。

- Jetson（ARM64）で `colcon build` → 起動 → topic 観測まで到達する
- Day13 の実センサー入力ノード（`hw_vital_sensor`）を **Edge で扱える運用形**にする（実機が無い場合は mock で受け入れ完走）
- Day10-12（fault/logging/rosbag）で整えた「再現性・観測可能性」を、Edge 実行でも維持する

## Background

Edge 実行は単に「別PCで動かす」ではなく、次の制約が一気に増える:

- CPU/メモリ/ストレージ制約（ログ肥大、バッファ、python の負荷）
- I/O 制約（I2C/SPI 権限、デバイスパス、udev）
- 再現性の劣化（環境差、バージョン差）

本プロジェクトは「学習」ではなく「第三者が再現できる成果物」を重視するため、Edge 実行も **コマンドで再現できる形**（env report、起動、観測、停止）に揃える。

## Why this day matters in the roadmap

- Day13（実センサー）で「現実世界の入力」を扱えるようになったが、実運用では多くが Edge に寄る
- Day12（rosbag）により、Edge で採取した入力を bag 化してオフライン検証できる
- Day15（EEG/BCI）では、入力がさらに高頻度/ノイジーになり、Edge での観測・負荷・遅延が重要になる

Day14 は「Edge に載せた瞬間に壊れる」典型（依存、権限、リソース、起動/停止）を早期に潰す。

## Target topics/components

### 対象 topic（契約は変更しない）

- vitals: `/patient_XX/patient_vitals`
- alerts: `/patient_XX/alerts`

本Dayは「実行環境（Edge）への移行」を扱うため、topic 名・message 定義の変更は行わない。

### 対象コンポーネント

- `icu_monitor`
- `rule_alert_engine` / `rule_alert_engine_lifecycle`
- `vital_sensor`（後方互換として維持）
- `hw_vital_sensor`（Day13 の実センサー/ mock）
- launch / scripts（Edge 向け運用の薄いラッパ）

## Design policy

- **後方互換**: 既存の起動（`icu_multi_patient.launch.py` 等）を壊さない
- **小さく増分**: 必須は「Jetson で動き、観測でき、クリーンに止まる」まで
- **観測可能**: `ros2 topic echo` とログ `grep` で合否判定できる
- **環境依存は optional**: Jetson 実機や I2C/SPI が無い場合でも mock で受け入れ完走
- **クリーン停止維持**: Ctrl+C（SIGINT）でスタックトレース無し（Day3 方針）

## Implementation requirements

### 1) Edge 環境レポート（必須）

Jetson 上で「何の環境で動かしたか」を機械的に残す。

- `src/medical_robot_sim/scripts/jetson_env_report.sh`（新規）
  - `uname -m` / OS / ROS distro / Python / Jetson release（あれば）を出力
  - Jetson 以外では `N/A` として終了 0（受け入れを落とさない）

### 2) Edge 起動の最小パイプライン（必須）

Jetson で最短起動・最短観測できる入口を追加する。

- `icu_edge.launch.py`（新規: 推奨）
  - まずは単一患者 `patient_01` を既定にする（patients 増加は任意）
  - 入力は **既定で mock**（Day13）にし、Jetson でも CI でも同じ acceptance が通るようにする
  - `enable_alerts` を true にしたときに `/patient_01/alerts` が生成される（既存 engine を使う）

（補足）この launch は “Edge で動かしやすい defaults” を持つ薄いラッパに留め、既存ノードの内部ロジックは再設計しない。

### 3) observability（必須）

Edge での切り分け速度を上げるため、起動ログへ最小の環境情報を残す。

- スクリプトでの環境レポート（必須）に加え、（任意）ノード起動時ログで `event=platform.info ...` を 1 行出す
  - これは “実装の都合が良い場合のみ” で良い（必須にしない）

### 4) Jetson/Edge 特有の optional 項目

- `tegrastats` で CPU/GPU/温度/メモリの概要を取る（未導入なら N/A）
- I2C/SPI のデバイス存在・権限確認（未接続なら N/A）

## Files expected to change

- `src/medical_robot_sim/scripts/jetson_env_report.sh`（新規）
- `src/medical_robot_sim/launch/icu_edge.launch.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/platform_info.py`（新規: 任意。pure function 推奨）
- `src/medical_robot_sim/test/test_platform_info.py`（新規: 任意部分を入れるなら必須）
- [docs/day14_jetson.md](docs/day14_jetson.md)（本ファイル）
- [specs/day14_jetson/spec.md](specs/day14_jetson/spec.md)
- [specs/day14_jetson/tasks.md](specs/day14_jetson/tasks.md)
- [specs/day14_jetson/acceptance.md](specs/day14_jetson/acceptance.md)

## Reproduction / validation steps

- 受け入れ手順（コピペ）: [specs/day14_jetson/acceptance.md](specs/day14_jetson/acceptance.md)

### 最短コピペ（Quickstart: 5分で観測）

この Quickstart は「Edge（Jetson）でも、手元PCでも」同じ確認ができる入口にする。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# すでにビルド済みなら install/setup.bash を source するだけ
# 初回はビルドが必要（Jetson は時間がかかる可能性あり）
colcon build --symlink-install
source install/setup.bash

# Edge 用の最小 launch（Day14 で追加する想定）
rm -f /tmp/day14_quick.log
timeout -s INT 20s ros2 launch medical_robot_sim icu_edge.launch.py \
  patient:=patient_01 \
  driver:=mock \
  mock_scenario:=spo2_drop \
  enable_alerts:=true \
  > /tmp/day14_quick.log 2>&1 || true

# 観測の痕跡（どれかが出る）
grep -n "event=vitals\." /tmp/day14_quick.log || true
grep -n "alerts\.emit" /tmp/day14_quick.log || true

echo "OK: day14 quickstart ran (see /tmp/day14_quick.log)"
```

## Success criteria

- Jetson（aarch64）で `colcon build` と `colcon test` が通る（少なくとも `medical_robot_sim`）
- `icu_edge.launch.py` で起動し、`/patient_01/patient_vitals` を 1 件以上観測できる
- `enable_alerts:=true` のとき `/patient_01/alerts` を 1 件以上観測できる（mock シナリオで可）
- Ctrl+C / SIGINT でスタックトレース無しに停止できる
- Jetson での環境レポート（arch/OS/Jetson release 等）をファイルに残せる

## Learning path

### Quickstart（最短で動かす）

- まずは Quickstart で `icu_edge.launch.py` を起動し、ログに痕跡が残ることを確認
- 次に acceptance で `topic echo --once` による機械判定（vitals/alerts）を行う

### 読み方（原因追跡の順番）

1. `scripts/jetson_env_report.sh` の出力
   - arch / Jetson release / ROS distro を固定化して記録
2. `/tmp/day14_*.log` の `event=` 行
   - vitals の入力（mock / i2c / spi）設定
   - alerts の emit があるか
3. `ros2 topic info -v`（必要な場合）
   - Publisher/Subscriber の数が期待通りか（多重起動の切り分け）
4. （Jetsonのみ・任意）`tegrastats`
   - 高負荷/サーマルで落ちていないか、リソース逼迫の兆候

### 必須（受け入れ）と任意（学習）

- 必須: mock 入力で Edge launch が動き、vitals/alerts を観測できる
- 任意: Jetson 実機での `tegrastats` 記録、I2C/SPI 実入力での起動

## Relevance to medical / BCI / Physical AI context

- 医療監視は「現場（Edge）で落ちない」ことが価値であり、環境差の吸収と観測可能性が重要
- BCI/EEG は入力の帯域やノイズが大きく、Edge 側での負荷・遅延・ログ設計が後工程の品質に直結する
- Physical AI は “入力→判断→行動” の閉ループに進むほど、Edge 上の再現性（入力固定/ログ/rosbag）が必須になる

## Connection to the next day

- Day15（EEG/BCI）: Jetson 上で新しい入力デバイスや高頻度入力を扱うため、まず Day14 で「Edge 起動・観測・停止」の土台を固める
- Day12（rosbag）: Jetson で採取した入力を bag に落とし、手元PCで再生・検証する運用へ繋げる
