# Day14 Edgeデバイス（Jetson） 実装仕様（spec）

## Goal

- Jetson（aarch64）上で `medical_robot_sim` をビルドして起動できる（依存/権限の問題を手順化して切り分けできる）
- Edge 向けの最小起動入口（launch + env report）を追加し、vitals/alerts が **観測可能**である
- Jetson 実機や I2C/SPI が無い環境でも、mock（Day13）で受け入れが完走する

## Design

### 1) Edge 実行は “薄い運用レイヤ” で吸収する

Edge 特有の差分（環境差・観測・起動の短縮）を、できるだけ以下に閉じ込める。

- `scripts/`（環境レポート、optional チェック）
- `launch/`（Edge 向け defaults を持つ起動ラッパ）

センサー/monitor/ルールエンジンの評価ロジック自体は Day14 で再設計しない。

### 2) `jetson_env_report.sh`（必須）

- 出力: arch/OS/ROS distro/Python などの最小セット
- Jetson 判定:
  - `/etc/nv_tegra_release` があれば Jetson とみなし、内容を出す
  - 無ければ `N/A: not a Jetson` を出して終了 0
- 受け入れは、このスクリプトが “Jetson 以外でも落ちない” ことを要求する

### 3) `icu_edge.launch.py`（必須）

- 目的: 起動コマンドを短くし、Edge での起動/観測/停止を最短化する
- 既定: `patient_01`、入力は `driver:=mock`（Day13）
- `enable_alerts:=true` のとき、`/patient_01/alerts` が生成されること
- topic 契約は既存のまま:
  - `/patient_XX/patient_vitals`
  - `/patient_XX/alerts`

### 4) テスト方針（最小）

Edge 実機を CI 前提にしない。
代わりに、Edge 向けに追加する “判定/整形の純粋関数” を用意して pytest で固定する。

- 例: `platform_info.py` に `parse_nv_tegra_release(text: str) -> dict` のような純粋関数を置く
- Jetson 特有のファイルを直接読む処理は I/O として分離し、純粋関数部分だけをテストする

## Affected files

- `src/medical_robot_sim/scripts/jetson_env_report.sh`（新規）
- `src/medical_robot_sim/launch/icu_edge.launch.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/platform_info.py`（新規: 任意、入れるならテストも追加）
- `src/medical_robot_sim/test/test_platform_info.py`（新規: 任意部分を入れるなら必須）
- `docs/day14_jetson.md`
- `specs/day14_jetson/spec.md`
- `specs/day14_jetson/tasks.md`
- `specs/day14_jetson/acceptance.md`

## Constraints

- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）を変更しない
- message 定義（`medical_interfaces`）を変更しない
- 既存 launch / node の後方互換を維持する（既存手順が壊れない）
- Ctrl+C clean shutdown を維持する
- Jetson 実機の有無で acceptance の合否が不安定にならないよう、optional は N/A 判定を必ず用意する

## Non-goals

- Docker / cross-compile / systemd 常駐などの本格デプロイ（本Dayでは扱わない）
- Jetson の電源モード最適化・カーネル調整（学習用 optional に留める）
- パフォーマンスベンチマークの厳密比較（数値目標の設定）
- 実センサーの信号処理高度化（Day13 の範囲外）
