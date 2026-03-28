# Day14 Edgeデバイス（Jetson） タスク（tasks）

## Implementation tasks

- [x] `src/medical_robot_sim/scripts/jetson_env_report.sh` を新規追加する（Jetson 以外は N/A で exit 0）
- [x] `src/medical_robot_sim/launch/icu_edge.launch.py` を新規追加する（単一患者 + mock 既定）
- [x] `icu_edge.launch.py` で `enable_alerts:=true` のとき alerts を生成できるように既存ノードを配線する
- [x] （任意）`platform_info.py` に Jetson 判定/整形の純粋関数を追加し、必要ならノード起動時ログに反映する

## Validation tasks

- [x] `colcon build --symlink-install` が通る（x86_64）
- [x] `colcon test --packages-select medical_robot_sim` が通る（x86_64）
- [x] `icu_edge.launch.py` 起動中に `/patient_01/patient_vitals` を `--once` で受信できる
- [x] `icu_edge.launch.py`（`enable_alerts:=true`）起動中に `/patient_01/alerts` を `--once` で受信できる
- [x] Ctrl+C / SIGINT で停止してスタックトレースが出ない
- [ ] （任意）Jetson（aarch64）で同じ validation を実行し、`jetson_env_report.sh` を添付できる

## Automated tests

- [x] （任意機能を入れる場合）`src/medical_robot_sim/test/test_platform_info.py` を追加し、純粋関数の境界値をテストする
- [x] `colcon test --packages-select medical_robot_sim` が成功する

## Docs update tasks

- [x] `docs/day14_jetson.md` を更新し、Quickstart と acceptance への導線を最新化する
- [x] README には詳細を重複させず、Day14 の docs/specs へのリンク導線のみ追記する
