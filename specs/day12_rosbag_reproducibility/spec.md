# Day12 rosbagによる再現性検証 実装仕様（spec）

## Goal

rosbag2 を用いて、次を **コマンドで再現可能** にする。

- `/patient_XX/patient_vitals` の記録（record）
- `ros2 bag info` による bag 内容の確認（topic/型/count）
- sensor を起動せずに bag 入力で monitor/alerts を動かす（replay）
- replay 中に `/patient_01/alerts` が生成されることを観測できる

## Design

### 最小ループ（record → info → play → observe）

1. live 実行（`vital_sensor` が publish）を行う
2. `ros2 bag record` で vitals（必要なら alerts も）を bag に保存
3. `ros2 bag info` で topic と Message Count を確認
4. replay 用パイプライン（sensor 無し）を起動
5. `ros2 bag play` で vitals を再生し、`rule_alert_engine` が `/patient_XX/alerts` を生成することを観測

### Replay パイプライン（sensor 無し）

`icu_multi_patient.launch.py` は常に `vital_sensor` を起動するため、
bag を入力にする replay と衝突する（publisher が二重になる）。

この問題を避けるため、**replay 専用 launch** を新規追加する。

- 起動するノード:
  - `icu_monitor`
  - `rule_alert_engine`（`enable_alerts:=true` のとき）
- 起動しないノード:
  - `vital_sensor`

`patients`、Day7 の `rules_path`、Day8 の QoS、Day9 の lifecycle 切替など、
既存 `icu_multi_patient.launch.py` と同等の引数を持ち、受け入れコマンドが短くなることを優先する。

### bag 記録対象

必須:
- `/patient_01/patient_vitals`

任意:
- `/patient_01/alerts`

（補足）alerts を同じ bag に入れる場合、replay 中に `ros2 bag play` が alerts も publish してしまうため、
「replay によって生成された alerts の観測」が曖昧になり得る。
本Dayの必須 acceptance は vitals のみを対象とし、alerts 同時記録は学習用に留める。

### Automated test 方針（最小）

rosbag 自体の統合テスト（実際の record/play）を CI で安定化させるのは重いため、本Dayの必須にはしない。
代わりに、replay/record の手順で共通に使う「patients CSV → topics 組み立て」の純粋関数を追加し、pytest で固定する。

- `build_record_topics(patients: list[str], vitals_topic: str, include_alerts: bool) -> list[str]`
  - 例: `['/patient_01/patient_vitals', '/patient_01/alerts']`
  - 前後空白、先頭スラッシュ、空要素の扱いが安定していること

## Affected files

- `src/medical_robot_sim/launch/icu_replay.launch.py`（新規）
- `src/medical_robot_sim/medical_robot_sim/rosbag_topics.py`（新規: topic 組み立て純粋関数）
- `src/medical_robot_sim/test/test_rosbag_topics.py`（新規）
- `docs/day12_rosbag_reproducibility.md`
- `specs/day12_rosbag_reproducibility/spec.md`
- `specs/day12_rosbag_reproducibility/tasks.md`
- `specs/day12_rosbag_reproducibility/acceptance.md`

## Constraints

- topic 契約（`/patient_XX/patient_vitals`, `/patient_XX/alerts`）を変更しない
- message 定義（`medical_interfaces`）を変更しない
- 既定挙動は後方互換（通常起動は従来通り）
- Ctrl+C clean shutdown を維持する
- 変更は rosbag による再現性確保に限定し、ルール評価ロジック自体は変更しない

## Non-goals

- rosbag を使った “完全一致” 比較（bitwise 同一性）を必須にすること
- QoS override（`--qos-profile-overrides-path` 等）の全面導入
- 大規模な launch 再設計（既存 `icu_multi_patient.launch.py` の置換）
- bag の長期保管/暗号化/監査（医療機器レベルの運用要件）
