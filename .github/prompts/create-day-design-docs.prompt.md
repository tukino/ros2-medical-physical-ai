---
name: create-day-design-docs
description: "ロードマップと既存リポジトリ文脈に沿って、指定Dayの設計ドキュメントと実装仕様(specs)一式を日本語で生成する（DAY_NAMEは8/day8/Day8も可）"
argument-hint: "例:\nDAY: 8\nDAY_THEME: QoS設計\n\nまたは:\nDAY_NAME: day8_qos\nDAY_THEME: QoS設計"
agent: agent
---

既存ロードマップとリポジトリ文脈に沿って、指定Dayの「設計ドキュメント」と「実装仕様(specs)」を作成してください。

出力言語は **日本語** とし、コマンドやファイルパスなどのリテラルは必要に応じてそのまま表記してください。

## Target day
DAY: <8 | day8 | Day8 など（任意）>
DAY_NAME: <day8_qos のような完全なDay名（任意）>
DAY_THEME: <このDayのテーマ>

Examples:
- DAY: 8
- DAY_THEME: QoS設計
- DAY_NAME: day8_qos
- DAY_THEME: QoS設計
- DAY_NAME: day9_lifecycle
- DAY_THEME: Lifecycle node introduction

---

## 入力の正規化（重要）
次のいずれの形式でも受け付け、内部で正規化してください。

- `DAY_NAME` が `day8_qos` のような **完全なDay名** の場合: そのまま採用。
- `DAY_NAME` または `DAY` が `8` / `day8` / `Day8` のような **Day番号のみ** の場合:
  - `docs/` と `specs/` を検索し、該当Dayに対応する候補（例: `day8_qos`）を列挙。
  - 候補が1つに確定する場合のみ、それを正規の `DAY_NAME` として採用。
  - 候補が複数ある・見つからない場合は、作業を始める前に **最大3つまで** 質問して確定させる。

以降の生成物のパスには、正規化後の `DAY_NAME` を使用してください。

## DAY_THEME の補完（重要）
`DAY` が `8` / `day8` / `Day8` のような Day番号指定で、`DAY_THEME` が未指定または空の場合は、**必ず** `docs/PLAN.md` を参照してそのDayのテーマを特定し、`DAY_THEME` として採用してください。

- `docs/PLAN.md` にそのDayのテーマが明記されている場合は、その文言を優先（例: Day8 → QoS設計）。
- `docs/PLAN.md` だけで曖昧な場合は、既存の `docs/day<番号>_*.md` のファイル名や内容から、最も自然なテーマを **1つ** 提案して `DAY_THEME` として採用。
- どうしても決められない場合のみ、作業開始前に **最大1つ** の質問で確定させる（質問は最小限）。

## Mandatory references
書き始める前に、存在する場合は必ず読み、内容に整合させてください:

1. `docs/PLAN.md`
2. `.github/copilot-instructions.md`
3. Existing `docs/day*.md`
4. Existing `specs/day*/spec.md`, `tasks.md`, `acceptance.md`
5. Current source tree under `src/`

`docs/PLAN.md` と矛盾する方針を新規に捏造しないでください。

---

## Goal
指定Dayについて、次の成果物を生成してください:

1. `docs/<DAY_NAME>.md`
2. `specs/<DAY_NAME>/spec.md`
3. `specs/<DAY_NAME>/tasks.md`
4. `specs/<DAY_NAME>/acceptance.md`

出力は「実装者が迷わず実装できる」粒度で、現リポジトリの状態と整合する内容にしてください。

---

## Authoring rules

### General
- これは **実装前の設計作業** として扱う
- コーディングエージェントがそのまま実装できる具体性にする
- 既存Dayとの整合を維持する
- スコープは狭く、増分であること
- 後方互換を優先する
- 要件にない部分を黙って再設計しない

### `docs/{{DAY_NAME}}.md`
Must include:
- Purpose
- Background
- Why this day matters in the roadmap
- Target topics/components
- Design policy
- Implementation requirements
- Files expected to change
- Reproduction / validation steps
- Success criteria
- Relevance to medical / BCI / Physical AI context
- Connection to the next day

この文書は **what** と **why** の両方を説明してください。

### `specs/{{DAY_NAME}}/spec.md`
Must include:
- Goal
- Design
- Affected files
- Constraints
- Non-goals

このファイルは実装者向けです。

### `specs/{{DAY_NAME}}/tasks.md`
Must include:
- Checkbox task list
- Implementation tasks
- Validation tasks
- Docs update tasks
- Keep tasks concrete and observable

### `specs/{{DAY_NAME}}/acceptance.md`
Must include:
- Observable acceptance criteria only
- Commands to run where relevant
- Expected outputs / conditions
- No vague language

---

## Output format
次の順序で、**ファイル内容のみ** を返してください:

1. `docs/<DAY_NAME>.md`
2. `specs/<DAY_NAME>/spec.md`
3. `specs/<DAY_NAME>/tasks.md`
4. `specs/<DAY_NAME>/acceptance.md`

For each file:
- Start with a heading line exactly like:
  `FILE: <path>`
- Then provide the full Markdown content for that file
- ファイル内容以外の説明文を混ぜない

---

## Quality checks before finalizing
最終化前に、必ず次を確認してください:
- Dayスコープが `docs/PLAN.md` と一致する
- 既存の完了済みDayの成果物と矛盾しない
- acceptance がテスト可能（観測可能）である
- docs/specs が GitHub Copilot による実装に使える
- 次のロードマップ工程に自然につながる

---

## Example invocation
DAY: 8
DAY_THEME: QoS設計

DAY_NAME: day8_qos
DAY_THEME: QoS設計
