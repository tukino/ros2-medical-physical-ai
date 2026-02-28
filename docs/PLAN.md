# Plan / Roadmap (for Copilot)

## 0. Goal
- Build a ROS2 (Humble) multi-patient vital monitoring simulation.
- Provide rule-based alerting + anomaly detection as the core.
- Treat LLM as an optional extension (advisory only), not the core decision maker.
- Keep the repo reproducible: build/run/record/replay and testable.

## 1. Current Status (Done)
- Multi patient vitals publishing: /patient_XX/patient_vitals
- ICU dashboard display via icu_monitor
- rosbag2 recording confirmed
- Alerts topic exists: /patient_XX/alerts (Alert.msg)
- Basic alert rule example confirmed (e.g. roc.spo2_drop)
- Clean shutdown on Ctrl+C (no stack trace; exit cleanly)

## 2. Principles (Non-negotiables)
- Rule-based first. LLM is strictly optional extension.
- Every feature must include:
  - a minimal reproducible command/script
  - at least one automated test
  - a short doc page under docs/
- Prefer small incremental changes and backward compatibility.
- Configuration should be externalizable (Day7): rules in yaml/json, not hard-coded.

## 3. Next Milestones (Priority Order)
### Day6: Expand "alert arena" without LLM
- Add at least one of:
  - flatline detection
  - outlier detection
  - combination rule (multi-signal)
- Done criteria:
  - new rule_id emits on /patient_XX/alerts
  - tests cover boundary cases
  - docs/day6_*.md explains behavior and verification

### Day7: Config externalization
- Introduce rules.yaml (or rules.json)
- Launch arg rules_path:=...
- README: how to add a rule without code changes

### Day8: Observability / reproducibility
- scripts/repro_day6_alerts.sh
- rosbag play based verification (record once, replay often)

### Day9: LLM extension "hook" only
- advisory_engine interface + dummy implementation
- topic: /patient_XX/advisories
- no external API calls by default

## 4. Naming / Conventions
- Topics:
  - vitals: /patient_XX/patient_vitals
  - alerts: /patient_XX/alerts
  - (future) advisories: /patient_XX/advisories
- rule_id: "<category>.<rule_name>" (e.g. flatline.hr, outlier.spo2, combo.spo2_hr)
- priority: RED / YELLOW / INFO

## 5. Non-goals
- Medical device compliance or clinical diagnosis.
- Sophisticated UI.
- Robust anti-cheat or security hardening.