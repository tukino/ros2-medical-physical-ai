# Copilot Instructions for ros2-medical-physical-ai

## 0. Mandatory Context

Before proposing or generating any code:
- Read `docs/PLAN.md`
- Respect the current architecture and priorities described there
- Do NOT introduce large refactors unless explicitly requested

This project is a ROS2 (Humble) multi-patient vital monitoring simulation
with rule-based alerting as the core design principle.

LLM-based reasoning is OPTIONAL and treated as an extension layer only.

---

## 1. Core Architectural Principles

1. Rule-based logic is primary.
2. LLM is an advisory extension, not a decision authority.
3. Every feature must include:
   - Minimal reproducible command(s)
   - At least one automated test
   - A short documentation update under `docs/`
4. Prefer small incremental changes.
5. Preserve backward compatibility unless instructed otherwise.
6. Clean shutdown must remain intact (Ctrl+C → no stack trace).

---

## 2. Topic Contracts (Do Not Break)

Vitals:
- `/patient_XX/patient_vitals`

Alerts:
- `/patient_XX/alerts`

Future (optional):
- `/patient_XX/advisories`

Do not rename or restructure topics unless explicitly requested.

---

## 3. Naming Conventions

rule_id format:
- "<category>.<rule_name>"
  Examples:
  - flatline.hr
  - outlier.spo2
  - roc.spo2_drop
  - combo.spo2_hr

priority:
- RED
- YELLOW
- INFO

Message definitions must remain stable once introduced.

---

## 4. Design Constraints

- Keep evaluation logic modular:
  - `alert_rules.py`
  - `rule_evaluator.py`
  - `anomaly_detector.py`
  - etc.
- Avoid tight coupling between sensor, monitor, and rule engine.
- Prefer pure functions for rule evaluation so they are testable.
- State (window buffers) must be isolated per patient.

---

## 5. Testing Requirements

When adding or modifying logic:

- Add or update pytest tests under:
  `src/medical_robot_sim/test/`
- Cover:
  - Boundary values
  - Negative cases
  - Expected alert emission

Tests must pass via:
```

colcon test

```

Do not introduce untested logic.

---

## 6. Reproducibility Requirements

If a feature affects runtime behavior:

- Update README with:
  - How to run it
  - What output is expected
- If relevant, update or add a script under `scripts/`
- Ensure rosbag recording still works

Bag validation must confirm:
- Messages > 0
- Topic information is present

---

## 7. Non-Goals

Do NOT:

- Implement medical device compliance
- Add heavy UI layers
- Add external API calls by default
- Introduce complex frameworks
- Over-engineer configuration prematurely

---

## 8. When Unsure

If architectural ambiguity exists:
- Propose 2–3 small design options
- Explain tradeoffs
- Default to the simplest incremental solution

---

## 9. Current Priority (Check PLAN.md)

Actively work toward:
- Expanding rule-based alert coverage (Day6)
- Configuration externalization (Day7)
- Observability improvements (Day8)

LLM integration is not the current focus.