"""Day16 advisory message helpers.

This module converts anomaly detection events (AnomalyEvent) into
`medical_interfaces/msg/Alert` messages published on `/patient_XX/advisories`.

Design:
- Advisories are NOT the decision authority (alerts remain rule-based).
- Keep this module rclpy-free for lightweight unit tests.
"""

from __future__ import annotations

import math

from medical_interfaces.msg import Alert

from medical_robot_sim.types import AnomalyEvent


def _nan() -> float:
    return float('nan')


def _to_float_or_nan(value) -> float:
    if value is None:
        return _nan()
    try:
        x = float(value)
    except (TypeError, ValueError):
        return _nan()
    if math.isnan(x):
        return _nan()
    return float(x)


def advisory_rule_id(event: AnomalyEvent) -> str:
    """Return a stable rule_id for an advisory event."""

    et = str(event.type or '')
    field = str(event.field or '')

    if et == 'flatline':
        if field == 'heart_rate':
            return 'ai.flatline_hr'
        if field == 'oxygen_saturation':
            return 'ai.flatline_spo2'
        if field:
            return f'ai.flatline_{field}'
        return 'ai.flatline'

    if et == 'spo2_drop':
        return 'ai.spo2_drop'

    if et == 'hr_jump':
        return 'ai.hr_jump'

    # Future-proof fallback (do not raise; keep publisher robust)
    return f'ai.{et}' if et else 'ai.unknown'


def advisory_priority(event: AnomalyEvent) -> str:
    """Return advisory priority string for ICU dashboard usage."""

    et = str(event.type or '')

    # Minimal mapping (can be refined later).
    if et in {'spo2_drop', 'hr_jump'}:
        return 'YELLOW'
    if et == 'flatline':
        return 'YELLOW'

    return 'INFO'


def advisory_message(event: AnomalyEvent) -> str:
    """Human-readable advisory message (short, stable)."""

    et = str(event.type or '').strip() or 'unknown'
    field = str(event.field or '').strip()

    if field:
        return f'{et} {field}'
    return str(et)


def anomaly_event_to_advisory_alert(*, patient_id: str, event: AnomalyEvent) -> Alert:
    """Convert a single AnomalyEvent to an Alert message (kind=advisory)."""

    msg = Alert()

    msg.patient_id = str(patient_id)
    msg.kind = 'advisory'
    msg.rule_id = str(advisory_rule_id(event))
    msg.priority = str(advisory_priority(event))
    msg.message = str(advisory_message(event))

    msg.ts = float(event.ts)
    msg.window_sec = int(event.window_sec)

    msg.field = str(event.field or '')

    msg.value = _to_float_or_nan(event.value)
    msg.delta = _to_float_or_nan(event.delta)
    msg.score = _to_float_or_nan(event.score)

    return msg
