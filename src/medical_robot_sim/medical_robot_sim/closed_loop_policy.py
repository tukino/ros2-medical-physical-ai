"""Day18 closed-loop control policy (pure functions).

This module is intentionally rclpy-free for lightweight unit tests.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

ACTION_HOLD = 'HOLD'
ACTION_OXYGEN_BOOST = 'OXYGEN_BOOST'
ACTION_CALL_STAFF = 'CALL_STAFF'


@dataclass(frozen=True)
class ControlDecision:
    action: str
    reason: str
    severity: str
    should_publish: bool


def _to_float_or_none(value: object) -> Optional[float]:
    if value is None:
        return None
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(x):
        return None
    return x


def decide_control_action(
    *,
    hr: object,
    spo2: object,
    latest_alert_priority: str,
    latest_alert_rule_id: str,
    age_sec: Optional[float],
    last_action: Optional[str],
    last_publish_age_sec: Optional[float],
    cooldown_sec: float,
    no_data_after_sec: float,
    low_spo2: float,
    critical_spo2: float,
) -> ControlDecision:
    """Return closed-loop control decision from current observations.

    Decision order:
    1) NO_DATA guard -> HOLD
    2) Critical condition -> CALL_STAFF
    3) Warning condition -> OXYGEN_BOOST
    4) Otherwise -> HOLD

    Publish suppression:
    - If within cooldown window, suppress publish.
    - If same action and last publish age unknown, suppress publish.
    """

    _ = _to_float_or_none(hr)
    spo2_value = _to_float_or_none(spo2)

    priority = str(latest_alert_priority or '').strip().upper()
    rule_id = str(latest_alert_rule_id or '').strip().lower()

    action = ACTION_HOLD
    reason = 'normal'
    severity = 'INFO'

    no_data = age_sec is None or float(age_sec) > float(no_data_after_sec)
    if no_data:
        action = ACTION_HOLD
        reason = 'no_data_guard'
        severity = 'INFO'
    elif priority == 'RED' or (
        spo2_value is not None and float(spo2_value) <= float(critical_spo2)
    ):
        action = ACTION_CALL_STAFF
        reason = 'critical_spo2_or_red_alert'
        severity = 'RED'
    elif (
        spo2_value is not None and float(spo2_value) <= float(low_spo2)
    ) or ('outlier.spo2' in rule_id):
        action = ACTION_OXYGEN_BOOST
        reason = 'low_spo2_or_spo2_outlier'
        severity = 'YELLOW'

    should_publish = True
    if last_publish_age_sec is not None and float(last_publish_age_sec) < float(cooldown_sec):
        should_publish = False
    elif last_action == action and last_publish_age_sec is None:
        should_publish = False

    return ControlDecision(
        action=action,
        reason=reason,
        severity=severity,
        should_publish=bool(should_publish),
    )
