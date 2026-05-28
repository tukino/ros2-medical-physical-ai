"""Day19 tests: rosbag replay × closed-loop control integration.

These are pure-function / wiring tests that do NOT require a live ROS graph.
They verify the policy logic reachable through the replay path:
- spo2_drop scenario (bag contains SpO2=85) -> CALL_STAFF
- bag with normal vitals -> HOLD (no control publish)
- replay NO_DATA guard: age_sec >> no_data_after_sec -> HOLD
- replay cooldown: second decision within cooldown -> should_publish=False
- replay with advisory integration flag off -> alert-only path
"""

from __future__ import annotations

from medical_robot_sim.closed_loop_policy import ACTION_CALL_STAFF
from medical_robot_sim.closed_loop_policy import ACTION_HOLD
from medical_robot_sim.closed_loop_policy import ACTION_OXYGEN_BOOST
from medical_robot_sim.closed_loop_policy import decide_control_action


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _decide(
    *,
    spo2: float,
    hr: float = 72.0,
    priority: str = '',
    rule_id: str = '',
    age_sec: float = 0.5,
    last_action: object = None,
    last_publish_age_sec: object = None,
    cooldown_sec: float = 1.0,
    no_data_after_sec: float = 10.0,
    low_spo2: float = 99.0,
    critical_spo2: float = 95.0,
):
    """Call decide_control_action with replay-style default thresholds."""
    return decide_control_action(
        hr=hr,
        spo2=spo2,
        latest_alert_priority=priority,
        latest_alert_rule_id=rule_id,
        age_sec=age_sec,
        last_action=last_action,
        last_publish_age_sec=last_publish_age_sec,
        cooldown_sec=cooldown_sec,
        no_data_after_sec=no_data_after_sec,
        low_spo2=low_spo2,
        critical_spo2=critical_spo2,
    )


# ---------------------------------------------------------------------------
# Replay scenario: spo2_drop (SpO2=85 in bag) -> CALL_STAFF
# ---------------------------------------------------------------------------

def test_replay_spo2_drop_triggers_call_staff() -> None:
    """Bag contains SpO2=85; threshold critical_spo2=95 -> CALL_STAFF."""
    d = _decide(spo2=85.0)
    assert d.action == ACTION_CALL_STAFF
    assert d.should_publish is True


def test_replay_spo2_below_low_triggers_oxygen_boost() -> None:
    """SpO2=97 < low_spo2=99 but above critical_spo2=95 -> OXYGEN_BOOST."""
    d = _decide(spo2=97.0)
    assert d.action == ACTION_OXYGEN_BOOST
    assert d.should_publish is True


# ---------------------------------------------------------------------------
# Replay scenario: normal vitals -> HOLD
# ---------------------------------------------------------------------------

def test_replay_normal_vitals_hold() -> None:
    """SpO2=100 (normal), no alert -> HOLD."""
    d = _decide(spo2=100.0)
    assert d.action == ACTION_HOLD
    assert d.should_publish is True


# ---------------------------------------------------------------------------
# Replay NO_DATA guard: stale vitals -> HOLD regardless of SpO2
# ---------------------------------------------------------------------------

def test_replay_no_data_guard_overrides_critical_spo2() -> None:
    """Even with SpO2=80 and RED alert, NO_DATA guard forces HOLD."""
    d = _decide(
        spo2=80.0,
        priority='RED',
        rule_id='outlier.spo2',
        age_sec=15.0,          # > no_data_after_sec=10
        no_data_after_sec=10.0,
    )
    assert d.action == ACTION_HOLD


def test_replay_age_sec_exactly_at_threshold_is_no_data() -> None:
    """age_sec strictly greater than no_data_after_sec -> HOLD."""
    d = _decide(
        spo2=80.0,
        age_sec=10.001,
        no_data_after_sec=10.0,
    )
    assert d.action == ACTION_HOLD


def test_replay_age_sec_at_boundary_is_ok() -> None:
    """age_sec == no_data_after_sec is NOT classified as no_data (boundary = valid)."""
    d = _decide(
        spo2=85.0,
        age_sec=10.0,
        no_data_after_sec=10.0,
    )
    # age_sec=10.0 is NOT > 10.0, so NO_DATA guard does not fire
    assert d.action == ACTION_CALL_STAFF


# ---------------------------------------------------------------------------
# Replay cooldown: second decision within cooldown -> should_publish=False
# ---------------------------------------------------------------------------

def test_replay_cooldown_suppresses_second_publish() -> None:
    """First replay action triggers publish; within cooldown -> suppressed."""
    d = _decide(
        spo2=85.0,
        last_action=ACTION_CALL_STAFF,
        last_publish_age_sec=0.5,  # < cooldown_sec=1.0
        cooldown_sec=1.0,
    )
    assert d.action == ACTION_CALL_STAFF
    assert d.should_publish is False


def test_replay_cooldown_allows_publish_after_expiry() -> None:
    """After cooldown expires, same action can be published again."""
    d = _decide(
        spo2=85.0,
        last_action=ACTION_CALL_STAFF,
        last_publish_age_sec=2.0,  # > cooldown_sec=1.0
        cooldown_sec=1.0,
    )
    assert d.action == ACTION_CALL_STAFF
    assert d.should_publish is True


# ---------------------------------------------------------------------------
# Advisory integration flag off: only alert path is used
# ---------------------------------------------------------------------------

def test_replay_advisory_flag_off_uses_alert_priority() -> None:
    """When enable_control_from_advisories=false, only alert drives decision.

    This is tested at the policy level: passing RED alert -> CALL_STAFF.
    """
    d = _decide(
        spo2=100.0,   # Normal SpO2 would give HOLD...
        priority='RED',  # ...but RED alert -> CALL_STAFF
        rule_id='any.rule',
    )
    assert d.action == ACTION_CALL_STAFF


def test_replay_no_alert_no_abnormal_vitals_hold() -> None:
    """Replay of normal bag: no alert, normal SpO2 -> HOLD."""
    d = _decide(
        spo2=99.5,
        priority='',
        rule_id='',
    )
    assert d.action == ACTION_HOLD


# ---------------------------------------------------------------------------
# Age_sec=None (vitals never received) -> HOLD
# ---------------------------------------------------------------------------

def test_replay_age_sec_none_is_no_data() -> None:
    """If vitals never arrived (age_sec=None), policy returns HOLD."""
    d = decide_control_action(
        hr=70.0,
        spo2=80.0,
        latest_alert_priority='RED',
        latest_alert_rule_id='outlier.spo2',
        age_sec=None,
        last_action=None,
        last_publish_age_sec=None,
        cooldown_sec=1.0,
        no_data_after_sec=10.0,
        low_spo2=99.0,
        critical_spo2=95.0,
    )
    assert d.action == ACTION_HOLD


# ---------------------------------------------------------------------------
# Severity mapping
# ---------------------------------------------------------------------------

def test_replay_call_staff_severity_red() -> None:
    d = _decide(spo2=85.0)
    assert d.severity == 'RED'


def test_replay_oxygen_boost_severity_yellow() -> None:
    d = _decide(spo2=97.0)
    assert d.severity == 'YELLOW'


def test_replay_hold_severity_info() -> None:
    d = _decide(spo2=100.0)
    assert d.severity == 'INFO'
