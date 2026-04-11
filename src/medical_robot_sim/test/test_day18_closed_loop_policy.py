from medical_robot_sim.closed_loop_policy import ACTION_CALL_STAFF
from medical_robot_sim.closed_loop_policy import ACTION_HOLD
from medical_robot_sim.closed_loop_policy import ACTION_OXYGEN_BOOST
from medical_robot_sim.closed_loop_policy import decide_control_action


def test_spo2_low_boundary_triggers_oxygen_boost() -> None:
    d = decide_control_action(
        hr=70.0,
        spo2=92.0,
        latest_alert_priority='',
        latest_alert_rule_id='',
        age_sec=0.1,
        last_action=None,
        last_publish_age_sec=None,
        cooldown_sec=5.0,
        no_data_after_sec=10.0,
        low_spo2=92.0,
        critical_spo2=88.0,
    )
    assert d.action == ACTION_OXYGEN_BOOST
    assert d.should_publish is True


def test_spo2_critical_boundary_triggers_call_staff() -> None:
    d = decide_control_action(
        hr=70.0,
        spo2=88.0,
        latest_alert_priority='',
        latest_alert_rule_id='',
        age_sec=0.1,
        last_action=None,
        last_publish_age_sec=None,
        cooldown_sec=5.0,
        no_data_after_sec=10.0,
        low_spo2=92.0,
        critical_spo2=88.0,
    )
    assert d.action == ACTION_CALL_STAFF


def test_no_data_guard_always_hold() -> None:
    d = decide_control_action(
        hr=120.0,
        spo2=75.0,
        latest_alert_priority='RED',
        latest_alert_rule_id='outlier.spo2',
        age_sec=10.0001,
        last_action=None,
        last_publish_age_sec=None,
        cooldown_sec=5.0,
        no_data_after_sec=10.0,
        low_spo2=92.0,
        critical_spo2=88.0,
    )
    assert d.action == ACTION_HOLD


def test_cooldown_suppresses_publish() -> None:
    d = decide_control_action(
        hr=70.0,
        spo2=80.0,
        latest_alert_priority='RED',
        latest_alert_rule_id='any.rule',
        age_sec=0.5,
        last_action=ACTION_CALL_STAFF,
        last_publish_age_sec=2.0,
        cooldown_sec=5.0,
        no_data_after_sec=10.0,
        low_spo2=92.0,
        critical_spo2=88.0,
    )
    assert d.action == ACTION_CALL_STAFF
    assert d.should_publish is False


def test_normal_case_hold() -> None:
    d = decide_control_action(
        hr=72.0,
        spo2=98.0,
        latest_alert_priority='',
        latest_alert_rule_id='',
        age_sec=0.2,
        last_action=None,
        last_publish_age_sec=None,
        cooldown_sec=5.0,
        no_data_after_sec=10.0,
        low_spo2=92.0,
        critical_spo2=88.0,
    )
    assert d.action == ACTION_HOLD
    assert d.should_publish is True
