import pytest


VitalSigns = pytest.importorskip('medical_interfaces.msg').VitalSigns


def _make_vitals(
    patient_id: str = 'p',
    heart_rate: int = 70,
    oxygen_saturation: int = 98,
    blood_pressure_systolic: int = 120,
    blood_pressure_diastolic: int = 80,
    body_temperature: float = 36.5,
):
    msg = VitalSigns()
    msg.patient_id = patient_id
    msg.measurement_id = 0
    msg.heart_rate = int(heart_rate)
    msg.oxygen_saturation = int(oxygen_saturation)
    msg.blood_pressure_systolic = int(blood_pressure_systolic)
    msg.blood_pressure_diastolic = int(blood_pressure_diastolic)
    msg.body_temperature = float(body_temperature)
    msg.status = 'monitoring'
    return msg


def test_rule_evaluator_levels():
    from medical_robot_sim.rule_evaluator import evaluate_event_level
    from medical_robot_sim.types import AnomalyEvent

    base = dict(value=None, delta=None, score=None, ts=0.0, window_sec=10)

    assert evaluate_event_level([]) == 'OK'
    assert (
        evaluate_event_level([
            AnomalyEvent(type='spo2_drop', field='oxygen_saturation', **base),
        ])
        == 'WARN'
    )
    assert (
        evaluate_event_level([
            AnomalyEvent(type='hr_jump', field='heart_rate', **base),
        ])
        == 'WARN'
    )
    assert (
        evaluate_event_level([
            AnomalyEvent(type='spo2_drop', field='oxygen_saturation', **base),
            AnomalyEvent(type='hr_jump', field='heart_rate', **base),
        ])
        == 'CRITICAL'
    )


def test_calc_alert_event_priority_without_downgrade():
    from medical_robot_sim.advisory_engine import calc_alert
    from medical_robot_sim.types import AnomalyEvent

    base = dict(value=None, delta=None, score=None, ts=0.0, window_sec=10)

    # 生値しきい値は OK だがイベント WARN で YELLOW になる
    vitals_ok = _make_vitals(heart_rate=70, oxygen_saturation=98, blood_pressure_systolic=120)
    events_warn = [AnomalyEvent(type='spo2_drop', field='oxygen_saturation', **base)]
    assert calc_alert(vitals_ok, events_warn) == 'YELLOW'

    # CRITICAL は RED
    events_crit = [
        AnomalyEvent(type='spo2_drop', field='oxygen_saturation', **base),
        AnomalyEvent(type='hr_jump', field='heart_rate', **base),
    ]
    assert calc_alert(vitals_ok, events_crit) == 'RED'

    # 生値 ORANGE はイベント WARN で下げない
    vitals_orange = _make_vitals(heart_rate=130, oxygen_saturation=98, blood_pressure_systolic=120)
    assert calc_alert(vitals_orange, events_warn) == 'ORANGE'
