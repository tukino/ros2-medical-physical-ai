import pytest

from medical_robot_sim.alert_rules import RuleParams
from medical_robot_sim.alert_rules import Sample
from medical_robot_sim.alert_rules import evaluate_alert_rules


def _s(
    *,
    ts: float,
    spo2: float = 98.0,
    hr: float = 70.0,
    sbp: float = 120.0,
    dbp: float = 80.0,
    temp: float = 36.5,
) -> Sample:
    return Sample(
        ts=float(ts),
        heart_rate=float(hr),
        blood_pressure_systolic=float(sbp),
        blood_pressure_diastolic=float(dbp),
        body_temperature=float(temp),
        oxygen_saturation=float(spo2),
    )


def test_single_spo2_lt_90_active():
    params = RuleParams(
        history_size=10,
        spo2_drop_threshold=4.0,
        hr_jump_threshold=20.0,
    )
    history = [_s(ts=0.0, spo2=89.0)]

    matches = evaluate_alert_rules(history, params)

    assert matches['single.spo2_lt_90'].active is True
    assert matches['single.spo2_lt_90'].priority == 'RED'


def test_roc_spo2_drop_detected():
    params = RuleParams(
        history_size=10,
        spo2_drop_threshold=4.0,
        hr_jump_threshold=20.0,
    )
    history = [
        _s(ts=0.0, spo2=98.0),
        _s(ts=1.0, spo2=97.0),
        _s(ts=2.0, spo2=93.0),
    ]

    matches = evaluate_alert_rules(history, params)

    roc = matches['roc.spo2_drop']
    assert roc.active is True
    assert roc.delta == pytest.approx(-5.0)
    assert roc.score == pytest.approx(1.25)


def test_combo_spo2_drop_and_hr_jump():
    params = RuleParams(
        history_size=10,
        spo2_drop_threshold=4.0,
        hr_jump_threshold=20.0,
    )
    history = [
        _s(ts=0.0, spo2=98.0, hr=70.0),
        _s(ts=1.0, spo2=97.0, hr=72.0),
        _s(ts=2.0, spo2=93.0, hr=95.0),
    ]

    matches = evaluate_alert_rules(history, params)

    assert matches['roc.spo2_drop'].active is True
    assert matches['roc.hr_jump'].active is True
    assert matches['combo.spo2_drop_and_hr_jump'].active is True
