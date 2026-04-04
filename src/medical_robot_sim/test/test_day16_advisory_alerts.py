import math

import pytest


Alert = pytest.importorskip('medical_interfaces.msg').Alert


def test_day16_advisory_event_to_alert_mapping_flatline_hr():
    from medical_robot_sim.advisory_alerts import anomaly_event_to_advisory_alert
    from medical_robot_sim.types import AnomalyEvent

    event = AnomalyEvent(
        type='flatline',
        field='heart_rate',
        value=72.0,
        delta=0.0,
        score=1.0,
        ts=123.0,
        window_sec=10,
    )

    msg = anomaly_event_to_advisory_alert(patient_id='patient_01', event=event)
    assert isinstance(msg, Alert)
    assert msg.kind == 'advisory'
    assert msg.rule_id == 'ai.flatline_hr'
    assert msg.priority == 'YELLOW'
    assert msg.patient_id == 'patient_01'
    assert msg.field == 'heart_rate'
    assert math.isfinite(float(msg.value))


def test_day16_advisory_event_to_alert_handles_non_numeric_values():
    from medical_robot_sim.advisory_alerts import anomaly_event_to_advisory_alert
    from medical_robot_sim.types import AnomalyEvent

    event = AnomalyEvent(
        type='flatline',
        field='oxygen_saturation',
        value='n/a',
        delta=None,
        score=None,
        ts=0.0,
        window_sec=10,
    )

    msg = anomaly_event_to_advisory_alert(patient_id='p', event=event)
    assert msg.kind == 'advisory'
    assert msg.rule_id == 'ai.flatline_spo2'

    assert math.isnan(float(msg.value))
    assert math.isnan(float(msg.delta))
    assert math.isnan(float(msg.score))
