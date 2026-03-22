from medical_interfaces.msg import Alert

from medical_robot_sim.rule_alert_engine import format_alert_emit_event


def test_format_alert_emit_event_contains_required_fields():
    a = Alert()
    a.patient_id = 'patient_01'
    a.rule_id = 'roc.spo2_drop'
    a.kind = 'rate_of_change'
    a.priority = 'YELLOW'
    a.message = 'SpO2 drop detected'
    a.ts = 123.0
    a.window_sec = 5
    a.field = 'oxygen_saturation'

    line = format_alert_emit_event(a, node='rule_alert_engine', ns='/')

    assert line.startswith('event=alerts.emit ')
    assert 'pid=patient_01' in line
    assert 'rule_id=roc.spo2_drop' in line
    assert 'priority=YELLOW' in line
    assert 'kind=rate_of_change' in line
    assert 'message=SpO2_drop_detected' in line
    assert 'node=rule_alert_engine' in line
    assert 'ns=/' in line
