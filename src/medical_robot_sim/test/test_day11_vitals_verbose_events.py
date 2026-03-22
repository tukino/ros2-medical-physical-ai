from medical_robot_sim.observability import format_vitals_drop_event
from medical_robot_sim.observability import format_vitals_enqueue_delayed_event


def test_format_vitals_drop_event_contains_required_fields():
    line = format_vitals_drop_event(
        node='vital_sensor',
        ns='/patient_01',
        tick=12,
        elapsed_sec=12.0,
        drop_rate=0.25,
    )

    assert line.startswith('event=vitals.drop ')
    assert 'node=vital_sensor' in line
    assert 'ns=/patient_01' in line
    assert 'tick=12' in line
    assert 'elapsed_sec=12.0' in line
    assert 'drop_rate=0.25' in line


def test_format_vitals_enqueue_delayed_event_contains_required_fields():
    line = format_vitals_enqueue_delayed_event(
        node='vital_sensor',
        ns='/patient_01',
        tick=5,
        elapsed_sec=5.0,
        delay_ms=500,
        jitter_ms=50,
        delay_ticks=1,
        due_tick=6,
        queue_len=3,
        clamped=False,
    )

    assert line.startswith('event=vitals.enqueue_delayed ')
    assert 'node=vital_sensor' in line
    assert 'ns=/patient_01' in line
    assert 'tick=5' in line
    assert 'delay_ms=500' in line
    assert 'jitter_ms=50' in line
    assert 'delay_ticks=1' in line
    assert 'due_tick=6' in line
    assert 'queue_len=3' in line
    assert 'clamped=false' in line
