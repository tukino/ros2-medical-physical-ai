import pytest


VitalSigns = pytest.importorskip('medical_interfaces.msg').VitalSigns


def _make_vitals(
    patient_id: str,
    heart_rate: int,
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


def test_flatline_emits_once_at_window_boundary():
    from medical_robot_sim.anomaly_detector import FlatlineDetector

    detector = FlatlineDetector(window_size=5)

    emitted = []
    for i in range(5):
        events = detector.update(_make_vitals('p1', heart_rate=70), ts=float(i))
        emitted.append(events)

    # 5件目で初めて flatline が出る
    last_events = emitted[-1]
    assert any(e.type == 'flatline' and e.field == 'heart_rate' for e in last_events)

    # 継続中は同じフィールドで重複発火しない
    events_after = detector.update(_make_vitals('p1', heart_rate=70), ts=5.0)
    assert not any(e.type == 'flatline' and e.field == 'heart_rate' for e in events_after)


def test_flatline_triggers_for_tiny_variation_within_epsilon():
    from medical_robot_sim.anomaly_detector import FlatlineDetector

    detector = FlatlineDetector(window_size=5)

    # デフォルトepsilon(heart_rate=1.0)の範囲内で小さく揺らす
    values = [70, 70, 71, 70, 71]
    events = []
    for i, v in enumerate(values):
        events = detector.update(_make_vitals('p2', heart_rate=v), ts=float(i))

    assert any(e.type == 'flatline' and e.field == 'heart_rate' for e in events)


def test_flatline_clears_and_can_reemit_after_recovery():
    from medical_robot_sim.anomaly_detector import FlatlineDetector

    detector = FlatlineDetector(window_size=4)

    # flatline 発火
    for i in range(4):
        events = detector.update(_make_vitals('p3', heart_rate=80), ts=float(i))
    assert any(e.type == 'flatline' and e.field == 'heart_rate' for e in events)

    # 大きく変化して回復（active解除）
    detector.update(_make_vitals('p3', heart_rate=95), ts=10.0)

    # 再び flatline になれば再発火する
    all_events2 = []
    for i in range(4):
        events2 = detector.update(_make_vitals('p3', heart_rate=95), ts=20.0 + float(i))
        all_events2.extend(events2)
    assert any(e.type == 'flatline' and e.field == 'heart_rate' for e in all_events2)


def test_spo2_drop_event_has_delta_and_score():
    from medical_robot_sim.anomaly_detector import FlatlineDetector

    detector = FlatlineDetector(window_size=5)

    events = []
    # 直近の最高値(98)から 90 へ drop (delta=-8)
    spo2_values = [98, 98, 98, 98, 90]
    for i, v in enumerate(spo2_values):
        msg = _make_vitals('p4', heart_rate=70, oxygen_saturation=v)
        events = detector.update(msg, ts=float(i))

    drop_events = [e for e in events if e.type == 'spo2_drop']
    assert drop_events
    assert drop_events[0].field == 'oxygen_saturation'
    assert drop_events[0].delta is not None
    assert drop_events[0].delta < 0
    assert drop_events[0].score is not None
    assert drop_events[0].score > 0


def test_spo2_drop_not_emitted_when_drop_is_small():
    from medical_robot_sim.anomaly_detector import FlatlineDetector

    detector = FlatlineDetector(window_size=5)

    # 最高値(98)から 95 へ (delta=-3) は閾値(4)未満なので検出しない
    spo2_values = [98, 98, 97, 96, 95]
    all_events = []
    for i, v in enumerate(spo2_values):
        msg = _make_vitals('p4n', heart_rate=70, oxygen_saturation=v)
        all_events.extend(detector.update(msg, ts=float(i)))

    assert not any(e.type == 'spo2_drop' for e in all_events)


def test_hr_jump_event_has_delta_and_score():
    from medical_robot_sim.anomaly_detector import FlatlineDetector

    detector = FlatlineDetector(window_size=5)

    events = []
    hr_values = [70, 70, 70, 70, 95]
    for i, v in enumerate(hr_values):
        msg = _make_vitals('p5', heart_rate=v, oxygen_saturation=98)
        events = detector.update(msg, ts=float(i))

    jump_events = [e for e in events if e.type == 'hr_jump']
    assert jump_events
    assert jump_events[0].field == 'heart_rate'
    assert jump_events[0].delta is not None
    assert jump_events[0].delta > 0
    assert jump_events[0].score is not None
    assert jump_events[0].score > 0


def test_hr_jump_not_emitted_when_jump_is_small():
    from medical_robot_sim.anomaly_detector import FlatlineDetector

    detector = FlatlineDetector(window_size=5)

    # 最小値(70)から 85 へ (delta=15) は閾値(20)未満なので検出しない
    hr_values = [70, 72, 75, 80, 85]
    all_events = []
    for i, v in enumerate(hr_values):
        msg = _make_vitals('p5n', heart_rate=v, oxygen_saturation=98)
        all_events.extend(detector.update(msg, ts=float(i)))

    assert not any(e.type == 'hr_jump' for e in all_events)


def test_missing_fields_do_not_raise():
    from medical_robot_sim.anomaly_detector import FlatlineDetector

    class Dummy:
        def __init__(self):
            self.patient_id = 'p6'
            # heart_rate / oxygen_saturation 等は定義しない

    detector = FlatlineDetector(window_size=3)

    # 欠損が混ざっても例外にしない
    detector.update(Dummy(), ts=0.0)
    detector.update(Dummy(), ts=1.0)
    events = detector.update(Dummy(), ts=2.0)
    assert events == []
