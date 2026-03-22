from medical_robot_sim.icu_monitor import classify_patient_state


def test_classify_patient_state_boundaries_default_thresholds():
    assert classify_patient_state(None) == 'NO DATA'

    assert classify_patient_state(0.0) == 'FRESH'
    assert classify_patient_state(3.0) == 'FRESH'
    assert classify_patient_state(3.0001) == 'STALE'

    # With defaults: NO DATA only when age > 10.0
    assert classify_patient_state(10.0) == 'STALE'
    assert classify_patient_state(10.0001) == 'NO DATA'


def test_classify_patient_state_custom_thresholds():
    assert (
        classify_patient_state(None, stale_after_sec=1.0, no_data_after_sec=2.0) == 'NO DATA'
    )
    assert classify_patient_state(0.5, stale_after_sec=1.0, no_data_after_sec=2.0) == 'FRESH'
    assert classify_patient_state(1.0, stale_after_sec=1.0, no_data_after_sec=2.0) == 'FRESH'
    assert classify_patient_state(1.1, stale_after_sec=1.0, no_data_after_sec=2.0) == 'STALE'
    assert classify_patient_state(2.0, stale_after_sec=1.0, no_data_after_sec=2.0) == 'STALE'
    assert classify_patient_state(2.1, stale_after_sec=1.0, no_data_after_sec=2.0) == 'NO DATA'
