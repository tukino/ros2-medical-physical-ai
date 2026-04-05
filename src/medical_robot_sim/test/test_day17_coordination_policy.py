from medical_robot_sim.coordination_policy import all_patients_no_data
from medical_robot_sim.coordination_policy import is_no_data
from medical_robot_sim.coordination_policy import is_ready


def test_is_ready_boundary() -> None:
    assert is_ready({'patient_01': 0, 'patient_02': 1}, min_messages_per_patient=1) is False
    assert is_ready({'patient_01': 1, 'patient_02': 1}, min_messages_per_patient=1) is True
    assert is_ready({'patient_01': 2}, min_messages_per_patient=2) is True
    assert is_ready({'patient_01': 1}, min_messages_per_patient=2) is False


def test_is_no_data_boundary() -> None:
    assert is_no_data(None, no_data_after_sec=10.0) is True
    assert is_no_data(10.0, no_data_after_sec=10.0) is False
    assert is_no_data(10.0001, no_data_after_sec=10.0) is True


def test_all_patients_no_data() -> None:
    ages = {'patient_01': None, 'patient_02': 100.0}
    assert all_patients_no_data(ages, no_data_after_sec=10.0) is True

    ages2 = {'patient_01': None, 'patient_02': 0.0}
    assert all_patients_no_data(ages2, no_data_after_sec=10.0) is False
