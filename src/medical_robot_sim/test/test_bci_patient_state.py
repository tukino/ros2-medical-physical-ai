import pytest

from medical_robot_sim.bci_patient_state import classify_bci_state


@pytest.mark.parametrize(
    'age_sec, expected',
    [
        (None, 'NO DATA'),
        (0.0, 'FRESH'),
        (3.0, 'FRESH'),
        (3.01, 'STALE'),
        (10.0, 'STALE'),
        (10.01, 'NO DATA'),
    ],
)
def test_classify_bci_state_boundaries(age_sec, expected):
    assert (
        classify_bci_state(age_sec, stale_after_sec=3.0, no_data_after_sec=10.0)
        == expected
    )
