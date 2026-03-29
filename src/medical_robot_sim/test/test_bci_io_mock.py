import math

import pytest

from medical_robot_sim.bci_io import MockBCIDevice


def test_mock_bci_device_deterministic_sequence():
    d1 = MockBCIDevice(scenario='drowsy', seed=42)
    d2 = MockBCIDevice(scenario='drowsy', seed=42)

    d1.open()
    d2.open()

    seq1 = [d1.read_features() for _ in range(5)]
    seq2 = [d2.read_features() for _ in range(5)]

    assert seq1 == seq2


def test_mock_bci_device_invalid_scenario_raises():
    dev = MockBCIDevice(scenario='unknown', seed=0)
    dev.open()
    with pytest.raises(ValueError):
        dev.read_features()


def test_mock_bci_device_signal_quality_in_range():
    dev = MockBCIDevice(scenario='artifact_spike', seed=1)
    dev.open()

    for _ in range(20):
        sample = dev.read_features()
        assert 0.0 <= sample.signal_quality <= 1.0
        assert sample.status in {'ok', 'degraded', 'no_signal'}
        assert not math.isnan(float(sample.attention))
        assert not math.isnan(float(sample.drowsiness))
