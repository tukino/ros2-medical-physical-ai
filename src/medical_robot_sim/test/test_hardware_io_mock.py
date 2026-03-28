from medical_robot_sim.hardware_io import MockVitalDevice


def test_mock_device_normal_is_reasonable():
    dev = MockVitalDevice(patient_id='patient_01', scenario='normal', publish_rate_hz=1.0)
    dev.open()

    sample1 = dev.read_sample()
    sample2 = dev.read_sample()

    assert 30 <= sample1.heart_rate <= 200
    assert 70 <= sample1.oxygen_saturation <= 100
    assert 30 <= sample2.heart_rate <= 200
    assert 70 <= sample2.oxygen_saturation <= 100


def test_mock_device_spo2_drop_decreases_and_floors():
    dev = MockVitalDevice(patient_id='patient_01', scenario='spo2_drop', publish_rate_hz=1.0)
    dev.open()

    spo2_values = [dev.read_sample().oxygen_saturation for _ in range(30)]

    # It should eventually reach the floor (=88.0 by spec/impl) and never go below.
    assert min(spo2_values) >= 88
    assert spo2_values[-1] == 88


def test_mock_device_flatline_is_constant():
    dev = MockVitalDevice(patient_id='patient_01', scenario='flatline', publish_rate_hz=2.0)
    dev.open()

    sample1 = dev.read_sample()
    sample2 = dev.read_sample()
    sample3 = dev.read_sample()

    assert sample1.heart_rate == sample2.heart_rate == sample3.heart_rate
    assert (
        sample1.oxygen_saturation == sample2.oxygen_saturation == sample3.oxygen_saturation
    )
