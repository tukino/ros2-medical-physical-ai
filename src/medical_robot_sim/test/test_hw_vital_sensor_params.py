import pytest

from medical_robot_sim.hw_vital_sensor import validate_hw_sensor_params


@pytest.mark.parametrize(
    'publish_rate_hz,driver,mock_scenario',
    [
        (1.0, 'mock', 'normal'),
        (10.0, 'mock', 'spo2_drop'),
        (0.5, 'mock', 'flatline'),
        (1.0, 'i2c', 'normal'),
        (1.0, 'spi', 'normal'),
    ],
)
def test_validate_hw_sensor_params_ok(publish_rate_hz, driver, mock_scenario):
    validate_hw_sensor_params(
        publish_rate_hz=publish_rate_hz,
        driver=driver,
        mock_scenario=mock_scenario,
    )


@pytest.mark.parametrize('bad_rate', [0.0, -1.0])
def test_validate_hw_sensor_params_rejects_nonpositive_rate(bad_rate):
    with pytest.raises(ValueError):
        validate_hw_sensor_params(
            publish_rate_hz=bad_rate,
            driver='mock',
            mock_scenario='normal',
        )


def test_validate_hw_sensor_params_rejects_unknown_driver():
    with pytest.raises(ValueError):
        validate_hw_sensor_params(
            publish_rate_hz=1.0,
            driver='serial',
            mock_scenario='normal',
        )


def test_validate_hw_sensor_params_rejects_unknown_mock_scenario():
    with pytest.raises(ValueError):
        validate_hw_sensor_params(
            publish_rate_hz=1.0,
            driver='mock',
            mock_scenario='weird',
        )
