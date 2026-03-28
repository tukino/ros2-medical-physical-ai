from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VitalSample:
    patient_id: str
    heart_rate: int
    blood_pressure_systolic: int
    blood_pressure_diastolic: int
    body_temperature: float
    oxygen_saturation: int
    status: str = 'monitoring'


class VitalDeviceError(RuntimeError):
    pass


class VitalDevice:
    """Hardware (or mock) vital device abstraction.

    This module intentionally avoids importing rclpy so it can be unit-tested.
    """

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def read_sample(self) -> VitalSample:
        raise NotImplementedError


def _clamp_int(value: int, *, min_value: int, max_value: int) -> int:
    return int(max(min_value, min(max_value, int(value))))


def _clamp_float(value: float, *, min_value: float, max_value: float) -> float:
    return float(max(min_value, min(max_value, float(value))))


class MockVitalDevice(VitalDevice):
    """Deterministic mock device.

    Scenarios are designed to reliably trigger existing rule-based alerts.
    """

    def __init__(
        self,
        *,
        patient_id: str,
        publish_rate_hz: float,
        scenario: str,
    ):
        self._patient_id = str(patient_id)
        self._publish_rate_hz = float(publish_rate_hz)
        self._scenario = str(scenario).strip().lower() or 'normal'
        self._opened = False
        self._tick = 0

        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        allowed = {'normal', 'spo2_drop', 'flatline'}
        if self._scenario not in allowed:
            raise ValueError(
                f"Invalid mock scenario '{scenario}'. Allowed values: {', '.join(sorted(allowed))}"
            )

        self._spo2_drop_delay_sec = 3.0
        self._spo2_drop_delay_samples = max(
            0, int(round(self._spo2_drop_delay_sec * self._publish_rate_hz))
        )

    @property
    def scenario(self) -> str:
        return self._scenario

    def open(self) -> None:
        self._opened = True

    def close(self) -> None:
        self._opened = False

    def read_sample(self) -> VitalSample:
        if not self._opened:
            raise VitalDeviceError('Device is not opened')

        base_hr = 72
        base_sys = 120
        base_dia = 80
        base_temp = 36.5
        base_spo2 = 98

        if self._scenario == 'flatline':
            hr = base_hr
            spo2 = base_spo2
            sys = base_sys
            dia = base_dia
            temp = base_temp
        else:
            # Deterministic tiny variations (no RNG)
            hr = base_hr + (self._tick % 5) - 2
            sys = base_sys + (self._tick % 3) - 1
            dia = base_dia + (self._tick % 3) - 1
            temp = base_temp + 0.05 * ((self._tick % 5) - 2)

            if self._scenario == 'spo2_drop':
                start = 98
                drop_step = 2
                min_value = 88
                steps = max(0, self._tick - self._spo2_drop_delay_samples)
                spo2 = max(min_value, start - drop_step * steps)
            else:
                spo2 = base_spo2 - (self._tick % 3)

        self._tick += 1

        return VitalSample(
            patient_id=self._patient_id,
            heart_rate=_clamp_int(hr, min_value=0, max_value=300),
            blood_pressure_systolic=_clamp_int(sys, min_value=0, max_value=300),
            blood_pressure_diastolic=_clamp_int(dia, min_value=0, max_value=300),
            body_temperature=_clamp_float(temp, min_value=0.0, max_value=50.0),
            oxygen_saturation=_clamp_int(spo2, min_value=0, max_value=100),
            status='monitoring',
        )


class I2CVitalDevice(VitalDevice):
    """Optional I2C device implementation.

    This is intentionally minimal: it provides a clear failure mode when the
    environment lacks required libraries or device nodes.
    """

    def __init__(self, *, patient_id: str, bus: int, address: int):
        self._patient_id = str(patient_id)
        self._bus = int(bus)
        self._address = int(address)
        self._opened = False

    def open(self) -> None:
        # Import inside to keep dependency optional.
        try:
            import smbus2  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise VitalDeviceError(
                'I2C driver requires smbus2 (optional dependency). '
                'Install: apt-get python3-smbus2 or pip install --user smbus2'
            ) from exc

        dev_path = f'/dev/i2c-{self._bus}'
        try:
            with open(dev_path, 'rb'):
                pass
        except FileNotFoundError as exc:  # pragma: no cover
            raise VitalDeviceError(
                f'I2C device node not found: {dev_path}. '
                'Check bus number and that /dev/i2c-* exists. '
                'On Linux: ensure i2c-dev is enabled (e.g., sudo modprobe i2c-dev). '
                'On WSL: I2C devices are typically unavailable.'
            ) from exc
        except PermissionError as exc:  # pragma: no cover
            raise VitalDeviceError(
                f'I2C device node permission denied: {dev_path}. '
                'Fix permissions (e.g., add user to i2c group, udev rules) '
                'or run with appropriate privileges.'
            ) from exc
        except Exception as exc:  # pragma: no cover
            raise VitalDeviceError(
                f'I2C device not accessible: {dev_path}'
            ) from exc

        # Keep handle for future extension.
        try:
            self._bus_handle = smbus2.SMBus(self._bus)
        except Exception as exc:  # pragma: no cover
            raise VitalDeviceError(f'Failed to open I2C bus {self._bus}') from exc

        self._opened = True

    def close(self) -> None:
        self._opened = False
        handle = getattr(self, '_bus_handle', None)
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def read_sample(self) -> VitalSample:
        if not self._opened:
            raise VitalDeviceError('Device is not opened')
        raise VitalDeviceError('I2C read_sample is not implemented yet')


class SPIVitalDevice(VitalDevice):
    """Optional SPI device implementation (stub)."""

    def __init__(self, *, patient_id: str, bus: int, device: int):
        self._patient_id = str(patient_id)
        self._bus = int(bus)
        self._device = int(device)
        self._opened = False

    def open(self) -> None:
        try:
            import spidev  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise VitalDeviceError(
                'SPI driver requires spidev (optional dependency). '
                'Install: apt-get python3-spidev or pip install --user spidev'
            ) from exc

        try:
            dev = spidev.SpiDev()
            dev.open(self._bus, self._device)
            self._spi_handle = dev
        except Exception as exc:  # pragma: no cover
            raise VitalDeviceError(
                f'Failed to open SPI bus={self._bus} device={self._device}'
            ) from exc

        self._opened = True

    def close(self) -> None:
        self._opened = False
        handle = getattr(self, '_spi_handle', None)
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def read_sample(self) -> VitalSample:
        if not self._opened:
            raise VitalDeviceError('Device is not opened')
        raise VitalDeviceError('SPI read_sample is not implemented yet')


def parse_int_auto_base(text: str) -> int:
    """Parse integer, allowing 0x.. hex strings."""
    return int(str(text).strip(), 0)
