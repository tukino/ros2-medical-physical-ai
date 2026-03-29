"""BCI device I/O abstraction (Day15).

This module is intentionally rclpy-free to keep unit tests lightweight.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Optional


def _clamp01(value: float) -> float:
    v = float(value)
    if math.isnan(v):
        return v
    return max(0.0, min(1.0, v))


@dataclass(frozen=True)
class BCIFeatureSample:
    """A single feature sample produced by a BCI device."""

    attention: float
    drowsiness: float
    signal_quality: float
    status: str


class BCIDevice:
    """BCI device interface."""

    def open(self) -> None:
        raise NotImplementedError

    def read_features(self) -> BCIFeatureSample:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class MockBCIDevice(BCIDevice):
    """Deterministic mock device for CI/reproducibility."""

    def __init__(
        self,
        *,
        scenario: str,
        seed: int,
    ):
        self._scenario = str(scenario or 'normal').strip().lower()
        self._rng = random.Random(int(seed))
        self._opened = False
        self._tick = 0

    @property
    def scenario(self) -> str:
        return self._scenario

    def open(self) -> None:
        self._opened = True

    def close(self) -> None:
        self._opened = False

    def read_features(self) -> BCIFeatureSample:
        if not self._opened:
            raise RuntimeError('device not opened')

        t = int(self._tick)
        self._tick += 1

        if self._scenario in {'', 'normal'}:
            attention = 0.65 + self._rng.uniform(-0.10, 0.10)
            drowsiness = 0.30 + self._rng.uniform(-0.10, 0.10)
            quality = 0.90 + self._rng.uniform(-0.05, 0.05)
        elif self._scenario == 'drowsy':
            # Gradually drift to drowsy.
            progress = min(1.0, t / 80.0)
            attention = (0.65 * (1.0 - progress)) + (0.25 * progress)
            attention += self._rng.uniform(-0.05, 0.05)

            drowsiness = (0.30 * (1.0 - progress)) + (0.80 * progress)
            drowsiness += self._rng.uniform(-0.05, 0.05)

            quality = 0.85 + self._rng.uniform(-0.08, 0.04)
        elif self._scenario == 'artifact_spike':
            attention = 0.60 + self._rng.uniform(-0.12, 0.12)
            drowsiness = 0.35 + self._rng.uniform(-0.12, 0.12)
            quality = 0.80 + self._rng.uniform(-0.30, 0.05)

            # Rare spikes (simulating motion/EMG artifacts).
            if self._rng.random() < 0.08:
                attention = 1.0
                drowsiness = 0.0
            if self._rng.random() < 0.08:
                attention = 0.0
                drowsiness = 1.0
        else:
            raise ValueError(
                "Invalid mock scenario. Allowed values: normal, drowsy, artifact_spike"
            )

        attention = _clamp01(attention)
        drowsiness = _clamp01(drowsiness)
        quality = _clamp01(quality)

        status = 'ok'
        if isinstance(quality, float) and not math.isnan(quality):
            if quality < 0.05:
                status = 'no_signal'
            elif quality < 0.30:
                status = 'degraded'

        return BCIFeatureSample(
            attention=float(attention),
            drowsiness=float(drowsiness),
            signal_quality=float(quality),
            status=str(status),
        )


class SerialBCIDevice(BCIDevice):
    """Optional serial device.

    This is intentionally minimal. If pyserial is missing, construction raises ImportError.
    """

    def __init__(
        self,
        *,
        port: str,
        baudrate: int = 115200,
        timeout_sec: float = 1.0,
    ):
        try:
            import serial  # type: ignore
        except Exception as exc:
            raise ImportError('pyserial is required for driver=serial') from exc

        self._serial_mod = serial
        self._port = str(port)
        self._baudrate = int(baudrate)
        self._timeout_sec = float(timeout_sec)
        self._ser: Optional[object] = None

    def open(self) -> None:
        if not self._port:
            raise ValueError('serial port is empty')
        self._ser = self._serial_mod.Serial(
            port=self._port,
            baudrate=self._baudrate,
            timeout=self._timeout_sec,
        )

    def close(self) -> None:
        ser = self._ser
        self._ser = None
        try:
            if ser is not None:
                ser.close()
        except Exception:
            return

    def read_features(self) -> BCIFeatureSample:
        ser = self._ser
        if ser is None:
            raise RuntimeError('device not opened')

        raw = ser.readline()
        if raw is None:
            raise RuntimeError('serial returned None')

        try:
            line = raw.decode('utf-8', errors='replace').strip()
        except Exception:
            line = str(raw).strip()

        if not line:
            raise TimeoutError('no serial data')

        # Supported formats (one line):
        # 1) CSV: attention,drowsiness,signal_quality[,status]
        # 2) KV:  attention=0.1 drowsiness=0.2 signal_quality=0.9 status=ok
        attention = math.nan
        drowsiness = math.nan
        quality = math.nan
        status = ''

        if '=' in line:
            # Normalize separators.
            tokens = [t for t in line.replace(',', ' ').split(' ') if t.strip()]
            fields = {}
            for token in tokens:
                if '=' not in token:
                    continue
                k, v = token.split('=', 1)
                fields[str(k).strip().lower()] = str(v).strip()

            if 'attention' in fields:
                attention = float(fields['attention'])
            if 'drowsiness' in fields:
                drowsiness = float(fields['drowsiness'])
            if 'signal_quality' in fields:
                quality = float(fields['signal_quality'])
            if 'status' in fields:
                status = str(fields['status'])
        else:
            parts = [p.strip() for p in line.split(',')]
            if len(parts) >= 1 and parts[0]:
                attention = float(parts[0])
            if len(parts) >= 2 and parts[1]:
                drowsiness = float(parts[1])
            if len(parts) >= 3 and parts[2]:
                quality = float(parts[2])
            if len(parts) >= 4 and parts[3]:
                status = str(parts[3])

        attention = _clamp01(attention)
        drowsiness = _clamp01(drowsiness)
        quality = _clamp01(quality)

        if not status:
            status = 'ok'
            if isinstance(quality, float) and not math.isnan(quality):
                if quality < 0.05:
                    status = 'no_signal'
                elif quality < 0.30:
                    status = 'degraded'

        return BCIFeatureSample(
            attention=float(attention),
            drowsiness=float(drowsiness),
            signal_quality=float(quality),
            status=str(status),
        )
