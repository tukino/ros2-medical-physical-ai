"""Day10: fault injection utilities (pure functions).

This module is intentionally rclpy-free to keep unit tests lightweight.
"""

from __future__ import annotations

import math
import random


def validate_fault_params(
    *,
    drop_rate: float,
    delay_ms: int,
    jitter_ms: int,
    pause_after_sec: float,
    pause_duration_sec: float,
    stop_after_sec: float,
    publish_rate_hz: float,
) -> None:
    """Validate fault injection parameters.

    Raises:
        ValueError: if any parameter is invalid.
    """

    if publish_rate_hz <= 0.0:
        raise ValueError('publish_rate_hz must be > 0')

    if not (0.0 <= float(drop_rate) <= 1.0):
        raise ValueError('drop_rate must be between 0.0 and 1.0')

    if int(delay_ms) < 0:
        raise ValueError('delay_ms must be >= 0')

    if int(jitter_ms) < 0:
        raise ValueError('jitter_ms must be >= 0')

    if float(pause_after_sec) < 0.0:
        raise ValueError('pause_after_sec must be >= 0.0')

    if float(pause_duration_sec) < 0.0:
        raise ValueError('pause_duration_sec must be >= 0.0')

    if float(stop_after_sec) < 0.0:
        raise ValueError('stop_after_sec must be >= 0.0')


def ms_to_ticks(ms: int, publish_rate_hz: float) -> int:
    """Convert milliseconds to timer ticks.

    Policy:
    - ms <= 0 => 0 ticks
    - ms > 0  => at least 1 tick

    This uses ceil() so the effective delay is never shorter than requested.
    """

    if ms <= 0:
        return 0

    if publish_rate_hz <= 0.0:
        raise ValueError('publish_rate_hz must be > 0')

    period_ms = 1000.0 / float(publish_rate_hz)
    ticks = int(math.ceil(float(ms) / period_ms))
    return max(1, ticks)


def compute_delay_ticks(
    delay_ms: int,
    jitter_ms: int,
    publish_rate_hz: float,
    rng: random.Random,
) -> int:
    """Compute delay ticks for delay+jitter."""

    base = int(delay_ms)
    jitter = int(jitter_ms)

    extra = 0
    if jitter > 0:
        extra = int(rng.randint(0, jitter))

    return ms_to_ticks(base + extra, publish_rate_hz)


def should_drop(drop_rate: float, rng: random.Random) -> bool:
    """Return True if the current sample should be dropped."""

    rate = float(drop_rate)
    if rate <= 0.0:
        return False
    if rate >= 1.0:
        return True
    return bool(rng.random() < rate)
