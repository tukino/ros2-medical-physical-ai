"""Day10: fault_injection のユニットテスト."""

from __future__ import annotations

import random

import pytest


def test_validate_fault_params_accepts_defaults():
    from medical_robot_sim.fault_injection import validate_fault_params

    validate_fault_params(
        drop_rate=0.0,
        delay_ms=0,
        jitter_ms=0,
        pause_after_sec=0.0,
        pause_duration_sec=0.0,
        stop_after_sec=0.0,
        publish_rate_hz=1.0,
    )


@pytest.mark.parametrize(
    ('drop_rate', 'delay_ms', 'jitter_ms', 'pause_after', 'pause_dur', 'stop_after'),
    [
        (1.1, 0, 0, 0.0, 0.0, 0.0),
        (-0.1, 0, 0, 0.0, 0.0, 0.0),
        (0.0, -1, 0, 0.0, 0.0, 0.0),
        (0.0, 0, -1, 0.0, 0.0, 0.0),
        (0.0, 0, 0, -1.0, 0.0, 0.0),
        (0.0, 0, 0, 0.0, -1.0, 0.0),
        (0.0, 0, 0, 0.0, 0.0, -1.0),
    ],
)
def test_validate_fault_params_rejects_invalid_values(
    drop_rate: float,
    delay_ms: int,
    jitter_ms: int,
    pause_after: float,
    pause_dur: float,
    stop_after: float,
):
    from medical_robot_sim.fault_injection import validate_fault_params

    with pytest.raises(ValueError):
        validate_fault_params(
            drop_rate=drop_rate,
            delay_ms=delay_ms,
            jitter_ms=jitter_ms,
            pause_after_sec=pause_after,
            pause_duration_sec=pause_dur,
            stop_after_sec=stop_after,
            publish_rate_hz=1.0,
        )


def test_ms_to_ticks_boundaries():
    from medical_robot_sim.fault_injection import ms_to_ticks

    assert ms_to_ticks(0, 1.0) == 0
    assert ms_to_ticks(-1, 1.0) == 0

    # 1Hz => period 1000ms
    assert ms_to_ticks(1, 1.0) == 1
    assert ms_to_ticks(1000, 1.0) == 1
    assert ms_to_ticks(1001, 1.0) == 2

    # 10Hz => period 100ms
    assert ms_to_ticks(1, 10.0) == 1
    assert ms_to_ticks(100, 10.0) == 1
    assert ms_to_ticks(101, 10.0) == 2


def test_should_drop_extremes_and_reproducibility():
    from medical_robot_sim.fault_injection import should_drop

    rng = random.Random(0)
    assert should_drop(0.0, rng) is False

    rng = random.Random(0)
    assert should_drop(1.0, rng) is True

    rng1 = random.Random(42)
    rng2 = random.Random(42)
    seq1 = [should_drop(0.5, rng1) for _ in range(20)]
    seq2 = [should_drop(0.5, rng2) for _ in range(20)]
    assert seq1 == seq2


def test_compute_delay_ticks_range():
    from medical_robot_sim.fault_injection import compute_delay_ticks
    from medical_robot_sim.fault_injection import ms_to_ticks

    rng = random.Random(0)

    # no delay/jitter
    assert compute_delay_ticks(0, 0, 1.0, rng) == 0

    # deterministic delay
    rng = random.Random(0)
    assert compute_delay_ticks(500, 0, 1.0, rng) == 1

    # jitter: result should be within [min_ticks, max_ticks]
    rng = random.Random(123)
    delay_ms = 200
    jitter_ms = 300
    min_ticks = ms_to_ticks(delay_ms, 10.0)
    max_ticks = ms_to_ticks(delay_ms + jitter_ms, 10.0)

    ticks = compute_delay_ticks(delay_ms, jitter_ms, 10.0, rng)
    assert min_ticks <= ticks <= max_ticks
