"""Day8: qos_profiles のユニットテスト."""

from __future__ import annotations

import pytest


def test_build_qos_profile_defaults_match_expected():
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSReliabilityPolicy,
    )

    from medical_robot_sim.qos_profiles import build_qos_profile

    qos = build_qos_profile(depth=10, reliability='reliable', durability='volatile')

    assert qos.history == QoSHistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == QoSReliabilityPolicy.RELIABLE
    assert qos.durability == QoSDurabilityPolicy.VOLATILE


@pytest.mark.parametrize(
    ('reliability', 'expected'),
    [
        ('RELIABLE', 'RELIABLE'),
        ('Reliable', 'RELIABLE'),
        ('best_effort', 'BEST_EFFORT'),
        ('Best-Effort', 'BEST_EFFORT'),
    ],
)
def test_build_qos_profile_reliability_normalization(reliability: str, expected: str):
    from rclpy.qos import QoSReliabilityPolicy

    from medical_robot_sim.qos_profiles import build_qos_profile

    qos = build_qos_profile(depth=1, reliability=reliability, durability='volatile')
    assert qos.reliability == getattr(QoSReliabilityPolicy, expected)


@pytest.mark.parametrize(
    ('durability', 'expected'),
    [
        ('VOLATILE', 'VOLATILE'),
        ('volatile', 'VOLATILE'),
        ('transient_local', 'TRANSIENT_LOCAL'),
        ('Transient Local', 'TRANSIENT_LOCAL'),
    ],
)
def test_build_qos_profile_durability_normalization(durability: str, expected: str):
    from rclpy.qos import QoSDurabilityPolicy

    from medical_robot_sim.qos_profiles import build_qos_profile

    qos = build_qos_profile(depth=1, reliability='reliable', durability=durability)
    assert qos.durability == getattr(QoSDurabilityPolicy, expected)


def test_build_qos_profile_rejects_invalid_values():
    from medical_robot_sim.qos_profiles import build_qos_profile

    with pytest.raises(ValueError, match='Invalid reliability'):
        build_qos_profile(depth=10, reliability='invalid_value', durability='volatile')

    with pytest.raises(ValueError, match='Invalid durability'):
        build_qos_profile(depth=10, reliability='reliable', durability='invalid_value')

    with pytest.raises(ValueError, match='depth must be >= 1'):
        build_qos_profile(depth=0, reliability='reliable', durability='volatile')
