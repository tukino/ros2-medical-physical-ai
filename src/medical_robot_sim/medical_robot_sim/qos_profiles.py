"""QoSProfile builder utilities (Day8).

This module intentionally keeps the surface small and testable:
- string parameters -> rclpy QoS policy enums
- QoSProfile construction

Invalid inputs raise ValueError with actionable messages.
"""

from __future__ import annotations

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def _normalize_token(value: str) -> str:
    v = str(value).strip().lower()
    v = v.replace('-', '_').replace(' ', '_')
    return v


def _parse_reliability(value: str) -> QoSReliabilityPolicy:
    token = _normalize_token(value)
    if token == 'reliable':
        return QoSReliabilityPolicy.RELIABLE
    if token == 'best_effort':
        return QoSReliabilityPolicy.BEST_EFFORT
    allowed = 'reliable, best_effort'
    raise ValueError(f"Invalid reliability '{value}'. Allowed values: {allowed}")


def _parse_durability(value: str) -> QoSDurabilityPolicy:
    token = _normalize_token(value)
    if token == 'volatile':
        return QoSDurabilityPolicy.VOLATILE
    if token == 'transient_local':
        return QoSDurabilityPolicy.TRANSIENT_LOCAL
    allowed = 'volatile, transient_local'
    raise ValueError(f"Invalid durability '{value}'. Allowed values: {allowed}")


def build_qos_profile(*, depth: int, reliability: str, durability: str) -> QoSProfile:
    """Build a QoSProfile from minimal inputs.

    Defaults are expected to match the project's previous behavior when passing depth only:
    - history: KEEP_LAST
    - depth: 10
    - reliability: RELIABLE
    - durability: VOLATILE

    Args:
        depth: KEEP_LAST depth (must be >= 1)
        reliability: 'reliable' or 'best_effort' (case-insensitive)
        durability: 'volatile' or 'transient_local' (case-insensitive)

    Returns:
        rclpy.qos.QoSProfile

    Raises:
        ValueError: if any input is invalid
    """

    depth_int = int(depth)
    if depth_int < 1:
        raise ValueError(f"Invalid depth '{depth}'. depth must be >= 1")

    reliability_policy = _parse_reliability(reliability)
    durability_policy = _parse_durability(durability)

    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth_int,
        reliability=reliability_policy,
        durability=durability_policy,
    )
