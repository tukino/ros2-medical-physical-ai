"""BCI patient state classification (Day15).

This module is intentionally rclpy-free for lightweight unit tests.
"""

from __future__ import annotations

from typing import Optional


def classify_bci_state(
    age_sec: Optional[float],
    *,
    stale_after_sec: float = 3.0,
    no_data_after_sec: float = 10.0,
) -> str:
    """Classify BCI data state from last-seen age.

    - FRESH:    age <= stale_after_sec
    - STALE:    stale_after_sec < age <= no_data_after_sec
    - NO DATA:  age is None (never seen) or age > no_data_after_sec
    """

    if age_sec is None or float(age_sec) > float(no_data_after_sec):
        return 'NO DATA'
    if float(age_sec) > float(stale_after_sec):
        return 'STALE'
    return 'FRESH'
