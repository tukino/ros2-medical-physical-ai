from __future__ import annotations

from typing import Dict
from typing import Optional
from typing import Tuple


def is_ready(message_counts: Dict[str, int], *, min_messages_per_patient: int) -> bool:
    """Return True if all patients have received at least min_messages_per_patient."""

    if int(min_messages_per_patient) <= 0:
        return True

    for _, count in message_counts.items():
        if int(count) < int(min_messages_per_patient):
            return False
    return True


def is_no_data(age_sec: Optional[float], *, no_data_after_sec: float) -> bool:
    """Return True if data is considered NO_DATA.

    - age_sec is None: never seen => NO_DATA
    - age_sec > no_data_after_sec => NO_DATA
    """

    if age_sec is None:
        return True
    return float(age_sec) > float(no_data_after_sec)


def all_patients_no_data(
    last_seen_age_sec: Dict[str, Optional[float]], *, no_data_after_sec: float
) -> bool:
    """Return True if all patients are in NO_DATA."""

    for _, age in last_seen_age_sec.items():
        if not is_no_data(age, no_data_after_sec=no_data_after_sec):
            return False
    return True


def lifecycle_action_to_reach_active(state_label: str) -> Optional[Tuple[str, str]]:
    """Return the next lifecycle action to reach active.

    Returns:
      (action_label, expected_state_label) or None if no action is needed/possible.

    This is a pure function so it can be unit-tested without ROS.
    """

    state = str(state_label).strip().lower()
    if state == 'active':
        return None
    if state == 'unconfigured':
        return ('configure', 'inactive')
    if state == 'inactive':
        return ('activate', 'active')
    return None


def lifecycle_action_to_reach_inactive(state_label: str) -> Optional[Tuple[str, str]]:
    """Return the next lifecycle action to reach inactive."""

    state = str(state_label).strip().lower()
    if state == 'active':
        return ('deactivate', 'inactive')
    return None
