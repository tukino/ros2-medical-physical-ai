"""
イベント列からルール評価（ステータス）を決定する.

`spo2_drop` と `hr_jump` の組合せを優先度付けし、最終ステータスを返す.
"""

from __future__ import annotations

from typing import Iterable

from medical_robot_sim.types import AnomalyEvent


def evaluate_event_level(events: Iterable[AnomalyEvent]) -> str:
    """イベント列からステータス（CRITICAL/WARN/OK）を返す."""

    types = {e.type for e in events}
    if 'spo2_drop' in types and 'hr_jump' in types:
        return 'CRITICAL'
    if 'spo2_drop' in types or 'hr_jump' in types:
        return 'WARN'
    return 'OK'


def level_to_alert(level: str) -> str:
    """ステータスを ICU ダッシュボードの alert 表示へ変換する."""

    if level == 'CRITICAL':
        return 'RED'
    if level == 'WARN':
        return 'YELLOW'
    return 'OK'
