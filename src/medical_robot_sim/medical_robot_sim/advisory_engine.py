"""
ICU 監視の助言（優先度付け）ロジック.

生のバイタル（および任意の検知イベント）から、ダッシュボード表示用の
alert ラベルへ変換する.
"""

from __future__ import annotations

from typing import Iterable, Optional

from medical_interfaces.msg import VitalSigns

from medical_robot_sim.types import AnomalyEvent
from medical_robot_sim.rule_evaluator import evaluate_event_level
from medical_robot_sim.rule_evaluator import level_to_alert


def calc_alert(vitals: VitalSigns, events: Optional[Iterable[AnomalyEvent]] = None) -> str:
    """
    ダッシュボード表示用の alert ラベルを返す.

    Notes
    -----
    - `events` は将来拡張のために受け取る（現状は未使用）.
    - 現状の挙動は元のしきい値ロジックと一致させる.

    """
    # 生値しきい値によるステータス（従来挙動）
    raw_alert = _calc_raw_alert(vitals)

    # イベントからのステータス（イベント優先）
    event_alert = 'OK'
    if events is not None:
        level = evaluate_event_level(events)
        event_alert = level_to_alert(level)

    # 最終決定: イベントを優先しつつ、しきい値アラートを下げない
    return _max_alert(raw_alert, event_alert)


def _calc_raw_alert(vitals: VitalSigns) -> str:
    """生値しきい値ベースの alert を返す（従来ロジック）."""

    if vitals.oxygen_saturation < 90:
        return 'RED'
    if vitals.heart_rate > 120:
        return 'ORANGE'
    if vitals.blood_pressure_systolic > 160:
        return 'YELLOW'
    return 'OK'


def _max_alert(a: str, b: str) -> str:
    """2つの alert のうち優先度が高い方を返す."""

    order = {
        'OK': 0,
        'YELLOW': 1,
        'ORANGE': 2,
        'RED': 3,
    }
    if order.get(b, -1) > order.get(a, -1):
        return b
    return a
