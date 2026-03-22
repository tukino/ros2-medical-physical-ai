"""Day12: rosbag record/play 用の topic 組み立て純粋関数.

このモジュールは rclpy に依存しない。
"""

from __future__ import annotations


def _normalize_name(value: str) -> str:
    name = str(value).strip()
    name = name.strip('/')
    return name


def build_record_topics(patients: list[str], vitals_topic: str, include_alerts: bool) -> list[str]:
    """ros2 bag record 用の topic リストを組み立てる.

    Args:
        patients: patient namespace のリスト（例: ['patient_01', 'patient_02']）
        vitals_topic: vitals の相対 topic 名（例: 'patient_vitals'）
        include_alerts: true の場合、'alerts' も含める

    Returns:
        '/<pid>/<topic>' 形式の topic リスト

    Raises:
        ValueError: 有効な patients または vitals_topic が空の場合
    """

    normalized_vitals = _normalize_name(vitals_topic)
    if not normalized_vitals:
        raise ValueError('vitals_topic is empty')

    topics: list[str] = []

    for raw_pid in patients:
        pid = _normalize_name(raw_pid)
        if not pid:
            continue

        topics.append(f'/{pid}/{normalized_vitals}')
        if include_alerts:
            topics.append(f'/{pid}/alerts')

    if not topics:
        raise ValueError('patients is empty')

    return topics
