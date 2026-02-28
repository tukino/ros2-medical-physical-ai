"""
ICU 監視ロジックで共通利用する型定義.

検知（anomaly detector）/助言（advisory engine）レイヤがやり取りする
イベント構造を定義する.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from enum import Enum
from typing import Any, Dict, Optional


class EventSeverity(str, Enum):
    """検知イベントの重大度."""

    RED = 'RED'
    ORANGE = 'ORANGE'
    YELLOW = 'YELLOW'
    INFO = 'INFO'


@dataclass(frozen=True, slots=True)
class Event:
    """
    検知されたイベント.

    Attributes
    ----------
    code
        下流処理で使う安定ID.
    severity
        重大度.
    message
        人が読むためのメッセージ.
    details
        デバッグ/テレメトリ用の任意詳細情報.

    """

    code: str
    severity: EventSeverity
    message: str
    details: Optional[Dict[str, Any]] = None


@dataclass(frozen=True, slots=True)
class VitalSample:
    """
    バイタルのスナップショット.

    ROS メッセージ（VitalSigns）をそのまま抱えるのではなく、検知ロジック側で
    扱いやすい形に正規化した値を保持する用途を想定する.
    """

    patient_id: str
    ts: float
    heart_rate: float
    blood_pressure_systolic: float
    blood_pressure_diastolic: float
    body_temperature: float
    oxygen_saturation: float


@dataclass(frozen=True, slots=True)
class AnomalyEvent:
    """
    異常検知イベント.

    ルール評価の結果をイベントとして表現し、下流の助言エンジン等が利用する.
    """

    type: str
    field: str | None
    value: float | str | None
    delta: float | None
    score: float | None
    ts: float
    window_sec: int


@dataclass(frozen=True, slots=True)
class RuleStatus:
    """
    ルールの評価状態.

    ルールが「発火したかどうか」だけでなく、評価に使ったスコアや説明文などを
    保持する用途を想定する.
    """

    rule_id: str
    ok: bool
    ts: float
    window_sec: int
    score: float | None = None
    message: str | None = None


@dataclass(frozen=True, slots=True)
class Advisory:
    """
    利用者へ提示する助言（サマリ）.

    複数イベント/ルール状態から生成した、表示・通知用のメッセージを表す.
    """

    level: str
    message: str
    ts: float
    window_sec: int
    events: list[AnomalyEvent] = field(default_factory=list)
    rules: list[RuleStatus] = field(default_factory=list)
