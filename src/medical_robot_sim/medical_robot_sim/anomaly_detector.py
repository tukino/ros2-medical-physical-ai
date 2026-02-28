"""
ICU 監視の異常検知レイヤ.

将来の拡張に備えて呼び出しインターフェースを固定化する.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from dataclasses import field
import math
from typing import Deque
from typing import Dict
from typing import List
from typing import Mapping
from typing import Tuple

from medical_interfaces.msg import VitalSigns

from medical_robot_sim.types import AnomalyEvent


DEFAULT_WINDOW_SEC = 10

# 直近10秒を「サンプル数」に換算したデフォルト（1Hz想定）
DEFAULT_WINDOW_SIZE = 10

DEFAULT_SPO2_DROP_THRESHOLD = 4.0
DEFAULT_HR_JUMP_THRESHOLD = 20.0

# 変動が「極小」とみなす許容レンジ（max-min）
DEFAULT_FIELD_EPSILON: Mapping[str, float] = {
    'heart_rate': 1.0,
    'oxygen_saturation': 1.0,
    'blood_pressure_systolic': 2.0,
    'blood_pressure_diastolic': 2.0,
    'body_temperature': 0.05,
}


def _to_float(value) -> float | None:
    """値を float に変換する（失敗したら None）."""

    if value is None:
        return None
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(x):
        return None
    return x


@dataclass(slots=True)
class FlatlineDetector:
    """直近 N サンプルで値がほぼ変化しない（flatline）状態を検知する."""

    window_sec: int = DEFAULT_WINDOW_SEC
    window_size: int = DEFAULT_WINDOW_SIZE
    field_epsilon: Mapping[str, float] = field(
        default_factory=lambda: dict(DEFAULT_FIELD_EPSILON)
    )

    spo2_drop_threshold: float = DEFAULT_SPO2_DROP_THRESHOLD
    hr_jump_threshold: float = DEFAULT_HR_JUMP_THRESHOLD

    _history: Dict[str, Deque[Tuple[float, Dict[str, float | None]]]] = field(
        init=False, default_factory=dict, repr=False
    )
    _active: Dict[Tuple[str, str, str], bool] = field(
        init=False, default_factory=dict, repr=False
    )

    def __post_init__(self) -> None:
        if self.window_sec <= 0:
            raise ValueError('window_sec must be > 0')
        if self.window_size <= 1:
            raise ValueError('window_size must be >= 2')

    def reset(self) -> None:
        """内部状態（履歴/発火状態）を初期化する."""
        self._history.clear()
        self._active.clear()

    def update(self, vitals: VitalSigns, ts: float | None = None) -> List[AnomalyEvent]:
        """サンプルを追加してイベントを返す."""
        sample_ts = float(time.time() if ts is None else ts)
        patient_id = str(getattr(vitals, 'patient_id', '') or 'unknown')

        values: Dict[str, float | None] = {
            'heart_rate': _to_float(getattr(vitals, 'heart_rate', None)),
            'oxygen_saturation': _to_float(getattr(vitals, 'oxygen_saturation', None)),
            'blood_pressure_systolic': _to_float(
                getattr(vitals, 'blood_pressure_systolic', None)
            ),
            'blood_pressure_diastolic': _to_float(
                getattr(vitals, 'blood_pressure_diastolic', None)
            ),
            'body_temperature': _to_float(getattr(vitals, 'body_temperature', None)),
        }

        history = self._history.get(patient_id)
        if history is None:
            history = deque(maxlen=self.window_size)
            self._history[patient_id] = history

        history.append((sample_ts, values))

        if len(history) < self.window_size:
            return []

        events: List[AnomalyEvent] = []
        window_sec = int(self.window_sec)

        # ----- drop/jump（直近 window_sec 秒 ≒ window_size サンプル） -----
        spo2_cur = history[-1][1].get('oxygen_saturation')
        if spo2_cur is not None:
            spo2_values = [
                frame_values.get('oxygen_saturation')
                for _, frame_values in history
                if frame_values.get('oxygen_saturation') is not None
            ]
            if spo2_values:
                spo2_baseline = max(spo2_values)
                delta = float(spo2_cur) - float(spo2_baseline)
                is_drop = delta <= -float(self.spo2_drop_threshold)
                key = (patient_id, 'spo2_drop', 'oxygen_saturation')
                was_active = self._active.get(key, False)
                self._active[key] = is_drop
                if is_drop and not was_active:
                    score = abs(delta) / float(self.spo2_drop_threshold)
                    events.append(
                        AnomalyEvent(
                            type='spo2_drop',
                            field='oxygen_saturation',
                            value=float(spo2_cur),
                            delta=delta,
                            score=float(score),
                            ts=sample_ts,
                            window_sec=window_sec,
                        )
                    )

        hr_cur = history[-1][1].get('heart_rate')
        if hr_cur is not None:
            hr_values = [
                frame_values.get('heart_rate')
                for _, frame_values in history
                if frame_values.get('heart_rate') is not None
            ]
            if hr_values:
                hr_baseline = min(hr_values)
                delta = float(hr_cur) - float(hr_baseline)
                is_jump = delta >= float(self.hr_jump_threshold)
                key = (patient_id, 'hr_jump', 'heart_rate')
                was_active = self._active.get(key, False)
                self._active[key] = is_jump
                if is_jump and not was_active:
                    score = delta / float(self.hr_jump_threshold)
                    events.append(
                        AnomalyEvent(
                            type='hr_jump',
                            field='heart_rate',
                            value=float(hr_cur),
                            delta=delta,
                            score=float(score),
                            ts=sample_ts,
                            window_sec=window_sec,
                        )
                    )

        # ----- flatline -----
        for field_name, epsilon in self.field_epsilon.items():
            field_values = [
                frame_values.get(field_name)
                for _, frame_values in history
                if frame_values.get(field_name) is not None
            ]
            if len(field_values) < 2:
                continue
            min_value = min(field_values)
            max_value = max(field_values)
            value_range = max_value - min_value
            is_flat = value_range <= float(epsilon)

            key = (patient_id, 'flatline', field_name)
            was_active = self._active.get(key, False)
            self._active[key] = is_flat

            if is_flat and not was_active:
                events.append(
                    AnomalyEvent(
                        type='flatline',
                        field=field_name,
                        value=float(field_values[-1]),
                        delta=value_range,
                        score=1.0,
                        ts=sample_ts,
                        window_sec=window_sec,
                    )
                )

        return events


_DEFAULT_DETECTOR = FlatlineDetector()


def reset_default_detector() -> None:
    """テスト等で module-level detector を初期化する."""
    _DEFAULT_DETECTOR.reset()


def detect_anomalies(vitals: VitalSigns, ts: float | None = None) -> List[AnomalyEvent]:
    """
    異常イベントを抽出する.

    現時点では flatline（直近 N サンプルで同値連続/変動極小）を検知する.
    """
    return _DEFAULT_DETECTOR.update(vitals, ts=ts)
