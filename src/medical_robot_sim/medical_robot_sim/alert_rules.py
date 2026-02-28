"""Day5: Alert Rules（純粋関数）.

このモジュールは、アラート判定ロジックを「純粋関数」として切り出すためのものです。

- 入力: 直近の履歴（サンプル列）としきい値などのパラメータ
- 出力: 各ルールの active 状態と、Alert生成に必要なメタ情報

ノード側（rule_alert_engine）は、この結果を用いて
エッジトリガ（False→True）や cooldown、publish を担当します。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Sequence


@dataclass(frozen=True, slots=True)
class Sample:
    """ルール評価用の入力サンプル（ROSメッセージに依存しない形）."""

    ts: float
    heart_rate: float
    blood_pressure_systolic: float
    blood_pressure_diastolic: float
    body_temperature: float
    oxygen_saturation: float


@dataclass(frozen=True, slots=True)
class RuleParams:
    """ルール評価用パラメータ."""

    history_size: int
    spo2_drop_threshold: float = 4.0
    hr_jump_threshold: float = 20.0


@dataclass(frozen=True, slots=True)
class RuleMatch:
    """ルール評価結果（Alert.msg生成に必要な要素を保持）."""

    rule_id: str
    kind: str
    priority: str
    message: str
    window_sec: int
    field: str = ''
    value: float = math.nan
    delta: float = math.nan
    score: float = math.nan
    active: bool = False


def evaluate_alert_rules(history: Sequence[Sample], params: RuleParams) -> Dict[str, RuleMatch]:
    """ルールを評価して、各ルールの active 状態とメタ情報を返す.

    - history: 直近の履歴（末尾が最新サンプル）
    - params: しきい値等
    """

    if not history:
        return {}

    current = history[-1]

    matches: Dict[str, RuleMatch] = {}

    # --- 単発ルール（1サンプルで判定） ---
    matches['single.spo2_lt_90'] = RuleMatch(
        rule_id='single.spo2_lt_90',
        kind='single',
        priority='RED',
        message='SpO2 が 90% 未満',
        window_sec=0,
        field='oxygen_saturation',
        value=float(current.oxygen_saturation),
        active=float(current.oxygen_saturation) < 90.0,
    )

    matches['single.hr_gt_120'] = RuleMatch(
        rule_id='single.hr_gt_120',
        kind='single',
        priority='ORANGE',
        message='心拍数(HR)が 120bpm 超',
        window_sec=0,
        field='heart_rate',
        value=float(current.heart_rate),
        active=float(current.heart_rate) > 120.0,
    )

    matches['single.sbp_gt_160'] = RuleMatch(
        rule_id='single.sbp_gt_160',
        kind='single',
        priority='YELLOW',
        message='収縮期血圧(SBP)が 160mmHg 超',
        window_sec=0,
        field='blood_pressure_systolic',
        value=float(current.blood_pressure_systolic),
        active=float(current.blood_pressure_systolic) > 160.0,
    )

    # --- 特異値ルール（データ整合性/現実的レンジ） ---
    matches['outlier.dbp_ge_sbp'] = RuleMatch(
        rule_id='outlier.dbp_ge_sbp',
        kind='outlier',
        priority='ORANGE',
        message='血圧の整合性異常（DBP>=SBP）',
        window_sec=0,
        field='blood_pressure_diastolic',
        value=float(current.blood_pressure_diastolic),
        active=float(current.blood_pressure_diastolic) >= float(current.blood_pressure_systolic),
    )

    temp_out = float(current.body_temperature) < 30.0 or float(current.body_temperature) > 42.0
    matches['outlier.temp_out_of_range'] = RuleMatch(
        rule_id='outlier.temp_out_of_range',
        kind='outlier',
        priority='ORANGE',
        message='体温が現実的レンジ外（<30°C または >42°C）',
        window_sec=0,
        field='body_temperature',
        value=float(current.body_temperature),
        active=bool(temp_out),
    )

    # --- 変化率ルール（履歴が必要） ---
    spo2_active = False
    hr_active = False

    if len(history) >= 2:
        spo2_values = [float(s.oxygen_saturation) for s in history]
        spo2_baseline = max(spo2_values) if spo2_values else float(current.oxygen_saturation)
        spo2_delta = float(current.oxygen_saturation) - float(spo2_baseline)
        spo2_active = spo2_delta <= -float(params.spo2_drop_threshold)
        spo2_score = abs(spo2_delta) / float(params.spo2_drop_threshold)

        matches['roc.spo2_drop'] = RuleMatch(
            rule_id='roc.spo2_drop',
            kind='rate_of_change',
            priority='YELLOW',
            message='SpO2 が window 内で閾値以上低下',
            window_sec=int(params.history_size),
            field='oxygen_saturation',
            value=float(current.oxygen_saturation),
            delta=float(spo2_delta),
            score=float(spo2_score),
            active=bool(spo2_active),
        )

        hr_values = [float(s.heart_rate) for s in history]
        hr_baseline = min(hr_values) if hr_values else float(current.heart_rate)
        hr_delta = float(current.heart_rate) - float(hr_baseline)
        hr_active = hr_delta >= float(params.hr_jump_threshold)
        hr_score = hr_delta / float(params.hr_jump_threshold)

        matches['roc.hr_jump'] = RuleMatch(
            rule_id='roc.hr_jump',
            kind='rate_of_change',
            priority='YELLOW',
            message='HR が window 内で閾値以上上昇',
            window_sec=int(params.history_size),
            field='heart_rate',
            value=float(current.heart_rate),
            delta=float(hr_delta),
            score=float(hr_score),
            active=bool(hr_active),
        )
    else:
        matches['roc.spo2_drop'] = RuleMatch(
            rule_id='roc.spo2_drop',
            kind='rate_of_change',
            priority='YELLOW',
            message='SpO2 が window 内で閾値以上低下',
            window_sec=int(params.history_size),
            field='oxygen_saturation',
            value=float(current.oxygen_saturation),
            active=False,
        )
        matches['roc.hr_jump'] = RuleMatch(
            rule_id='roc.hr_jump',
            kind='rate_of_change',
            priority='YELLOW',
            message='HR が window 内で閾値以上上昇',
            window_sec=int(params.history_size),
            field='heart_rate',
            value=float(current.heart_rate),
            active=False,
        )

    # --- 組合せルール ---
    combo_active = bool(len(history) >= 2 and spo2_active and hr_active)
    matches['combo.spo2_drop_and_hr_jump'] = RuleMatch(
        rule_id='combo.spo2_drop_and_hr_jump',
        kind='combination',
        priority='RED',
        message='SpO2 低下 + HR 急上昇（組合せ）',
        window_sec=int(params.history_size),
        active=bool(combo_active),
    )

    return matches
