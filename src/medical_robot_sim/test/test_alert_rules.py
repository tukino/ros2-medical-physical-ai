import pytest

from medical_robot_sim.alert_rules import RuleParams
from medical_robot_sim.alert_rules import Sample
from medical_robot_sim.alert_rules import evaluate_alert_rules


def _s(
    *,
    ts: float,
    spo2: float = 98.0,
    hr: float = 70.0,
    sbp: float = 120.0,
    dbp: float = 80.0,
    temp: float = 36.5,
) -> Sample:
    return Sample(
        ts=float(ts),
        heart_rate=float(hr),
        blood_pressure_systolic=float(sbp),
        blood_pressure_diastolic=float(dbp),
        body_temperature=float(temp),
        oxygen_saturation=float(spo2),
    )


def test_single_spo2_lt_90_active():
    params = RuleParams(
        history_size=10,
        spo2_drop_threshold=4.0,
        hr_jump_threshold=20.0,
    )
    history = [_s(ts=0.0, spo2=89.0)]

    matches = evaluate_alert_rules(history, params)

    assert matches['single.spo2_lt_90'].active is True
    assert matches['single.spo2_lt_90'].priority == 'RED'


def test_roc_spo2_drop_detected():
    params = RuleParams(
        history_size=10,
        spo2_drop_threshold=4.0,
        hr_jump_threshold=20.0,
    )
    history = [
        _s(ts=0.0, spo2=98.0),
        _s(ts=1.0, spo2=97.0),
        _s(ts=2.0, spo2=93.0),
    ]

    matches = evaluate_alert_rules(history, params)

    roc = matches['roc.spo2_drop']
    assert roc.active is True
    assert roc.delta == pytest.approx(-5.0)
    assert roc.score == pytest.approx(1.25)


def test_combo_spo2_drop_and_hr_jump():
    params = RuleParams(
        history_size=10,
        spo2_drop_threshold=4.0,
        hr_jump_threshold=20.0,
    )
    history = [
        _s(ts=0.0, spo2=98.0, hr=70.0),
        _s(ts=1.0, spo2=97.0, hr=72.0),
        _s(ts=2.0, spo2=93.0, hr=95.0),
    ]

    matches = evaluate_alert_rules(history, params)

    assert matches['roc.spo2_drop'].active is True
    assert matches['roc.hr_jump'].active is True
    assert matches['combo.spo2_drop_and_hr_jump'].active is True


# ----- flatline（時間的安定性）ルールのテスト -----
# Day6 の設計: flatline_history_size=8（デフォルト）の単一窓で評価する。
# 複数窓は設計外。window_sec == 8 を明示的にアサートする。

def _params_flatline(
    flatline_history_size: int = 8,
    flatline_hr_epsilon: float = 1.0,
    flatline_spo2_epsilon: float = 1.0,
) -> RuleParams:
    return RuleParams(
        history_size=10,
        spo2_drop_threshold=4.0,
        hr_jump_threshold=20.0,
        flatline_history_size=flatline_history_size,
        flatline_hr_epsilon=flatline_hr_epsilon,
        flatline_spo2_epsilon=flatline_spo2_epsilon,
    )


def test_flatline_hr_active_when_constant():
    """HR が一定値で flatline_history_size=8 件以上のとき flatline.hr が active."""
    history = [_s(ts=float(i), hr=72.0, spo2=98.0) for i in range(8)]
    matches = evaluate_alert_rules(history, _params_flatline())

    m = matches['flatline.hr']
    assert m.active is True
    assert m.kind == 'temporal_stability'
    assert m.rule_id == 'flatline.hr'
    assert m.field == 'heart_rate'
    assert m.delta == pytest.approx(0.0)
    assert m.score == pytest.approx(1.0)
    assert m.window_sec == 8


def test_flatline_spo2_active_when_constant():
    """SpO2 が一定値で flatline_history_size=8 件以上のとき flatline.spo2 が active."""
    history = [_s(ts=float(i), hr=72.0, spo2=98.0) for i in range(8)]
    matches = evaluate_alert_rules(history, _params_flatline())

    m = matches['flatline.spo2']
    assert m.active is True
    assert m.kind == 'temporal_stability'
    assert m.delta == pytest.approx(0.0)
    assert m.window_sec == 8


def test_flatline_hr_not_active_when_variation_exceeds_epsilon():
    """HR の変動幅が epsilon を超えているとき flatline.hr は active にならない."""
    # 72 ~ 79 の範囲（range=7 > epsilon=1.0）
    history = [_s(ts=float(i), hr=float(72 + i), spo2=98.0) for i in range(8)]
    matches = evaluate_alert_rules(history, _params_flatline())

    assert matches['flatline.hr'].active is False


def test_flatline_spo2_not_active_when_variation_exceeds_epsilon():
    """SpO2 の変動幅が epsilon を超えているとき flatline.spo2 は active にならない."""
    history = [_s(ts=float(i), hr=72.0, spo2=float(98 - i)) for i in range(8)]
    matches = evaluate_alert_rules(history, _params_flatline())

    assert matches['flatline.spo2'].active is False


def test_flatline_not_active_when_history_too_short():
    """履歴が flatline_history_size=8 未満のとき flatline は active にならない."""
    history = [_s(ts=float(i), hr=72.0, spo2=98.0) for i in range(7)]  # 7件 < 8
    matches = evaluate_alert_rules(history, _params_flatline())

    assert matches['flatline.hr'].active is False
    assert matches['flatline.spo2'].active is False


def test_flatline_hr_boundary_exactly_at_epsilon():
    """HR の変動幅がちょうど epsilon に等しいとき flatline.hr は active になる（境界値）."""
    # max=73, min=72, range=1.0 = epsilon（境界）― 8サンプルで交互
    history = [_s(ts=float(i), hr=72.0 + (i % 2)) for i in range(8)]
    matches = evaluate_alert_rules(history, _params_flatline())

    assert matches['flatline.hr'].active is True
    assert matches['flatline.hr'].window_sec == 8


def test_flatline_uses_only_recent_window():
    """flatline 判定は末尾 flatline_history_size=8 件のみを使う."""
    # 先頭2件は大きな変動、末尾8件は固定値72
    history = (
        [_s(ts=0.0, hr=50.0), _s(ts=1.0, hr=110.0)]
        + [_s(ts=float(i + 2), hr=72.0) for i in range(8)]
    )
    # flatline_history_size=8 なので末尾8件（全て72）だけ見る → active
    matches = evaluate_alert_rules(history, _params_flatline())

    assert matches['flatline.hr'].active is True
    assert matches['flatline.hr'].window_sec == 8
