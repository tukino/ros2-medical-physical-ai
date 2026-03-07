from medical_robot_sim.alert_rules import RuleMatch
from medical_robot_sim.rule_alert_engine import _build_enabled_rule_id_set
from medical_robot_sim.rule_alert_engine import _select_edge_fired_matches


def _m(*, rule_id: str, active: bool) -> RuleMatch:
    return RuleMatch(
        rule_id=rule_id,
        kind='k',
        priority='INFO',
        message='msg',
        window_sec=0,
        active=bool(active),
    )


def test_enabled_rule_ids_empty_means_all_enabled():
    enabled = _build_enabled_rule_id_set([])
    assert enabled is None


def test_enabled_rule_ids_filters_publish_targets_even_if_active():
    pid = 'patient_01'

    matches = {
        'single.spo2_lt_90': _m(rule_id='single.spo2_lt_90', active=True),
        'roc.spo2_drop': _m(rule_id='roc.spo2_drop', active=True),
    }

    enabled = _build_enabled_rule_id_set(['single.spo2_lt_90'])
    active_state = {}

    fired = _select_edge_fired_matches(
        pid=pid,
        matches=matches,
        enabled_rule_ids=enabled,
        active_state=active_state,
    )

    assert [m.rule_id for m in fired] == ['single.spo2_lt_90']

    # フィルタ対象外のルールは edge 状態も更新しない（後で有効化したときに edge を取り逃さない）
    assert (pid, 'roc.spo2_drop') not in active_state


def test_enabled_rule_ids_does_not_emit_twice_for_same_edge():
    pid = 'patient_01'
    matches = {
        'single.spo2_lt_90': _m(rule_id='single.spo2_lt_90', active=True),
        'roc.spo2_drop': _m(rule_id='roc.spo2_drop', active=True),
    }

    enabled = _build_enabled_rule_id_set(['single.spo2_lt_90'])
    active_state = {}

    fired1 = _select_edge_fired_matches(
        pid=pid,
        matches=matches,
        enabled_rule_ids=enabled,
        active_state=active_state,
    )
    assert [m.rule_id for m in fired1] == ['single.spo2_lt_90']

    fired2 = _select_edge_fired_matches(
        pid=pid,
        matches=matches,
        enabled_rule_ids=enabled,
        active_state=active_state,
    )
    assert fired2 == []


def test_enabled_rule_ids_allows_all_when_none():
    pid = 'patient_01'
    matches = {
        'single.spo2_lt_90': _m(rule_id='single.spo2_lt_90', active=True),
        'roc.spo2_drop': _m(rule_id='roc.spo2_drop', active=True),
    }

    active_state = {}

    fired = _select_edge_fired_matches(
        pid=pid,
        matches=matches,
        enabled_rule_ids=None,
        active_state=active_state,
    )

    assert {m.rule_id for m in fired} == {'single.spo2_lt_90', 'roc.spo2_drop'}
