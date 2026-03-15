"""Day7: rule_config_loader のユニットテスト.

テスト対象:
  - load_rule_config(path) の正常系・異常系
  - get_float / get_int / get_string_list ヘルパー
  - yaml 未記載キーはスキップされる（後方互換）
"""

from __future__ import annotations

import os
import textwrap
import tempfile
import pytest

from medical_robot_sim.rule_config_loader import (
    load_rule_config,
    RuleConfigLoadError,
    get_float,
    get_int,
    get_string_list,
    resolve_rules_path,
)


# ---------------------------------------------------------------------------
# テスト用 YAML ファイルを tmp に作成するヘルパー
# ---------------------------------------------------------------------------

def _write_yaml(content: str) -> str:
    """一時ファイルに YAML を書き込み、そのパスを返す."""
    tf = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, encoding='utf-8'
    )
    tf.write(textwrap.dedent(content))
    tf.flush()
    tf.close()
    return tf.name


# ---------------------------------------------------------------------------
# 正常系
# ---------------------------------------------------------------------------

def test_load_full_config():
    """全キーを含む YAML を正しく読み込む."""
    path = _write_yaml("""
        rule_config:
          spo2_drop_threshold: 5.0
          hr_jump_threshold: 25.0
          flatline_history_size: 12
          flatline_hr_epsilon: 0.5
          flatline_spo2_epsilon: 0.3
          enabled_rule_ids:
            - flatline.hr
            - flatline.spo2
    """)
    try:
        cfg = load_rule_config(path)
        assert cfg['spo2_drop_threshold'] == pytest.approx(5.0)
        assert cfg['hr_jump_threshold'] == pytest.approx(25.0)
        assert cfg['flatline_history_size'] == 12
        assert cfg['flatline_hr_epsilon'] == pytest.approx(0.5)
        assert cfg['flatline_spo2_epsilon'] == pytest.approx(0.3)
        assert cfg['enabled_rule_ids'] == ['flatline.hr', 'flatline.spo2']
    finally:
        os.unlink(path)


def test_load_partial_config_missing_keys_are_absent():
    """一部キーだけ記載した YAML を読み込む（未記載キーは dict に存在しない）."""
    path = _write_yaml("""
        rule_config:
          spo2_drop_threshold: 3.5
    """)
    try:
        cfg = load_rule_config(path)
        assert cfg['spo2_drop_threshold'] == pytest.approx(3.5)
        assert 'hr_jump_threshold' not in cfg
        assert 'flatline_history_size' not in cfg
        assert 'enabled_rule_ids' not in cfg
    finally:
        os.unlink(path)


def test_load_empty_rule_config_section():
    """rule_config キーはあるが空の場合、空 dict を返す."""
    path = _write_yaml("""
        rule_config:
    """)
    try:
        cfg = load_rule_config(path)
        assert cfg == {}
    finally:
        os.unlink(path)


def test_load_no_rule_config_key():
    """rule_config キーが存在しない場合、空 dict を返す（上位キーが別名でも問題なし）."""
    path = _write_yaml("""
        other_section:
          foo: bar
    """)
    try:
        cfg = load_rule_config(path)
        assert cfg == {}
    finally:
        os.unlink(path)


def test_load_empty_enabled_rule_ids():
    """enabled_rule_ids が空リストの場合、空リストを返す."""
    path = _write_yaml("""
        rule_config:
          enabled_rule_ids: []
    """)
    try:
        cfg = load_rule_config(path)
        assert cfg.get('enabled_rule_ids') == []
    finally:
        os.unlink(path)


def test_load_enabled_rule_ids_whitespace_stripped():
    """enabled_rule_ids の空白要素が除去されること."""
    path = _write_yaml("""
        rule_config:
          enabled_rule_ids:
            - "  flatline.hr  "
            - ""
            - "roc.spo2_drop"
    """)
    try:
        cfg = load_rule_config(path)
        assert cfg['enabled_rule_ids'] == ['flatline.hr', 'roc.spo2_drop']
    finally:
        os.unlink(path)


def test_load_actual_config_file():
    """リポジトリに含まれる config/alert_rules.yaml を実際に読み込めること."""
    # このテストファイルの位置から config/ を探す
    here = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(here, '..', 'config', 'alert_rules.yaml')
    config_path = os.path.normpath(config_path)

    if not os.path.isfile(config_path):
        pytest.skip(f'config/alert_rules.yaml が見つかりません: {config_path}')

    cfg = load_rule_config(config_path)
    # デフォルト値が正しく設定されていることを確認
    assert get_float(cfg, 'spo2_drop_threshold', 4.0) == pytest.approx(4.0)
    assert get_float(cfg, 'hr_jump_threshold', 20.0) == pytest.approx(20.0)
    assert get_int(cfg, 'flatline_history_size', 8) == 8


# ---------------------------------------------------------------------------
# 異常系
# ---------------------------------------------------------------------------

def test_load_file_not_found():
    """存在しないパスを指定すると RuleConfigLoadError が上がる."""
    with pytest.raises(RuleConfigLoadError, match='見つかりません'):
        load_rule_config('/nonexistent/path/alert_rules.yaml')


def test_load_invalid_yaml():
    """不正な YAML を指定すると RuleConfigLoadError が上がる."""
    path = _write_yaml(":\t:invalid yaml: [\n")
    try:
        with pytest.raises(RuleConfigLoadError, match='パースエラー'):
            load_rule_config(path)
    finally:
        os.unlink(path)


def test_load_top_level_not_dict():
    """トップレベルが dict でない YAML は RuleConfigLoadError."""
    path = _write_yaml("- item1\n- item2\n")
    try:
        with pytest.raises(RuleConfigLoadError, match='dict'):
            load_rule_config(path)
    finally:
        os.unlink(path)


def test_load_rule_config_not_dict():
    """rule_config の値が dict でない場合は RuleConfigLoadError."""
    path = _write_yaml("rule_config: some_string\n")
    try:
        with pytest.raises(RuleConfigLoadError, match='dict'):
            load_rule_config(path)
    finally:
        os.unlink(path)


def test_load_invalid_float_value():
    """float に変換できない値は RuleConfigLoadError."""
    path = _write_yaml("""
        rule_config:
          spo2_drop_threshold: "not_a_float"
    """)
    try:
        with pytest.raises(RuleConfigLoadError, match='float'):
            load_rule_config(path)
    finally:
        os.unlink(path)


def test_load_invalid_int_value():
    """int に変換できない値は RuleConfigLoadError."""
    path = _write_yaml("""
        rule_config:
          flatline_history_size: "bad"
    """)
    try:
        with pytest.raises(RuleConfigLoadError, match='int'):
            load_rule_config(path)
    finally:
        os.unlink(path)


def test_load_enabled_rule_ids_not_list():
    """enabled_rule_ids がリストでない場合は RuleConfigLoadError."""
    path = _write_yaml("""
        rule_config:
          enabled_rule_ids: "flatline.hr"
    """)
    try:
        with pytest.raises(RuleConfigLoadError, match='リスト'):
            load_rule_config(path)
    finally:
        os.unlink(path)


# ---------------------------------------------------------------------------
# ヘルパー関数
# ---------------------------------------------------------------------------

def test_get_float_existing_key():
    cfg = {'spo2_drop_threshold': 3.5}
    assert get_float(cfg, 'spo2_drop_threshold', 4.0) == pytest.approx(3.5)


def test_get_float_missing_key_returns_default():
    cfg = {}
    assert get_float(cfg, 'spo2_drop_threshold', 4.0) == pytest.approx(4.0)


def test_get_int_existing_key():
    cfg = {'flatline_history_size': 12}
    assert get_int(cfg, 'flatline_history_size', 8) == 12


def test_get_int_missing_key_returns_default():
    cfg = {}
    assert get_int(cfg, 'flatline_history_size', 8) == 8


def test_get_string_list_existing_key():
    cfg = {'enabled_rule_ids': ['flatline.hr', 'flatline.spo2']}
    expected = ['flatline.hr', 'flatline.spo2']
    assert get_string_list(cfg, 'enabled_rule_ids') == expected


def test_get_string_list_missing_key_returns_default():
    cfg = {}
    assert get_string_list(cfg, 'enabled_rule_ids', []) == []


def test_get_string_list_missing_key_custom_default():
    cfg = {}
    assert get_string_list(cfg, 'enabled_rule_ids', ['a', 'b']) == ['a', 'b']


def test_resolve_rules_path_absolute(tmp_path):
    path = tmp_path / 'alert_rules.yaml'
    path.write_text('rule_config: {}', encoding='utf-8')
    resolved = resolve_rules_path(str(path), base_dir=str(tmp_path))
    assert resolved == os.path.normpath(str(path))


def test_resolve_rules_path_relative_with_base_dir(tmp_path):
    base = tmp_path / 'share' / 'medical_robot_sim'
    base.mkdir(parents=True)
    rel_path = os.path.join('config', 'alert_rules.yaml')
    target = base / rel_path
    target.parent.mkdir(parents=True)
    target.write_text('rule_config: {}', encoding='utf-8')

    resolved = resolve_rules_path(rel_path, base_dir=str(base))
    assert resolved == os.path.normpath(str(target))
