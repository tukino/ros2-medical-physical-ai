"""Day7: ルール設定 YAML ローダー（純粋関数）.

- load_rule_config(path) が YAML から設定を読み込み dict で返す
- ノード（rule_alert_engine）はこの dict を使って declare_parameter のデフォルト値を設定する

設計方針:
  - 純粋関数のみ（ROS / rclpy 非依存）
  - 未記載キーはすべて省略可能（呼び出し側がデフォルトを持つ）
  - YAML の top-level キー 'rule_config' を使用する
  - ROS param > YAML > コード内デフォルト の優先順位は rule_alert_engine 側で保証する
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional


# YAML 解析に yaml (PyYAML) を使用。
# ROS 2 Humble 環境では python3-yaml が標準で利用可能。
try:
    import yaml
    _YAML_AVAILABLE = True
except ImportError:  # pragma: no cover
    _YAML_AVAILABLE = False


class RuleConfigLoadError(Exception):
    """YAML ロード時の異常（ファイル不存在・パース失敗など）."""


def load_rule_config(path: str) -> Dict[str, Any]:
    """YAML ファイルを読み込み rule_config セクションを dict で返す.

    Args:
        path: YAML ファイルの絶対パスまたは相対パス

    Returns:
        rule_config セクションの内容を表す dict。
        以下のキーが含まれる場合がある（すべて省略可能）:
            spo2_drop_threshold   (float)
            hr_jump_threshold     (float)
            flatline_history_size (int)
            flatline_hr_epsilon   (float)
            flatline_spo2_epsilon (float)
            enabled_rule_ids      (list[str])

    Raises:
        RuleConfigLoadError: ファイルが見つからない / パース失敗 / 形式不正
    """

    if not _YAML_AVAILABLE:
        raise RuleConfigLoadError(  # pragma: no cover
            "PyYAML が見つかりません。'pip install pyyaml' または "
            "'sudo apt install python3-yaml' でインストールしてください。"
        )

    expanded_path = os.path.expandvars(os.path.expanduser(str(path)))

    if not os.path.isfile(expanded_path):
        raise RuleConfigLoadError(
            f"rules_path で指定されたファイルが見つかりません: {expanded_path!r}"
        )

    try:
        with open(expanded_path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        raise RuleConfigLoadError(
            f"YAML パースエラー ({expanded_path!r}): {exc}"
        ) from exc

    if raw is None:
        return {}

    if not isinstance(raw, dict):
        raise RuleConfigLoadError(
            f"YAML のトップレベルは dict である必要があります: {expanded_path!r}"
        )

    # rule_config セクションを取得（存在しない場合は空 dict）
    section = raw.get('rule_config', {})
    if section is None:
        section = {}
    if not isinstance(section, dict):
        raise RuleConfigLoadError(
            f"'rule_config' キーの値は dict である必要があります: {expanded_path!r}"
        )

    return _validate_and_coerce(section, source=expanded_path)


def resolve_rules_path(path: str, *, base_dir: Optional[str] = None) -> str:
    """rules_path を解決する.

    - 絶対パスはそのまま返す
    - 相対パスは base_dir があれば base_dir からのパスを優先
    - base_dir から見つからなければ、相対パス（作業ディレクトリ基準）を返す
    """

    if path is None:
        return ''

    expanded = os.path.expandvars(os.path.expanduser(str(path))).strip()
    if not expanded:
        return ''

    if os.path.isabs(expanded):
        return os.path.normpath(expanded)

    if base_dir:
        base_dir_expanded = os.path.expandvars(os.path.expanduser(str(base_dir))).strip()
        if base_dir_expanded:
            candidate = os.path.join(base_dir_expanded, expanded)
            if os.path.isfile(candidate):
                return os.path.normpath(candidate)

    return os.path.normpath(expanded)


def _validate_and_coerce(
    cfg: Dict[str, Any],
    *,
    source: str,
) -> Dict[str, Any]:
    """型バリデーションと強制変換を行い、正規化された dict を返す."""

    result: Dict[str, Any] = {}

    _float_keys = ('spo2_drop_threshold', 'hr_jump_threshold',
                   'flatline_hr_epsilon', 'flatline_spo2_epsilon')
    _int_keys = ('flatline_history_size',)

    for key in _float_keys:
        val = cfg.get(key)
        if val is None:
            continue
        try:
            result[key] = float(val)
        except (TypeError, ValueError) as exc:
            raise RuleConfigLoadError(
                f"'{key}' の値を float に変換できません ({source!r}): {val!r}"
            ) from exc

    for key in _int_keys:
        val = cfg.get(key)
        if val is None:
            continue
        try:
            result[key] = int(val)
        except (TypeError, ValueError) as exc:
            raise RuleConfigLoadError(
                f"'{key}' の値を int に変換できません ({source!r}): {val!r}"
            ) from exc

    # enabled_rule_ids: list[str] or None
    raw_ids = cfg.get('enabled_rule_ids')
    if raw_ids is not None:
        if not isinstance(raw_ids, list):
            raise RuleConfigLoadError(
                f"'enabled_rule_ids' はリストである必要があります ({source!r}): {raw_ids!r}"
            )
        result['enabled_rule_ids'] = [
            str(x).strip() for x in raw_ids if str(x).strip()
        ]

    return result


def get_float(cfg: Dict[str, Any], key: str, default: float) -> float:
    """cfg から float 値を取得する。存在しない場合は default を返す."""
    val = cfg.get(key)
    if val is None:
        return default
    return float(val)


def get_int(cfg: Dict[str, Any], key: str, default: int) -> int:
    """cfg から int 値を取得する。存在しない場合は default を返す."""
    val = cfg.get(key)
    if val is None:
        return default
    return int(val)


def get_string_list(
    cfg: Dict[str, Any],
    key: str,
    default: Optional[List[str]] = None,
) -> List[str]:
    """cfg から list[str] を取得する。存在しない場合は default を返す."""
    if default is None:
        default = []
    val = cfg.get(key)
    if val is None:
        return list(default)
    return [str(x) for x in val]
