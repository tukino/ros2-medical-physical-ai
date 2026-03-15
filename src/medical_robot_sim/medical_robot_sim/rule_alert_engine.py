#!/usr/bin/env python3
"""ルールベースのアラートエンジン.

- 患者ごとに `/{pid}/{vitals_topic}` を subscribe（デフォルト: patient_vitals）
- 患者ごとの履歴（deque）を用いてルール判定
- `medical_interfaces/msg/Alert` を `/{pid}/{alert_topic}` に publish（デフォルト: alerts）
- (patient_id, rule_id) ごとの cooldown（通知抑制）を実装

このノードはサンプル用途のため、ルールセットを小さく明示的に保つ.
"""

from __future__ import annotations

import math
import signal
import sys
import threading
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.parameter import Parameter

from medical_interfaces.msg import Alert
from medical_interfaces.msg import VitalSigns

from medical_robot_sim.alert_rules import RuleMatch
from medical_robot_sim.alert_rules import RuleParams
from medical_robot_sim.alert_rules import Sample
from medical_robot_sim.alert_rules import evaluate_alert_rules
from medical_robot_sim.rule_config_loader import RuleConfigLoadError
from medical_robot_sim.rule_config_loader import get_float
from medical_robot_sim.rule_config_loader import get_int
from medical_robot_sim.rule_config_loader import get_string_list
from medical_robot_sim.rule_config_loader import load_rule_config
from medical_robot_sim.rule_config_loader import resolve_rules_path


def _now_s(node: Node) -> float:
    return float(node.get_clock().now().nanoseconds) / 1e9


def _nan() -> float:
    return float('nan')


def _cooldown_for_priority(priority: str) -> float:
    # Day5 のデフォルト（秒）
    if priority == 'RED':
        return 5.0
    if priority == 'ORANGE':
        return 10.0
    if priority == 'YELLOW':
        return 30.0
    if priority == 'INFO':
        return 60.0
    return 30.0


# temporal_stability（flatline）は継続中も定期再通知するためのクールダウン（秒）
# edge trigger ではなく level trigger + cooldown を使う
_TEMPORAL_STABILITY_COOLDOWN_SEC = 10.0


def _build_enabled_rule_id_set(rule_ids: List[str]) -> Optional[set[str]]:
    """enabled_rule_ids parameter value -> set.

    - 空/未指定（空リスト相当）なら None を返し「全ルール有効」とみなす。
    - 空文字や前後空白は除去。
    """

    cleaned = {str(x).strip() for x in (rule_ids or []) if str(x).strip()}
    if not cleaned:
        return None
    return cleaned


def _is_rule_enabled(rule_id: str, enabled_rule_ids: Optional[set[str]]) -> bool:
    if enabled_rule_ids is None:
        return True
    return str(rule_id) in enabled_rule_ids


def _edge_fire(
    active_state: Dict[Tuple[str, str], bool],
    *,
    pid: str,
    rule_id: str,
    is_active: bool,
) -> bool:
    """Edge trigger: False->True の瞬間だけ True を返す.

    enabled_rule_ids フィルタの外側で呼ぶことを想定。
    """

    key = (str(pid), str(rule_id))
    was_active = bool(active_state.get(key, False))
    active_state[key] = bool(is_active)
    return bool(is_active) and not was_active


def _select_edge_fired_matches(
    *,
    pid: str,
    matches: Dict[str, RuleMatch],
    enabled_rule_ids: Optional[set[str]],
    active_state: Dict[Tuple[str, str], bool],
) -> List[RuleMatch]:
    """enabled_rule_ids と edge-trigger を加味して publish 対象だけ返す.

    temporal_stability 種別は除外する（level trigger 経路で処理する）。
    """

    fired: List[RuleMatch] = []
    for _key, match in matches.items():
        # temporal_stability は別経路（level trigger）で処理するのでスキップ
        if str(match.kind) == 'temporal_stability':
            continue
        rid = str(match.rule_id)
        if not _is_rule_enabled(rid, enabled_rule_ids):
            continue
        if _edge_fire(
            active_state,
            pid=str(pid),
            rule_id=rid,
            is_active=bool(match.active),
        ):
            fired.append(match)
    return fired


def _select_level_fired_matches(
    *,
    pid: str,
    matches: Dict[str, RuleMatch],
    enabled_rule_ids: Optional[set[str]],
    last_emit_ts: Dict[Tuple[str, str], float],
    now_s: float,
    cooldown_sec: float,
) -> List[RuleMatch]:
    """temporal_stability 種別向け: active=True かつ cooldown 経過していれば返す.

    エッジトリガは使わず、cooldown が経過するたびに再発火する（level trigger）。
    """

    fired: List[RuleMatch] = []
    for _key, match in matches.items():
        if str(match.kind) != 'temporal_stability':
            continue
        rid = str(match.rule_id)
        if not _is_rule_enabled(rid, enabled_rule_ids):
            continue
        if not bool(match.active):
            continue
        key = (str(pid), rid)
        last = last_emit_ts.get(key)
        if last is not None and (float(now_s) - float(last)) < float(cooldown_sec):
            continue
        fired.append(match)
    return fired


class RuleAlertEngineNode(Node):
    def __init__(self):
        super().__init__('rule_alert_engine')

        # --- Step1: rules_path を先に宣言・取得して YAML を読み込む ---
        self.declare_parameter('rules_path', '')
        rules_path = str(self.get_parameter('rules_path').value).strip()

        if rules_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                package_share = get_package_share_directory('medical_robot_sim')
            except Exception:
                package_share = None

            resolved_path = resolve_rules_path(rules_path, base_dir=package_share)
            if resolved_path != rules_path:
                self.get_logger().info(
                    f'[Day7] rules_path resolved: {rules_path!r} -> {resolved_path!r}'
                )
            rules_path = resolved_path

        yaml_cfg: dict = {}
        if rules_path:
            try:
                yaml_cfg = load_rule_config(rules_path)
                self.get_logger().info(f'[Day7] alert_rules.yaml を読み込みました: {rules_path!r}')
            except RuleConfigLoadError as exc:
                self.get_logger().error(
                    f'[Day7] rules_path の読み込みに失敗しました: {exc}\n'
                    'コード内デフォルト値で続行します。'
                )
                yaml_cfg = {}
            except Exception as exc:
                self.get_logger().error(
                    f'[Day7] rules_path の読み込み中に例外が発生しました: {exc!r}\n'
                    'コード内デフォルト値で続行します。'
                )
                yaml_cfg = {}
        else:
            self.get_logger().info('[Day7] rules_path 未指定: コード内デフォルト値を使用します')

        # --- Step2: 残りのパラメータを宣言する ---
        # 優先順位: ROS param (launch arg) > YAML > コード内デフォルト
        # declare_parameter の default_value が YAML 由来になるため、
        # launch arg で明示指定された場合は自動的に launch arg 値が勝つ。

        self.declare_parameter('patients', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('vitals_topic', 'patient_vitals')
        self.declare_parameter('alert_topic', 'alerts')

        # 患者ごとに直近 N サンプルを保持
        self.declare_parameter('history_size', 10)

        # 変化率（rate-of-change）ルールのしきい値
        self.declare_parameter(
            'spo2_drop_threshold', get_float(yaml_cfg, 'spo2_drop_threshold', 4.0)
        )
        self.declare_parameter(
            'hr_jump_threshold', get_float(yaml_cfg, 'hr_jump_threshold', 20.0)
        )

        # flatline（時間的安定性）ルールのパラメータ
        self.declare_parameter(
            'flatline_history_size', get_int(yaml_cfg, 'flatline_history_size', 8)
        )
        self.declare_parameter(
            'flatline_hr_epsilon', get_float(yaml_cfg, 'flatline_hr_epsilon', 1.0)
        )
        self.declare_parameter(
            'flatline_spo2_epsilon', get_float(yaml_cfg, 'flatline_spo2_epsilon', 1.0)
        )

        # publish 対象ルール（空/未指定なら全て有効）
        # YAML に enabled_rule_ids が記載されていればそれをデフォルトとして使う
        yaml_rule_ids = get_string_list(yaml_cfg, 'enabled_rule_ids', [])
        if yaml_rule_ids:
            self.declare_parameter('enabled_rule_ids', yaml_rule_ids)
        else:
            self.declare_parameter('enabled_rule_ids', Parameter.Type.STRING_ARRAY)

        patient_ids = list(
            self.get_parameter('patients').get_parameter_value().string_array_value
        )
        vitals_topic = str(
            self.get_parameter('vitals_topic').get_parameter_value().string_value
        )
        alert_topic = str(
            self.get_parameter('alert_topic').get_parameter_value().string_value
        )

        history_size = int(self.get_parameter('history_size').value)
        if history_size < 2:
            raise ValueError("Parameter 'history_size' must be >= 2")

        spo2_drop_threshold = float(self.get_parameter('spo2_drop_threshold').value)
        hr_jump_threshold = float(self.get_parameter('hr_jump_threshold').value)

        flatline_history_size = int(self.get_parameter('flatline_history_size').value)
        flatline_hr_epsilon = float(self.get_parameter('flatline_hr_epsilon').value)
        flatline_spo2_epsilon = float(self.get_parameter('flatline_spo2_epsilon').value)

        enabled_rule_ids_raw = list(
            self.get_parameter('enabled_rule_ids').get_parameter_value().string_array_value
        )

        patient_ids = [str(pid).strip().lstrip('/') for pid in patient_ids if str(pid).strip()]
        vitals_topic = vitals_topic.strip().lstrip('/')
        alert_topic = alert_topic.strip().lstrip('/')

        # 重複除去（順序維持）
        unique_patient_ids: List[str] = []
        seen = set()
        for pid in patient_ids:
            if pid not in seen:
                unique_patient_ids.append(pid)
                seen.add(pid)
        patient_ids = unique_patient_ids

        if not patient_ids:
            raise RuntimeError("Parameter 'patients' is empty")
        if not vitals_topic:
            raise RuntimeError("Parameter 'vitals_topic' is empty")
        if not alert_topic:
            raise RuntimeError("Parameter 'alert_topic' is empty")

        self._patient_ids = patient_ids
        self._vitals_topic = vitals_topic
        self._alert_topic = alert_topic

        self._history_size = history_size
        self._spo2_drop_threshold = spo2_drop_threshold
        self._hr_jump_threshold = hr_jump_threshold
        self._flatline_history_size = flatline_history_size
        self._flatline_hr_epsilon = flatline_hr_epsilon
        self._flatline_spo2_epsilon = flatline_spo2_epsilon
        self._enabled_rule_ids = _build_enabled_rule_id_set(enabled_rule_ids_raw)

        self._subscriptions = []
        self._pubs: Dict[str, rclpy.publisher.Publisher] = {}

        self._history: Dict[str, Deque[Sample]] = {
            pid: deque(maxlen=self._history_size) for pid in patient_ids
        }

        # ルールごとの active 状態（エッジトリガ用）
        self._active: Dict[Tuple[str, str], bool] = {}

        # cooldown 用の時刻記録
        self._last_emit_ts: Dict[Tuple[str, str], float] = {}

        for pid in patient_ids:
            vitals_abs = f'/{pid}/{vitals_topic}'
            alerts_abs = f'/{pid}/{alert_topic}'

            sub = self.create_subscription(
                VitalSigns,
                vitals_abs,
                lambda msg, pid=pid: self._on_vitals(pid, msg),
                10,
            )
            self._subscriptions.append(sub)

            self._pubs[pid] = self.create_publisher(Alert, alerts_abs, 10)

        self.get_logger().info('rule_alert_engine を起動しました')
        if rules_path:
            self.get_logger().info(f'[Day7] rules_path={rules_path!r}')
        self.get_logger().info(f'patients={patient_ids}')
        self.get_logger().info(f"vitals_topic='{vitals_topic}'")
        self.get_logger().info(f"alert_topic='{alert_topic}'")
        self.get_logger().info(f'history_size={history_size}')
        self.get_logger().info(
            f'spo2_drop_threshold={spo2_drop_threshold}, hr_jump_threshold={hr_jump_threshold}'
        )
        self.get_logger().info(
            f'flatline_history_size={flatline_history_size}, '
            f'flatline_hr_epsilon={flatline_hr_epsilon}, '
            f'flatline_spo2_epsilon={flatline_spo2_epsilon}'
        )
        if self._enabled_rule_ids is None:
            self.get_logger().info('enabled_rule_ids=ALL')
        else:
            self.get_logger().info(f'enabled_rule_ids={sorted(self._enabled_rule_ids)}')

    def _emit(self, pid: str, alert_msg: Alert, cooldown_sec: float) -> None:
        key = (pid, alert_msg.rule_id)
        now_s = float(alert_msg.ts)

        last = self._last_emit_ts.get(key)
        if last is not None and (now_s - float(last)) < float(cooldown_sec):
            self.get_logger().debug(
                f'[{pid}] emit skipped (cooldown): rule_id={alert_msg.rule_id} '
                f'elapsed={now_s - float(last):.1f}s < cooldown={cooldown_sec:.0f}s'
            )
            return

        pub = self._pubs.get(pid)
        if pub is None:
            return
        pub.publish(alert_msg)
        self._last_emit_ts[key] = now_s

    def _build_alert(
        self,
        *,
        pid: str,
        rule_id: str,
        kind: str,
        priority: str,
        message: str,
        ts: float,
        window_sec: int,
        field: str = '',
        value: float = math.nan,
        delta: float = math.nan,
        score: float = math.nan,
    ) -> Alert:
        msg = Alert()
        msg.patient_id = str(pid)
        msg.rule_id = str(rule_id)
        msg.kind = str(kind)
        msg.priority = str(priority)
        msg.message = str(message)
        msg.ts = float(ts)
        msg.window_sec = int(window_sec)
        msg.field = str(field)
        msg.value = float(value)
        msg.delta = float(delta)
        msg.score = float(score)
        return msg

    def _edge_fire(self, pid: str, rule_id: str, is_active: bool) -> bool:
        return _edge_fire(
            self._active,
            pid=str(pid),
            rule_id=str(rule_id),
            is_active=bool(is_active),
        )

    def _on_vitals(self, pid: str, msg: VitalSigns) -> None:
        ts = _now_s(self)

        # 履歴を保持（deque）
        sample = Sample(
            ts=ts,
            heart_rate=float(msg.heart_rate),
            blood_pressure_systolic=float(msg.blood_pressure_systolic),
            blood_pressure_diastolic=float(msg.blood_pressure_diastolic),
            body_temperature=float(msg.body_temperature),
            oxygen_saturation=float(msg.oxygen_saturation),
        )
        hist = self._history.get(pid)
        if hist is None:
            hist = deque(maxlen=self._history_size)
            self._history[pid] = hist
        hist.append(sample)

        params = RuleParams(
            history_size=int(self._history_size),
            spo2_drop_threshold=float(self._spo2_drop_threshold),
            hr_jump_threshold=float(self._hr_jump_threshold),
            flatline_history_size=int(self._flatline_history_size),
            flatline_hr_epsilon=float(self._flatline_hr_epsilon),
            flatline_spo2_epsilon=float(self._flatline_spo2_epsilon),
        )
        matches = evaluate_alert_rules(list(hist), params)

        # デバッグ: flatline の active 状態をログに出す（DEBUG レベル）
        for fl_rule_id in ('flatline.hr', 'flatline.spo2'):
            m = matches.get(fl_rule_id)
            if m is not None and _is_rule_enabled(fl_rule_id, self._enabled_rule_ids):
                self.get_logger().debug(
                    f'[DBG][{pid}] {fl_rule_id} active={m.active} '
                    f'delta={m.delta} history_len={len(hist)}'
                )

        # edge trigger: single / roc / combination 系ルール
        edge_fired = _select_edge_fired_matches(
            pid=str(pid),
            matches=matches,
            enabled_rule_ids=self._enabled_rule_ids,
            active_state=self._active,
        )

        # level trigger: temporal_stability (flatline) 系ルール
        level_fired = _select_level_fired_matches(
            pid=str(pid),
            matches=matches,
            enabled_rule_ids=self._enabled_rule_ids,
            last_emit_ts=self._last_emit_ts,
            now_s=ts,
            cooldown_sec=_TEMPORAL_STABILITY_COOLDOWN_SEC,
        )

        # level_fired は cooldown 更新を _emit に任せるが、
        # _emit の cooldown_sec は 0 を渡して二重チェックを避ける
        for match in list(edge_fired) + list(level_fired):
            a = self._build_alert(
                pid=pid,
                rule_id=match.rule_id,
                kind=match.kind,
                priority=match.priority,
                message=match.message,
                ts=ts,
                window_sec=int(match.window_sec),
                field=match.field,
                value=match.value,
                delta=match.delta,
                score=match.score,
            )
            if match.kind == 'temporal_stability':
                # level trigger 経路: cooldown は _select_level_fired_matches で
                # 既にチェック済みなので、ここでは記録だけ行う（cooldown_sec=0 渡し）
                self._emit(pid, a, 0.0)
            else:
                self._emit(pid, a, _cooldown_for_priority(a.priority))


def main(args=None):
    shutdown_requested = threading.Event()

    def _request_shutdown(_signum=None, _frame=None):
        shutdown_requested.set()

    original_sigint_handler = signal.getsignal(signal.SIGINT)
    original_sigterm_handler = signal.getsignal(signal.SIGTERM)

    rclpy.init(args=args)

    # NOTE: signal handler では destroy_node() / rclpy.shutdown() を呼ばない
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    node: Optional[RuleAlertEngineNode] = None
    executor = None

    try:
        node = RuleAlertEngineNode()

        from rclpy.executors import ExternalShutdownException
        from rclpy.executors import SingleThreadedExecutor

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while rclpy.ok() and not shutdown_requested.is_set():
                executor.spin_once(timeout_sec=0.1)
            return 0
        except (KeyboardInterrupt, ExternalShutdownException):
            shutdown_requested.set()
            return 0
        except Exception as exc:
            try:
                node.get_logger().error(f'Unhandled exception: {exc!r}')
            except Exception:
                pass
            return 0
        finally:
            try:
                if executor is not None and node is not None:
                    executor.remove_node(node)
            except Exception:
                pass

            try:
                if executor is not None:
                    executor.shutdown()
            except Exception:
                pass

            if node is not None:
                try:
                    for sub in list(getattr(node, '_subscriptions', [])):
                        try:
                            node.destroy_subscription(sub)
                        except (ValueError, RCLError):
                            pass
                except Exception:
                    pass

                try:
                    node.destroy_node()
                except Exception:
                    pass
    except Exception:
        return 0
    finally:
        try:
            signal.signal(signal.SIGINT, original_sigint_handler)
            signal.signal(signal.SIGTERM, original_sigterm_handler)
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
