#!/usr/bin/env python3
"""Lifecycle 対応のルールベースアラートエンジン.

- 対象 topics は classic 版と同一:
  - subscribe: /<patient>/patient_vitals (default)
  - publish:   /<patient>/alerts (default)

Lifecycle state:
- unconfigured: pub/sub を作らない
- inactive:     内部状態を初期化（ルールパラメータ読み込み等）するが publish しない
- active:       pub/sub を作成し、alerts を publish する
- deactivate:   pub/sub を破棄し、publish を止める

注意:
- Day7/Day8 の parameter 群は classic 版と同様に受け取れるようにする。
- clean shutdown（Ctrl+C）でスタックトレースを出さない。
"""

from __future__ import annotations

import math
import signal
import sys
import threading
import traceback
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
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
from medical_robot_sim.qos_profiles import build_qos_profile
from medical_robot_sim.rule_alert_engine import format_alert_emit_event


def _now_s(node: LifecycleNode) -> float:
    return float(node.get_clock().now().nanoseconds) / 1e9


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
    """Edge trigger: False->True の瞬間だけ True を返す."""

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
    """temporal_stability 種別向け: active=True かつ cooldown 経過していれば返す."""

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


class RuleAlertEngineLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('rule_alert_engine')

        # Lifecycle 起動互換
        self.declare_parameter('lifecycle_autostart', True)

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

        if rules_path:
            self.get_logger().info(f'[Day7] rules_path={rules_path!r}')
        else:
            self.get_logger().info('[Day7] rules_path is empty; using code defaults')

        yaml_cfg: dict = {}
        if rules_path:
            try:
                yaml_cfg = load_rule_config(rules_path)
                self.get_logger().info(f'[Day7] alert_rules.yaml loaded: {rules_path!r}')

                yaml_flatline_history = yaml_cfg.get('flatline_history_size', 'default')
                yaml_flatline_hr_eps = yaml_cfg.get('flatline_hr_epsilon', 'default')
                yaml_flatline_spo2_eps = yaml_cfg.get('flatline_spo2_epsilon', 'default')
                yaml_enabled_rule_ids = yaml_cfg.get('enabled_rule_ids', 'default')

                self.get_logger().info(
                    '[Day7] yaml_config values: '
                    f'flatline_history_size={yaml_flatline_history}, '
                    f'flatline_hr_epsilon={yaml_flatline_hr_eps}, '
                    f'flatline_spo2_epsilon={yaml_flatline_spo2_eps}, '
                    f'enabled_rule_ids={yaml_enabled_rule_ids}'
                )
            except RuleConfigLoadError as exc:
                self.get_logger().error(
                    f'[Day7] rules_path load failed: {exc}\n'
                    'Falling back to code defaults.'
                )
                yaml_cfg = {}
            except Exception as exc:
                self.get_logger().error(
                    f'[Day7] rules_path load exception: {exc!r}\n'
                    'Falling back to code defaults.'
                )
                yaml_cfg = {}
        else:
            self.get_logger().info('[Day7] rules_path not provided; using code defaults')

        # --- Step2: 残りのパラメータを宣言する ---
        # 優先順位: ROS param (launch arg) > YAML > コード内デフォルト
        self.declare_parameter('patients', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('vitals_topic', 'patient_vitals')
        self.declare_parameter('alert_topic', 'alerts')

        # Day8: QoS params
        self.declare_parameter('vitals_qos_depth', 10)
        self.declare_parameter('vitals_qos_reliability', 'reliable')
        self.declare_parameter('vitals_qos_durability', 'volatile')

        self.declare_parameter('alerts_qos_depth', 10)
        self.declare_parameter('alerts_qos_reliability', 'reliable')
        self.declare_parameter('alerts_qos_durability', 'volatile')

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
        # 空リストの場合、rclpy の型推論が曖昧になり BYTE_ARRAY 扱いになることがあるため、
        # 明示的に STRING_ARRAY として declare しておく（launch からの上書きを可能にする）。
        yaml_rule_ids = get_string_list(yaml_cfg, 'enabled_rule_ids', [])
        if yaml_rule_ids:
            self.declare_parameter('enabled_rule_ids', list(yaml_rule_ids))
        else:
            self.declare_parameter('enabled_rule_ids', Parameter.Type.STRING_ARRAY)
            # 型が確定した後で、空の STRING_ARRAY として初期化する
            self.set_parameters(
                [
                    Parameter(
                        'enabled_rule_ids',
                        type_=Parameter.Type.STRING_ARRAY,
                        value=[],
                    )
                ]
            )

        # runtime state
        self._configured = False
        self._active_state: bool = False

        self._patient_ids: List[str] = []
        self._vitals_topic: str = ''
        self._alert_topic: str = ''

        self._vitals_qos = None
        self._alerts_qos = None

        self._history_size: int = 10
        self._spo2_drop_threshold: float = 4.0
        self._hr_jump_threshold: float = 20.0
        self._flatline_history_size: int = 8
        self._flatline_hr_epsilon: float = 1.0
        self._flatline_spo2_epsilon: float = 1.0
        self._enabled_rule_ids: Optional[set[str]] = None

        self._engine_subscriptions = []
        self._engine_pubs: Dict[str, rclpy.publisher.Publisher] = {}

        self._history: Dict[str, Deque[Sample]] = {}
        self._active: Dict[Tuple[str, str], bool] = {}
        self._last_emit_ts: Dict[Tuple[str, str], float] = {}

    def _apply_parameters(self) -> None:
        patient_ids = list(
            self.get_parameter('patients').get_parameter_value().string_array_value
        )
        vitals_topic = str(self.get_parameter('vitals_topic').value)
        alert_topic = str(self.get_parameter('alert_topic').value)

        vitals_qos_depth = int(self.get_parameter('vitals_qos_depth').value)
        vitals_qos_reliability = str(self.get_parameter('vitals_qos_reliability').value)
        vitals_qos_durability = str(self.get_parameter('vitals_qos_durability').value)

        alerts_qos_depth = int(self.get_parameter('alerts_qos_depth').value)
        alerts_qos_reliability = str(self.get_parameter('alerts_qos_reliability').value)
        alerts_qos_durability = str(self.get_parameter('alerts_qos_durability').value)

        try:
            vitals_qos = build_qos_profile(
                depth=vitals_qos_depth,
                reliability=vitals_qos_reliability,
                durability=vitals_qos_durability,
            )
            alerts_qos = build_qos_profile(
                depth=alerts_qos_depth,
                reliability=alerts_qos_reliability,
                durability=alerts_qos_durability,
            )
        except ValueError as exc:
            self.get_logger().error(f"[Day8] Invalid QoS params: {exc}")
            raise SystemExit(2)

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

        self._vitals_qos = vitals_qos
        self._alerts_qos = alerts_qos

        self._history_size = history_size
        self._spo2_drop_threshold = spo2_drop_threshold
        self._hr_jump_threshold = hr_jump_threshold
        self._flatline_history_size = flatline_history_size
        self._flatline_hr_epsilon = flatline_hr_epsilon
        self._flatline_spo2_epsilon = flatline_spo2_epsilon
        self._enabled_rule_ids = _build_enabled_rule_id_set(enabled_rule_ids_raw)

        self.get_logger().info('[Lifecycle] configured parameters applied')
        self.get_logger().info(f'patients={patient_ids}')
        self.get_logger().info(f"vitals_topic='{vitals_topic}'")
        self.get_logger().info(f"alert_topic='{alert_topic}'")
        self.get_logger().info(
            '[Day8] vitals_qos='
            f"KEEP_LAST depth={vitals_qos_depth}, reliability={vitals_qos_reliability}, "
            f"durability={vitals_qos_durability}"
        )
        self.get_logger().info(
            '[Day8] alerts_qos='
            f"KEEP_LAST depth={alerts_qos_depth}, reliability={alerts_qos_reliability}, "
            f"durability={alerts_qos_durability}"
        )

    def _destroy_io(self) -> None:
        for sub in list(self._engine_subscriptions):
            try:
                self.destroy_subscription(sub)
            except (ValueError, RCLError):
                pass
        self._engine_subscriptions = []

        for pub in list(self._engine_pubs.values()):
            try:
                self.destroy_publisher(pub)
            except Exception:
                pass
        self._engine_pubs = {}

    def on_configure(self, _state) -> TransitionCallbackReturn:
        try:
            self._apply_parameters()

            self._history = {
                pid: deque(maxlen=int(self._history_size)) for pid in list(self._patient_ids)
            }
            self._active = {}
            self._last_emit_ts = {}

            self._configured = True
            self._active_state = False
            self.get_logger().info('[Lifecycle] on_configure -> SUCCESS')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            try:
                self.get_logger().error(f'[Lifecycle] on_configure failed: {exc!r}')
            except Exception:
                pass
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, _state) -> TransitionCallbackReturn:
        if not self._configured:
            return TransitionCallbackReturn.FAILURE

        try:
            self._destroy_io()

            vitals_qos = self._vitals_qos
            alerts_qos = self._alerts_qos
            if vitals_qos is None or alerts_qos is None:
                return TransitionCallbackReturn.FAILURE

            for pid in list(self._patient_ids):
                vitals_abs = f'/{pid}/{self._vitals_topic}'
                alerts_abs = f'/{pid}/{self._alert_topic}'

                sub = self.create_subscription(
                    VitalSigns,
                    vitals_abs,
                    lambda msg, pid=pid: self._on_vitals(pid, msg),
                    vitals_qos,
                )
                self._engine_subscriptions.append(sub)

                self._engine_pubs[pid] = self.create_publisher(Alert, alerts_abs, alerts_qos)

            self._active_state = True
            self.get_logger().info('[Lifecycle] on_activate -> SUCCESS')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            try:
                self.get_logger().error(f'[Lifecycle] on_activate failed: {exc!r}')
            except Exception:
                pass
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        try:
            self._destroy_io()

            # 再現性を上げるため、deactivate 時に内部状態をクリア
            self._active_state = False
            self._history = {
                pid: deque(maxlen=int(self._history_size)) for pid in list(self._patient_ids)
            }
            self._active = {}
            self._last_emit_ts = {}

            self.get_logger().info('[Lifecycle] on_deactivate -> SUCCESS')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            try:
                self.get_logger().error(f'[Lifecycle] on_deactivate failed: {exc!r}')
            except Exception:
                pass
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        try:
            self._destroy_io()
            self._configured = False
            self._active_state = False
            self._history = {}
            self._active = {}
            self._last_emit_ts = {}
            self.get_logger().info('[Lifecycle] on_cleanup -> SUCCESS')
            return TransitionCallbackReturn.SUCCESS
        except Exception:
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, _state) -> TransitionCallbackReturn:
        try:
            self._destroy_io()
            self.get_logger().info('[Lifecycle] on_shutdown -> SUCCESS')
            return TransitionCallbackReturn.SUCCESS
        except Exception:
            return TransitionCallbackReturn.FAILURE

    def _emit(self, pid: str, alert_msg: Alert, cooldown_sec: float) -> None:
        if not self._active_state:
            return

        key = (pid, alert_msg.rule_id)
        now_s = float(alert_msg.ts)

        last = self._last_emit_ts.get(key)
        if last is not None and (now_s - float(last)) < float(cooldown_sec):
            self.get_logger().debug(
                f'[{pid}] emit skipped (cooldown): rule_id={alert_msg.rule_id} '
                f'elapsed={now_s - float(last):.1f}s < cooldown={cooldown_sec:.0f}s'
            )
            return

        pub = self._engine_pubs.get(pid)
        if pub is None:
            return
        pub.publish(alert_msg)

        try:
            self.get_logger().info(
                format_alert_emit_event(
                    alert_msg,
                    node=str(self.get_name()),
                    ns=str(self.get_namespace()),
                )
            )
        except Exception:
            pass

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

    def _on_vitals(self, pid: str, msg: VitalSigns) -> None:
        if not self._active_state:
            return

        ts = _now_s(self)

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

        edge_fired = _select_edge_fired_matches(
            pid=str(pid),
            matches=matches,
            enabled_rule_ids=self._enabled_rule_ids,
            active_state=self._active,
        )

        level_fired = _select_level_fired_matches(
            pid=str(pid),
            matches=matches,
            enabled_rule_ids=self._enabled_rule_ids,
            last_emit_ts=self._last_emit_ts,
            now_s=ts,
            cooldown_sec=_TEMPORAL_STABILITY_COOLDOWN_SEC,
        )

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

    node: Optional[RuleAlertEngineLifecycleNode] = None
    executor = None

    try:
        node = RuleAlertEngineLifecycleNode()

        from rclpy.executors import ExternalShutdownException
        from rclpy.executors import SingleThreadedExecutor

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # lifecycle_autostart=true の場合は configure->activate を自動で行う
        # false の場合は unconfigured のまま待機し、外部から遷移させる
        try:
            autostart = bool(node.get_parameter('lifecycle_autostart').value)
        except Exception:
            autostart = True
        if autostart:
            try:
                node.trigger_configure()
                node.trigger_activate()
            except Exception as exc:
                try:
                    node.get_logger().error(f'[Lifecycle] autostart failed: {exc!r}')
                except Exception:
                    pass

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
                    node.destroy_node()
                except Exception:
                    pass
    except Exception as exc:
        err = traceback.format_exc()
        try:
            rclpy.logging.get_logger('rule_alert_engine').error(
                f'Unhandled exception during startup: {exc!r}\n{err}'
            )
        except Exception:
            pass
        try:
            sys.stderr.write(err + '\n')
        except Exception:
            pass
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
