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

from medical_robot_sim.alert_rules import RuleParams
from medical_robot_sim.alert_rules import Sample
from medical_robot_sim.alert_rules import evaluate_alert_rules


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


class RuleAlertEngineNode(Node):
    def __init__(self):
        super().__init__('rule_alert_engine')

        self.declare_parameter('patients', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('vitals_topic', 'patient_vitals')
        self.declare_parameter('alert_topic', 'alerts')

        # 患者ごとに直近 N サンプルを保持
        self.declare_parameter('history_size', 10)

        # 変化率（rate-of-change）ルールのしきい値
        self.declare_parameter('spo2_drop_threshold', 4.0)
        self.declare_parameter('hr_jump_threshold', 20.0)

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
        self.get_logger().info(f'patients={patient_ids}')
        self.get_logger().info(f"vitals_topic='{vitals_topic}'")
        self.get_logger().info(f"alert_topic='{alert_topic}'")
        self.get_logger().info(f'history_size={history_size}')
        self.get_logger().info(
            f'spo2_drop_threshold={spo2_drop_threshold}, hr_jump_threshold={hr_jump_threshold}'
        )

    def _emit(self, pid: str, alert_msg: Alert, cooldown_sec: float) -> None:
        key = (pid, alert_msg.rule_id)
        now_s = float(alert_msg.ts)

        last = self._last_emit_ts.get(key)
        if last is not None and (now_s - float(last)) < float(cooldown_sec):
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
        key = (pid, rule_id)
        was_active = self._active.get(key, False)
        self._active[key] = bool(is_active)
        return bool(is_active) and not bool(was_active)

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
        )
        matches = evaluate_alert_rules(list(hist), params)

        for rule_id, match in matches.items():
            if not self._edge_fire(pid, rule_id, bool(match.active)):
                continue

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
