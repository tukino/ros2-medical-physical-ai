#!/usr/bin/env python3
"""Day18 closed-loop controller node.

Subscribes to patient vitals/alerts and emits control actions as Alert messages
on /<patient>/control_actions.
"""

from __future__ import annotations

import math
import sys
from typing import Optional

import rclpy
from medical_interfaces.msg import Alert
from medical_interfaces.msg import VitalSigns
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from medical_robot_sim.closed_loop_policy import ACTION_CALL_STAFF
from medical_robot_sim.closed_loop_policy import ACTION_OXYGEN_BOOST
from medical_robot_sim.closed_loop_policy import decide_control_action
from medical_robot_sim.observability import format_event
from medical_robot_sim.qos_profiles import build_qos_profile


def _nan32() -> float:
    return float('nan')


def _rule_id_for_action(action: str) -> str:
    if action == ACTION_CALL_STAFF:
        return 'control.call_staff'
    if action == ACTION_OXYGEN_BOOST:
        return 'control.oxygen_boost'
    return 'control.hold'


def _priority_for_action(action: str) -> str:
    if action == ACTION_CALL_STAFF:
        return 'RED'
    if action == ACTION_OXYGEN_BOOST:
        return 'YELLOW'
    return 'INFO'


def _score_for_action(action: str) -> float:
    if action == ACTION_CALL_STAFF:
        return 1.0
    if action == ACTION_OXYGEN_BOOST:
        return 0.5
    return 0.0


class ClosedLoopControllerNode(Node):
    def __init__(self):
        super().__init__('closed_loop_controller')

        self.declare_parameter('patient_id', '')
        self.declare_parameter('vitals_topic', 'patient_vitals')
        self.declare_parameter('alerts_topic', 'alerts')
        self.declare_parameter('advisories_topic', 'advisories')
        self.declare_parameter('control_topic', 'control_actions')

        self.declare_parameter('enable_control_from_advisories', False)
        self.declare_parameter('control_cooldown_sec', 5.0)
        self.declare_parameter('control_no_data_after_sec', 10.0)
        self.declare_parameter('control_low_spo2', 92.0)
        self.declare_parameter('control_critical_spo2', 88.0)

        # Reuse existing Day8 knobs.
        self.declare_parameter('vitals_qos_depth', 10)
        self.declare_parameter('vitals_qos_reliability', 'reliable')
        self.declare_parameter('vitals_qos_durability', 'volatile')
        self.declare_parameter('alerts_qos_depth', 10)
        self.declare_parameter('alerts_qos_reliability', 'reliable')
        self.declare_parameter('alerts_qos_durability', 'volatile')

        patient_id = str(self.get_parameter('patient_id').value).strip().lstrip('/')
        if not patient_id:
            # /patient_01 -> patient_01
            patient_id = str(self.get_namespace()).strip().lstrip('/')

        vitals_topic = str(self.get_parameter('vitals_topic').value).strip().lstrip('/')
        alerts_topic = str(self.get_parameter('alerts_topic').value).strip().lstrip('/')
        advisories_topic = str(self.get_parameter('advisories_topic').value).strip().lstrip('/')
        control_topic = str(self.get_parameter('control_topic').value).strip().lstrip('/')

        if not patient_id:
            raise RuntimeError("Parameter 'patient_id' is empty")
        if not vitals_topic:
            raise RuntimeError("Parameter 'vitals_topic' is empty")
        if not alerts_topic:
            raise RuntimeError("Parameter 'alerts_topic' is empty")
        if not control_topic:
            raise RuntimeError("Parameter 'control_topic' is empty")

        self._patient_id = patient_id
        self._vitals_topic = vitals_topic
        self._alerts_topic = alerts_topic
        self._advisories_topic = advisories_topic
        self._control_topic = control_topic

        self._enable_control_from_advisories = bool(
            self.get_parameter('enable_control_from_advisories').value
        )
        self._control_cooldown_sec = float(self.get_parameter('control_cooldown_sec').value)
        self._control_no_data_after_sec = float(
            self.get_parameter('control_no_data_after_sec').value
        )
        self._control_low_spo2 = float(self.get_parameter('control_low_spo2').value)
        self._control_critical_spo2 = float(
            self.get_parameter('control_critical_spo2').value
        )

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

        self._latest_vitals: Optional[VitalSigns] = None
        self._latest_vitals_received_ns: Optional[int] = None

        self._latest_alert_priority = ''
        self._latest_alert_rule_id = ''

        self._latest_advisory_priority = ''
        self._latest_advisory_rule_id = ''

        self._last_action: Optional[str] = None
        self._last_publish_ns: Optional[int] = None

        self._sub_vitals = self.create_subscription(
            VitalSigns,
            self._vitals_topic,
            self._on_vitals,
            vitals_qos,
        )
        self._sub_alerts = self.create_subscription(
            Alert,
            self._alerts_topic,
            self._on_alert,
            alerts_qos,
        )

        self._sub_advisories = None
        if self._enable_control_from_advisories:
            self._sub_advisories = self.create_subscription(
                Alert,
                self._advisories_topic,
                self._on_advisory,
                alerts_qos,
            )

        self._pub_control = self.create_publisher(Alert, self._control_topic, alerts_qos)

        self.get_logger().info(
            format_event(
                'control.config',
                node=self.get_name(),
                ns=self.get_namespace(),
                patient_id=self._patient_id,
                vitals_topic=self._vitals_topic,
                alerts_topic=self._alerts_topic,
                advisories_topic=self._advisories_topic,
                control_topic=self._control_topic,
                enable_control_from_advisories=self._enable_control_from_advisories,
                control_cooldown_sec=self._control_cooldown_sec,
                control_no_data_after_sec=self._control_no_data_after_sec,
                control_low_spo2=self._control_low_spo2,
                control_critical_spo2=self._control_critical_spo2,
            )
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _age_sec(self) -> Optional[float]:
        if self._latest_vitals_received_ns is None:
            return None
        return max(0.0, float(self._now_ns() - self._latest_vitals_received_ns) / 1e9)

    def _last_publish_age_sec(self) -> Optional[float]:
        if self._last_publish_ns is None:
            return None
        return max(0.0, float(self._now_ns() - self._last_publish_ns) / 1e9)

    def _on_vitals(self, msg: VitalSigns) -> None:
        self._latest_vitals = msg
        self._latest_vitals_received_ns = self._now_ns()
        self._evaluate_and_publish(trigger='vitals')

    def _on_alert(self, msg: Alert) -> None:
        self._latest_alert_priority = str(msg.priority)
        self._latest_alert_rule_id = str(msg.rule_id)
        self._evaluate_and_publish(trigger='alerts')

    def _on_advisory(self, msg: Alert) -> None:
        self._latest_advisory_priority = str(msg.priority)
        self._latest_advisory_rule_id = str(msg.rule_id)
        self._evaluate_and_publish(trigger='advisories')

    def _choose_control_context(self) -> tuple[str, str]:
        priority = self._latest_alert_priority
        rule_id = self._latest_alert_rule_id
        if not self._enable_control_from_advisories:
            return priority, rule_id

        adv_pri = str(self._latest_advisory_priority or '').strip().upper()
        if adv_pri == 'RED' and str(priority or '').strip().upper() != 'RED':
            return self._latest_advisory_priority, self._latest_advisory_rule_id

        if not str(rule_id or '').strip() and str(self._latest_advisory_rule_id or '').strip():
            return self._latest_advisory_priority, self._latest_advisory_rule_id

        return priority, rule_id

    def _evaluate_and_publish(self, *, trigger: str) -> None:
        msg = self._latest_vitals
        if msg is None:
            return

        alert_priority, alert_rule_id = self._choose_control_context()

        decision = decide_control_action(
            hr=msg.heart_rate,
            spo2=msg.oxygen_saturation,
            latest_alert_priority=alert_priority,
            latest_alert_rule_id=alert_rule_id,
            age_sec=self._age_sec(),
            last_action=self._last_action,
            last_publish_age_sec=self._last_publish_age_sec(),
            cooldown_sec=self._control_cooldown_sec,
            no_data_after_sec=self._control_no_data_after_sec,
            low_spo2=self._control_low_spo2,
            critical_spo2=self._control_critical_spo2,
        )

        self.get_logger().info(
            format_event(
                'control.decision',
                node=self.get_name(),
                ns=self.get_namespace(),
                pid=self._patient_id,
                trigger=trigger,
                action=decision.action,
                reason=decision.reason,
                severity=decision.severity,
                should_publish=decision.should_publish,
                spo2=float(msg.oxygen_saturation),
                hr=float(msg.heart_rate),
                latest_alert_priority=alert_priority,
                latest_alert_rule_id=alert_rule_id,
                age_sec=(self._age_sec() if self._age_sec() is not None else 'N/A'),
                cooldown_sec=self._control_cooldown_sec,
            )
        )

        age_sec = self._age_sec()
        if age_sec is None or age_sec > self._control_no_data_after_sec:
            self.get_logger().info(
                format_event(
                    'control.safe_hold',
                    node=self.get_name(),
                    ns=self.get_namespace(),
                    pid=self._patient_id,
                    reason='no_data_guard',
                    age_sec=(age_sec if age_sec is not None else 'N/A'),
                    no_data_after_sec=self._control_no_data_after_sec,
                )
            )

        if not decision.should_publish:
            self.get_logger().info(
                format_event(
                    'control.suppressed',
                    node=self.get_name(),
                    ns=self.get_namespace(),
                    pid=self._patient_id,
                    action=decision.action,
                    reason=decision.reason,
                    cooldown_sec=self._control_cooldown_sec,
                    since_last_publish_sec=(
                        self._last_publish_age_sec()
                        if self._last_publish_age_sec() is not None
                        else 'N/A'
                    ),
                )
            )
            return

        out = Alert()
        out.patient_id = str(getattr(msg, 'patient_id', '') or self._patient_id)
        out.rule_id = _rule_id_for_action(decision.action)
        out.kind = 'control_action'
        out.priority = _priority_for_action(decision.action)
        out.message = f"control_action={decision.action} reason={decision.reason}"
        out.ts = float(self._now_ns()) / 1e9
        out.window_sec = 0
        out.field = 'oxygen_saturation'

        spo2 = float(getattr(msg, 'oxygen_saturation', _nan32()))
        if math.isnan(spo2):
            out.value = _nan32()
        else:
            out.value = spo2
        out.delta = _nan32()
        out.score = float(_score_for_action(decision.action))

        self._pub_control.publish(out)
        self._last_action = decision.action
        self._last_publish_ns = self._now_ns()

        self.get_logger().info(
            format_event(
                'control.publish',
                node=self.get_name(),
                ns=self.get_namespace(),
                pid=out.patient_id,
                action=decision.action,
                rule_id=out.rule_id,
                kind=out.kind,
                priority=out.priority,
                field=out.field,
                value=(float(out.value) if out.value == out.value else 'NaN'),
                score=float(out.score),
            )
        )

    def destroy_node(self):
        try:
            if getattr(self, '_sub_vitals', None) is not None:
                try:
                    self.destroy_subscription(self._sub_vitals)
                except Exception:
                    pass
            if getattr(self, '_sub_alerts', None) is not None:
                try:
                    self.destroy_subscription(self._sub_alerts)
                except Exception:
                    pass
            if getattr(self, '_sub_advisories', None) is not None:
                try:
                    self.destroy_subscription(self._sub_advisories)
                except Exception:
                    pass
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None

    try:
        node = ClosedLoopControllerNode()

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
            return 0
        except (KeyboardInterrupt, ExternalShutdownException):
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
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
