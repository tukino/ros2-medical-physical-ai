#!/usr/bin/env python3
"""Day16 advisory publisher node.

Subscribes to `/patient_XX/patient_vitals` and publishes anomaly advisories to
`/patient_XX/advisories` using `medical_interfaces/msg/Alert`.

Important:
- Advisories are NOT alerts; rule-based `/patient_XX/alerts` remains unchanged.
- This node is optional and guarded by a launch arg (enable_advisories).
"""

from __future__ import annotations

import sys

import rclpy
from medical_interfaces.msg import Alert
from medical_interfaces.msg import VitalSigns
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from medical_robot_sim.advisory_alerts import anomaly_event_to_advisory_alert
from medical_robot_sim.anomaly_detector import DEFAULT_FIELD_EPSILON
from medical_robot_sim.anomaly_detector import DEFAULT_HR_JUMP_THRESHOLD
from medical_robot_sim.anomaly_detector import DEFAULT_SPO2_DROP_THRESHOLD
from medical_robot_sim.anomaly_detector import DEFAULT_WINDOW_SEC
from medical_robot_sim.anomaly_detector import DEFAULT_WINDOW_SIZE
from medical_robot_sim.anomaly_detector import FlatlineDetector
from medical_robot_sim.observability import format_event
from medical_robot_sim.qos_profiles import build_qos_profile


class AdvisoryPublisherNode(Node):
    def __init__(self):
        super().__init__('advisory_publisher')

        self.declare_parameter('patient_id', 'patient_01')

        # Detection params (keep defaults aligned with anomaly_detector)
        self.declare_parameter('window_sec', int(DEFAULT_WINDOW_SEC))
        self.declare_parameter('window_size', int(DEFAULT_WINDOW_SIZE))
        self.declare_parameter('spo2_drop_threshold', float(DEFAULT_SPO2_DROP_THRESHOLD))
        self.declare_parameter('hr_jump_threshold', float(DEFAULT_HR_JUMP_THRESHOLD))

        # Keep naming consistent with Day6/Day7 knobs where possible.
        self.declare_parameter('flatline_hr_epsilon', float(DEFAULT_FIELD_EPSILON['heart_rate']))
        self.declare_parameter(
            'flatline_spo2_epsilon', float(DEFAULT_FIELD_EPSILON['oxygen_saturation'])
        )

        # Day8: QoS params (subscribe vitals, publish advisories)
        self.declare_parameter('vitals_qos_depth', 10)
        self.declare_parameter('vitals_qos_reliability', 'reliable')
        self.declare_parameter('vitals_qos_durability', 'volatile')

        self.declare_parameter('advisories_qos_depth', 10)
        self.declare_parameter('advisories_qos_reliability', 'reliable')
        self.declare_parameter('advisories_qos_durability', 'volatile')

        patient_id = str(self.get_parameter('patient_id').value).strip().lstrip('/')
        window_sec = int(self.get_parameter('window_sec').value)
        window_size = int(self.get_parameter('window_size').value)
        spo2_drop_threshold = float(self.get_parameter('spo2_drop_threshold').value)
        hr_jump_threshold = float(self.get_parameter('hr_jump_threshold').value)
        flatline_hr_epsilon = float(self.get_parameter('flatline_hr_epsilon').value)
        flatline_spo2_epsilon = float(self.get_parameter('flatline_spo2_epsilon').value)

        vitals_qos_depth = int(self.get_parameter('vitals_qos_depth').value)
        vitals_qos_reliability = str(self.get_parameter('vitals_qos_reliability').value)
        vitals_qos_durability = str(self.get_parameter('vitals_qos_durability').value)

        advisories_qos_depth = int(self.get_parameter('advisories_qos_depth').value)
        advisories_qos_reliability = str(self.get_parameter('advisories_qos_reliability').value)
        advisories_qos_durability = str(self.get_parameter('advisories_qos_durability').value)

        if not patient_id:
            raise RuntimeError("Parameter 'patient_id' is empty")

        try:
            vitals_qos = build_qos_profile(
                depth=vitals_qos_depth,
                reliability=vitals_qos_reliability,
                durability=vitals_qos_durability,
            )
            advisories_qos = build_qos_profile(
                depth=advisories_qos_depth,
                reliability=advisories_qos_reliability,
                durability=advisories_qos_durability,
            )
        except ValueError as exc:
            self.get_logger().error(f"[Day8] Invalid QoS params: {exc}")
            raise SystemExit(2)

        self._patient_id = patient_id

        # Per-node detector instance (keeps state isolated and testable).
        field_epsilon = dict(DEFAULT_FIELD_EPSILON)
        field_epsilon['heart_rate'] = float(flatline_hr_epsilon)
        field_epsilon['oxygen_saturation'] = float(flatline_spo2_epsilon)

        self._detector = FlatlineDetector(
            window_sec=int(window_sec),
            window_size=int(window_size),
            field_epsilon=field_epsilon,
            spo2_drop_threshold=float(spo2_drop_threshold),
            hr_jump_threshold=float(hr_jump_threshold),
        )

        self._sub = self.create_subscription(
            VitalSigns,
            'patient_vitals',
            self._on_vitals,
            vitals_qos,
        )

        self._pub = self.create_publisher(Alert, 'advisories', advisories_qos)

        self.get_logger().info(
            format_event(
                'advisory.config',
                node=self.get_name(),
                ns=self.get_namespace(),
                patient_id=patient_id,
                window_sec=window_sec,
                window_size=window_size,
                spo2_drop_threshold=spo2_drop_threshold,
                hr_jump_threshold=hr_jump_threshold,
                flatline_hr_epsilon=flatline_hr_epsilon,
                flatline_spo2_epsilon=flatline_spo2_epsilon,
                vitals_qos_depth=vitals_qos_depth,
                vitals_qos_reliability=vitals_qos_reliability,
                vitals_qos_durability=vitals_qos_durability,
                advisories_qos_depth=advisories_qos_depth,
                advisories_qos_reliability=advisories_qos_reliability,
                advisories_qos_durability=advisories_qos_durability,
            )
        )

    def _on_vitals(self, msg: VitalSigns) -> None:
        now_s = float(self.get_clock().now().nanoseconds) / 1e9

        try:
            events = self._detector.update(msg, ts=now_s)
        except Exception as exc:
            self.get_logger().error(
                format_event(
                    'advisory.detect_fail',
                    node=self.get_name(),
                    ns=self.get_namespace(),
                    error=repr(exc),
                )
            )
            return

        if not events:
            return

        pid = str(getattr(msg, 'patient_id', '') or self._patient_id)

        for event in events:
            alert = anomaly_event_to_advisory_alert(patient_id=pid, event=event)
            self._pub.publish(alert)

            self.get_logger().info(
                format_event(
                    'advisory.publish',
                    node=self.get_name(),
                    ns=self.get_namespace(),
                    pid=str(alert.patient_id),
                    rule_id=str(alert.rule_id),
                    kind=str(alert.kind),
                    priority=str(alert.priority),
                    field=str(alert.field),
                    score=(float(alert.score) if alert.score == alert.score else 'NaN'),
                    window_sec=int(alert.window_sec),
                )
            )

    def destroy_node(self):
        try:
            if getattr(self, '_sub', None) is not None:
                try:
                    self.destroy_subscription(self._sub)
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
        node = AdvisoryPublisherNode()

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
