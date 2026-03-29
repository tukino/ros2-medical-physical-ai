#!/usr/bin/env python3
"""BCI monitor node (Day15).

Subscribes to `/{pid}/patient_bci` and emits state transitions as event logs.
"""

from __future__ import annotations

import sys
from typing import Dict
from typing import List
from typing import Optional

import rclpy
from medical_interfaces.msg import BCIFeatures
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from medical_robot_sim.bci_patient_state import classify_bci_state
from medical_robot_sim.observability import format_event
from medical_robot_sim.qos_profiles import build_qos_profile


class BciMonitorNode(Node):
    def __init__(self):
        super().__init__('bci_monitor')

        self.declare_parameter('patients', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('bci_topic', 'patient_bci')

        # Day8: QoS params
        self.declare_parameter('bci_qos_depth', 10)
        self.declare_parameter('bci_qos_reliability', 'reliable')
        self.declare_parameter('bci_qos_durability', 'volatile')

        bci_qos_depth = int(self.get_parameter('bci_qos_depth').value)
        bci_qos_reliability = str(self.get_parameter('bci_qos_reliability').value)
        bci_qos_durability = str(self.get_parameter('bci_qos_durability').value)

        try:
            bci_qos = build_qos_profile(
                depth=bci_qos_depth,
                reliability=bci_qos_reliability,
                durability=bci_qos_durability,
            )
        except ValueError as exc:
            self.get_logger().error(f"[Day8] Invalid BCI QoS params: {exc}")
            raise SystemExit(2)

        patient_ids = list(
            self.get_parameter('patients').get_parameter_value().string_array_value
        )
        bci_topic = str(self.get_parameter('bci_topic').value)

        patient_ids = [str(pid).strip().lstrip('/') for pid in patient_ids if str(pid).strip()]
        bci_topic = bci_topic.strip().lstrip('/')

        # De-dup (keep order)
        unique_patient_ids: List[str] = []
        seen = set()
        for pid in patient_ids:
            if pid not in seen:
                unique_patient_ids.append(pid)
                seen.add(pid)
        patient_ids = unique_patient_ids

        if not patient_ids:
            raise RuntimeError("Parameter 'patients' is empty")
        if not bci_topic:
            raise RuntimeError("Parameter 'bci_topic' is empty")

        self._patient_ids = patient_ids
        self._bci_topic = bci_topic

        self._subscriptions = []
        self._latest_stamp_ns: Dict[str, int] = {}
        self._last_measurement_id: Dict[str, int] = {}

        # Day11 style: state transitions only
        self._last_state: Dict[str, Optional[str]] = {pid: None for pid in patient_ids}

        for pid in patient_ids:
            topic = f'/{pid}/{bci_topic}'
            sub = self.create_subscription(
                BCIFeatures,
                topic,
                lambda msg, pid=pid: self._on_bci(pid, msg),
                bci_qos,
            )
            self._subscriptions.append(sub)

        self.get_logger().info('BCI monitor を起動しました')
        self.get_logger().info(f"patients={patient_ids}")
        self.get_logger().info(f"bci_topic='{bci_topic}'")
        self.get_logger().info(
            '[Day8] bci_qos='
            f"KEEP_LAST depth={bci_qos_depth}, reliability={bci_qos_reliability}, "
            f"durability={bci_qos_durability}"
        )

        self._timer = self.create_timer(1.0, self._log_states)

    def _on_bci(self, pid: str, msg: BCIFeatures) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._latest_stamp_ns[pid] = int(now_ns)
        self._last_measurement_id[pid] = int(msg.measurement_id)

    def _age_seconds(self, pid: str) -> Optional[float]:
        stamp_ns = self._latest_stamp_ns.get(pid)
        if stamp_ns is None:
            return None
        now_ns = self.get_clock().now().nanoseconds
        return max(0.0, (now_ns - int(stamp_ns)) / 1e9)

    def _emit_state_if_changed(self, pid: str) -> None:
        try:
            age_s = self._age_seconds(pid)
            state = classify_bci_state(age_s, stale_after_sec=3.0, no_data_after_sec=10.0)
            prev = self._last_state.get(pid)
            if prev == state:
                return

            self._last_state[pid] = state

            last_mid = self._last_measurement_id.get(pid)

            self.get_logger().info(
                format_event(
                    'bci.patient_state',
                    node=str(self.get_name()),
                    ns=str(self.get_namespace()),
                    pid=str(pid),
                    state=str(state),
                    age_sec=(float(f"{age_s:.1f}") if age_s is not None else 'N/A'),
                    last_measurement_id=(int(last_mid) if last_mid is not None else 'N/A'),
                )
            )
        except Exception:
            return

    def _log_states(self) -> None:
        for pid in self._patient_ids:
            self._emit_state_if_changed(pid)

    def destroy_node(self):
        try:
            if getattr(self, '_timer', None) is not None:
                try:
                    self._timer.cancel()
                except Exception:
                    pass
                try:
                    self.destroy_timer(self._timer)
                except Exception:
                    pass
        except Exception:
            pass

        try:
            for sub in list(getattr(self, '_subscriptions', [])):
                try:
                    self.destroy_subscription(sub)
                except (ValueError, RCLError):
                    pass
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None

    try:
        node = BciMonitorNode()

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
