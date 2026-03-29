#!/usr/bin/env python3
"""BCI input node (Day15).

Publishes `medical_interfaces/msg/BCIFeatures` to the relative topic `patient_bci`.
"""

from __future__ import annotations

import math
import sys

import rclpy
from medical_interfaces.msg import BCIFeatures
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from medical_robot_sim.bci_io import MockBCIDevice
from medical_robot_sim.bci_io import SerialBCIDevice
from medical_robot_sim.observability import format_event
from medical_robot_sim.qos_profiles import build_qos_profile


def _normalize_driver(value: str) -> str:
    return str(value or '').strip().lower()


class BciSensorNode(Node):
    """BCI sensor node."""

    def __init__(self):
        super().__init__('bci_sensor')

        self.declare_parameter('patient_id', 'patient_01')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('driver', 'mock')
        self.declare_parameter('mock_scenario', 'normal')
        self.declare_parameter('seed', 42)

        # Optional (driver=serial)
        self.declare_parameter('serial_port', '')

        # Day8: QoS params
        self.declare_parameter('bci_qos_depth', 10)
        self.declare_parameter('bci_qos_reliability', 'reliable')
        self.declare_parameter('bci_qos_durability', 'volatile')

        patient_id = str(self.get_parameter('patient_id').value).strip().lstrip('/')
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        driver = _normalize_driver(self.get_parameter('driver').value)
        mock_scenario = str(self.get_parameter('mock_scenario').value).strip()
        seed = int(self.get_parameter('seed').value)
        serial_port = str(self.get_parameter('serial_port').value).strip()

        bci_qos_depth = int(self.get_parameter('bci_qos_depth').value)
        bci_qos_reliability = str(self.get_parameter('bci_qos_reliability').value)
        bci_qos_durability = str(self.get_parameter('bci_qos_durability').value)

        if not patient_id:
            raise RuntimeError("Parameter 'patient_id' is empty")
        if publish_rate_hz <= 0.0:
            raise RuntimeError("Parameter 'publish_rate_hz' must be > 0")
        if not driver:
            raise RuntimeError("Parameter 'driver' is empty")

        try:
            qos = build_qos_profile(
                depth=bci_qos_depth,
                reliability=bci_qos_reliability,
                durability=bci_qos_durability,
            )
        except ValueError as exc:
            self.get_logger().error(f"[Day8] Invalid BCI QoS params: {exc}")
            raise SystemExit(2)

        self._patient_id = patient_id
        self._publish_rate_hz = publish_rate_hz
        self._driver = driver
        self._mock_scenario = mock_scenario
        self._seed = seed
        self._serial_port = serial_port

        self._publisher = self.create_publisher(BCIFeatures, 'patient_bci', qos)

        self._measurement_id = 0

        # Device selection
        if driver == 'mock':
            self._device = MockBCIDevice(scenario=mock_scenario, seed=seed)
        elif driver == 'serial':
            self._device = SerialBCIDevice(port=serial_port)
        else:
            allowed = 'mock, serial'
            raise RuntimeError(f"Invalid driver '{driver}'. Allowed values: {allowed}")

        self.get_logger().info(
            format_event(
                'bci.device_config',
                driver=driver,
                mock_scenario=mock_scenario,
                node=self.get_name(),
                ns=self.get_namespace(),
                patient_id=patient_id,
                publish_rate_hz=publish_rate_hz,
                qos_depth=bci_qos_depth,
                qos_reliability=bci_qos_reliability,
                qos_durability=bci_qos_durability,
                seed=seed,
                serial_port=(serial_port or 'N/A'),
            )
        )

        try:
            self._device.open()
            self.get_logger().info(
                format_event(
                    'bci.device_open_ok',
                    driver=driver,
                    node=self.get_name(),
                    ns=self.get_namespace(),
                )
            )
        except Exception as exc:
            self.get_logger().error(
                format_event(
                    'bci.device_open_fail',
                    driver=driver,
                    error=repr(exc),
                    node=self.get_name(),
                    ns=self.get_namespace(),
                )
            )
            raise SystemExit(2)

        self._timer = self.create_timer(1.0 / float(publish_rate_hz), self._tick)

    def _tick(self) -> None:
        try:
            sample = self._device.read_features()
        except Exception as exc:
            # Keep emitting but mark as no_signal.
            sample = None
            self.get_logger().warn(
                format_event(
                    'bci.read_fail',
                    error=repr(exc),
                    node=self.get_name(),
                    ns=self.get_namespace(),
                )
            )

        msg = BCIFeatures()
        msg.patient_id = str(self._patient_id)

        mid = int(self._measurement_id) & 0xFFFFFFFF
        msg.measurement_id = mid
        self._measurement_id = (mid + 1) & 0xFFFFFFFF

        if sample is None:
            msg.attention = float('nan')
            msg.drowsiness = float('nan')
            msg.signal_quality = float('nan')
            msg.status = 'no_signal'
        else:
            msg.attention = float(sample.attention)
            msg.drowsiness = float(sample.drowsiness)
            msg.signal_quality = float(sample.signal_quality)
            msg.status = str(sample.status)

        # Ensure NaN is representable for float32 fields.
        if math.isnan(float(msg.attention)):
            msg.attention = float('nan')

        self._publisher.publish(msg)

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
            if getattr(self, '_device', None) is not None:
                try:
                    self._device.close()
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
        node = BciSensorNode()

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
            return 0
        except (KeyboardInterrupt, ExternalShutdownException):
            return 0
        except Exception:
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
