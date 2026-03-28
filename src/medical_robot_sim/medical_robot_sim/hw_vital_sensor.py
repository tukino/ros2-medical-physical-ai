#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import asdict
from typing import Literal, Optional, Tuple

import rclpy
from medical_interfaces.msg import VitalSigns
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from medical_robot_sim.hardware_io import I2CVitalDevice
from medical_robot_sim.hardware_io import MockVitalDevice
from medical_robot_sim.hardware_io import SPIVitalDevice
from medical_robot_sim.hardware_io import VitalDevice
from medical_robot_sim.hardware_io import VitalDeviceError
from medical_robot_sim.hardware_io import parse_int_auto_base
from medical_robot_sim.observability import format_event
from medical_robot_sim.qos_profiles import build_qos_profile

DriverKind = Literal['mock', 'i2c', 'spi']


def validate_hw_sensor_params(
    *,
    publish_rate_hz: float,
    driver: str,
    mock_scenario: str,
) -> Tuple[float, DriverKind, str]:
    publish_rate_hz_f = float(publish_rate_hz)
    if publish_rate_hz_f <= 0.0:
        raise ValueError("Parameter 'publish_rate_hz' must be > 0")

    driver_norm = str(driver).strip().lower()
    if driver_norm not in {'mock', 'i2c', 'spi'}:
        raise ValueError(
            "Parameter 'driver' must be one of: mock, i2c, spi"
        )

    scenario_norm = str(mock_scenario).strip().lower() or 'normal'
    allowed_scenarios = {'normal', 'spo2_drop', 'flatline'}
    if scenario_norm not in allowed_scenarios:
        raise ValueError(
            "Parameter 'mock_scenario' must be one of: normal, spo2_drop, flatline"
        )

    return publish_rate_hz_f, driver_norm, scenario_norm  # type: ignore[return-value]


def _build_device(
    *,
    patient_id: str,
    publish_rate_hz: float,
    driver: DriverKind,
    mock_scenario: str,
    i2c_bus: int,
    i2c_addr: str,
    spi_bus: int,
    spi_device: int,
) -> VitalDevice:
    if driver == 'mock':
        return MockVitalDevice(
            patient_id=patient_id,
            publish_rate_hz=publish_rate_hz,
            scenario=mock_scenario,
        )

    if driver == 'i2c':
        address_int = parse_int_auto_base(i2c_addr)
        return I2CVitalDevice(patient_id=patient_id, bus=int(i2c_bus), address=address_int)

    if driver == 'spi':
        return SPIVitalDevice(patient_id=patient_id, bus=int(spi_bus), device=int(spi_device))

    raise ValueError(f'Unexpected driver: {driver}')


class HardwareVitalSensorNode(Node):
    def __init__(self):
        super().__init__('hw_vital_sensor')

        # Observability
        self.declare_parameter('observability_verbose', False)
        self._observability_verbose = bool(self.get_parameter('observability_verbose').value)

        # Patient and rate
        self.declare_parameter('patient_id', 'patient_01')
        self._patient_id = str(self.get_parameter('patient_id').value)

        self.declare_parameter('publish_rate_hz', 1.0)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # Driver selection
        self.declare_parameter('driver', 'mock')
        driver_raw = str(self.get_parameter('driver').value)

        self.declare_parameter('mock_scenario', 'normal')
        mock_scenario_raw = str(self.get_parameter('mock_scenario').value)

        # Optional I2C params
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', '0x57')
        i2c_bus = int(self.get_parameter('i2c_bus').value)
        i2c_addr = str(self.get_parameter('i2c_addr').value)

        # Optional SPI params
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        spi_bus = int(self.get_parameter('spi_bus').value)
        spi_device = int(self.get_parameter('spi_device').value)

        try:
            publish_rate_hz, driver, mock_scenario = validate_hw_sensor_params(
                publish_rate_hz=publish_rate_hz,
                driver=driver_raw,
                mock_scenario=mock_scenario_raw,
            )
        except ValueError as exc:
            self.get_logger().error(f"[Day13] Invalid hw sensor params: {exc}")
            raise SystemExit(2)

        # QoS (Day8)
        self.declare_parameter('vitals_qos_depth', 10)
        self.declare_parameter('vitals_qos_reliability', 'reliable')
        self.declare_parameter('vitals_qos_durability', 'volatile')

        vitals_qos_depth = int(self.get_parameter('vitals_qos_depth').value)
        vitals_qos_reliability = str(self.get_parameter('vitals_qos_reliability').value)
        vitals_qos_durability = str(self.get_parameter('vitals_qos_durability').value)

        try:
            vitals_qos = build_qos_profile(
                depth=vitals_qos_depth,
                reliability=vitals_qos_reliability,
                durability=vitals_qos_durability,
            )
        except ValueError as exc:
            self.get_logger().error(f"[Day8] Invalid vitals QoS params: {exc}")
            raise SystemExit(2)

        self._publisher = self.create_publisher(VitalSigns, 'patient_vitals', vitals_qos)

        # Emit config event
        self.get_logger().info(
            format_event(
                'vitals.hw_device_config',
                driver=driver,
                i2c_addr=i2c_addr,
                i2c_bus=i2c_bus,
                mock_scenario=mock_scenario,
                node=self.get_name(),
                ns=self.get_namespace(),
                patient_id=self._patient_id,
                publish_rate_hz=publish_rate_hz,
                spi_bus=spi_bus,
                spi_device=spi_device,
                verbose=self._observability_verbose,
            )
        )

        self._device: VitalDevice = _build_device(
            patient_id=self._patient_id,
            publish_rate_hz=publish_rate_hz,
            driver=driver,
            mock_scenario=mock_scenario,
            i2c_bus=i2c_bus,
            i2c_addr=i2c_addr,
            spi_bus=spi_bus,
            spi_device=spi_device,
        )

        try:
            self._device.open()
        except Exception as exc:
            self.get_logger().error(
                format_event(
                    'vitals.hw_device_open_fail',
                    driver=driver,
                    error=str(exc),
                    node=self.get_name(),
                    ns=self.get_namespace(),
                    patient_id=self._patient_id,
                )
            )
            raise SystemExit(2)

        self.get_logger().info(
            format_event(
                'vitals.hw_device_open_ok',
                driver=driver,
                node=self.get_name(),
                ns=self.get_namespace(),
                patient_id=self._patient_id,
            )
        )

        self._measurement_id = 0
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def destroy_node(self) -> bool:
        try:
            self._device.close()
        except Exception:
            pass
        return super().destroy_node()

    def _on_timer(self) -> None:
        try:
            sample = self._device.read_sample()
        except VitalDeviceError as exc:
            if self._observability_verbose:
                self.get_logger().error(
                    format_event(
                        'vitals.hw_read_fail',
                        error=str(exc),
                        node=self.get_name(),
                        ns=self.get_namespace(),
                        patient_id=self._patient_id,
                    )
                )
            return

        msg = VitalSigns()
        msg.patient_id = sample.patient_id
        msg.measurement_id = int(self._measurement_id)
        msg.heart_rate = int(sample.heart_rate)
        msg.blood_pressure_systolic = int(sample.blood_pressure_systolic)
        msg.blood_pressure_diastolic = int(sample.blood_pressure_diastolic)
        msg.body_temperature = float(sample.body_temperature)
        msg.oxygen_saturation = int(sample.oxygen_saturation)
        msg.status = str(sample.status)

        self._publisher.publish(msg)
        self._measurement_id += 1

        if self._observability_verbose:
            fields = asdict(sample)
            fields.update(
                {
                    'measurement_id': int(msg.measurement_id),
                    'node': self.get_name(),
                    'ns': self.get_namespace(),
                }
            )
            self.get_logger().info(format_event('vitals.hw_read_ok', **fields))


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[HardwareVitalSensorNode] = None
    executor = SingleThreadedExecutor()

    try:
        node = HardwareVitalSensorNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                executor.remove_node(node)
                node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass
