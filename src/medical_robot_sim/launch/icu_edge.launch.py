#!/usr/bin/env python3
"""ICU Edge launch (Day14).

Design goal:
- Provide a short, edge-friendly entrypoint with safe defaults.
- Keep existing topic contracts unchanged.

Implementation:
- This launch is intentionally a thin wrapper over Day13's icu_hw_sensor.launch.py
  (single-patient + hw_vital_sensor + icu_monitor + optional rule_alert_engine).
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    patient = LaunchConfiguration('patient')
    enable_alerts = LaunchConfiguration('enable_alerts')
    driver = LaunchConfiguration('driver')
    mock_scenario = LaunchConfiguration('mock_scenario')

    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    observability_verbose = LaunchConfiguration('observability_verbose')

    included = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare('medical_robot_sim'),
                '/launch/icu_hw_sensor.launch.py',
            ]
        ),
        launch_arguments={
            'patient': patient,
            'enable_alerts': enable_alerts,
            'driver': driver,
            'mock_scenario': mock_scenario,
            'publish_rate_hz': publish_rate_hz,
            'observability_verbose': observability_verbose,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'patient',
                default_value='patient_01',
                description='Single patient namespace (e.g. patient_01).',
            ),
            DeclareLaunchArgument(
                'enable_alerts',
                default_value='true',
                description='If true, start rule_alert_engine (publishes /<patient>/alerts).',
            ),
            DeclareLaunchArgument(
                'driver',
                default_value='mock',
                description='Day13 input driver: mock | i2c | spi',
            ),
            DeclareLaunchArgument(
                'mock_scenario',
                default_value='normal',
                description='Day13 mock scenario: normal | spo2_drop | flatline',
            ),
            DeclareLaunchArgument(
                'publish_rate_hz',
                default_value='1.0',
                description='Publish rate for hw_vital_sensor.',
            ),
            DeclareLaunchArgument(
                'observability_verbose',
                default_value='false',
                description='If true, enable verbose Day11-style events for hw_vital_sensor.',
            ),
            included,
        ]
    )
