#!/usr/bin/env python3
"""ICU hardware-input single-patient launch (Day13).

Starts:
- hw_vital_sensor (namespaced under the patient)
- icu_monitor (root namespace)
- rule_alert_engine (optional)

Design goal: keep topic contracts unchanged.
- hw_vital_sensor publishes relative 'patient_vitals' -> /<patient>/patient_vitals
"""

from __future__ import annotations

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _package_share_rules_path() -> str:
    try:
        package_share = get_package_share_directory('medical_robot_sim')
    except Exception:
        return ''
    candidate = os.path.join(package_share, 'config', 'alert_rules.yaml')
    return candidate if os.path.isfile(candidate) else ''


def _source_tree_rules_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(here, '..', 'config', 'alert_rules.yaml'))
    return candidate if os.path.isfile(candidate) else ''


def _resolve_rules_path(path: str) -> str:
    raw = str(path).strip()
    if not raw:
        return ''
    expanded = os.path.expandvars(os.path.expanduser(raw))
    if os.path.isabs(expanded):
        return os.path.normpath(expanded)

    try:
        package_share = get_package_share_directory('medical_robot_sim')
    except Exception:
        package_share = ''
    if package_share:
        share_candidate = os.path.join(package_share, expanded)
        if os.path.isfile(share_candidate):
            return os.path.normpath(share_candidate)

    here = os.path.dirname(os.path.abspath(__file__))
    source_root = os.path.normpath(os.path.join(here, '..'))
    source_candidate = os.path.join(source_root, expanded)
    if os.path.isfile(source_candidate):
        return os.path.normpath(source_candidate)

    return os.path.normpath(expanded)


def _launch_setup(context, *args, **kwargs):
    patient = LaunchConfiguration('patient').perform(context).strip()
    if not patient:
        raise RuntimeError("Launch argument 'patient' is empty")

    enable_alerts = _parse_bool(LaunchConfiguration('enable_alerts').perform(context))

    driver = LaunchConfiguration('driver').perform(context).strip()
    mock_scenario = LaunchConfiguration('mock_scenario').perform(context).strip()

    publish_rate_hz = float(LaunchConfiguration('publish_rate_hz').perform(context))
    observability_verbose = _parse_bool(
        LaunchConfiguration('observability_verbose').perform(context)
    )

    i2c_bus = int(LaunchConfiguration('i2c_bus').perform(context))
    i2c_addr = LaunchConfiguration('i2c_addr').perform(context).strip()

    spi_bus = int(LaunchConfiguration('spi_bus').perform(context))
    spi_device = int(LaunchConfiguration('spi_device').perform(context))

    # Day8: QoS params
    vitals_qos_depth = int(LaunchConfiguration('vitals_qos_depth').perform(context))
    vitals_qos_reliability = LaunchConfiguration('vitals_qos_reliability').perform(
        context
    ).strip()
    vitals_qos_durability = LaunchConfiguration('vitals_qos_durability').perform(
        context
    ).strip()

    alerts_qos_depth = int(LaunchConfiguration('alerts_qos_depth').perform(context))
    alerts_qos_reliability = LaunchConfiguration('alerts_qos_reliability').perform(
        context
    ).strip()
    alerts_qos_durability = LaunchConfiguration('alerts_qos_durability').perform(
        context
    ).strip()

    enabled_rule_ids_str = LaunchConfiguration('enabled_rule_ids').perform(context).strip()
    enabled_rule_ids = [x.strip() for x in enabled_rule_ids_str.split(',') if x.strip()]

    rules_path = LaunchConfiguration('rules_path').perform(context).strip()
    if not rules_path:
        rules_path = _package_share_rules_path() or _source_tree_rules_path()
    rules_path = _resolve_rules_path(rules_path)

    flatline_history_size = int(LaunchConfiguration('flatline_history_size').perform(context))
    flatline_hr_epsilon = float(LaunchConfiguration('flatline_hr_epsilon').perform(context))
    flatline_spo2_epsilon = float(
        LaunchConfiguration('flatline_spo2_epsilon').perform(context)
    )

    sigterm_timeout = LaunchConfiguration('sigterm_timeout')
    sigkill_timeout = LaunchConfiguration('sigkill_timeout')

    actions = []

    # Day13: hw sensor node
    hw_params: List[dict] = [
        {'patient_id': patient},
        {'publish_rate_hz': publish_rate_hz},
        {'driver': driver},
        {'mock_scenario': mock_scenario},
        {'i2c_bus': i2c_bus},
        {'i2c_addr': i2c_addr},
        {'spi_bus': spi_bus},
        {'spi_device': spi_device},
        {'observability_verbose': bool(observability_verbose)},
        {'vitals_qos_depth': vitals_qos_depth},
        {'vitals_qos_reliability': vitals_qos_reliability},
        {'vitals_qos_durability': vitals_qos_durability},
    ]

    actions.append(
        Node(
            package='medical_robot_sim',
            executable='hw_vital_sensor',
            name='hw_vital_sensor',
            namespace=patient,
            output='screen',
            parameters=hw_params,
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
        )
    )

    # Monitor (root)
    actions.append(
        Node(
            package='medical_robot_sim',
            executable='icu_monitor',
            name='icu_monitor',
            output='screen',
            parameters=[
                {'patients': [patient]},
                {'vitals_topic': 'patient_vitals'},
                {'vitals_qos_depth': vitals_qos_depth},
                {'vitals_qos_reliability': vitals_qos_reliability},
                {'vitals_qos_durability': vitals_qos_durability},
            ],
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
        )
    )

    if enable_alerts:
        engine_params: List[dict] = [
            {'patients': [patient]},
            {'vitals_topic': 'patient_vitals'},
            {'alert_topic': 'alerts'},
            {'vitals_qos_depth': vitals_qos_depth},
            {'vitals_qos_reliability': vitals_qos_reliability},
            {'vitals_qos_durability': vitals_qos_durability},
            {'alerts_qos_depth': alerts_qos_depth},
            {'alerts_qos_reliability': alerts_qos_reliability},
            {'alerts_qos_durability': alerts_qos_durability},
            {'flatline_history_size': flatline_history_size},
            {'flatline_hr_epsilon': flatline_hr_epsilon},
            {'flatline_spo2_epsilon': flatline_spo2_epsilon},
        ]
        if rules_path:
            engine_params.append({'rules_path': rules_path})
        if enabled_rule_ids:
            engine_params.append({'enabled_rule_ids': enabled_rule_ids})

        actions.append(
            Node(
                package='medical_robot_sim',
                executable='rule_alert_engine',
                name='rule_alert_engine',
                output='screen',
                parameters=engine_params,
                sigterm_timeout=sigterm_timeout,
                sigkill_timeout=sigkill_timeout,
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    default_rules_path = _package_share_rules_path() or _source_tree_rules_path()

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
            # Day13: driver selection
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
                description='Day11-style verbose event logs for hw_vital_sensor.',
            ),
            # Optional I2C/SPI params
            DeclareLaunchArgument('i2c_bus', default_value='1', description='I2C bus number.'),
            DeclareLaunchArgument(
                'i2c_addr',
                default_value='0x57',
                description='I2C address (int or hex like 0x57).',
            ),
            DeclareLaunchArgument('spi_bus', default_value='0', description='SPI bus number.'),
            DeclareLaunchArgument(
                'spi_device', default_value='0', description='SPI device (CS) number.'
            ),
            # Day7: rules
            DeclareLaunchArgument(
                'rules_path',
                default_value=default_rules_path,
                description='Path to alert_rules.yaml (empty to disable YAML loading).',
            ),
            DeclareLaunchArgument(
                'enabled_rule_ids',
                default_value='',
                description='CSV of enabled rule_ids (empty means all rules).',
            ),
            # Day6: flatline params (used by alert engine)
            DeclareLaunchArgument('flatline_history_size', default_value='5'),
            DeclareLaunchArgument('flatline_hr_epsilon', default_value='0.0'),
            DeclareLaunchArgument('flatline_spo2_epsilon', default_value='0.0'),
            # Day8: QoS params
            DeclareLaunchArgument('vitals_qos_depth', default_value='10'),
            DeclareLaunchArgument('vitals_qos_reliability', default_value='reliable'),
            DeclareLaunchArgument('vitals_qos_durability', default_value='volatile'),
            DeclareLaunchArgument('alerts_qos_depth', default_value='10'),
            DeclareLaunchArgument('alerts_qos_reliability', default_value='reliable'),
            DeclareLaunchArgument('alerts_qos_durability', default_value='volatile'),
            # Shutdown grace
            DeclareLaunchArgument('sigterm_timeout', default_value='5'),
            DeclareLaunchArgument('sigkill_timeout', default_value='5'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
