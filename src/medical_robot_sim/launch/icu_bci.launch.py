#!/usr/bin/env python3
"""ICU BCI launch (Day15).

目的:
- 単一患者の BCI センサ（bci_sensor）を namespace 付きで起動
- 集約ノード bci_monitor を root で起動し、state 遷移を event ログ化

動作確認（例）:
- ros2 launch medical_robot_sim icu_bci.launch.py patient:=patient_01 driver:=mock
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    patient = LaunchConfiguration('patient').perform(context).strip().lstrip('/')
    driver = LaunchConfiguration('driver').perform(context).strip()
    mock_scenario = LaunchConfiguration('mock_scenario').perform(context).strip()
    publish_rate_hz = float(LaunchConfiguration('publish_rate_hz').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    serial_port = LaunchConfiguration('serial_port').perform(context).strip()

    bci_qos_depth = int(LaunchConfiguration('bci_qos_depth').perform(context))
    bci_qos_reliability = LaunchConfiguration('bci_qos_reliability').perform(context).strip()
    bci_qos_durability = LaunchConfiguration('bci_qos_durability').perform(context).strip()

    sigterm_timeout = LaunchConfiguration('sigterm_timeout')
    sigkill_timeout = LaunchConfiguration('sigkill_timeout')

    if not patient:
        raise RuntimeError("Launch argument 'patient' is empty")

    sensor_params = [
        {'patient_id': patient},
        {'publish_rate_hz': publish_rate_hz},
        {'driver': driver},
        {'mock_scenario': mock_scenario},
        {'seed': seed},
        {'serial_port': serial_port},
        {'bci_qos_depth': bci_qos_depth},
        {'bci_qos_reliability': bci_qos_reliability},
        {'bci_qos_durability': bci_qos_durability},
    ]

    monitor_params = [
        {'patients': [patient]},
        {'bci_topic': 'patient_bci'},
        {'bci_qos_depth': bci_qos_depth},
        {'bci_qos_reliability': bci_qos_reliability},
        {'bci_qos_durability': bci_qos_durability},
    ]

    return [
        Node(
            package='medical_robot_sim',
            executable='bci_sensor',
            name='bci_sensor',
            namespace=patient,
            output='screen',
            parameters=sensor_params,
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
        ),
        Node(
            package='medical_robot_sim',
            executable='bci_monitor',
            name='bci_monitor',
            output='screen',
            parameters=monitor_params,
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('patient', default_value='patient_01'),
            DeclareLaunchArgument('driver', default_value='mock'),
            DeclareLaunchArgument('mock_scenario', default_value='normal'),
            DeclareLaunchArgument('publish_rate_hz', default_value='10.0'),
            DeclareLaunchArgument('seed', default_value='42'),
            DeclareLaunchArgument('serial_port', default_value=''),
            # Day8 QoS
            DeclareLaunchArgument('bci_qos_depth', default_value='10'),
            DeclareLaunchArgument('bci_qos_reliability', default_value='reliable'),
            DeclareLaunchArgument('bci_qos_durability', default_value='volatile'),
            # Clean shutdown timeouts
            DeclareLaunchArgument('sigterm_timeout', default_value='5'),
            DeclareLaunchArgument('sigkill_timeout', default_value='5'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
