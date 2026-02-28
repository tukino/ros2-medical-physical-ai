#!/usr/bin/env python3
"""
ICU multi-patient launch (ROS 2 Humble).

目的:
- patients 引数(CSV)で指定された複数患者ぶんの vital_sensor を namespace 別に起動
- 集約ノード icu_monitor を 1 つ起動

前提/制約:
- vital_sensor / icu_monitor 側の topic 名は相対名（先頭'/'なし）を前提
  - vital_sensor が publish する 'patient_vitals' は namespace により
    /patient_01/patient_vitals のように解決される
  - icu_monitor は parameters の patients + vitals_topic を使って購読先を解決する想定

動作確認（例）:
- ビルド:  colcon build --symlink-install
- 実行:    ros2 launch medical_robot_sim icu_multi_patient.launch.py
- 患者指定: ros2 launch medical_robot_sim icu_multi_patient.launch.py patients:=patient_01,patient_02
- topic確認: ros2 topic list | grep patient_vitals
"""

from __future__ import annotations

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_patients_csv(value: str) -> List[str]:
    # CSV を split し、空要素と前後空白を除去
    patient_ids = [item.strip() for item in value.split(',') if item.strip()]
    return patient_ids


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _launch_setup(context, *args, **kwargs):
    patients_csv = LaunchConfiguration('patients').perform(context)
    patient_ids = _parse_patients_csv(patients_csv)

    enable_alerts_str = LaunchConfiguration('enable_alerts').perform(context)
    enable_alerts = _parse_bool(enable_alerts_str)

    sigterm_timeout = LaunchConfiguration('sigterm_timeout')
    sigkill_timeout = LaunchConfiguration('sigkill_timeout')

    if not patient_ids:
        raise RuntimeError("Launch argument 'patients' resulted in empty patient list")

    actions = []

    # 患者ごとに vital_sensor を namespace 付きで起動
    for pid in patient_ids:
        actions.append(
            Node(
                package='medical_robot_sim',
                executable='vital_sensor',
                name='vital_sensor',
                namespace=pid,
                output='screen',
                parameters=[{'patient_id': pid}],
                sigterm_timeout=sigterm_timeout,
                sigkill_timeout=sigkill_timeout,
            )
        )

    # 集約ノード（root namespace）
    actions.append(
        Node(
            package='medical_robot_sim',
            executable='icu_monitor',
            name='icu_monitor',
            output='screen',
            parameters=[{'patients': patient_ids}, {'vitals_topic': 'patient_vitals'}],
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
        )
    )

    if enable_alerts:
        actions.append(
            Node(
                package='medical_robot_sim',
                executable='rule_alert_engine',
                name='rule_alert_engine',
                output='screen',
                parameters=[
                    {'patients': patient_ids},
                    {'vitals_topic': 'patient_vitals'},
                    {'alert_topic': 'alerts'},
                ],
                sigterm_timeout=sigterm_timeout,
                sigkill_timeout=sigkill_timeout,
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'patients',
                default_value='patient_01,patient_02,patient_03,patient_04,patient_05',
                description='Comma-separated patient namespaces (e.g. patient_01,patient_02)',
            ),
            DeclareLaunchArgument(
                'enable_alerts',
                default_value='false',
                description='If true, start rule_alert_engine (publishes /<patient>/alerts).',
            ),
            DeclareLaunchArgument(
                'sigterm_timeout',
                default_value='5',
                description=(
                    'Seconds to wait after SIGINT before escalating to SIGTERM '
                    '(launch.actions.ExecuteLocal/ExecuteProcess)'
                ),
            ),
            DeclareLaunchArgument(
                'sigkill_timeout',
                default_value='5',
                description=(
                    'Additional seconds to wait after SIGTERM before escalating to SIGKILL '
                    '(launch.actions.ExecuteLocal/ExecuteProcess)'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
