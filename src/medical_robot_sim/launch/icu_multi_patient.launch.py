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

    scenario = LaunchConfiguration('scenario').perform(context).strip()

    enabled_rule_ids_str = LaunchConfiguration('enabled_rule_ids').perform(context).strip()
    # CSV → list（空文字はそのまま渡すと空配列扱い）
    enabled_rule_ids = [x.strip() for x in enabled_rule_ids_str.split(',') if x.strip()]

    flatline_history_size = int(LaunchConfiguration('flatline_history_size').perform(context))
    flatline_hr_epsilon = float(LaunchConfiguration('flatline_hr_epsilon').perform(context))
    flatline_spo2_epsilon = float(LaunchConfiguration('flatline_spo2_epsilon').perform(context))

    sigterm_timeout = LaunchConfiguration('sigterm_timeout')
    sigkill_timeout = LaunchConfiguration('sigkill_timeout')

    if not patient_ids:
        raise RuntimeError("Launch argument 'patients' resulted in empty patient list")

    actions = []

    # 患者ごとに vital_sensor を namespace 付きで起動
    for pid in patient_ids:
        sensor_params: list = [{'patient_id': pid}]
        if scenario:
            sensor_params.append({'scenario': scenario})

        actions.append(
            Node(
                package='medical_robot_sim',
                executable='vital_sensor',
                name='vital_sensor',
                namespace=pid,
                output='screen',
                parameters=sensor_params,
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
        engine_params: list = [
            {'patients': patient_ids},
            {'vitals_topic': 'patient_vitals'},
            {'alert_topic': 'alerts'},
            {'flatline_history_size': flatline_history_size},
            {'flatline_hr_epsilon': flatline_hr_epsilon},
            {'flatline_spo2_epsilon': flatline_spo2_epsilon},
        ]
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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'patients',
                default_value='patient_01,patient_02,patient_03,patient_04,patient_05',
                description='Comma-separated patient namespaces (e.g. patient_01,patient_02)',
            ),
            DeclareLaunchArgument(
                'enable_alerts',
                default_value='true',
                description='If true, start rule_alert_engine (publishes /<patient>/alerts).',
            ),
            DeclareLaunchArgument(
                'scenario',
                default_value='',
                description=(
                    "vital_sensor scenario: '' (normal), 'spo2_drop', 'flatline'. "
                    "Applied to all patients."
                ),
            ),
            DeclareLaunchArgument(
                'enabled_rule_ids',
                default_value='',
                description=(
                    'Comma-separated rule IDs to enable in rule_alert_engine. '
                    'Empty means all rules are active. '
                    'e.g. flatline.hr,flatline.spo2'
                ),
            ),
            DeclareLaunchArgument(
                'flatline_history_size',
                default_value='8',
                description='Number of recent samples used for flatline detection.',
            ),
            DeclareLaunchArgument(
                'flatline_hr_epsilon',
                default_value='1.0',
                description='Max HR range (max-min) to be classified as flatline [bpm].',
            ),
            DeclareLaunchArgument(
                'flatline_spo2_epsilon',
                default_value='1.0',
                description='Max SpO2 range (max-min) to be classified as flatline [%].',
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
