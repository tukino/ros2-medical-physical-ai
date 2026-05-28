#!/usr/bin/env python3
"""ICU replay launch (ROS 2 Humble).

目的:
- rosbag replay 用に、vital_sensor を起動せずに下流だけを立ち上げる
  - icu_monitor（vitals 購読）
  - rule_alert_engine / rule_alert_engine_lifecycle（任意。vitals 購読→alerts publish）
  - closed_loop_controller（任意。Day18/Day19。vitals/alerts 購読→control_actions publish）

注意:
- bag play は別プロセスで実行し、/patient_XX/patient_vitals を publish する
- 本 launch は topic 契約を変えず、既存の icu_multi_patient.launch.py と同様の引数体系を保つ
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


def _parse_patients_csv(value: str) -> List[str]:
    return [item.strip() for item in value.split(',') if item.strip()]


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
    patients_csv = LaunchConfiguration('patients').perform(context)
    patient_ids = _parse_patients_csv(patients_csv)

    enable_alerts_str = LaunchConfiguration('enable_alerts').perform(context)
    enable_alerts = _parse_bool(enable_alerts_str)

    alerts_node_kind = LaunchConfiguration('alerts_node_kind').perform(context).strip().lower()
    lifecycle_autostart_str = LaunchConfiguration('lifecycle_autostart').perform(context)
    lifecycle_autostart = _parse_bool(lifecycle_autostart_str)

    enabled_rule_ids_str = LaunchConfiguration('enabled_rule_ids').perform(context).strip()
    enabled_rule_ids = [x.strip() for x in enabled_rule_ids_str.split(',') if x.strip()]

    rules_path = LaunchConfiguration('rules_path').perform(context).strip()
    if not rules_path:
        rules_path = _package_share_rules_path() or _source_tree_rules_path()
    rules_path = _resolve_rules_path(rules_path)

    flatline_history_size = int(LaunchConfiguration('flatline_history_size').perform(context))
    flatline_hr_epsilon = float(LaunchConfiguration('flatline_hr_epsilon').perform(context))
    flatline_spo2_epsilon = float(LaunchConfiguration('flatline_spo2_epsilon').perform(context))

    vitals_qos_depth = int(LaunchConfiguration('vitals_qos_depth').perform(context))
    vitals_qos_reliability = LaunchConfiguration('vitals_qos_reliability').perform(context).strip()
    vitals_qos_durability = LaunchConfiguration('vitals_qos_durability').perform(context).strip()

    alerts_qos_depth = int(LaunchConfiguration('alerts_qos_depth').perform(context))
    alerts_qos_reliability = LaunchConfiguration('alerts_qos_reliability').perform(context).strip()
    alerts_qos_durability = LaunchConfiguration('alerts_qos_durability').perform(context).strip()

    # [Day19] Closed-loop control params (mirrored from icu_multi_patient.launch.py)
    enable_closed_loop_str = LaunchConfiguration('enable_closed_loop').perform(context)
    enable_closed_loop = _parse_bool(enable_closed_loop_str)

    control_topic = LaunchConfiguration('control_topic').perform(context).strip()
    control_cooldown_sec = float(LaunchConfiguration('control_cooldown_sec').perform(context))
    control_no_data_after_sec = float(
        LaunchConfiguration('control_no_data_after_sec').perform(context)
    )
    control_low_spo2 = float(LaunchConfiguration('control_low_spo2').perform(context))
    control_critical_spo2 = float(LaunchConfiguration('control_critical_spo2').perform(context))
    enable_control_from_advisories_str = LaunchConfiguration(
        'enable_control_from_advisories'
    ).perform(context)
    enable_control_from_advisories = _parse_bool(enable_control_from_advisories_str)

    sigterm_timeout = LaunchConfiguration('sigterm_timeout')
    sigkill_timeout = LaunchConfiguration('sigkill_timeout')

    if not patient_ids:
        raise RuntimeError("Launch argument 'patients' resulted in empty patient list")

    actions = []

    actions.append(
        Node(
            package='medical_robot_sim',
            executable='icu_monitor',
            name='icu_monitor',
            output='screen',
            parameters=[
                {'patients': patient_ids},
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
        engine_params: list = [
            {'patients': patient_ids},
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

        engine_executable = 'rule_alert_engine'
        if alerts_node_kind == 'lifecycle':
            engine_executable = 'rule_alert_engine_lifecycle'
            engine_params.append({'lifecycle_autostart': bool(lifecycle_autostart)})

        actions.append(
            Node(
                package='medical_robot_sim',
                executable=engine_executable,
                name='rule_alert_engine',
                output='screen',
                parameters=engine_params,
                sigterm_timeout=sigterm_timeout,
                sigkill_timeout=sigkill_timeout,
            )
        )

    # [Day19] Closed-loop controller: one node per patient namespace
    if enable_closed_loop:
        for pid in patient_ids:
            controller_params: list = [
                {'patient_id': pid},
                {'vitals_topic': 'patient_vitals'},
                {'alerts_topic': 'alerts'},
                {'control_topic': control_topic},
                {'control_cooldown_sec': control_cooldown_sec},
                {'control_no_data_after_sec': control_no_data_after_sec},
                {'control_low_spo2': control_low_spo2},
                {'control_critical_spo2': control_critical_spo2},
                {'enable_control_from_advisories': enable_control_from_advisories},
                {'vitals_qos_depth': vitals_qos_depth},
                {'vitals_qos_reliability': vitals_qos_reliability},
                {'vitals_qos_durability': vitals_qos_durability},
                {'alerts_qos_depth': alerts_qos_depth},
                {'alerts_qos_reliability': alerts_qos_reliability},
                {'alerts_qos_durability': alerts_qos_durability},
            ]
            actions.append(
                Node(
                    package='medical_robot_sim',
                    executable='closed_loop_controller',
                    name='closed_loop_controller',
                    namespace=pid,
                    output='screen',
                    parameters=controller_params,
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
                'alerts_node_kind',
                default_value='classic',
                description='[Day9] Alerts engine node kind: classic | lifecycle',
            ),
            DeclareLaunchArgument(
                'lifecycle_autostart',
                default_value='true',
                description='[Day9] If true, lifecycle alerts engine auto configure->activate.',
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
                'rules_path',
                default_value=default_rules_path,
                description=(
                    '[Day7] Path to alert_rules.yaml. '
                    'Empty means no YAML is loaded (code defaults are used). '
                    'e.g. rules_path:=/path/to/config/alert_rules.yaml'
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
                'vitals_qos_depth',
                default_value='10',
                description='[Day8] QoS depth for /<patient>/patient_vitals (KEEP_LAST).',
            ),
            DeclareLaunchArgument(
                'vitals_qos_reliability',
                default_value='reliable',
                description='[Day8] QoS reliability for vitals: reliable | best_effort',
            ),
            DeclareLaunchArgument(
                'vitals_qos_durability',
                default_value='volatile',
                description='[Day8] QoS durability for vitals: volatile | transient_local',
            ),
            DeclareLaunchArgument(
                'alerts_qos_depth',
                default_value='10',
                description='[Day8] QoS depth for /<patient>/alerts (KEEP_LAST).',
            ),
            DeclareLaunchArgument(
                'alerts_qos_reliability',
                default_value='reliable',
                description='[Day8] QoS reliability for alerts: reliable | best_effort',
            ),
            DeclareLaunchArgument(
                'alerts_qos_durability',
                default_value='volatile',
                description='[Day8] QoS durability for alerts: volatile | transient_local',
            ),
            # [Day19] Closed-loop control arguments
            DeclareLaunchArgument(
                'enable_closed_loop',
                default_value='false',
                description=(
                    '[Day19] If true, start closed_loop_controller for each patient. '
                    'Requires enable_alerts:=true to generate alerts for decision input.'
                ),
            ),
            DeclareLaunchArgument(
                'control_topic',
                default_value='control_actions',
                description='[Day19] Relative topic name for control action output.',
            ),
            DeclareLaunchArgument(
                'control_cooldown_sec',
                default_value='5.0',
                description='[Day19] Minimum seconds between consecutive control publishes.',
            ),
            DeclareLaunchArgument(
                'control_no_data_after_sec',
                default_value='10.0',
                description=(
                    '[Day19] Seconds since last vitals after which HOLD is forced (NO_DATA guard).'
                ),
            ),
            DeclareLaunchArgument(
                'control_low_spo2',
                default_value='92.0',
                description='[Day19] SpO2 threshold [%] below which OXYGEN_BOOST is issued.',
            ),
            DeclareLaunchArgument(
                'control_critical_spo2',
                default_value='88.0',
                description='[Day19] SpO2 threshold [%] below which CALL_STAFF is issued.',
            ),
            DeclareLaunchArgument(
                'enable_control_from_advisories',
                default_value='false',
                description='[Day19] If true, advisory alerts also drive control decisions.',
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
