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

import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _parse_patients_csv(value: str) -> List[str]:
    # CSV を split し、空要素と前後空白を除去
    patient_ids = [item.strip() for item in value.split(',') if item.strip()]
    return patient_ids


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

    scenario = LaunchConfiguration('scenario').perform(context).strip()

    # Day10: Fault Injection params（vital_sensor only）
    vitals_fault_drop_rate = float(
        LaunchConfiguration('vitals_fault_drop_rate').perform(context)
    )
    vitals_fault_delay_ms = int(LaunchConfiguration('vitals_fault_delay_ms').perform(context))
    vitals_fault_jitter_ms = int(
        LaunchConfiguration('vitals_fault_jitter_ms').perform(context)
    )
    vitals_fault_pause_after_sec = float(
        LaunchConfiguration('vitals_fault_pause_after_sec').perform(context)
    )
    vitals_fault_pause_duration_sec = float(
        LaunchConfiguration('vitals_fault_pause_duration_sec').perform(context)
    )
    vitals_fault_stop_after_sec = float(
        LaunchConfiguration('vitals_fault_stop_after_sec').perform(context)
    )

    observability_verbose_str = LaunchConfiguration('observability_verbose').perform(context)
    observability_verbose = _parse_bool(observability_verbose_str)
    vitals_fault_seed = int(LaunchConfiguration('vitals_fault_seed').perform(context))

    enabled_rule_ids_str = LaunchConfiguration('enabled_rule_ids').perform(context).strip()
    # CSV → list（空文字はそのまま渡すと空配列扱い）
    enabled_rule_ids = [x.strip() for x in enabled_rule_ids_str.split(',') if x.strip()]

    # Day7: 外部設定ファイルのパス（空文字 = YAML 読み込みなし）
    rules_path = LaunchConfiguration('rules_path').perform(context).strip()
    if not rules_path:
        rules_path = _package_share_rules_path() or _source_tree_rules_path()
    rules_path = _resolve_rules_path(rules_path)

    flatline_history_size = int(LaunchConfiguration('flatline_history_size').perform(context))
    flatline_hr_epsilon = float(LaunchConfiguration('flatline_hr_epsilon').perform(context))
    flatline_spo2_epsilon = float(LaunchConfiguration('flatline_spo2_epsilon').perform(context))

    # Day8: QoS params
    vitals_qos_depth = int(LaunchConfiguration('vitals_qos_depth').perform(context))
    vitals_qos_reliability = LaunchConfiguration('vitals_qos_reliability').perform(context).strip()
    vitals_qos_durability = LaunchConfiguration('vitals_qos_durability').perform(context).strip()

    alerts_qos_depth = int(LaunchConfiguration('alerts_qos_depth').perform(context))
    alerts_qos_reliability = LaunchConfiguration('alerts_qos_reliability').perform(context).strip()
    alerts_qos_durability = LaunchConfiguration('alerts_qos_durability').perform(context).strip()

    sigterm_timeout = LaunchConfiguration('sigterm_timeout')
    sigkill_timeout = LaunchConfiguration('sigkill_timeout')

    if not patient_ids:
        raise RuntimeError("Launch argument 'patients' resulted in empty patient list")

    actions = []

    # 患者ごとに vital_sensor を namespace 付きで起動
    for pid in patient_ids:
        sensor_params: list = [
            {'patient_id': pid},
            {'vitals_qos_depth': vitals_qos_depth},
            {'vitals_qos_reliability': vitals_qos_reliability},
            {'vitals_qos_durability': vitals_qos_durability},
            {'observability_verbose': bool(observability_verbose)},
            {'vitals_fault_drop_rate': vitals_fault_drop_rate},
            {'vitals_fault_delay_ms': vitals_fault_delay_ms},
            {'vitals_fault_jitter_ms': vitals_fault_jitter_ms},
            {'vitals_fault_pause_after_sec': vitals_fault_pause_after_sec},
            {'vitals_fault_pause_duration_sec': vitals_fault_pause_duration_sec},
            {'vitals_fault_stop_after_sec': vitals_fault_stop_after_sec},
            {'vitals_fault_seed': vitals_fault_seed},
        ]
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
                'scenario',
                default_value='',
                description=(
                    "vital_sensor scenario: '' (normal), 'spo2_drop', 'flatline'. "
                    "Applied to all patients."
                ),
            ),

            DeclareLaunchArgument(
                'observability_verbose',
                default_value='false',
                description=(
                    '[Day11] If true, vital_sensor emits verbose-only events '
                    '(vitals.drop, vitals.enqueue_delayed).'
                ),
            ),

            # Day10: Fault Injection (vital_sensor)
            DeclareLaunchArgument(
                'vitals_fault_drop_rate',
                default_value='0.0',
                description='[Day10] Drop rate for vitals publish: 0.0-1.0 (vital_sensor only).',
            ),
            DeclareLaunchArgument(
                'vitals_fault_delay_ms',
                default_value='0',
                description='[Day10] Fixed delay for vitals publish [ms] (vital_sensor only).',
            ),
            DeclareLaunchArgument(
                'vitals_fault_jitter_ms',
                default_value='0',
                description=(
                    '[Day10] Random delay jitter added to delay_ms [ms] '
                    '(vital_sensor only).'
                ),
            ),
            DeclareLaunchArgument(
                'vitals_fault_pause_after_sec',
                default_value='0.0',
                description='[Day10] Pause vitals publish after N seconds (vital_sensor only).',
            ),
            DeclareLaunchArgument(
                'vitals_fault_pause_duration_sec',
                default_value='0.0',
                description='[Day10] Pause duration in seconds (vital_sensor only).',
            ),
            DeclareLaunchArgument(
                'vitals_fault_stop_after_sec',
                default_value='0.0',
                description='[Day10] Stop vital_sensor after N seconds (vital_sensor only).',
            ),
            DeclareLaunchArgument(
                'vitals_fault_seed',
                default_value='0',
                description='[Day10] RNG seed for fault injection (vital_sensor only).',
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

            # Day8: QoS
            DeclareLaunchArgument(
                'vitals_qos_depth',
                default_value='10',
                description='[Day8] QoS depth for /<patient>/patient_vitals (KEEP_LAST).',
            ),
            DeclareLaunchArgument(
                'vitals_qos_reliability',
                default_value='reliable',
                description="[Day8] QoS reliability for vitals: reliable | best_effort",
            ),
            DeclareLaunchArgument(
                'vitals_qos_durability',
                default_value='volatile',
                description="[Day8] QoS durability for vitals: volatile | transient_local",
            ),
            DeclareLaunchArgument(
                'alerts_qos_depth',
                default_value='10',
                description='[Day8] QoS depth for /<patient>/alerts (KEEP_LAST).',
            ),
            DeclareLaunchArgument(
                'alerts_qos_reliability',
                default_value='reliable',
                description="[Day8] QoS reliability for alerts: reliable | best_effort",
            ),
            DeclareLaunchArgument(
                'alerts_qos_durability',
                default_value='volatile',
                description="[Day8] QoS durability for alerts: volatile | transient_local",
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
