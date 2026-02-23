#!/usr/bin/env python3
"""
病院監視システム統合起動ファイル.

複数の医療ノードを一括起動し、実用的な医療システムを構築する.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """医療ロボットシステムの launch 設定."""
    return LaunchDescription([
        # バイタルセンサーノード
        Node(
            package='medical_robot_sim',
            executable='vital_sensor',
            name='vital_sensor_patient_001',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),

        # 医療監視ノード
        Node(
            package='medical_robot_sim',
            executable='medical_monitor',
            name='medical_monitor_central',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
