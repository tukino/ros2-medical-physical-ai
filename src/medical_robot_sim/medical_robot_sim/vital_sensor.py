#!/usr/bin/env python3
"""
医療ロボット用バイタルセンサーシミュレーション（カスタムメッセージ版）.

型安全な医療データ通信を実現する.
"""

import math
import random

import rclpy
from medical_interfaces.msg import VitalSigns
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class VitalSensorNode(Node):
    """患者のバイタルサインを監視・送信するセンサーノード."""

    def __init__(self):
        super().__init__('vital_sensor')

        # パブリッシャーの作成（カスタムメッセージ型を使用）
        self.publisher_ = self.create_publisher(
            VitalSigns,
            'patient_vitals',
            10
        )

        # 患者ID（launch から parameter で上書き可能）
        self.declare_parameter('patient_id', 'PATIENT_001')
        self.patient_id = str(self.get_parameter('patient_id').value)

        # publish レート [Hz]（launch から parameter で上書き可能）
        self.declare_parameter('publish_rate_hz', 1.0)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz <= 0.0:
            raise ValueError("Parameter 'publish_rate_hz' must be > 0")

        # レートに応じてデータを送信
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_vital_data)

        # 測定カウンター
        self.measurement_count = 0

        self.get_logger().info('バイタルセンサーノード（カスタムメッセージ版）を起動しました')
        self.get_logger().info(f'患者ID: {self.patient_id}')
        self.get_logger().info(f'publish_rate_hz: {publish_rate_hz}')

    def generate_realistic_vitals(self):
        """現実的なバイタルサインを生成."""
        # 基準値
        base_heart_rate = 72
        base_blood_pressure_sys = 120
        base_blood_pressure_dia = 80
        base_temperature = 36.5
        base_oxygen_saturation = 98

        # 時間経過による自然な変動
        time_factor = self.measurement_count * 0.1

        # VitalSignsメッセージの作成
        msg = VitalSigns()

        # 患者情報
        msg.patient_id = self.patient_id
        msg.measurement_id = self.measurement_count

        # バイタルサイン測定値（型安全な代入）
        # 正常：
        msg.heart_rate = int(base_heart_rate + random.randint(-8, 12) + 2 * math.sin(time_factor))
        # テスト用（意図的エラー）：
        # msg.heart_rate = "invalid_string"  # 文字列を整数フィールドに代入
        msg.blood_pressure_systolic = int(base_blood_pressure_sys + random.randint(-10, 15))
        msg.blood_pressure_diastolic = int(base_blood_pressure_dia + random.randint(-8, 10))
        msg.body_temperature = float(base_temperature + random.uniform(-0.5, 0.8))
        msg.oxygen_saturation = int(base_oxygen_saturation + random.randint(-2, 2))

        # ステータス
        msg.status = "monitoring"

        return msg

    def publish_vital_data(self):
        """バイタルサインデータをパブリッシュ."""
        msg = self.generate_realistic_vitals()

        self.publisher_.publish(msg)

        # ログ出力
        self.get_logger().info(
            f'送信 #{self.measurement_count}: '
            f'心拍数={msg.heart_rate}bpm, '
            f'血圧={msg.blood_pressure_systolic}/{msg.blood_pressure_diastolic}mmHg, '
            f'体温={msg.body_temperature:.1f}°C, '
            f'SpO2={msg.oxygen_saturation}%'
        )

        self.measurement_count += 1


def main(args=None):
    rclpy.init(args=args)

    node = VitalSensorNode()

    try:
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
