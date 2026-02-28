#!/usr/bin/env python3
"""
医療ロボット用バイタルセンサーシミュレーション（カスタムメッセージ版）.

型安全な医療データ通信を実現する.
"""

from __future__ import annotations

import math
import random
import sys

import rclpy
from medical_interfaces.msg import VitalSigns
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
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

        # シナリオ（デモ/再現性向け）
        # - '' / 'normal': 通常のランダム変動
        # - 'spo2_drop': 開始数秒後からSpO2を段階的に下げ、roc.spo2_drop を確実に誘発
        self.declare_parameter('scenario', '')
        self.scenario = str(self.get_parameter('scenario').value)

        # spo2_drop 用の開始遅延（秒）
        self._spo2_drop_delay_sec = 3.0
        self._spo2_drop_delay_samples = max(
            0, int(round(self._spo2_drop_delay_sec * publish_rate_hz))
        )

        # publishレートは後続の計算にも使う
        self.publish_rate_hz = publish_rate_hz

        # レートに応じてデータを送信
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_vital_data)

        # 測定カウンター
        self.measurement_count = 0

        self.get_logger().info('バイタルセンサーノード（カスタムメッセージ版）を起動しました')
        self.get_logger().info(f'患者ID: {self.patient_id}')
        self.get_logger().info(f'publish_rate_hz: {publish_rate_hz}')
        self.get_logger().info(f'scenario: {self.scenario!r}')

    def _compute_oxygen_saturation(self, base_oxygen_saturation: int) -> int:
        if self.scenario in {'spo2_drop'}:
            # まずは高めの値を維持し、その後に段階的に低下させる
            # 例（1Hzの場合）: 0-2秒は 98%、以降 2%ずつ低下し最終的に 88% で下げ止め
            start = 98
            drop_step = 2
            min_value = 88

            steps = max(0, self.measurement_count - self._spo2_drop_delay_samples)
            value = start - drop_step * steps
            value = max(min_value, value)
            return int(max(0, min(100, value)))

        # 通常シナリオ: 小さなランダム変動
        value = int(base_oxygen_saturation + random.randint(-2, 2))
        return int(max(0, min(100, value)))

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
        msg.oxygen_saturation = self._compute_oxygen_saturation(base_oxygen_saturation)

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

    node = None
    executor = None

    try:
        node = VitalSensorNode()

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
            return 0
        except (KeyboardInterrupt, ExternalShutdownException):
            return 0
        except Exception:
            return 0
        finally:
            try:
                if executor is not None and node is not None:
                    executor.remove_node(node)
            except Exception:
                pass

            try:
                if executor is not None:
                    executor.shutdown()
            except Exception:
                pass

            if node is not None:
                try:
                    if getattr(node, 'timer', None) is not None:
                        try:
                            node.timer.cancel()
                        except Exception:
                            pass
                        try:
                            node.destroy_timer(node.timer)
                        except Exception:
                            pass
                except Exception:
                    pass

                try:
                    node.destroy_node()
                except Exception:
                    pass
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
