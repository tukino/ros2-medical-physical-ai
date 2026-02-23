#!/usr/bin/env python3
"""
医療データ監視・警告システム（カスタムメッセージ版）.

型安全な医療データ分析と異常検知を行う.
"""

import rclpy
from rclpy.node import Node
from medical_interfaces.msg import VitalSigns


class MedicalMonitorNode(Node):
    """患者のバイタルサインを監視し、異常を検出するノード."""

    def __init__(self):
        super().__init__('medical_monitor')

        # サブスクライバーの作成（カスタムメッセージ型を使用）
        self.subscription = self.create_subscription(
            VitalSigns,
            'patient_vitals',
            self.analyze_vital_data,
            10
        )

        # 正常値の範囲を定義（医療基準に基づく）
        self.normal_ranges = {
            "heart_rate": {"min": 60, "max": 100},
            "blood_pressure_systolic": {"min": 90, "max": 140},
            "blood_pressure_diastolic": {"min": 60, "max": 90},
            "body_temperature": {"min": 36.0, "max": 37.5},
            "oxygen_saturation": {"min": 95, "max": 100}
        }

        # 統計情報
        self.total_measurements = 0
        self.alert_count = 0

        self.get_logger().info('医療データ監視ノード（カスタムメッセージ版）を起動しました')
        self.get_logger().info('バイタルサイン監視を開始します...')

    def analyze_vital_data(self, msg):
        """受信したバイタルデータを分析し、異常を検出."""
        self.total_measurements += 1

        patient_id = msg.patient_id
        measurement_id = msg.measurement_id

        # 各バイタルサインの異常検出
        alerts = []

        # 心拍数チェック（直接属性アクセス - JSON不要）
        hr = msg.heart_rate
        hr_range = self.normal_ranges["heart_rate"]
        if hr < hr_range["min"]:
            alerts.append(
                f"⚠️ 徐脈: 心拍数 {hr}bpm "
                f"(正常範囲: {hr_range['min']}-{hr_range['max']}bpm)"
            )
        elif hr > hr_range["max"]:
            alerts.append(
                f"⚠️ 頻脈: 心拍数 {hr}bpm "
                f"(正常範囲: {hr_range['min']}-{hr_range['max']}bpm)"
            )

        # 血圧チェック（収縮期）
        sys_bp = msg.blood_pressure_systolic
        sys_range = self.normal_ranges["blood_pressure_systolic"]
        if sys_bp > sys_range["max"]:
            alerts.append(
                f"⚠️ 高血圧: 収縮期 {sys_bp}mmHg "
                f"(正常範囲: {sys_range['min']}-{sys_range['max']}mmHg)"
            )
        elif sys_bp < sys_range["min"]:
            alerts.append(
                f"🚨 低血圧: 収縮期 {sys_bp}mmHg "
                f"(正常範囲: {sys_range['min']}-{sys_range['max']}mmHg)"
            )

        # 体温チェック
        temp = msg.body_temperature
        temp_range = self.normal_ranges["body_temperature"]
        if temp > temp_range["max"]:
            alerts.append(
                f"⚠️ 発熱: 体温 {temp:.1f}°C "
                f"(正常範囲: {temp_range['min']}-{temp_range['max']}°C)"
            )
        elif temp < temp_range["min"]:
            alerts.append(
                f"🚨 低体温: 体温 {temp:.1f}°C "
                f"(正常範囲: {temp_range['min']}-{temp_range['max']}°C)"
            )

        # 酸素飽和度チェック
        spo2 = msg.oxygen_saturation
        spo2_range = self.normal_ranges["oxygen_saturation"]
        if spo2 < spo2_range["min"]:
            severity = "🚨 緊急" if spo2 < 90 else "⚠️ 注意"
            alerts.append(
                f"{severity} 低酸素: SpO2 {spo2}% "
                f"(正常範囲: {spo2_range['min']}-{spo2_range['max']}%)"
            )

        # 結果のログ出力
        if alerts:
            self.alert_count += len(alerts)
            self.get_logger().warn(
                f'患者 {patient_id} - 測定 #{measurement_id}: '
                f'{len(alerts)}件の異常を検出'
            )
            for alert in alerts:
                self.get_logger().warn(f'  {alert}')
        else:
            self.get_logger().info(
                f'患者 {patient_id} - 測定 #{measurement_id}: '
                f'全バイタルサイン正常 '
                f'(HR={hr}, BP={sys_bp}/{msg.blood_pressure_diastolic}, '
                f'Temp={temp:.1f}°C, SpO2={spo2}%)'
            )


def main(args=None):
    rclpy.init(args=args)

    node = MedicalMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'医療監視ノードを停止します '
            f'(総測定数: {node.total_measurements}, アラート数: {node.alert_count})'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
