#!/usr/bin/env python3
"""
医療ロボット用バイタルセンサーシミュレーション（カスタムメッセージ版）.

型安全な医療データ通信を実現する.
"""

from __future__ import annotations

import collections
import math
import random
import sys

import rclpy
from medical_interfaces.msg import VitalSigns
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from medical_robot_sim.fault_injection import compute_delay_ticks
from medical_robot_sim.fault_injection import should_drop
from medical_robot_sim.fault_injection import validate_fault_params
from medical_robot_sim.observability import format_event
from medical_robot_sim.observability import format_vitals_drop_event
from medical_robot_sim.observability import format_vitals_enqueue_delayed_event
from medical_robot_sim.qos_profiles import build_qos_profile


class VitalSensorNode(Node):
    """患者のバイタルサインを監視・送信するセンサーノード."""

    def __init__(self):
        super().__init__('vital_sensor')

        # Day11: Observability params
        self.declare_parameter('observability_verbose', False)
        self._observability_verbose = bool(self.get_parameter('observability_verbose').value)

        # Day8: QoS params（launch から上書き可能）
        self.declare_parameter('vitals_qos_depth', 10)
        self.declare_parameter('vitals_qos_reliability', 'reliable')
        self.declare_parameter('vitals_qos_durability', 'volatile')

        vitals_qos_depth = int(self.get_parameter('vitals_qos_depth').value)
        vitals_qos_reliability = str(self.get_parameter('vitals_qos_reliability').value)
        vitals_qos_durability = str(self.get_parameter('vitals_qos_durability').value)

        try:
            vitals_qos = build_qos_profile(
                depth=vitals_qos_depth,
                reliability=vitals_qos_reliability,
                durability=vitals_qos_durability,
            )
        except ValueError as exc:
            self.get_logger().error(f"[Day8] Invalid vitals QoS params: {exc}")
            raise SystemExit(2)

        # パブリッシャーの作成（カスタムメッセージ型を使用）
        self.publisher_ = self.create_publisher(
            VitalSigns,
            'patient_vitals',
            vitals_qos,
        )

        # 患者ID（launch から parameter で上書き可能）
        self.declare_parameter('patient_id', 'PATIENT_001')
        self.patient_id = str(self.get_parameter('patient_id').value)

        # publish レート [Hz]（launch から parameter で上書き可能）
        self.declare_parameter('publish_rate_hz', 1.0)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz <= 0.0:
            raise ValueError("Parameter 'publish_rate_hz' must be > 0")

        # Day10: Fault Injection params（既定は無効 = 後方互換）
        self.declare_parameter('vitals_fault_drop_rate', 0.0)
        self.declare_parameter('vitals_fault_delay_ms', 0)
        self.declare_parameter('vitals_fault_jitter_ms', 0)
        self.declare_parameter('vitals_fault_pause_after_sec', 0.0)
        self.declare_parameter('vitals_fault_pause_duration_sec', 0.0)
        self.declare_parameter('vitals_fault_stop_after_sec', 0.0)
        self.declare_parameter('vitals_fault_seed', 0)

        fault_drop_rate = float(self.get_parameter('vitals_fault_drop_rate').value)
        fault_delay_ms = int(self.get_parameter('vitals_fault_delay_ms').value)
        fault_jitter_ms = int(self.get_parameter('vitals_fault_jitter_ms').value)
        fault_pause_after_sec = float(
            self.get_parameter('vitals_fault_pause_after_sec').value
        )
        fault_pause_duration_sec = float(
            self.get_parameter('vitals_fault_pause_duration_sec').value
        )
        fault_stop_after_sec = float(self.get_parameter('vitals_fault_stop_after_sec').value)
        fault_seed = int(self.get_parameter('vitals_fault_seed').value)

        try:
            validate_fault_params(
                drop_rate=fault_drop_rate,
                delay_ms=fault_delay_ms,
                jitter_ms=fault_jitter_ms,
                pause_after_sec=fault_pause_after_sec,
                pause_duration_sec=fault_pause_duration_sec,
                stop_after_sec=fault_stop_after_sec,
                publish_rate_hz=publish_rate_hz,
            )
        except ValueError as exc:
            self.get_logger().error(f"[Day10] Invalid fault params: {exc}")
            raise SystemExit(2)

        self._fault_drop_rate = fault_drop_rate
        self._fault_delay_ms = fault_delay_ms
        self._fault_jitter_ms = fault_jitter_ms
        self._fault_pause_after_sec = fault_pause_after_sec
        self._fault_pause_duration_sec = fault_pause_duration_sec
        self._fault_stop_after_sec = fault_stop_after_sec
        self._fault_rng = random.Random(fault_seed)

        # due_tick（サンプル番号）で publish を遅延させるキュー（順序維持のため due_tick は単調増加にする）
        self._delayed_queue: collections.deque[tuple[int, VitalSigns]] = collections.deque()
        self._last_due_tick = 0
        self._is_paused = False

        # シナリオ（デモ/再現性向け）
        # - '' / 'normal': 通常のランダム変動
        # - 'spo2_drop': 開始数秒後からSpO2を段階的に下げ、roc.spo2_drop を確実に誘発
        # - 'flatline': HR と SpO2 を固定値で連投し、flatline.hr / flatline.spo2 を確実に誘発
        self.declare_parameter('scenario', '')
        self.scenario = str(self.get_parameter('scenario').value)

        # spo2_drop 用の開始遅延（秒）
        self._spo2_drop_delay_sec = 3.0
        self._spo2_drop_delay_samples = max(
            0, int(round(self._spo2_drop_delay_sec * publish_rate_hz))
        )

        # flatline シナリオで固定送信する値
        self.declare_parameter('flatline_hr_value', 72)
        self.declare_parameter('flatline_spo2_value', 98)
        self._flatline_hr_value = int(self.get_parameter('flatline_hr_value').value)
        self._flatline_spo2_value = int(self.get_parameter('flatline_spo2_value').value)

        # publishレートは後続の計算にも使う
        self.publish_rate_hz = publish_rate_hz

        # レートに応じてデータを送信
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_vital_data)

        # 測定カウンター（= サンプル tick）
        self.measurement_count = 0

        self.get_logger().info('バイタルセンサーノード（カスタムメッセージ版）を起動しました')
        self.get_logger().info(f'患者ID: {self.patient_id}')
        self.get_logger().info(f'publish_rate_hz: {publish_rate_hz}')
        self.get_logger().info(f'scenario: {self.scenario!r}')
        self.get_logger().info(
            '[Day8] vitals_qos='
            f"KEEP_LAST depth={vitals_qos_depth}, reliability={vitals_qos_reliability}, "
            f"durability={vitals_qos_durability}"
        )
        self.get_logger().info(
            '[Day10] fault='
            f"drop_rate={fault_drop_rate}, delay_ms={fault_delay_ms}, "
            f"jitter_ms={fault_jitter_ms}, "
            f"pause_after_sec={fault_pause_after_sec}, "
            f"pause_duration_sec={fault_pause_duration_sec}, "
            f"stop_after_sec={fault_stop_after_sec}, seed={fault_seed}"
        )

        # Day11: Emit fault configuration as a machine-grepable event.
        self.get_logger().info(
            format_event(
                'vitals.fault_config',
                delay_ms=fault_delay_ms,
                drop_rate=fault_drop_rate,
                jitter_ms=fault_jitter_ms,
                node=self.get_name(),
                ns=self.get_namespace(),
                pause_after_sec=fault_pause_after_sec,
                pause_duration_sec=fault_pause_duration_sec,
                publish_rate_hz=publish_rate_hz,
                seed=fault_seed,
                stop_after_sec=fault_stop_after_sec,
                verbose=self._observability_verbose,
            )
        )

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

        if self.scenario == 'flatline':
            # 固定値を返す（flatline.spo2 を確実に誘発）
            return int(max(0, min(100, self._flatline_spo2_value)))

        # 通常シナリオ: 小さなランダム変動
        value = int(base_oxygen_saturation + random.randint(-2, 2))
        return int(max(0, min(100, value)))

    def _compute_heart_rate(self, base_heart_rate: int, time_factor: float) -> int:
        """心拍数を計算する（シナリオに応じて変動パターンを切り替える）."""
        if self.scenario == 'flatline':
            # 固定値を返す（flatline.hr を確実に誘発）
            return int(self._flatline_hr_value)

        # 通常シナリオ: ランダム変動 + サイン波
        value = base_heart_rate + random.randint(-8, 12) + 2 * math.sin(time_factor)
        return int(value)

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
        msg.heart_rate = self._compute_heart_rate(base_heart_rate, time_factor)
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

        tick = int(self.measurement_count)
        elapsed_sec = tick / float(self.publish_rate_hz)

        # Day10: stop injection
        if self._fault_stop_after_sec > 0.0 and elapsed_sec >= self._fault_stop_after_sec:
            self.get_logger().warn(
                f"[Day10] vitals_fault_stop_after_sec reached ({self._fault_stop_after_sec}); "
                'shutting down...'
            )

            self.get_logger().info(
                format_event(
                    'vitals.stop_trigger',
                    elapsed_sec=elapsed_sec,
                    node=self.get_name(),
                    ns=self.get_namespace(),
                    tick=tick,
                )
            )
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            return

        # Day10: pause injection
        paused = False
        if self._fault_pause_after_sec > 0.0 and self._fault_pause_duration_sec > 0.0:
            start = self._fault_pause_after_sec
            end = self._fault_pause_after_sec + self._fault_pause_duration_sec
            paused = start <= elapsed_sec < end

        if paused and not self._is_paused:
            self.get_logger().warn('[Day10] entering pause window (publish halted)')

            self.get_logger().info(
                format_event(
                    'vitals.pause_enter',
                    elapsed_sec=elapsed_sec,
                    node=self.get_name(),
                    ns=self.get_namespace(),
                    tick=tick,
                )
            )
        if (not paused) and self._is_paused:
            self.get_logger().warn('[Day10] leaving pause window (publish resumed)')

            self.get_logger().info(
                format_event(
                    'vitals.pause_exit',
                    elapsed_sec=elapsed_sec,
                    node=self.get_name(),
                    ns=self.get_namespace(),
                    tick=tick,
                )
            )
        self._is_paused = bool(paused)

        if self._is_paused:
            # publish/flush せずに tick だけ進める
            self.measurement_count += 1
            return

        # Day10: drop injection
        dropped = should_drop(self._fault_drop_rate, self._fault_rng)
        if dropped and self._observability_verbose:
            try:
                self.get_logger().info(
                    format_vitals_drop_event(
                        node=str(self.get_name()),
                        ns=str(self.get_namespace()),
                        tick=int(tick),
                        elapsed_sec=float(elapsed_sec),
                        drop_rate=float(self._fault_drop_rate),
                    )
                )
            except Exception:
                pass

        if not dropped:
            msg = self.generate_realistic_vitals()
            delay_ticks = compute_delay_ticks(
                self._fault_delay_ms,
                self._fault_jitter_ms,
                self.publish_rate_hz,
                self._fault_rng,
            )
            due_tick = tick + int(delay_ticks)

            clamped = False

            # 順序を守る（due_tick が逆転しないように丸める）
            if due_tick < self._last_due_tick:
                due_tick = self._last_due_tick
                clamped = True
            self._last_due_tick = due_tick

            self._delayed_queue.append((due_tick, msg))

            if self._observability_verbose and int(delay_ticks) > 0:
                try:
                    self.get_logger().info(
                        format_vitals_enqueue_delayed_event(
                            node=str(self.get_name()),
                            ns=str(self.get_namespace()),
                            tick=int(tick),
                            elapsed_sec=float(elapsed_sec),
                            delay_ms=int(self._fault_delay_ms),
                            jitter_ms=int(self._fault_jitter_ms),
                            delay_ticks=int(delay_ticks),
                            due_tick=int(due_tick),
                            queue_len=int(len(self._delayed_queue)),
                            clamped=bool(clamped),
                        )
                    )
                except Exception:
                    pass

        # due_tick 到達済みのものを publish
        while self._delayed_queue and self._delayed_queue[0][0] <= tick:
            _, out_msg = self._delayed_queue.popleft()
            self.publisher_.publish(out_msg)

            # ログ出力
            self.get_logger().info(
                f'送信 #{out_msg.measurement_id}: '
                f'心拍数={out_msg.heart_rate}bpm, '
                f'血圧={out_msg.blood_pressure_systolic}/{out_msg.blood_pressure_diastolic}mmHg, '
                f'体温={out_msg.body_temperature:.1f}°C, '
                f'SpO2={out_msg.oxygen_saturation}%'
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
