#!/usr/bin/env python3
"""
ICU monitor node.

要件:
- parameter:
    - patients (string list)
    - vitals_topic (string, default 'patient_vitals')
- 各患者 pid について '/{pid}/{vitals_topic}' を subscribe
- 最新値を pid ごとに保持し、1秒ごとに一覧表示
- しきい値ベースの優先度アラートを pid ごとに計算
    - RED: spo2 < 90
    - ORANGE: heart_rate > 120
    - YELLOW: systolic_bp > 160
"""

from __future__ import annotations

import os
import signal
import sys
import shutil
import threading
from typing import Dict
from typing import List
from typing import Optional

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.parameter import Parameter

from medical_interfaces.msg import VitalSigns


class IcuMonitorNode(Node):
    def __init__(self):
        super().__init__('icu_monitor')

        # NOTE: 空の list は型推論が曖昧になり、launch からの string array 上書きが
        #       反映されずにデフォルト扱いになることがあるため、型を明示して宣言する
        self.declare_parameter('patients', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('vitals_topic', 'patient_vitals')

        patient_ids = list(
            self.get_parameter('patients').get_parameter_value().string_array_value
        )
        vitals_topic = str(
            self.get_parameter('vitals_topic').get_parameter_value().string_value
        )

        patient_ids = [str(pid).strip().lstrip('/') for pid in patient_ids if str(pid).strip()]
        vitals_topic = vitals_topic.strip().lstrip('/')

        # 重複除去（順序維持）
        unique_patient_ids: List[str] = []
        seen = set()
        for pid in patient_ids:
            if pid not in seen:
                unique_patient_ids.append(pid)
                seen.add(pid)
        patient_ids = unique_patient_ids

        if not patient_ids:
            raise RuntimeError("Parameter 'patients' is empty")
        if not vitals_topic:
            raise RuntimeError("Parameter 'vitals_topic' is empty")

        self._patient_ids = patient_ids
        self._vitals_topic = vitals_topic

        self._subscriptions = []
        self._message_counts: Dict[str, int] = {pid: 0 for pid in patient_ids}
        self._latest: Dict[str, VitalSigns] = {}
        self._latest_stamp_ns: Dict[str, int] = {}

        for pid in patient_ids:
            topic = f'/{pid}/{vitals_topic}'
            sub = self.create_subscription(
                VitalSigns,
                topic,
                lambda msg, pid=pid: self._on_vitals(pid, msg),
                10,
            )
            self._subscriptions.append(sub)

        self.get_logger().info('ICU monitor を起動しました')
        self.get_logger().info(f"patients={patient_ids}")
        self.get_logger().info(f"vitals_topic='{vitals_topic}'")

        # 1秒ごとにサマリー表示
        self._summary_timer = self.create_timer(1.0, self._refresh_dashboard)

    def _on_vitals(self, patient_ns: str, msg: VitalSigns) -> None:
        self._message_counts[patient_ns] = self._message_counts.get(patient_ns, 0) + 1
        self._latest[patient_ns] = msg
        self._latest_stamp_ns[patient_ns] = self.get_clock().now().nanoseconds

    @staticmethod
    def _calc_alert(msg: VitalSigns) -> str:
        if msg.oxygen_saturation < 90:
            return 'RED'
        if msg.heart_rate > 120:
            return 'ORANGE'
        if msg.blood_pressure_systolic > 160:
            return 'YELLOW'
        return 'OK'

    def _age_seconds(self, pid: str) -> Optional[float]:
        stamp_ns = self._latest_stamp_ns.get(pid)
        if stamp_ns is None:
            return None
        now_ns = self.get_clock().now().nanoseconds
        return max(0.0, (now_ns - stamp_ns) / 1e9)

    def _calc_data_state(self, pid: str) -> str:
        """Return data state based on last_seen age."""
        # Thresholds:
        # - STALE: > 3s
        # - NO DATA: > 10s (or never seen)
        age_s = self._age_seconds(pid)
        if age_s is None or age_s > 10.0:
            return 'NO DATA'
        if age_s > 3.0:
            return 'STALE'
        return 'FRESH'

    def _age_seconds_str(self, pid: str) -> str:
        age_s = self._age_seconds(pid)
        if age_s is None:
            return 'N/A'
        return f"{age_s:.0f}s"

    def _render_row(self, pid: str, msg: Optional[VitalSigns]) -> str:
        data_state = self._calc_data_state(pid)

        if msg is None:
            # 未受信の患者は NO DATA 表示
            return f"{pid:<10} | {'NO DATA':<7} | --- | ---- | ---/--- | ---- | {'N/A':>3}"

        # STALE/NO DATA は vitals 由来のアラートより優先
        if data_state == 'NO DATA':
            alert = 'NO DATA'
        elif data_state == 'STALE':
            alert = 'STALE'
        else:
            alert = self._calc_alert(msg)

        age_s = self._age_seconds_str(pid)
        hr = int(msg.heart_rate)
        spo2 = int(msg.oxygen_saturation)
        sys_bp = int(msg.blood_pressure_systolic)
        dia_bp = int(msg.blood_pressure_diastolic)
        temp = float(msg.body_temperature)

        return (
            f"{pid:<10} | {alert:<7} | {hr:>3} | {spo2:>4} | {sys_bp:>3}/{dia_bp:<3} | "
            f"{temp:>4.1f} | {age_s:>3}"
        )

    def _render(self) -> str:
        # 端末幅に応じて横罫の長さを調整（失敗したら固定幅）
        term_width = shutil.get_terminal_size((100, 20)).columns
        line = '-' * max(40, min(term_width, 140))

        title = f"ICU DASHBOARD  (patients={len(self._patient_ids)})"
        header = "pid        | alert   |  HR | SpO2 | BP      | Temp | age"

        rows = [self._render_row(pid, self._latest.get(pid)) for pid in self._patient_ids]
        return "\n".join([title, line, header, line, *rows, line])

    @staticmethod
    def _stdout_is_tty() -> bool:
        try:
            return sys.stdout.isatty()
        except Exception:
            return False

    def _clear_screen(self) -> None:
        if not self._stdout_is_tty():
            return

        # 要望に合わせて、可能なら clear コマンドを優先
        try:
            ret = os.system('clear')
            if ret == 0:
                return
        except Exception:
            pass

        # フォールバック: ANSI でクリア
        sys.stdout.write('\x1b[2J\x1b[H')

    def _refresh_dashboard(self) -> None:
        rendered = self._render()

        # 可能なら画面クリアして上書き
        if self._stdout_is_tty():
            self._clear_screen()
            sys.stdout.write(rendered + '\n')
            sys.stdout.flush()
            return

        # tty でない（launch 経由など）場合はブロック表示（クリアはしない）
        sys.stdout.write(rendered + '\n')
        sys.stdout.flush()


def main(args=None):
    shutdown_requested = threading.Event()

    def _request_shutdown(_signum=None, _frame=None):
        shutdown_requested.set()

    original_sigint_handler = signal.getsignal(signal.SIGINT)
    original_sigterm_handler = signal.getsignal(signal.SIGTERM)

    rclpy.init(args=args)

    # NOTE: signal handler では destroy_node() / rclpy.shutdown() を呼ばない
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    node: Optional[IcuMonitorNode] = None
    executor = None

    try:
        node = IcuMonitorNode()

        from rclpy.executors import ExternalShutdownException
        from rclpy.executors import SingleThreadedExecutor

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while rclpy.ok() and not shutdown_requested.is_set():
                executor.spin_once(timeout_sec=0.1)
            return 0
        except (KeyboardInterrupt, ExternalShutdownException):
            shutdown_requested.set()
            return 0
        except Exception as exc:
            try:
                node.get_logger().error(f'Unhandled exception: {exc!r}')
            except Exception:
                pass
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
                    if getattr(node, '_summary_timer', None) is not None:
                        try:
                            node._summary_timer.cancel()
                        except Exception:
                            pass
                        try:
                            node.destroy_timer(node._summary_timer)
                        except (ValueError, RCLError):
                            pass
                except Exception:
                    pass

                try:
                    for sub in list(getattr(node, '_subscriptions', [])):
                        try:
                            node.destroy_subscription(sub)
                        except (ValueError, RCLError):
                            pass
                except Exception:
                    pass

                try:
                    node.destroy_node()
                except (ValueError, RCLError) as exc:
                    try:
                        node.get_logger().warn(
                            f'destroy_node() failed during shutdown: {exc!r}'
                        )
                    except Exception:
                        pass
                except Exception:
                    pass
    except Exception:
        # 起動/終了時の例外は exit code 0 で握りつぶす
        return 0
    finally:
        try:
            signal.signal(signal.SIGINT, original_sigint_handler)
            signal.signal(signal.SIGTERM, original_sigterm_handler)
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
