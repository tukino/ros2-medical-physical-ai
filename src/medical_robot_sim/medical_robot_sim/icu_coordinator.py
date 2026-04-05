#!/usr/bin/env python3
"""ICU coordinator node (Day17).

Orchestrates multi-node behavior by controlling the lifecycle state of
`rule_alert_engine_lifecycle` (node name: /rule_alert_engine).

- Watches /patient_XX/patient_vitals readiness
- Calls lifecycle transitions (configure/activate/deactivate)
- Emits Day11-style one-line event logs: event=coord.*
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
from typing import List
from typing import Optional

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState

from medical_interfaces.msg import VitalSigns

from medical_robot_sim.coordination_policy import all_patients_no_data
from medical_robot_sim.coordination_policy import lifecycle_action_to_reach_active
from medical_robot_sim.coordination_policy import lifecycle_action_to_reach_inactive
from medical_robot_sim.coordination_policy import is_ready
from medical_robot_sim.observability import format_event
from medical_robot_sim.qos_profiles import build_qos_profile


@dataclass(frozen=True)
class _LifecycleTarget:
    node_name: str

    @property
    def get_state_service(self) -> str:
        return f'/{self.node_name}/get_state'

    @property
    def change_state_service(self) -> str:
        return f'/{self.node_name}/change_state'


class IcuCoordinatorNode(Node):
    def __init__(self):
        super().__init__('icu_coordinator')

        # Parameters (patients & topics)
        self.declare_parameter('patients', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('vitals_topic', 'patient_vitals')

        # Day17 params
        self.declare_parameter('min_messages_per_patient', 1)
        self.declare_parameter('ready_timeout_sec', 10.0)
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter('stale_after_sec', 3.0)
        self.declare_parameter('no_data_after_sec', 10.0)
        self.declare_parameter('deactivate_on_no_data', True)

        # Day8: QoS params (vitals subscribe)
        self.declare_parameter('vitals_qos_depth', 10)
        self.declare_parameter('vitals_qos_reliability', 'reliable')
        self.declare_parameter('vitals_qos_durability', 'volatile')

        patient_ids = list(
            self.get_parameter('patients').get_parameter_value().string_array_value
        )
        vitals_topic = str(self.get_parameter('vitals_topic').value)

        patient_ids = [str(pid).strip().lstrip('/') for pid in patient_ids if str(pid).strip()]
        vitals_topic = vitals_topic.strip().lstrip('/')

        # Remove duplicates while keeping order
        unique: List[str] = []
        seen = set()
        for pid in patient_ids:
            if pid not in seen:
                unique.append(pid)
                seen.add(pid)
        patient_ids = unique

        if not patient_ids:
            raise RuntimeError("Parameter 'patients' is empty")
        if not vitals_topic:
            raise RuntimeError("Parameter 'vitals_topic' is empty")

        self._patient_ids = patient_ids
        self._vitals_topic = vitals_topic

        self._min_messages_per_patient = int(self.get_parameter('min_messages_per_patient').value)
        self._ready_timeout_sec = float(self.get_parameter('ready_timeout_sec').value)
        self._check_period_sec = float(self.get_parameter('check_period_sec').value)
        self._stale_after_sec = float(self.get_parameter('stale_after_sec').value)
        self._no_data_after_sec = float(self.get_parameter('no_data_after_sec').value)
        self._deactivate_on_no_data = bool(self.get_parameter('deactivate_on_no_data').value)

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

        self._counts: Dict[str, int] = {pid: 0 for pid in patient_ids}
        self._last_seen_ns: Dict[str, Optional[int]] = {pid: None for pid in patient_ids}

        self._lifecycle = _LifecycleTarget(node_name='rule_alert_engine')
        self._get_state_client = self.create_client(GetState, self._lifecycle.get_state_service)
        self._change_state_client = self.create_client(
            ChangeState, self._lifecycle.change_state_service
        )

        self._last_lifecycle_state_label: Optional[str] = None
        self._pending_get_state: Optional[Future] = None
        self._pending_change_state: Optional[Future] = None
        self._pending_change_label: Optional[str] = None

        # When we request a lifecycle transition, we must not request another one
        # until the new state is observable, otherwise we can spam transitions and
        # crash lifecycle nodes (e.g., activate while already active).
        self._expected_state_label: Optional[str] = None
        self._expected_state_deadline_ns: Optional[int] = None
        self._expected_state_reason: Optional[str] = None

        self._ready_logged = False
        self._ready_timeout_logged = False
        self._started_ns = self.get_clock().now().nanoseconds

        # Timer-computed flags used when reconciling lifecycle state.
        self._ready_now: bool = False
        self._all_no_data_now: bool = False

        self._subscriptions = []
        for pid in patient_ids:
            topic = f'/{pid}/{vitals_topic}'
            sub = self.create_subscription(
                VitalSigns,
                topic,
                lambda msg, pid=pid: self._on_vitals(pid, msg),
                vitals_qos,
            )
            self._subscriptions.append(sub)

        self._log_event(
            'coord.config',
            patients=','.join(patient_ids),
            vitals_topic=vitals_topic,
            min_messages_per_patient=self._min_messages_per_patient,
            ready_timeout_sec=self._ready_timeout_sec,
            check_period_sec=self._check_period_sec,
            stale_after_sec=self._stale_after_sec,
            no_data_after_sec=self._no_data_after_sec,
            deactivate_on_no_data=self._deactivate_on_no_data,
            vitals_qos_depth=vitals_qos_depth,
            vitals_qos_reliability=vitals_qos_reliability,
            vitals_qos_durability=vitals_qos_durability,
        )

        self._timer = self.create_timer(self._check_period_sec, self._on_timer)

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _log_event(self, event: str, **fields) -> None:
        base = {
            'node': self.get_name(),
            'ns': self.get_namespace(),
        }
        safe_fields = {k: ('' if v is None else str(v)) for k, v in fields.items()}
        base.update(safe_fields)
        try:
            line = format_event(event, **base)
        except Exception:
            # Never fail the node because of logging
            line = f"event={event} node={self.get_name()} ns={self.get_namespace()}"
        self.get_logger().info(line)

    def _on_vitals(self, pid: str, _msg: VitalSigns) -> None:
        self._counts[pid] = int(self._counts.get(pid, 0)) + 1
        self._last_seen_ns[pid] = self.get_clock().now().nanoseconds

    def _age_sec(self, pid: str) -> Optional[float]:
        stamp = self._last_seen_ns.get(pid)
        if stamp is None:
            return None
        now_ns = self.get_clock().now().nanoseconds
        return max(0.0, (now_ns - int(stamp)) / 1e9)

    def _elapsed_sec(self) -> float:
        now_ns = self.get_clock().now().nanoseconds
        return max(0.0, (now_ns - int(self._started_ns)) / 1e9)

    def _calc_ready(self) -> bool:
        return is_ready(self._counts, min_messages_per_patient=self._min_messages_per_patient)

    def _calc_all_no_data(self) -> bool:
        ages = {pid: self._age_sec(pid) for pid in self._patient_ids}
        return all_patients_no_data(ages, no_data_after_sec=self._no_data_after_sec)

    def _maybe_request_get_state(self) -> None:
        if not self._get_state_client.service_is_ready():
            return
        if self._pending_get_state is not None and not self._pending_get_state.done():
            return

        future = self._get_state_client.call_async(GetState.Request())
        self._pending_get_state = future
        future.add_done_callback(self._on_get_state_done)

    def _on_get_state_done(self, future: Future) -> None:
        try:
            result = future.result()
        except Exception:
            return
        if result is None:
            return
        try:
            self._last_lifecycle_state_label = str(result.current_state.label)
        except Exception:
            return

        if (
            self._expected_state_label is not None
            and self._last_lifecycle_state_label == self._expected_state_label
        ):
            self._expected_state_label = None
            self._expected_state_deadline_ns = None
            self._expected_state_reason = None

        # Decide transitions based on the freshest observed state.
        self._reconcile_lifecycle_state()

    def _reconcile_lifecycle_state(self) -> None:
        if self._expected_state_label is not None:
            return

        state = self._last_lifecycle_state_label
        if state is None:
            return

        if self._ready_now:
            action = lifecycle_action_to_reach_active(state)
        elif self._deactivate_on_no_data and self._all_no_data_now:
            action = lifecycle_action_to_reach_inactive(state)
        else:
            action = None

        if action is None:
            return

        action_label, expected_state = action
        if action_label == 'configure':
            self._maybe_request_change_state(
                Transition.TRANSITION_CONFIGURE,
                'configure',
                expected_state_label=expected_state,
            )
        elif action_label == 'activate':
            self._maybe_request_change_state(
                Transition.TRANSITION_ACTIVATE,
                'activate',
                expected_state_label=expected_state,
            )
        elif action_label == 'deactivate':
            self._maybe_request_change_state(
                Transition.TRANSITION_DEACTIVATE,
                'deactivate',
                expected_state_label=expected_state,
            )

    def _maybe_request_change_state(
        self,
        transition_id: int,
        transition_label: str,
        *,
        expected_state_label: str,
        wait_timeout_sec: float = 5.0,
    ) -> None:
        if not self._change_state_client.service_is_ready():
            return
        if self._pending_change_state is not None and not self._pending_change_state.done():
            return
        if self._expected_state_label is not None:
            # Wait for the previous transition to take effect.
            return

        req = ChangeState.Request()
        req.transition.id = int(transition_id)

        future = self._change_state_client.call_async(req)
        self._pending_change_state = future
        self._pending_change_label = transition_label

        self._expected_state_label = str(expected_state_label)
        self._expected_state_reason = str(transition_label)
        self._expected_state_deadline_ns = self._now_ns() + int(float(wait_timeout_sec) * 1e9)

        future.add_done_callback(self._on_change_state_done)
        self._log_event(
            'coord.lifecycle_set',
            target=self._lifecycle.node_name,
            transition=transition_label,
            result='requested',
            expected_state=expected_state_label,
        )

    def _on_change_state_done(self, future: Future) -> None:
        label = self._pending_change_label or 'unknown'
        self._pending_change_label = None

        ok = False
        try:
            result = future.result()
            if result is not None:
                ok = bool(result.success)
        except Exception:
            ok = False

        self._log_event(
            'coord.lifecycle_set',
            target=self._lifecycle.node_name,
            transition=label,
            result='success' if ok else 'failure',
        )

        # Refresh cached state after any transition attempt.
        self._maybe_request_get_state()

    def _handle_transition_wait_timeout(self) -> None:
        if self._expected_state_label is None or self._expected_state_deadline_ns is None:
            return
        if self._now_ns() < int(self._expected_state_deadline_ns):
            return
        # Give up waiting; allow retry on next timer.
        self._log_event(
            'coord.lifecycle_wait_timeout',
            expected_state=self._expected_state_label,
            transition=self._expected_state_reason or 'unknown',
            result='timeout',
        )
        self._expected_state_label = None
        self._expected_state_deadline_ns = None
        self._expected_state_reason = None

    def _on_timer(self) -> None:
        self._handle_transition_wait_timeout()

        ready = self._calc_ready()
        elapsed = self._elapsed_sec()

        self._ready_now = bool(ready)
        self._all_no_data_now = bool(self._calc_all_no_data())

        if ready and not self._ready_logged:
            self._ready_logged = True
            self._log_event(
                'coord.ready',
                elapsed_sec=f"{elapsed:.3f}",
                min_messages_per_patient=self._min_messages_per_patient,
                counts=','.join(f"{pid}:{self._counts.get(pid, 0)}" for pid in self._patient_ids),
            )

        if (not ready) and (elapsed > self._ready_timeout_sec) and not self._ready_timeout_logged:
            self._ready_timeout_logged = True
            self._log_event(
                'coord.ready_timeout',
                elapsed_sec=f"{elapsed:.3f}",
                ready_timeout_sec=self._ready_timeout_sec,
                min_messages_per_patient=self._min_messages_per_patient,
                counts=','.join(f"{pid}:{self._counts.get(pid, 0)}" for pid in self._patient_ids),
            )

        # Keep state fresh; transition decisions are made in _on_get_state_done.
        self._maybe_request_get_state()

        # Optional: log when NO_DATA condition is met (deactivation is handled by reconcile).
        if self._deactivate_on_no_data and self._all_no_data_now:
            self._log_event(
                'coord.deactivate_on_no_data',
                no_data_after_sec=self._no_data_after_sec,
                ages=','.join(f"{pid}:{self._age_sec(pid)}" for pid in self._patient_ids),
            )


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        node = IcuCoordinatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except RCLError:
        raise
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
