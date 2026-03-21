import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from medical_robot_sim.rule_alert_engine_lifecycle import RuleAlertEngineLifecycleNode


def test_lifecycle_transitions_create_and_destroy_io():
    rclpy.init()
    try:
        node = RuleAlertEngineLifecycleNode()
        try:
            node.set_parameters(
                [
                    Parameter('patients', value=['patient_01']),
                ]
            )

            assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS

            assert getattr(node, '_engine_subscriptions', []) == []
            assert getattr(node, '_engine_pubs', {}) == {}

            assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

            assert len(getattr(node, '_engine_subscriptions', [])) == 1
            assert len(getattr(node, '_engine_pubs', {})) == 1

            assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS

            assert getattr(node, '_engine_subscriptions', []) == []
            assert getattr(node, '_engine_pubs', {}) == {}

            assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
