from medical_robot_sim.coordination_policy import lifecycle_action_to_reach_active
from medical_robot_sim.coordination_policy import lifecycle_action_to_reach_inactive


def test_lifecycle_action_to_reach_active() -> None:
    assert lifecycle_action_to_reach_active('active') is None
    assert lifecycle_action_to_reach_active('unconfigured') == ('configure', 'inactive')
    assert lifecycle_action_to_reach_active('inactive') == ('activate', 'active')
    assert lifecycle_action_to_reach_active('finalized') is None


def test_lifecycle_action_to_reach_inactive() -> None:
    assert lifecycle_action_to_reach_inactive('inactive') is None
    assert lifecycle_action_to_reach_inactive('unconfigured') is None
    assert lifecycle_action_to_reach_inactive('active') == ('deactivate', 'inactive')
