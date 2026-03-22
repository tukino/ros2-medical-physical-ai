from medical_robot_sim.observability import format_event


def test_format_event_is_single_line_and_sorted_keys():
    line = format_event('vitals.fault_config', z=1, a=2, m='x')
    assert '\n' not in line
    assert line.startswith('event=vitals.fault_config')

    # Key order must be stable (sorted).
    assert line == 'event=vitals.fault_config a=2 m=x z=1'


def test_format_event_sanitizes_whitespace_and_newlines():
    line = format_event('x', msg='hello world\nnext')
    assert line == 'event=x msg=hello_world\\nnext'
