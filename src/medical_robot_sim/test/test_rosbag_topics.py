"""Day12: rosbag_topics のユニットテスト."""

from __future__ import annotations

import pytest


def test_build_record_topics_basic_vitals_only():
    from medical_robot_sim.rosbag_topics import build_record_topics

    topics = build_record_topics(['patient_01'], 'patient_vitals', include_alerts=False)
    assert topics == ['/patient_01/patient_vitals']


def test_build_record_topics_trims_and_normalizes_slashes():
    from medical_robot_sim.rosbag_topics import build_record_topics

    topics = build_record_topics([' /patient_01/ ', 'patient_02', ''], '/patient_vitals/', True)
    assert topics == [
        '/patient_01/patient_vitals',
        '/patient_01/alerts',
        '/patient_02/patient_vitals',
        '/patient_02/alerts',
    ]


def test_build_record_topics_rejects_empty_vitals_topic():
    from medical_robot_sim.rosbag_topics import build_record_topics

    with pytest.raises(ValueError):
        build_record_topics(['patient_01'], '   ', False)


@pytest.mark.parametrize(
    'patients',
    [
        [],
        [''],
        ['   '],
        ['/', ' / '],
    ],
)
def test_build_record_topics_rejects_empty_patients(patients: list[str]):
    from medical_robot_sim.rosbag_topics import build_record_topics

    with pytest.raises(ValueError):
        build_record_topics(patients, 'patient_vitals', False)
