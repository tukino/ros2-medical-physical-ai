from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'medical_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launchファイルを追加
        # NOTE: --symlink-install 時に最初のファイルで 'launch' がファイル扱いにならないよう、
        #       先にディレクトリを作る目的で空エントリを追加する
        (os.path.join('share', package_name, 'launch'), []),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Day7: config ディレクトリ（alert_rules.yaml など）
        (os.path.join('share', package_name, 'config'), []),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Codex',
    maintainer_email='codex@openai.com',
    description='Medical robot simulation nodes and launch assets',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vital_sensor = medical_robot_sim.vital_sensor:main',
            'hw_vital_sensor = medical_robot_sim.hw_vital_sensor:main',
            'medical_monitor = medical_robot_sim.medical_monitor:main',
            'icu_monitor = medical_robot_sim.icu_monitor:main',
            'icu_coordinator = medical_robot_sim.icu_coordinator:main',
            'advisory_publisher = medical_robot_sim.advisory_publisher:main',
            'closed_loop_controller = medical_robot_sim.closed_loop_controller:main',
            'bci_sensor = medical_robot_sim.bci_sensor:main',
            'bci_monitor = medical_robot_sim.bci_monitor:main',
            'rule_alert_engine = medical_robot_sim.rule_alert_engine:main',
            'rule_alert_engine_lifecycle = medical_robot_sim.rule_alert_engine_lifecycle:main',
        ],
    },
)
