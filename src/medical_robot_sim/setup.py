# TODO: ICU monitor が「最後に受信した時刻」から患者状態を判定できるようにする。
# 要件:
# - 患者ごとに last_seen（rclpy clock）を保持
# - age_sec = now - last_seen
# - last_seen が None の場合 -> alert = "NO DATA"
# - age_sec > 10 の場合 -> alert = "NO DATA"
# - それ以外で age_sec > 3 の場合 -> alert = "STALE"
# - それ以外 -> alert = "OK"
# - ダッシュボードの alert 列に反映
# - 最後の値は保持しつつ、状態（NO DATA/STALE 等）を明確に表示

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tukino',
    maintainer_email='tukino@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vital_sensor = medical_robot_sim.vital_sensor:main',
            'medical_monitor = medical_robot_sim.medical_monitor:main',
            'icu_monitor = medical_robot_sim.icu_monitor:main',
            'rule_alert_engine = medical_robot_sim.rule_alert_engine:main',
        ],
    },
)
