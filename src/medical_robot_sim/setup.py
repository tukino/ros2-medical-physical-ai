# TODO: ICU monitor should mark each patient state based on last message age.
# Requirements:
# - Maintain per-patient last_seen timestamp (rclpy clock)
# - age_sec = now - last_seen
# - If last_seen is None -> alert = "NO DATA"
# - If age_sec > 10 -> alert = "NO DATA"
# - Else if age_sec > 3 -> alert = "STALE"
# - Else -> alert = "OK"
# - Dashboard prints alert column accordingly
# - Keep last values but must show alert state clearly

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
    maintainer='yohei',
    maintainer_email='yohei@todo.todo',
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
        ],
    },
)
