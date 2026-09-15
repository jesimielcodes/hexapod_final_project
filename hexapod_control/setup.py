from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'hexapod_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name + '/config', ['config/hexapod_controller.yaml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='frimponghedia@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'wall_follower = hexapod_control.wall_follower:main', #wall following has been included in the astar so it's fine
            'gait_planner = hexapod_control.gait_planner:main',
            'local_astar_navigator = hexapod_control.astar_planner:main',
            'odom_tfbroadcaster = hexapod_control.odom_tfbroadcaster:main',
        ],
    },
)
