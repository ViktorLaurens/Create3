import glob
import os
from setuptools import find_packages, setup

package_name = 'create3_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[(
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viktordg',
    maintainer_email='viktor.degroote@gmail.com',
    description='Control package for the iRobot Create-3 in ROS 2',
    license='MIT',  # Update with the appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your Python node scripts here if you plan to run them as executables
            # 'undock_robot = create3_control.undock_robot:main',
            # 'move_to_point = create3_control.move_to_point:main',
            # 'dock_robot = create3_control.dock_robot:main',
            'move_robot = create3_control.move_robot:main',
            'test = create3_control.test:main',
        ],
    },
)
