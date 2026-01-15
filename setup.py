from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'ur3_traj'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),      
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='titouan',
    maintainer_email='titouan.briancon@sigma-clermont.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_robot = ur3_traj.pose_robot:main',
            'ur3_moveit_action = ur3_traj.ur3_moveit_action:main',
            'scene_object = ur3_traj.scene_object:main',
            'visu_3D_point = ur3_traj.visu_3D_point:main',
            'record_F = ur3_traj.record_F:main',
        ],
    },
)
