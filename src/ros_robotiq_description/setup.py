from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'ros_robotiq_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf',
            glob(os.path.join(package_name, 'urdf', '*'))),
        ('share/' + package_name + '/launch',
            glob(os.path.join(package_name, 'launch', '*.py'))),
        ('share/' + package_name + '/rviz',
            glob(os.path.join(package_name, 'rviz', '*'))),
        ('share/' + package_name + '/config',
            glob(os.path.join(package_name, 'config', '*'))),
        ('share/' + package_name + '/worlds',
            glob(os.path.join(package_name, 'worlds', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pierce',
    maintainer_email='coylep2@wwu.edu',
    description='ur3e motion planning + gripper',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "detection = ros_robotiq_description.detection:main",
            'disparity_to_depth = ros_robotiq_description.disparity_to_depth:main',
            'stereo_camera_info_republisher = ros_robotiq_description.stereo_camera_info_republisher:main',
        ],
    },
)
