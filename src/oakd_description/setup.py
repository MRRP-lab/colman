from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'oakd_description'

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
        ('share/' + package_name + '/meshes',
            glob(os.path.join(package_name, 'meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pierce',
    maintainer_email='dev@piercecoyle.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
