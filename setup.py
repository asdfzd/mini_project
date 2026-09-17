from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taehwan',
    maintainer_email='jolviadr@gmail.com',
    description='Hybrid YOLO, RGB-D, TF2, Nav2, and visual tracking mission for TurtleBot 4',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    entry_points={
        'console_scripts': [
            'mission_controller = rokey_pjt.mission_controller:main',
            'webcam_trigger = rokey_pjt.webcam_trigger:main',
        ],
    },
)
