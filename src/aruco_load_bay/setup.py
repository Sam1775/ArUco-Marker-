import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aruco_load_bay'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'models/load_bay_marker'), glob('models/load_bay_marker/*.*')),
        (os.path.join('share', package_name, 'models/load_bay_marker/materials/scripts'), glob('models/load_bay_marker/materials/scripts/*')),
        (os.path.join('share', package_name, 'models/load_bay_marker/materials/textures'), glob('models/load_bay_marker/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='sam_17',
    maintainer_email='sam@example.com',
    description='ROS 2 Jazzy ArUco Load-Bay Pose Verification System',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_load_bay.aruco_detector:main',
            'pose_verifier = aruco_load_bay.pose_verifier:main',
        ],
    },
)
