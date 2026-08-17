from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'igvc_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adwaith',
    maintainer_email='pkadwaith06@gmail.com',
    description='Perception: lane detection, YOLO object detection, training data collection and fine-tuning',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'lane_detector = igvc_perception.lane_detector:main',
            'goal_generator = igvc_perception.goal_generator:main'
        ],
    },
)
