from setuptools import find_packages, setup

package_name = 'igvc_goal_logic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adwaith',
    maintainer_email='pkadwaith06@gmail.com',
    description='Goal generation - hardcoded test goal for now, YOLO centerline later',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_goal = igvc_goal_logic.send_goal:main',
        ],
    },
)
