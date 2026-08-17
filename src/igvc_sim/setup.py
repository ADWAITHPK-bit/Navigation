import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'igvc_sim'


def package_files(directory, destination):
    files = []

    for root, dirs, filenames in os.walk(directory):
        for filename in filenames:
            source = os.path.join(root, filename)
            relative_path = os.path.relpath(root, directory)
            destination_path = os.path.join(destination, relative_path)

            files.append(
                (
                    destination_path,
                    [source]
                )
            )

    return files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/worlds',
            glob('worlds/*.world')
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.py')
        ),
        (
            'share/' + package_name + '/urdf',
            glob('urdf/*')
        ),
    ]
    + package_files(
        'models/igvc_ground',
        'share/' + package_name + '/models/igvc_ground'
    )
    + package_files(
        'igvc_vehicle',
        'share/' + package_name + '/igvc_vehicle'
    ),

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='adwaith',
    maintainer_email='pkadwaith06@gmail.com',

    description='IGVC simulation package',
    license='TODO',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [],
    },
)