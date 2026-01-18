from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
package_name = 'parking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blueeagle',
    maintainer_email='blueeagle@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rectified_parking = parking.rectified_parking:main',
            'box_estimator = parking.box_estimator:main', # <--- Add this
            'mission_controller = parking.mission_controller:main',
            'scan_republisher = parking.scan_republisher:main',
        ],
    },
)
