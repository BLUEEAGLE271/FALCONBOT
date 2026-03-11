from setuptools import find_packages, setup
import os
from glob import glob

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
    description='Parking and Odometry Logic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # NOTE: In ament_cmake, these entry points are not automatically generated.
    # The CMakeLists 'install(PROGRAMS...)' handles the executables.
    entry_points={
        'console_scripts': [
            'rectified_parking = parking.rectified_parking:main',
            'box_estimator = parking.box_estimator:main',
            'mission_controller = parking.mission_controller:main',
            'pwm_data_collector = parking.pwm_data_collector:main'
        ],
    },
)