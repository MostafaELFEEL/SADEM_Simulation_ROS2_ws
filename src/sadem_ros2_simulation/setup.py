from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'sadem_ros2_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elfeel',
    maintainer_email='elfeel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'a_star = sadem_ros2_simulation.A_star:main',
        'path_to_goal = sadem_ros2_simulation.Path_to_goal:main',
        'motors = sadem_ros2_simulation.motors:main',
        'controller = sadem_ros2_simulation.controller:main',
        ],
    },
)
