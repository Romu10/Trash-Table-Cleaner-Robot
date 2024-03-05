import os
from glob import glob
from setuptools import setup

package_name = 'path_planner_server'
launch_files = (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
config_files = (os.path.join('share', package_name, 'config'), glob('config/*'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        launch_files,
        config_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='romulobryanp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
