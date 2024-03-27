import os
from glob import glob
from setuptools import setup

package_name = 'nav2_interface'
launch_files = (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        launch_files
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
            'look_for_trash_table = nav2_interface.look_for_trash_table:main'
        ],
    },
)
