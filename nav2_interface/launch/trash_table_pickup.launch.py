import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_filtering',
            executable='laser_filtering',
            name='laser_filtering'
        ),
        Node(
            package='trash_table_detect',
            executable='trash_table_detect',
            name='trash_table_detect'
        ),
        Node(
            package='approach_controller',
            executable='approach_control',
            name='approach_control'
        ),
        Node(
            package='approach_controller',
            executable='move_table_back',
            name='move_table_back'
        ),
        Node(
            package='approach_controller',
            executable='table_transforms',
            name='table_transforms'
        )

    ])