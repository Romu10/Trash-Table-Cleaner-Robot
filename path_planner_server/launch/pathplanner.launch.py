import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    map_file = os.path.join(get_package_share_directory('map_server'), 'maps', 'rb1_cafeteria_sim_map.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters.yaml')

    return LaunchDescription([ 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'robot_base_link'],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'robot_base_footprint', 'base_link'],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/ros2_ws/src/path_planner_server/rviz2/pathplanner_rviz_config.rviz'],
            parameters=[{'use_sim_time': True}]),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]),
        
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server',
                                        'costmap_filter_info_server',
                                        'filter_mask_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}])
    ])