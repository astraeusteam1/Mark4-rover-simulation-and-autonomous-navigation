import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonomo_pkg')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # TF estática para que local_costmap ubique el sensor frontal respecto al chasis
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_sensor_frontal',
            arguments=['0.15', '0.0', '0.10', '0.0', '0.0', '0.0', 'base_link', 'sensor_frontal_link']
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[params_file]
        ),
    ])