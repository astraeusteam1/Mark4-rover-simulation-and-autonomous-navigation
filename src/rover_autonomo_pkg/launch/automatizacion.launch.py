from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_autonomo_pkg',
            executable='ejecutable_sensores',
            name='nodo_sensores',
            output='screen'
        ),
        Node(
            package='rover_autonomo_pkg',
            executable='ejecutable_control',
            name='nodo_control_autonomo',
            output='screen'
        ),
        Node(
            package='rover_autonomo_pkg',
            executable='ejecutable_odometria',
            name='nodo_odometria',
            output='screen'
        ),
        Node(
            package='rover_autonomo_pkg',
            executable='ejecutable_camara',
            name='nodo_camara_vision',
            output='screen'
        ),
        Node(
            package='rover_autonomo_pkg',
            executable='ejecutable_mapeo',
            name='nodo_mapeo',
            output='screen'
        ),
        Node(
            package='rover_autonomo_pkg',
            executable='ejecutable_mapa_peligros',
            name='nodo_mapa_peligros',
            output='screen'
        ),
        Node(
            package='rover_autonomo_pkg',
            executable='ejecutable_esp32',
            name='nodo_esp32_bridge',
            output='screen'
        ),
    ])