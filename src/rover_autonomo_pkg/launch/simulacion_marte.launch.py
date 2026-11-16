import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover = get_package_share_directory('rover_autonomo_pkg')
    pkg_leo_worlds = get_package_share_directory('leo_gz_worlds')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    mundo_marte = os.path.join(pkg_leo_worlds, 'worlds', 'marsyard2022.sdf')

    # 1. Apuntar al nuevo archivo mark4.urdf.xacro
    xacro_file = os.path.join(pkg_rover, 'urdf', 'mark4.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # 2. Incluir la ruta padre de rover_autonomo_pkg para que Gazebo encuentre los .stl
    rutas_recursos = ':'.join([
        os.path.dirname(pkg_rover),
        os.path.join(pkg_leo_worlds, 'models'),
        os.path.join(pkg_leo_worlds, 'worlds'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ])

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=rutas_recursos
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {mundo_marte}'}.items(),
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mark4',
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '2.0',
            '-z', '1.68'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/sim/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/steer_FL/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/steer_FR/cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan_precipicio@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    muros_sdf = os.path.join(pkg_rover, 'worlds', 'muros_perimetro.sdf')
    spawn_muros = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'muros_perimetro', '-file', muros_sdf],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        robot_state_pub,
        spawn_rover,
        spawn_muros,
        bridge,
    ])