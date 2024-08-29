from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value=os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_house.world')),
        DeclareLaunchArgument('robot_name', default_value='triton'),
        DeclareLaunchArgument('sdf_robot_file', default_value=os.path.join(
            get_package_share_directory('particle_filter'), 'models', LaunchConfiguration('robot_name'), 'model.sdf')),
        DeclareLaunchArgument('x', default_value='-3.0'),
        DeclareLaunchArgument('y', default_value='1.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world_file'),
                'use_sim_time': 'true',
                'paused': 'false',
                'gui': 'true',
                'verbose': 'true',
                'recording': 'false'
            }.items(),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=LaunchConfiguration('robot_name') + '_spawn_urdf',
            output='screen',
            arguments=[
                '-file', LaunchConfiguration('sdf_robot_file'),
                '-sdf',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('roll'),
                '-P', LaunchConfiguration('pitch'),
                '-Y', LaunchConfiguration('yaw'),
                '-entity', LaunchConfiguration('robot_name')
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_scan',
            arguments=['0', '0', '0.161', '0', '0', '0', 'base_link', 'base_scan', '100']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom', '100']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0.0', 'map', 'world', '100']
        ),

        Node(
            package='particle_filter',
            executable='position_publisher.py',
            name='position_publisher',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(
                get_package_share_directory('particle_filter'), 'rviz', 'lidar.rviz')],
            output='screen'
        ),
    ])
