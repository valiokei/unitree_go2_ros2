import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_bag = DeclareLaunchArgument('bag', description='Path to rosbag file')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('unitree_go2_sim'),
                'launch',
                'unitree_go2_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag')],
        output='screen'
    )

    follower_node = Node(
        package='unitree_go2_sim',
        executable='tf_follower.py',
        name='tf_follower',
        parameters=[{
            'target_frame': 'vicon/robodog_magnetic/robodog_magnetic',
            'robot_frame': 'base_footprint',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        declare_bag,
        declare_use_sim_time,
        base_launch,
        bag_player,
        follower_node,
    ])
