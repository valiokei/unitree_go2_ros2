import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag = LaunchConfiguration('bag')

    declare_bag = DeclareLaunchArgument(
        'bag',
        description='Path to rosbag containing Vicon TF data'
    )

    # Include default simulation launch
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('unitree_go2_sim'),
                'launch',
                'unitree_go2_launch.py')
        )
    )

    # Play the rosbag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag],
        output='screen'
    )

    # Node to convert Vicon TF to body pose setpoint
    vicon_pose = Node(
        package='champ_base',
        executable='vicon_tf_to_pose.py',
        name='vicon_tf_to_pose',
        output='screen',
        parameters=[{
            'target_frame': 'vicon/robodog_magnetic/robodog_magnetic',
            'world_frame': 'vicon/world',
            'pose_topic': 'body_pose',
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        declare_bag,
        base_launch,
        bag_play,
        vicon_pose,
    ])
