import os
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import tf_transformations


def generate_launch_description():
    declare_bag = DeclareLaunchArgument('bag', description='Path to rosbag file')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    def launch_setup(context, *args, **kwargs):
        bag_path = LaunchConfiguration('bag').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

        def read_initial_pose(path, target):
            reader = SequentialReader()
            storage_options = StorageOptions(uri=path, storage_id='sqlite3')
            converter_options = ConverterOptions('', '')
            reader.open(storage_options, converter_options)

            topics = reader.get_all_topics_and_types()
            type_map = {t.name: t.type for t in topics}
            tf_topics = [t.name for t in topics if t.type == 'tf2_msgs/msg/TFMessage']
            reader.set_filter(StorageFilter(topics=tf_topics))

            while reader.has_next():
                topic, data, _ = reader.read_next()
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                for tr in msg.transforms:
                    if tr.child_frame_id == target:
                        x = tr.transform.translation.x
                        y = tr.transform.translation.y
                        z = tr.transform.translation.z
                        q = tr.transform.rotation
                        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                        return x, y, z, yaw
            return 0.0, 0.0, 0.375, 0.0

        x, y, z, yaw = read_initial_pose(bag_path, 'vicon/robodog_magnetic/robodog_magnetic')

        base_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('unitree_go2_sim'),
                    'launch',
                    'unitree_go2_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world_init_x': str(x),
                'world_init_y': str(y),
                'world_init_z': str(z),
                'world_init_heading': str(yaw)
            }.items()
        )

        bag_player = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', bag_path,
                '--remap', '/tf:=/vicon_tf',
                '--remap', '/tf_static:=/vicon_tf_static'
            ],
            output='screen'
        )

        follower_node = Node(
            package='unitree_go2_sim',
            executable='tf_follower.py',
            name='tf_follower',
            parameters=[{
                'target_frame': 'vicon/robodog_magnetic/robodog_magnetic',
                'robot_frame': 'base_footprint',
                'tf_topic': '/vicon_tf',
                'tf_static_topic': '/vicon_tf_static',
                'use_sim_time': use_sim_time
            }]
        )

        return [base_launch, bag_player, follower_node]

    return LaunchDescription([
        declare_bag,
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup)
    ])
