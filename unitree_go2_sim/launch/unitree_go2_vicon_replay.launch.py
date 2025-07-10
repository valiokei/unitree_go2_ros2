import os
import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rosbags.highlevel import AnyReader
import transformations


def _get_initial_pose(bag_path: str, target: str, world: str):
    """Extract initial pose from the given rosbag."""
    if not os.path.exists(bag_path):
        return 0.0, 0.0, 0.0, 0.0

    with AnyReader([bag_path]) as reader:
        conns = [c for c in reader.connections if c.topic in ('/tf', '/tf_static')]
        for connection, timestamp, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, connection.msgtype)
            for t in msg.transforms:
                if t.child_frame_id == target and t.header.frame_id == world:
                    q = [
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w,
                    ]
                    _, _, yaw = transformations.euler_from_quaternion(q)
                    return (
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                        yaw,
                    )
    return 0.0, 0.0, 0.0, 0.0


def _set_spawn_from_bag(context, *args, **kwargs):
    bag_path = LaunchConfiguration('bag').perform(context)
    target = LaunchConfiguration('target_frame').perform(context)
    world = LaunchConfiguration('world_frame').perform(context)

    x, y, z, yaw = _get_initial_pose(bag_path, target, world)

    return [
        SetLaunchConfiguration('world_init_x', str(x)),
        SetLaunchConfiguration('world_init_y', str(y)),
        SetLaunchConfiguration('world_init_z', str(z)),
        SetLaunchConfiguration('world_init_heading', str(yaw)),
    ]


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    target_frame = LaunchConfiguration('target_frame')
    world_frame = LaunchConfiguration('world_frame')
    nominal_height = LaunchConfiguration('nominal_height')

    declare_bag = DeclareLaunchArgument(
        'bag',
        description='Path to rosbag containing Vicon TF data'
    )
    declare_target_frame = DeclareLaunchArgument(
        'target_frame',
        default_value='vicon/robodog_magnetic/robodog_magnetic',
        description='Child frame in the bag',
    )
    declare_world_frame = DeclareLaunchArgument(
        'world_frame',
        default_value='vicon/world',
        description='Parent frame in the bag',
    )
    declare_nominal_height = DeclareLaunchArgument(
        'nominal_height',
        default_value='0.225',
        description='Nominal robot height',
    )

    # Include default simulation launch
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('unitree_go2_sim'),
                'launch',
                'unitree_go2_launch.py')
        ),
        launch_arguments={
            'world_init_x': LaunchConfiguration('world_init_x'),
            'world_init_y': LaunchConfiguration('world_init_y'),
            'world_init_z': LaunchConfiguration('world_init_z'),
            'world_init_heading': LaunchConfiguration('world_init_heading'),
        }.items()
    )

    # Play the rosbag
    bag_play = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag],
            output='screen'
        )]
    )

    # Setup spawn pose using the first TF in the bag
    spawn_from_bag = OpaqueFunction(function=_set_spawn_from_bag)

    # Node to convert Vicon TF to body pose setpoint
    vicon_pose = Node(
        package='champ_base',
        executable='vicon_tf_to_pose.py',
        name='vicon_tf_to_pose',
        output='screen',
        parameters=[{
            'target_frame': target_frame,
            'world_frame': world_frame,
            'pose_topic': 'body_pose',
            'nominal_height': nominal_height,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        declare_bag,
        declare_target_frame,
        declare_world_frame,
        declare_nominal_height,
        spawn_from_bag,
        base_launch,
        bag_play,
        vicon_pose,
    ])
