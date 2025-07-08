import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = "base_link"

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")
    
    joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        unitree_go2_sim, "config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(unitree_go2_description, "worlds/default.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.375")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )
    
    # Description nodes and parameters
    robot_description = {"robot_description": Command(["xacro ", LaunchConfiguration("unitree_go2_description_path")])}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # CHAMP controller nodes
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": False},  # Non stiamo usando Gazebo
            {"publish_joint_states": True},
            {"publish_joint_control": True},  # Pubblichiamo comandi di controllo per RViz
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"publish_foot_contacts": False},
            {"close_loop_odom": False},  # Odom semplificata
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    # Semplifichiamo rimuovendo i nodi EKF per ora
    # state_estimator_node = Node(
    #     package="champ_base",
    #     executable="state_estimation_node",
    #     output="screen",
    #     parameters=[
    #         {"use_sim_time": use_sim_time},
    #         {"orientation_from_imu": True},
    #         {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
    #         joints_config,
    #         links_config,
    #         gait_config,
    #     ],
    # )

    # base_to_footprint_ekf = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="base_to_footprint_ekf",
    #     output="screen",
    #     parameters=[
    #         {"base_link_frame": base_frame},
    #         {"use_sim_time": use_sim_time},
    #         os.path.join(
    #             get_package_share_directory("champ_base"),
    #             "config",
    #             "ekf",
    #             "base_to_footprint.yaml",
    #         ),
    #     ],
    #     remappings=[("odometry/filtered", "odom/local")],
    # )

    # footprint_to_odom_ekf = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="footprint_to_odom_ekf",
    #     output="screen",
    #     parameters=[
    #         {"base_link_frame": base_frame},
    #         {"use_sim_time": use_sim_time},
    #         os.path.join(
    #             get_package_share_directory("champ_base"),
    #             "config",
    #             "ekf",
    #             "footprint_to_odom.yaml",
    #         ),
    #     ],
    #     remappings=[("odometry/filtered", "odom")],
    # )

    # Joint State Publisher per muovere i joint in RViz
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration("gui")),
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Joint State Publisher (versione non-GUI come fallback)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Teleop keyboard per controllare il robot
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
        prefix='xterm -e',  # Apre in una finestra separata
    )
    
    # TF statico per fornire una base transform stabile
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '--physics-engine gz-physics-bullet-plugin -r',
            'world': PathJoinSubstitution([
                unitree_go2_description,
                'worlds',
                'simple.sdf'
            ]), 
        }.items(),
    )
    
    # Spawn robot in Gazebo Sim
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', LaunchConfiguration('world_init_z'),
            '-Y', LaunchConfiguration('world_init_heading')
        ],
    )
    
    # Bridge ROS 2 topics to Gazebo Sim
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Gazebo to ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/velodyne_points@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',
            
            # ROS to Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
    )
    
    # Commentiamo temporaneamente i controller automatici per avviarli manualmente
    # controller_spawner_js = TimerAction(
    #     period=30.0,  # Wait longer for Gazebo and plugins to fully initialize
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             output="screen",
    #             arguments=[
    #                 "--controller-manager-timeout", "120",  # Much longer timeout
    #                 "joint_states_controller",  # No --inactive flag to ensure full activation
    #             ],
    #             parameters=[{"use_sim_time": use_sim_time}],
    #         )
    #     ]
    # )

    # controller_spawner_effort = TimerAction(
    #     period=35.0,  # Wait 5 seconds after joint_states_controller
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             output="screen",
    #             arguments=[
    #                 "--controller-manager-timeout", "120",  # Much longer timeout
    #                 "joint_group_effort_controller",  # No --inactive flag to ensure full activation
    #             ],
    #             parameters=[{"use_sim_time": use_sim_time}],
    #         )
    #     ]
    # )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    return LaunchDescription(
        [
            # Launch arguments
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path, 
            
            # Solo RViz e robot description - NO Gazebo
            robot_state_publisher_node,
            
            # TF statico
            static_tf_publisher,
            
            # Joint publishers per il controllo
            joint_state_publisher_gui,
            joint_state_publisher,
            
            # Teleop per controllo da tastiera
            teleop_keyboard,
            
            # CHAMP controller nodes per la cinematica
            quadruped_controller_node,
            # state_estimator_node,  # Commentato per semplificare
            
            # Visualization (only if rviz flag is set)
            rviz2,
        ]
    )