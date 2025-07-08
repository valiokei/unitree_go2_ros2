import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    unitree_go2_description = get_package_share_directory("unitree_go2_description")
    unitree_go2_sim = get_package_share_directory("unitree_go2_sim")
    champ_base = get_package_share_directory("champ_base")
    
    # Launch arguments
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='unitree_go2',
        description='Name of the robot'
    )
    
    declare_description_path = DeclareLaunchArgument(
        'description_path', 
        default_value=os.path.join(unitree_go2_description, "urdf", "unitree_go2_robot.xacro"),
        description='Path to robot URDF/XACRO file'
    )
    
    # Robot description command
    robot_description_content = Command([
        'xacro ',
        LaunchConfiguration('description_path'),
        ' robot_name:=', LaunchConfiguration('robot_name'),
        ' lite:=false'
    ])
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )
    
    # TF Static Transform Publishers
    # NOTA: world->base_footprint sarà pubblicata dinamicamente dal nodo di odometria
    # Il base_link è sollevato dal base_footprint alla giusta altezza
    # Misurazione precedente: piedi a z=0.095 con base_link a z=0.32
    # Per far toccare i piedi a z=0, base_link deve essere a z=0.32-0.095=0.225
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.225', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )
    
    # CHAMP Quadruped Controller (for standing and movement)
    quadruped_controller = Node(
        package='champ_base',
        executable='quadruped_controller_node',
        name='quadruped_controller_node',
        output='screen',
        parameters=[
            os.path.join(unitree_go2_sim, "config/gait/gait.yaml"),
            os.path.join(unitree_go2_sim, "config/joints/joints.yaml"),
            os.path.join(unitree_go2_sim, "config/links/links.yaml"),
            {
                'use_sim_time': False,
                'publish_joint_states': False,  # Disable direct joint state publishing
                'publish_joint_control': True,  # Enable joint control commands
                'publish_foot_contacts': True,
                'gazebo': False,
                'joint_controller_topic': 'joint_group_effort_controller/joint_trajectory',
                'loop_rate': 200.0,
                'urdf': robot_description_content,
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/body_pose', '/body_pose')
        ]
    )

    # Joint Trajectory to Joint States Bridge
    joint_bridge = Node(
        package='champ_base',
        executable='joint_trajectory_to_joint_states.py',
        name='joint_trajectory_to_joint_states',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Simple Odometry for RViz-only mode (integrates cmd_vel and publishes world->base_footprint)
    simple_odometry = Node(
        package='champ_base',
        executable='simple_odometry.py',
        name='simple_odometry',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Teleop Keyboard for control
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel')]
    )
    
    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": False}]
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_rviz,
        declare_robot_name,
        declare_description_path,
        
        # Core nodes
        robot_state_publisher_node,
        base_footprint_to_base_link,
        
        # Controller (CHAMP for standing and movement)
        quadruped_controller,
        
        # Joint bridge for RViz-only mode
        joint_bridge,
        
        # Simple odometry for robot movement in RViz
        simple_odometry,
        
        # Control
        teleop_keyboard,
        
        # Visualization
        rviz2,
    ])
