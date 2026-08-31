#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_load_bay')

    urdf_file = os.path.join(pkg_share, 'urdf', 'load_bay_bot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'load_bay.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'load_bay.rviz')
    models_dir = os.path.join(pkg_share, 'models')

    # Environment Variables for Gazebo & Qt
    env_gz_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_dir)
    env_gz_partition = SetEnvironmentVariable('GZ_PARTITION', 'load_bay_sim')
    env_qt = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    # Process Xacro
    doc = xacro.process_file(urdf_file)
    robot_description_config = doc.toxml()

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # Spawn Robot Entity in Gazebo
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_config,
            '-name', 'load_bay_bot',
            '-x', '1.0',
            '-y', '0.1',  # Initial slight lateral offset
            '-z', '0.1',
            '-Y', '0.0',
            '-allow_renaming', 'true'
        ]
    )

    # ROS-GZ Bridge for Sensors & Controls
    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/vacuum_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vacuum_bot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/vacuum_bot/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/vacuum_bot/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/model/vacuum_bot/cmd_vel', '/cmd_vel'),
            ('/model/vacuum_bot/odom', '/odom'),
            ('/model/vacuum_bot/image_raw', '/camera/image_raw'),
            ('/model/vacuum_bot/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # ArUco Detector Node
    node_aruco_detector = Node(
        package='aruco_load_bay',
        executable='aruco_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Pose Verifier Node
    node_pose_verifier = Node(
        package='aruco_load_bay',
        executable='pose_verifier',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz2 Node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        env_gz_resource,
        env_gz_partition,
        env_qt,
        node_robot_state_publisher,
        gz_sim,
        node_spawn_entity,
        node_gz_bridge,
        node_aruco_detector,
        node_pose_verifier,
        node_rviz
    ])

