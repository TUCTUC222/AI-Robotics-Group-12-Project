#!/usr/bin/env python3

"""
Launch file for the Obstacle Course simulation

This launch file:
1. Starts Gazebo with the obstacle course world
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare('lidar_target_follower').find('lidar_target_follower')
    models_path = os.path.join(pkg_share, 'models')
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'obstacle_course.world')
    
    # Set IGN_GAZEBO_RESOURCE_PATH to include the models directory
    # We need to append to the existing path if it exists
    ign_resource_path = models_path
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        ign_resource_path += ':' + os.environ['IGN_GAZEBO_RESOURCE_PATH']

    set_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path)

    # Gazebo server (simulator)
    gz_sim_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world_file],
        output='screen'
    )
    
    # Gazebo client (GUI)
    gz_sim_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen'
    )

    # Spawn Robot - At origin, on the ground
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot',
            '-file', os.path.join(models_path, 'turtlebot', 'model.sdf'),
            '-x', '0.0', '-y', '0.0', '-z', '0.01'
        ],
        output='screen'
    )

    # Spawn Target - 5 meters ahead so robot has room to move
    spawn_target = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cyan_target',
            '-file', os.path.join(models_path, 'cyan_target', 'model.sdf'),
            '-x', '5.0', '-y', '0.0', '-z', '0.3'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
        ],
        output='screen'
    )

    # Vision Target Follower Node (uses camera + LIDAR)
    follower_node = Node(
        package='lidar_target_follower',
        executable='vision_target_follower_node',
        name='vision_target_follower_node',
        output='screen'
    )
    
    return LaunchDescription([
        set_env,
        gz_sim_server,
        gz_sim_gui,
        spawn_robot,
        spawn_target,
        bridge,
        follower_node
    ])

