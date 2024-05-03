"""
Author: Mustafa Algan
Date: 03.05.2024

Describer:  Launch RVIZ and Gazebo with the mobi Model
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ####### DATA INPUT ##########
    urdf_file = 'mobi.urdf.xacro'
    #xacro_file = "urdfbot.xacro"
    package_description = "mobi_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    xacro_file = get_package_share_directory(package_description) + '/urdf/' + urdf_file

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': Command(['xacro', ' ', xacro_file])
        }])

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'mobi.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])

    # Spawn the robot in Gazebo
    spawn_entity_robot = Node(package='gazebo_ros',
                              executable='spawn_entity.py',
                              arguments=['-entity', 'mobi', '-topic', 'robot_description'],
                              output='screen')

    # Start Gazebo with my empty world
    world_file_name = 'test.world'
    world = os.path.join(get_package_share_directory(package_description), 'worlds', world_file_name)
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'], output='screen')

    return LaunchDescription([robot_state_publisher_node, rviz_node, spawn_entity_robot, gazebo_node])
