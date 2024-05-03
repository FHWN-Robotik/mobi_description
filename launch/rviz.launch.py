"""
Author: Mustafa Algan
Date: 03.05.2024

Describer:  Launch RVIZ with Joint State Publisher GUI
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

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
        
    # Joint State Publisher
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        name='joint_state_publisher_gui')

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            rviz_node,
            joint_state_publisher_gui_node
        ]
    )