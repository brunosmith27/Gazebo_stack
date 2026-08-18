#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    gazebo_world = 'empty.world'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('hunter_gazebo')
    description_package_name = "hunter_description"
    install_dir = get_package_prefix(description_package_name)
    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib' + ':/opt/ros/humble/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib' + ':/opt/ros/humble/lib'

    print("GAZEBO MODELS PATH==" + str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH==" + str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Load URDF
    urdf_file = os.path.join(
        get_package_share_directory('hunter_description'),
        'urdf', 'robot.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': os.path.join(pkg_box_bot_gazebo, 'worlds', gazebo_world),
            'extra_gazebo_args': '-s libgazebo_ros_factory.so'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_box_bot_gazebo, 'worlds', gazebo_world), ''],
            description='SDF world file'),
        robot_state_publisher,
        gazebo,
    ])
