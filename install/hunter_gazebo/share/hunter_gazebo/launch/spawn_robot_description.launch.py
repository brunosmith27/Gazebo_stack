#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Spawn Position and Orientation
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]

    robot_base_name = "hunter"
    entity_name = robot_base_name + "-" + str(int(random.random() * 100000))

    # Node to spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', entity_name,
            '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
            '-topic', '/robot_description'
        ]
    )

    # Add a small delay to ensure controller_manager is up
    delayed_load_joint_state_controller = TimerAction(
        period=5.0,  # wait 5 seconds before loading
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller', '--set-state', 'active',
                    'joint_state_broadcaster'
                ],
                output='screen'
            )
        ]
    )

    delayed_load_ackermann_controller = TimerAction(
        period=7.0,  # load after additional delay
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller', '--set-state', 'active',
                    'ackermann_steering_controller'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        spawn_robot,
        delayed_load_joint_state_controller,
        delayed_load_ackermann_controller,
    ])
