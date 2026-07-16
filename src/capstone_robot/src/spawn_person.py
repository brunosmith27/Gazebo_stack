#!/usr/bin/env python3
import argparse
import math
import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetModelList, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node


class PersonSpawner(Node):
    def __init__(self):
        super().__init__('person_spawner')
        self.get_list_client = self.create_client(GetModelList, '/get_model_list')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

    def call(self, client, request, timeout=10.0):
        client.wait_for_service(timeout_sec=timeout)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def find_robot_name(self, prefix='hunter-'):
        result = self.call(self.get_list_client, GetModelList.Request())
        for name in result.model_names:
            if name.startswith(prefix):
                return name
        return None

    def get_model_pose(self, model_name):
        output = subprocess.check_output(['gz', 'model', '-m', model_name, '-p'], text=True)
        x, y, z, roll, pitch, yaw = (float(v) for v in output.split())
        return x, y, z, yaw

    def spawn(self, entity_name, sdf_path, x, y, z):
        with open(sdf_path, 'r') as f:
            xml = f.read()
        request = SpawnEntity.Request()
        request.name = entity_name
        request.xml = xml
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        return self.call(self.spawn_client, request)


def main():
    parser = argparse.ArgumentParser(description='Spawn the test person model in front of the robot.')
    parser.add_argument('--entity', default='person1', help='Name for the spawned entity')
    parser.add_argument('--distance', type=float, default=3.0, help='Meters ahead of the robot to place it')
    parser.add_argument('--x', type=float, default=None, help='Absolute world x (overrides --distance)')
    parser.add_argument('--y', type=float, default=None, help='Absolute world y (overrides --distance)')
    parser.add_argument('--z', type=float, default=0.0, help='Absolute world z')
    args = parser.parse_args()

    rclpy.init()
    node = PersonSpawner()

    sdf_path = os.path.join(
        get_package_share_directory('capstone_robot'), 'models', 'person.sdf'
    )

    if args.x is not None and args.y is not None:
        x, y, z = args.x, args.y, args.z
    else:
        robot_name = node.find_robot_name()
        if robot_name is None:
            node.get_logger().error('Could not find a spawned "hunter-*" robot model. Is the sim running?')
            node.destroy_node()
            rclpy.shutdown()
            return
        rx, ry, rz, yaw = node.get_model_pose(robot_name)
        x = rx + args.distance * math.cos(yaw)
        y = ry + args.distance * math.sin(yaw)
        z = args.z
        node.get_logger().info(f'Robot "{robot_name}" at ({rx:.2f}, {ry:.2f}), yaw {yaw:.2f} rad')

    node.get_logger().info(f'Spawning "{args.entity}" at ({x:.2f}, {y:.2f}, {z:.2f})')
    result = node.spawn(args.entity, sdf_path, x, y, z)
    if result is not None and result.success:
        node.get_logger().info(f'Spawn OK: {result.status_message}')
    else:
        msg = result.status_message if result is not None else 'no response (timed out)'
        node.get_logger().error(f'Spawn failed: {msg}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
