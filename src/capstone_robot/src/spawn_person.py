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
    parser = argparse.ArgumentParser(
        description='Spawn test person models relative to the robot: one straight '
                     'ahead, one ahead-and-to-the-left.'
    )
    parser.add_argument('--prefix', default='person', help='Base name for the spawned entities')
    parser.add_argument('--front-distance', type=float, default=3.0,
                         help='Meters ahead of the robot for the front person')
    parser.add_argument('--left-distance', type=float, default=2.5,
                         help='Meters ahead of the robot for the left person')
    parser.add_argument('--left-offset', type=float, default=1.5,
                         help='Meters to the left of the robot heading for the left person')
    parser.add_argument('--z', type=float, default=0.0, help='Absolute world z for both')
    parser.add_argument('--x', type=float, default=None,
                         help='Absolute world x -- spawns a single person here instead of the front/left pair')
    parser.add_argument('--y', type=float, default=None,
                         help='Absolute world y -- spawns a single person here instead of the front/left pair')
    parser.add_argument('--entity', default='person1',
                         help='Entity name when using --x/--y single-spawn mode')
    args = parser.parse_args()

    rclpy.init()
    node = PersonSpawner()

    sdf_path = os.path.join(
        get_package_share_directory('capstone_robot'), 'models', 'person.sdf'
    )

    if args.x is not None and args.y is not None:
        targets = [(args.entity, args.x, args.y, args.z)]
    else:
        robot_name = node.find_robot_name()
        if robot_name is None:
            node.get_logger().error('Could not find a spawned "hunter-*" robot model. Is the sim running?')
            node.destroy_node()
            rclpy.shutdown()
            return
        rx, ry, rz, yaw = node.get_model_pose(robot_name)
        node.get_logger().info(f'Robot "{robot_name}" at ({rx:.2f}, {ry:.2f}), yaw {yaw:.2f} rad')

        fwd = (math.cos(yaw), math.sin(yaw))
        left = (-math.sin(yaw), math.cos(yaw))

        front_x = rx + args.front_distance * fwd[0]
        front_y = ry + args.front_distance * fwd[1]

        left_x = rx + args.left_distance * fwd[0] + args.left_offset * left[0]
        left_y = ry + args.left_distance * fwd[1] + args.left_offset * left[1]

        targets = [
            (f'{args.prefix}_front', front_x, front_y, args.z),
            (f'{args.prefix}_left', left_x, left_y, args.z),
        ]

    # Spawn one at a time (not concurrently) -- Gazebo under WSLg has shown it can
    # choke/hang if hit with multiple simultaneous factory requests.
    for entity_name, x, y, z in targets:
        node.get_logger().info(f'Spawning "{entity_name}" at ({x:.2f}, {y:.2f}, {z:.2f})')
        result = node.spawn(entity_name, sdf_path, x, y, z)
        if result is not None and result.success:
            node.get_logger().info(f'Spawn OK: {result.status_message}')
        else:
            msg = result.status_message if result is not None else 'no response (timed out)'
            node.get_logger().error(f'Spawn failed for "{entity_name}": {msg}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
