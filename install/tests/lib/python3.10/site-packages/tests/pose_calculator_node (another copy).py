import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import pyproj

class PoseCalculatorNode(Node):
    def __init__(self):
        super().__init__('pose_calculator')

        # Subscribers
        self.create_subscription(Imu, '/camera/imu/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)

        # Publishers
        self.pose_publisher = self.create_publisher(PoseStamped, '/calculated_pose', 10)
        self.theta_publisher = self.create_publisher(Float32, '/theta_degrees', 10)

        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Internal storage for sensor data
        self.gps_latitude = None
        self.gps_longitude = None
        self.previous_gps_time = None
        self.imu_orientation = None
        self.current_x = None
        self.current_y = None

        # GPS to local frame converter
        self.geod = pyproj.Geod(ellps='WGS84')
        self.base_latitude = None
        self.base_longitude = None

        # Static transforms
        self.broadcast_map_to_odom_transform()

    def broadcast_map_to_odom_transform(self):
        # Create a static transform between map and odom
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Static transform between 'map' and 'odom' published.")

    def gps_callback(self, msg):
        # Extract GPS coordinates
        self.gps_latitude = msg.latitude
        self.gps_longitude = msg.longitude

        # Set base reference for the first reading
        if self.base_latitude is None or self.base_longitude is None:
            self.base_latitude = self.gps_latitude
            self.base_longitude = self.gps_longitude

        # Convert GPS to local X, Y coordinates
        _, _, distance_x = self.geod.inv(self.base_longitude, self.base_latitude, self.gps_longitude, self.base_latitude)
        _, _, distance_y = self.geod.inv(self.base_longitude, self.base_latitude, self.base_longitude, self.gps_latitude)

        # Adjust signs based on movement
        if self.gps_longitude > self.base_longitude:
            distance_x = -distance_x
        if self.gps_latitude < self.base_latitude:
            distance_y = -distance_y

        self.current_x = distance_x
        self.current_y = distance_y

    def imu_callback(self, msg):
        # Extract IMU orientation
        self.imu_orientation = msg.orientation

        # If GPS data is not available, skip
        if self.current_x is None or self.current_y is None:
            return

        # Calculate yaw angle (theta)
        siny_cosp = 2 * (self.imu_orientation.w * self.imu_orientation.z + self.imu_orientation.x * self.imu_orientation.y)
        cosy_cosp = 1 - 2 * (self.imu_orientation.y**2 + self.imu_orientation.z**2)
        theta = math.atan2(siny_cosp, cosy_cosp)  # In radians
        theta_degrees = math.degrees(theta)

        # Publish theta angle
        theta_msg = Float32()
        theta_msg.data = theta_degrees
        self.theta_publisher.publish(theta_msg)

        # Publish dynamic transform for odom -> base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.current_x
        transform.transform.translation.y = self.current_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = self.imu_orientation

        self.tf_broadcaster.sendTransform(transform)

        # Publish dynamic transform for base_link -> map
        base_link_to_map = TransformStamped()
        base_link_to_map.header.stamp = self.get_clock().now().to_msg()
        base_link_to_map.header.frame_id = 'base_link'
        base_link_to_map.child_frame_id = 'map'
        base_link_to_map.transform.translation.x = -self.current_x
        base_link_to_map.transform.translation.y = -self.current_y
        base_link_to_map.transform.translation.z = 0.0
        base_link_to_map.transform.rotation.x = 0.0
        base_link_to_map.transform.rotation.y = 0.0
        base_link_to_map.transform.rotation.z = 0.0
        base_link_to_map.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(base_link_to_map)

        # Publish pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = self.current_x
        pose_stamped.pose.position.y = -self.current_y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation = self.imu_orientation

        self.pose_publisher.publish(pose_stamped)
        self.get_logger().info(f"Published Pose: x={self.current_x}, y={-self.current_y}, theta={theta_degrees:.2f} degrees")

def main(args=None):
    rclpy.init(args=args)
    node = PoseCalculatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
