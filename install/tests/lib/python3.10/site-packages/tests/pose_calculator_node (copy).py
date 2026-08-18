
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
import math
import pyproj

class PoseCalculatorNode(Node):
    def __init__(self):
        super().__init__('pose_calculator')

        # Subscribers
        self.create_subscription(Imu, '/camera/imu/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)

        # Publisher
        self.pose_publisher = self.create_publisher(PoseStamped, '/calculated_pose', 10)

        # Internal storage for sensor data
        self.gps_latitude = None
        self.gps_longitude = None
        self.previous_gps_time = None
        self.imu_orientation = None
        self.current_x = None
        self.current_y = None
        self.previous_x = None
        self.previous_y = None

        # GPS to local frame converter
        self.geod = pyproj.Geod(ellps='WGS84')
        self.base_latitude = None
        self.base_longitude = None

    def gps_callback(self, msg):
        # Extract GPS coordinates
        self.gps_latitude = msg.latitude
        self.gps_longitude = msg.longitude

        # If this is the first GPS reading, set it as the base reference
        if self.base_latitude is None or self.base_longitude is None:
            self.base_latitude = self.gps_latitude
            self.base_longitude = self.gps_longitude

        # Convert GPS to local X, Y coordinates (relative to the base reference)
        _, _, distance_x = self.geod.inv(self.base_longitude, self.base_latitude, self.gps_longitude, self.base_latitude)
        _, _, distance_y = self.geod.inv(self.base_longitude, self.base_latitude, self.base_longitude, self.gps_latitude)

        # Correct signs based on latitude and longitude
        if self.gps_longitude < self.base_longitude:
            distance_x = -distance_x
        if self.gps_latitude < self.base_latitude:
            distance_y = -distance_y

        self.previous_x = self.current_x
        self.previous_y = self.current_y
        self.current_x = distance_x
        self.current_y = distance_y
        self.previous_gps_time = self.get_clock().now()

    def imu_callback(self, msg):
        # Extract IMU orientation
        self.imu_orientation = msg.orientation

        # If GPS data is not available, skip publishing
        if self.current_x is None or self.current_y is None:
            return

        # Calculate theta (yaw angle) from IMU orientation
        siny_cosp = 2 * (self.imu_orientation.w * self.imu_orientation.z + self.imu_orientation.x * self.imu_orientation.y)
        cosy_cosp = 1 - 2 * (self.imu_orientation.y**2 + self.imu_orientation.z**2)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Interpolate position if previous GPS data is available
        if self.previous_x is not None and self.previous_y is not None:
            current_time = self.get_clock().now()
            time_delta = (current_time - self.previous_gps_time).nanoseconds / 1e9  # Convert to seconds
            if time_delta > 0:
                ratio = min(1.0, 1.0 / (time_delta * 100))  # Interpolate towards the latest GPS at 100 Hz
                interpolated_x = self.previous_x + ratio * (self.current_x - self.previous_x)
                interpolated_y = self.previous_y + ratio * (self.current_y - self.previous_y)
            else:
                interpolated_x = self.current_x
                interpolated_y = self.current_y
        else:
            interpolated_x = self.current_x
            interpolated_y = self.current_y

        # Publish pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = interpolated_x
        pose_stamped.pose.position.y = interpolated_y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation = self.imu_orientation

        self.pose_publisher.publish(pose_stamped)
        self.get_logger().info(f"Published Pose: x={interpolated_x}, y={interpolated_y}, theta={math.degrees(theta):.2f} degrees")

def main(args=None):
    rclpy.init(args=args)
    node = PoseCalculatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

