import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import math
import csv
import pyproj

# Helper functions for calculations
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

class NavigationAndControlNode(Node):
    def __init__(self):
        super().__init__('navigation_and_control')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/ackermann_steering_controller/reference_unstamped', 10
        )
        self.create_subscription(
            NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/calculated_pose', self.pose_callback, 10
        )
        self.theta_sub = self.create_subscription(
            Float32, '/theta_degrees', self.theta_callback, 10
        )

        # Load waypoints from CSV
        self.waypoints = self.load_waypoints(
            '/home/user/abd_ws/src/tests/tests/zigzag_paths4/combined_zigzag_path_waypoints.csv'
        )
        self.current_waypoint_index = 0

        # Robot state (local pose)
        self.current_x = None
        self.current_y = None
        self.current_theta = None  # in degrees

        # GPS state
        self.current_lat = None
        self.current_lon = None

        # Base reference for converting GPS to local coordinates.
        # This will be set from the first received GPS message.
        self.base_lat = None
        self.base_lon = None

        # pyproj Geod object for WGS84 ellipsoid
        self.geod = pyproj.Geod(ellps="WGS84")

    def load_waypoints(self, filepath):
        waypoints = []
        try:
            with open(filepath, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
                        # Assumes CSV rows are in the format: lon, lat
                        lat = float(row[1])
                        lon = float(row[0])
                        waypoints.append((lat, lon))
                    except (ValueError, IndexError):
                        self.get_logger().warn(f"Skipping invalid row {row_number}: {row}")
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file '{filepath}' not found.")
            exit()
        return waypoints

    def pose_callback(self, msg):
        # Update the robot's local position from the PoseStamped message.
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def gps_callback(self, msg):
        # Update the current GPS coordinates.
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        self.get_logger().info(
            f"GPS Callback: Current GPS - Lat: {self.current_lat}, Lon: {self.current_lon}"
        )
        # Set the base GPS coordinate if not already set.
        if self.base_lat is None or self.base_lon is None:
            self.base_lat = self.current_lat
            self.base_lon = self.current_lon
            self.get_logger().info(
                f"Base GPS set to: Lat: {self.base_lat}, Lon: {self.base_lon}"
            )

    def theta_callback(self, msg):
        # Update the robot's current heading (in degrees).
        self.current_theta = msg.data
        self.get_logger().debug(f"Theta Callback: Current Theta - {self.current_theta:.2f} degrees")

    def convert_gps_to_local(self, lat, lon):
        # Convert GPS coordinates (lat, lon) into local Cartesian coordinates (meters)
        if self.base_lat is None or self.base_lon is None:
            self.get_logger().warn("Base GPS position not initialized; cannot convert GPS to local coordinates.")
            return 0.0, 0.0
        
        # Compute east-west (x) displacement using the base latitude
        _, _, distance_x = self.geod.inv(self.base_lon, self.base_lat, lon, self.base_lat)
        # Compute north-south (y) displacement using the base longitude
        _, _, distance_y = self.geod.inv(self.base_lon, self.base_lat, self.base_lon, lat)
        
        # Adjust signs: if target is west or south of the base, the displacement is negative.
        if lon < self.base_lon:
            distance_x = -distance_x
        if lat < self.base_lat:
            distance_y = -distance_y

        self.get_logger().info(
            f"Converted GPS to Local: Lat {lat}, Lon {lon} -> x: {distance_x:.2f}, y: {distance_y:.2f}"
        )
        return distance_x, distance_y

    def navigate(self):
        # Ensure all necessary data is available
        if self.current_x is None or self.current_y is None or self.current_theta is None:
            self.get_logger().info("Waiting for valid pose and orientation data...")
            return

        if self.current_lat is None or self.current_lon is None:
            self.get_logger().info("Waiting for valid GPS data...")
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        target_lat, target_lon = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"Current Target: Lat {target_lat}, Lon {target_lon}")

        # Convert target waypoint to local coordinates (meters) using the fixed base.
        target_x, target_y = self.convert_gps_to_local(target_lat, target_lon)

        # Use the robot's current position (assumed to be already in the local frame)
        current_x = self.current_x
        current_y = self.current_y

        # Calculate the distance (in meters) and angle (in radians) to the target.
        distance = calculate_distance(current_x, current_y, target_x, target_y)
        angle_to_target = calculate_angle(current_x, current_y, target_x, target_y)
        angle_to_target_deg = math.degrees(angle_to_target)

        # Compute the steering error (alpha) between the robot's heading and the target angle.
        alpha = angle_to_target_deg - self.current_theta

        # Normalize alpha to the range [-180, 180]
        if alpha > 180:
            alpha -= 360
        elif alpha < -180:
            alpha += 360

        self.get_logger().info(
            f"Distance to Target: {distance:.2f} meters, Angle to Target: {alpha:.2f} degrees"
        )

        # Check if the waypoint is reached (using a threshold, e.g., 0.5 meters)
        if distance < 0.5:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
        else:
            self.control_robot(alpha)

    def control_robot(self, alpha):
        # Define steering parameters
        max_steering = 1.5  # Maximum steering value
        steering_interval = 0.01  # Steering increment
        turn = 10  # Maximum angle in degrees for normalization
        
        # Clamp alpha to [-turn, turn] and normalize to [-1, 1]
        clamped_alpha = max(-turn, min(alpha, turn))
        normalized_alpha = clamped_alpha / turn
        
        # Map the normalized value to the steering range
        steering = round(normalized_alpha * max_steering / steering_interval) * steering_interval

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = 1.0  # Constant speed (adjust as needed)
        twist.angular.z = steering

        self.get_logger().info(
            f"Publishing: Speed {twist.linear.x:.2f}, Steering {twist.angular.z:.2f}"
        )
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationAndControlNode()

    # Run navigate() periodically (e.g., at 10 Hz)
    timer_period = 0.1  # 0.1 seconds => 10 Hz
    node.create_timer(timer_period, node.navigate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

