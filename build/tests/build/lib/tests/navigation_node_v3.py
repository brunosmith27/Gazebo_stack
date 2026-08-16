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

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/calculated_pose', self.pose_callback, 10)
        self.theta_sub = self.create_subscription(Float32, '/theta_degrees', self.theta_callback, 10)

        # Waypoints from CSV file
        self.waypoints = self.load_waypoints('/home/user/abd_ws/src/tests/tests/zigzag_paths/combined_zigzag_path_waypoints.csv')  # Update this path
        self.current_waypoint_index = 0

        # Robot state
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.theta = None


        # GPS state
        self.current_lat = None  # Initialize to None
        self.current_lon = None  # Initialize to None

    def load_waypoints(self, filepath):
        waypoints = []
        try:
            with open(filepath, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
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
        # Extract pose information from PoseStamped message
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        #self.get_logger().info(f"Pose Callback: Current Position - x: {self.current_x}, y: {self.current_y}")

    def gps_callback(self, msg):
        # Extract GPS information from NavSatFix message
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        self.get_logger().info(f"GPS Callback: Current GPS - Lat: {self.current_lat}, Lon: {self.current_lon}")

    def theta_callback(self, msg):
        # Extract the current robot heading angle
        self.current_theta = msg.data
        self.get_logger().debug(f"Theta Callback: Current Theta - {self.current_theta:.2f} degrees")

    def navigate(self):
        if self.current_x is None or self.current_y is None or self.current_theta is None:
            self.get_logger().info("Waiting for valid pose and orientation data...")
            return

        if self.current_lon is None or self.current_lat is None:
            self.get_logger().info("Waiting for valid GPS data...")
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        target_lat, target_lon = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"Current Target: Lat {target_lat}, Lon {target_lon}")

        # Convert GPS waypoints to local X, Y coordinates
        target_x, target_y = self.convert_gps_to_local(target_lat, target_lon)
        # self.get_logger().info(f"Converted Target to Local Coordinates: x: {target_x}, y: {target_y}")

        # Calculate distance and angle to the target
        # distance = calculate_distance(self.current_x, self.current_y, target_x, target_y)
        # alpha = calculate_angle(self.current_x, self.current_y, target_x, target_y)
        
        distance = calculate_distance(-self.current_lon, -self.current_lat, -target_lon,	-target_lat)
        alpha = calculate_angle(-self.current_lon, -self.current_lat, -target_lon,	-target_lat)
        

        # Calculate alpha angle
        # alpha = angle_to_target - self.current_theta
        # alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # Normalize to [-pi, pi]
        # self.current_theta = 90 - self.current_theta
        
        # if self.current_theta <-90:

        #     self.theta = -(270 + self.current_theta)
        # else:
        #     self.theta = 90 - self.current_theta
        alpha = math.degrees(alpha)
        alpha = (alpha - self.current_theta)

        if alpha > 180:
            alpha = alpha -360
        elif alpha <-180:
            alpha = 360 + alpha

        self.get_logger().info(f"Distance to Target: {distance*10000:.2f}, Angle to Target: {alpha:.2f} degrees")
        #self.get_logger().info(f"Alpha angle: {alpha:.2f} degrees")

        if distance < 0.000009:  # Threshold for reaching the waypoint
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
        else:
            self.control_robot(alpha)

    def convert_gps_to_local(self, lat, lon):
        # Convert GPS coordinates to local Cartesian coordinates
        # Assuming self.current_x and self.current_y are initialized
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn("Current position not initialized; cannot convert GPS to local coordinates.")
            return 0.0, 0.0
        
        if self.current_lat is None or self.current_lon is None:
            self.get_logger().warn("Base GPS position not initialized; cannot convert GPS to local coordinates.")
            return 0.0, 0.0

        # Using a reference point as the origin (base GPS coordinates)
        base_lat = self.current_lat  # Assuming the first waypoint is the reference
        base_lon = self.current_lon 

        geod = pyproj.Geod(ellps="WGS84")
        _, _, distance_x = geod.inv(base_lon, base_lat, lon, base_lat)
        _, _, distance_y = geod.inv(base_lon, base_lat, base_lon, lat)

        # Adjust for signs
        if lon > base_lon:
            distance_x = -distance_x
        if lat < base_lat:
            distance_y = -distance_y

        self.get_logger().info(f"Converted GPS to Local: Lat {lat}, Lon {lon} -> x: {distance_x}, y: {distance_y}")
        return distance_x, distance_y

    def control_robot(self, alpha):
        # Calculate steering dynamically based on alpha angle
        max_steering = 1.5  # Maximum steering value
        steering_interval = 0.01  # Steering increment

        # # Map the alpha angle to a steering value
        # if math.degrees(alpha) >= 0:  # Right/East side
        #     normalized_alpha = min(math.degrees(alpha) / 180, 1.0)  # Normalize alpha to [0, 1]
        #     steering = round(normalized_alpha * max_steering / steering_interval) * steering_interval
        # else:  # Left/West side
        #     normalized_alpha = min(math.degrees(-alpha) / 180, 1.0)  # Normalize alpha to [0, 1]
        #     steering = -round(normalized_alpha * max_steering / steering_interval) * steering_interval

        # # Map the alpha angle to a steering value
        # if alpha >= 0:  # Right/East side
        #     normalized_alpha = min(alpha / 180, 1.0)  # Normalize alpha to [0, 1]
        #     steering = round(normalized_alpha * max_steering / steering_interval) * steering_interval
        # else:  # Left/West side
        #     normalized_alpha = min(-alpha / 180, 1.0)  # Normalize alpha to [0, 1]
        #     steering = -round(normalized_alpha * max_steering / steering_interval) * steering_interval

        turn = 90
        


        # This means any angle less than -45 becomes -45 and any angle greater than 45 becomes 45.
        clamped_alpha = max(-turn, min(alpha, turn))

        # Normalize the clamped alpha so that -45 maps to -1.0 and 45 maps to 1.0
        normalized_alpha = clamped_alpha / turn  # Now in range [-1, 1]

        # Map the normalized value to the steering range
        steering = round(normalized_alpha * max_steering / steering_interval) * steering_interval

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = 1.0  # Constant speed
        twist.angular.z = steering

        self.get_logger().info(f"Publishing: Speed {twist.linear.x:.2f}, Steering {twist.angular.z:.2f}")
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationAndControlNode()
    # node.navigate()

    def timer_callback():
        node.navigate()

    timer_period = 0.01  # 10 Hz
    node.create_timer(timer_period, timer_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
