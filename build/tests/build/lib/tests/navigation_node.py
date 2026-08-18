import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import math
import csv
import pyproj

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

class NavigationAndControlNode(Node):
    def __init__(self):
        super().__init__('navigation_and_control')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)
        self.gps_heading_pub = self.create_publisher(Float32, '/gps_heading', 10)

        # Subscribers
        # self.pose_sub = self.create_subscription(PoseStamped, '/calculated_pose', self.pose_callback, 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)

        # Waypoints
        self.waypoints = self.load_waypoints('/home/user/abd_ws/src/tests/tests/zigzag_paths/combined_zigzag_path_waypoints.csv')
        self.current_waypoint_index = 0

        # Robot state
        self.current_x = None
        self.current_y = None
        self.current_theta = None

        # GPS state
        self.current_lat = None
        self.current_lon = None
        self.prev_lat = None
        self.prev_lon = None

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

    # def pose_callback(self, msg):
    #     self.current_x = msg.pose.position.x
    #     self.current_y = msg.pose.position.y

    def gps_callback(self, msg):
        new_lat = float(msg.latitude)
        new_lon = float(msg.longitude)

        if self.prev_lat is not None and self.prev_lon is not None:
            gps_heading = self.compute_heading_from_gps(self.prev_lat, self.prev_lon, new_lat, new_lon)
            if gps_heading is not None:
                self.current_theta = gps_heading
                self.get_logger().info(f"Updated heading from GPS: {self.current_theta:.2f} degrees")

                # Publish heading
                heading_msg = Float32()
                heading_msg.data = gps_heading
                self.gps_heading_pub.publish(heading_msg)
        else:
            self.get_logger().debug("Not enough GPS data yet to compute heading.")

        self.prev_lat = self.current_lat
        self.prev_lon = self.current_lon
        self.current_lat = new_lat
        self.current_lon = new_lon

    def compute_heading_from_gps(self, lat1, lon1, lat2, lon2):
        delta_lon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        x = math.sin(delta_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)

        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        heading = (bearing + 360) % 360
        return (bearing + 360) % 360

    def navigate(self):
        # if self.current_x is None or self.current_y is None:
        #     self.get_logger().info("Waiting for valid pose data...")
        #     return

        if self.current_lat is None or self.current_lon is None:
            self.get_logger().info("Waiting for valid GPS data...")
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        if self.current_theta is None:
            self.get_logger().warn("Heading not yet initialized. Moving forward slowly to estimate GPS heading.")
            twist = Twist()
            twist.linear.x = 0.5  # Move slowly forward
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        target_lat, target_lon = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"Current Target: Lat {target_lat}, Lon {target_lon}")

        # Convert GPS waypoints to local X, Y coordinates
        target_x, target_y = self.convert_gps_to_local(target_lat, target_lon)

        # Calculate distance and angle to the target
        distance = calculate_distance(-self.current_lon, -self.current_lat, -target_lon, -target_lat)
        alpha = calculate_angle(-self.current_lon, -self.current_lat, -target_lon, -target_lat)

        alpha = math.degrees(alpha)
        alpha = (alpha - self.current_theta)

        if alpha > 180:
            alpha -= 360
        elif alpha < -180:
            alpha += 360

        self.get_logger().info(f"Distance to Target: {distance*10000:.2f}, Angle to Target: {alpha:.2f} degrees")

        if distance < 0.000009:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
        else:
            self.control_robot(alpha)

    def convert_gps_to_local(self, lat, lon):
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn("Current position not initialized; cannot convert GPS to local coordinates.")
            return 0.0, 0.0

        if self.current_lat is None or self.current_lon is None:
            self.get_logger().warn("Base GPS position not initialized; cannot convert GPS to local coordinates.")
            return 0.0, 0.0

        base_lat = self.current_lat
        base_lon = self.current_lon 

        geod = pyproj.Geod(ellps="WGS84")
        _, _, distance_x = geod.inv(base_lon, base_lat, lon, base_lat)
        _, _, distance_y = geod.inv(base_lon, base_lat, base_lon, lat)

        if lon > base_lon:
            distance_x = -distance_x
        if lat < base_lat:
            distance_y = -distance_y

        self.get_logger().info(f"Converted GPS to Local: Lat {lat}, Lon {lon} -> x: {distance_x}, y: {distance_y}")
        return distance_x, distance_y

    def control_robot(self, alpha):
        max_steering = 1.5
        steering_interval = 0.01

        turn = 45
        clamped_alpha = max(-turn, min(alpha, turn))
        normalized_alpha = clamped_alpha / turn
        steering = round(normalized_alpha * max_steering / steering_interval) * steering_interval

        twist = Twist()
        twist.linear.x = 0.5  # Constant speed
        twist.angular.z = steering

        self.get_logger().info(f"Publishing: Speed {twist.linear.x:.2f}, Steering {twist.angular.z:.2f}")
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationAndControlNode()

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
