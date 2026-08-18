import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from sensor_msgs.msg import NavSatFix
import math
import csv
import numpy as np
import os

class NavigationAndControlNode(Node):
    def __init__(self):
        super().__init__('navigation_and_control')

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)

        # Subscribers
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)
        self.create_subscription(Float32, '/heading', self.heading_callback, 10)
        self.create_subscription(Float32, '/obstacle_speed_factor', self.speed_factor_callback, 10)
        self.create_subscription(String, '/start_navigation', self.start_callback, 10)
        self.create_subscription(String, '/stop_navigation', self.stop_callback, 10)

        self.speed_factor = 1.0
        self.last_factor_time = self.get_clock().now()

        self.navigation_active = False

        self.waypoints = self.load_waypoints('/home/user/abd_ws_2/src/tests/tests/zigzag_paths3/combined_zigzag_path_waypoints.csv')
        self.current_waypoint_index = 0

        self.current_lat = None
        self.current_lon = None
        self.heading = None

        self.k = -2
        self.v = 1.5

        self.create_timer(0.2, self.navigate)

        self.gps_log_file = open('/home/user/abd_ws_2/src/tests/tests/gps_log.csv', 'w', newline='')
        self.gps_writer = csv.writer(self.gps_log_file)
        self.gps_writer.writerow(['Latitude', 'Longitude'])

    def load_waypoints(self, filepath):
        latlon = []
        with open(filepath, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                try:
                    lat = float(row[1])
                    lon = float(row[0])
                    latlon.append((lat, lon))
                except:
                    continue

        interpolated = []
        for i in range(len(latlon) - 1):
            lat0, lon0 = latlon[i]
            lat1, lon1 = latlon[i + 1]
            for t in np.linspace(0, 1, 101):
                lati = (1 - t) * lat0 + t * lat1
                loni = (1 - t) * lon0 + t * lon1
                interpolated.append((lati, loni))
        return interpolated

    def speed_factor_callback(self, msg: Float32):
        self.speed_factor = max(0.0, min(1.0, msg.data))
        self.last_factor_time = self.get_clock().now()
        self.get_logger().info(f"Received speed factor: {self.speed_factor:.2f}")

    def gps_callback(self, msg):
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        if self.gps_writer:
            self.gps_writer.writerow([self.current_lat, self.current_lon])

    def heading_callback(self, msg):
        self.heading = msg.data

    def start_callback(self, msg):
        if msg.data.strip().lower() == 'start':
            self.navigation_active = True
            self.get_logger().info("Navigation started via bot command.")

    def stop_callback(self, msg):
        if msg.data.strip().lower() == 'stop':
            self.navigation_active = False
            self.get_logger().info("Navigation stopped via bot command.")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def latlon_to_xy(self, lat1, lon1, lat2, lon2):
        avg_lat_rad = math.radians((lat1 + lat2) / 2.0)
        dx = (lon2 - lon1) * 111000 * math.cos(avg_lat_rad)
        dy = (lat2 - lat1) * 111000
        return dx, dy

    def stanley_control(self, lat, lon, yaw_rad, v=1.5, lookahead_dist=2.0, search_window=3.0):
        if self.current_waypoint_index >= len(self.waypoints):
            return 0.0

        nearest_index = self.current_waypoint_index
        min_dist = float('inf')

        for i in range(self.current_waypoint_index, len(self.waypoints)):
            wp_lat, wp_lon = self.waypoints[i]
            dx, dy = self.latlon_to_xy(lat, lon, wp_lat, wp_lon)
            dist = math.hypot(dx, dy)
            if dist < min_dist and dist < search_window:
                min_dist = dist
                nearest_index = i
            elif dist > search_window:
                break

        lookahead_index = nearest_index
        for j in range(nearest_index + 1, len(self.waypoints)):
            dx, dy = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[j])
            dist = math.hypot(dx, dy)
            if dist >= lookahead_dist:
                lookahead_index = j
                break

        dx_path, dy_path = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[lookahead_index])
        path_yaw = math.atan2(dy_path, dx_path)

        heading_error = math.atan2(math.sin(path_yaw - yaw_rad), math.cos(path_yaw - yaw_rad))

        dx_ct, dy_ct = self.latlon_to_xy(lat, lon, *self.waypoints[nearest_index])
        cross_track_error = dx_ct * math.sin(yaw_rad) - dy_ct * math.cos(yaw_rad)

        steer_angle = heading_error + math.atan2(self.k * cross_track_error, v)
        steer_deg = max(-35, min(35, math.degrees(steer_angle)))

        self.get_logger().info(f"Cross-track error: {cross_track_error:.2f} m, Steering angle: {steer_deg:.2f}°")

        self.current_waypoint_index = nearest_index
        return steer_deg

    def navigate(self):
        if not self.navigation_active:
            return

        if None in (self.current_lat, self.current_lon, self.heading):
            return

        heading_deg = (self.heading - 180) % 360.0
        yaw_rad = math.radians(heading_deg)

        if self.current_waypoint_index >= len(self.waypoints) - 2:
            self.get_logger().info("Navigation complete.")
            return

        steering_angle = self.stanley_control(self.current_lat, self.current_lon, yaw_rad, v=self.v)
        self.control_robot(steering_angle)

    def control_robot(self, steering_angle):
        max_steering_units = 0.8
        max_steering_angle = 35.0

        steering = (max_steering_units * steering_angle / max_steering_angle)

        time_since_last = (self.get_clock().now() - self.last_factor_time).nanoseconds * 1e-9
        if time_since_last > 1.0:
            self.speed_factor = 1.0
            self.get_logger().warn("No speed factor received recently. Defaulting to 1.0")

        twist = Twist()
        twist.linear.x = self.v * self.speed_factor
        twist.angular.z = steering

        self.get_logger().info(f"Publishing: Speed {twist.linear.x:.2f} m/s, Steering {twist.angular.z:.2f} units")
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationAndControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.gps_log_file.close()

if __name__ == '__main__':
    main()
