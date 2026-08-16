# === gazebo_navigation_node.py ===
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import math
import csv
import numpy as np

class GazeboNavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_and_control')

        self.cmd_vel_pub = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)

        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)
        self.create_subscription(Float32, '/heading', self.heading_callback, 10)
        self.create_subscription(Float32, '/obstacle_speed_factor', self.speed_factor_callback, 10)
        self.create_subscription(Float32, '/steering_correction_left', self.steering_left_callback, 10)
        self.create_subscription(Float32, '/steering_correction_right', self.steering_right_callback, 10)
        self.create_subscription(Float32, '/steering_correction_center', self.steering_center_callback, 10)

        self.waypoints = self.load_waypoints('/home/user/abd_ws_2/src/tests/tests/perimeter/abd2/final_combined_waypoints.csv')
        self.current_waypoint_index = 0

        self.current_lat = None
        self.current_lon = None
        self.heading = None

        self.k = -1.5
        self.v = 1.5

        self.speed_factor = 1.0
        self.last_factor_time = self.get_clock().now()

        self.left_steering = 0.0
        self.right_steering = 0.0
        self.center_steering = 0.0
        self.last_left_time = self.get_clock().now()
        self.last_right_time = self.get_clock().now()
        self.last_center_time = self.get_clock().now()

        self.avoidance_mode = False
        self.avoidance_start_time = self.get_clock().now()
        self.avoidance_duration = 2.0

        self.create_timer(0.2, self.navigate)

    def load_waypoints(self, filepath):
        latlon = []
        with open(filepath, 'r') as file:
            reader = csv.reader(file)
            next(reader, None)
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

    def steering_left_callback(self, msg: Float32):
        self.left_steering = msg.data
        self.last_left_time = self.get_clock().now()

    def steering_right_callback(self, msg: Float32):
        self.right_steering = msg.data
        self.last_right_time = self.get_clock().now()

    def steering_center_callback(self, msg: Float32):
        self.center_steering = msg.data
        self.last_center_time = self.get_clock().now()
        self.avoidance_mode = True
        self.avoidance_start_time = self.get_clock().now()

    def gps_callback(self, msg):
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)

    def heading_callback(self, msg):
        self.heading = msg.data

    def latlon_to_xy(self, lat1, lon1, lat2, lon2):
        avg_lat_rad = math.radians((lat1 + lat2) / 2.0)
        dx = (lon2 - lon1) * 111000 * math.cos(avg_lat_rad)
        dy = (lat2 - lat1) * 111000
        return dx, dy

    def stanley_control(self, lat, lon, yaw_rad, v=1.5, lookahead_dist=2, search_window=3):
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

        if nearest_index == self.current_waypoint_index:
            min_dist = float('inf')
            for i in range(self.current_waypoint_index, len(self.waypoints)):
                wp_lat, wp_lon = self.waypoints[i]
                dx, dy = self.latlon_to_xy(lat, lon, wp_lat, wp_lon)
                dist = math.hypot(dx, dy)
                if dist < min_dist:
                    min_dist = dist
                    nearest_index = i
                if dist > min_dist:
                    break

        lookahead_index = nearest_index
        for j in range(nearest_index + 1, len(self.waypoints)+50):
            dx, dy = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[j])
            dist = math.hypot(dx, dy)
            if dist >= lookahead_dist:
                lookahead_index = j
                break

        dx_path, dy_path = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[lookahead_index])
        path_yaw = math.atan2(dy_path, dx_path)

        heading_error = math.atan2(math.sin(path_yaw - yaw_rad), math.cos(path_yaw - yaw_rad))

        dx_ct, dy_ct = self.latlon_to_xy(lat, lon, *self.waypoints[nearest_index])
        cross_track_error = dx_ct * math.sin(path_yaw) - dy_ct * math.cos(path_yaw)

        steer_angle = heading_error + math.atan2(self.k * cross_track_error, v)
        steer_deg = max(-35, min(35, math.degrees(steer_angle)))

        self.current_waypoint_index = nearest_index
        return steer_deg

    def navigate(self):
        if None in (self.current_lat, self.current_lon, self.heading):
            return

        heading_deg = (self.heading - 180) % 360.0
        yaw_rad = math.radians(heading_deg)

        if self.current_waypoint_index >= len(self.waypoints) - 2:
            self.get_logger().info("Navigation complete.")
            return

        in_avoidance = (self.get_clock().now() - self.avoidance_start_time).nanoseconds * 1e-9 < self.avoidance_duration

        if in_avoidance:
            correction_deg = -math.degrees(self.center_steering)
            if correction_deg == 0.0:
                if self.left_steering > 0.0:
                    correction_deg = -math.degrees(self.left_steering)
                elif self.right_steering > 0.0:
                    correction_deg = math.degrees(self.right_steering)
            self.control_robot(0.0, correction_deg)
        else:
            self.avoidance_mode = False
            stanley_steering = self.stanley_control(self.current_lat, self.current_lon, yaw_rad, v=self.v)
            self.control_robot(stanley_steering, 0.0)

    def control_robot(self, steering_angle, obstacle_correction):
        max_steering_units = 0.8
        max_steering_angle_deg = 35.0

        combined_steering_deg = steering_angle + obstacle_correction
        combined_steering_deg = max(-max_steering_angle_deg, min(max_steering_angle_deg, combined_steering_deg))
        steering = (max_steering_units * combined_steering_deg / max_steering_angle_deg)

        time_since_speed = (self.get_clock().now() - self.last_factor_time).nanoseconds * 1e-9
        if time_since_speed > 1.0:
            self.speed_factor = 1.0

        twist = Twist()
        twist.linear.x = self.v * self.speed_factor
        twist.angular.z = steering

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
