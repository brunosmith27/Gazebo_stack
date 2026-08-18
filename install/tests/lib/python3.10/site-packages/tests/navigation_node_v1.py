import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import math
import csv

# Helper functions for pose calculations
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

class PoseNavigation(Node):
    def __init__(self):
        super().__init__('pose_navigation')

        # Publisher for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)

        # Subscription to the new calculated pose topic
        self.pose_sub = self.create_subscription(PoseStamped, '/calculated_pose', self.pose_callback, 10)

        # Load waypoints from CSV file
        self.waypoints = self.load_waypoints('/home/user/abd_ws/src/tests/tests/zigzag_path_waypoints.csv')
        self.current_waypoint_index = 0

        # Robot state
        self.current_x = None
        self.current_y = None
        self.current_theta = None

    def load_waypoints(self, filepath):
        waypoints = []
        try:
            with open(filepath, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        waypoints.append((x, y))
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

        orientation_q = msg.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(f"Current Pose: x={self.current_x}, y={self.current_y}, theta={math.degrees(self.current_theta):.2f} degrees")

        # Navigate to the next waypoint
        self.navigate_to_waypoints()

    def navigate_to_waypoints(self):
        if self.current_x is None or self.current_y is None or self.current_theta is None:
            self.get_logger().info("Waiting for valid pose data...")
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"Target Waypoint: ({target_x}, {target_y})")

        # Calculate distance and angle to the target
        distance = calculate_distance(self.current_x, self.current_y, target_x, target_y)
        angle_to_target = calculate_angle(self.current_x, self.current_y, target_x, target_y)

        self.get_logger().info(f"Distance to Target: {distance:.2f} meters")

        if distance < 0.1:  # If close to the waypoint
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
        else:
            self.control_robot(distance, angle_to_target)

    def control_robot(self, distance, angle_to_target):
        # Calculate heading error
        heading_error = angle_to_target - self.current_theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalize to [-pi, pi]

        # Adjust speed dynamically based on heading error and distance
        angular_velocity = max(-1.5, min(1.5, 2.0 * heading_error))  # Clamp angular speed
        linear_velocity = 0.6 if abs(heading_error) < 0.1 else 0.3  # Reduce speed for large heading errors

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        self.get_logger().info(f"Publishing: Linear {linear_velocity:.2f}, Angular {angular_velocity:.2f}")

        try:
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Error publishing commands: {e}")

def main(args=None):
    try:
        rclpy.init(args=args)
        pose_navigation = PoseNavigation()
        rclpy.spin(pose_navigation)
    except KeyboardInterrupt:
        print("Shutting down node.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
