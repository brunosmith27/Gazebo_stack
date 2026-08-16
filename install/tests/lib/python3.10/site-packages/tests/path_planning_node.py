import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json

class CoveragePathExecutor(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        self.vel_pub = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)
        self.robot_width = 1.0  # Width of the robot in meters
        self.robot_speed = 0.5  # Speed of the robot in m/s
        self.coverage_path = self.load_perimeter_and_generate_path()

    def load_perimeter_and_generate_path(self):
        with open('/home/user/abd_ws/src/tests/tests/perimeter_points.json', 'r') as f:
            perimeter = json.load(f)
        self.get_logger().info(f"Loaded Perimeter: {perimeter}")
        return self.generate_coverage_path(perimeter, self.robot_width)

    def generate_coverage_path(self, perimeter, robot_width):
        min_lat = min([p['latitude'] for p in perimeter])
        max_lat = max([p['latitude'] for p in perimeter])
        min_lon = min([p['longitude'] for p in perimeter])
        max_lon = max([p['longitude'] for p in perimeter])

        coverage_path = []
        current_lat = min_lat
        sweep_direction = 1

        while current_lat <= max_lat:
            if sweep_direction == 1:
                coverage_path.append((current_lat, min_lon))
                coverage_path.append((current_lat, max_lon))
            else:
                coverage_path.append((current_lat, max_lon))
                coverage_path.append((current_lat, min_lon))

            current_lat += robot_width * (1 / 111000)  # Convert meters to degrees
            sweep_direction *= -1

        self.get_logger().info(f"Generated Coverage Path: {coverage_path}")
        return coverage_path

    def execute_path(self):
        for point in self.coverage_path:
            self.get_logger().info(f"Moving to GPS Point: {point}")
            twist = Twist()
            twist.linear.x = self.robot_speed
            self.vel_pub.publish(twist)
            self.get_logger().info("Simulating movement to the point (adjust this with localization)")
            self.wait(1.0)  # Simulate movement delay (replace with real checks)

    def wait(self, duration):
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    executor = CoveragePathExecutor()
    try:
        executor.execute_path()
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

