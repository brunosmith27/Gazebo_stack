import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
from datetime import datetime
import threading
import matplotlib.pyplot as plt


class GPSRecorderNode(Node):
    def __init__(self):
        super().__init__('gps_recorder_node')

        # Subscribe to the GPS topic
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)

        # Variables
        self.recording = False
        self.file_path = '/home/user/abd_ws/src/tests/tests/gps_data2.csv'
        self.gps_points = []  # Store GPS points (latitude, longitude)

        # Open the CSV file and write the header
        with open(self.file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["Timestamp", "Latitude", "Longitude", "Altitude"])

        # Log initialization message
        self.get_logger().info("GPS Recorder Node initialized. Type 'start' to begin recording, 'stop' to save and plot, or 'exit' to quit.")

    def gps_callback(self, msg):
        """Callback for GPS data."""
        self.get_logger().info(f"GPS callback triggered. Lat: {msg.latitude}, Lon: {msg.longitude}, Alt: {msg.altitude}")
        if self.recording:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude

            # Save the point to the list
            self.gps_points.append((longitude, latitude))

            # Save directly to CSV
            with open(self.file_path, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([timestamp, latitude, longitude, altitude])

            self.get_logger().info(f"Recorded: {timestamp}, Lat: {latitude}, Lon: {longitude}, Alt: {altitude}")

    def start_recording(self):
        """Start recording GPS data."""
        self.recording = True
        self.get_logger().info("Started recording GPS data.")

    def stop_recording(self):
        """Stop recording GPS data and save the plot."""
        self.recording = False
        self.get_logger().info("Stopped recording GPS data. Saving plot...")
        self.plot_polygon()

    def plot_polygon(self):
        """Plot a polygon connecting all GPS points."""
        if len(self.gps_points) < 3:
            self.get_logger().warning("Not enough points to form a polygon (minimum 3 required).")
            return

        # Close the polygon by appending the first point at the end
        polygon_points = self.gps_points + [self.gps_points[0]]
        longitudes, latitudes = zip(*polygon_points)

        # Plot the polygon
        plt.figure(figsize=(8, 8))
        plt.plot(longitudes, latitudes, color='blue', linewidth=2, label='Polygon')
        plt.scatter(*zip(*self.gps_points), color='red', marker='o', label='GPS Points')

        # Add annotations for points
        for i, point in enumerate(self.gps_points):
            plt.text(point[0], point[1], f'{i + 1}', fontsize=8, color='green')

        plt.title("GPS Polygon Visualization")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.legend()
        output_path = '/home/user/abd_ws/src/tests/tests/gps_polygon_plot.png'
        plt.savefig(output_path)
        plt.close()  # Close the figure to free memory
        self.get_logger().info(f"Polygon plot saved to {output_path}")


def user_input_thread(node):
    """Separate thread for user input."""
    try:
        while rclpy.ok():
            command = input("Enter command ('start', 'stop', or 'exit'): ").strip().lower()

            if command == 'start':
                node.start_recording()
            elif command == 'stop':
                node.stop_recording()
            elif command == 'exit':
                node.get_logger().info("Exiting the program.")
                rclpy.shutdown()
                break
            else:
                node.get_logger().info("Invalid command. Type 'start', 'stop', or 'exit'.")
    except Exception as e:
        node.get_logger().error(f"Error in user input thread: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSRecorderNode()

    # Start the user input thread
    input_thread = threading.Thread(target=user_input_thread, args=(node,))
    input_thread.daemon = True  # Ensure the thread exits when the main program exits
    input_thread.start()

    try:
        # Keep spinning the node to process GPS messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
