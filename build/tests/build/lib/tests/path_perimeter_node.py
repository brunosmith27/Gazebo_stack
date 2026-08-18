import csv
import pandas as pd
import matplotlib.pyplot as plt
from shapely import Point
from shapely.geometry import MultiPoint, LineString, Polygon, MultiPolygon
from shapely.ops import unary_union
from scipy.interpolate import CubicSpline, make_interp_spline
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import os
from datetime import datetime

class PathPerimeterRecorder(Node):
    def __init__(self):
        super().__init__('path_perimeter_node')
        self.get_logger().info("Starting Path Perimeter Node")

        self.workspace_dir = "/home/user/abd_ws"
        self.perimeter_dir = os.path.join(self.workspace_dir, "src/tests/tests/perimeter")
        self.perimeter_name = None
        self.gps_file_path = None
        self.plot_output_path = None

        self.gps_points = []
        self.recording = False

        # Subscriptions
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)
        self.name_sub = self.create_subscription(String, '/perimeter_name', self.name_callback, 10)
        self.command_sub = self.create_subscription(String, '/perimeter_cmd', self.command_callback, 10)

    def name_callback(self, msg):
        self.perimeter_name = msg.data.strip()
        dir_path = os.path.join(self.perimeter_dir, self.perimeter_name)
        os.makedirs(dir_path, exist_ok=True)
        self.gps_file_path = os.path.join(dir_path, "gps_data.csv")
        self.plot_output_path = os.path.join(dir_path, "gps_polygon_plot.png")

        # Reset state
        self.gps_points.clear()
        self.recording = False

        # Initialize CSV file
        with open(self.gps_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["Timestamp", "Latitude", "Longitude", "Altitude"])
        self.get_logger().info(f"Perimeter '{self.perimeter_name}' ready. GPS recording initialized.")

    def command_callback(self, msg):
            command = msg.data.strip().lower()
            if command == 'start':
                self.start_recording()
            elif command == 'stop':
                self.stop_recording()
            elif command == 'exit':
                self.get_logger().info("Received 'exit' command. Shutting down node.")
                rclpy.shutdown()
            else:
                self.get_logger().info("Invalid command. Use 'start', 'stop', or 'exit'.")

    def gps_callback(self, msg):
        if self.recording and self.gps_file_path:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
            self.gps_points.append((longitude, latitude))

            with open(self.gps_file_path, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([timestamp, latitude, longitude, altitude])

            self.get_logger().info(f"Recorded: {timestamp}, Lat: {latitude}, Lon: {longitude}, Alt: {altitude}")

    def start_recording(self):
        self.recording = True
        self.get_logger().info("Started recording GPS data.")

    def stop_recording(self):
        self.recording = False
        self.get_logger().info("Stopped recording GPS data. Saving plot...")
        self.plot_polygon()

    def plot_polygon(self):
        if len(self.gps_points) < 3:
            self.get_logger().warning("Not enough points to form a polygon (minimum 3 required).")
            return

        polygon_points = self.gps_points + [self.gps_points[0]]
        longitudes, latitudes = zip(*polygon_points)

        plt.figure(figsize=(8, 8))
        plt.plot(longitudes, latitudes, color='blue', linewidth=2, label='Polygon')
        plt.scatter(*zip(*self.gps_points), color='red', marker='o', label='GPS Points')

        for i, point in enumerate(self.gps_points):
            plt.text(point[0], point[1], f'{i + 1}', fontsize=8, color='green')

        plt.title("GPS Polygon Visualization")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.legend()
        plt.grid()
        plt.savefig(self.plot_output_path)
        plt.close()
        self.get_logger().info(f"Polygon plot saved to {self.plot_output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = PathPerimeterRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
