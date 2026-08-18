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
import os

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        self.get_logger().info("Starting Path Generator Node")

        self.workspace_dir = "/home/user/abd_ws"
        self.perimeter_dir = os.path.join(self.workspace_dir, "src/tests/tests/perimeter")
        self.perimeter_name = None
        self.file_path = None
        self.output_image = None

        self.row_spacing = 0.00002  # Distance between zigzag rows
        self.point_density = 15  # Number of points per curve segment

        self.name_sub = self.create_subscription(String, '/perimeter_name', self.name_callback, 10)

    def name_callback(self, msg):
        self.perimeter_name = msg.data.strip()
        self.file_path = os.path.join(self.perimeter_dir, self.perimeter_name, "gps_data.csv")
        self.output_image = os.path.join(self.perimeter_dir, self.perimeter_name, "coverage_paths.png")

        if not os.path.exists(self.file_path):
            self.get_logger().error(f" GPS data not found: {self.file_path}")
            return

        self.run()

    def run(self):
        try:
            coordinates = self.load_and_clean_data()
            boundary = self.create_polygon_boundary(coordinates)
            primary_waypoints, secondary_waypoints, tertiary_waypoints = self.generate_primary_secondary_tertiary_paths(boundary)

            self.save_waypoints_to_csv(primary_waypoints, os.path.join(self.perimeter_dir, self.perimeter_name, "primary_zigzag_path_waypoints.csv"))
            self.save_waypoints_to_csv(secondary_waypoints, os.path.join(self.perimeter_dir, self.perimeter_name, "secondary_zigzag_path_waypoints.csv"))
            self.save_waypoints_to_csv(tertiary_waypoints, os.path.join(self.perimeter_dir, self.perimeter_name, "tertiary_zigzag_path_waypoints.csv"))

            self.plot_all_paths(boundary, primary_waypoints, secondary_waypoints, tertiary_waypoints, coordinates)

            self.load_and_concatenate_waypoints(
                os.path.join(self.perimeter_dir, self.perimeter_name, "primary_zigzag_path_waypoints.csv"),
                os.path.join(self.perimeter_dir, self.perimeter_name, "secondary_zigzag_path_waypoints.csv"),
                os.path.join(self.perimeter_dir, self.perimeter_name, "tertiary_zigzag_path_waypoints.csv"),
                os.path.join(self.perimeter_dir, self.perimeter_name, "final_combined_waypoints.csv")
            )

            self.get_logger().info("Three-path zigzag generation completed successfully.")
        except Exception as e:
            self.get_logger().error(f"Error in path generation: {e}")

    def load_and_clean_data(self):
        gps_data = pd.read_csv(self.file_path)
        self.get_logger().info("GPS data loaded successfully.")
        gps_data = gps_data.drop_duplicates().dropna()
        latitudes = gps_data['Latitude']
        longitudes = gps_data['Longitude']
        coordinates = list(zip(longitudes, latitudes))
        self.get_logger().info(f"Cleaned GPS data: {len(coordinates)} points")
        return coordinates

    def create_polygon_boundary(self, coordinates):
        polygon = Polygon(coordinates)
        if not polygon.is_valid:
            self.get_logger().warning("Input polygon is invalid. Attempting to fix it.")
            polygon = polygon.buffer(0)
        if isinstance(polygon, MultiPolygon):
            polygon = max(polygon.geoms, key=lambda p: p.area)
        if not polygon.is_valid:
            raise ValueError("Polygon boundary is invalid and could not be fixed.")
        self.get_logger().info("Largest polygon boundary created successfully.")
        return polygon

    def save_waypoints_to_csv(self, waypoints, file_name):
        df = pd.DataFrame(waypoints, columns=["longitude", "latitude"])
        df.to_csv(file_name, index=False)
        self.get_logger().info(f"Waypoints saved to {file_name}")

    def generate_zigzag_path(self, boundary, offset=0.0):
        min_x, min_y, max_x, max_y = boundary.bounds
        waypoints = []
        x_values = np.arange(min_x + offset, max_x, self.row_spacing)
        for i, x in enumerate(x_values):
            line = LineString([(x, min_y), (x, max_y)])
            clipped_line = line.intersection(boundary)
            if not clipped_line.is_empty and clipped_line.geom_type == 'LineString':
                coords = list(clipped_line.coords)
                if i % 2 == 1:
                    coords = coords[::-1]
                waypoints.extend(coords)
        return waypoints

    def generate_primary_secondary_tertiary_paths(self, boundary):
        primary_waypoints = self.generate_zigzag_path(boundary)
        secondary_waypoints = self.generate_zigzag_path(boundary, offset=self.row_spacing / 3)
        tertiary_waypoints = self.generate_zigzag_path(boundary, offset=2 * self.row_spacing / 3)
        return primary_waypoints, secondary_waypoints, tertiary_waypoints

    def plot_all_paths(self, boundary, path1, path2, path3, coordinates):
        plt.figure(figsize=(8, 8))
        plt.plot(*boundary.exterior.xy, color='blue', label='Polygon Boundary')
        x1, y1 = zip(*path1)
        plt.plot(x1, y1, color='orange', label='Primary Zigzag Path')
        x2, y2 = zip(*path2)
        plt.plot(x2, y2, color='green', label='Secondary Zigzag Path')
        x3, y3 = zip(*path3)
        plt.plot(x3, y3, color='purple', label='Tertiary Zigzag Path')
        longitudes, latitudes = zip(*coordinates)
        plt.scatter(longitudes, latitudes, color='red', label='GPS Points')
        plt.title("Primary, Secondary, and Tertiary Zigzag Paths")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.legend()
        plt.grid()
        plt.savefig(self.output_image)
        self.get_logger().info(f"Paths plot saved to {self.output_image}")
        plt.close()

    def load_and_concatenate_waypoints(self, primary_file, secondary_file, tertiary_file, output_file):
        waypoints = []
        try:
            with open(secondary_file, 'r') as file:
                reader = csv.reader(file)
                next(reader)
                for row in reader:
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        waypoints.append((lon, lat))
                    except (ValueError, IndexError):
                        continue

            primary_waypoints = []
            with open(primary_file, 'r') as file:
                reader = csv.reader(file)
                next(reader)
                for row in reader:
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        primary_waypoints.insert(0, (lon, lat))
                    except (ValueError, IndexError):
                        continue
            waypoints.extend(primary_waypoints)

            with open(tertiary_file, 'r') as file:
                reader = csv.reader(file)
                next(reader)
                for row in reader:
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        waypoints.append((lon, lat))
                    except (ValueError, IndexError):
                        continue

            with open(output_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["longitude", "latitude"])
                writer.writerows(waypoints)
            self.get_logger().info(f"Waypoints successfully saved to {output_file}")

        except FileNotFoundError as e:
            self.get_logger().error(f"Waypoint file not found: {e}")

        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
