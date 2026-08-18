#!/usr/bin/env python3

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


class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        self.get_logger().info("Starting Path Generator Node")

        # File paths
        self.file_path = "/home/user/abd_ws/src/tests/tests/gps_data.csv"  # Input GPS data
        self.output_image = "primary_secondary_zigzag_path.png"  # Output image for path visualization

        # Path generation parameters
        self.row_spacing = 0.00006 # Distance between zigzag rows
        self.point_density = 10 # Number of points per curve segment


        # Run the path generation process
        self.run()

    def run(self):
        try:
            # Step 1: Load GPS data
            coordinates = self.load_and_clean_data()

            # Step 2: Create polygon boundary for the largest area
            boundary = self.create_polygon_boundary(coordinates)

            # Step 3: Generate primary zigzag path
            primary_waypoints, smoothed_primary_path = self.generate_zigzag_path(boundary)

            # Step 4: Generate secondary zigzag path
            secondary_waypoints, smoothed_secondary_path = self.generate_secondary_zigzag_path(boundary)

            # Step 5: Save waypoints to CSV
            self.save_waypoints_to_csv(smoothed_primary_path, "primary_zigzag_path_waypoints.csv")
            self.save_waypoints_to_csv(smoothed_secondary_path, "secondary_zigzag_path_waypoints.csv")

            # Step 6: Plot and save the paths
            self.plot_secondary_path(boundary, smoothed_primary_path, smoothed_secondary_path, coordinates)

            self.load_and_concatenate_waypoints("primary_zigzag_path_waypoints.csv","secondary_zigzag_path_waypoints.csv","combined_zigzag_path_waypoints.csv")

            self.get_logger().info("Waypoints zigzag path generation completed successfully.")
        except Exception as e:
            self.get_logger().error(f"Error in path generation: {e}")

    def load_and_clean_data(self):
        """Load GPS data and clean it."""
        gps_data = pd.read_csv(self.file_path)
        self.get_logger().info("GPS data loaded successfully.")

        # Remove duplicates and NaNs
        gps_data = gps_data.drop_duplicates().dropna()
        latitudes = gps_data['Latitude']
        longitudes = gps_data['Longitude']

        # Combine latitudes and longitudes into a list of coordinates
        coordinates = list(zip(longitudes, latitudes))
        self.get_logger().info(f"Cleaned GPS data: {len(coordinates)} points")

        return coordinates

    def create_polygon_boundary(self, coordinates):
        """Create a polygon boundary from GPS coordinates, considering the largest area."""
        polygon = Polygon(coordinates)

        # Validate the geometry
        if not polygon.is_valid:
            self.get_logger().warning("Input polygon is invalid. Attempting to fix it.")
            polygon = polygon.buffer(0)  # Attempt to fix invalid geometry

        # Handle MultiPolygon (if GPS points create disconnected areas)
        if isinstance(polygon, MultiPolygon):
            self.get_logger().info("Detected multiple areas. Extracting the largest one.")
            polygon = max(polygon.geoms, key=lambda p: p.area)

        if not polygon.is_valid:
            raise ValueError("Polygon boundary is invalid and could not be fixed.")

        self.get_logger().info("Largest polygon boundary created successfully.")
        return polygon

    def generate_zigzag_path(self, boundary):
        """Generate zigzag path with smooth curves inside the boundary."""
        min_x, min_y, max_x, max_y = boundary.bounds
        waypoints = []

        # Create zigzag lines spaced by `row_spacing`
        x_values = np.arange(min_x, max_x, self.row_spacing)
        for i, x in enumerate(x_values):
            line = LineString([(x, min_y), (x, max_y)])
            clipped_line = line.intersection(boundary)
            if not clipped_line.is_empty and clipped_line.geom_type == 'LineString':
                coords = list(clipped_line.coords)
                if i % 2 == 1:
                    coords = coords[::-1]  # Alternate direction for zigzag
                waypoints.extend(coords)

        smoothed_path = self.smooth_zigzag_path(waypoints, boundary)
        return waypoints, smoothed_path

    def generate_secondary_zigzag_path(self, boundary):
        """Generate a secondary zigzag path to fill the space between primary zigzag rows."""
        min_x, min_y, max_x, max_y = boundary.bounds
        secondary_waypoints = []

        # Offset the zigzag rows by half the row spacing
        x_values = np.arange(min_x + self.row_spacing / 2, max_x, self.row_spacing)
        for i, x in enumerate(x_values):
            line = LineString([(x, min_y), (x, max_y)])
            clipped_line = line.intersection(boundary)
            if not clipped_line.is_empty and clipped_line.geom_type == 'LineString':
                coords = list(clipped_line.coords)
                if i % 2 == 0:
                    coords = coords[::-1]  # Alternate direction for continuity
                secondary_waypoints.extend(coords)

        smoothed_secondary_path = self.smooth_zigzag_path(secondary_waypoints, boundary)
        return secondary_waypoints, smoothed_secondary_path

    def smooth_zigzag_path(self, waypoints, boundary):
        """Smooth transitions between zigzag lines using spline interpolation."""
        if len(waypoints) < 2:
            self.get_logger().warning("Not enough waypoints to smooth the path.")
            return waypoints

        waypoints = np.array(waypoints)
        x, y = waypoints[:, 0], waypoints[:, 1]

        # Generate smooth spline
        t = np.linspace(0, 1, len(x))
        spline_x = make_interp_spline(t, x, k=3)
        spline_y = make_interp_spline(t, y, k=3)

        t_smooth = np.linspace(0, 1, len(x) * self.point_density)
        smooth_x = spline_x(t_smooth)
        smooth_y = spline_y(t_smooth)

        smoothed_path = [point for point in zip(smooth_x, smooth_y) if boundary.contains(Point(point))]
        return smoothed_path

    def save_waypoints_to_csv(self, waypoints, file_name):
        """Save the waypoints to a CSV file."""
        df = pd.DataFrame(waypoints, columns=["longitude", "latitude"])
        df.to_csv(file_name, index=False)
        self.get_logger().info(f"Waypoints saved to {file_name}")

    def plot_secondary_path(self, boundary, smoothed_path1, smoothed_path2, coordinates):
        """Plot and save both the primary and secondary zigzag paths."""
        plt.figure(figsize=(8, 8))

        # Plot the polygon boundary
        plt.plot(*boundary.exterior.xy, color='blue', label='Polygon Boundary')

        # Plot the primary zigzag path
        smooth_x1, smooth_y1 = zip(*smoothed_path1)
        plt.plot(smooth_x1, smooth_y1, color='orange', label='Primary Zigzag Path')

        # Plot the secondary zigzag path
        smooth_x2, smooth_y2 = zip(*smoothed_path2)
        plt.plot(smooth_x2, smooth_y2, color='green', label='Secondary Zigzag Path')

        # Plot original GPS points
        longitudes, latitudes = zip(*coordinates)
        plt.scatter(longitudes, latitudes, color='red', label='GPS Points')

        plt.title("Primary and Secondary Zigzag Paths")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.legend()
        plt.grid()
        plt.savefig(self.output_image)
        self.get_logger().info(f"Paths plot saved to {self.output_image}")
        plt.close()



    def load_and_concatenate_waypoints(self, primary_file, secondary_file, output_file):
        """
        """
        waypoints = []
        try:
            # Load secondary waypoints (original order)
            with open(secondary_file, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        waypoints.append((lon, lat))  # Append in original order
                    except (ValueError, IndexError):
                        self.get_logger().warn(f"Skipping invalid row in {secondary_file}, row {row_number}: {row}")

            # Load primary waypoints (reverse order)
            primary_waypoints = []
            with open(primary_file, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        primary_waypoints.insert(0, (lon, lat))  # Prepend to reverse order
                    except (ValueError, IndexError):
                        self.get_logger().warn(f"Skipping invalid row in {primary_file}, row {row_number}: {row}")

            # Combine secondary (original) + primary (reversed)
            waypoints.extend(primary_waypoints)

            # Optionally save to CSV
            if output_file:
                try:
                    with open(output_file, mode='w', newline='') as file:
                        writer = csv.writer(file)
                        # Write header
                        writer.writerow(["longitude", "latitude"])
                        # Write waypoints
                        writer.writerows(waypoints)
                    self.get_logger().info(f"Waypoints successfully saved to {output_file}")
                except Exception as e:
                    self.get_logger().error(f"An error occurred while saving to CSV: {e}")

        except FileNotFoundError as e:
            self.get_logger().error(f"Waypoint file not found: {e}")
            exit()

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
