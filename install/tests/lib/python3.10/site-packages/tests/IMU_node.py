import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import serial
import serial.tools.list_ports
import time
import math

def parse_quaternion(line):
    """
    Parse the quaternion data from a JSON-like line.
    Expected format: '"quat_w":1.000, "quat_x":-0.000, "quat_y":-0.001, "quat_z":-0.002'
    """
    try:
        # Clean up the line and split into key-value pairs
        line = line.replace('"', '').strip()
        parts = {item.split(':')[0].strip(): float(item.split(':')[1]) for item in line.split(', ') if ':' in item}

        q0 = parts.get("quat_w", 0.0)
        q1 = parts.get("quat_x", 0.0)
        q2 = parts.get("quat_y", 0.0)
        q3 = parts.get("quat_z", 0.0)

        return q1, q2, q3, q0
    except Exception as e:
        print(f"Error parsing line: {line}. Error: {e}")
        return None

class QuaternionPublisher(Node):
    def __init__(self):
        super().__init__('quaternion_publisher')
        self.publisher_ = self.create_publisher(Quaternion, 'quaternion', 10)

        self.arduino_serial_number = "24333313131351A0A121"  # Replace with your Arduino's serial number
        self.arduino_port = self.find_arduino_port(self.arduino_serial_number)
        self.ser = None

        if self.arduino_port:
            self.ser = serial.Serial(self.arduino_port, 115200)
            self.get_logger().info(f"Connected to Arduino on port {self.arduino_port}")
        else:
            self.get_logger().error("Arduino with the specified serial number not found.")

        self.timer = self.create_timer(0.01, self.publish_quaternion)  # 10 Hz
        self.buffer = ""  # Buffer to accumulate data

    def find_arduino_port(self, serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return port.device  # e.g., '/dev/ttyACM0' or 'COM3'
        return None

    def publish_quaternion(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port is not open.")
            return

        try:
            raw_data = self.ser.read(self.ser.in_waiting or 1).decode('utf-8', errors='ignore')
            self.buffer += raw_data

            # Process complete lines
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                line = line.strip()
                if line:
                    quaternion = parse_quaternion(line)
                    if quaternion:
                        q1, q2, q3, q0 = quaternion
                        msg = Quaternion()
                        msg.x = q1
                        msg.y = q2
                        msg.z = q3
                        msg.w = q0

                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published Quaternion: [x: {q1:.3f}, y: {q2:.3f}, z: {q3:.3f}, w: {q0:.3f}]")
                    else:
                        self.get_logger().warn(f"Failed to parse line: {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    quaternion_publisher = QuaternionPublisher()

    try:
        rclpy.spin(quaternion_publisher)
    except KeyboardInterrupt:
        quaternion_publisher.get_logger().info("Node stopped by user.")
    finally:
        quaternion_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
