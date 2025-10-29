import threading
import socket
import re
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


class ServoJointStateBridge(Node):
    """
    A ROS2 node that bridges servo angle readings from a TCP socket
    to /joint_states for visualization via robot_state_publisher and RViz2.

    Parameters:
    - bind_host: str (default: 0.0.0.0)
    - bind_port: int (default: 3333)
    - left_joint_name: str (default: "Revolute 2")
    - right_joint_name: str (default: "Revolute 4")
    - angle_units: str ("degrees" or "radians", default: "degrees")
    - left_sign: float (default: 1.0)
    - right_sign: float (default: 1.0)

    Expected TCP message formats (newline-delimited recommended):
    - "L:45,R:30" or "l=45,r=30"
    - "45,30" (assumed order: left,right)
    - Any message containing two numeric values will be interpreted as left,right.
    """

    def __init__(self):
        super().__init__('servo_joint_state_bridge')

        # Declare parameters
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('bind_port', 3333)
        self.declare_parameter('left_joint_name', 'Revolute 2')
        self.declare_parameter('right_joint_name', 'Revolute 4')
        self.declare_parameter('angle_units', 'degrees')  # or 'radians'
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)

        # Resolve parameters
        self.bind_host: str = self.get_parameter('bind_host').get_parameter_value().string_value
        self.bind_port: int = self.get_parameter('bind_port').get_parameter_value().integer_value
        self.left_joint_name: str = self.get_parameter('left_joint_name').get_parameter_value().string_value
        self.right_joint_name: str = self.get_parameter('right_joint_name').get_parameter_value().string_value
        self.angle_units: str = self.get_parameter('angle_units').get_parameter_value().string_value.lower()
        self.left_sign: float = self.get_parameter('left_sign').get_parameter_value().double_value
        self.right_sign: float = self.get_parameter('right_sign').get_parameter_value().double_value

        # Publisher for joint states
        qos = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos)

        # Publish an initial zero state so RViz has something immediately
        self.publish_joint_state(0.0, 0.0)

        # Start TCP server in a background thread
        self._server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self._server_thread.start()
        self.get_logger().info(f"ServoJointStateBridge listening on {self.bind_host}:{self.bind_port}"
                               f" | joints=('{self.left_joint_name}', '{self.right_joint_name}')"
                               f" | units={self.angle_units}")

    def publish_joint_state(self, left_angle_rad: float, right_angle_rad: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.left_joint_name, self.right_joint_name]
        msg.position = [left_angle_rad, right_angle_rad]
        self.joint_pub.publish(msg)

    def _server_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self.bind_host, self.bind_port))
            srv.listen(1)
        except Exception as e:
            self.get_logger().error(f"Failed to bind TCP server on {self.bind_host}:{self.bind_port} -> {e}")
            return

        self.get_logger().info("TCP server ready; waiting for client...")
        buffer = ''
        while rclpy.ok():
            try:
                srv.settimeout(1.0)
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                with conn:
                    self.get_logger().info(f"Client connected: {addr}")
                    conn.settimeout(1.0)
                    while rclpy.ok():
                        try:
                            data = conn.recv(1024)
                        except socket.timeout:
                            continue
                        if not data:
                            self.get_logger().info("Client disconnected")
                            break
                        buffer += data.decode(errors='ignore')
                        # Process by lines
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if not line:
                                continue
                            parsed = self._parse_angles(line)
                            if parsed is None:
                                self.get_logger().warn(f"Failed to parse angles from: '{line}'")
                                try:
                                    conn.sendall(b"ERR\n")
                                except Exception:
                                    pass
                                continue
                            left, right = parsed
                            # Unit conversion and sign application
                            if self.angle_units == 'degrees':
                                left = math.radians(left)
                                right = math.radians(right)
                            elif self.angle_units != 'radians':
                                self.get_logger().warn(f"Unknown angle_units '{self.angle_units}', assuming radians")
                            left *= self.left_sign
                            right *= self.right_sign
                            self.publish_joint_state(left, right)
                            try:
                                conn.sendall(b"OK\n")
                            except Exception:
                                pass
            except Exception as e:
                self.get_logger().error(f"Server loop error: {e}")
                continue

        try:
            srv.close()
        except Exception:
            pass

    @staticmethod
    def _parse_angles(text: str) -> Optional[Tuple[float, float]]:
        """
        Try multiple formats:
        - With keys: L:45,R:30 | l=45 r=30
        - CSV: 45,30
        - Any two numbers anywhere in the string
        Returns (left, right) in the message's native units.
        """
        # Keyed format
        keyed = re.findall(r"([lLrR])\s*[:=]\s*(-?\d+(?:\.\d+)?)", text)
        if keyed:
            vals = {k.lower(): float(v) for k, v in keyed}
            if 'l' in vals and 'r' in vals:
                return vals['l'], vals['r']
        # CSV format
        if ',' in text:
            parts = [p.strip() for p in text.split(',')]
            if len(parts) >= 2:
                try:
                    left = float(parts[0])
                    right = float(parts[1])
                    return left, right
                except ValueError:
                    pass
        # Any two numbers
        nums = re.findall(r"-?\d+(?:\.\d+)?", text)
        if len(nums) >= 2:
            try:
                return float(nums[0]), float(nums[1])
            except ValueError:
                return None
        return None


def main(args=None):
    rclpy.init(args=args)
    node = ServoJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
