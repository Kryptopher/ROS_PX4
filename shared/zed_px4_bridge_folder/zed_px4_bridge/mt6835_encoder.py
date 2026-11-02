#!/usr/bin/env python3

import re
import threading

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


ANGLE_PATTERN = re.compile(
    r'Encoder 1:\s*(-?\d+(?:\.\d+)?)\s*deg\s*\|\s*'
    r'Encoder 2:\s*(-?\d+(?:\.\d+)?)\s*deg'
)


class MT6835Encoder(Node):
    """Read the Nano's angle stream and publish both angles to ROS."""

    def __init__(self):
        super().__init__('mt6835_encoder')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        self.port = str(self.get_parameter('port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(
            Float64MultiArray, '/mt6835/angles', qos)
        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(
            target=self._read_serial, daemon=True)
        self.reader_thread.start()
        self.get_logger().info(
            f'Reading MT6835 angles from {self.port} at {self.baud_rate} baud')

    def _read_serial(self):
        while not self.stop_event.is_set():
            try:
                with serial.Serial(
                    self.port, self.baud_rate, timeout=1.0
                ) as connection:
                    self.get_logger().info(f'Connected to Nano on {self.port}')
                    while not self.stop_event.is_set():
                        line = connection.readline().decode(
                            'ascii', errors='replace').strip()
                        match = ANGLE_PATTERN.search(line)
                        if match is None:
                            continue

                        message = Float64MultiArray()
                        message.data = [
                            float(match.group(1)),
                            float(match.group(2)),
                        ]
                        self.publisher.publish(message)
            except (serial.SerialException, OSError) as error:
                if not self.stop_event.is_set():
                    self.get_logger().warning(
                        f'Cannot read {self.port}: {error}; retrying in 2 seconds')
                    self.stop_event.wait(2.0)

    def destroy_node(self):
        self.stop_event.set()
        self.reader_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MT6835Encoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
