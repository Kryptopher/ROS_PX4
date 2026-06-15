#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

try:
    import pigpio
except ImportError:
    pigpio = None


QUAD_TABLE = {
    (0b00, 0b01): 1, (0b01, 0b11): 1,
    (0b11, 0b10): 1, (0b10, 0b00): 1,
    (0b00, 0b10): -1, (0b10, 0b11): -1,
    (0b11, 0b01): -1, (0b01, 0b00): -1,
}


class PayloadEncoder(Node):
    """Publish pitch/roll encoder angles without MAVROS-specific messages."""

    def __init__(self):
        super().__init__('payload_encoder')
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('pitch_a', 6)
        self.declare_parameter('pitch_b', 13)
        self.declare_parameter('roll_a', 19)
        self.declare_parameter('roll_b', 26)
        self.declare_parameter('ppr', 1000)
        self.declare_parameter('count_mode', 4)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('min_pulse_us', 300)

        self.pitch_pins = (
            self.get_parameter('pitch_a').value,
            self.get_parameter('pitch_b').value,
        )
        self.roll_pins = (
            self.get_parameter('roll_a').value,
            self.get_parameter('roll_b').value,
        )
        counts_per_rev = (
            self.get_parameter('ppr').value
            * self.get_parameter('count_mode').value
            * self.get_parameter('gear_ratio').value
        )
        self.deg_per_count = 360.0 / counts_per_rev
        self.counts = {'pitch': 0, 'roll': 0}
        self.errors = {'pitch': 0, 'roll': 0}
        self.levels = {}

        if pigpio is None:
            raise RuntimeError('python3-pigpio is not installed')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('pigpiod is not running')

        min_pulse = self.get_parameter('min_pulse_us').value
        for pin in self.pitch_pins + self.roll_pins:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_glitch_filter(pin, min_pulse)
            self.levels[pin] = self.pi.read(pin)

        self.states = {
            'pitch': self._state(self.pitch_pins),
            'roll': self._state(self.roll_pins),
        }
        self.callbacks = []
        for pin in self.pitch_pins:
            self.callbacks.append(
                self.pi.callback(pin, pigpio.EITHER_EDGE, self._pitch_cb))
        for pin in self.roll_pins:
            self.callbacks.append(
                self.pi.callback(pin, pigpio.EITHER_EDGE, self._roll_cb))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(
            Float64MultiArray, '/payload/angles', qos)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'Payload encoder running at {rate:.0f} Hz; '
            f'deg/count={self.deg_per_count:.5f}')

    def _state(self, pins):
        return (self.levels[pins[0]] << 1) | self.levels[pins[1]]

    def _update(self, axis, pins, gpio, level):
        self.levels[gpio] = level
        new_state = self._state(pins)
        delta = QUAD_TABLE.get((self.states[axis], new_state), 0)
        if delta == 0 and new_state != self.states[axis]:
            self.errors[axis] += 1
        self.counts[axis] += delta
        self.states[axis] = new_state

    def _pitch_cb(self, gpio, level, tick):
        self._update('pitch', self.pitch_pins, gpio, level)

    def _roll_cb(self, gpio, level, tick):
        self._update('roll', self.roll_pins, gpio, level)

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = [
            self.counts['pitch'] * self.deg_per_count,
            self.counts['roll'] * self.deg_per_count,
            float(self.counts['pitch']),
            float(self.counts['roll']),
            self.deg_per_count,
        ]
        self.publisher.publish(msg)

    def destroy_node(self):
        for callback in self.callbacks:
            callback.cancel()
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PayloadEncoder()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if not isinstance(exc, KeyboardInterrupt):
            print(f'[payload_encoder] {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
