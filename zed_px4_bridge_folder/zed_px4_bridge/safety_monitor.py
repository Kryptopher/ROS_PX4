#!/usr/bin/env python3

import math

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleControlMode, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class SafetyMonitor(Node):
    """Independent PX4 DDS watchdog that can abort a mission and command RTL."""

    def __init__(self):
        super().__init__('safety_monitor')
        self.declare_parameter('max_altitude_m', 120.0)
        self.declare_parameter('warn_altitude_m', 100.0)
        self.declare_parameter('max_velocity_ms', 12.0)
        self.declare_parameter('comms_timeout_s', 2.0)
        self.declare_parameter('local_radius_m', 500.0)
        self.declare_parameter('command_rtl_on_abort', True)
        self.declare_parameter('test_mode', False)
        self.declare_parameter('startup_grace_s', 5.0)
        self.max_alt = float(self.get_parameter('max_altitude_m').value)
        self.warn_alt = float(self.get_parameter('warn_altitude_m').value)
        self.max_vel = float(self.get_parameter('max_velocity_ms').value)
        self.timeout = float(self.get_parameter('comms_timeout_s').value)
        self.radius = float(self.get_parameter('local_radius_m').value)
        self.command_rtl = bool(self.get_parameter('command_rtl_on_abort').value)
        self.test_mode = bool(self.get_parameter('test_mode').value)
        self.startup_grace = float(self.get_parameter('startup_grace_s').value)
        self.started_at = self.get_clock().now()

        self.position = None
        self.status = None
        self.control = None
        self.last_position_time = None
        self.abort_sent = False
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position_cb, qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._position_cb, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            lambda msg: setattr(self, 'status', msg), qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            lambda msg: setattr(self, 'status', msg), qos)
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode',
            lambda msg: setattr(self, 'control', msg), qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.abort_pub = self.create_publisher(String, '/mission/abort', 10)
        self.event_pub = self.create_publisher(String, '/safety/event', 10)
        self.create_timer(0.1, self._check)

    def _position_cb(self, msg):
        self.position = msg
        self.last_position_time = self.get_clock().now()

    def _armed(self):
        if self.control is not None:
            return bool(self.control.flag_armed)
        return self.status is not None and int(self.status.arming_state) == 2

    def _publish_event(self, event_type, detail):
        msg = String()
        msg.data = f'{event_type}|{detail}'
        self.event_pub.publish(msg)

    def _rtl(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _abort(self, reason):
        if self.abort_sent:
            return
        self.abort_sent = True
        self.get_logger().error(f'SAFETY ABORT: {reason}')
        msg = String()
        msg.data = reason
        self.abort_pub.publish(msg)
        self._publish_event('SAFETY_ABORT', reason)
        if self.command_rtl:
            self._rtl()

    def _check(self):
        if self.test_mode or not self._armed():
            self.abort_sent = False
            return
        startup_age = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if startup_age < self.startup_grace:
            return
        if self.last_position_time is None:
            self._abort('NO_LOCAL_POSITION')
            return
        age = (self.get_clock().now() - self.last_position_time).nanoseconds / 1e9
        if age > self.timeout:
            self._abort(f'LOCAL_POSITION_TIMEOUT {age:.2f}s')
            return
        p = self.position
        if not p.xy_valid or not p.z_valid:
            self._abort('LOCAL_POSITION_INVALID')
            return
        altitude = abs(float(p.z))
        speed = math.sqrt(float(p.vx) ** 2 + float(p.vy) ** 2 + float(p.vz) ** 2)
        distance = math.hypot(float(p.x), float(p.y))
        if altitude > self.max_alt:
            self._abort(f'ALTITUDE_LIMIT {altitude:.1f}m')
        elif speed > self.max_vel:
            self._abort(f'VELOCITY_LIMIT {speed:.1f}m/s')
        elif distance > self.radius:
            self._abort(f'LOCAL_GEOFENCE {distance:.1f}m')
        elif altitude > self.warn_alt:
            self._publish_event('SAFETY_WARNING', f'ALTITUDE_HIGH {altitude:.1f}m')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
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
