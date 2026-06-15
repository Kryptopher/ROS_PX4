#!/usr/bin/env python3

import csv
import json
import shutil
from datetime import datetime
from pathlib import Path

import rclpy
from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String


class FlightLogger(Node):
    """Write synchronized PX4 state, payload angles, and mission events."""

    def __init__(self):
        super().__init__('flight_logger')
        self.declare_parameter('log_base_dir', str(Path.home() / 'logs'))
        self.declare_parameter('run_label', 'dds_mission')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('sample_rate_hz', 50.0)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        label = self.get_parameter('run_label').value
        self.mission_file = self.get_parameter('mission_file').value
        self.run_dir = Path(self.get_parameter('log_base_dir').value) / f'{stamp}_{label}'
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.local_position = None
        self.vehicle_status = None
        self.control_mode = None
        self.payload = None
        self.rows = 0
        self.events = 0

        self.flight_file = open(self.run_dir / 'flight.csv', 'w', newline='')
        self.flight_writer = csv.writer(self.flight_file)
        self.flight_writer.writerow([
            't', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading',
            'xy_valid', 'z_valid', 'armed', 'offboard', 'nav_state',
            'arming_state', 'failsafe', 'pitch_deg', 'roll_deg',
            'pitch_count', 'roll_count',
        ])
        self.event_file = open(self.run_dir / 'events.csv', 'w', newline='')
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(['t_wall', 't', 'event_type', 'detail'])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            lambda msg: setattr(self, 'local_position', msg), qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            lambda msg: setattr(self, 'local_position', msg), qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            lambda msg: setattr(self, 'vehicle_status', msg), qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            lambda msg: setattr(self, 'vehicle_status', msg), qos)
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode',
            lambda msg: setattr(self, 'control_mode', msg), qos)
        self.create_subscription(
            Float64MultiArray, '/payload/angles',
            lambda msg: setattr(self, 'payload', msg), qos)
        self.create_subscription(String, '/mission_executor/event', self._event_cb, 10)
        self.create_subscription(String, '/safety/event', self._event_cb, 10)
        self.create_timer(
            1.0 / float(self.get_parameter('sample_rate_hz').value),
            self._sample)
        self.get_logger().info(f'Flight logging to {self.run_dir}')

    def _elapsed(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.start_time

    def _event_cb(self, msg):
        event_type, _, detail = msg.data.partition('|')
        self.event_writer.writerow([
            datetime.now().isoformat(), f'{self._elapsed():.4f}',
            event_type, detail,
        ])
        self.event_file.flush()
        self.events += 1

    def _sample(self):
        p = self.local_position
        if p is None:
            return
        status = self.vehicle_status
        control = self.control_mode
        payload = list(self.payload.data) if self.payload is not None else []
        payload += [0.0] * (5 - len(payload))
        self.flight_writer.writerow([
            f'{self._elapsed():.4f}',
            p.x, p.y, p.z, p.vx, p.vy, p.vz, p.heading,
            int(p.xy_valid), int(p.z_valid),
            int(getattr(control, 'flag_armed', False)),
            int(getattr(control, 'flag_control_offboard_enabled', False)),
            getattr(status, 'nav_state', ''),
            getattr(status, 'arming_state', ''),
            int(getattr(status, 'failsafe', False)),
            *payload[:4],
        ])
        self.rows += 1
        if self.rows % 50 == 0:
            self.flight_file.flush()

    def destroy_node(self):
        metadata = {
            'run_dir': str(self.run_dir),
            'mission_file': self.mission_file,
            'flight_rows': self.rows,
            'event_rows': self.events,
        }
        with open(self.run_dir / 'metadata.json', 'w') as stream:
            json.dump(metadata, stream, indent=2)
        if self.mission_file and Path(self.mission_file).is_file():
            shutil.copy2(self.mission_file, self.run_dir)
        self.flight_file.close()
        self.event_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlightLogger()
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
