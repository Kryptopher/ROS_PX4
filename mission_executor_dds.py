#!/usr/bin/env python3

import csv
import math
from pathlib import Path

from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleControlMode,
)


class MissionExecutorDDS(Node):
    def __init__(self):
        super().__init__('mission_executor_dds')

        self.declare_parameter('mission_file', str(Path.home() / 'mission.csv'))
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('auto_offboard', False)

        self.mission_file = self.get_parameter('mission_file').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_offboard = bool(self.get_parameter('auto_offboard').value)

        self.px4_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            self.px4_pub_qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.px4_pub_qos,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            self.px4_pub_qos,
        )
        self.event_pub = self.create_publisher(
            String,
            '/mission_executor/event',
            10,
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_cb,
            px4_qos,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            px4_qos,
        )
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_cb,
            px4_qos,
        )

        self.local_position = None
        self.vehicle_status = None
        self.vehicle_control_mode = None

        self.rows = self.load_mission(self.mission_file)

        self.start_time = None
        self.offboard_counter = 0
        self.mission_done = False
        self.last_active_row = None
        self.reported_mission_done = False
        self.reported_waiting_for_start = False
        self.reported_mission_started = False

        self.get_logger().info(f'Loaded mission: {self.mission_file}')
        self.get_logger().info(f'Rows: {len(self.rows)}')
        self.get_logger().warn('Safety: auto_arm and auto_offboard are disabled unless explicitly enabled.')

        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_cb)

    def load_mission(self, path):
        rows = []
        with open(path, 'r', newline='') as f:
            reader = csv.DictReader(f, delimiter='\t')
            required = ['t', 'type', 'mode', 'profile', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax']
            missing = [name for name in required if name not in (reader.fieldnames or [])]
            if missing:
                raise RuntimeError(
                    f'Mission file is missing columns {missing}. '
                    f'This executor expects a TAB-delimited .tsv file.'
                )

            for r in reader:
                rows.append({
                    't': float(r['t']),
                    'type': r['type'],
                    'mode': r['mode'],
                    'profile': r['profile'],
                    'x': float(r['x']),
                    'y': float(r['y']),
                    'z': float(r['z']),
                    'vx': float(r['vx']),
                    'vy': float(r['vy']),
                    'vz': float(r['vz']),
                    'ax': float(r['ax']),
                })

        if not rows:
            raise RuntimeError(f'Mission file has no rows: {path}')

        rows.sort(key=lambda x: x['t'])
        return rows

    def local_position_cb(self, msg):
        self.local_position = msg

    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg

    def vehicle_control_mode_cb(self, msg):
        self.vehicle_control_mode = msg

    def is_armed_and_offboard(self):
        # Prefer VehicleControlMode because it directly reports armed/offboard control state.
        if self.vehicle_control_mode is not None:
            return (
                bool(self.vehicle_control_mode.flag_armed)
                and bool(self.vehicle_control_mode.flag_control_offboard_enabled)
            )

        # Fallback to VehicleStatus:
        # arming_state 2 = ARMED, nav_state 14 = OFFBOARD.
        if self.vehicle_status is not None:
            return (
                int(self.vehicle_status.arming_state) == 2
                and int(self.vehicle_status.nav_state) == 14
            )

        return False

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_control_mode(self, mode):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()

        # Choose control layer.
        # For velocity rows, velocity=True.
        # For position/takeoff/hold rows, position=True.
        msg.position = mode == 'pos'
        msg.velocity = mode == 'vel'
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    def publish_setpoint(self, row):
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()

        # Mission convention:
        # x forward, y right, z positive UP.
        #
        # PX4 local frame is NED:
        # x forward/north, y right/east, z positive DOWN.
        px4_x = row['x']
        px4_y = row['y']
        px4_z = -row['z']

        if row['mode'] == 'pos':
            msg.position = [float(px4_x), float(px4_y), float(px4_z)]
            msg.velocity = [math.nan, math.nan, math.nan]
            msg.acceleration = [math.nan, math.nan, math.nan]

        elif row['mode'] == 'vel':
            # Velocity mode: publish velocity and leave position NaN.
            # Mission vz is positive up -> PX4 vz is positive down.
            msg.position = [math.nan, math.nan, math.nan]
            msg.velocity = [
                float(row['vx']),
                float(row['vy']),
                float(-row['vz']),
            ]
            msg.acceleration = [math.nan, math.nan, math.nan]

        else:
            msg.position = [math.nan, math.nan, math.nan]
            msg.velocity = [0.0, 0.0, 0.0]
            msg.acceleration = [math.nan, math.nan, math.nan]

        msg.yaw = math.nan
        msg.yawspeed = math.nan

        self.setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_event(self, event_type, detail=''):
        msg = String()
        msg.data = f'{event_type}|{detail}'
        self.event_pub.publish(msg)
        self.get_logger().warn(f'MISSION_EVENT {event_type}: {detail}')

    def arm(self):
        self.get_logger().warn('Sending ARM command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.get_logger().warn('Sending DISARM command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard_mode(self):
        self.get_logger().warn('Sending OFFBOARD mode command')
        # PX4 custom mode: param1=1, param2=6 for Offboard.
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def get_current_row(self, elapsed):
        current = self.rows[0]
        for row in self.rows:
            if elapsed >= row['t']:
                current = row
            else:
                break
        return current

    def timer_cb(self):
        # Wait until PX4 local position is available.
        if self.local_position is None:
            self.get_logger().info(
                'Waiting for /fmu/out/vehicle_local_position...',
                throttle_duration_sec=2.0,
            )
            return

        # Before PX4 is armed and in Offboard, keep streaming the first setpoint
        # so Offboard mode is available, but DO NOT start the mission clock yet.
        if self.start_time is None:
            row = self.rows[0]
            mode = row['mode'] if row['mode'] in ['pos', 'vel'] else 'pos'
            self.publish_offboard_control_mode(mode)
            self.publish_setpoint(row)

            self.offboard_counter += 1

            if self.offboard_counter == int(self.rate_hz * 2.0):
                if self.auto_offboard:
                    self.set_offboard_mode()
                else:
                    self.get_logger().warn('auto_offboard disabled. Switch to Offboard manually when ready.')

                if self.auto_arm:
                    self.arm()
                else:
                    self.get_logger().warn('auto_arm disabled. Arm manually when ready.')

            if not self.is_armed_and_offboard():
                if not self.reported_waiting_for_start:
                    self.get_logger().warn(
                        'Waiting for PX4 to be armed and in Offboard before starting mission timer...'
                    )
                    self.publish_event('WAITING_FOR_ARM_OFFBOARD', 'mission timer not started')
                    self.reported_waiting_for_start = True
                return

            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.reported_waiting_for_start = False
            self.get_logger().warn('PX4 is armed and in Offboard. Starting mission timer at t=0.')

            if not self.reported_mission_started:
                self.publish_event('MISSION_STARTED', 'PX4 armed and Offboard; t=0')
                self.reported_mission_started = True

        elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        row = self.get_current_row(elapsed)

        if row['type'] == 'end':
            if self.last_active_row is None:
                self.get_logger().error('Mission reached end but no previous active setpoint exists.')
                return

            self.get_logger().warn(
                'MISSION COMPLETE - holding final setpoint. Land manually or keep holding.',
                throttle_duration_sec=5.0,
            )
            if not self.reported_mission_done:
                self.publish_event('MISSION_COMPLETE_HOLDING', 'holding final setpoint')
            self.reported_mission_done = True

            row = self.last_active_row
        else:
            self.last_active_row = row

        # Always stream OffboardControlMode and TrajectorySetpoint.
        # IMPORTANT: even after mission end, keep publishing the final setpoint.
        # If this stops while airborne, PX4 exits Offboard and triggers failsafe.
        mode = row['mode'] if row['mode'] in ['pos', 'vel'] else 'pos'
        self.publish_offboard_control_mode(mode)
        self.publish_setpoint(row)


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorDDS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
