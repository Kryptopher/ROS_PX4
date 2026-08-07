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

        self.declare_parameter(
            'mission_file',
            str(Path.home() / 'ROS_PX4' / 'shared' / 'missions' / 'Default.tsv'))
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('auto_offboard', False)
        self.declare_parameter('auto_land', False)
        self.declare_parameter('auto_disarm_after_land', False)
        self.declare_parameter('force_disarm_after_land_timeout_s', 0.0)
        self.declare_parameter('max_velocity_ms', 12.0)
        self.declare_parameter('start_position_tolerance_m', 0.12)
        self.declare_parameter('start_velocity_tolerance_ms', 0.15)
        self.declare_parameter('start_settle_time_s', 1.5)

        self.mission_file = self.get_parameter('mission_file').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_offboard = bool(self.get_parameter('auto_offboard').value)
        self.auto_land = bool(self.get_parameter('auto_land').value)
        self.auto_disarm_after_land = bool(
            self.get_parameter('auto_disarm_after_land').value)
        self.force_disarm_after_land_timeout_s = float(
            self.get_parameter('force_disarm_after_land_timeout_s').value)
        self.max_velocity = float(self.get_parameter('max_velocity_ms').value)
        self.start_position_tolerance = float(
            self.get_parameter('start_position_tolerance_m').value)
        self.start_velocity_tolerance = float(
            self.get_parameter('start_velocity_tolerance_ms').value)
        self.start_settle_time = float(
            self.get_parameter('start_settle_time_s').value)

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
        self.active_pub = self.create_publisher(String, '/mission/active', 10)
        self.create_subscription(String, '/mission/abort', self.abort_cb, 10)
        self.create_subscription(String, '/mission/control', self.control_cb, 10)

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
        self.local_pos_v1_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_cb,
            px4_qos,
        )
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_cb,
            px4_qos,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            px4_qos,
        )
        self.vehicle_status_v1_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_cb,
            px4_qos,
        )
        self.vehicle_status_v4_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.vehicle_status_cb,
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
        self.reported_waiting_for_takeoff = False
        self.reported_waiting_for_offboard_request = False
        self.reported_mission_started = False
        self.takeoff_requested = False
        self.mission_origin = None
        self.offboard_requested = False
        self.offboard_command_sent = False
        self.offboard_request_time = None
        self.aborted = False
        self.abort_reason = ''
        self.abort_command_sent = False
        self.disarm_command_sent = False
        self.land_command_time = None
        self.start_settle_begin = None
        self.reported_waiting_for_settle = False
        self.control_relinquished = False

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
                heading_text = r.get('heading_deg', '')
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
                    'heading_deg': float(heading_text) % 360.0 if heading_text else math.nan,
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

    def abort_cb(self, msg):
        if self.aborted:
            return
        self.aborted = True
        self.abort_reason = msg.data
        self.publish_event('MISSION_ABORTED', self.abort_reason)
        self.publish_active(False)
        self.get_logger().error(f'Mission abort requested: {self.abort_reason}')

    def control_cb(self, msg):
        parts = msg.data.split('|', 1)
        command = parts[0].strip().upper()
        value = parts[1].strip() if len(parts) > 1 else ''

        if command == 'OFFBOARD':
            self.request_offboard_mode()
        elif command == 'ARM':
            self.arm()
        elif command == 'DISARM':
            self.relinquish_control('DISARM_REQUESTED', 'disarm requested by mission control')
            self.disarm(force=False)
        elif command == 'TAKEOFF':
            if self.start_time is None:
                if self.capture_mission_origin():
                    self.takeoff_requested = True
                    self.reported_waiting_for_takeoff = False
                    self.publish_event('TAKEOFF_REQUESTED', self.mission_file)
                    self.get_logger().warn(
                        'Takeoff requested; current position is the mission origin '
                        f'(x={self.mission_origin["x"]:.3f}, y={self.mission_origin["y"]:.3f}, '
                        f'z={self.mission_origin["z"]:.3f} up).'
                    )
        elif command == 'LAND':
            self.relinquish_control('LAND_REQUESTED', 'land requested by mission control')
            self.land()
        elif command == 'RTL':
            self.relinquish_control('RTL_REQUESTED', 'rtl requested by mission control')
            self.rtl()
        elif command == 'SELECT_MISSION':
            self.select_mission(value)
        else:
            self.get_logger().warn(f'Ignoring unknown mission control command: {msg.data}')

    def select_mission(self, path):
        if self.start_time is not None:
            self.get_logger().error('Cannot change mission after takeoff has started.')
            self.publish_event('MISSION_SELECT_REJECTED', 'mission already started')
            return

        candidate = str(Path(path).expanduser())
        try:
            rows = self.load_mission(candidate)
        except Exception as exc:
            self.get_logger().error(f'Could not load selected mission {candidate}: {exc}')
            self.publish_event('MISSION_SELECT_REJECTED', f'{candidate}: {exc}')
            return

        self.mission_file = candidate
        self.rows = rows
        self.last_active_row = None
        self.reported_waiting_for_start = False
        self.reported_waiting_for_takeoff = False
        self.reported_waiting_for_offboard_request = False
        self.reported_waiting_for_settle = False
        self.start_settle_begin = None
        self.takeoff_requested = False
        self.mission_origin = None
        self.offboard_requested = False
        self.offboard_command_sent = False
        self.offboard_request_time = None
        self.publish_event('MISSION_SELECTED', self.mission_file)
        self.get_logger().warn(f'Selected mission: {self.mission_file}')

    def is_offboard_enabled(self):
        if self.vehicle_control_mode is not None:
            return bool(self.vehicle_control_mode.flag_control_offboard_enabled)

        if self.vehicle_status is not None:
            return int(self.vehicle_status.nav_state) == 14

        return False

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

    def capture_mission_origin(self):
        if self.local_position is None:
            self.get_logger().error('Cannot start mission: local position is not available.')
            self.publish_event('TAKEOFF_REJECTED', 'local position unavailable')
            return False
        if self.mission_origin is None:
            self.mission_origin = {
                'x': float(self.local_position.x),
                'y': float(self.local_position.y),
                'z': -float(self.local_position.z),
            }
            self.publish_event(
                'MISSION_ORIGIN',
                f'x={self.mission_origin["x"]:.6f},'
                f'y={self.mission_origin["y"]:.6f},'
                f'z={self.mission_origin["z"]:.6f}',
            )
        return True

    def resolve_position_row(self, row):
        x = float(row['x'])
        y = float(row['y'])
        z = float(row['z'])
        if row.get('absolute_frame', False) or self.mission_origin is None:
            return x, y, z
        return (
            self.mission_origin['x'] + x,
            self.mission_origin['y'] + y,
            self.mission_origin['z'] + z,
        )

    def publish_setpoint(self, row):
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()

        # Mission convention:
        # x forward, y right, z positive UP.
        #
        # PX4 local frame is NED:
        # x forward/north, y right/east, z positive DOWN.
        local_x, local_y, local_z_up = self.resolve_position_row(row)
        px4_x = local_x
        px4_y = local_y
        px4_z = -local_z_up

        if row['mode'] == 'pos':
            msg.position = [float(px4_x), float(px4_y), float(px4_z)]
            msg.velocity = [math.nan, math.nan, math.nan]
            msg.acceleration = [math.nan, math.nan, math.nan]

        elif row['mode'] == 'vel':
            # Velocity mode: publish velocity and leave position NaN.
            # Mission vz is positive up -> PX4 vz is positive down.
            msg.position = [math.nan, math.nan, math.nan]
            vx, vy, vz = float(row['vx']), float(row['vy']), float(row['vz'])
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            if speed > self.max_velocity:
                scale = self.max_velocity / speed
                vx, vy, vz = vx * scale, vy * scale, vz * scale
            msg.velocity = [vx, vy, -vz]
            msg.acceleration = [math.nan, math.nan, math.nan]

        else:
            msg.position = [math.nan, math.nan, math.nan]
            msg.velocity = [0.0, 0.0, 0.0]
            msg.acceleration = [math.nan, math.nan, math.nan]

        # PX4 NED yaw is compass heading: 0=north, 90=east, increasing clockwise.
        msg.yaw = (
            math.radians(row['heading_deg'])
            if math.isfinite(row['heading_deg'])
            else math.nan
        )
        msg.yawspeed = math.nan

        self.setpoint_pub.publish(msg)

    def current_position_hold_row(self):
        row = dict(self.rows[0])
        if self.local_position is not None:
            row.update({
                'type': 'hold',
                'mode': 'pos',
                'profile': 'hold',
                'x': float(self.local_position.x),
                'y': float(self.local_position.y),
                'z': -float(self.local_position.z),
                'vx': 0.0,
                'vy': 0.0,
                'vz': 0.0,
                'ax': 0.0,
                'absolute_frame': True,
            })
        return row

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

    def publish_active(self, active):
        msg = String()
        msg.data = str(active).lower()
        self.active_pub.publish(msg)

    def relinquish_control(self, event_type, detail=''):
        if self.control_relinquished:
            return
        self.control_relinquished = True
        self.publish_active(False)
        self.publish_event(event_type, detail)
        self.get_logger().warn(
            'Mission executor is relinquishing Offboard setpoint control. '
            'No more Offboard setpoints will be published.'
        )

    def arm(self):
        self.get_logger().warn('Sending ARM command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self, force=False):
        self.get_logger().warn(f'Sending DISARM command (force={force})')
        force_magic = 21196.0 if force else 0.0
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0,
            force_magic,
        )

    def set_offboard_mode(self):
        self.get_logger().warn('Sending OFFBOARD mode command')
        # PX4 custom mode: param1=1, param2=6 for Offboard.
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def request_offboard_mode(self):
        self.offboard_requested = True
        self.offboard_command_sent = False
        self.offboard_request_time = self.get_clock().now().nanoseconds / 1e9
        self.reported_waiting_for_offboard_request = False
        self.publish_event('OFFBOARD_REQUESTED', 'warming setpoint stream before mode command')
        self.get_logger().warn('Offboard requested; warming setpoint stream before sending mode command.')

    def land(self):
        self.get_logger().warn('Sending LAND command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def rtl(self):
        self.get_logger().error('Sending RTL command')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def get_current_row(self, elapsed):
        current = self.rows[0]
        for row in self.rows:
            if elapsed >= row['t']:
                current = row
            else:
                break
        return current

    def get_setpoint_row(self, elapsed):
        current = self.get_current_row(elapsed)
        if current['profile'] != 'linear' or current['mode'] != 'pos':
            return current

        current_index = self.rows.index(current)
        if current_index + 1 >= len(self.rows):
            return current

        following = self.rows[current_index + 1]
        if following['type'] == 'end' or following['mode'] != 'pos':
            return current

        segment_duration = following['t'] - current['t']
        if segment_duration <= 0:
            return current

        blend = min(1.0, max(0.0, (elapsed - current['t']) / segment_duration))
        interpolated = self.interpolate_position_row(current, following, blend)
        return interpolated

    def interpolate_position_row(self, start_row, target_row, blend):
        interpolated = dict(target_row)
        for key in ('x', 'y', 'z'):
            interpolated[key] = start_row[key] + blend * (target_row[key] - start_row[key])
        if math.isfinite(start_row['heading_deg']) and math.isfinite(target_row['heading_deg']):
            start = math.radians(start_row['heading_deg'])
            end = math.radians(target_row['heading_deg'])
            delta = math.atan2(math.sin(end - start), math.cos(end - start))
            interpolated['heading_deg'] = math.degrees(start + blend * delta) % 360.0
        return interpolated

    def first_setpoint_is_settled(self):
        row = self.rows[0]
        if row['mode'] != 'pos' or self.local_position is None:
            return True, 0.0, 0.0
        target_x, target_y, target_z = self.resolve_position_row(row)

        position_error = math.sqrt(
            (float(self.local_position.x) - target_x) ** 2
            + (float(self.local_position.y) - target_y) ** 2
            + (-float(self.local_position.z) - target_z) ** 2
        )
        speed = math.sqrt(
            float(self.local_position.vx) ** 2
            + float(self.local_position.vy) ** 2
            + float(self.local_position.vz) ** 2
        )
        settled = (
            position_error <= self.start_position_tolerance
            and speed <= self.start_velocity_tolerance
        )
        return settled, position_error, speed

    def timer_cb(self):
        # Wait until PX4 local position is available.
        if self.local_position is None:
            self.get_logger().info(
                'Waiting for /fmu/out/vehicle_local_position...',
                throttle_duration_sec=2.0,
            )
            return

        if self.control_relinquished:
            return

        if self.aborted:
            if not self.abort_command_sent:
                self.rtl()
                self.abort_command_sent = True
                self.relinquish_control('MISSION_ABORT_RELINQUISHED', self.abort_reason)
            return

        # Before PX4 is armed and in Offboard, keep streaming the first setpoint
        # so Offboard mode is available, but DO NOT start the mission clock yet.
        if self.start_time is None:
            armed_and_offboard = self.is_armed_and_offboard()
            if self.auto_offboard and not self.offboard_requested:
                self.request_offboard_mode()

            if not self.offboard_requested and not armed_and_offboard:
                if not self.reported_waiting_for_offboard_request:
                    self.get_logger().warn(
                        'Offboard stream is idle. Select Switch Offboard in the UI when ready.'
                    )
                    self.publish_event('WAITING_FOR_OFFBOARD_REQUEST', 'no offboard stream until UI request')
                    self.reported_waiting_for_offboard_request = True
                return

            row = self.rows[0] if self.takeoff_requested and armed_and_offboard else self.current_position_hold_row()
            mode = row['mode'] if row['mode'] in ['pos', 'vel'] else 'pos'
            self.publish_offboard_control_mode(mode)
            self.publish_setpoint(row)

            if (
                self.offboard_requested
                and not self.offboard_command_sent
                and self.offboard_request_time is not None
            ):
                now = self.get_clock().now().nanoseconds / 1e9
                if now - self.offboard_request_time >= 1.2:
                    self.set_offboard_mode()
                    self.offboard_command_sent = True

            self.offboard_counter += 1

            if self.offboard_counter == int(self.rate_hz * 2.0):
                if not self.auto_offboard:
                    self.get_logger().warn('auto_offboard disabled. Switch to Offboard manually when ready.')

                if self.auto_arm:
                    self.arm()
                else:
                    self.get_logger().warn('auto_arm disabled. Arm manually when ready.')

            if not armed_and_offboard:
                if not self.reported_waiting_for_start:
                    self.get_logger().warn(
                        'Waiting for PX4 to be armed and in Offboard. Takeoff is still gated by terminal UI.'
                    )
                    self.publish_event('WAITING_FOR_ARM_OFFBOARD', 'holding current position; mission timer not started')
                    self.reported_waiting_for_start = True
                return

            if not self.takeoff_requested:
                if not self.reported_waiting_for_takeoff:
                    self.get_logger().warn(
                        'PX4 is armed and in Offboard. Holding current position until TAKEOFF is selected.'
                    )
                    self.publish_event('WAITING_FOR_TAKEOFF', 'armed/offboard; holding current position')
                    self.reported_waiting_for_takeoff = True
                return

            now = self.get_clock().now().nanoseconds / 1e9
            settled, position_error, speed = self.first_setpoint_is_settled()
            if not settled:
                self.start_settle_begin = None
                if not self.reported_waiting_for_settle:
                    self.get_logger().warn(
                        'Armed and Offboard; waiting to reach and settle at the first setpoint '
                        f'(error={position_error:.3f} m, speed={speed:.3f} m/s)...'
                    )
                    self.publish_event(
                        'WAITING_FOR_FIRST_SETPOINT',
                        f'position_error_m={position_error:.3f}, speed_ms={speed:.3f}',
                    )
                    self.reported_waiting_for_settle = True
                return

            if self.start_settle_begin is None:
                self.start_settle_begin = now
                return
            if now - self.start_settle_begin < self.start_settle_time:
                return

            self.start_time = now
            self.reported_waiting_for_start = False
            self.reported_waiting_for_settle = False
            self.get_logger().warn(
                'PX4 reached and settled at the first setpoint. Starting mission timer at t=0.'
            )

            if not self.reported_mission_started:
                self.publish_event(
                    'MISSION_STARTED',
                    'PX4 armed, Offboard, and settled at first setpoint; t=0',
                )
            self.publish_active(True)
            self.reported_mission_started = True

        if self.start_time is not None and not self.is_offboard_enabled():
            nav_state = (
                int(self.vehicle_status.nav_state)
                if self.vehicle_status is not None
                else 'unknown'
            )
            self.relinquish_control(
                'PILOT_MODE_TAKEOVER',
                f'PX4 left Offboard; nav_state={nav_state}',
            )
            return

        elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        row = self.get_setpoint_row(elapsed)

        if row['type'] == 'end':
            if self.last_active_row is None:
                self.get_logger().error('Mission reached end but no previous active setpoint exists.')
                return

            self.get_logger().warn(
                'MISSION COMPLETE - holding final setpoint while PX4 remains in Offboard.',
                throttle_duration_sec=5.0,
            )
            if not self.reported_mission_done:
                self.publish_event('MISSION_COMPLETE_HOLDING', 'holding final setpoint')
                self.publish_active(False)
                if self.auto_land:
                    self.land()
                    self.land_command_time = self.get_clock().now().nanoseconds / 1e9
                    self.relinquish_control(
                        'AUTO_LAND_RELINQUISHED',
                        'auto_land requested at mission completion',
                    )
                    return
            self.reported_mission_done = True

            if (
                self.auto_disarm_after_land
                and self.force_disarm_after_land_timeout_s > 0
                and self.land_command_time is not None
                and not self.disarm_command_sent
            ):
                now = self.get_clock().now().nanoseconds / 1e9
                if now - self.land_command_time >= self.force_disarm_after_land_timeout_s:
                    self.disarm(force=True)
                    self.disarm_command_sent = True

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
