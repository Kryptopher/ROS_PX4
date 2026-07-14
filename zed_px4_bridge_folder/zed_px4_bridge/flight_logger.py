#!/usr/bin/env python3

import csv
import json
import math
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleStatus,
)
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
        self.declare_parameter('copy_ulog', True)
        self.declare_parameter('ulog_source_dir', str(Path.home() / '.local/share/logloader/logs'))
        self.declare_parameter('ulog_wait_s', 5.0)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        label = self.get_parameter('run_label').value
        self.mission_file = self.get_parameter('mission_file').value
        self.run_dir = Path(self.get_parameter('log_base_dir').value) / f'{stamp}_{label}'
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.start_wall_time = time.time()
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.local_position = None
        self.vehicle_status = None
        self.control_mode = None
        self.payload = None
        self.rows = 0
        self.events = 0
        self.commands = 0

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
        self.command_file = open(self.run_dir / 'commands.csv', 'w', newline='')
        self.command_writer = csv.writer(self.command_file)
        self.command_writer.writerow([
            't', 'x', 'y', 'z_ned', 'z_up', 'vx', 'vy', 'vz_ned', 'vz_up',
            'ax', 'ay', 'az_ned', 'yaw_rad', 'heading_deg', 'yawspeed_rad_s',
        ])

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
            VehicleStatus, '/fmu/out/vehicle_status_v1',
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
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self._trajectory_setpoint_cb, qos)
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
        if event_type == 'MISSION_SELECTED' and detail:
            self.mission_file = detail
        self.event_writer.writerow([
            datetime.now().isoformat(), f'{self._elapsed():.4f}',
            event_type, detail,
        ])
        self.event_file.flush()
        self.events += 1

    def _trajectory_setpoint_cb(self, msg):
        yaw = float(msg.yaw)
        self.command_writer.writerow([
            f'{self._elapsed():.6f}',
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
            -float(msg.position[2]),
            float(msg.velocity[0]),
            float(msg.velocity[1]),
            float(msg.velocity[2]),
            -float(msg.velocity[2]),
            float(msg.acceleration[0]),
            float(msg.acceleration[1]),
            float(msg.acceleration[2]),
            yaw,
            math.degrees(yaw) % 360.0 if math.isfinite(yaw) else math.nan,
            float(msg.yawspeed),
        ])
        self.commands += 1
        if self.commands % 200 == 0:
            self.command_file.flush()

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
            'command_rows': self.commands,
        }
        if self.mission_file and Path(self.mission_file).is_file():
            shutil.copy2(self.mission_file, self.run_dir)
        copied_ulog = self._copy_latest_ulog()
        metadata['ulog_file'] = str(copied_ulog) if copied_ulog else ''
        metadata['ulog_source_dir'] = str(Path(self.get_parameter('ulog_source_dir').value).expanduser())
        with open(self.run_dir / 'metadata.json', 'w') as stream:
            json.dump(metadata, stream, indent=2)
        self.command_file.flush()
        self._write_plots()
        self._write_summary()
        self.flight_file.close()
        self.event_file.close()
        self.command_file.close()
        super().destroy_node()

    def _copy_latest_ulog(self):
        if not bool(self.get_parameter('copy_ulog').value):
            return None

        source_dir = Path(self.get_parameter('ulog_source_dir').value).expanduser()
        wait_s = max(0.0, float(self.get_parameter('ulog_wait_s').value))
        deadline = time.time() + wait_s
        latest = None
        while time.time() <= deadline:
            candidates = [
                path for path in source_dir.glob('*.ulg')
                if path.is_file() and path.stat().st_mtime >= self.start_wall_time - 30.0
            ] if source_dir.is_dir() else []
            if candidates:
                latest = max(candidates, key=lambda path: path.stat().st_mtime)
                break
            if wait_s <= 0.0:
                break
            time.sleep(0.5)

        if latest is None:
            self.get_logger().warn(f'No recent ULog found in {source_dir}')
            return None

        target = self.run_dir / latest.name.replace(':', '')
        try:
            shutil.copy2(latest, target)
            self.get_logger().warn(f'Copied PX4 ULog: {target}')
            return target
        except Exception as exc:
            self.get_logger().warn(f'Could not copy PX4 ULog {latest}: {exc}')
            return None

    def _write_plots(self):
        repo = Path(os.environ.get('ROS_PX4_HOME', Path.home() / 'ROS_PX4'))
        tools_dir = repo / 'tools'
        if tools_dir.is_dir():
            sys.path.insert(0, str(tools_dir))
        try:
            from mission_logbook import (
                write_3d_plot,
                mission_start_time,
                mission_row_markers,
                takeoff_requested_time,
                write_x_time_plot,
                write_xy_plot,
                write_y_time_plot,
                write_z_plot,
            )
        except Exception as exc:
            self.get_logger().warn(f'Could not load plot helpers: {exc}')
            return

        def read_csv(path):
            if not path.is_file():
                return []
            with path.open(newline='') as stream:
                return list(csv.DictReader(stream))

        samples = read_csv(self.run_dir / 'flight.csv')
        events = read_csv(self.run_dir / 'events.csv')
        commands = read_csv(self.run_dir / 'commands.csv')
        start_t = mission_start_time(events)
        takeoff_t = takeoff_requested_time(events)
        color_split_t = takeoff_t if takeoff_t is not None else start_t
        mission_file = self.mission_file
        if (not mission_file or not Path(mission_file).is_file()):
            copied = sorted(self.run_dir.glob('*.tsv'))
            mission_file = str(copied[0]) if copied else ''

        plot_path = self.run_dir / 'trajectory_3d.png'
        xy_plot_path = self.run_dir / 'xy.png'
        z_plot_path = self.run_dir / 'height.png'
        x_time_plot_path = self.run_dir / 'x_time.png'
        y_time_plot_path = self.run_dir / 'y_time.png'
        row_markers = mission_row_markers(mission_file, start_t)
        if write_3d_plot(plot_path, mission_file, samples, commands, color_split_t, row_markers):
            self.get_logger().warn(f'Saved 3D plot: {plot_path}')
        if write_xy_plot(xy_plot_path, mission_file, samples, commands, color_split_t, row_markers):
            self.get_logger().warn(f'Saved XY plot: {xy_plot_path}')
        if write_z_plot(z_plot_path, samples, commands, color_split_t, row_markers):
            self.get_logger().warn(f'Saved height plot: {z_plot_path}')
        if write_x_time_plot(x_time_plot_path, samples, commands, color_split_t, row_markers):
            self.get_logger().warn(f'Saved X time plot: {x_time_plot_path}')
        if write_y_time_plot(y_time_plot_path, samples, commands, color_split_t, row_markers):
            self.get_logger().warn(f'Saved Y time plot: {y_time_plot_path}')

    def _write_summary(self):
        def read_csv(path):
            if not path.is_file():
                return []
            with path.open(newline='') as stream:
                return list(csv.DictReader(stream))

        samples = read_csv(self.run_dir / 'flight.csv')
        events = read_csv(self.run_dir / 'events.csv')
        commands = read_csv(self.run_dir / 'commands.csv')
        metadata = {}
        metadata_path = self.run_dir / 'metadata.json'
        if metadata_path.is_file():
            try:
                with metadata_path.open() as stream:
                    metadata = json.load(stream)
            except Exception:
                metadata = {}
        start_t = None
        takeoff_t = None
        try:
            from mission_logbook import mission_row_markers, mission_start_time, takeoff_requested_time
            start_t = mission_start_time(events)
            takeoff_t = takeoff_requested_time(events)
            row_markers = mission_row_markers(self.mission_file, start_t)
        except Exception:
            row_markers = []
            pass
        final = samples[-1] if samples else {}
        sample_duration = 0.0
        achieved_rate = 0.0
        if len(samples) > 1:
            try:
                sample_duration = float(samples[-1]['t']) - float(samples[0]['t'])
                if sample_duration > 0:
                    achieved_rate = (len(samples) - 1) / sample_duration
            except (KeyError, TypeError, ValueError):
                pass

        summary_path = self.run_dir / 'summary.txt'
        with summary_path.open('w') as stream:
            stream.write('Real Flight Log Summary\n')
            stream.write(f'Run directory: {self.run_dir}\n')
            stream.write(f'Mission file: {self.mission_file}\n')
            stream.write(f'Flight CSV: {self.run_dir / "flight.csv"}\n')
            stream.write(f'Events CSV: {self.run_dir / "events.csv"}\n')
            stream.write(f'Commands CSV: {self.run_dir / "commands.csv"}\n')
            stream.write(f'Metadata JSON: {self.run_dir / "metadata.json"}\n')
            stream.write(f'PX4 ULog: {metadata.get("ulog_file", "") or "not copied"}\n')
            stream.write(f'Event count: {len(events)}\n')
            stream.write(f'Sample count: {len(samples)}\n')
            stream.write(f'Command sample count: {len(commands)}\n')
            stream.write(f'Requested sample rate: {float(self.get_parameter("sample_rate_hz").value):g} Hz\n')
            stream.write(f'Achieved sample rate: {achieved_rate:.3f} Hz\n')
            stream.write(f'3D trajectory plot: {self.run_dir / "trajectory_3d.png"}\n')
            stream.write(f'Top-down XY plot: {self.run_dir / "xy.png"}\n')
            stream.write(f'Height plot: {self.run_dir / "height.png"}\n')
            stream.write(f'X vs time plot: {self.run_dir / "x_time.png"}\n')
            stream.write(f'Y vs time plot: {self.run_dir / "y_time.png"}\n')
            stream.write(f'Takeoff requested time: {takeoff_t if takeoff_t is not None else "not recorded"}\n')
            stream.write(f'Mission start time: {start_t if start_t is not None else "not recorded"}\n')
            stream.write(f'TSV row markers: {len(row_markers)}\n')
            stream.write('\nFinal sample:\n')
            for key, value in final.items():
                stream.write(f'  {key}: {value}\n')
        self.get_logger().warn(f'Saved summary: {summary_path}')


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
