#!/usr/bin/env python3

import csv
import json
import math
import os
import shutil
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleControlMode,
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleThrustSetpoint,
    VehicleTorqueSetpoint,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String

# The Jetson has no RTC battery: without network time it boots at 1970-01-01,
# so any wall clock before this is treated as unset.
CLOCK_VALID_AFTER = datetime(2020, 1, 1).timestamp()


def clock_is_valid():
    return time.time() >= CLOCK_VALID_AFTER


class FlightLogger(Node):
    """Write synchronized PX4 state, payload angles, and mission events."""

    def __init__(self):
        super().__init__('flight_logger')
        self.declare_parameter('log_base_dir', str(Path.home() / 'logs'))
        self.declare_parameter('run_label', 'dds_mission')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('sample_rate_hz', 100.0)
        self.declare_parameter('copy_ulog', True)
        self.declare_parameter('ulog_source_dir', str(Path.home() / '.local/share/logloader/logs'))
        self.declare_parameter('ulog_wait_s', 5.0)
        self.declare_parameter('fsync_interval_s', 1.0)

        log_base_dir = Path(self.get_parameter('log_base_dir').value)
        log_base_dir.mkdir(parents=True, exist_ok=True)
        self.run_number = self._next_run_number(log_base_dir)
        self.start_monotonic = time.monotonic()
        self.clock_valid_at_start = clock_is_valid()
        if self.clock_valid_at_start:
            self.run_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        else:
            # Same 15-character width as a real stamp; renamed to the real
            # start time at shutdown if the clock gets set during the run.
            self.run_stamp = f'nodate_{self.run_number:08d}'
        label = self.get_parameter('run_label').value
        self.initial_run_label = str(label)
        self.mission_file = self.get_parameter('mission_file').value
        self.run_dir = log_base_dir / f'{self.run_stamp}_{label}'
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.start_wall_time = time.time()
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.local_position = None
        self.attitude = None
        self.angular_velocity = None
        self.thrust_setpoint = None
        self.torque_setpoint = None
        self.vehicle_status = None
        self.control_mode = None
        self.payload = None
        self.rows = 0
        self.events = 0
        self.commands = 0
        self.encoder_angle_rows = 0
        self.encoder_recording_started = False
        self.encoder_recording_stopped = False
        self.encoder_recording_start_t = None
        self.encoder_recording_stop_t = None
        self.encoder_recording_stop_reason = ''

        self.flight_file = open(self.run_dir / 'flight.csv', 'w', newline='')
        self.flight_writer = csv.writer(self.flight_file)
        self.flight_writer.writerow([
            't', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading',
            'xy_valid', 'z_valid', 'armed', 'offboard', 'nav_state',
            'arming_state', 'failsafe', 'pitch_deg', 'roll_deg',
            'pitch_count', 'roll_count',
            'roll_rad', 'pitch_rad', 'yaw_rad',
            'roll_rate_rad_s', 'pitch_rate_rad_s', 'yaw_rate_rad_s',
            'thrust_command', 'thrust_x', 'thrust_y', 'thrust_z',
            'roll_torque_command', 'pitch_torque_command', 'yaw_torque_command',
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
        self.encoder_file = open(self.run_dir / 'encoder_angles.csv', 'w', newline='')
        self.encoder_writer = csv.writer(self.encoder_file)
        self.encoder_writer.writerow([
            't_wall', 't', 'encoder1_angle_deg', 'encoder2_angle_deg',
        ])
        # Make the header visible immediately, even if the process is interrupted
        # before the first batch flush.
        self.encoder_file.flush()
        self.log_files = [self.flight_file, self.event_file, self.command_file, self.encoder_file]
        self._write_metadata({'status': 'recording'})
        if not self.clock_valid_at_start:
            self.get_logger().warn(
                'System clock is not set (no network time); logging to '
                f'{self.run_dir.name}. It will be renamed to the real start time '
                'at shutdown if the clock is set before then.'
            )

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
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            lambda msg: setattr(self, 'attitude', msg), qos)
        self.create_subscription(
            VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity',
            lambda msg: setattr(self, 'angular_velocity', msg), qos)
        self.create_subscription(
            VehicleThrustSetpoint, '/fmu/out/vehicle_thrust_setpoint',
            lambda msg: setattr(self, 'thrust_setpoint', msg), qos)
        self.create_subscription(
            VehicleTorqueSetpoint, '/fmu/out/vehicle_torque_setpoint',
            lambda msg: setattr(self, 'torque_setpoint', msg), qos)
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
            Float64MultiArray, '/mt6835/angles', self._encoder_angles_cb, qos)
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self._trajectory_setpoint_cb, qos)
        self.create_subscription(String, '/mission_executor/event', self._event_cb, 10)
        self.create_subscription(String, '/safety/event', self._event_cb, 10)
        self.create_timer(
            1.0 / float(self.get_parameter('sample_rate_hz').value),
            self._sample)
        fsync_interval = float(self.get_parameter('fsync_interval_s').value)
        if fsync_interval > 0.0:
            self.create_timer(fsync_interval, self._sync_to_disk)
        self.get_logger().info(f'Flight logging to {self.run_dir}')

    @staticmethod
    def _next_run_number(log_base_dir):
        counter_path = log_base_dir / '.run_counter'
        try:
            number = int(counter_path.read_text().strip()) + 1
        except (OSError, ValueError):
            number = 1
        counter_path.write_text(f'{number}\n')
        return number

    def _sync_to_disk(self):
        # flush() only reaches the OS cache; fsync() makes it survive a power cut
        # (an unsynced cut loses the last ~30 s and leaves NUL padding in the CSVs).
        for stream in self.log_files:
            if stream.closed:
                continue
            stream.flush()
            os.fsync(stream.fileno())

    def _write_metadata(self, extra):
        metadata = {
            'run_dir': str(self.run_dir),
            'run_number': self.run_number,
            'boot_id': self._boot_id(),
            'clock_valid_at_start': self.clock_valid_at_start,
            'mission_file': self.mission_file,
            **extra,
        }
        path = self.run_dir / 'metadata.json'
        tmp_path = path.with_suffix('.json.tmp')
        with open(tmp_path, 'w') as stream:
            json.dump(metadata, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        tmp_path.replace(path)

    @staticmethod
    def _boot_id():
        try:
            return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        except OSError:
            return ''

    def _elapsed(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.start_time

    @staticmethod
    def _quat_to_euler(q):
        w, x, y, z = [float(value) for value in q]
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _vector3(msg, attr='xyz'):
        if msg is None:
            return math.nan, math.nan, math.nan
        values = getattr(msg, attr, [])
        return tuple(float(values[index]) for index in range(3))

    def _event_cb(self, msg):
        event_type, _, detail = msg.data.partition('|')
        if event_type == 'MISSION_SELECTED' and detail and detail != self.mission_file:
            self.mission_file = detail
            self._write_metadata({'status': 'recording'})
        self.event_writer.writerow([
            datetime.now().isoformat(), f'{self._elapsed():.4f}',
            event_type, detail,
        ])
        self.event_file.flush()
        self.events += 1

        if event_type == 'TAKEOFF_REQUESTED' and self.encoder_recording_stopped:
            # Leaving Offboard before takeoff must not cost the mission its angles.
            self.encoder_recording_stopped = False
            self.encoder_recording_stop_t = None
            self.encoder_recording_stop_reason = ''

        if event_type in {
            'MISSION_COMPLETE_HOLDING',
            'MISSION_ABORTED',
            'MISSION_ABORT_RELINQUISHED',
            'PILOT_MODE_TAKEOVER',
            'SAFETY_ABORT',
        }:
            self._stop_encoder_recording(event_type)

    def _stop_encoder_recording(self, reason):
        if self.encoder_recording_stopped:
            return
        self.encoder_recording_stopped = True
        self.encoder_recording_stop_t = self._elapsed()
        self.encoder_recording_stop_reason = reason
        self.encoder_file.flush()
        os.fsync(self.encoder_file.fileno())
        self.get_logger().warn(
            f'Encoder recording stopped and saved ({reason}); '
            f'{self.encoder_angle_rows} samples in {self.run_dir / "encoder_angles.csv"}'
        )

    def _encoder_angles_cb(self, msg):
        if len(msg.data) < 2:
            return
        if self.encoder_recording_stopped:
            return
        if (
            self.control_mode is None
            or not self.control_mode.flag_control_offboard_enabled
        ):
            return

        if not self.encoder_recording_started:
            self.encoder_recording_started = True
            self.encoder_recording_start_t = self._elapsed()
            self.get_logger().warn(
                'PX4 entered Offboard; encoder angle recording started.'
            )

        self.encoder_writer.writerow([
            datetime.now().isoformat(timespec='milliseconds'),
            f'{self._elapsed():.6f}',
            float(msg.data[0]),
            float(msg.data[1]),
        ])
        self.encoder_angle_rows += 1
        if self.encoder_angle_rows % 100 == 0:
            self.encoder_file.flush()

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
        if (
            self.encoder_recording_started
            and not self.encoder_recording_stopped
            and control is not None
            and not control.flag_control_offboard_enabled
        ):
            self._stop_encoder_recording('PX4_LEFT_OFFBOARD')
        payload = list(self.payload.data) if self.payload is not None else []
        payload += [0.0] * (5 - len(payload))
        roll, pitch, yaw = (
            self._quat_to_euler(self.attitude.q)
            if self.attitude is not None else
            (math.nan, math.nan, math.nan)
        )
        roll_rate, pitch_rate, yaw_rate = self._vector3(self.angular_velocity)
        thrust_x, thrust_y, thrust_z = self._vector3(self.thrust_setpoint)
        torque_x, torque_y, torque_z = self._vector3(self.torque_setpoint)
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
            roll, pitch, yaw,
            roll_rate, pitch_rate, yaw_rate,
            thrust_z, thrust_x, thrust_y, thrust_z,
            torque_x, torque_y, torque_z,
        ])
        self.rows += 1
        if self.rows % 50 == 0:
            self.flight_file.flush()

    def destroy_node(self):
        if not self.encoder_recording_stopped:
            self._stop_encoder_recording('LOGGER_SHUTDOWN')
        self.flight_file.flush()
        self.event_file.flush()
        self.command_file.flush()
        self.encoder_file.flush()
        self._fix_run_stamp_if_clock_was_set()
        self._rename_run_dir_for_selected_mission()
        metadata = {
            'status': 'complete',
            'flight_rows': self.rows,
            'event_rows': self.events,
            'command_rows': self.commands,
            'encoder_angle_rows': self.encoder_angle_rows,
            'encoder_recording_started': self.encoder_recording_started,
            'encoder_recording_start_t': self.encoder_recording_start_t,
            'encoder_recording_stop_t': self.encoder_recording_stop_t,
            'encoder_recording_stop_reason': self.encoder_recording_stop_reason,
        }
        if self.mission_file and Path(self.mission_file).is_file():
            shutil.copy2(self.mission_file, self.run_dir)
        copied_ulog = self._copy_latest_ulog()
        metadata['ulog_file'] = str(copied_ulog) if copied_ulog else ''
        metadata['ulog_source_dir'] = str(Path(self.get_parameter('ulog_source_dir').value).expanduser())
        self._sync_to_disk()
        self._write_metadata(metadata)
        self._write_plots()
        self._write_summary()
        self.flight_file.close()
        self.event_file.close()
        self.command_file.close()
        self.encoder_file.close()
        super().destroy_node()

    def _fix_run_stamp_if_clock_was_set(self):
        if self.clock_valid_at_start or not clock_is_valid():
            return
        real_start = time.time() - (time.monotonic() - self.start_monotonic)
        new_stamp = datetime.fromtimestamp(real_start).strftime('%Y%m%d_%H%M%S')
        target = self.run_dir.with_name(new_stamp + self.run_dir.name[len(self.run_stamp):])
        if target.exists():
            target = target.with_name(f'{target.name}_run{self.run_number}')
        try:
            self.run_dir.rename(target)
        except OSError as exc:
            self.get_logger().warn(f'Could not rename {self.run_dir.name} to the real start time: {exc}')
            return
        self.run_dir = target
        self.run_stamp = new_stamp
        self.get_logger().warn(f'Clock was set during the run; log folder renamed to {self.run_dir}')

    @staticmethod
    def _safe_label(value):
        cleaned = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in value)
        cleaned = cleaned.strip('_')
        return cleaned or 'mission'

    def _rename_run_dir_for_selected_mission(self):
        mission_path = Path(self.mission_file) if self.mission_file else None
        if mission_path is None or not mission_path.name:
            return

        selected = self._safe_label(mission_path.stem)
        initial = self._safe_label(self.initial_run_label)
        prefix = initial.split('__', 1)[0] if '__' in initial else initial
        target_label = f'{prefix}__{selected}'
        current_label = self.run_dir.name.split('_', 2)[-1] if '_' in self.run_dir.name else self.run_dir.name
        if current_label == target_label:
            return

        target = self.run_dir.with_name(f'{self.run_dir.name[:15]}_{target_label}')
        if target == self.run_dir:
            return

        candidate = target
        index = 2
        while candidate.exists():
            candidate = target.with_name(f'{target.name}_{index}')
            index += 1

        try:
            self.run_dir.rename(candidate)
            self.run_dir = candidate
            self.get_logger().warn(f'Renamed log folder for selected mission: {self.run_dir}')
        except Exception as exc:
            self.get_logger().warn(f'Could not rename log folder for selected mission: {exc}')

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
        repo = Path(os.environ.get('ROS_PX4_HOME', Path.home() / 'ROS_PX4' / 'shared'))
        tools_dir = repo / 'tools'
        if tools_dir.is_dir():
            sys.path.insert(0, str(tools_dir))
        try:
            from mission_logbook import (
                write_3d_plot,
                write_3d_plot_html,
                mission_start_time,
                mission_event_markers,
                mission_row_markers,
                takeoff_requested_time,
                write_x_time_plot,
                write_x_time_plot_html,
                write_xy_plot,
                write_xy_plot_html,
                write_y_time_plot,
                write_y_time_plot_html,
                write_z_plot,
                write_z_plot_html,
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
        plot_html_path = self.run_dir / 'trajectory_3d.html'
        xy_plot_path = self.run_dir / 'xy.png'
        xy_html_path = self.run_dir / 'xy.html'
        z_plot_path = self.run_dir / 'height.png'
        z_html_path = self.run_dir / 'height.html'
        x_time_plot_path = self.run_dir / 'x_time.png'
        x_time_html_path = self.run_dir / 'x_time.html'
        y_time_plot_path = self.run_dir / 'y_time.png'
        y_time_html_path = self.run_dir / 'y_time.html'
        markers = mission_event_markers(events)
        time_markers = markers + mission_row_markers(mission_file, start_t)
        if write_3d_plot(plot_path, mission_file, samples, commands, color_split_t, markers):
            self.get_logger().warn(f'Saved 3D plot: {plot_path}')
        if write_3d_plot_html(plot_html_path, mission_file, samples, commands, color_split_t, markers):
            self.get_logger().warn(f'Saved interactive 3D plot: {plot_html_path}')
        if write_xy_plot(xy_plot_path, mission_file, samples, commands, color_split_t, markers):
            self.get_logger().warn(f'Saved XY plot: {xy_plot_path}')
        if write_xy_plot_html(xy_html_path, mission_file, samples, commands, color_split_t, markers):
            self.get_logger().warn(f'Saved interactive XY plot: {xy_html_path}')
        if write_z_plot(z_plot_path, samples, commands, color_split_t, time_markers):
            self.get_logger().warn(f'Saved height plot: {z_plot_path}')
        if write_z_plot_html(z_html_path, samples, commands, color_split_t, time_markers):
            self.get_logger().warn(f'Saved interactive height plot: {z_html_path}')
        if write_x_time_plot(x_time_plot_path, samples, commands, color_split_t, time_markers):
            self.get_logger().warn(f'Saved X time plot: {x_time_plot_path}')
        if write_x_time_plot_html(x_time_html_path, samples, commands, color_split_t, time_markers):
            self.get_logger().warn(f'Saved interactive X time plot: {x_time_html_path}')
        if write_y_time_plot(y_time_plot_path, samples, commands, color_split_t, time_markers):
            self.get_logger().warn(f'Saved Y time plot: {y_time_plot_path}')
        if write_y_time_plot_html(y_time_html_path, samples, commands, color_split_t, time_markers):
            self.get_logger().warn(f'Saved interactive Y time plot: {y_time_html_path}')

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
            from mission_logbook import (
                mission_event_markers,
                mission_row_markers,
                mission_start_time,
                takeoff_requested_time,
            )
            start_t = mission_start_time(events)
            takeoff_t = takeoff_requested_time(events)
            markers = mission_event_markers(events)
            time_markers = markers + mission_row_markers(self.mission_file, start_t)
        except Exception:
            markers = []
            time_markers = []
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
            stream.write(f'Encoder angles CSV: {self.run_dir / "encoder_angles.csv"}\n')
            stream.write(f'Metadata JSON: {self.run_dir / "metadata.json"}\n')
            stream.write(f'PX4 ULog: {metadata.get("ulog_file", "") or "not copied"}\n')
            stream.write(f'Event count: {len(events)}\n')
            stream.write(f'Sample count: {len(samples)}\n')
            stream.write(f'Command sample count: {len(commands)}\n')
            stream.write(f'Encoder angle sample count: {self.encoder_angle_rows}\n')
            stream.write(f'Encoder recording started: {self.encoder_recording_started}\n')
            stream.write(f'Encoder recording start time: {self.encoder_recording_start_t}\n')
            stream.write(f'Encoder recording stop time: {self.encoder_recording_stop_t}\n')
            stream.write(
                f'Encoder recording stop reason: '
                f'{self.encoder_recording_stop_reason or "not recorded"}\n'
            )
            stream.write(f'Requested sample rate: {float(self.get_parameter("sample_rate_hz").value):g} Hz\n')
            stream.write(f'Achieved sample rate: {achieved_rate:.3f} Hz\n')
            stream.write(f'3D trajectory plot: {self.run_dir / "trajectory_3d.png"}\n')
            stream.write(f'Interactive 3D trajectory plot: {self.run_dir / "trajectory_3d.html"}\n')
            stream.write(f'Top-down XY plot: {self.run_dir / "xy.png"}\n')
            stream.write(f'Interactive top-down XY plot: {self.run_dir / "xy.html"}\n')
            stream.write(f'Height plot: {self.run_dir / "height.png"}\n')
            stream.write(f'Interactive height plot: {self.run_dir / "height.html"}\n')
            stream.write(f'X vs time plot: {self.run_dir / "x_time.png"}\n')
            stream.write(f'Interactive X vs time plot: {self.run_dir / "x_time.html"}\n')
            stream.write(f'Y vs time plot: {self.run_dir / "y_time.png"}\n')
            stream.write(f'Interactive Y vs time plot: {self.run_dir / "y_time.html"}\n')
            stream.write(f'Takeoff requested time: {takeoff_t if takeoff_t is not None else "not recorded"}\n')
            stream.write(f'Mission start time: {start_t if start_t is not None else "not recorded"}\n')
            stream.write(f'Plot event markers: {len(markers)}\n')
            stream.write(f'Time plot markers: {len(time_markers)}\n')
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
