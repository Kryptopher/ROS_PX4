#!/usr/bin/env python3

import argparse
import csv
import math
import signal
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    VehicleStatus,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleCommandAck,
    OffboardControlMode,
    TrajectorySetpoint,
)


NAV_STATES = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    6: "AUTO_LANDENGFAIL",
    7: "AUTO_LANDGPSFAIL",
    10: "ACRO",
    11: "UNUSED",
    12: "DESCEND",
    13: "TERMINATION",
    14: "OFFBOARD",
    15: "STAB",
    17: "AUTO_TAKEOFF",
    18: "AUTO_LAND",
    19: "AUTO_FOLLOW_TARGET",
    20: "AUTO_PRECLAND",
    21: "ORBIT",
    22: "AUTO_VTOL_TAKEOFF",
    23: "EXTERNAL1",
    24: "EXTERNAL2",
    25: "EXTERNAL3",
    26: "EXTERNAL4",
    27: "EXTERNAL5",
    28: "EXTERNAL6",
    29: "EXTERNAL7",
    30: "EXTERNAL8",
}

ARMING_STATES = {
    1: "DISARMED",
    2: "ARMED",
    3: "STANDBY_ERROR",
    4: "SHUTDOWN",
    5: "IN_AIR_RESTORE",
}


def load_mission_waypoints(mission_file):
    if not mission_file or not Path(mission_file).is_file():
        return []

    waypoints = []
    with open(mission_file, "r", newline="") as f:
        for row in csv.DictReader(f, delimiter="\t"):
            if row.get("type") == "end" or row.get("mode") != "pos":
                continue
            waypoint = (float(row["x"]), float(row["y"]), float(row["z"]))
            if not waypoints or waypoint != waypoints[-1]:
                waypoints.append(waypoint)
    return waypoints


def load_mission_rows(mission_file):
    if not mission_file or not Path(mission_file).is_file():
        return []
    with open(mission_file, "r", newline="") as f:
        return list(csv.DictReader(f, delimiter="\t"))


def get_plot_fonts():
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    try:
        return ImageFont.truetype(font_path, 18), ImageFont.truetype(font_path, 28)
    except OSError:
        font = ImageFont.load_default()
        return font, font


def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def flight_samples(samples):
    armed = [row for row in samples if str(row.get("armed", "")).lower() == "true"]
    return armed if armed else samples


def draw_dashed_polyline(draw, points, fill, width=5, dash_length=14, gap_length=9):
    pattern_length = dash_length + gap_length
    pattern_position = 0.0
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length == 0:
            continue
        distance = 0.0
        while distance < length:
            in_dash = pattern_position < dash_length
            pattern_boundary = dash_length if in_dash else pattern_length
            chunk = min(pattern_boundary - pattern_position, length - distance)
            if in_dash:
                draw.line(
                    [
                        (start[0] + dx * distance / length, start[1] + dy * distance / length),
                        (
                            start[0] + dx * (distance + chunk) / length,
                            start[1] + dy * (distance + chunk) / length,
                        ),
                    ],
                    fill=fill,
                    width=width,
                )
            distance += chunk
            pattern_position = (pattern_position + chunk) % pattern_length


def write_3d_plot(plot_path, mission_file, samples, commands):
    waypoints = load_mission_waypoints(mission_file)
    path = []
    for sample in flight_samples(samples):
        try:
            point = (
                float(sample["x"]),
                float(sample["y"]),
                -float(sample["z"]),  # PX4 NED down becomes mission-frame up.
            )
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in point):
            path.append(point)

    commanded_path = []
    for command in commands:
        point = (
            finite_float(command.get("x")),
            finite_float(command.get("y")),
            finite_float(command.get("z_up")),
        )
        if None not in point:
            commanded_path.append(point)

    if not waypoints and not path and not commanded_path:
        return False

    # Keep rendering bounded for long, high-rate experiments.
    if len(path) > 5000:
        stride = math.ceil(len(path) / 5000)
        path = path[::stride]
    if len(commanded_path) > 5000:
        stride = math.ceil(len(commanded_path) / 5000)
        commanded_path = commanded_path[::stride]

    points = waypoints + path + commanded_path
    mins = [min(point[i] for point in points) for i in range(3)]
    maxs = [max(point[i] for point in points) for i in range(3)]
    spans = [max(maxs[i] - mins[i], 1.0) for i in range(3)]

    width, height = 1600, 1100
    margin = 150
    image = Image.new("RGB", (width, height), "#fbfcfe")
    draw = ImageDraw.Draw(image)
    font, title_font = get_plot_fonts()

    def normalized(point):
        return tuple((point[i] - mins[i]) / spans[i] for i in range(3))

    def raw_project(point):
        x, y, z = normalized(point)
        # Mirror the isometric camera horizontally to match Gazebo's default view.
        return (y - x, 0.52 * (x + y) + 1.25 * z)

    bounds = [
        (x, y, z)
        for x in (mins[0], maxs[0])
        for y in (mins[1], maxs[1])
        for z in (mins[2], maxs[2])
    ]
    projected = [raw_project(point) for point in bounds]
    proj_min_x = min(point[0] for point in projected)
    proj_max_x = max(point[0] for point in projected)
    proj_min_y = min(point[1] for point in projected)
    proj_max_y = max(point[1] for point in projected)
    scale = min(
        (width - 2 * margin) / max(proj_max_x - proj_min_x, 0.1),
        (height - 2 * margin) / max(proj_max_y - proj_min_y, 0.1),
    )

    def project(point):
        px, py = raw_project(point)
        return (
            margin + (px - proj_min_x) * scale,
            height - margin - (py - proj_min_y) * scale,
        )

    ground_z = mins[2]
    grid_color = "#d9e0e8"
    for step in range(6):
        fraction = step / 5
        x = mins[0] + fraction * spans[0]
        y = mins[1] + fraction * spans[1]
        draw.line([project((x, mins[1], ground_z)), project((x, maxs[1], ground_z))], fill=grid_color, width=2)
        draw.line([project((mins[0], y, ground_z)), project((maxs[0], y, ground_z))], fill=grid_color, width=2)

    origin = (mins[0], mins[1], ground_z)
    axes = [
        ((maxs[0], mins[1], ground_z), "X forward"),
        ((mins[0], maxs[1], ground_z), "Y right"),
        ((mins[0], mins[1], maxs[2]), "Z up"),
    ]
    for endpoint, label in axes:
        draw.line([project(origin), project(endpoint)], fill="#58677a", width=4)
        ex, ey = project(endpoint)
        draw.text((ex + 8, ey - 12), label, fill="#344054", font=font)

    if len(path) > 1:
        draw.line([project(point) for point in path], fill="#e87524", width=5, joint="curve")
        for point, color, label in [(path[0], "#16803c", "start"), (path[-1], "#b42318", "end")]:
            px, py = project(point)
            draw.ellipse((px - 7, py - 7, px + 7, py + 7), fill=color, outline="white", width=2)
            draw.text((px + 10, py - 9), label, fill=color, font=font)

    if len(commanded_path) > 1:
        draw_dashed_polyline(
            draw,
            [project(point) for point in commanded_path],
            fill="#6d28d9",
            width=6,
        )

    for index, waypoint in enumerate(waypoints, start=1):
        px, py = project(waypoint)
        draw.ellipse((px - 10, py - 10, px + 10, py + 10), fill="#2563eb", outline="white", width=3)
        draw.text((px + 11, py - 10), f"WP{index}", fill="#1747a6", font=font)

    draw.text((margin, 35), "Mission Commands and Recorded Flight Path", fill="#101828", font=title_font)
    draw.ellipse((margin + 14, 78, margin + 28, 92), fill="#2563eb")
    draw.text((margin + 55, 78), "Mission waypoints", fill="#344054", font=font)
    draw_dashed_polyline(draw, [(margin + 260, 85), (margin + 305, 85)], fill="#6d28d9", width=6)
    draw.text((margin + 315, 78), "Streamed command", fill="#344054", font=font)
    draw.line([(margin + 560, 85), (margin + 605, 85)], fill="#e87524", width=5)
    draw.text((margin + 615, 78), "Recorded path", fill="#344054", font=font)
    draw.text(
        (margin, height - 45),
        f"Waypoints: {len(waypoints)}   Command samples: {len(commanded_path)}   Recorded samples: {len(path)}",
        fill="#667085",
        font=font,
    )

    image.save(plot_path)
    return True


def write_xy_plot(plot_path, mission_file, samples, commands):
    waypoints = load_mission_waypoints(mission_file)
    actual = [
        (finite_float(row.get("x")), finite_float(row.get("y")))
        for row in flight_samples(samples)
    ]
    actual = [point for point in actual if None not in point]
    commanded = [
        (finite_float(row.get("x")), finite_float(row.get("y")))
        for row in commands
    ]
    commanded = [point for point in commanded if None not in point]
    points = waypoints + actual + commanded
    if not points:
        return False

    width, height, margin = 1400, 1100, 140
    image = Image.new("RGB", (width, height), "#fbfcfe")
    draw = ImageDraw.Draw(image)
    font, title_font = get_plot_fonts()
    min_x, max_x = min(p[0] for p in points), max(p[0] for p in points)
    min_y, max_y = min(p[1] for p in points), max(p[1] for p in points)
    pad = max(max_x - min_x, max_y - min_y, 1.0) * 0.12
    min_x, max_x, min_y, max_y = min_x - pad, max_x + pad, min_y - pad, max_y + pad
    scale = min((width - 2 * margin) / (max_x - min_x), (height - 2 * margin) / (max_y - min_y))

    def project(point):
        # Mirror X horizontally to match Gazebo's default camera view.
        return (
            width - margin - (point[0] - min_x) * scale,
            height - margin - (point[1] - min_y) * scale,
        )

    for step in range(6):
        fraction = step / 5
        x = min_x + fraction * (max_x - min_x)
        y = min_y + fraction * (max_y - min_y)
        draw.line([project((x, min_y)), project((x, max_y))], fill="#e2e8f0", width=2)
        draw.line([project((min_x, y)), project((max_x, y))], fill="#e2e8f0", width=2)

    if len(actual) > 1:
        draw.line([project(p) for p in actual], fill="#e87524", width=5)
    if len(commanded) > 1:
        draw_dashed_polyline(draw, [project(p) for p in commanded], fill="#6d28d9", width=6)
    for index, waypoint in enumerate(waypoints, start=1):
        px, py = project(waypoint[:2])
        draw.ellipse((px - 9, py - 9, px + 9, py + 9), fill="#2563eb", outline="white", width=3)
        draw.text((px + 10, py - 10), f"WP{index}", fill="#1747a6", font=font)

    draw.text((margin, 35), "Top-Down XY Position", fill="#101828", font=title_font)
    draw.ellipse((margin + 14, 83, margin + 28, 97), fill="#2563eb")
    draw.text((margin + 55, 81), "Mission waypoints", fill="#344054", font=font)
    draw_dashed_polyline(draw, [(margin + 260, 90), (margin + 305, 90)], fill="#6d28d9", width=6)
    draw.text((margin + 315, 81), "Streamed command", fill="#344054", font=font)
    draw.line([(margin + 560, 90), (margin + 605, 90)], fill="#e87524", width=5)
    draw.text((margin + 615, 81), "Actual path", fill="#344054", font=font)
    draw.text((margin, height - 45), "X forward (mirrored to Gazebo view)    Y right", fill="#667085", font=font)
    image.save(plot_path)
    return True


def write_z_plot(plot_path, samples, commands):
    actual = [
        (finite_float(row.get("t")), -finite_float(row.get("z")) if finite_float(row.get("z")) is not None else None)
        for row in flight_samples(samples)
    ]
    actual = [point for point in actual if None not in point]
    commanded = [
        (finite_float(row.get("t")), finite_float(row.get("z_up")))
        for row in commands
    ]
    commanded = [point for point in commanded if None not in point]
    points = actual + commanded
    if not points:
        return False

    width, height, margin = 1500, 900, 130
    image = Image.new("RGB", (width, height), "#fbfcfe")
    draw = ImageDraw.Draw(image)
    font, title_font = get_plot_fonts()
    min_t, max_t = min(p[0] for p in points), max(p[0] for p in points)
    min_z, max_z = min(p[1] for p in points), max(p[1] for p in points)
    t_span, z_span = max(max_t - min_t, 1.0), max(max_z - min_z, 0.5)
    min_z -= z_span * 0.1
    max_z += z_span * 0.1

    def project(point):
        return (
            margin + (point[0] - min_t) / t_span * (width - 2 * margin),
            height - margin - (point[1] - min_z) / (max_z - min_z) * (height - 2 * margin),
        )

    for step in range(6):
        fraction = step / 5
        t = min_t + fraction * t_span
        z = min_z + fraction * (max_z - min_z)
        draw.line([project((t, min_z)), project((t, max_z))], fill="#e2e8f0", width=2)
        draw.line([project((min_t, z)), project((max_t, z))], fill="#e2e8f0", width=2)
        tx, _ = project((t, min_z))
        _, zy = project((min_t, z))
        draw.text((tx - 20, height - margin + 12), f"{t:.0f}", fill="#667085", font=font)
        draw.text((35, zy - 10), f"{z:.2f}", fill="#667085", font=font)

    if len(commanded) > 1:
        draw.line([project(p) for p in commanded], fill="#7c3aed", width=4)
    if len(actual) > 1:
        draw.line([project(p) for p in actual], fill="#e87524", width=5)
    draw.text((margin, 35), "Height vs Time", fill="#101828", font=title_font)
    draw.text((width // 2 - 60, height - 50), "Log time (s)", fill="#344054", font=font)
    draw.text((35, 80), "Z up (m)", fill="#344054", font=font)
    draw.line([(margin, 90), (margin + 45, 90)], fill="#7c3aed", width=5)
    draw.text((margin + 55, 81), "Streamed command", fill="#344054", font=font)
    draw.line([(margin + 310, 90), (margin + 355, 90)], fill="#e87524", width=5)
    draw.text((margin + 365, 81), "Actual height", fill="#344054", font=font)
    image.save(plot_path)
    return True


class MissionLogbook(Node):
    def __init__(self, label: str, duration: float, sample_rate_hz: float, mission_file: str):
        super().__init__("mission_logbook")

        self.label = label
        self.duration = duration
        self.sample_rate_hz = sample_rate_hz
        self.mission_file = mission_file
        self.start_ros_time = self.get_clock().now().nanoseconds / 1e9
        self.done = False

        self.log_dir = Path.home() / ".ros" / "mission_logs"
        self.log_dir.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        safe_label = "".join(c if c.isalnum() or c in "-_" else "_" for c in label)
        self.base_path = self.log_dir / f"{stamp}__{safe_label}"

        self.events = []
        self.samples = []
        self.commands = []

        self.last = {}
        self.seen_offboard_heartbeat = False
        self.seen_trajectory_setpoint = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.vehicle_control_mode_cb, qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_cb, qos)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.command_ack_cb, qos)
        self.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode", self.offboard_control_mode_cb, qos)
        self.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.trajectory_setpoint_cb, qos)
        self.create_subscription(String, "/mission_executor/event", self.mission_executor_event_cb, 10)

        self.create_timer(1.0 / self.sample_rate_hz, self.sample_timer_cb)

        if self.duration > 0:
            self.create_timer(0.25, self.duration_timer_cb)

        self.log_event("LOGBOOK_START", f"label={self.label}")
        self.get_logger().warn(f"Mission logbook running. Logs will save to: {self.base_path}_events.csv")

    def elapsed(self):
        return (self.get_clock().now().nanoseconds / 1e9) - self.start_ros_time

    def log_event(self, event_type, detail):
        t = self.elapsed()
        self.events.append({
            "t": f"{t:.3f}",
            "event_type": event_type,
            "detail": str(detail),
        })
        self.get_logger().warn(f"[{t:7.2f}s] {event_type}: {detail}")

    def change_event(self, key, value, event_type, formatter=str):
        old = self.last.get(key, None)
        if old != value:
            self.last[key] = value
            self.log_event(event_type, formatter(value))

    def vehicle_status_cb(self, msg):
        nav = int(msg.nav_state)
        arm = int(msg.arming_state)

        self.change_event(
            "nav_state",
            nav,
            "NAV_STATE",
            lambda v: f"{v} ({NAV_STATES.get(v, 'UNKNOWN')})",
        )

        self.change_event(
            "arming_state",
            arm,
            "ARMING_STATE",
            lambda v: f"{v} ({ARMING_STATES.get(v, 'UNKNOWN')})",
        )

        self.change_event("failsafe", bool(msg.failsafe), "FAILSAFE")
        self.change_event("gcs_connection_lost", bool(msg.gcs_connection_lost), "GCS_CONNECTION_LOST")
        self.change_event("pre_flight_checks_pass", bool(msg.pre_flight_checks_pass), "PREFLIGHT_CHECKS_PASS")

    def vehicle_control_mode_cb(self, msg):
        self.change_event("flag_armed", bool(msg.flag_armed), "CONTROL_FLAG_ARMED")
        self.change_event(
            "flag_control_offboard_enabled",
            bool(msg.flag_control_offboard_enabled),
            "CONTROL_FLAG_OFFBOARD",
        )

    def local_position_cb(self, msg):
        self.change_event("xy_valid", bool(msg.xy_valid), "LOCAL_POSITION_XY_VALID")
        self.change_event("z_valid", bool(msg.z_valid), "LOCAL_POSITION_Z_VALID")
        self.change_event("v_xy_valid", bool(msg.v_xy_valid), "LOCAL_VELOCITY_XY_VALID")
        self.change_event("v_z_valid", bool(msg.v_z_valid), "LOCAL_VELOCITY_Z_VALID")
        self.change_event("xy_global", bool(msg.xy_global), "GLOBAL_XY_VALID")
        self.change_event("z_global", bool(msg.z_global), "GLOBAL_Z_VALID")
        self.change_event("heading_good_for_control", bool(msg.heading_good_for_control), "HEADING_GOOD_FOR_CONTROL")

        self.last["x"] = float(msg.x)
        self.last["y"] = float(msg.y)
        self.last["z"] = float(msg.z)
        self.last["vx"] = float(msg.vx)
        self.last["vy"] = float(msg.vy)
        self.last["vz"] = float(msg.vz)
        self.last["heading_deg"] = math.degrees(float(msg.heading)) % 360.0

    def command_ack_cb(self, msg):
        command = getattr(msg, "command", None)
        result = getattr(msg, "result", None)
        self.log_event("VEHICLE_COMMAND_ACK", f"command={command}, result={result}")

    def offboard_control_mode_cb(self, msg):
        if not self.seen_offboard_heartbeat:
            self.seen_offboard_heartbeat = True
            self.log_event(
                "OFFBOARD_HEARTBEAT_START",
                f"position={msg.position}, velocity={msg.velocity}, acceleration={msg.acceleration}",
            )

    def trajectory_setpoint_cb(self, msg):
        if not self.seen_trajectory_setpoint:
            self.seen_trajectory_setpoint = True
            self.log_event("TRAJECTORY_SETPOINT_START", "trajectory setpoint stream detected")
        self.commands.append({
            "t": f"{self.elapsed():.6f}",
            "x": float(msg.position[0]),
            "y": float(msg.position[1]),
            "z_ned": float(msg.position[2]),
            "z_up": -float(msg.position[2]),
            "vx": float(msg.velocity[0]),
            "vy": float(msg.velocity[1]),
            "vz_ned": float(msg.velocity[2]),
            "vz_up": -float(msg.velocity[2]),
            "ax": float(msg.acceleration[0]),
            "ay": float(msg.acceleration[1]),
            "az_ned": float(msg.acceleration[2]),
            "yaw_rad": float(msg.yaw),
            "heading_deg": math.degrees(float(msg.yaw)) % 360.0,
            "yawspeed_rad_s": float(msg.yawspeed),
        })

    def mission_executor_event_cb(self, msg):
        data = msg.data
        if "|" in data:
            event_type, detail = data.split("|", 1)
        else:
            event_type, detail = data, ""

        self.log_event(f"MISSION_EXECUTOR_{event_type}", detail)
        if event_type in {"MISSION_COMPLETE_HOLDING", "MISSION_ABORTED"}:
            self.write_logs()

    def sample_timer_cb(self):
        if "nav_state" not in self.last and "x" not in self.last:
            return

        self.samples.append({
            "t": f"{self.elapsed():.6f}",
            "nav_state": self.last.get("nav_state", ""),
            "arming_state": self.last.get("arming_state", ""),
            "failsafe": self.last.get("failsafe", ""),
            "gcs_connection_lost": self.last.get("gcs_connection_lost", ""),
            "pre_flight_checks_pass": self.last.get("pre_flight_checks_pass", ""),
            "offboard_enabled": self.last.get("flag_control_offboard_enabled", ""),
            "armed": self.last.get("flag_armed", ""),
            "landed": self.last.get("landed", ""),
            "ground_contact": self.last.get("ground_contact", ""),
            "xy_valid": self.last.get("xy_valid", ""),
            "z_valid": self.last.get("z_valid", ""),
            "x": self.last.get("x", ""),
            "y": self.last.get("y", ""),
            "z": self.last.get("z", ""),
            "vx": self.last.get("vx", ""),
            "vy": self.last.get("vy", ""),
            "vz": self.last.get("vz", ""),
            "heading_deg": self.last.get("heading_deg", ""),
        })

    def duration_timer_cb(self):
        if self.elapsed() >= self.duration:
            self.log_event("LOGBOOK_DURATION_COMPLETE", f"duration={self.duration}s")
            self.done = True

    def write_logs(self):
        events_path = Path(str(self.base_path) + "_events.csv")
        samples_path = Path(str(self.base_path) + "_samples.csv")
        summary_path = Path(str(self.base_path) + "_summary.txt")
        plot_path = Path(str(self.base_path) + "_trajectory_3d.png")
        commands_path = Path(str(self.base_path) + "_commands.csv")
        xy_plot_path = Path(str(self.base_path) + "_xy.png")
        z_plot_path = Path(str(self.base_path) + "_height.png")

        with open(events_path, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["t", "event_type", "detail"],
                lineterminator="\n",
            )
            writer.writeheader()
            writer.writerows(self.events)

        sample_fields = [
            "t",
            "nav_state",
            "arming_state",
            "failsafe",
            "gcs_connection_lost",
            "pre_flight_checks_pass",
            "offboard_enabled",
            "armed",
            "landed",
            "ground_contact",
            "xy_valid",
            "z_valid",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "heading_deg",
        ]
        with open(samples_path, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=sample_fields,
                lineterminator="\n",
            )
            writer.writeheader()
            writer.writerows(self.samples)

        command_fields = [
            "t", "x", "y", "z_ned", "z_up", "vx", "vy", "vz_ned", "vz_up",
            "ax", "ay", "az_ned", "yaw_rad", "heading_deg", "yawspeed_rad_s",
        ]
        with open(commands_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=command_fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(self.commands)

        final = self.samples[-1] if self.samples else {}
        sample_duration = 0.0
        achieved_rate = 0.0
        if len(self.samples) > 1:
            sample_duration = float(self.samples[-1]["t"]) - float(self.samples[0]["t"])
            if sample_duration > 0:
                achieved_rate = (len(self.samples) - 1) / sample_duration

        with open(summary_path, "w") as f:
            f.write("Mission Logbook Summary\n")
            f.write(f"Label: {self.label}\n")
            f.write(f"Events CSV: {events_path}\n")
            f.write(f"Samples CSV: {samples_path}\n")
            f.write(f"Streamed commands CSV: {commands_path}\n")
            f.write(f"Event count: {len(self.events)}\n")
            f.write(f"Sample count: {len(self.samples)}\n")
            f.write(f"Requested sample rate: {self.sample_rate_hz:g} Hz\n")
            f.write(f"Achieved sample rate: {achieved_rate:.3f} Hz\n")
            f.write(f"3D trajectory plot: {plot_path}\n")
            f.write(f"Top-down XY plot: {xy_plot_path}\n")
            f.write(f"Height plot: {z_plot_path}\n")
            f.write("\nFinal sample:\n")
            for k, v in final.items():
                f.write(f"  {k}: {v}\n")

        if write_3d_plot(plot_path, self.mission_file, self.samples, self.commands):
            self.get_logger().warn(f"Saved 3D plot: {plot_path}")
        if write_xy_plot(xy_plot_path, self.mission_file, self.samples, self.commands):
            self.get_logger().warn(f"Saved XY plot: {xy_plot_path}")
        if write_z_plot(z_plot_path, self.samples, self.commands):
            self.get_logger().warn(f"Saved height plot: {z_plot_path}")
        self.get_logger().warn(f"Saved commands: {commands_path}")
        self.get_logger().warn(f"Saved events:  {events_path}")
        self.get_logger().warn(f"Saved samples: {samples_path}")
        self.get_logger().warn(f"Saved summary: {summary_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="sitl_mission")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means run until Ctrl+C.")
    parser.add_argument("--sample-rate-hz", type=float, default=20.0)
    parser.add_argument("--mission-file", default="")
    args = parser.parse_args()
    if args.sample_rate_hz <= 0:
        parser.error("--sample-rate-hz must be greater than zero")

    rclpy.init()
    node = MissionLogbook(args.label, args.duration, args.sample_rate_hz, args.mission_file)

    stop_requested = {"value": False}

    def handle_signal(signum, frame):
        stop_requested["value"] = True
        node.log_event("LOGBOOK_STOP_REQUESTED", f"signal={signum}")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not node.done and not stop_requested["value"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.log_event("LOGBOOK_STOP", "writing logs")
        node.write_logs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
