#!/usr/bin/env python3

import argparse
import csv
import signal
from datetime import datetime
from pathlib import Path

from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    VehicleStatus,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleCommandAck,
    EstimatorStatusFlags,
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


class MissionLogbook(Node):
    def __init__(self, label: str, duration: float):
        super().__init__("mission_logbook")

        self.label = label
        self.duration = duration
        self.start_ros_time = self.get_clock().now().nanoseconds / 1e9
        self.done = False

        self.log_dir = Path.home() / ".ros" / "mission_logs"
        self.log_dir.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = "".join(c if c.isalnum() or c in "-_" else "_" for c in label)
        self.base_path = self.log_dir / f"{stamp}_{safe_label}"

        self.events = []
        self.samples = []

        self.last = {}
        self.seen_offboard_heartbeat = False
        self.seen_trajectory_setpoint = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_cb, qos)
        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.vehicle_control_mode_cb, qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_cb, qos)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.command_ack_cb, qos)
        self.create_subscription(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", self.estimator_flags_cb, qos)
        self.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode", self.offboard_control_mode_cb, qos)
        self.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.trajectory_setpoint_cb, qos)
        self.create_subscription(String, "/mission_executor/event", self.mission_executor_event_cb, 10)

        self.create_timer(1.0, self.sample_timer_cb)

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

    def command_ack_cb(self, msg):
        command = getattr(msg, "command", None)
        result = getattr(msg, "result", None)
        self.log_event("VEHICLE_COMMAND_ACK", f"command={command}, result={result}")

    def estimator_flags_cb(self, msg):
        keys = [
            "cs_tilt_align",
            "cs_yaw_align",
            "cs_gnss_pos",
            "cs_gnss_vel",
            "cs_baro_hgt",
            "cs_ev_pos",
            "cs_ev_vel",
            "cs_ev_hgt",
            "cs_ev_yaw",
            "reject_hor_pos",
            "reject_ver_pos",
            "reject_yaw",
            "fs_bad_hdg",
            "fs_bad_mag_x",
            "fs_bad_mag_y",
            "fs_bad_mag_z",
        ]

        for k in keys:
            if hasattr(msg, k):
                self.change_event(f"est_{k}", bool(getattr(msg, k)), f"ESTIMATOR_{k.upper()}")

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

    def mission_executor_event_cb(self, msg):
        data = msg.data
        if "|" in data:
            event_type, detail = data.split("|", 1)
        else:
            event_type, detail = data, ""

        self.log_event(f"MISSION_EXECUTOR_{event_type}", detail)

    def sample_timer_cb(self):
        self.samples.append({
            "t": f"{self.elapsed():.3f}",
            "nav_state": self.last.get("nav_state", ""),
            "arming_state": self.last.get("arming_state", ""),
            "failsafe": self.last.get("failsafe", ""),
            "gcs_connection_lost": self.last.get("gcs_connection_lost", ""),
            "pre_flight_checks_pass": self.last.get("pre_flight_checks_pass", ""),
            "offboard_enabled": self.last.get("flag_control_offboard_enabled", ""),
            "armed": self.last.get("flag_armed", ""),
            "xy_valid": self.last.get("xy_valid", ""),
            "z_valid": self.last.get("z_valid", ""),
            "x": self.last.get("x", ""),
            "y": self.last.get("y", ""),
            "z": self.last.get("z", ""),
            "vx": self.last.get("vx", ""),
            "vy": self.last.get("vy", ""),
            "vz": self.last.get("vz", ""),
        })

    def duration_timer_cb(self):
        if self.elapsed() >= self.duration:
            self.log_event("LOGBOOK_DURATION_COMPLETE", f"duration={self.duration}s")
            self.done = True

    def write_logs(self):
        events_path = Path(str(self.base_path) + "_events.csv")
        samples_path = Path(str(self.base_path) + "_samples.csv")
        summary_path = Path(str(self.base_path) + "_summary.txt")

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
            "xy_valid",
            "z_valid",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
        ]
        with open(samples_path, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=sample_fields,
                lineterminator="\n",
            )
            writer.writeheader()
            writer.writerows(self.samples)

        final = self.samples[-1] if self.samples else {}

        with open(summary_path, "w") as f:
            f.write("Mission Logbook Summary\n")
            f.write(f"Label: {self.label}\n")
            f.write(f"Events CSV: {events_path}\n")
            f.write(f"Samples CSV: {samples_path}\n")
            f.write(f"Event count: {len(self.events)}\n")
            f.write(f"Sample count: {len(self.samples)}\n")
            f.write("\nFinal sample:\n")
            for k, v in final.items():
                f.write(f"  {k}: {v}\n")

        self.get_logger().warn(f"Saved events:  {events_path}")
        self.get_logger().warn(f"Saved samples: {samples_path}")
        self.get_logger().warn(f"Saved summary: {summary_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="sitl_mission")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means run until Ctrl+C.")
    args = parser.parse_args()

    rclpy.init()
    node = MissionLogbook(args.label, args.duration)

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
