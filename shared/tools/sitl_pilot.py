#!/usr/bin/env python3
"""SITL stand-in for the RC pilot: centered sticks, plus typed hold/position/land commands."""

import os
import threading

import rclpy
from px4_msgs.msg import ManualControlSetpoint, VehicleCommand
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)


class SitlPilot(Node):
    def __init__(self):
        super().__init__('sitl_pilot')
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', QOS)
        self.stick_pub = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', QOS)
        self.create_timer(0.05, self._sticks)

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _sticks(self):
        msg = ManualControlSetpoint()
        msg.timestamp = msg.timestamp_sample = self._now_us()
        msg.valid = True
        msg.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        self.stick_pub.publish(msg)

    def send(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1, msg.param2, msg.param3 = float(param1), float(param2), float(param3)
        msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main():
    rclpy.init()
    node = SitlPilot()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('Centered sticks publishing. Type hold, position, or land.', flush=True)
    actions = {
        'hold': (VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 3),
        'position': (VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 3),
        'land': (VehicleCommand.VEHICLE_CMD_NAV_LAND,),
    }
    try:
        while True:
            word = input().strip().lower()
            if word in actions:
                node.send(*actions[word])
                print(f'sent {word}', flush=True)
            elif word:
                print('Type hold, position, or land.', flush=True)
    except (EOFError, KeyboardInterrupt):
        pass
    # The spin thread is still running; leave without tearing it down mid-callback.
    os._exit(0)


if __name__ == '__main__':
    main()
