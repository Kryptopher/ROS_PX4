#!/usr/bin/env python3

import argparse
import curses
import os
from pathlib import Path
import subprocess

from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleControlMode, VehicleStatus


class MissionControlNode(Node):
    def __init__(self, mission_dir, kill_sitl_command='', shutdown_command=''):
        super().__init__('mission_control_ui')
        self.mission_dir = Path(mission_dir).expanduser().resolve()
        self.kill_sitl_command = kill_sitl_command
        self.shutdown_command = shutdown_command
        self.control_pub = self.create_publisher(String, '/mission/control', 10)
        self.event_sub = self.create_subscription(String, '/mission_executor/event', self.event_cb, 10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_cb, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_cb, px4_qos)

        self.armed = False
        self.offboard = False
        self.nav_state = None
        self.arming_state = None
        self.last_event = ''
        self.selected_mission = ''

    def event_cb(self, msg):
        self.last_event = msg.data
        if msg.data.startswith('MISSION_SELECTED|'):
            self.selected_mission = msg.data.split('|', 1)[1]

    def control_mode_cb(self, msg):
        self.armed = bool(msg.flag_armed)
        self.offboard = bool(msg.flag_control_offboard_enabled)

    def status_cb(self, msg):
        self.nav_state = int(msg.nav_state)
        self.arming_state = int(msg.arming_state)
        if self.arming_state == 2:
            self.armed = True
        if self.nav_state == 14:
            self.offboard = True

    def publish(self, command, value=''):
        msg = String()
        msg.data = command if not value else f'{command}|{value}'
        self.control_pub.publish(msg)

    def missions(self):
        return sorted(self.mission_dir.glob('*.tsv'))

    def kill_sitl(self):
        if not self.kill_sitl_command:
            self.last_event = 'Kill SITL is not configured for this UI'
            return
        self.run_shell_command(self.kill_sitl_command)
        self.last_event = 'Kill SITL requested'

    def shutdown_and_save_logs(self):
        if not self.shutdown_command:
            self.last_event = 'Shutdown is not configured for this UI'
            return
        self.run_shell_command(self.shutdown_command)
        self.last_event = 'Shutdown / Save Logs requested'

    def run_shell_command(self, command):
        subprocess.Popen(
            command,
            shell=True,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def draw_menu(stdscr, node, items, selected):
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    title = 'PX4 DDS Mission Control'
    stdscr.addstr(0, 0, title[:width - 1], curses.A_BOLD)
    stdscr.addstr(2, 0, f'Armed: {node.armed}   Offboard: {node.offboard}'[:width - 1])
    stdscr.addstr(3, 0, f'nav_state: {node.nav_state}   arming_state: {node.arming_state}'[:width - 1])
    stdscr.addstr(4, 0, f'Mission folder: {node.mission_dir}'[:width - 1])
    if node.selected_mission:
        stdscr.addstr(5, 0, f'Mission: {node.selected_mission}'[:width - 1])
    if node.last_event:
        stdscr.addstr(6, 0, f'Event: {node.last_event}'[:width - 1])
    stdscr.addstr(8, 0, 'Use arrows, Enter to select, q to quit.'[:width - 1])

    for idx, item in enumerate(items):
        y = 10 + idx
        if y >= height:
            break
        attr = curses.A_REVERSE if idx == selected else curses.A_NORMAL
        stdscr.addstr(y, 0, item[:width - 1], attr)
    stdscr.refresh()


def mission_picker(stdscr, node):
    missions = node.missions()
    if not missions:
        node.last_event = f'No .tsv missions in {node.mission_dir}'
        return
    selected = 0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        stdscr.addstr(0, 0, 'Select Mission'[:width - 1], curses.A_BOLD)
        stdscr.addstr(2, 0, 'Enter selects, Esc returns.'[:width - 1])
        for idx, mission in enumerate(missions):
            y = 4 + idx
            if y >= height:
                break
            attr = curses.A_REVERSE if idx == selected else curses.A_NORMAL
            stdscr.addstr(y, 0, mission.name[:width - 1], attr)
        stdscr.refresh()
        key = stdscr.getch()
        if key == curses.KEY_UP:
            selected = (selected - 1) % len(missions)
        elif key == curses.KEY_DOWN:
            selected = (selected + 1) % len(missions)
        elif key in (10, 13):
            node.selected_mission = str(missions[selected])
            node.publish('SELECT_MISSION', str(missions[selected]))
            return
        elif key == 27:
            return


def curses_main(stdscr, node):
    curses.curs_set(0)
    stdscr.nodelay(True)
    selected = 0
    items = [
        'Switch Offboard',
        'Arm',
        'Select Mission',
        'Takeoff / Start TSV',
        'Land',
        'Return to Launch',
        'Disarm',
        'Quit UI',
    ]
    commands = ['OFFBOARD', 'ARM', 'SELECT', 'TAKEOFF', 'LAND', 'RTL', 'DISARM', 'QUIT']
    if node.kill_sitl_command:
        items.insert(-1, 'Kill SITL')
        commands.insert(-1, 'KILL_SITL')
    if node.shutdown_command:
        items.insert(-1, 'Shutdown / Save Logs')
        commands.insert(-1, 'SHUTDOWN')

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        draw_menu(stdscr, node, items, selected)
        key = stdscr.getch()
        if key == -1:
            curses.napms(100)
            continue
        if key == curses.KEY_UP:
            selected = (selected - 1) % len(items)
        elif key == curses.KEY_DOWN:
            selected = (selected + 1) % len(items)
        elif key in (ord('q'), ord('Q')):
            return
        elif key in (10, 13):
            command = commands[selected]
            if command == 'QUIT':
                return
            if command == 'KILL_SITL':
                node.kill_sitl()
                return
            if command == 'SHUTDOWN':
                node.shutdown_and_save_logs()
                return
            if command == 'SELECT':
                stdscr.nodelay(False)
                mission_picker(stdscr, node)
                stdscr.nodelay(True)
            else:
                node.publish(command)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--mission-dir',
        default=os.environ.get('MISSION_DIR', str(Path.home() / 'ROS_PX4' / 'missions')),
        help='Folder containing selectable .tsv mission files.',
    )
    parser.add_argument(
        '--kill-sitl-command',
        default=os.environ.get('KILL_SITL_COMMAND', ''),
        help='Optional shell command used by the Kill SITL menu item.',
    )
    parser.add_argument(
        '--shutdown-command',
        default=os.environ.get('SHUTDOWN_COMMAND', ''),
        help='Optional shell command used by the Shutdown / Save Logs menu item.',
    )
    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = MissionControlNode(
        parsed_args.mission_dir,
        parsed_args.kill_sitl_command,
        parsed_args.shutdown_command,
    )
    try:
        curses.wrapper(curses_main, node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
