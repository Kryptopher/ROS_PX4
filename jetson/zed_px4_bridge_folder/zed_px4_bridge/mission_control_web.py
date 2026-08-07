#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import subprocess
import threading
from collections import deque
from pathlib import Path

from flask import Flask, jsonify, request, Response
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, VehicleStatus


NAV_STATES = {
    0: 'Manual',
    1: 'Altitude',
    2: 'Position',
    3: 'Mission',
    4: 'Loiter',
    5: 'RTL',
    10: 'Acro',
    14: 'Offboard',
    15: 'Stabilized',
    17: 'Takeoff',
    18: 'Land',
}

ARMING_STATES = {
    1: 'Disarmed',
    2: 'Armed',
}


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>PX4 Mission Control</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f6f7f4;
      --panel: #ffffff;
      --text: #1f2933;
      --muted: #63717f;
      --line: #d6ddd8;
      --ok: #147d64;
      --warn: #a05a00;
      --bad: #b42318;
      --blue: #2457a6;
      --button: #293241;
      --buttonText: #ffffff;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      letter-spacing: 0;
    }
    .app {
      min-height: 100vh;
      display: grid;
      grid-template-rows: auto 1fr;
    }
    header {
      border-bottom: 1px solid var(--line);
      background: #eef2ed;
      padding: 12px 18px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
    }
    h1 { font-size: 20px; margin: 0; font-weight: 700; }
    .sub { color: var(--muted); font-size: 13px; margin-top: 2px; }
    main {
      display: grid;
      grid-template-columns: minmax(310px, 0.85fr) minmax(420px, 1.15fr);
      gap: 14px;
      padding: 14px;
      max-width: 1280px;
      width: 100%;
      margin: 0 auto;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
    }
    h2 { font-size: 15px; margin: 0 0 10px; }
    .statusGrid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .metric {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 9px;
      min-height: 62px;
      background: #fbfcfb;
    }
    .metric .label { color: var(--muted); font-size: 12px; }
    .metric .value { font-size: 18px; font-weight: 700; margin-top: 5px; overflow-wrap: anywhere; }
    .ok { color: var(--ok); }
    .warn { color: var(--warn); }
    .bad { color: var(--bad); }
    .controls {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    button, select {
      width: 100%;
      min-height: 42px;
      border-radius: 6px;
      border: 1px solid var(--line);
      font: inherit;
      font-size: 14px;
    }
    button {
      background: var(--button);
      color: var(--buttonText);
      cursor: pointer;
      font-weight: 650;
    }
    button.secondary { background: #ffffff; color: var(--text); }
    button.danger { background: var(--bad); color: #ffffff !important; }
    button.warn { background: var(--warn); color: #ffffff !important; }
    button:disabled { opacity: 0.45; cursor: not-allowed; }
    select { background: #fff; color: var(--text); padding: 0 10px; }
    .commandStatus {
      min-height: 20px;
      color: var(--muted);
      font-size: 12px;
      margin-top: 8px;
      overflow-wrap: anywhere;
    }
    .missionRow {
      display: grid;
      grid-template-columns: 1fr 150px;
      gap: 8px;
      margin-bottom: 12px;
    }
    .eventLog {
      height: 260px;
      overflow: auto;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fbfcfb;
      padding: 8px;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      line-height: 1.45;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }
    th, td {
      border-bottom: 1px solid var(--line);
      text-align: left;
      padding: 6px;
      white-space: nowrap;
    }
    th { color: var(--muted); font-weight: 650; }
    .missionPreview {
      overflow: auto;
      max-height: 300px;
      border: 1px solid var(--line);
      border-radius: 6px;
    }
    .note {
      color: var(--muted);
      font-size: 12px;
      margin-top: 8px;
    }
    .banner {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 10px;
      margin-bottom: 12px;
      background: #fff7ed;
      color: #7c2d12;
      font-weight: 650;
    }
    @media (max-width: 850px) {
      main { grid-template-columns: 1fr; }
      .missionRow { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="app">
    <header>
      <div>
        <h1>PX4 Mission Control</h1>
        <div class="sub" id="folder">Mission folder</div>
      </div>
      <div class="sub" id="updated">Waiting for status</div>
    </header>
    <main>
      <div>
        <section>
          <h2>Vehicle Status</h2>
          <div id="failsafeBanner"></div>
          <div class="statusGrid">
            <div class="metric"><div class="label">DDS</div><div class="value" id="dds">Unknown</div></div>
            <div class="metric"><div class="label">Mode</div><div class="value" id="mode">Unknown</div></div>
            <div class="metric"><div class="label">Arming</div><div class="value" id="arming">Unknown</div></div>
            <div class="metric"><div class="label">Offboard</div><div class="value" id="offboard">Unknown</div></div>
            <div class="metric"><div class="label">Local Position</div><div class="value" id="local">Unknown</div></div>
            <div class="metric"><div class="label">Altitude</div><div class="value" id="alt">--</div></div>
            <div class="metric"><div class="label">Speed</div><div class="value" id="speed">--</div></div>
            <div class="metric"><div class="label">Selected Mission</div><div class="value" id="selectedMission">--</div></div>
          </div>
        </section>
        <section style="margin-top:14px">
          <h2>Mission</h2>
          <div class="missionRow">
            <select id="missions"></select>
            <button type="button" class="secondary" id="selectMissionBtn">Select Mission</button>
          </div>
          <div class="missionPreview">
            <table>
              <thead><tr><th>t</th><th>type</th><th>mode</th><th>profile</th><th>x</th><th>y</th><th>z</th><th>heading</th></tr></thead>
              <tbody id="preview"></tbody>
            </table>
          </div>
          <div class="note">For linear rows, speed is set by distance divided by the time gap to the next row.</div>
        </section>
      </div>
      <div>
        <section>
          <h2>Controls</h2>
          <div class="controls">
            <button type="button" data-command="offboard">Switch Offboard</button>
            <button type="button" data-command="arm">Arm</button>
            <button type="button" id="takeoffBtn" data-command="takeoff" data-confirm="Start the selected TSV mission?">Takeoff / Start TSV</button>
            <button type="button" class="secondary" data-command="disarm">Disarm</button>
            <button type="button" class="warn" data-command="land" data-confirm="Command Land now?"><span>Land Now</span></button>
            <button type="button" class="warn" data-command="rtl" data-confirm="Command Return to Launch now?"><span>RTL Now</span></button>
            <button type="button" class="danger" id="shutdownBtn" data-command="shutdown" data-confirm="Only after landing and disarming. Shutdown and save logs?">Shutdown / Save Logs</button>
            <button type="button" class="danger" id="killSitlBtn" data-command="kill_sitl" data-confirm="Stop the SITL tmux session and related processes?">Kill SITL</button>
          </div>
          <div class="commandStatus" id="commandStatus">Ready</div>
          <div class="note">Use RC mode switch/manual control for immediate real-flight recovery if the vehicle behaves unexpectedly.</div>
        </section>
        <section style="margin-top:14px">
          <h2>Events</h2>
          <div class="eventLog" id="events"></div>
        </section>
      </div>
    </main>
  </div>
  <script>
    let state = {};

    function cls(el, value, good) {
      el.className = 'value ' + (good === true ? 'ok' : good === false ? 'bad' : 'warn');
      el.textContent = value;
    }

    async function api(path, options) {
      const res = await fetch(path, options);
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || 'Request failed');
      return data;
    }

    async function sendCommand(name) {
      const label = name.toUpperCase();
      const status = document.getElementById('commandStatus');
      status.textContent = 'Sending ' + label + '...';
      try {
        const result = await api('/api/command/' + name, {method: 'POST', cache: 'no-store'});
        status.textContent = 'Sent ' + (result.command || label) + ' at ' + new Date().toLocaleTimeString();
        await refresh();
        return result;
      }
      catch (err) {
        status.textContent = label + ' failed: ' + err.message;
        alert(err.message);
        throw err;
      }
    }

    async function confirmCommand(name, text) {
      if (!confirm(text)) return;
      await sendCommand(name);
    }

    async function selectMission() {
      const select = document.getElementById('missions');
      const mission = select.value;
      if (!mission) return;
      try {
        await api('/api/mission/select', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({path: mission})
        });
        await loadMissionPreview(mission);
        await refresh();
      } catch (err) {
        alert(err.message);
      }
    }

    async function loadMissions() {
      const data = await api('/api/missions');
      const select = document.getElementById('missions');
      const current = select.value;
      select.innerHTML = '';
      data.missions.forEach(m => {
        const opt = document.createElement('option');
        opt.value = m.path;
        opt.textContent = m.name;
        select.appendChild(opt);
      });
      if (current) select.value = current;
      document.getElementById('folder').textContent = data.mission_dir;
      if (select.value) loadMissionPreview(select.value);
    }

    async function loadMissionPreview(path) {
      const data = await api('/api/mission?path=' + encodeURIComponent(path));
      const body = document.getElementById('preview');
      body.innerHTML = '';
      data.rows.forEach(r => {
        const tr = document.createElement('tr');
        ['t','type','mode','profile','x','y','z','heading_deg'].forEach(k => {
          const td = document.createElement('td');
          td.textContent = r[k] ?? '';
          tr.appendChild(td);
        });
        body.appendChild(tr);
      });
    }

    function renderStatus(s) {
      const ddsOk = s.dds_age_s !== null && s.dds_age_s < 2.0;
      cls(document.getElementById('dds'), ddsOk ? 'Live' : 'No Data', ddsOk);
      cls(document.getElementById('mode'), s.nav_state_name || 'Unknown', s.nav_state === 14 ? true : null);
      cls(document.getElementById('arming'), s.armed ? 'Armed' : 'Disarmed', s.armed ? true : null);
      cls(document.getElementById('offboard'), s.offboard ? 'Enabled' : 'Disabled', s.offboard ? true : null);
      cls(document.getElementById('local'), s.local_position_valid ? 'Valid' : 'Invalid', s.local_position_valid);
      document.getElementById('alt').textContent = s.altitude_m === null ? '--' : s.altitude_m.toFixed(2) + ' m';
      document.getElementById('speed').textContent = s.speed_ms === null ? '--' : s.speed_ms.toFixed(2) + ' m/s';
      document.getElementById('selectedMission').textContent = s.selected_mission_name || '--';
      document.getElementById('updated').textContent = 'Updated ' + new Date().toLocaleTimeString();
      const banner = document.getElementById('failsafeBanner');
      banner.innerHTML = s.failsafe ? '<div class="banner">PX4 failsafe is active</div>' : '';
      const takeoffBtn = document.getElementById('takeoffBtn');
      takeoffBtn.disabled = !(s.armed && s.offboard && s.local_position_valid);
      takeoffBtn.title = takeoffBtn.disabled
        ? 'Takeoff is enabled after PX4 is armed, Offboard is active, and local position is valid.'
        : 'Start the selected TSV mission.';
      document.getElementById('shutdownBtn').style.display = s.shutdown_available ? '' : 'none';
      document.getElementById('killSitlBtn').style.display = s.kill_sitl_available ? '' : 'none';
      const log = document.getElementById('events');
      log.innerHTML = '';
      (s.events || []).slice().reverse().forEach(e => {
        const div = document.createElement('div');
        div.textContent = e;
        log.appendChild(div);
      });
    }

    async function refresh() {
      try {
        const s = await api('/api/status');
        state = s;
        renderStatus(s);
      } catch (err) {
        document.getElementById('updated').textContent = err.message;
      }
    }

    document.querySelectorAll('[data-command]').forEach(button => {
      button.addEventListener('click', async () => {
        const name = button.dataset.command;
        const text = button.dataset.confirm;
        if (text) {
          await confirmCommand(name, text);
        } else {
          await sendCommand(name);
        }
      });
    });
    document.getElementById('selectMissionBtn').addEventListener('click', selectMission);

    loadMissions().then(refresh);
    setInterval(refresh, 1000);
    setInterval(loadMissions, 8000);
  </script>
</body>
</html>
"""


class MissionControlWeb(Node):
    def __init__(self, mission_dir, host, port, shutdown_command='', kill_sitl_command=''):
        super().__init__('mission_control_web')
        self.mission_dir = Path(mission_dir).expanduser().resolve()
        self.host = host
        self.port = int(port)
        self.shutdown_command = shutdown_command
        self.kill_sitl_command = kill_sitl_command
        self.status = None
        self.control = None
        self.local_position = None
        self.events = []
        self.selected_mission = ''
        self.last_web_command = ''
        self.command_queue = deque()
        self.command_lock = threading.Lock()

        self.control_pub = self.create_publisher(String, '/mission/control', 10)
        self.command_timer = self.create_timer(0.1, self.publish_queued_commands)
        self.create_subscription(String, '/mission_executor/event', self.event_cb, 10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_cb, px4_qos)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_cb, px4_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_cb, px4_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_cb, px4_qos)

        self.app = Flask(__name__)
        self.configure_routes()

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def status_cb(self, msg):
        self.status = (msg, self.now_s())

    def control_cb(self, msg):
        self.control = (msg, self.now_s())

    def local_position_cb(self, msg):
        self.local_position = (msg, self.now_s())

    def event_cb(self, msg):
        self.events.append(msg.data)
        self.events = self.events[-80:]
        if msg.data.startswith('MISSION_SELECTED|'):
            self.selected_mission = msg.data.split('|', 1)[1]

    def enqueue_command(self, command, value='', repeats=3):
        payload = command if not value else f'{command}|{value}'
        with self.command_lock:
            for _ in range(max(1, repeats)):
                self.command_queue.append(payload)
        self.last_web_command = payload
        self.get_logger().warn(f'Web command queued: {payload}')
        self.events.append(f'WEB_COMMAND|{payload}')
        self.events = self.events[-80:]
        self.publish_control_payload(payload)

    def publish_control_payload(self, payload):
        msg = String()
        msg.data = payload
        self.get_logger().warn(f'Web command publish: {payload}')
        self.control_pub.publish(msg)

    def publish_queued_commands(self):
        payload = None
        with self.command_lock:
            if self.command_queue:
                payload = self.command_queue.popleft()
        if payload is None:
            return
        self.publish_control_payload(payload)

    def run_shell_command(self, command):
        if not command:
            raise RuntimeError('Command is not configured')
        subprocess.Popen(
            command,
            shell=True,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def mission_rows(self, path):
        candidate = Path(path).expanduser().resolve()
        if self.mission_dir not in candidate.parents and candidate != self.mission_dir:
            raise ValueError('Mission path is outside the configured mission folder')
        rows = []
        with candidate.open(newline='') as stream:
            reader = csv.DictReader(stream, delimiter='\t')
            for row in reader:
                rows.append(row)
        return rows

    def status_payload(self):
        now = self.now_s()
        status_msg, status_t = self.status if self.status else (None, None)
        control_msg, control_t = self.control if self.control else (None, None)
        local_msg, local_t = self.local_position if self.local_position else (None, None)
        newest = max([t for t in (status_t, control_t, local_t) if t is not None], default=None)

        armed = bool(getattr(control_msg, 'flag_armed', False))
        offboard = bool(getattr(control_msg, 'flag_control_offboard_enabled', False))
        if status_msg is not None:
            armed = armed or int(getattr(status_msg, 'arming_state', 0)) == 2
            offboard = offboard or int(getattr(status_msg, 'nav_state', -1)) == 14

        altitude = None
        speed = None
        local_valid = False
        if local_msg is not None:
            local_valid = bool(getattr(local_msg, 'xy_valid', False)) and bool(getattr(local_msg, 'z_valid', False))
            altitude = -float(local_msg.z)
            speed = math.sqrt(float(local_msg.vx) ** 2 + float(local_msg.vy) ** 2 + float(local_msg.vz) ** 2)

        nav_state = int(getattr(status_msg, 'nav_state', -1)) if status_msg is not None else None
        arming_state = int(getattr(status_msg, 'arming_state', -1)) if status_msg is not None else None
        selected_name = Path(self.selected_mission).name if self.selected_mission else ''

        return {
            'dds_age_s': None if newest is None else now - newest,
            'armed': armed,
            'offboard': offboard,
            'nav_state': nav_state,
            'nav_state_name': NAV_STATES.get(nav_state, str(nav_state) if nav_state is not None else ''),
            'arming_state': arming_state,
            'arming_state_name': ARMING_STATES.get(arming_state, str(arming_state) if arming_state is not None else ''),
            'failsafe': bool(getattr(status_msg, 'failsafe', False)),
            'local_position_valid': local_valid,
            'altitude_m': altitude,
            'speed_ms': speed,
            'selected_mission': self.selected_mission,
            'selected_mission_name': selected_name,
            'mission_dir': str(self.mission_dir),
            'shutdown_available': bool(self.shutdown_command),
            'kill_sitl_available': bool(self.kill_sitl_command),
            'events': list(self.events),
            'last_web_command': self.last_web_command,
        }

    def configure_routes(self):
        node = self

        @self.app.after_request
        def add_no_cache_headers(response):
            response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
            response.headers['Pragma'] = 'no-cache'
            response.headers['Expires'] = '0'
            return response

        @self.app.get('/')
        def index():
            return Response(INDEX_HTML, mimetype='text/html')

        @self.app.get('/api/status')
        def api_status():
            return jsonify(node.status_payload())

        @self.app.get('/api/missions')
        def api_missions():
            missions = [
                {'name': p.name, 'path': str(p)}
                for p in sorted(node.mission_dir.glob('*.tsv'))
            ]
            return jsonify({'mission_dir': str(node.mission_dir), 'missions': missions})

        @self.app.get('/api/mission')
        def api_mission():
            try:
                path = request.args.get('path', '')
                return jsonify({'rows': node.mission_rows(path)})
            except Exception as exc:
                return jsonify({'error': str(exc)}), 400

        @self.app.post('/api/mission/select')
        def api_select_mission():
            try:
                data = request.get_json(force=True) or {}
                path = data.get('path', '')
                node.mission_rows(path)
                node.selected_mission = str(Path(path).expanduser().resolve())
                node.enqueue_command('SELECT_MISSION', node.selected_mission)
                return jsonify({'ok': True, 'command': f'SELECT_MISSION|{node.selected_mission}'})
            except Exception as exc:
                return jsonify({'error': str(exc)}), 400

        @self.app.post('/api/command/<command>')
        def api_command(command):
            command_map = {
                'offboard': 'OFFBOARD',
                'arm': 'ARM',
                'disarm': 'DISARM',
                'takeoff': 'TAKEOFF',
                'land': 'LAND',
                'rtl': 'RTL',
            }
            if command == 'shutdown':
                try:
                    node.run_shell_command(node.shutdown_command)
                    return jsonify({'ok': True})
                except Exception as exc:
                    return jsonify({'error': str(exc)}), 400
            if command == 'kill_sitl':
                try:
                    node.run_shell_command(node.kill_sitl_command)
                    return jsonify({'ok': True})
                except Exception as exc:
                    return jsonify({'error': str(exc)}), 400
            if command not in command_map:
                return jsonify({'error': 'Unknown command'}), 404
            payload = command_map[command]
            node.enqueue_command(payload)
            return jsonify({'ok': True, 'command': payload})

    def start_web(self):
        thread = threading.Thread(
            target=lambda: self.app.run(host=self.host, port=self.port, threaded=True, use_reloader=False),
            daemon=True,
        )
        thread.start()
        self.get_logger().warn(f'Mission web control available at http://{self.host}:{self.port}')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission-dir', default=os.environ.get('MISSION_DIR', str(Path.home() / 'ROS_PX4' / 'missions')))
    parser.add_argument('--host', default=os.environ.get('MISSION_WEB_HOST', '0.0.0.0'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('MISSION_WEB_PORT', '8080')))
    parser.add_argument('--shutdown-command', default=os.environ.get('SHUTDOWN_COMMAND', ''))
    parser.add_argument('--kill-sitl-command', default=os.environ.get('KILL_SITL_COMMAND', ''))
    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = MissionControlWeb(
        parsed_args.mission_dir,
        parsed_args.host,
        parsed_args.port,
        parsed_args.shutdown_command,
        parsed_args.kill_sitl_command,
    )
    node.start_web()
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
