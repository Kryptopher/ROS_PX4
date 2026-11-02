#!/usr/bin/python3 -s
# Preview mission setpoints. Usage: plot_missions.py [mission.tsv ...]  (default: square + straight line)
import csv, math, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Mirrors mission_executor_dds.get_setpoint_row / interpolate_position_row.
def load(path):
    rows = []
    for r in csv.DictReader(open(path), delimiter='\t'):
        h = r.get('heading_deg', '')
        rows.append({k: (float(r[k]) if k in 'txyz' else r[k]) for k in ('t','type','mode','profile','x','y','z')}
                    | {'heading_deg': float(h) % 360 if h else math.nan})
    return sorted(rows, key=lambda r: r['t'])

def setpoint(rows, t):
    cur = rows[0]
    for r in rows:
        if t >= r['t']: cur = r
        else: break
    if cur['profile'] != 'linear' or cur['mode'] != 'pos': return cur
    i = rows.index(cur)
    if i + 1 >= len(rows): return cur
    nxt = rows[i+1]
    if nxt['type'] == 'end' or nxt['mode'] != 'pos' or nxt['t'] <= cur['t']: return cur
    b = min(1, max(0, (t - cur['t']) / (nxt['t'] - cur['t'])))
    out = dict(nxt)
    for k in 'xyz': out[k] = cur[k] + b*(nxt[k]-cur[k])
    s, e = math.radians(cur['heading_deg']), math.radians(nxt['heading_deg'])
    d = math.atan2(math.sin(e-s), math.cos(e-s))
    out['heading_deg'] = math.degrees(s + b*d) % 360
    return out

def plot(path, out):
    rows = load(path)
    t_end = rows[-1]['t']
    ts = np.arange(0, t_end, 0.02)
    sp = [setpoint(rows, t) for t in ts]
    x = np.array([s['x'] for s in sp]); y = np.array([s['y'] for s in sp]); z = np.array([s['z'] for s in sp])
    h = np.array([s['heading_deg'] for s in sp])
    hr = np.degrees(np.unwrap(np.radians(h)))
    yawrate = np.gradient(hr, ts)
    speed = np.hypot(np.hypot(np.gradient(x, ts), np.gradient(y, ts)), np.gradient(z, ts))

    fig = plt.figure(figsize=(15, 9))
    fig.suptitle(f'{path.split("/")[-1]} — commanded setpoints (x=North, y=East, z=Up, relative to takeoff)')
    ax = fig.add_subplot(2, 3, 1, projection='3d')
    ax.plot(y, x, z); ax.set_xlabel('y East [m]'); ax.set_ylabel('x North [m]'); ax.set_zlabel('z Up [m]')
    ax.set_title('3D path')
    ax = fig.add_subplot(2, 3, 2)
    ax.plot(y, x, lw=2)
    for r in rows:
        if r['type'] == 'end': continue
        ax.plot(r['y'], r['x'], 'ko', ms=4)
        ax.annotate(f"t={r['t']:g}", (r['y'], r['x']), textcoords='offset points', xytext=(5, 5), fontsize=7)
    for i in range(0, len(ts), 50):  # heading arrow every 1 s
        a = math.radians(h[i])
        ax.arrow(y[i], x[i], 0.8*math.sin(a), 0.8*math.cos(a), head_width=0.25, color='tab:red', alpha=0.6)
    ax.set_xlabel('y East [m]'); ax.set_ylabel('x North [m]'); ax.set_aspect('equal'); ax.grid(alpha=.3)
    ax.set_title('Top-down (red arrows = nose heading, every 1 s)')
    ax.margins(0.15)
    ax = fig.add_subplot(2, 3, 3)
    ax.plot(ts, x, label='x N'); ax.plot(ts, y, label='y E'); ax.plot(ts, z, label='z Up')
    ax.set_xlabel('t [s]'); ax.set_ylabel('[m]'); ax.legend(); ax.grid(alpha=.3); ax.set_title('Position vs time')
    ax = fig.add_subplot(2, 3, 4)
    ax.plot(ts, h); ax.set_xlabel('t [s]'); ax.set_ylabel('heading [deg]'); ax.grid(alpha=.3); ax.set_title('Heading setpoint')
    ax = fig.add_subplot(2, 3, 5)
    ax.plot(ts, yawrate); ax.set_xlabel('t [s]'); ax.set_ylabel('[deg/s]'); ax.grid(alpha=.3)
    ax.set_title(f'Yaw rate (peak {np.abs(yawrate).max():.1f} deg/s)')
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(ts, speed); ax.set_xlabel('t [s]'); ax.set_ylabel('[m/s]'); ax.grid(alpha=.3)
    ax.set_title(f'Commanded speed (peak {speed.max():.2f} m/s)')
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(out, 'peak yaw rate', round(np.abs(yawrate).max(), 1), 'peak speed', round(speed.max(), 2))

MISSIONS = '/home/scs/ROS_PX4/shared/missions/'
paths = sys.argv[1:] or [MISSIONS + 'square.tsv', MISSIONS + 'straight_line_mission.tsv']
for p in paths:
    plot(p, '/home/scs/logs/mission_previews/' + p.split('/')[-1].replace('.tsv', '_preview.png'))
