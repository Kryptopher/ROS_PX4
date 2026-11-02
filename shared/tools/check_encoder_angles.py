#!/usr/bin/env python3
"""Check encoder angles for gaps, glitches, and drift: from a log, a growing log, or live ROS."""

import argparse
import bisect
import csv
import io
import math
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path


def read_csv(path):
    # Strip NUL padding left behind when logging is cut off uncleanly.
    text = path.read_bytes().replace(b"\x00", b"").decode()
    return list(csv.DictReader(io.StringIO(text)))


def longest_constant_run(t, e):
    best = cur = 0
    best_start = start = 0
    for i in range(1, len(e)):
        if e[i] == e[i - 1]:
            cur += 1
        else:
            cur = 0
            start = i
        if cur > best:
            best = cur
            best_start = start
    return best, t[best_start + best] - t[best_start], t[best_start], e[best_start]


class EncoderStats:
    def __init__(self, name):
        self.name = name
        self.start = self.last = None
        self.lo = math.inf
        self.hi = -math.inf
        self.jumps = 0
        self.max_step = 0.0
        self.run_start_t = None

    def add(self, t, value):
        if self.last is None:
            self.start = value
            self.run_start_t = t
        else:
            step = abs((value - self.last + 180) % 360 - 180)
            self.max_step = max(self.max_step, step)
            if step > 10:
                self.jumps += 1
            if value != self.last:
                self.run_start_t = t
        self.last = value
        self.lo = min(self.lo, value)
        self.hi = max(self.hi, value)


def latest_run_dir(logs_dir):
    runs = [p for p in logs_dir.iterdir() if (p / "encoder_angles.csv").is_file()]
    if not runs:
        raise SystemExit(f"No run folders with encoder_angles.csv in {logs_dir}")
    return max(runs, key=lambda p: (p / "encoder_angles.csv").stat().st_mtime)


class LiveMonitor:
    """Running stats over a stream of (t, enc1, enc2) samples, drawn in place."""

    def __init__(self, source, window, stale_after):
        self.source = source
        self.window = window
        self.stale_after = stale_after
        self.stats = {"enc1": EncoderStats("enc1"), "enc2": EncoderStats("enc2")}
        self.recent = deque()  # (t, enc1, enc2) within the last `window` seconds
        self.count = 0
        self.bad = 0
        self.first_t = self.last_t = None
        self.max_gap = 0.0
        self.gaps = 0
        self.last_data_wall = time.monotonic()

    def add(self, t, e1, e2):
        if math.isnan(e1) or math.isnan(e2):
            self.bad += 1
            return
        if self.last_t is not None:
            gap = t - self.last_t
            self.max_gap = max(self.max_gap, gap)
            if gap > 0.05:
                self.gaps += 1
        else:
            self.first_t = t
        self.last_t = t
        self.count += 1
        self.last_data_wall = time.monotonic()
        self.stats["enc1"].add(t, e1)
        self.stats["enc2"].add(t, e2)
        self.recent.append((t, e1, e2))
        while self.recent and self.recent[0][0] < t - self.window:
            self.recent.popleft()

    def draw(self):
        out = [f"Following {self.source}   (Ctrl+C to stop)", ""]
        if not self.count:
            out.append("waiting for data...")
        else:
            recent = self.recent
            span = recent[-1][0] - recent[0][0]
            rate = (len(recent) - 1) / span if span > 0 else 0.0
            idle = time.monotonic() - self.last_data_wall
            status = f"STALE: no new data for {idle:.1f}s" if idle > self.stale_after else "OK"
            out += [
                f"status   {status}",
                f"t        {self.last_t:.2f}s  (logged {self.last_t - self.first_t:.0f}s, {self.count} rows, {self.bad} bad)",
                f"rate     {rate:.1f} Hz over last {self.window:.0f}s",
                f"gaps     {self.gaps} over 50ms, largest {self.max_gap * 1000:.0f} ms",
                "",
                "         current     start       net      min       max    noise(last %.0fs)  max step  jumps>10  unchanged for" % self.window,
            ]
            for k, s in enumerate(self.stats.values(), start=1):
                vals = [r[k] for r in recent]
                out.append(
                    f"{s.name}  {s.last:9.3f} {s.start:9.3f} {s.last - s.start:+9.3f} {s.lo:9.3f} {s.hi:9.3f}"
                    f"   {max(vals) - min(vals):9.3f}        {s.max_step:7.3f}  {s.jumps:8d}  {self.last_t - s.run_start_t:8.1f}s"
                )
        # Overwrite in place (home + clear-to-end-of-line) to avoid flicker at high refresh rates.
        sys.stdout.write("\033[H" + "".join(line + "\033[K\n" for line in out) + "\033[J")
        sys.stdout.flush()


def follow_file(path, interval, window, stale_after):
    monitor = LiveMonitor(path, window, stale_after)
    header = None
    pending = b""
    sys.stdout.write("\033[2J")
    with path.open("rb") as stream:
        while True:
            chunk = stream.read()
            # Stop at NUL padding; real data may later be written over it.
            nul = chunk.find(b"\x00")
            if nul >= 0:
                stream.seek(nul - len(chunk), io.SEEK_CUR)
                chunk = chunk[:nul]
            pending += chunk
            *lines, pending = pending.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                if header is None:
                    header = line.split(",")
                    continue
                row = dict(zip(header, line.split(",")))
                try:
                    monitor.add(float(row["t"]), float(row["encoder1_angle_deg"]), float(row["encoder2_angle_deg"]))
                except (KeyError, ValueError):
                    monitor.bad += 1
            monitor.draw()
            time.sleep(interval)


def follow_topic(topic, interval, window, stale_after):
    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64MultiArray

    monitor = LiveMonitor(f"ROS topic {topic}", window, stale_after)
    start = time.monotonic()

    def on_angles(msg):
        if len(msg.data) < 2:
            monitor.bad += 1
            return
        monitor.add(time.monotonic() - start, float(msg.data[0]), float(msg.data[1]))

    rclpy.init()
    node = rclpy.create_node("encoder_angle_monitor")
    # Match the publisher in mt6835_encoder.py.
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
    node.create_subscription(Float64MultiArray, topic, on_angles, qos)
    node.create_timer(interval, monitor.draw)
    sys.stdout.write("\033[2J")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", nargs="?",
                        help="Folder containing encoder_angles.csv (default: newest run in ~/logs)")
    parser.add_argument("--bucket", type=float, default=60.0, help="Window size in seconds for the drift table")
    parser.add_argument("--follow", "-f", action="store_true", help="Keep reading the file as it grows and show live stats")
    parser.add_argument("--live", "-l", nargs="?", const="/mt6835/angles", metavar="TOPIC",
                        help="Show live stats straight from the ROS topic (default /mt6835/angles)")
    parser.add_argument("--interval", type=float, default=0.01, help="Screen refresh period in seconds for --follow/--live")
    parser.add_argument("--window", type=float, default=10.0, help="Seconds of recent data used for live rate/noise")
    parser.add_argument("--stale", type=float, default=1.0, help="Flag STALE after this many seconds without new rows")
    args = parser.parse_args()
    if args.live:
        try:
            follow_topic(args.live, args.interval, args.window, args.stale)
        except KeyboardInterrupt:
            print()
        return

    run_dir = Path(args.run_dir) if args.run_dir else latest_run_dir(Path.home() / "logs")

    if args.follow:
        try:
            follow_file(run_dir / "encoder_angles.csv", args.interval, args.window, args.stale)
        except KeyboardInterrupt:
            print()
        return

    rows = read_csv(run_dir / "encoder_angles.csv")
    print(f"rows {len(rows)}  cols {list(rows[0].keys())}")
    bad = [i for i, x in enumerate(rows) if any(v in ("", None, "nan") for v in x.values())]
    print(f"bad/missing rows {len(bad)} {bad[:5]}")
    rows = [x for i, x in enumerate(rows) if i not in set(bad)]

    t = [float(x["t"]) for x in rows]
    enc = {
        "enc1": [float(x["encoder1_angle_deg"]) for x in rows],
        "enc2": [float(x["encoder2_angle_deg"]) for x in rows],
    }

    # Timing
    dt = sorted(b - a for a, b in zip(t, t[1:]))
    n = len(dt)
    print(f"\nTIMING  t {t[0]:.3f} -> {t[-1]:.3f}  duration {t[-1] - t[0]:.1f}s")
    print(f"  dt median {dt[n // 2]:.4f}  p1 {dt[n // 100]:.4f}  p99 {dt[-(n // 100) - 1]:.4f}  max {dt[-1]:.4f}")
    raw_dt = [b - a for a, b in zip(t, t[1:])]
    print(f"  non-monotonic {sum(d <= 0 for d in raw_dt)}  gaps>50ms {sum(d > 0.05 for d in raw_dt)}  gaps>0.5s {sum(d > 0.5 for d in raw_dt)}")
    i = max(range(len(raw_dt)), key=raw_dt.__getitem__)
    print(f"  largest gap {raw_dt[i]:.3f}s at t={t[i]:.2f}")
    if "t_wall" in rows[0]:
        wall = [datetime.fromisoformat(x["t_wall"]).timestamp() for x in rows]
        off = [w - s - (wall[0] - t[0]) for w, s in zip(wall, t)]
        print(f"  wall-clock vs t drift: {min(off) * 1000:.1f} .. {max(off) * 1000:.1f} ms")

    # Per-encoder signal quality
    for name, e in enc.items():
        de = [(b - a + 180) % 360 - 180 for a, b in zip(e, e[1:])]
        changes = sum(d != 0 for d in de)
        steps = sorted(abs(d) for d in de if d != 0)
        print(f"\n{name.upper()}  min {min(e):.3f}  max {max(e):.3f}  unique {len(set(e))}  "
              f"changes {changes} ({100 * changes / len(de):.1f}%)")
        if steps:
            m = len(steps)
            print(f"  step |d| min {steps[0]:.4f}  median {steps[m // 2]:.4f}  "
                  f"p99 {steps[-(m // 100) - 1]:.3f}  max {steps[-1]:.3f}")
        jumps = [(round(t[k + 1], 3), e[k], e[k + 1]) for k, d in enumerate(de) if abs(d) > 10]
        print(f"  jumps>10deg {len(jumps)}  >45deg {sum(abs(d) > 45 for d in de)}  {jumps[:10]}")
        run, secs, start, val = longest_constant_run(t, e)
        print(f"  longest constant run {run} samples ({secs:.1f}s) from t={start:.1f} at {val:.3f}")
        print(f"  start {e[0]:.3f}  end {e[-1]:.3f}  net change {e[-1] - e[0]:+.3f} deg")

    # Drift vs vehicle motion
    flight_path = run_dir / "flight.csv"
    ft, speed = [], []
    if flight_path.is_file():
        for x in read_csv(flight_path):
            try:
                ft.append(float(x["t"]))
                speed.append(math.hypot(float(x["vx"]), float(x["vy"])))
            except (KeyError, ValueError):
                pass

    print(f"\nDRIFT ({args.bucket:.0f}s windows)")
    print("  window          enc1 mean  range    enc2 mean  range    veh speed max")
    b0 = math.floor(t[0] / args.bucket) * args.bucket
    while b0 < t[-1]:
        a, c = bisect.bisect_left(t, b0), bisect.bisect_left(t, b0 + args.bucket)
        if c > a:
            s1, s2 = enc["enc1"][a:c], enc["enc2"][a:c]
            line = (f"  {b0:6.0f}-{b0 + args.bucket:5.0f}s  {sum(s1) / len(s1):9.2f} {max(s1) - min(s1):6.2f}"
                    f"   {sum(s2) / len(s2):9.2f} {max(s2) - min(s2):6.2f}")
            fa, fc = bisect.bisect_left(ft, b0), bisect.bisect_left(ft, b0 + args.bucket)
            if fc > fa:
                line += f"      {max(speed[fa:fc]):.2f}"
            print(line)
        b0 += args.bucket


if __name__ == "__main__":
    main()
