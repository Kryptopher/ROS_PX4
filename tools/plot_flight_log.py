#!/usr/bin/env python3

import argparse
import csv
import json
from pathlib import Path

from mission_logbook import (
    write_3d_plot,
    mission_row_markers,
    mission_start_time,
    takeoff_requested_time,
    write_x_time_plot,
    write_xy_plot,
    write_y_time_plot,
    write_z_plot,
)


def read_csv(path):
    if not path.is_file():
        return []
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def mission_file_for_run(run_dir):
    metadata_path = run_dir / "metadata.json"
    if metadata_path.is_file():
        with metadata_path.open() as stream:
            metadata = json.load(stream)
        mission_file = metadata.get("mission_file", "")
        if mission_file and Path(mission_file).is_file():
            return mission_file

    copied = sorted(run_dir.glob("*.tsv"))
    return str(copied[0]) if copied else ""


def write_plots(run_dir):
    samples = read_csv(run_dir / "flight.csv")
    events = read_csv(run_dir / "events.csv")
    commands = read_csv(run_dir / "commands.csv")
    mission_file = mission_file_for_run(run_dir)
    start_t = mission_start_time(events)
    takeoff_t = takeoff_requested_time(events)
    color_split_t = takeoff_t if takeoff_t is not None else start_t
    row_markers = mission_row_markers(mission_file, start_t)

    if not samples:
        raise RuntimeError(f"No flight.csv samples found in {run_dir}")

    written = []
    outputs = [
        (run_dir / "trajectory_3d.png", write_3d_plot, (mission_file, samples, commands, color_split_t, row_markers)),
        (run_dir / "xy.png", write_xy_plot, (mission_file, samples, commands, color_split_t, row_markers)),
        (run_dir / "height.png", write_z_plot, (samples, commands, color_split_t, row_markers)),
        (run_dir / "x_time.png", write_x_time_plot, (samples, commands, color_split_t, row_markers)),
        (run_dir / "y_time.png", write_y_time_plot, (samples, commands, color_split_t, row_markers)),
    ]
    for path, func, args in outputs:
        if func(path, *args):
            written.append(path)
    return written


def write_summary(run_dir):
    samples = read_csv(run_dir / "flight.csv")
    events = read_csv(run_dir / "events.csv")
    commands = read_csv(run_dir / "commands.csv")
    start_t = mission_start_time(events)
    takeoff_t = takeoff_requested_time(events)
    row_markers = mission_row_markers(mission_file_for_run(run_dir), start_t)
    metadata_path = run_dir / "metadata.json"
    metadata = {}
    if metadata_path.is_file():
        with metadata_path.open() as stream:
            metadata = json.load(stream)

    final = samples[-1] if samples else {}
    achieved_rate = 0.0
    if len(samples) > 1:
        try:
            duration = float(samples[-1]["t"]) - float(samples[0]["t"])
            if duration > 0:
                achieved_rate = (len(samples) - 1) / duration
        except (KeyError, TypeError, ValueError):
            pass

    summary_path = run_dir / "summary.txt"
    with summary_path.open("w") as stream:
        stream.write("Real Flight Log Summary\n")
        stream.write(f"Run directory: {run_dir}\n")
        stream.write(f"Mission file: {metadata.get('mission_file', mission_file_for_run(run_dir))}\n")
        stream.write(f"Flight CSV: {run_dir / 'flight.csv'}\n")
        stream.write(f"Events CSV: {run_dir / 'events.csv'}\n")
        stream.write(f"Commands CSV: {run_dir / 'commands.csv'}\n")
        stream.write(f"Metadata JSON: {metadata_path}\n")
        stream.write(f"Event count: {len(events)}\n")
        stream.write(f"Sample count: {len(samples)}\n")
        stream.write(f"Command sample count: {len(commands)}\n")
        stream.write(f"Achieved sample rate: {achieved_rate:.3f} Hz\n")
        stream.write(f"3D trajectory plot: {run_dir / 'trajectory_3d.png'}\n")
        stream.write(f"Top-down XY plot: {run_dir / 'xy.png'}\n")
        stream.write(f"Height plot: {run_dir / 'height.png'}\n")
        stream.write(f"X vs time plot: {run_dir / 'x_time.png'}\n")
        stream.write(f"Y vs time plot: {run_dir / 'y_time.png'}\n")
        stream.write(f"Takeoff requested time: {takeoff_t if takeoff_t is not None else 'not recorded'}\n")
        stream.write(f"Mission start time: {start_t if start_t is not None else 'not recorded'}\n")
        stream.write(f"TSV row markers: {len(row_markers)}\n")
        stream.write("\nFinal sample:\n")
        for key, value in final.items():
            stream.write(f"  {key}: {value}\n")
    return summary_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate SITL-style trajectory plots for a real flight log folder."
    )
    parser.add_argument("run_dir", help="Folder containing flight.csv, events.csv, and metadata.json")
    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser().resolve()
    written = write_plots(run_dir)
    written.append(write_summary(run_dir))
    for path in written:
        print(path)


if __name__ == "__main__":
    main()
