#!/usr/bin/env python3
"""Download a completed PX4 ULog through mavlink-router without QGC."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys
import time


def receive(link, kinds, system, component, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = link.recv_match(type=kinds, blocking=True,
                              timeout=max(0, deadline - time.monotonic()))
        if msg is None:
            break
        if msg.get_srcSystem() == system and msg.get_srcComponent() == component:
            if msg.get_type() == 'HEARTBEAT':
                if msg.base_mode & 128:
                    raise RuntimeError('Vehicle is armed; land and disarm before downloading.')
                if 'HEARTBEAT' not in kinds:
                    continue
            return msg
    return None


def list_logs(link, system, component):
    entries = {}
    for _ in range(3):
        link.mav.log_request_list_send(system, component, 0, 65535)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            msg = receive(link, ['LOG_ENTRY', 'HEARTBEAT'], system, component, 1)
            if msg is None or msg.get_type() != 'LOG_ENTRY':
                continue
            if msg.num_logs == 0:
                return []
            entries[msg.id] = msg
            if len(entries) == msg.num_logs:
                return list(entries.values())
    raise RuntimeError('Incomplete log list; check the MAVLink connection and retry.')


def download(link, system, component, entry, target):
    if entry.size <= 0:
        raise RuntimeError('Selected log is empty.')
    partial = target.with_suffix('.ulg.part')
    # Exclusive creation prevents overwriting an existing or interrupted download.
    if target.exists():
        raise RuntimeError(f'Output already exists: {target}')
    with partial.open('xb') as stream:
        offset = 0
        while offset < entry.size:
            count = min(90 * 128, entry.size - offset)
            data = bytearray(count)
            seen = bytearray(count)
            for _ in range(5):
                link.mav.log_request_data_send(system, component, entry.id, offset, count)
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    msg = receive(link, ['LOG_DATA', 'HEARTBEAT'], system, component,
                                  min(1, max(0, deadline - time.monotonic())))
                    if msg is None or msg.get_type() != 'LOG_DATA' or msg.id != entry.id:
                        continue
                    start = msg.ofs - offset
                    length = min(msg.count, len(msg.data))
                    if start < 0 or start + length > count or length == 0:
                        continue
                    data[start:start + length] = bytes(msg.data[:length])
                    seen[start:start + length] = b'\x01' * length
                    if all(seen):
                        break
                if all(seen):
                    break
            if not all(seen):
                raise RuntimeError(f'Incomplete data at offset {offset}; partial file: {partial}')
            stream.write(data)
            offset += count
            print(f'\rDownloaded {offset:,}/{entry.size:,} bytes', end='', flush=True)
        stream.flush()
        import os
        os.fsync(stream.fileno())
    with partial.open('rb') as stream:
        if stream.read(7) != b'ULog\x01\x12\x35':
            raise RuntimeError(f'Download is not a ULog; retained at {partial}')
    partial.rename(target)
    print(f'\nSaved: {target}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='udpin:127.0.0.1:14569',
                        help='Default: existing companion_aux MAVLink router endpoint')
    parser.add_argument('--list', action='store_true', help='Only list available logs')
    parser.add_argument('--log-id', type=int, help='Download this ID; default is latest')
    parser.add_argument('--output-dir', type=Path,
                        default=Path.home() / '.local/share/logloader/logs')
    args = parser.parse_args()
    from pymavlink import mavutil
    link = mavutil.mavlink_connection(args.connection, source_system=245, source_component=191)
    system = component = None
    try:
        deadline = time.monotonic() + 10
        heartbeat = None
        while time.monotonic() < deadline:
            msg = link.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                heartbeat = msg
                break
        if heartbeat is None:
            raise RuntimeError('No PX4 heartbeat. Check USB and mavlink-router.service.')
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError('Vehicle is armed; land and disarm before downloading.')
        system, component = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
        entries = list_logs(link, system, component)
        if not entries:
            raise RuntimeError('PX4 reports no stored logs.')
        for entry in sorted(entries, key=lambda e: e.id):
            stamp = datetime.fromtimestamp(entry.time_utc, timezone.utc).isoformat() if entry.time_utc else 'unknown time'
            print(f'ID {entry.id}: {entry.size:,} bytes, {stamp}')
        if args.list:
            return
        entry = (next((e for e in entries if e.id == args.log_id), None)
                 if args.log_id is not None else max(entries, key=lambda e: e.id))
        if entry is None:
            raise RuntimeError(f'Log ID {args.log_id} was not found.')
        args.output_dir = args.output_dir.expanduser()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        target = args.output_dir / f'px4_{entry.time_utc}_{entry.id}.ulg'
        download(link, system, component, entry, target)
    finally:
        if system is not None:
            link.mav.log_request_end_send(system, component)
        link.close()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ImportError) as exc:
        sys.exit(str(exc))
    except KeyboardInterrupt:
        sys.exit('Download interrupted; any .part file was retained.')
