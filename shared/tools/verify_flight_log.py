#!/usr/bin/env python3
"""PASS/FAIL check that a run recorded angles during its mission and saved cleanly."""

import csv
import json
import sys
from pathlib import Path

END_EVENTS = {'MISSION_COMPLETE_HOLDING', 'MISSION_ABORTED', 'SAFETY_ABORT', 'PILOT_MODE_TAKEOVER',
              'SAVE_AND_HANDOVER_REQUESTED'}


def rows(path):
    with path.open(errors='replace') as stream:
        yield from csv.DictReader(line.replace('\0', '') for line in stream)


def main():
    if len(sys.argv) != 2:
        sys.exit('usage: verify_flight_log.py <run directory>')
    run = Path(sys.argv[1]).expanduser()
    failed = 0

    def check(ok, label, detail=''):
        nonlocal failed
        failed += not ok
        print(f'{"PASS" if ok else "FAIL"}  {label}{f"  ({detail})" if detail else ""}')

    print(f'Run: {run}\n\n-- Saved --')
    metadata = {}
    if (run / 'metadata.json').is_file():
        metadata = json.loads((run / 'metadata.json').read_text())
    check(metadata.get('status') == 'complete' or 'flight_rows' in metadata,
          'logger finished and wrote metadata', f'status={metadata.get("status")}')
    check((run / 'summary.txt').is_file(), 'summary.txt written')
    check(any(run.glob('*.png')), 'plots written')
    with (run / 'flight.csv').open('rb') as stream:
        stream.seek(-min(4096, (run / 'flight.csv').stat().st_size), 2)
        check(b'\0' not in stream.read(), 'flight.csv ends cleanly (no power-cut padding)')

    print('\n-- Mission --')
    events = list(rows(run / 'events.csv'))
    names = [e['event_type'] for e in events]
    start = next((float(e['t']) for e in events if e['event_type'] == 'MISSION_STARTED'), None)
    check(start is not None, 'mission started', f't={start}')
    end = None
    if start is not None:
        end = next((float(e['t']) for e in events
                    if e['event_type'] in END_EVENTS and float(e['t']) >= start), None)
    check('MISSION_COMPLETE_HOLDING' in names, 'mission reached its end row')

    print('\n-- Angles during the mission --')
    if start is None or end is None:
        check(False, 'angles recorded during mission', 'no mission window')
    else:
        angles = [r for r in rows(run / 'encoder_angles.csv') if start <= float(r['t']) <= end]
        times = [float(r['t']) for r in angles]
        duration = end - start
        rate = len(angles) / duration if duration > 0 else 0.0
        check(bool(angles), 'angles recorded during mission', f'{len(angles)} rows in {duration:.1f} s')
        if angles:
            check(times[0] - start < 0.5, 'recording covers mission start', f'first row +{times[0] - start:.3f} s')
            check(end - times[-1] < 0.5, 'recording covers mission end', f'last row -{end - times[-1]:.3f} s')
            check(rate >= 20.0, 'sample rate', f'{rate:.1f} Hz')
            gap = max((b - a for a, b in zip(times, times[1:])), default=0.0)
            check(gap < 0.5, 'no long gaps', f'largest {gap:.3f} s')
            for column in ('encoder1_angle_deg', 'encoder2_angle_deg'):
                values = [float(r[column]) for r in angles]
                check(max(values) - min(values) > 0.5, f'{column} changes',
                      f'{min(values):.2f} to {max(values):.2f} deg; move the payload during a bench test')

    print('\n-- Handover --')
    last = None
    for last in rows(run / 'flight.csv'):
        pass
    if 'SAVE_AND_HANDOVER_REQUESTED' in names:
        check(bool({'MISSION_COMPLETE_POSITION', 'SAVE_AND_HANDOVER_STOPPING', 'PILOT_MODE_TAKEOVER'} & set(names)),
              'Shutdown waited for PX4 to leave Offboard')
    else:
        print('INFO  no operator Shutdown request in this run')
    if last is not None:
        print(f'INFO  last logged state: armed={last.get("armed")} offboard={last.get("offboard")} '
              f'nav_state={last.get("nav_state")!r} (2 = Position)')
        check(last.get('offboard') != '1', 'PX4 was out of Offboard when logging stopped')

    print(f'\nRESULT: {"PASS" if not failed else f"FAIL ({failed} checks)"}')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
