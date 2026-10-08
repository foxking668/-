"""Replay existing forward recordings; check image advice and zero actuator output."""
import argparse
import collections
import csv
import hashlib
import json
import subprocess
import sys
from pathlib import Path

SESSIONS = (('manual_push_steer_0', 123), ('manual_push_steer_plus5', 123),
            ('manual_push_steer_minus5', 123), ('visual_observer_forward', 73))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--target-x', type=float, required=True)
    parser.add_argument('--target-heading', type=float, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    capture_root, output = args.capture_root.resolve(), args.output.resolve()
    if output.is_relative_to(capture_root):
        parser.error('Results must be outside the original capture tree')
    output.mkdir(parents=True, exist_ok=True)
    summary = {'sessions': [], 'target_x': args.target_x, 'target_heading_image': args.target_heading,
               'target_is_vehicle_calibration': False, 'actuator_writes': 0,
               'recognition_accuracy': None, 'metric_pose_validated': False}
    for name, count in SESSIONS:
        # The +5 folder also contains separate reverse-probe subfolders.
        clocks = list((capture_root / name).glob('manual_*/frames.csv'))
        if len(clocks) != 1:
            raise ValueError(f'{name}: expected one direct original session')
        session = clocks[0].parent
        files = (session / 'camera.avi', session / 'frames.csv', session / 'vehicle_config.ini')
        before = {p: sha(p) for p in files}
        clock = clocks[0]
        if name == 'visual_observer_forward':
            observer = session / 'visual_observer.csv'
            before[observer] = sha(observer)
            with observer.open(encoding='utf-8-sig', newline='') as stream:
                original_rows = list(csv.DictReader(stream))
            assert [int(row['frame_index']) for row in original_rows] == list(range(count))
            clock = output / (name + '_read_start_times.csv')
            with clock.open('w', encoding='utf-8', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(('frame_index', 'elapsed_s'))
                writer.writerows((row['frame_index'], row['frame_read_start_s']) for row in original_rows)
        target = output / (name + '.jsonl')
        subprocess.run([sys.executable, str(root / 'tools/replay.py'), str(session / 'camera.avi'),
                        '--config', str(session / 'vehicle_config.ini'), '--frame-times', str(clock),
                        '--stage', 'ToCones', '--observe-straight', str(args.target_x), str(args.target_heading),
                        '--output', str(target)], cwd=root, check=True)
        rows = [json.loads(line) for line in target.read_text().splitlines()]
        with clock.open(encoding='utf-8-sig', newline='') as stream:
            times = [float(row['elapsed_s']) for row in csv.DictReader(stream)]
        assert len(rows) == len(times) == count
        for index, (row, time) in enumerate(zip(rows, times)):
            assert row['time'] == time
            assert row['speed'] == row['steer'] == row['actuator_writes'] == 0
            assert row['reason'] == 'PERCEPTION_ONLY'
            if row['straight_suggestion_valid']:
                assert row['is_straight'] and row['line_confidence'] >= .65
                assert not row['line_ambiguous'] and not row['line_discontinuous']
                assert abs(row['straight_suggested_command']) <= 5
                if index and rows[index - 1]['straight_suggestion_valid']:
                    assert abs(row['straight_suggested_command'] - rows[index - 1]['straight_suggested_command']) <= (
                        3 * (time - rows[index - 1]['time']) + 2e-5)
            else:
                assert row['straight_suggested_command'] is None
        assert all(sha(path) == value for path, value in before.items())
        entry = {'name': name, 'frames': count,
                 'suggestion_frames': sum(bool(row['straight_suggestion_valid']) for row in rows),
                 'straight_frames': sum(bool(row['is_straight']) for row in rows),
                 'states': dict(collections.Counter(row['straight_state'] for row in rows)),
                 'input_files_unchanged': True}
        summary['sessions'].append(entry)
        print(json.dumps(entry, ensure_ascii=True), flush=True)
    summary['total_frames'] = sum(entry['frames'] for entry in summary['sessions'])
    sources = ('src/straight_follow.cpp', 'src/straight_follow.hpp', 'src/straight_image_features.hpp',
               'src/core.hpp', 'src/vision.cpp', 'src/replay_stream.cpp', 'tools/replay.py')
    summary['source_sha256'] = {name: sha(root / name) for name in sources}
    summary['executable_sha256'] = sha(root / 'build/vision_stream.exe')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print(f"PASS {summary['total_frames']} recorded forward frames; image advice only, original files unchanged")


if __name__ == '__main__':
    main()
