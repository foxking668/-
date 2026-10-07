"""Replay the five manual clips through shared C++ observation; verify timing/output, not metric pose."""
import argparse
import collections
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

SESSIONS = (
    ('manual_push_steer_0', 'forward'), ('manual_push_steer_plus5', 'forward'),
    ('manual_push_steer_minus5', 'forward'), ('manual_pull_steer_plus5', 'reverse'),
    ('manual_pull_steer_minus5', 'reverse'),
)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--analysis-root', type=Path, help='Directory containing existing manual_*_analysis_20261007 folders')
    parser.add_argument('--previous', type=Path, help='Previous five perception JSONL results for comparison')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    capture_root, output = args.capture_root.resolve(), args.output.resolve()
    analysis_root = (args.analysis_root or root / 'deploy').resolve()
    previous_root = (args.previous or root / 'deploy/visual_steering_fixed_20261007').resolve()
    if output.is_relative_to(capture_root):
        parser.error('Results must be outside the original capture tree')
    for name, _ in SESSIONS:
        originals = analysis_root / (name + '_analysis_20261007') / 'original_frames'
        if output.is_relative_to(originals):
            parser.error('Results must be outside original frame directories')
    output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    bundled_cv = root / 'tools/python_compiler'
    env['PYTHONPATH'] = str(bundled_cv) + os.pathsep + env.get('PYTHONPATH', '')
    summary = {'sessions': [], 'no_actuator_writes': True, 'metric_pose_validated': False,
               'suggestions_are_applied': False}
    for name, motion in SESSIONS:
        clocks = list((capture_root / name).rglob('frames.csv'))
        if len(clocks) != 1:
            raise ValueError(f'{name}: expected exactly one original session')
        session = clocks[0].parent
        originals = analysis_root / (name + '_analysis_20261007') / 'original_frames'
        before = {p: sha(p) for p in session.rglob('*') if p.is_file()}
        frame_hashes = {p: sha(p) for p in originals.glob('*.jpg')}
        target = output / (name + '.jsonl')
        subprocess.run([sys.executable, str(root / 'tools/replay.py'), str(originals),
                        '--config', str(session / 'vehicle_config.ini'), '--frame-times', str(clocks[0]),
                        '--stage', 'GarageReverse', '--observe-steering', motion,
                        '--output', str(target)], cwd=root, env=env, check=True)
        rows = [json.loads(s) for s in target.read_text(encoding='utf-8').splitlines()]
        with clocks[0].open(encoding='utf-8-sig', newline='') as stream:
            times = [float(r['elapsed_s']) for r in csv.DictReader(stream)]
        assert len(rows) == len(times) == len(frame_hashes) == 123
        for index, (row, time) in enumerate(zip(rows, times)):
            assert row['time'] == time and row['reason'] == 'PERCEPTION_ONLY'
            assert row['speed'] == row['steer'] == row['actuator_writes'] == 0
            if row['suggestion_valid']:
                assert not (row['line_ambiguous'] or row['line_discontinuous'])
                assert row['line_confidence'] >= .65 and abs(row['suggested_command']) <= 5
                if index and rows[index - 1]['suggestion_valid']:
                    change = abs(row['suggested_command'] - rows[index - 1]['suggested_command'])
                    assert change <= 3 * (time - rows[index - 1]['time']) + 2e-5
            else:
                assert all(row[k] is None for k in ('suggested_command', 'lateral_error_image', 'heading_error_image'))
        assert len({r['reference_id'] for r in rows if r['has_reference']}) <= 1
        assert all(sha(p) == value for p, value in {**before, **frame_hashes}.items())
        previous_file = previous_root / target.name
        previous = [json.loads(s) for s in previous_file.read_text(encoding='utf-8').splitlines()]
        entry = {'name': name, 'motion': motion, 'frames': len(rows),
                 'previous_score_ge_035': sum(r['line_confidence'] >= .35 for r in previous),
                 'score_ge_035': sum(r['line_confidence'] >= .35 for r in rows),
                 'suggestion_valid_frames': sum(bool(r['suggestion_valid']) for r in rows),
                 'states': dict(collections.Counter(r['observer_state'] for r in rows)),
                 'recognition_accuracy': None, 'input_files_unchanged': True}
        summary['sessions'].append(entry)
        print(name, entry['score_ge_035'], entry['suggestion_valid_frames'], flush=True)
    summary['total_frames'] = sum(r['frames'] for r in summary['sessions'])
    sources = ('src/core.hpp', 'src/vision.cpp', 'src/visual_observer.hpp', 'src/visual_observer.cpp',
               'src/replay_stream.cpp', 'tools/manual_capture.cpp', 'tools/manual_capture_options.hpp',
               'src/opencv_image.hpp', 'tools/replay.py', 'CMakeLists.txt')
    summary['source_sha256'] = {name: sha(root / name) for name in sources}
    summary['executable_sha256'] = sha(root / 'build/vision_stream.exe')
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('PASS 615 recorded frames; timing, quality rejection, command limits and original file hashes')

if __name__ == '__main__':
    main()
