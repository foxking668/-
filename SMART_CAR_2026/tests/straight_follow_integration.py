"""Exercise the actual replay frontend, Vision, and forward straight controller."""
import csv
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

root = Path(__file__).resolve().parents[1]
temporary = root / 'build/straight-follow-fixtures'
temporary.mkdir(parents=True, exist_ok=True)
clock = temporary / 'frames.csv'
times = [.71 + index * .2 for index in range(9)]
with clock.open('w', encoding='utf-8', newline='') as stream:
    writer = csv.writer(stream)
    writer.writerow(('frame_index', 'elapsed_s'))
    writer.writerows(enumerate(times))


def frames(name, curve=False):
    folder = temporary / name
    folder.mkdir(exist_ok=True)
    image = np.full((240, 320, 3), (210, 90, 0), np.uint8)
    image[:, 48:272] = (240, 240, 240)
    for y in range(240):
        center = 208 if not curve else round(168 + 190 * (y / 240 - .65) ** 2)
        image[y, center - 4:center + 4] = 0
    for index in range(len(times)):
        ok, encoded = cv2.imencode('.png', image)
        assert ok
        encoded.tofile(folder / f'{index:03}.png')
    return folder


def run(source, target, output, *extra):
    command = [sys.executable, str(root / 'tools/replay.py'), str(source),
               '--frame-times', str(clock), '--stage', 'ToCones',
               '--observe-straight', *map(str, target), '--output', str(output), *extra]
    return subprocess.run(command, cwd=root, capture_output=True, text=True)


offset = frames('offset')
output = temporary / 'offset.jsonl'
completed = run(offset, (.5, 0), output)
assert completed.returncode == 0, completed.stderr
rows = [json.loads(line) for line in output.read_text().splitlines()]
assert [row['time'] for row in rows] == times
assert all(row['straight_target_x'] == .5 and row['straight_target_heading_image'] == 0 for row in rows)
assert all(row['speed'] == row['steer'] == row['actuator_writes'] == 0 for row in rows)
valid = [row for row in rows if row['straight_suggestion_valid']]
assert len(valid) >= 4
assert all(row['is_straight'] and not row['straight_aligned'] and
           row['straight_lateral_error_image'] > .14 and
           row['straight_suggested_command'] > 0 for row in valid)
for previous, current in zip(rows, rows[1:]):
    if current['straight_suggestion_valid'] and previous['straight_suggestion_valid']:
        assert abs(current['straight_suggested_command'] - previous['straight_suggested_command']) <= (
            3 * (current['time'] - previous['time']) + 2e-5)
assert all(row['straight_suggested_command'] is None for row in rows if not row['straight_suggestion_valid'])

completed = run(offset, (207.5 / 320, 0), output)
assert completed.returncode == 0, completed.stderr
rows = [json.loads(line) for line in output.read_text().splitlines()]
assert all(row['straight_aligned'] and row['straight_suggested_command'] == 0
           for row in rows if row['straight_suggestion_valid'])

completed = run(frames('curve', curve=True), (.5, 0), output)
assert completed.returncode == 0, completed.stderr
rows = [json.loads(line) for line in output.read_text().splitlines()]
assert any(row['straight_state'] == 'NOT_STRAIGHT' for row in rows)
assert not any(row['straight_suggestion_valid'] for row in rows)

for target in ((float('nan'), 0), (1.1, 0), (.1, -.2), (.5, float('inf'))):
    assert run(offset, target, output).returncode != 0, 'invalid target was accepted'
for extra in (('--stage', 'GarageReverse'), ('--observe-steering', 'forward')):
    assert run(offset, (.5, 0), output, *extra).returncode != 0, 'incompatible mode was accepted'
telemetry = temporary / 'telemetry.csv'
telemetry.write_text('time,distance,speed,yaw,yaw_valid,yaw_measured\n0,0,0,0,0,0\n')
assert run(offset, (.5, 0), output, '--telemetry', str(telemetry)).returncode != 0

# Native frontend also rejects malformed numeric suffixes; Python float parsing
# must not be the only guard against invalid direct invocations.
exe = root / 'build/vision_stream.exe'
for target in (('0.5junk', '0'), ('0.5', 'nan'), ('0.5', '0junk')):
    result = subprocess.run([str(exe), 'config/competition.ini', 'ToCones',
                             '--observe-straight', *target], cwd=root, input=b'',
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert result.returncode != 0
print('PASS real frontend/Vision/straight control: offset correction, explicit target, curves, timing, zero writes, invalid modes')
