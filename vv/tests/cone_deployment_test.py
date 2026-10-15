"""Run with Python 3; optional --shell specifies a POSIX shell on Windows."""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--shell', default=shutil.which('sh') or shutil.which('bash'))
args = parser.parse_args()
if not args.shell:
    raise SystemExit('A POSIX shell is required')
root = Path(__file__).resolve().parents[1]
source = root / 'apply_cone_config.sh'
with tempfile.TemporaryDirectory(prefix='vv-cone-deploy-') as folder:
    work = Path(folder)
    helper = work / source.name
    shutil.copyfile(source, helper)
    parameters = work / 'field params.txt'
    original = ('# Keep all field calibration\ncenter_pratio=0.42\nmotor_aim_speed=9\n'
                'new_track_ring_enable=1\nnew_track_ring_side_enable=1\n'
                'new_track_cone_sequence_enable=0\nnew_track_cone_sequence_enable=0\n'
                ' new_track_blue_cone_enable = 1 # legacy\n'
                'new_track_crossing_enable=1\nnew_track_config_complete=1\n')
    parameters.write_text(original, encoding='utf-8')
    def run(path, success=True):
        result = subprocess.run([args.shell, str(helper), str(path)], cwd=work,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        assert (result.returncode == 0) == success, result.stdout.decode(errors='replace')
    run(parameters)
    migrated = parameters.read_text(encoding='utf-8')
    for line in ['center_pratio=0.42','motor_aim_speed=9','new_track_ring_enable=1',
                 'new_track_crossing_enable=1','new_track_config_complete=1']:
        assert line in migrated
    expected = {'new_track_cone_sequence_enable':'1',
                'new_track_blue_cone_enable':'0',
                'new_track_ring_side_enable':'0','new_track_ring_entry_cue_enable':'0'}
    for key,value in expected.items():
        assert migrated.splitlines().count(f'{key}={value}') == 1
    backups = list(work.glob('field params.txt.before_cone_window.*'))
    assert len(backups) == 1 and backups[0].read_text(encoding='utf-8') == original
    run(parameters)
    assert parameters.read_text(encoding='utf-8') == migrated
    assert len(list(work.glob('field params.txt.before_cone_window.*'))) == 2
    run(work / 'missing.txt', success=False)
    assert not (work / 'missing.txt').exists()
    (work / 'image_params.reference.txt').write_text('center_pratio=0.42\n', encoding='utf-8')
    run(work / 'new.txt')
    assert 'center_pratio=0.42' in (work / 'new.txt').read_text(encoding='utf-8')
    directory = work / 'directory'
    directory.mkdir()
    run(directory, success=False)
    assert not list(directory.iterdir())
print('PASS cone deployment: preserve field settings, duplicate removal, backup, idempotence, missing reference, directory rejection')
