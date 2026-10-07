"""Exercise the one-command launcher with a fake recorder, never board devices."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
launcher = root / 'deploy/run_auto_reverse_probe.sh'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shell', default='sh')
    args = parser.parse_args()
    assert b'\r' not in launcher.read_bytes(), 'Board shell script must retain LF endings'
    subprocess.run([args.shell, '-n', str(launcher)], check=True)
    build = (root / 'build').resolve()
    build.mkdir(exist_ok=True)
    scratch = Path(tempfile.mkdtemp(prefix='auto-probe-launcher-', dir=build)).resolve()
    if not scratch.is_relative_to(build):
        raise RuntimeError('Unexpected test directory outside build')
    try:
        script = scratch / launcher.name
        shutil.copyfile(launcher, script)
        recorder = scratch / 'manual_capture_auto_probe_20261007'
        invoked = scratch / 'invoked.txt'
        env = dict(os.environ, LD_LIBRARY_PATH='retained-library-path')
        def run(extra=()):
            return subprocess.run([args.shell, './'+script.name, *extra], cwd=scratch, env=env,
                                  capture_output=True, text=True, encoding='utf-8')
        def fake(version, automatic=True, code=0):
            flag = '--auto-probe reverse' if automatic else 'no-auto-mode'
            recorder.write_text('#!/bin/sh\nif [ "$1" = --help ]; then\n'
                                f"printf '%s\\n' 'manual_capture version={version}' '{flag}'\nexit 0\nfi\n"
                                "printf '%s\\n' run >> invoked.txt\n"
                                "printf '%s\\n' \"$@\" > args.txt\n"
                                "printf '%s' \"$LD_LIBRARY_PATH\" > library.txt\n"
                                f'exit {code}\n', encoding='utf-8', newline='\n')
            recorder.chmod(0o755)
        missing = run()
        assert missing.returncode != 0 and 'Missing executable' in missing.stderr and not invoked.exists()
        fake('2026-10-07.3')
        old = run()
        assert old.returncode != 0 and 'Wrong recorder version' in old.stderr and not invoked.exists()
        fake('2026-10-07.4', automatic=False)
        unsupported = run()
        assert unsupported.returncode != 0 and 'does not support' in unsupported.stderr and not invoked.exists()
        fake('2026-10-07.4')
        rejected = run(('unexpected',))
        assert rejected.returncode != 0 and 'Usage:' in rejected.stderr and not invoked.exists()
        success = run()
        assert success.returncode == 0 and 'AUTO_PULL_READY' in success.stdout
        passed = (scratch / 'args.txt').read_text().splitlines()
        assert passed == ['--config', 'manual_capture.ini', '--hardware-config', 'config/calibration_hardware.ini',
                          '--vehicle-config', 'config/calibration_vehicle.ini', '--allow-partial', '--duration', '45',
                          '--auto-probe', 'reverse', '--output', 'captures/auto_reverse_probe_plus2']
        assert (scratch / 'library.txt').read_text() == '/home/root/opencv-4.11-loongarch/install/lib:retained-library-path'
        fake('2026-10-07.4', code=2)
        assert run().returncode == 2, 'Recorder exit status must propagate without rerunning hardware'
        assert invoked.read_text().splitlines() == ['run', 'run'], 'Each launch invokes the recorder once'
        print('PASS launcher syntax, missing/old/unsupported recorder rejection, exact auto-only arguments, library path and exit propagation')
    finally:
        # scratch was resolved and checked under the workspace build directory.
        shutil.rmtree(scratch)

if __name__ == '__main__':
    main()
