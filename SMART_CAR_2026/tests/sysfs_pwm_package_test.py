"""Hardware-free tests for the dedicated sysfs/PWM handoff and launcher."""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from package_sysfs_pwm_parking import package


class Packaging(unittest.TestCase):
    def test_package_validates_all_payloads_without_executing_target(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            # Deliberately non-executable fixture; proves packaging never runs it.
            elf = bytearray(64)
            elf[:6] = b'\x7fELF\x02\x01'
            elf[18:20] = (258).to_bytes(2, 'little')
            program = path / 'fixture'
            program.write_bytes(elf + b'2026-10-16.1 --sysfs-pwm-control')
            archive, _ = package(program, path / 'package.zip')
            with zipfile.ZipFile(archive) as contents:
                self.assertIsNone(contents.testzip())
                self.assertEqual(len(contents.namelist()), 8)
                for line in contents.read('SYSFS_PWM_SHA256SUMS').decode().splitlines():
                    digest, name = line.split('  ', 1)
                    self.assertEqual(hashlib.sha256(contents.read(name)).hexdigest(), digest)
                self.assertNotIn('config/parking_tuning.new_car.ini', contents.namelist())
            elf[18:20] = (62).to_bytes(2, 'little')
            program.write_bytes(elf + b'2026-10-16.1 --sysfs-pwm-control')
            with self.assertRaisesRegex(ValueError, 'LoongArch'):
                package(program, path / 'wrong.zip')

    def test_launcher_preserves_user_tuning_and_old_profiles(self):
        shell = Path('C:/Program Files/Git/bin/sh.exe') if os.name == 'nt' else shutil.which('sh')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            for name in ('run_sysfs_pwm_parking.sh', 'sysfs_pwm_hardware.ini', 'sysfs_pwm_vehicle.ini',
                         'parking_tuning.sysfs_pwm.example.ini', 'manual_capture.sysfs_pwm.ini'):
                shutil.copyfile(ROOT / 'deploy' / name, path / name)
            (path / 'config').mkdir()
            snapshots = {
                'parking_tuning.new_car.ini': b'user stage1 nine seconds, do not overwrite\n',
                'new_car_hardware.ini': b'motor_backend=xx\n',
                'new_car_vehicle.ini': b'original xx profile\n',
            }
            for name, data in snapshots.items():
                (path / 'config' / name).write_bytes(data)
            program = path / 'parking_sysfs_pwm_20261016'
            program.write_text(
                '#!/bin/sh\n'
                'if [ "${1:-}" = --help ]; then\n'
                ' echo "sysfs_pwm_parking version=2026-10-16.1 --sysfs-pwm-control"\n'
                ' exit 0\nfi\n'
                'printf "%s\\n" "$@" > captured_args.txt\n', encoding='utf-8', newline='\n')
            program.chmod(0o755)
            run = lambda: subprocess.run([str(shell), './run_sysfs_pwm_parking.sh'], cwd=path,
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            result = run()
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
            for name, data in snapshots.items():
                self.assertEqual((path / 'config' / name).read_bytes(), data)
            args = (path / 'captured_args.txt').read_text()
            self.assertIn('--sysfs-pwm-control', args)
            self.assertIn('config/sysfs_pwm_hardware.ini', args)
            self.assertIn('config/parking_tuning.new_car.ini', args)
            current = path / 'config' / 'sysfs_pwm_hardware.ini'
            current.write_text('existing user sysfs profile\n', encoding='utf-8')
            self.assertEqual(run().returncode, 0)
            self.assertEqual(current.read_text(), 'existing user sysfs profile\n')
            program.write_text('#!/bin/sh\necho old\n', encoding='utf-8')
            self.assertNotEqual(run().returncode, 0)


if __name__ == '__main__':
    unittest.main()
