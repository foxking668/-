"""Offline package/launcher tests. Fake ELF input is never a deployable build."""
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import package_parking_rehearsal as packager

class PackageTests(unittest.TestCase):
    def setUp(self):
        build = ROOT / 'build'
        build.mkdir(exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix='parking-package-test-', dir=build)).resolve()
        self.directory.relative_to(build.resolve())
        self.program = self.directory / 'program'
        self.archive = self.directory / 'test.zip'
    def tearDown(self):
        shutil.rmtree(self.directory)
    def elf(self, machine=258, marker=b'--tuning-config', version=b'2026-10-10.3'):
        data = bytearray(64)
        data[:6] = b'\x7fELF\x02\x01'
        data[18:20] = machine.to_bytes(2, 'little')
        self.program.write_bytes(bytes(data) + version + marker)
    def test_roundtrip(self):
        self.elf()
        output, manifest = packager.package(self.program, self.archive)
        with zipfile.ZipFile(output) as archive:
            self.assertEqual(len(archive.namelist()), 11)
            self.assertIsNone(archive.testzip())
            for name in ('run_parking_rehearsal.sh', 'run_new_car_parking.sh'):
                self.assertTrue(archive.read(name).startswith(b'#!/bin/sh\n'))
                self.assertNotIn(b'\r', archive.read(name))
            self.assertEqual(archive.read('parking_tuning.example.ini'), (ROOT / 'deploy/config/parking_tuning.ini').read_bytes())
            for line in manifest.splitlines():
                checksum, name = line.split('  ', 1)
                self.assertEqual(hashlib.sha256(archive.read(name)).hexdigest(), checksum)
    def test_wrong_machine(self):
        self.elf(machine=62)
        with self.assertRaisesRegex(ValueError, 'LoongArch'): packager.package(self.program, self.archive)
    def test_missing_feature(self):
        self.elf(marker=b'')
        with self.assertRaisesRegex(ValueError, 'required mode'): packager.package(self.program, self.archive)
    def test_old_version_preserves_destination(self):
        self.archive.write_bytes(b'previous')
        self.elf(version=b'2026-10-07.5')
        with self.assertRaisesRegex(ValueError, 'rebuild first'): packager.package(self.program, self.archive)
        self.assertEqual(self.archive.read_bytes(), b'previous')
    def test_launcher_keeps_existing_tuning(self):
        shell = shutil.which('bash')
        if sys.platform == 'win32': shell = r'C:\Program Files\Git\bin\bash.exe'
        if not shell or not Path(shell).exists(): self.skipTest('Bash unavailable')
        launcher = self.directory / 'run_parking_rehearsal.sh'
        launcher.write_bytes((ROOT / 'deploy/run_parking_rehearsal.sh').read_bytes())
        fake = self.directory / 'parking_rehearsal_20261007'
        fake.write_text('#!/bin/sh\nif [ "$1" = --help ]; then echo "parking_rehearsal version=2026-10-10.3 --tuning-config"; else printf "%s\\n" "$@" > received_args; fi\n', newline='\n')
        fake.chmod(0o755)
        (self.directory / 'parking_tuning.example.ini').write_bytes(b'template')
        folder = self.directory / 'config'
        folder.mkdir()
        tuning = folder / 'parking_tuning.ini'
        tuning.write_bytes(b'user-selected-params')
        subprocess.run([shell, '-c', 'chmod +x parking_rehearsal_20261007; sh ./run_parking_rehearsal.sh'], cwd=self.directory, check=True, capture_output=True)
        self.assertEqual(tuning.read_bytes(), b'user-selected-params')
        args = (self.directory / 'received_args').read_text().splitlines()
        self.assertIn('--tuning-config', args)
        self.assertNotIn('--cc-motor-control', args)
        self.assertNotIn('--duration', args)
        tuning.unlink()
        subprocess.run([shell, './run_parking_rehearsal.sh'], cwd=self.directory, check=True, capture_output=True)
        self.assertEqual(tuning.read_bytes(), b'template')

    def test_new_car_launcher_keeps_custom_tuning_and_profiles(self):
        shell = r'C:\Program Files\Git\bin\bash.exe' if sys.platform == 'win32' else shutil.which('bash')
        if not shell or not Path(shell).exists(): self.skipTest('Bash unavailable')
        (self.directory / 'run_new_car_parking.sh').write_bytes((ROOT / 'deploy/run_new_car_parking.sh').read_bytes())
        fake = self.directory / 'parking_rehearsal_20261007'
        fake.write_text('#!/bin/sh\nif [ "$1" = --help ]; then echo "parking_rehearsal version=2026-10-10.3 --upgrade-cc-tuning --cc-motor-control sysfs:duty_ns"; else printf "%s\\n" "$@" > received_args; printf "%s\\n" "$@" >> all_args; fi\n', newline='\n')
        fake.chmod(0o755)
        folder = self.directory / 'config'; folder.mkdir()
        (self.directory / 'parking_tuning.new_car.example.ini').write_bytes(b'template')
        for name in ('new_car_hardware.ini', 'new_car_vehicle.ini'):
            (self.directory / name).write_bytes(b'profile-template')
        subprocess.run([shell, '-c', 'chmod +x parking_rehearsal_20261007; sh ./run_new_car_parking.sh'], cwd=self.directory, check=True, capture_output=True)
        tuning = folder / 'parking_tuning.new_car.ini'
        self.assertEqual(tuning.read_bytes(), b'template')
        for name in ('new_car_hardware.ini', 'new_car_vehicle.ini'):
            self.assertEqual((folder / name).read_bytes(), b'profile-template')
            (folder / name).write_bytes(b'user-profile')
        tuning.write_bytes(b'user-tuning')
        subprocess.run([shell, './run_new_car_parking.sh'], cwd=self.directory, check=True, capture_output=True)
        self.assertEqual(tuning.read_bytes(), b'user-tuning')
        for name in ('new_car_hardware.ini', 'new_car_vehicle.ini'):
            self.assertEqual((folder / name).read_bytes(), b'user-profile')
        args = (self.directory / 'received_args').read_text().splitlines()
        self.assertIn('config/new_car_hardware.ini', args)
        self.assertIn('config/parking_tuning.new_car.ini', args)
        self.assertIn('--cc-motor-control', args)
        all_args=(self.directory / 'all_args').read_text().splitlines()
        self.assertEqual(all_args.count('--upgrade-cc-tuning'), 2)
        self.assertLess(all_args.index('--upgrade-cc-tuning'), all_args.index('--config'))

    def test_new_car_template_only_has_global_target_and_durations(self):
        tuning=(ROOT / 'deploy/config/parking_tuning.new_car.ini').read_text(encoding='utf-8')
        self.assertNotIn('motor_left_command=', tuning)
        self.assertNotIn('motor_right_command=', tuning)
        self.assertEqual(tuning.count('motor_target_rps='), 1)
        self.assertEqual(tuning.count('motor_run_time_s='), 6)
        self.assertEqual(tuning, (ROOT / 'deploy/parking_tuning.new_car.example.ini').read_text(encoding='utf-8'))

    def test_new_car_launcher_aborts_if_upgrade_fails(self):
        shell=r'C:\Program Files\Git\bin\bash.exe' if sys.platform=='win32' else shutil.which('bash')
        if not shell or not Path(shell).exists(): self.skipTest('Bash unavailable')
        (self.directory / 'run_new_car_parking.sh').write_bytes((ROOT / 'deploy/run_new_car_parking.sh').read_bytes())
        fake=self.directory / 'parking_rehearsal_20261007'
        fake.write_text('#!/bin/sh\nif [ "$1" = --help ]; then echo "parking_rehearsal version=2026-10-10.3 --upgrade-cc-tuning --cc-motor-control sysfs:duty_ns"; elif [ "$1" = --cc-motor-control ]; then exit 1; else touch hardware_started; fi\n', newline='\n')
        fake.chmod(0o755)
        folder=self.directory / 'config';folder.mkdir()
        (folder / 'parking_tuning.new_car.ini').write_bytes(b'invalid-user-config')
        for name in ('new_car_hardware.ini', 'new_car_vehicle.ini'): (folder / name).write_bytes(b'profile')
        result=subprocess.run([shell,'-c','chmod +x parking_rehearsal_20261007; sh ./run_new_car_parking.sh'],cwd=self.directory,capture_output=True)
        self.assertNotEqual(result.returncode,0)
        self.assertFalse((self.directory / 'hardware_started').exists())
        self.assertEqual((folder / 'parking_tuning.new_car.ini').read_bytes(),b'invalid-user-config')

    def test_new_car_launcher_rejects_old_binary(self):
        shell = r'C:\Program Files\Git\bin\bash.exe' if sys.platform == 'win32' else shutil.which('bash')
        if not shell or not Path(shell).exists(): self.skipTest('Bash unavailable')
        (self.directory / 'run_new_car_parking.sh').write_bytes((ROOT / 'deploy/run_new_car_parking.sh').read_bytes())
        fake = self.directory / 'parking_rehearsal_20261007'
        fake.write_text('#!/bin/sh\necho "parking_rehearsal version=2026-10-08.5 --tuning-config"\n', newline='\n')
        fake.chmod(0o755)
        result = subprocess.run([shell, '-c', 'chmod +x parking_rehearsal_20261007; sh ./run_new_car_parking.sh'], cwd=self.directory, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'Old executable', result.stderr)
        self.assertFalse((self.directory / 'config').exists())

if __name__ == '__main__': unittest.main()
