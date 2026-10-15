"""Package the existing parking engine with the separate 7.25 sysfs/PWM profile."""
import argparse
import hashlib
import re
from pathlib import Path
from package_auto_probe import ROOT, package_program


def package(program, output):
    names = (
        'sysfs_pwm_hardware.ini', 'sysfs_pwm_vehicle.ini',
        'parking_tuning.sysfs_pwm.example.ini', 'manual_capture.sysfs_pwm.ini',
        'SYSFS_PWM_PARKING.md',
    )
    version = re.search(r'sysfsPwmVersion="([0-9.-]+)"', (ROOT / 'tools/sysfs_pwm_tuning.hpp').read_text(encoding='utf-8')).group(1)
    return package_program(
        program, output, version=version,
        program_name='parking_sysfs_pwm_20261016',
        script_name='run_sysfs_pwm_parking.sh',
        manifest_name='SYSFS_PWM_SHA256SUMS',
        required_marker='--sysfs-pwm-control',
        extra_payloads={name: (ROOT / 'deploy' / name).read_bytes() for name in names},
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--program', required=True)
    parser.add_argument('--output', default=str(ROOT / 'deploy/parking_sysfs_pwm_20261016_4_verified.zip'))
    args = parser.parse_args()
    path, manifest = package(args.program, args.output)
    print('VERIFIED_PACKAGE ' + str(path))
    print('zip_sha256=' + hashlib.sha256(Path(path).read_bytes()).hexdigest())
    print(manifest, end='')


if __name__ == '__main__':
    main()
