"""Package a real LoongArch parking tuning build with checked launcher/template."""
import argparse
import hashlib
import re
from pathlib import Path
from package_auto_probe import ROOT, package_program

def package(program, output):
    header = (ROOT / 'tools/parking_rehearsal.hpp').read_text(encoding='utf-8')
    version = re.search(r'rehearsalVersion="([0-9.-]+)"', header).group(1)
    return package_program(program, output, version=version,
        program_name='parking_rehearsal_20261007', script_name='run_parking_rehearsal.sh',
        manifest_name='PARKING_TUNING_SHA256SUMS', required_marker='--tuning-config',
        extra_payloads={'parking_tuning.example.ini': (ROOT / 'deploy/config/parking_tuning.ini').read_bytes(),
                        'PARKING_TUNING.md': (ROOT / 'deploy/PARKING_TUNING.md').read_bytes(),
                        'new_car_hardware.ini': (ROOT / 'deploy/config/new_car_hardware.ini').read_bytes(),
                        'new_car_vehicle.ini': (ROOT / 'deploy/config/new_car_vehicle.ini').read_bytes(),
                        **{name: (ROOT / 'deploy' / name).read_bytes() for name in (
                            'run_new_car_parking.sh', 'parking_tuning.new_car.example.ini',
                            'manual_capture.new_car.ini', 'NEW_CAR_PARKING.md')}})

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--program', required=True)
    parser.add_argument('--output', default=str(ROOT / 'deploy/parking_rehearsal_20261010_1_verified.zip'))
    args = parser.parse_args()
    output, manifest = package(args.program, args.output)
    print('VERIFIED_PACKAGE ' + str(output))
    print('zip_sha256=' + hashlib.sha256(Path(output).read_bytes()).hexdigest())
    print(manifest, end='')

if __name__ == '__main__':
    main()
