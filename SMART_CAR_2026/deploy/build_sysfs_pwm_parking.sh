#!/bin/sh
set -eu
source_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_root=${PARKING_BUILD_ROOT:-/home/gy/builds/SMART_CAR_2026-loongarch}
cmake -S "$source_root" -B "$build_root" -DBUILD_VEHICLE=ON
cmake --build "$build_root" --target parking_rehearsal --parallel 2
python3 "$source_root/tools/package_sysfs_pwm_parking.py" --program "$build_root/parking_rehearsal"
printf '%s\n' 'READY deploy/parking_sysfs_pwm_20261016_5_verified.zip; upload to /home/root/gy, preserve the existing tuning INI.'
