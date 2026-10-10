#!/bin/sh
set -eu
source_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_root=${PARKING_BUILD_ROOT:-/home/gy/builds/SMART_CAR_2026-loongarch}
cmake -S "$source_root" -B "$build_root" -DBUILD_VEHICLE=ON
cmake --build "$build_root" --target parking_rehearsal reference_cc_lane_tests reference_cc_steering_tests --parallel 2
cmake -E chdir "$build_root" ctest -R '^reference_cc_(lane|steering)_tests$' --output-on-failure
python3 "$source_root/tools/package_parking_rehearsal.py" --program "$build_root/parking_rehearsal"
# Only a checked real LoongArch binary is copied; existing custom tuning remains.
cp "$build_root/parking_rehearsal" "$source_root/deploy/parking_rehearsal_20261007"
chmod +x "$source_root/deploy/parking_rehearsal_20261007"
printf '%s\n' 'READY 2026-10-10.4: deploy/parking_rehearsal_20261007; start with sh deploy/run_new_car_parking.sh on the car.'
