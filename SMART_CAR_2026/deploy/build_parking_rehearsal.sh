#!/bin/sh
set -eu
source_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_root=/home/gy/builds/SMART_CAR_2026-loongarch
cmake -S "$source_root" -B "$build_root" -DBUILD_VEHICLE=ON
cmake --build "$build_root" --target parking_rehearsal --parallel 2
python3 "$source_root/tools/package_parking_rehearsal.py" --program "$build_root/parking_rehearsal"
