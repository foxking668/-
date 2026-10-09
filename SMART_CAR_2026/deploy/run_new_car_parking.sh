#!/bin/sh
set -eu
if [ "$#" -ne 0 ]; then
    printf '%s\n' 'Usage: sh ./run_new_car_parking.sh (no arguments)' >&2
    exit 1
fi
cd -- "$(dirname -- "$0")"
program=./parking_rehearsal_20261007
for library_dir in /home/root/opencv-4.11-loongarch/install/lib /home/qy/opencv-4.11.0/loongson/install/lib /home/han/opencv-4.11.0/loongson/install/lib; do
    if [ -d "$library_dir" ]; then
        export LD_LIBRARY_PATH="$library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
done
if [ ! -x "$program" ]; then
    printf '%s\n' 'Missing parking_rehearsal_20261007; build this version with build_new_car_parking.sh first.' >&2
    exit 1
fi
program_help=$("$program" --help)
case "$program_help" in
    *'parking_rehearsal version=2026-10-09.1'*'sysfs:duty_ns'*) ;;
    *) printf '%s\n' 'Old executable: require 2026-10-09.1 sysfs version; do not reuse the old binary.' >&2; exit 1 ;;
esac
mkdir -p config
for profile in new_car_hardware.ini new_car_vehicle.ini; do
    if [ ! -e "config/$profile" ]; then
        cp "$profile" "config/$profile"
    fi
done
if [ ! -e config/parking_tuning.new_car.ini ]; then
    mkdir -p config
    cp parking_tuning.new_car.example.ini config/parking_tuning.new_car.ini
fi
printf '%s\n' '新车直接调参：config/parking_tuning.new_car.ini。PWM单位纳秒，3000=6%，0..50000。一次Enter授权执行；P停机并暂停，C只恢复记录，Enter再运动；Q结束。到时停机、回正，Y/N保存。运行中保存当前段参数会重新执行。'
exec "$program" --config manual_capture.new_car.ini \
    --hardware-config config/new_car_hardware.ini \
    --vehicle-config config/new_car_vehicle.ini \
    --tuning-config config/parking_tuning.new_car.ini \
    --allow-partial --output captures/parking_tuning
