#!/bin/sh
set -eu
if [ "$#" -ne 0 ]; then
    printf '%s\n' 'Usage: sh ./run_parking_rehearsal.sh (no arguments)' >&2
    exit 1
fi
cd -- "$(dirname -- "$0")"
program=./parking_rehearsal_20261007
if [ ! -x "$program" ]; then
    printf '%s\n' 'Missing executable parking_rehearsal_20261007; upload the verified LoongArch build first.' >&2
    exit 1
fi
export LD_LIBRARY_PATH="/home/root/opencv-4.11-loongarch/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
program_help=$("$program" --help)
case "$program_help" in
    *'parking_rehearsal version=2026-10-07.2'*'--tuning-config'*) ;;
    *) printf '%s\n' 'Wrong tuning program; require parking_rehearsal version=2026-10-07.2.' >&2; exit 1 ;;
esac
# Install the template once. Never overwrite the user's tuning file.
if [ ! -e config/parking_tuning.ini ]; then
    if [ ! -f parking_tuning.example.ini ]; then
        printf '%s\n' 'Missing parking_tuning.example.ini and config/parking_tuning.ini.' >&2
        exit 1
    fi
    mkdir -p config
    cp parking_tuning.example.ini config/parking_tuning.ini
fi
printf '%s\n' '保存调参文件直接更新并执行；从角度写入成功重新计时，到时回正。P暂停记录，C恢复记录，Q回零保存；字母键后按Enter。'
exec "$program" --config manual_capture.ini \
    --hardware-config config/calibration_hardware.ini \
    --vehicle-config config/calibration_vehicle.ini \
    --tuning-config config/parking_tuning.ini \
    --allow-partial --output captures/parking_tuning
