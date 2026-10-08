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
    *'parking_rehearsal version=2026-10-08.1'*'--tuning-config'*) ;;
    *) printf '%s\n' 'Wrong tuning program; require parking_rehearsal version=2026-10-08.1.' >&2; exit 1 ;;
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
printf '%s\n' '第一弯左打→右修正→零位；第二弯右打→左修正→零位。启动会显示实际参数及启用状态。保存参数直接执行，最后一步到时按模式收尾。P暂停记录但动作计时继续，C恢复，Q提前结束；P/C/Q后按Enter。结束只询问一次Y/N，直接按键，无需回车；下一次手动启动。'
exec "$program" --config manual_capture.ini \
    --hardware-config config/calibration_hardware.ini \
    --vehicle-config config/calibration_vehicle.ini \
    --tuning-config config/parking_tuning.ini \
    --allow-partial --output captures/parking_tuning
