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
    *'parking_rehearsal version=2026-10-08.5'*'--tuning-config'*) ;;
    *) printf '%s\n' 'Wrong tuning program; require parking_rehearsal version=2026-10-08.5.' >&2; exit 1 ;;
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
printf '%s\n' '单阶段电机调试：原始PWM（填2000就写2000，无2000人为上限，须在硬件duty_max内）；运行时间0禁用电机；一次Enter开始记录、设置舵机，静止等待后执行。阶段1启用巡线时先确认黑线再前进，纠偏不重启计时。P停电机并暂停记录，C只恢复记录，之后须Enter才运动；Q提前结束，P/C/Q后按Enter。保存当前段参数授权重执行（P后的电机仍需Enter）。时间到或持续丢线停机回正、采集短停稳尾段，结束只询问一次Y/N，直接按键；下一次手动启动。'
exec "$program" --config manual_capture.ini \
    --hardware-config config/calibration_hardware.ini \
    --vehicle-config config/calibration_vehicle.ini \
    --tuning-config config/parking_tuning.ini \
    --allow-partial --output captures/parking_tuning
