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
    *'parking_rehearsal version=2026-10-10.5'*'--upgrade-cc-tuning'*'--cc-motor-control'*'sysfs:duty_ns'*) ;;
    *) printf '%s\n' 'Old executable: require 2026-10-10.5 single-speed cc PID version; do not reuse the old binary.' >&2; exit 1 ;;
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
"$program" --cc-motor-control --upgrade-cc-tuning --tuning-config config/parking_tuning.new_car.ini
printf '%s\n' '每次试验启动两次：第一次关闭舵机开关启动，等初始化完成提示后Ctrl+C退出；第二次再启动，看到正式试验提示后打开舵机，再Enter执行和记录。结束先关舵机再Q退出。六个阶段均适用。'
printf '%s\n' '新车直接调参：config/parking_tuning.new_car.ini。只用[session] motor_target_rps调全部阶段左右轮快慢，不再配置左右电机PWM；各阶段motor_run_time_s控制启停，0禁用，阶段6停车。缺省9rps，PID64/32/48，每50ms计算PWM。一次Enter执行；P停机并暂停，C只恢复记录，Enter再运动；Q结束，Y/N保存。'
exec "$program" --config manual_capture.new_car.ini \
    --hardware-config config/new_car_hardware.ini \
    --vehicle-config config/new_car_vehicle.ini \
    --tuning-config config/parking_tuning.new_car.ini \
    --cc-motor-control --allow-partial --output captures/parking_tuning
