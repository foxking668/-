#!/bin/sh
set -eu
[ "$#" -eq 0 ] || { echo 'Usage: sh run_sysfs_pwm_parking.sh'; exit 1; }
cd -- "$(dirname -- "$0")"
program=./parking_sysfs_pwm_20261016
for library_dir in /home/root/qy/opencv411/lib /home/root/opencv-4.11-loongarch/install/lib /home/qy/opencv-4.11.0/loongson/install/lib /home/han/opencv-4.11.0/loongson/install/lib; do
    if [ -d "$library_dir" ]; then export LD_LIBRARY_PATH="$library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; fi
done
[ -x "$program" ] || { echo 'Missing parking_sysfs_pwm_20261016; build this version first.' >&2; exit 1; }
case "$("$program" --help)" in
    *'sysfs_pwm_parking version=2026-10-16.4'*'--sysfs-pwm-control'*'--no-camera'*) ;;
    *) echo 'Wrong executable; require sysfs PWM compatibility 2026-10-16.4.' >&2; exit 1;;
esac
mkdir -p config
if [ ! -e config/parking_tuning.new_car.ini ]; then
    cp parking_tuning.sysfs_pwm.example.ini config/parking_tuning.new_car.ini
fi
# Separate hardware profile: never overwrite the user's XX or CC profiles.
for name in hardware vehicle; do
    if [ ! -e "config/sysfs_pwm_$name.ini" ]; then cp "sysfs_pwm_$name.ini" "config/sysfs_pwm_$name.ini"; fi
done
# Existing .1/.2 installations retain their old profile after unzipping. Migrate
# only the known PWM2 default, with an exact backup and hardware-free validation.
profile=config/sysfs_pwm_hardware.ini
servo_path=$(sed -n 's/^[[:space:]]*servo_pwm[[:space:]]*=[[:space:]]*//p' "$profile" | sed 's/[[:space:]]*#.*$//; s/[[:space:]]*$//')
case "$servo_path" in
    /sys/class/pwm/pwmchip1/pwm0/duty_cycle) ;;
    /sys/class/pwm/pwmchip2/pwm0/duty_cycle)
        backup="$profile.before_pwm1_$(date +%s)_$$.bak"
        temporary="$profile.pwm1_upgrade_$$.tmp"
        [ ! -e "$backup" ] && [ ! -L "$backup" ] && [ ! -e "$temporary" ] && [ ! -L "$temporary" ] || { echo 'PWM1 migration path already exists; no profile changed.' >&2; exit 1; }
        cp -p "$profile" "$backup"
        cp -p "$profile" "$temporary"
        if ! sed '/^[[:space:]]*servo_pwm[[:space:]]*=/ s#/sys/class/pwm/pwmchip2/pwm0/duty_cycle#/sys/class/pwm/pwmchip1/pwm0/duty_cycle#' "$backup" > "$temporary"; then
            rm -f -- "$temporary"; exit 1
        fi
        checked=0
        "$program" --sysfs-pwm-control --no-camera --check-config --allow-partial \
            --config manual_capture.sysfs_pwm.ini --hardware-config "$temporary" \
            --vehicle-config config/sysfs_pwm_vehicle.ini --tuning-config config/parking_tuning.new_car.ini || checked=$?
        if { [ "$checked" -ne 0 ] && [ "$checked" -ne 2 ]; } || ! cmp -s "$profile" "$backup"; then
            rm -f -- "$temporary"; echo 'PWM1 migration rejected or profile changed while checking; original profile retained.' >&2; exit 1
        fi
        mv -- "$temporary" "$profile"
        printf 'SERVO_PROFILE_MIGRATED PWM2->PWM1; BACKUP %s\n' "$backup"
        ;;
    *) echo 'Expected exactly one servo_pwm path on PWM1 or the known old PWM2 profile; no profile changed.' >&2; exit 1;;
esac
printf 'SERVO_PWM_PATH %s\n' /sys/class/pwm/pwmchip1/pwm0/duty_cycle
printf '%s\n' '无相机运行：不打开相机、不录像、不巡线；按舵机值和电机时长执行，保留编码器PID与停机控制。'
printf '%s\n' 'SYSFS 7.25：沿用当前parking_tuning.new_car.ini；3370右/4470中/5170左，逻辑标度10000。40等效counts/5ms=7.8125rps；两轮驱动。'
printf '%s\n' '两次启动：先关舵机开关，首次等PREPARE提示后Ctrl+C；再次运行同一命令，正式试验提示后开舵机、Enter执行。结束先关舵机再Q；Y/N保存。'
exec "$program" --config manual_capture.sysfs_pwm.ini \
    --hardware-config config/sysfs_pwm_hardware.ini \
    --vehicle-config config/sysfs_pwm_vehicle.ini \
    --tuning-config config/parking_tuning.new_car.ini \
    --sysfs-pwm-control --no-camera --allow-partial --output captures/parking_tuning
