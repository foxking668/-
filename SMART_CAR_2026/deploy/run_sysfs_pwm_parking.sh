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
    *'sysfs_pwm_parking version=2026-10-16.5'*'--sysfs-pwm-control'*'--no-camera'*) ;;
    *) echo 'Wrong executable; require sysfs PWM compatibility 2026-10-16.5.' >&2; exit 1;;
esac
mkdir -p config
if [ ! -e config/parking_tuning.new_car.ini ]; then
    cp parking_tuning.sysfs_pwm.example.ini config/parking_tuning.new_car.ini
fi
# Separate hardware profile: never overwrite the user's XX or CC profiles.
for name in hardware vehicle; do
    if [ ! -e "config/sysfs_pwm_$name.ini" ]; then cp "sysfs_pwm_$name.ini" "config/sysfs_pwm_$name.ini"; fi
done
# Preserve the user's actual speed/times. Only the old named neutral is known;
# non-neutral PWM counts have no verified angle mapping and require user input.
tuning=config/parking_tuning.new_car.ini
if grep -Eq '^[[:space:]]*(correction_)?servo_pwm[[:space:]]*=' "$tuning"; then
    tuning_backup="$tuning.before_original725_$(date +%s)_$$.bak"
    tuning_temporary="$tuning.servo725_upgrade_$$.tmp"
    [ ! -e "$tuning_backup" ] && [ ! -L "$tuning_backup" ] && [ ! -e "$tuning_temporary" ] && [ ! -L "$tuning_temporary" ] || { echo 'Tuning migration path already exists; no tuning changed.' >&2; exit 1; }
    cp -p "$tuning" "$tuning_backup"
    cp -p "$tuning" "$tuning_temporary"
    if ! { printf '%s\n' '# CURRENT 2026-10-16.5: edit steer_command/correction_steer_command; 0=center, negative=left, positive=right.' \
            '# Previous comments below are historical; servo_pwm counts no longer control steering.' && sed -E \
        -e 's/^([[:space:]]*)servo_pwm[[:space:]]*=[[:space:]]*4470(\.0+)?([[:space:]]*(#.*)?)$/\1steer_command=0\3/' \
        -e 's/^([[:space:]]*)correction_servo_pwm[[:space:]]*=[[:space:]]*4470(\.0+)?([[:space:]]*(#.*)?)$/\1correction_steer_command=0\3/' \
        "$tuning_backup"; } > "$tuning_temporary"; then
        rm -f -- "$tuning_temporary"; exit 1
    fi
    if grep -Eq '^[[:space:]]*(correction_)?servo_pwm[[:space:]]*=' "$tuning_temporary"; then
        rm -f -- "$tuning_temporary"
        printf 'OLD_PWM_REQUIRES_ANGLE: tuning retained; backup=%s\n' "$tuning_backup" >&2
        echo 'Replace non-neutral servo_pwm/correction_servo_pwm with steer_command/correction_steer_command; 0=center, negative=left, positive=right. Old PWM values are not angles.' >&2
        exit 1
    fi
    checked=0
    "$program" --sysfs-pwm-control --no-camera --check-config --allow-partial \
        --config manual_capture.sysfs_pwm.ini --hardware-config sysfs_pwm_hardware.ini \
        --vehicle-config config/sysfs_pwm_vehicle.ini --tuning-config "$tuning_temporary" || checked=$?
    if { [ "$checked" -ne 0 ] && [ "$checked" -ne 2 ]; } || ! cmp -s "$tuning" "$tuning_backup"; then
        rm -f -- "$tuning_temporary"; echo 'Original7.25 tuning migration rejected or edited concurrently; original tuning retained.' >&2; exit 1
    fi
    mv -- "$tuning_temporary" "$tuning"
    printf 'TUNING_MIGRATED original7.25 steer_command; speed/times preserved; BACKUP %s\n' "$tuning_backup"
fi
# Existing profiles survive unzip. Migrate only the known channel/period pair;
# validate original-formula tuning before replacing the backed-up hardware INI.
profile=config/sysfs_pwm_hardware.ini
servo_path=$(sed -n 's/^[[:space:]]*servo_pwm[[:space:]]*=[[:space:]]*//p' "$profile" | sed 's/[[:space:]]*#.*$//; s/[[:space:]]*$//')
servo_period=$(sed -n 's/^[[:space:]]*servo_period_ns[[:space:]]*=[[:space:]]*//p' "$profile" | sed 's/[[:space:]]*#.*$//; s/[[:space:]]*$//')
case "$servo_path:$servo_period" in
    /sys/class/pwm/pwmchip1/pwm0/duty_cycle:3040000) ;;
    /sys/class/pwm/pwmchip1/pwm0/duty_cycle:3333333|/sys/class/pwm/pwmchip2/pwm0/duty_cycle:3333333|/sys/class/pwm/pwmchip2/pwm0/duty_cycle:3040000)
        backup="$profile.before_725_servo_$(date +%s)_$$.bak"
        temporary="$profile.servo725_upgrade_$$.tmp"
        [ ! -e "$backup" ] && [ ! -L "$backup" ] && [ ! -e "$temporary" ] && [ ! -L "$temporary" ] || { echo '7.25 migration path already exists; no profile changed.' >&2; exit 1; }
        cp -p "$profile" "$backup"
        cp -p "$profile" "$temporary"
        if ! sed -e '/^[[:space:]]*servo_pwm[[:space:]]*=/ s#/sys/class/pwm/pwmchip2/pwm0/duty_cycle#/sys/class/pwm/pwmchip1/pwm0/duty_cycle#' \
            -e '/^[[:space:]]*servo_period_ns[[:space:]]*=/ s/3333333/3040000/' "$backup" > "$temporary"; then
            rm -f -- "$temporary"; exit 1
        fi
        checked=0
        "$program" --sysfs-pwm-control --no-camera --check-config --allow-partial \
            --config manual_capture.sysfs_pwm.ini --hardware-config "$temporary" \
            --vehicle-config config/sysfs_pwm_vehicle.ini --tuning-config config/parking_tuning.new_car.ini || checked=$?
        if { [ "$checked" -ne 0 ] && [ "$checked" -ne 2 ]; } || ! cmp -s "$profile" "$backup"; then
            rm -f -- "$temporary"; echo '7.25 migration rejected or profile changed while checking; original profile retained. Use steer_command instead of non-neutral servo_pwm.' >&2; exit 1
        fi
        mv -- "$temporary" "$profile"
        printf 'SERVO_PROFILE_MIGRATED original7.25 PWM1 period3040000; BACKUP %s\n' "$backup"
        ;;
    *) echo 'Expected one PWM1/PWM2 servo path and known period3040000/3333333; no profile changed.' >&2; exit 1;;
esac
printf 'SERVO_PWM_PATH %s\n' /sys/class/pwm/pwmchip1/pwm0/duty_cycle
printf '%s\n' '无相机运行：不打开相机、不录像、不巡线；按舵机值和电机时长执行，保留编码器PID与停机控制。'
printf '%s\n' 'SYSFS原版7.25：方向用steer_command，0回正、负左正右；PWM1周期3040000ns，运行偏移-15。旧servo_pwm=4470仅兼容为0，其他旧PWM值须改成角度命令。电机参数和阶段时间保留。'
printf '%s\n' '两次启动：先关舵机开关，首次等PREPARE提示后Ctrl+C；再次运行同一命令，正式试验提示后开舵机、Enter执行。结束先关舵机再Q；Y/N保存。'
exec "$program" --config manual_capture.sysfs_pwm.ini \
    --hardware-config config/sysfs_pwm_hardware.ini \
    --vehicle-config config/sysfs_pwm_vehicle.ini \
    --tuning-config config/parking_tuning.new_car.ini \
    --sysfs-pwm-control --no-camera --allow-partial --output captures/parking_tuning
