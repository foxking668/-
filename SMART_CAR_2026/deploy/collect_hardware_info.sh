#!/bin/sh
# Inventory only: no GPIO/PWM writes, encoder reads, I2C scans, or motor control.
set -eu
cd "$(dirname "$0")"
mkdir -p captures
report="captures/hardware_info_$(date -u +%Y%m%dT%H%M%SZ)_$$.txt"
{
    printf 'UTC: '; date -u
    printf '\nHost / architecture / current directory\n'
    hostname
    uname -a
    pwd
    printf '\nRelevant device names (listing only)\n'
    for device in /dev/video* /dev/zf* /dev/i2c-* /dev/ttyS* /dev/ttyUSB* /dev/gpiochip*; do
        [ ! -e "$device" ] || ls -l "$device"
    done
    printf '\nAlready exported GPIO / ADC paths (listing only)\n'
    for root in /sys/class/gpio /sys/bus/iio/devices; do
        [ ! -d "$root" ] || ls -l "$root"
    done
    printf '\nVehicle process names\n'
    if command -v pgrep >/dev/null 2>&1; then pgrep -x SMART_CAR_2026 || true; fi
    printf '\nGPIO line metadata (no value reads or output changes)\n'
    if command -v gpioinfo >/dev/null 2>&1; then gpioinfo || true; fi
    printf '\nC++ recorder library linkage\n'
    if [ -f ./manual_capture ] && command -v ldd >/dev/null 2>&1; then
        ldd ./manual_capture || true
    else
        printf 'Upload compiled manual_capture to inspect its dependencies\n'
    fi
    printf '\nCamera formats (query only)\n'
    if command -v v4l2-ctl >/dev/null 2>&1; then
        v4l2-ctl --list-devices || true
        v4l2-ctl -d /dev/video0 --list-formats-ext || true
    else
        printf 'v4l2-ctl: unavailable\n'
    fi
    printf '\nStorage\n'
    df -h .
} > "$report" 2>&1
cat "$report"
printf '\nReport saved: %s/%s\n' "$(pwd)" "$report"
