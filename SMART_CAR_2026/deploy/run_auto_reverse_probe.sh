#!/bin/sh
# Explicit servo-only +2 test. No motor commands or configuration rewrites.
set -eu
if [ "$#" -ne 0 ]; then
    printf '%s\n' 'Usage: sh ./run_auto_reverse_probe.sh (no arguments)' >&2
    exit 1
fi
cd -- "$(dirname -- "$0")"
probe_program=./manual_capture_auto_probe_20261007
if [ ! -x "$probe_program" ]; then
    printf '%s\n' 'Missing executable manual_capture_auto_probe_20261007; upload and chmod +x it first.' >&2
    exit 1
fi
export LD_LIBRARY_PATH="/home/root/opencv-4.11-loongarch/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
probe_help=$("$probe_program" --help)
case "$probe_help" in
    *'version=2026-10-07.4'*) ;;
    *) printf '%s\n' 'Wrong recorder version: require 2026-10-07.4.' >&2; exit 1 ;;
esac
case "$probe_help" in
    *'--auto-probe reverse'*) ;;
    *) printf '%s\n' 'Recorder does not support the automatic probe.' >&2; exit 1 ;;
esac
printf '%s\n' '保持小车静止；看到【开始后拉 / AUTO_PULL_READY】后再后拉20~30cm，然后停稳。无需再输入按键。'
exec "$probe_program" --config manual_capture.ini \
    --hardware-config config/calibration_hardware.ini \
    --vehicle-config config/calibration_vehicle.ini \
    --allow-partial --duration 45 --auto-probe reverse \
    --output captures/auto_reverse_probe_plus2
