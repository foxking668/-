#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
BIN="$(mktemp /tmp/vv-parking-test.XXXXXX)"
PWM_DIR="$(mktemp -d /tmp/vv-parking-pwm-test.XXXXXX)"
trap 'rm -f "$BIN" "$PWM_DIR/duty_cycle" "$PWM_DIR/period" "$PWM_DIR/enable"; rmdir "$PWM_DIR"' EXIT
c++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -UNDEBUG -pthread -Isrc \
    tests/parking_controller_test.cpp -o "$BIN"
"$BIN" "$PWM_DIR"
