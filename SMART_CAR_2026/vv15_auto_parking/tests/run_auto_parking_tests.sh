#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
binary=$(mktemp /tmp/vv15-auto-parking-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
${CXX:-c++} -std=c++17 -O1 -Wall -Wextra -Werror -Isrc tests/auto_parking_test.cpp -o "$binary"
"$binary" config/parking_tuning.vv.ini
