#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT
c++ -std=c++17 -O1 -UNDEBUG -Isrc tests/zebra_unit_tests.cpp src/ZebraTest/ZebraTest.cpp src/ZebraTest/ZebraDetector.cpp \
  $(pkg-config --cflags --libs opencv4) -o "$tmpdir/zebra_tests"
"$tmpdir/zebra_tests" config/zebra_test.ini
