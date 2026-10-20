#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT
c++ -std=c++17 -O1 -UNDEBUG -pthread -Isrc tests/zebra_sysfs_tests.cpp \
  src/Contral/PWM/PwmController.cpp src/Contral/GPIO/GPIO.cpp src/ZebraTest/IoStatus.cpp -lstdc++fs -o "$tmpdir/sysfs_tests"
"$tmpdir/sysfs_tests"
cc -std=c11 -O1 -UNDEBUG -Isrc -c src/Contral/PID/PID.c -o "$tmpdir/pid.o"
c++ -std=c++17 -O1 -UNDEBUG -pthread -Isrc tests/zebra_hardware_tests.cpp \
  src/Contral/Motor/Motor.cpp src/Contral/Steer/Steer.cpp src/Contral/PWM/PwmController.cpp \
  src/Contral/GPIO/GPIO.cpp src/ZebraTest/IoStatus.cpp "$tmpdir/pid.o" src/Other/Log/Log.cpp \
  $(pkg-config --cflags --libs opencv4) -lstdc++fs -o "$tmpdir/hardware_tests"
"$tmpdir/hardware_tests"
c++ -std=c++17 -O1 -UNDEBUG -Isrc tests/zebra_unit_tests.cpp src/ZebraTest/ZebraTest.cpp src/ZebraTest/ZebraDetector.cpp \
  $(pkg-config --cflags --libs opencv4) -o "$tmpdir/zebra_tests"
"$tmpdir/zebra_tests" config/zebra_test.ini
