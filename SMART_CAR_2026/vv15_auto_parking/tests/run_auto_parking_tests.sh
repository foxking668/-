#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
work=$(mktemp -d /tmp/vv15-auto-parking-tests.XXXXXX)
trap 'rm -rf "$work"' EXIT
${CXX:-c++} -std=c++17 -O1 -Wall -Wextra -Werror -Isrc tests/auto_parking_test.cpp -o "$work/state_test"
"$work/state_test" config/parking_tuning.vv.ini
${CXX:-c++} -std=c++17 -O1 -Wall -Wextra -Werror -Isrc tests/steering_range_test.cpp -o "$work/steering_range_test"
"$work/steering_range_test"
${CXX:-c++} -std=c++17 -O1 -Wall -Wextra -Werror -pthread -Itests/auto_parking_stub -Isrc \
    src/Parking/AutoParkingRuntime.cpp tests/auto_parking_runtime_test.cpp -lstdc++fs -o "$work/runtime_test"
"$work/runtime_test" "$work/race"
"$work/runtime_test" "$work/manual" manual

${CXX:-c++} -std=c++17 -O1 -Wall -Wextra -Werror -Isrc tests/servo_center_test.cpp -lstdc++fs -o "$work/servo_center_test"
"$work/servo_center_test" "$work/center"
${CXX:-c++} -std=c++17 -O1 -Wall -Wextra -Werror -pthread -Isrc \
    src/Contral/PWM/PwmController.cpp tests/pwm_initialization_test.cpp -lstdc++fs -o "$work/pwm_test"
"$work/pwm_test" "$work/pwm"
