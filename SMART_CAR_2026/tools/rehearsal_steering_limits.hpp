#pragma once
#include "../src/steering_command_limits.hpp"
namespace car2026 { namespace capture {
// Command limits, not measured wheel angles or verified mechanical travel.
constexpr double legacyParkingSteerLimit=15;
constexpr double ccParkingSteerMin=nativeServoCommandMin;
constexpr double ccParkingSteerLimit=nativeServoCommandMax;
}}
