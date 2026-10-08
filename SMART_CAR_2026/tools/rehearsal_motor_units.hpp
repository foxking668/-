#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
namespace car2026 { namespace capture {
// Wire-format bound, not a tuning cap. Each device additionally enforces duty_max.
constexpr double rehearsalPwmStorageMax=std::numeric_limits<uint16_t>::max();
inline bool validRehearsalRawPwm(double command) {
    return std::isfinite(command) && command>=0 && command<=rehearsalPwmStorageMax && std::floor(command)==command;
}
}}
