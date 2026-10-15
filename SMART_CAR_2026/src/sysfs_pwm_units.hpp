#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace car2026 {
// Motor compatibility units only; steering uses the original 7.25 formula.
constexpr int parkingMotorPwmScale=10000;
inline double parkingCountsToRps(double counts) {
    if(!std::isfinite(counts) || counts<1 || counts>400 || std::floor(counts)!=counts)
        throw std::runtime_error("motor_target_counts must be integer 1..400 equivalent pulses/5ms (7.25 PPR=1024)");
    return counts/(1024.*.005);
}
inline int parkingMotorLimitNs(double raw) {
    if(!std::isfinite(raw) || raw<20 || raw>3500 || std::floor(raw)!=raw)
        throw std::runtime_error("motor_raw_pwm must be integer 20..3500 in explicit 10000 compatibility units");
    return int(raw*50000/parkingMotorPwmScale);
}
}
