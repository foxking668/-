#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace car2026 {
// Explicit compatibility units, NOT metadata reported by an absent /dev/zf driver.
constexpr int parkingPwmScale=10000;
constexpr int parkingPwmRight=3370,parkingPwmCenter=4470,parkingPwmLeft=5170;
constexpr uint32_t parkingPwmServoPeriod=3333333; // 300 Hz, rounded to ns.
inline double parkingPwmDelta(double pwm) {
    if(!std::isfinite(pwm) || std::floor(pwm)!=pwm || pwm<parkingPwmRight || pwm>parkingPwmLeft)
        throw std::runtime_error("servo_pwm must be integer 3370..5170; center=4470");
    return parkingPwmCenter-pwm; // Shared stage model: positive right, negative left.
}
inline int parkingPwmValue(double delta) {
    const double pwm=parkingPwmCenter-delta;
    parkingPwmDelta(pwm);return int(pwm);
}
inline uint32_t parkingPwmServoNs(double delta,uint32_t period) {
    if(period!=parkingPwmServoPeriod) throw std::runtime_error("PWM compatibility servo requires period=3333333 ns");
    return uint32_t(std::llround(double(parkingPwmValue(delta))*period/parkingPwmScale));
}
inline double parkingCountsToRps(double counts) {
    if(!std::isfinite(counts) || counts<1 || counts>400 || std::floor(counts)!=counts)
        throw std::runtime_error("motor_target_counts must be integer 1..400 equivalent pulses/5ms (7.25 PPR=1024)");
    return counts/(1024.*.005);
}
inline int parkingMotorLimitNs(double raw) {
    if(!std::isfinite(raw) || raw<20 || raw>3500 || std::floor(raw)!=raw)
        throw std::runtime_error("motor_raw_pwm must be integer 20..3500 in explicit 10000 compatibility units");
    return int(raw*50000/parkingPwmScale);
}
}
