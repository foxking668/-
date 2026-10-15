#pragma once
#include "steering_command_limits.hpp"
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace car2026 {
constexpr uint32_t reference725ServoPeriod=3040000;
constexpr float reference725ServoOffset=-15.0f;
// Exact active expression in 7.25.zip Steer::setAngle, including float arithmetic
// and conversion to unsigned nanoseconds at PwmController::setDutyCycle.
inline uint32_t reference725ServoPulse(float angle,float offset) {
    if(!std::isfinite(angle) || !std::isfinite(offset))
        throw std::runtime_error("7.25 servo angle/offset must be finite");
    angle+=offset;
    if(angle<0 || angle>180) throw std::runtime_error("7.25 adjusted servo angle outside 0..180");
    const float duty=(((angle-90.0f)/90.0f)*5.0f)*
        (float(reference725ServoPeriod)/100.0f)+1520000.0f;
    return uint32_t(duty);
}
inline uint32_t reference725ServoInitPulse() {
    // Global.cpp sets -15 only AFTER Steer::init() calls setAngle(90).
    return reference725ServoPulse(90.0f,0.0f);
}
inline uint32_t reference725ServoCommandPulse(double command,uint32_t period) {
    if(period!=reference725ServoPeriod)
        throw std::runtime_error("Original 7.25 servo requires period=3040000 ns");
    if(!std::isfinite(command) || command<nativeServoCommandMin || command>nativeServoCommandMax)
        throw std::runtime_error("7.25 steer_command must be -75..105; 0 is nominal centered command");
    return reference725ServoPulse(90.0f+float(command),reference725ServoOffset);
}
}
