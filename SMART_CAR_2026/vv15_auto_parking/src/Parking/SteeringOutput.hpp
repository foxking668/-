#pragma once
#include <cmath>
namespace AutoParking {
// Shared adapter; the original VV pulse formula remains solely in Steer::setAngle.
template<class Steer>
bool writeSteeringCommand(Steer* steer,double command,double offset) {
    if(!steer || !std::isfinite(command) || !std::isfinite(offset)) return false;
    steer->setSteerOffset(static_cast<float>(offset));
    return steer->setAngle(static_cast<float>(90.0+command));
}
template<class Motor>
bool stopMotorOutputs(Motor* motor) {
    if(!motor) return false;
    const bool left=motor->setLeftPwm(0);
    const bool right=motor->setRightPwm(0); // Attempt both even if the first fails.
    return left && right;
}
}
