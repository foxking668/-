#pragma once
#include "Contral/Steer/SteeringPulse.hpp"
namespace AutoParking {
// Shared adapter; preflight and the actual driver use the same original VV formula.
template<class Steer>
bool writeSteeringCommand(Steer* steer,double command,double offset) {
    float duty;
    if(!steer || !Contral::SteeringPulse::commandToDuty(command,offset,duty)) return false;
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
