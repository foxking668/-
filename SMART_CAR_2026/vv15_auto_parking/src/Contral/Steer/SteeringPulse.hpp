#pragma once
#include <cmath>
#include <limits>

namespace Contral {
namespace SteeringPulse {
inline constexpr int periodNs=3040000;

// Original VV arithmetic and float operation order; shared by preflight and output.
inline float dutyForAngle(float angle,float offset,int period=periodNs) {
    angle+=offset;
    return (((angle-90.0f)/90.0f)*5.0f)*(float)(period/100.0f)+1520000.0f;
}
inline bool validDuty(float duty,int period=periodNs) {
    return std::isfinite(duty) && duty>=0 && duty<=period;
}
inline bool commandToDuty(double command,double offset,float& duty) {
    duty=std::numeric_limits<float>::quiet_NaN();
    const double angle=90.0+command;
    const double maximum=std::numeric_limits<float>::max();
    if(!std::isfinite(command) || !std::isfinite(offset) || !std::isfinite(angle) ||
       std::abs(angle)>maximum || std::abs(offset)>maximum) return false;
    duty=dutyForAngle(static_cast<float>(angle),static_cast<float>(offset));
    return validDuty(duty);
}
}
}
