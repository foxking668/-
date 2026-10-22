#pragma once

namespace Contral {
// Period must be valid before disable/duty writes, including a freshly exported
// channel whose kernel period is zero. Used only by the parking-specific API.
template<class Pwm, class WriteAngle>
bool restartSteeringPwm(Pwm &pwm, unsigned int period, WriteAngle writeAngle) {
    if (!pwm.setPeriod(period)) return false;
    if (!pwm.disable()) return false;
    if (!writeAngle()) return false; // keep PWM disabled if the target write fails
    return pwm.enable();
}
}
