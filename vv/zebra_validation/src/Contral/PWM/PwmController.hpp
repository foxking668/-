#pragma once
#include <string>

namespace Contral {
// Linux sysfs PWM; the optional root is for host tests, production uses /sys/class/pwm.
class PwmController {
public:
    PwmController(int chip, int channel, const std::string& root="/sys/class/pwm");
    ~PwmController();
    PwmController(const PwmController&) = delete;
    PwmController& operator=(const PwmController&) = delete;
    bool initialize();
    // Configure while disabled: duty=0 BEFORE changing period, then desired duty.
    bool prepare(unsigned int periodNs, unsigned int dutyNs=0);
    bool enable();
    bool disable();
    bool setPeriod(unsigned int periodNs);
    bool setDutyCycle(unsigned int dutyNs);
    int readPeriod() { return period_; }
    int readDutyCycle() { return duty_; }
private:
    std::string path_, exportPath_;
    int chip_, channel_, period_=0, duty_=0;
    bool initialized_=false;
    bool readState(int& period, int& duty, int& enabled);
    bool verify(int expectedEnable);
};
}
