#pragma once
#include "ParkingController.hpp"
#include <fstream>
#include <string>

namespace Parking {
struct SteeringPwmSnapshot {
    int expectedDutyNs = -1;
    int sysfsDutyNs = -1;
    int sysfsPeriodNs = -1;
    int sysfsEnabled = -1;
    const char *status() const {
        if (expectedDutyNs < 0) return "INVALID_COMMAND";
        if (sysfsDutyNs < 0 || sysfsPeriodNs < 0 || sysfsEnabled < 0)
            return "READ_FAILED";
        if (sysfsEnabled != 1) return "PWM_DISABLED";
        if (sysfsPeriodNs != 3040000) return "PERIOD_MISMATCH";
        return sysfsDutyNs == expectedDutyNs ? "MATCH" : "MISMATCH";
    }
    bool matches() const { return std::string(status()) == "MATCH"; }
};

inline int readPwmSysfsValue(const std::string &path) {
    std::ifstream input(path);
    int value = -1;
    return (input >> value) && value >= 0 ? value : -1;
}

// Read the kernel attributes after the real write, under steering ownership.
// They describe kernel PWM configuration, not physical wheel position. Failure
// is diagnostic only: it never drives a stage transition or adjusts steering.
inline SteeringPwmSnapshot inspectSteeringPwm(
    float angle, float servoOffset, const std::string &pwmDirectory) {
    SteeringPwmSnapshot snapshot;
    const float duty = steeringDutyNs(angle, servoOffset);
    if (std::isfinite(duty) && duty > 0 && duty < 3040000)
        snapshot.expectedDutyNs = static_cast<int>(duty);
    snapshot.sysfsDutyNs = readPwmSysfsValue(pwmDirectory + "/duty_cycle");
    snapshot.sysfsPeriodNs = readPwmSysfsValue(pwmDirectory + "/period");
    snapshot.sysfsEnabled = readPwmSysfsValue(pwmDirectory + "/enable");
    return snapshot;
}
}
