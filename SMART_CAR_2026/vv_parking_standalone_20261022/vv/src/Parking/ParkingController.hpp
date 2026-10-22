#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

// Parameter-only reverse parking. No camera, encoder or IMU dependency.
namespace Parking {
inline bool isSecondLap(int completedLaps) { return completedLaps == 1; }

struct Stage {
    bool enabled = true;
    int steerOffset = 0; // existing setAngle units relative to center; not wheel degrees
    int pwm = 0;         // positive magnitude; controller supplies the reverse sign
    int totalMs = 0;     // includes settleMs, preserving the supplied ZIP's timing
    int holdMs = 0;      // measured from drive start; 0 = hold throughout drive
};
struct Config {
    bool enabled = true;
    float center = 102.0f; // measured neutral ~1700us with servoOffset=95
    float servoOffset = 95.0f;
    int approachMs = 4000;
    int settleMs = 500;
    bool leftEnabled = false;
    bool rightEnabled = true;
    std::array<Stage, 3> stages{{
        {true, -107, 12000, 2250, 1400},
        {true, 118, 12000, 2000, 1000},
        {true, 0, 9000, 1000, 0}}};
};
enum class Phase { Idle, StopHold, Approach, S2, S4, S5, Done, Fault };
inline const char *phaseName(Phase p) {
    switch(p) {
    case Phase::Idle: return "IDLE";
    case Phase::StopHold: return "SECOND_LAP_STOP";
    case Phase::Approach: return "APPROACH_LINE";
    case Phase::S2: return "S2_LEFT";
    case Phase::S4: return "S4_RIGHT";
    case Phase::S5: return "S5_STRAIGHT";
    case Phase::Done: return "DONE";
    case Phase::Fault: return "FAULT";
    }
    return "UNKNOWN";
}
struct Output {
    bool ownsActuators = false;
    bool steeringValid = true;
    float angle = 102;
    int leftPwm = 0, rightPwm = 0;
    Phase phase = Phase::Idle;
};
// Mirrors Steer::setAngle's existing float calculation, in nanoseconds.
// These are command values, not measured wheel angles or electrical pulses.
inline float steeringDutyNs(float angle, float servoOffset) {
    return (((angle + servoOffset - 90.0f) / 90.0f) * 5.0f) *
        (3040000.0f / 100.0f) + 1520000.0f;
}
inline bool validSteering(const Config &c, int bias = 0) {
    const float duty = steeringDutyNs(c.center + bias, c.servoOffset);
    return std::isfinite(c.center) && std::isfinite(c.servoOffset) &&
        c.center >= -400 && c.center <= 400 &&
        c.servoOffset >= -150 && c.servoOffset <= 150 && duty > 0 && duty < 3040000;
}
inline std::string validate(const Config &c) {
    if(!validSteering(c)) return "invalid parking center/servo offset";
    if(c.approachMs < 0 || c.approachMs > 60000) return "invalid approach_ms";
    if(c.settleMs < 0 || c.settleMs > 5000) return "invalid settle_ms";
    if(!c.enabled) return {};
    for(const auto &s : c.stages) {
        if(!s.enabled) continue;
        if(s.totalMs <= c.settleMs || s.totalMs > 20000)
            return "enabled stage total_ms must exceed settle_ms";
        if(s.holdMs < 0 || s.holdMs > s.totalMs-c.settleMs)
            return "stage hold_ms exceeds drive duration";
        if(s.pwm <= 0 || s.pwm > 50000) return "invalid reverse PWM";
        if(!c.leftEnabled && !c.rightEnabled) return "both parking motors disabled";
        if(s.steerOffset < -400 || s.steerOffset > 400 || !validSteering(c,s.steerOffset))
            return "invalid stage steering";
    }
    return {};
}

class Controller {
public:
    bool arm(const Config &c, std::int64_t holdUntilMs) {
        if(phase_ != Phase::Idle) return false;
        config_ = c; holdUntilMs_ = holdUntilMs;
        fault_ = validate(c);
        phase_ = fault_.empty() ? Phase::StopHold : Phase::Fault;
        return true;
    }
    void forwardApplied(std::int64_t nowMs) {
        if(phase_ == Phase::Approach && forwardAtMs_ < 0) forwardAtMs_ = nowMs;
    }
    void stop(const char *reason) {
        if(phase_ != Phase::Idle) { phase_ = Phase::Fault; fault_ = reason; }
    }
    Phase phase() const { return phase_; }
    const Config &config() const { return config_; }
    const std::string &fault() const { return fault_; }
    bool approachStarted() const { return phase_ == Phase::Approach && forwardAtMs_ >= 0; }
    Output tick(std::int64_t nowMs, bool initialReverseFinished = true) {
        bool entered = false;
        if(phase_ == Phase::StopHold && nowMs >= holdUntilMs_ && initialReverseFinished) {
            phase_ = Phase::Approach;
            if(config_.approachMs == 0) { enterNext(nowMs); entered = true; }
        }
        if(phase_ == Phase::Approach && forwardAtMs_ >= 0 &&
           nowMs-forwardAtMs_ >= config_.approachMs) {
            enterNext(nowMs); entered = true;
        }
        if(isStage() && !entered && nowMs-stageAtMs_ >= config_.stages[index_].totalMs) {
            enterNext(nowMs); entered = true;
        }
        Output out;
        out.phase = phase_; out.angle = config_.center;
        out.steeringValid = validSteering(config_);
        out.ownsActuators = isStage() || phase_ == Phase::Done || phase_ == Phase::Fault;
        if(isStage()) {
            const Stage &s = config_.stages[index_];
            const auto elapsed = nowMs-stageAtMs_;
            const bool centered = s.holdMs > 0 && elapsed >= config_.settleMs+s.holdMs;
            out.angle += centered ? 0 : s.steerOffset;
            // Every transition emits zero first, even with settle_ms=0.
            if(!entered && elapsed >= config_.settleMs) {
                out.leftPwm = config_.leftEnabled ? -s.pwm : 0;
                out.rightPwm = config_.rightEnabled ? -s.pwm : 0;
            }
        }
        return out;
    }
private:
    bool isStage() const { return index_ >= 0 && index_ < 3 &&
        (phase_ == Phase::S2 || phase_ == Phase::S4 || phase_ == Phase::S5); }
    void enterNext(std::int64_t nowMs) {
        do { ++index_; } while(index_ < 3 && !config_.stages[index_].enabled);
        if(!config_.enabled || index_ >= 3) { phase_ = Phase::Done; return; }
        static constexpr Phase phases[] = {Phase::S2, Phase::S4, Phase::S5};
        phase_ = phases[index_]; stageAtMs_ = nowMs;
    }
    Config config_;
    Phase phase_ = Phase::Idle;
    int index_ = -1;
    std::int64_t holdUntilMs_ = 0, forwardAtMs_ = -1, stageAtMs_ = 0;
    std::string fault_;
};
}
