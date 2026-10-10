#pragma once
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
namespace car2026 { namespace capture {
// cc(1): 50 ms incremental speed PID, native signed encoder revolutions/s.
struct ReferenceMotorTuning {
    double leftRps=9,rightRps=9,kp=64,ki=32,kd=48;
    int pwmLimit=12000; // cc default; user may raise to the physical 50000 ns period.
    void validate() const {
        for(double value:{leftRps,rightRps})
            if(!std::isfinite(value) || value<0 || value>100) throw std::runtime_error("motor target rps must be 0..100");
        for(double value:{kp,ki,kd})
            if(!std::isfinite(value) || value<0 || value>1000000) throw std::runtime_error("motor PID gains must be 0..1000000");
        if(pwmLimit<100 || pwmLimit>50000) throw std::runtime_error("motor_pid_pwm_limit must be integer 100..50000 ns");
    }
    bool operator==(const ReferenceMotorTuning& other) const {
        return leftRps==other.leftRps && rightRps==other.rightRps && kp==other.kp && ki==other.ki && kd==other.kd && pwmLimit==other.pwmLimit;
    }
};
class ReferenceMotorSpeed {
public:
    ReferenceMotorSpeed();
    ~ReferenceMotorSpeed();
    ReferenceMotorSpeed(const ReferenceMotorSpeed&)=delete;
    ReferenceMotorSpeed& operator=(const ReferenceMotorSpeed&)=delete;
    // Reset history only at a new authorized motor trial; correction keeps history.
    void reset(const ReferenceMotorTuning&,double direction,bool leftEnabled,bool rightEnabled);
    std::array<int,2> update(double leftMeasuredRps,double rightMeasuredRps);
    std::array<double,2> targets() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}}
