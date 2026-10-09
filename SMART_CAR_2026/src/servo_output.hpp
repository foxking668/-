#pragma once
#include "hardware.hpp"
#include <iostream>

namespace car2026 {
// Servo-only output. Never constructs FactoryBoard or touches motor devices.
class ServoOutput {
public:
    ServoOutput(DeviceIo& io,const HardwareConfig& config,const Params& params)
        : io_(io),config_(config),params_(params) {
        config_.validate();params_.validate();
        info_=readPwmInfo(io_,config_.servo_pwm); // Metadata only; no startup write.
    }
    ~ServoOutput() noexcept {
        try {close();}
        catch(const std::exception& error) {std::cerr<<"Servo return-to-zero failed: "<<error.what()<<'\n';}
    }
    ServoOutput(const ServoOutput&)=delete;
    ServoOutput& operator=(const ServoOutput&)=delete;
    uint32_t set(double commandDeg) {
        if(closed_) throw std::runtime_error("Servo output already closed");
        if(!std::isfinite(commandDeg) || std::abs(commandDeg)>params_.max_steer_deg)
            throw std::runtime_error("Steering command outside configured command limit");
        attempted_=true; // A failed write may already have changed the output.
        if(io_.nativeServo()) return io_.writeServoCommand(params_.steer_sign*commandDeg+params_.steer_offset_deg);
        const auto duty=servoDuty(params_.steer_sign*commandDeg+params_.steer_offset_deg,config_,info_);
        io_.writePwmDuty(config_.servo_pwm,duty);return duty;
    }
    void close() {
        if(closed_) return;
        closed_=true; // A failed shutdown is reported, never silently retried.
        if(attempted_ && io_.nativeServo()) io_.writeServoCommand(params_.steer_offset_deg);
        else if(attempted_)
            io_.writePwmDuty(config_.servo_pwm,servoDuty(params_.steer_offset_deg,config_,info_));
    }
    const PwmInfo& info() const {return info_;}
    bool commandAttempted() const {return attempted_;}
private:
    DeviceIo& io_;HardwareConfig config_;Params params_;PwmInfo info_{};
    bool attempted_=false,closed_=false;
};
}
