#pragma once
#include "hardware.hpp"
#include "reference_725_servo.hpp"
#include "steering_command_limits.hpp"
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
        if(io_.reference725Servo()) {
            if(std::abs(commandDeg)>params_.max_steer_deg || params_.steer_sign!=1 || params_.steer_offset_deg!=0)
                throw std::runtime_error("Original 7.25 servo requires command within vehicle limit and no second offset/sign transform");
            reference725ServoCommandPulse(commandDeg,uint32_t(config_.servo_period_ns));attempted_=true;return io_.writeServoCommand(commandDeg);
        }
        if(!std::isfinite(commandDeg) || std::abs(commandDeg)>params_.max_steer_deg ||
           (io_.nativeServo() && (commandDeg<nativeServoCommandMin || commandDeg>nativeServoCommandMax)))
            throw std::runtime_error("Steering command outside configured command limit");
        attempted_=true; // A failed write may already have changed the output.
        if(io_.nativeServo()) return io_.writeServoCommand(params_.steer_sign*commandDeg+params_.steer_offset_deg);
        const auto duty=servoDuty(params_.steer_sign*commandDeg+params_.steer_offset_deg,config_,info_);
        io_.writePwmDuty(config_.servo_pwm,duty);return duty;
    }
    void close() {
        if(closed_) return;
        closed_=true; // A failed shutdown is reported, never silently retried.
        if(attempted_ && io_.reference725Servo()) io_.writeServoCommand(0);
        else if(attempted_ && io_.nativeServo()) io_.writeServoCommand(params_.steer_offset_deg);
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
