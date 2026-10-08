#pragma once
#include "rehearsal_servo.hpp"
#include <functional>
namespace car2026 { namespace capture {
struct MotorEvent {
    int stage=0;unsigned revision=0,trial=0;
    int64_t beginNs=0,endNs=0;
    double left=0,right=0;
    std::string event,error;
    int64_t motorZeroNs=0; // End of both motor zero writes, before steering return latency.
};
// Only the authorized main thread starts motion. The worker only writes zero.
// No encoder, servo, beep or IMU initialization; capture owns all sensor reads.
class RehearsalMotor {
public:
    RehearsalMotor(DeviceIo& io,const HardwareConfig& hardware,const Params& params)
        :io_(io),hardware_(hardware),params_(params) {
        hardware_.validate();params_.validate();
        try {
            zeroLocked();
            info_[0]=readPwmInfo(io_,hardware_.motor_left_pwm);
            info_[1]=readPwmInfo(io_,hardware_.motor_right_pwm);
            worker_=std::thread([this] {watch();});
        } catch(...) {try {zeroLocked();}catch(...) {}throw;}
    }
    ~RehearsalMotor() noexcept {try {close();}catch(...) {}}
    RehearsalMotor(const RehearsalMotor&)=delete;
    RehearsalMotor& operator=(const RehearsalMotor&)=delete;
    void start(double left,double right,double seconds,int stage,unsigned revision,unsigned trial) {
        if(stage<0 || stage>4 || !std::isfinite(seconds) || seconds<=0 || seconds>120 ||
           !std::isfinite(left) || !std::isfinite(right) || (left==0 && right==0) ||
           std::abs(left)>std::min(12000.,params_.pwm_limit) || std::abs(right)>std::min(12000.,params_.pwm_limit) ||
           (stage==0 ? (left<0 || right<0) : (left>0 || right>0)))
            throw std::runtime_error("Invalid powered stage request");
        std::lock_guard<std::mutex> held(mutex_);
        if(closed_ || active_ || !fault_.empty()) throw std::runtime_error("Motor unavailable or already active: "+fault_);
        context_={stage,revision,trial,rehearsalMonotonicNs(),0,left,right,"MOTOR_START",""};
        try {
            // A stationary wheel stays PWM-zero without changing its direction GPIO.
            if(left!=0) writeMotorCommand(io_,hardware_.motor_left_pwm,hardware_.motor_left_dir,info_[0],left,
                hardware_.motor_command_range,params_.pwm_limit,int(hardware_.motor_left_forward_level),directions_[0]);
            if(right!=0) writeMotorCommand(io_,hardware_.motor_right_pwm,hardware_.motor_right_dir,info_[1],right,
                hardware_.motor_command_range,params_.pwm_limit,int(hardware_.motor_right_forward_level),directions_[1]);
        } catch(const std::exception& error) {
            fault_=error.what();try {zeroLocked();}catch(const std::exception& zero) {fault_+="; "+std::string(zero.what());}
            auto failed=context_;failed.event="MOTOR_START_FAILED";failed.error=fault_;failed.endNs=rehearsalMonotonicNs();events_.push_back(failed);
            throw std::runtime_error(fault_);
        }
        context_.endNs=rehearsalMonotonicNs();events_.push_back(context_);
        active_=true;deadline_=context_.endNs+int64_t(seconds*1e9);heartbeat_=context_.endNs;
        wake_.notify_all();
    }
    void heartbeat() {std::lock_guard<std::mutex> held(mutex_);if(active_) heartbeat_=rehearsalMonotonicNs();wake_.notify_all();}
    bool active() const {std::lock_guard<std::mutex> held(mutex_);return active_;}
    void setStopAction(std::function<void()> action) {
        std::lock_guard<std::mutex> held(mutex_);
        if(closed_ || active_) throw std::runtime_error("Stop action must be installed before motion");
        stopAction_=std::move(action);
    }
    // Same lock as worker stop: feedback cannot write a nonzero steering command
    // after motor expiry. Lock order is motor -> servo; no caller may reverse it.
    template<class Action> bool whileActive(Action action) {
        std::lock_guard<std::mutex> held(mutex_);
        if(closed_ || !active_) return false;
        const auto now=rehearsalMonotonicNs();
        if(now>=deadline_) {stopLocked("MOTOR_STOP_DURATION");return false;}
        if(now>=heartbeat_+int64_t(params_.frame_timeout_s*1e9)) {stopLocked("MOTOR_STOP_WATCHDOG");return false;}
        action();return true;
    }
    void stop(const std::string& reason) {
        std::lock_guard<std::mutex> held(mutex_);if(closed_) return;
        if(active_) stopLocked(reason); // An already expired trial is never restarted here.
        if(!fault_.empty()) throw std::runtime_error(fault_);
    }
    std::deque<MotorEvent> takeEvents() {
        std::lock_guard<std::mutex> held(mutex_);std::deque<MotorEvent> result;result.swap(events_);return result;
    }
    void close() {
        {std::lock_guard<std::mutex> held(mutex_);if(closed_) return;closed_=true;wake_.notify_all();}
        if(worker_.joinable()) worker_.join();
        std::lock_guard<std::mutex> held(mutex_);
        if(active_) stopLocked("MOTOR_STOP_EXIT");else zeroLocked();
        if(!fault_.empty()) throw std::runtime_error(fault_);
    }
private:
    void zeroLocked() {
        std::string error;
        for(const auto* path:{&hardware_.motor_left_pwm,&hardware_.motor_right_pwm})
            try {io_.writePwmDuty(*path,0);}catch(const std::exception& failure) {error+=*path+": "+failure.what()+"; ";}
        if(!error.empty()) throw std::runtime_error("Both motor zeros attempted: "+error);
    }
    void stopLocked(const std::string& reason) {
        active_=false;auto event=context_;event.event=reason;event.beginNs=rehearsalMonotonicNs();
        try {zeroLocked();event.motorZeroNs=rehearsalMonotonicNs();}catch(const std::exception& error) {event.error=error.what();fault_=event.error;}
        // Center independently of a stalled camera/recording loop, after both motor zeros.
        if(stopAction_) try {stopAction_();}catch(const std::exception& error) {
            event.error+="; STOP_ACTION_FAILED: "+std::string(error.what());fault_=event.error;
        }
        event.endNs=rehearsalMonotonicNs();events_.push_back(event);wake_.notify_all();
    }
    void watch() noexcept {
        std::unique_lock<std::mutex> held(mutex_);
        while(!closed_) {
            if(!active_) {wake_.wait(held,[this] {return closed_ || active_;});continue;}
            const auto heartbeatDeadline=heartbeat_+int64_t(params_.frame_timeout_s*1e9);
            const auto expiry=std::min(deadline_,heartbeatDeadline);
            wake_.wait_until(held,std::chrono::steady_clock::time_point(std::chrono::nanoseconds(expiry)));
            if(closed_ || !active_) continue;
            const auto now=rehearsalMonotonicNs();
            if(now>=deadline_) stopLocked("MOTOR_STOP_DURATION");
            else if(now>=heartbeat_+int64_t(params_.frame_timeout_s*1e9)) stopLocked("MOTOR_STOP_WATCHDOG");
        }
    }
    DeviceIo& io_;HardwareConfig hardware_;Params params_;PwmInfo info_[2];int directions_[2]{-1,-1};
    mutable std::mutex mutex_;std::condition_variable wake_;bool closed_=false,active_=false;
    int64_t deadline_=0,heartbeat_=0;std::string fault_;MotorEvent context_;std::deque<MotorEvent> events_;
    std::function<void()> stopAction_;
    std::thread worker_;
};
}}
