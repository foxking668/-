#pragma once
#include "rehearsal_servo.hpp"
#include "rehearsal_motor_units.hpp"
#include <array>
#include <functional>
#include <sstream>
namespace car2026 { namespace capture {
struct MotorEvent {
    int stage=0;unsigned revision=0,trial=0;
    int64_t beginNs=0,endNs=0;
    double left=0,right=0;
    std::string event,error;
    int64_t motorZeroNs=0; // End of both motor zero writes, before steering return latency.
    std::array<int,2> writtenRaw{{-1,-1}},gpio{{-1,-1}};
    std::array<uint32_t,2> dutyMax{{0,0}},frequency{{0,0}};
    std::array<int,2> pwmEnabled{{-1,-1}};int driverEnabled=-1;
    bool factoryReadback=false;
    bool sysfs=false;
    std::array<double,2> measuredRps{{0,0}};
};
inline std::string motorEventDetail(const MotorEvent& event) {
    std::ostringstream out;
    out<<"units="<<(event.sysfs ? "duty_ns" : "raw_pwm")<<" requested_left="<<event.left<<" requested_right="<<event.right;
    for(size_t i=0;i<2;++i) {
        const auto* side=i==0 ? "left" : "right";
        out<<' '<<side<<"_written_raw="<<event.writtenRaw[i]<<' '<<side<<"_gpio="<<event.gpio[i]
            <<' '<<side<<"_duty_max="<<event.dutyMax[i]<<' '<<side<<"_frequency_hz="<<event.frequency[i]
            <<' '<<side<<"_pwm_enable="<<event.pwmEnabled[i];
    }
    out<<" gpio73_enable="<<event.driverEnabled<<" write_result="<<(event.error.empty() ? "completed" : "failed")
        <<" verification="<<(event.sysfs ? "sysfs_text_readback" : event.factoryReadback ? "factory_status_zero_readback" : "byte_count")
        <<" motor_zero_ns="<<event.motorZeroNs<<" measured_left_rps="<<event.measuredRps[0]<<" measured_right_rps="<<event.measuredRps[1]<<" error="<<event.error;
    return out.str(); // Successful software write only; -1 means unknown, not a measured waveform.
}
// Only the authorized main thread starts motion. Within that trial the worker
// updates the configured speed PID and enforces the original stop deadline.
// No encoder, servo, beep or IMU initialization; capture owns all sensor reads.
class RehearsalMotor {
public:
    RehearsalMotor(DeviceIo& io,const HardwareConfig& hardware,const Params& params)
        :io_(io),hardware_(hardware),params_(params) {
        hardware_.validate();params_.validate();
        try {
            io_.setMotorEnable(false);
            zeroLocked();
            info_[0]=readPwmInfo(io_,hardware_.motor_left_pwm);
            info_[1]=readPwmInfo(io_,hardware_.motor_right_pwm);
            validatePwmMetadataRead(info_[0],sizeof(PwmInfo),hardware_.motor_left_pwm);
            validatePwmMetadataRead(info_[1],sizeof(PwmInfo),hardware_.motor_right_pwm);
            worker_=std::thread([this] {watch();});
        } catch(...) {try {zeroLocked();}catch(...) {}throw;}
    }
    ~RehearsalMotor() noexcept {try {close();}catch(...) {}}
    RehearsalMotor(const RehearsalMotor&)=delete;
    RehearsalMotor& operator=(const RehearsalMotor&)=delete;
    void start(double left,double right,double seconds,int stage,unsigned revision,unsigned trial,bool speedControlled=false) {
        if(stage<0 || stage>4 || !std::isfinite(seconds) || seconds<=0 || seconds>120 ||
           !validRehearsalRawPwm(std::abs(left)) || !validRehearsalRawPwm(std::abs(right)) ||
           (!speedControlled && ((left==0 && right==0) || (stage==0 ? (left<0 || right<0) : (left>0 || right>0)))))
            throw std::runtime_error("Invalid powered stage request");
        std::lock_guard<std::mutex> held(mutex_);
        if(closed_ || active_ || !fault_.empty()) throw std::runtime_error("Motor unavailable or already active: "+fault_);
        if(speedControlled && (!speedControl_ || !feedbackReady_)) throw std::runtime_error("Speed controller requires fresh encoder feedback before start");
        // Check BOTH devices before any nonzero output; never clip a saved tuning value.
        if(std::abs(left)>info_[0].duty_max || std::abs(right)>info_[1].duty_max)
            throw std::runtime_error("Raw PWM exceeds device duty_max: left="+std::to_string(left)+"/"+
                std::to_string(info_[0].duty_max)+" right="+std::to_string(right)+"/"+std::to_string(info_[1].duty_max));
        context_={stage,revision,trial,rehearsalMonotonicNs(),0,left,right,"MOTOR_START",""};
        try {
            io_.setMotorEnable(false);
            // A stationary wheel stays PWM-zero without changing its direction GPIO.
            if(left!=0) writeRawLocked(0,left);
            if(right!=0) writeRawLocked(1,right);
            io_.setMotorEnable(true); // Both PWM/direction writes completed before releasing nSLEEP.
        } catch(const std::exception& error) {
            fault_=error.what();try {zeroLocked();}catch(const std::exception& zero) {fault_+="; "+std::string(zero.what());}
            auto failed=context_;snapshot(failed);failed.event="MOTOR_START_FAILED";failed.error=fault_;failed.endNs=rehearsalMonotonicNs();events_.push_back(failed);
            throw std::runtime_error(fault_);
        }
        snapshot(context_);context_.endNs=rehearsalMonotonicNs();events_.push_back(context_);
        active_=true;deadline_=context_.endNs+int64_t(seconds*1e9);heartbeat_=context_.endNs;
        speedActive_=speedControlled;nextPidNs_=context_.endNs+speedPeriodNs;
        wake_.notify_all();
    }
    void heartbeat() {std::lock_guard<std::mutex> held(mutex_);if(active_) heartbeat_=rehearsalMonotonicNs();wake_.notify_all();}
    bool active() const {std::lock_guard<std::mutex> held(mutex_);return active_;}
    std::array<int,2> outputs() const {std::lock_guard<std::mutex> held(mutex_);return {writtenRaw_[0],writtenRaw_[1]};}
    void setSpeedController(std::function<std::array<int,2>(double,double)> control) {
        std::lock_guard<std::mutex> held(mutex_);
        if(closed_ || active_) throw std::runtime_error("Configure speed controller before motion");
        speedControl_=std::move(control);
    }
    void submitSpeedFeedback(double left,double right) {
        if(!std::isfinite(left) || !std::isfinite(right)) throw std::runtime_error("Invalid speed feedback");
        std::lock_guard<std::mutex> held(mutex_);feedback_={left,right};feedbackReady_=true;
    }
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
    static constexpr int64_t speedPeriodNs=50000000;
    void updateSpeedPwmLocked(int left,int right) {
        if(std::abs(double(left))>info_[0].duty_max || std::abs(double(right))>info_[1].duty_max)
            throw std::runtime_error("Reference PID output exceeds device period");
        writeRawLocked(0,left);writeRawLocked(1,right);
    }
    void snapshot(MotorEvent& event) const {
        for(size_t i=0;i<2;++i) {
            event.writtenRaw[i]=writtenRaw_[i];event.gpio[i]=directions_[i];
            event.dutyMax[i]=info_[i].duty_max;event.frequency[i]=info_[i].freq;
        }
        event.factoryReadback=hardware_.factory_write_readback;
        event.sysfs=hardware_.motor_backend=="sysfs";
        event.measuredRps=feedback_;
        if(event.sysfs) {
            // Diagnostics on failure are best effort; do not hide the original error.
            try {
                for(size_t i=0;i<2;++i) {
                    const auto& path=i==0 ? hardware_.motor_left_pwm : hardware_.motor_right_pwm;
                    event.pwmEnabled[i]=int(std::stoul(io_.readText(path.substr(0,path.rfind('/'))+"/enable")));
                }
                event.driverEnabled=int(std::stoul(io_.readText(hardware_.motor_enable_gpio)));
            } catch(...) {}
        }
    }
    void writeRawLocked(size_t index,double command) {
        const auto& pwm=index==0 ? hardware_.motor_left_pwm : hardware_.motor_right_pwm;
        const auto& dir=index==0 ? hardware_.motor_left_dir : hardware_.motor_right_dir;
        const int forward=int(index==0 ? hardware_.motor_left_forward_level : hardware_.motor_right_forward_level);
        writtenRaw_[index]=-1;
        try {
            if(command==0) {io_.writePwmDuty(pwm,0);writtenRaw_[index]=0;return;} // Reference leaves GPIO unchanged at zero.
            // Reuse zero-before-direction handling with an identity PWM scale.
            // Generic competition control retains its existing normalized units.
            writeMotorCommand(io_,pwm,dir,info_[index],command,info_[index].duty_max,info_[index].duty_max,forward,directions_[index]);
            writtenRaw_[index]=int(std::abs(command));
        } catch(...) {directions_[index]=-1;throw;}
    }
    void zeroLocked() {
        std::string error;
        const std::string* paths[]={&hardware_.motor_left_pwm,&hardware_.motor_right_pwm};
        for(size_t i=0;i<2;++i) {
            writtenRaw_[i]=-1;
            try {io_.writePwmDuty(*paths[i],0);writtenRaw_[i]=0;}
            catch(const std::exception& failure) {error+=*paths[i]+": "+failure.what()+"; ";}
        }
        try {io_.setMotorEnable(false);}catch(const std::exception& failure) {error+="motor enable: "+std::string(failure.what())+"; ";}
        if(!error.empty()) throw std::runtime_error("Both motor zeros attempted: "+error);
    }
    void stopLocked(const std::string& reason) {
        active_=false;feedbackReady_=false;auto event=context_;event.event=reason;event.beginNs=rehearsalMonotonicNs();
        try {zeroLocked();event.motorZeroNs=rehearsalMonotonicNs();}catch(const std::exception& error) {event.error=error.what();fault_=event.error;}
        // Center independently of a stalled camera/recording loop, after both motor zeros.
        if(stopAction_) try {stopAction_();}catch(const std::exception& error) {
            event.error+="; STOP_ACTION_FAILED: "+std::string(error.what());fault_=event.error;
        }
        snapshot(event);event.endNs=rehearsalMonotonicNs();events_.push_back(event);wake_.notify_all();
    }
    void watch() noexcept {
        std::unique_lock<std::mutex> held(mutex_);
        while(!closed_) {
            if(!active_) {wake_.wait(held,[this] {return closed_ || active_;});continue;}
            const auto heartbeatDeadline=heartbeat_+int64_t(params_.frame_timeout_s*1e9);
            const auto expiry=std::min(std::min(deadline_,heartbeatDeadline),speedActive_ ? nextPidNs_ : deadline_);
            wake_.wait_until(held,std::chrono::steady_clock::time_point(std::chrono::nanoseconds(expiry)));
            if(closed_ || !active_) continue;
            const auto now=rehearsalMonotonicNs();
            if(now>=deadline_) stopLocked("MOTOR_STOP_DURATION");
            else if(now>=heartbeat_+int64_t(params_.frame_timeout_s*1e9)) stopLocked("MOTOR_STOP_WATCHDOG");
            else if(speedActive_ && now>=nextPidNs_) {
                auto event=context_;event.event="MOTOR_PID_UPDATE";event.beginNs=now;
                try {
                    const auto pwm=speedControl_(feedback_[0],feedback_[1]);
                    updateSpeedPwmLocked(pwm[0],pwm[1]);event.left=pwm[0];event.right=pwm[1];
                    snapshot(event);event.endNs=rehearsalMonotonicNs();events_.push_back(event);
                    nextPidNs_=event.endNs+speedPeriodNs; // No burst of stale catch-up calculations.
                } catch(const std::exception& error) {fault_=error.what();stopLocked("MOTOR_STOP_PID_FAILED");events_.back().error=fault_;}
                catch(...) {fault_="Unknown speed PID failure";stopLocked("MOTOR_STOP_PID_FAILED");events_.back().error=fault_;}
            }
        }
    }
    DeviceIo& io_;HardwareConfig hardware_;Params params_;PwmInfo info_[2]{};int directions_[2]{-1,-1},writtenRaw_[2]{-1,-1};
    mutable std::mutex mutex_;std::condition_variable wake_;bool closed_=false,active_=false;
    int64_t deadline_=0,heartbeat_=0;std::string fault_;MotorEvent context_;std::deque<MotorEvent> events_;
    std::function<void()> stopAction_;
    std::function<std::array<int,2>(double,double)> speedControl_;
    std::array<double,2> feedback_{{0,0}};bool feedbackReady_=false,speedActive_=false;int64_t nextPidNs_=0;
    std::thread worker_;
};
}}
