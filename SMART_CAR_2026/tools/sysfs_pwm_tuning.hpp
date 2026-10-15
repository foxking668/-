#pragma once
#include "capture_data.hpp"
#include "sysfs_pwm_units.hpp"
#include <array>
#include <algorithm>
#include <iomanip>
#include <set>
namespace car2026 { namespace capture {
constexpr const char* sysfsPwmVersion="2026-10-16.2";
struct ConvertedPwmTuning {
    std::string normalized;
    std::array<double,6> primary{},correction{};
};
// Only translates configuration units. Stage execution and cc PID stay shared.
inline ConvertedPwmTuning convertPwmTuning(const std::string& text) {
    if(text.size()>16384) throw std::runtime_error("Tuning file too large");
    ConvertedPwmTuning result;std::istringstream input(text);std::ostringstream output;
    output<<std::setprecision(17);
    std::set<std::string> keys;std::string line,section;int index=-1;unsigned number=0;
    while(std::getline(input,line)) {
        if(++number==1 && line.compare(0,3,"\xef\xbb\xbf")==0) line.erase(0,3);
        line=trimmed(line.substr(0,line.find('#')));if(line.empty()) continue;
        if(line.front()=='[' && line.back()==']') {
            section=line.substr(1,line.size()-2);index=-1;
            if(section.size()==7 && section.substr(0,6)=="stage_" && section[6]>='1' && section[6]<='6') index=section[6]-'1';
            output<<line<<'\n';continue;
        }
        const auto equal=line.find('=');
        if(section.empty() || equal==std::string::npos) throw std::runtime_error("Expected [section] and key=value at line "+std::to_string(number));
        const auto key=trimmed(line.substr(0,equal)),value=trimmed(line.substr(equal+1));
        if(!keys.insert(section+"."+key).second) throw std::runtime_error("Duplicate PWM tuning key: "+key);
        if(section=="session" && key=="motor_control") {
            if(value!="speed") throw std::runtime_error("sysfs PWM compatibility requires motor_control=speed; raw factory output is not a sysfs command");
            output<<"reverse_right_only=0\n";
        } else if(section=="session" && key=="motor_target_counts") output<<"motor_target_rps="<<parkingCountsToRps(finiteNumber(value))<<'\n';
        else if(section=="session" && key=="motor_raw_pwm") output<<"motor_pid_pwm_limit="<<parkingMotorLimitNs(finiteNumber(value))<<'\n';
        else if(index>=0 && (key=="servo_pwm" || key=="correction_servo_pwm")) {
            if(key=="correction_servo_pwm" && index!=1 && index!=3) throw std::runtime_error("Correction only supported in stages 2/4");
            const double delta=parkingPwmDelta(finiteNumber(value));
            if(key=="servo_pwm") result.primary[size_t(index)]=delta;else result.correction[size_t(index)]=delta;
            // Base parser checks the same signs/hold pairing using a unit placeholder.
            output<<(key=="servo_pwm" ? "steer_command" : "correction_steer_command")<<'='<<(delta==0 ? 0 : delta>0 ? 1 : -1)<<'\n';
        } else if(index==0 && key.rfind("line_steer_",0)==0) {
            if(key=="line_steer_kp") output<<"line_pwm_kp="<<value<<'\n';
            else if(key=="line_steer_kd") output<<"line_pwm_kd="<<value<<'\n';
            else if(key=="line_steer_deadband_px") output<<"line_pwm_deadband="<<value<<'\n';
            else if(key=="line_steer_step_per_frame") output<<"line_pwm_step="<<value<<'\n';
            else throw std::runtime_error("Unknown PWM line key: "+key);
        } else {
            if(key=="steer_command" || key=="correction_steer_command" || key=="reverse_right_only" || key.find("target_rps")!=std::string::npos || key.rfind("motor_pid_",0)==0 || key=="motor_left_command" || key=="motor_right_command" || key.rfind("line_pwm_",0)==0)
                throw std::runtime_error("Do not mix legacy angle/rps/PWM settings into sysfs PWM mode: "+key);
            output<<key<<'='<<value<<'\n';
        }
    }
    for(const auto* key:{"motor_control","motor_target_counts","motor_raw_pwm"})
        if(!keys.count(std::string("session.")+key)) throw std::runtime_error(std::string("Missing session.")+key);
    for(int i=1;i<=6;++i) if(!keys.count("stage_"+std::to_string(i)+".servo_pwm")) throw std::runtime_error("All six servo_pwm keys required");
    result.normalized=output.str();return result;
}
// Retains the XX PWM steering sign, gains, frame derivative scaling and step cap.
class SysfsPwmLineController {
public:
    void reset() {previousNs_=0;previousError_=0;pwm_=parkingPwmCenter;}
    double update(double target,int64_t now,double kp,double kd,double deadband,int step) {
        if(!std::isfinite(target) || now<=0 || (previousNs_ && now<=previousNs_)) throw std::runtime_error("PWM line requires fresh finite feedback");
        double error=79-target;if(std::abs(error)<=deadband) error=0;
        const double scale=previousNs_ && now-previousNs_<250000000 ? std::min(2.,16666667./double(now-previousNs_)) : 0;
        const int requested=int(std::clamp(parkingPwmCenter+kp*error+kd*(error-previousError_)*scale,double(parkingPwmRight),double(parkingPwmLeft)));
        pwm_=std::clamp(requested,pwm_-step,pwm_+step);previousNs_=now;previousError_=error;
        return parkingPwmDelta(pwm_);
    }
private:
    int64_t previousNs_=0;double previousError_=0;int pwm_=parkingPwmCenter;
};
}}
