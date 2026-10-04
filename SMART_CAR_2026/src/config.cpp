#include "core.hpp"
#include <cctype>
#include <set>

namespace car2026 {
namespace {
std::string trim(const std::string& s) {
    const auto a=s.find_first_not_of(" \t\r\n");
    if(a==std::string::npos) return {};
    return s.substr(a,s.find_last_not_of(" \t\r\n")-a+1);
}
}
void Params::validate() const {
#define CHECK(n,d,lo,hi) if(!std::isfinite(n)||n<lo||n>hi) throw std::runtime_error("Invalid parameter: " #n);
    CAR_PARAMETERS(CHECK)
#undef CHECK
    for(double s : {steer_sign,encoder_left_sign,encoder_right_sign,imu_yaw_sign,ring_side})
        if(s!=-1 && s!=1) throw std::runtime_error("Direction sign must be -1 or +1");
    if(blue_h_min>=blue_h_max || bar_roi_top>=bar_roi_bottom ||
       line_width_min_ratio>=line_width_max_ratio || ring_min_yaw_deg>=ring_max_yaw_deg ||
       line_hold_s>=stage_timeout_s || cross_near_y_ratio>cross_stop_y_ratio)
        throw std::runtime_error("Inconsistent parameter ranges");
    if(imu_format!="wit11" && imu_format!="ascii") throw std::runtime_error("Unknown IMU format");
    if(imu_baud!=9600 && imu_baud!=57600 && imu_baud!=115200) throw std::runtime_error("Unsupported IMU baud");
#define INTEGER(n,d,lo,hi) if((std::string(#n).find("frames")!=std::string::npos || std::string(#n).find("width")!=std::string::npos && std::string(#n).find("ratio")==std::string::npos || std::string(#n).find("height")!=std::string::npos || std::string(#n)=="encoder_ppr" || std::string(#n)=="camera_index") && std::floor(n)!=n) throw std::runtime_error("Expected integer: " #n);
    CAR_PARAMETERS(INTEGER)
#undef INTEGER
    for(double b : {motion_calibrated,require_imu,stop_every_cross})
        if(b!=0 && b!=1) throw std::runtime_error("Boolean parameter must be 0 or 1");
}
Params Params::load(const std::string& filename) {
    std::ifstream file(filename);
    if(!file) throw std::runtime_error("Cannot open config: "+filename);
    Params p; std::set<std::string> seen;
    std::string line; int lineNumber=0;
    while(std::getline(file,line)) {
        ++lineNumber;
        if(lineNumber==1 && line.size()>=3 && uint8_t(line[0])==0xef && uint8_t(line[1])==0xbb && uint8_t(line[2])==0xbf) line.erase(0,3);
        line=trim(line.substr(0,line.find('#'))); if(line.empty()) continue;
        const auto equal=line.find('='); if(equal==std::string::npos) throw std::runtime_error("Missing '=' at config line "+std::to_string(lineNumber));
        const auto key=trim(line.substr(0,equal)), value=trim(line.substr(equal+1));
        if(!seen.insert(key).second) throw std::runtime_error("Duplicate parameter: "+key);
        if(key=="imu_device") { p.imu_device=value; continue; }
        if(key=="imu_format") { p.imu_format=value; continue; }
        size_t consumed=0; const double number=std::stod(value,&consumed);
        if(consumed!=value.size() || !std::isfinite(number)) throw std::runtime_error("Invalid number: "+key);
        if(key=="imu_baud") { if(std::floor(number)!=number) throw std::runtime_error("IMU baud must be integer"); p.imu_baud=static_cast<int>(number); continue; }
        bool known=false;
#define READ(n,d,lo,hi) if(key==#n) { p.n=number; known=true; }
        CAR_PARAMETERS(READ)
#undef READ
        if(!known) throw std::runtime_error("Unknown parameter: "+key);
    }
    p.validate(); return p;
}
bool YawTracker::ingest(double wrappedYaw,double time) {
    if(!std::isfinite(wrappedYaw)||!std::isfinite(time)) return false;
    if(!initialized_) { previous_=wrapDegrees(wrappedYaw); lastTime_=time; initialized_=true; return true; }
    const double dt=time-lastTime_, delta=wrapDegrees(wrappedYaw-previous_);
    if(dt<=0 || dt>0.35 || std::abs(delta)>std::max(5.0,300*dt)) {
        // After a gap, re-anchor without claiming the unobserved turn.
        if(dt>0.35) { previous_=wrapDegrees(wrappedYaw); lastTime_=time; }
        return false;
    }
    accumulated_+=delta; previous_=wrapDegrees(wrappedYaw); lastTime_=time; return true;
}
} // namespace car2026
