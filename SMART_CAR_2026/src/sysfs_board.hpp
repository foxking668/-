#pragma once
#include "hardware.hpp"
#include "capture_data.hpp"
#include <chrono>
#include <thread>
#include <set>
#include <iostream>
namespace car2026 {
// Text-only transport, independently testable without GPIO/PWM hardware.
class SysfsNodes {
public:
    virtual ~SysfsNodes()=default;
    virtual bool exists(const std::string&)=0;
    virtual std::string read(const std::string&)=0;
    virtual void write(const std::string&,const std::string&)=0;
};
inline uint32_t sysfsNumber(const std::string& text) {
    const auto cleaned=capture::trimmed(text);
    if(cleaned.empty() || cleaned.find_first_not_of("0123456789")!=std::string::npos) throw std::runtime_error("Invalid sysfs integer: "+text);
    size_t used=0;const auto number=std::stoull(cleaned,&used);
    if(used!=cleaned.size() || number>UINT32_MAX)
        throw std::runtime_error("Invalid sysfs integer: "+text);
    return uint32_t(number);
}
inline void sysfsWriteChecked(SysfsNodes& nodes,const std::string& path,uint32_t value) {
    nodes.write(path,std::to_string(value));
    if(sysfsNumber(nodes.read(path))!=value) throw std::runtime_error("Sysfs readback mismatch: "+path+" requested="+std::to_string(value));
}
inline void sysfsExport(SysfsNodes& nodes,const std::string& marker,const std::string& exportPath,uint32_t number) {
    if(nodes.exists(marker)) return;
    try {nodes.write(exportPath,std::to_string(number));}
    catch(...) {if(!nodes.exists(marker)) throw;}
    for(int retry=0;retry<50 && !nodes.exists(marker);++retry) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if(!nodes.exists(marker)) throw std::runtime_error("Export did not create: "+marker);
}
inline void sysfsGpio(SysfsNodes& nodes,const std::string& valuePath,bool output,bool referenceDirection=false) {
    const std::string prefix="/sys/class/gpio/gpio",suffix="/value";
    if(valuePath.rfind(prefix,0)!=0 || valuePath.size()<=prefix.size()+suffix.size() || valuePath.substr(valuePath.size()-suffix.size())!=suffix)
        throw std::runtime_error("Invalid sysfs GPIO value path: "+valuePath);
    const auto number=valuePath.substr(prefix.size(),valuePath.size()-prefix.size()-suffix.size());
    if(number.find_first_not_of("0123456789")!=std::string::npos) throw std::runtime_error("Invalid GPIO number");
    sysfsExport(nodes,valuePath,"/sys/class/gpio/export",sysfsNumber(number));
    nodes.write(valuePath.substr(0,valuePath.size()-suffix.size())+"/direction",output ? (referenceDirection ? "out" : "low") : "in");
    const auto direction=capture::trimmed(nodes.read(valuePath.substr(0,valuePath.size()-suffix.size())+"/direction"));
    if(direction!=(output ? "out" : "in")) throw std::runtime_error("GPIO direction readback failed: "+valuePath);
}
inline std::string sysfsPwmDirectory(const std::string& dutyPath) {
    const std::string prefix="/sys/class/pwm/pwmchip",suffix="/duty_cycle";
    if(dutyPath.rfind(prefix,0)!=0 || dutyPath.size()<=suffix.size() || dutyPath.substr(dutyPath.size()-suffix.size())!=suffix)
        throw std::runtime_error("Invalid sysfs PWM path: "+dutyPath);
    const auto dir=dutyPath.substr(0,dutyPath.size()-suffix.size());const auto split=dir.find("/pwm",prefix.size());
    if(split==std::string::npos || split==prefix.size() || dir.substr(prefix.size(),split-prefix.size()).find_first_not_of("0123456789")!=std::string::npos ||
       dir.substr(split+4).empty() || dir.substr(split+4).find_first_not_of("0123456789")!=std::string::npos)
        throw std::runtime_error("Invalid PWM chip/channel: "+dutyPath);
    return dir;
}
inline void referenceMotorPwmInit(SysfsNodes& nodes,const std::string& dutyPath,uint32_t period) {
    const auto dir=sysfsPwmDirectory(dutyPath);const auto split=dir.rfind("/pwm");
    sysfsExport(nodes,dutyPath,dir.substr(0,split)+"/export",sysfsNumber(dir.substr(split+4)));
    // cc/src/Contral/Motor/Motor.cpp: initialize -> enable -> period -> zero.
    // No pre-period disable (which failed on the board) or added polarity write.
    // The reference does not abort on the initial enable return value. Keep that
    // behavior and order, but report a rejected write instead of hiding it.
    try {sysfsWriteChecked(nodes,dir+"/enable",1);}
    catch(const std::exception& error) {
        std::cerr<<"REFERENCE_PWM_ENABLE "<<dir<<" requested=1: "<<error.what()<<"; continuing reference period/zero sequence\n";
    }
    sysfsWriteChecked(nodes,dir+"/period",period);sysfsWriteChecked(nodes,dutyPath,0);
}
inline void referenceServoPwmInit(SysfsNodes& nodes,const std::string& dutyPath,uint32_t period) {
    const auto dir=sysfsPwmDirectory(dutyPath);const auto split=dir.rfind("/pwm");
    sysfsExport(nodes,dutyPath,dir.substr(0,split)+"/export",sysfsNumber(dir.substr(split+4)));
    // cc/src/Contral/Steer/Steer.cpp: period -> setAngle(90) -> enable.
    sysfsWriteChecked(nodes,dir+"/period",period);
    sysfsWriteChecked(nodes,dutyPath,1520000);
    sysfsWriteChecked(nodes,dir+"/enable",1);
}
inline uint32_t newCarServoNs(double adjustedCommand,double period) {
    if(!std::isfinite(adjustedCommand) || !std::isfinite(period) || period<2500000 || period>20000000)
        throw std::runtime_error("Invalid new-car servo request");
    const double duty=1520000.+adjustedCommand*period/1800.; // Reference setAngle(90+command).
    if(duty<500000 || duty>2500000 || duty>=period) throw std::runtime_error("New-car servo pulse outside valid range");
    return uint32_t(std::llround(duty));
}
inline double newCarEncoderRps(uint32_t periodTicks,uint32_t direction) {
    if(direction>1) throw std::runtime_error("Encoder direction must be 0 or 1");
    return periodTicks ? 100000000./double(periodTicks)/1024.*(direction ? 1. : -1.) : 0.;
}
class SysfsBoardIo final:public DeviceIo {
public:
    SysfsBoardIo(SysfsNodes& nodes,const HardwareConfig& config):nodes_(nodes),config_(config) {
        config_.validate();if(config_.motor_backend!="sysfs") throw std::runtime_error("Sysfs board requires sysfs profile");
        sysfsPwmDirectory(config_.motor_left_pwm);sysfsPwmDirectory(config_.motor_right_pwm);sysfsPwmDirectory(config_.servo_pwm);
    }
    void setMotorEnable(bool enabled) override {
        prepareMotors();sysfsWriteChecked(nodes_,config_.motor_enable_gpio,enabled ? 1 : 0);
    }
    bool nativeServo() const override {return true;}
    void initializeReferenceServo() {prepareServo();}
    uint32_t writeServoCommand(double adjustedCommand) override {
        const auto duty=newCarServoNs(adjustedCommand,config_.servo_period_ns);
        prepareServo();sysfsWriteChecked(nodes_,config_.servo_pwm,duty);return duty;
    }
    PwmInfo readPwmMetadata(const std::string& path) override {
        if(path==config_.servo_pwm) {
            // Metadata-only before authorization: no export or output operation.
            const auto period=uint32_t(config_.servo_period_ns);
            return {uint32_t(std::llround(1e9/period)),0,period,0,period,1000000000};
        }
        checkMotorPath(path);prepareMotors();
        const auto dir=sysfsPwmDirectory(path);const auto period=sysfsNumber(nodes_.read(dir+"/period"));
        const auto duty=sysfsNumber(nodes_.read(path));
        if(period!=uint32_t(config_.motor_period_ns) || duty>period) throw std::runtime_error("Motor PWM metadata changed: "+path);
        return {uint32_t(std::llround(1e9/period)),duty,period,duty,period,1000000000};
    }
    void writePwmDuty(const std::string& path,uint16_t duty) override {
        checkMotorPath(path);prepareMotors();
        if(duty>config_.motor_period_ns) throw std::runtime_error("Motor duty exceeds period");
        sysfsWriteChecked(nodes_,path,duty);
    }
    void writeGpioLevel(const std::string& path,uint8_t level) override {
        if((path!=config_.motor_left_dir && path!=config_.motor_right_dir) || level>1) throw std::runtime_error("Unconfigured motor direction GPIO");
        prepareMotors();sysfsWriteChecked(nodes_,path,level);
    }
    std::string readText(const std::string& path) override {return nodes_.read(path);}
    void readBinary(const std::string&,void*,size_t) override {throw std::runtime_error("Sysfs board has no factory binary ABI");}
    void writeBinary(const std::string&,const void*,size_t) override {throw std::runtime_error("Sysfs board requires decimal text writes");}
private:
    void checkMotorPath(const std::string& path) const {
        if(path!=config_.motor_left_pwm && path!=config_.motor_right_pwm) throw std::runtime_error("Unconfigured motor PWM: "+path);
    }
    void prepareMotors() {
        if(motorsReady_) return;
        // Match the GPIO constructors in Motor.cpp: export left/right/nSLEEP
        // before PWM initialization; set their output directions afterwards.
        for(const auto& path:{config_.motor_left_dir,config_.motor_right_dir,config_.motor_enable_gpio}) {
            const auto dir=path.substr(0,path.size()-6);
            const auto number=dir.substr(std::string("/sys/class/gpio/gpio").size());
            sysfsExport(nodes_,path,"/sys/class/gpio/export",sysfsNumber(number));
        }
        referenceMotorPwmInit(nodes_,config_.motor_left_pwm,uint32_t(config_.motor_period_ns));
        referenceMotorPwmInit(nodes_,config_.motor_right_pwm,uint32_t(config_.motor_period_ns));
        for(const auto& path:{config_.motor_left_dir,config_.motor_right_dir,config_.motor_enable_gpio}) {
            sysfsGpio(nodes_,path,true,true);sysfsWriteChecked(nodes_,path,1);
        }
        motorsReady_=true;
    }
    void prepareServo() {if(!servoReady_) {referenceServoPwmInit(nodes_,config_.servo_pwm,uint32_t(config_.servo_period_ns));servoReady_=true;}}
    SysfsNodes& nodes_;HardwareConfig config_;bool motorsReady_=false,servoReady_=false;
};
}
