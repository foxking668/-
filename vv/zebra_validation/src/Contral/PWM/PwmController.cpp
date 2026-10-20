#include "PwmController.hpp"
#include "ZebraTest/SysfsIo.hpp"
#include <chrono>
#include <climits>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

namespace Contral {
using ZebraTest::IoStatus::set;
using ZebraTest::SysfsIo::writeValue;
PwmController::PwmController(int chip, int channel, const std::string& root)
    : path_(root+"/pwmchip"+std::to_string(chip)+"/pwm"+std::to_string(channel)+"/"),
      exportPath_(root+"/pwmchip"+std::to_string(chip)+"/export"), chip_(chip), channel_(channel) {}
PwmController::~PwmController() { if (initialized_) disable(); }

bool PwmController::readState(int& period, int& duty, int& enabled) {
    int* values[] = {&period, &duty, &enabled};
    const char* names[] = {"period", "duty_cycle", "enable"};
    for (int i=0; i<3; ++i) {
        errno=0;
        // Require existing read/write nodes, but do not modify them during a readiness check.
        std::fstream node(path_+names[i], std::ios::in|std::ios::out);
        if (!node.is_open()) { set("PWM_NODE_OPEN",path_+names[i],errno?errno:EIO); return false; }
        long long value=-1;
        if (!(node>>value) || value<0 || value>INT_MAX) {
            set("PWM_METADATA",path_+names[i],EINVAL); return false;
        }
        node>>std::ws;
        if (!node.eof()) { set("PWM_METADATA",path_+names[i],EINVAL); return false; }
        *values[i]=static_cast<int>(value);
    }
    if (duty>period || enabled>1 || (period==0 && enabled!=0)) { set("PWM_METADATA",path_,ERANGE); return false; }
    return true;
}

bool PwmController::initialize() {
    if (chip_<0 || channel_<0) { set("PWM_CHANNEL",path_,EINVAL); return false; }
    int enabled=0;
    if (readState(period_,duty_,enabled)) {
        initialized_=true;
        std::cout<<"[PWM_INIT] channel="<<path_<<" mode=reused"<<std::endl;
        return true;
    }
    std::error_code error;
    const bool exists=std::filesystem::is_directory(path_,error);
    if (error && error!=std::errc::no_such_file_or_directory) {
        set("PWM_DIRECTORY",path_,error.value()); return false;
    }
    std::string exportError;
    if (!exists && !writeValue(exportPath_,std::to_string(channel_)))
        exportError=ZebraTest::IoStatus::lastError;
    // A competing exporter or delayed sysfs nodes are accepted ONLY after all nodes validate.
    for (int attempt=0; attempt<50; ++attempt) {
        if (readState(period_,duty_,enabled)) {
            initialized_=true;
            std::cout<<"[PWM_INIT] channel="<<path_<<" mode="<<(exists||!exportError.empty()?"reused":"exported")<<std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!exportError.empty()) ZebraTest::IoStatus::lastError += " export_error={"+exportError+"}";
    return false;
}

bool PwmController::verify(int expectedEnable) {
    int period=0,duty=0,enabled=0;
    if (!readState(period,duty,enabled)) return false;
    if (period!=period_ || duty!=duty_ || enabled!=expectedEnable) {
        set("PWM_READBACK",path_,EIO);
        ZebraTest::IoStatus::lastError += " expected="+std::to_string(period_)+"/"+std::to_string(duty_)+"/"+std::to_string(expectedEnable)
            +" actual="+std::to_string(period)+"/"+std::to_string(duty)+"/"+std::to_string(enabled);
        return false;
    }
    return true;
}
bool PwmController::prepare(unsigned int periodNs,unsigned int dutyNs) {
    if (periodNs==0 || periodNs>INT_MAX || dutyNs>periodNs) { set("PWM_CONFIG_RANGE",path_,ERANGE); return false; }
    if (!initialize()) return false;
    // Fresh exports can be 0/0/disabled. Linux rejects even enable=0 or duty=0
    // while period is zero. Establish a period in this verified inactive case first.
    if (period_==0 && !setPeriod(periodNs)) return false;
    return disable() && setDutyCycle(0) && setPeriod(periodNs) && setDutyCycle(dutyNs) && verify(0);
}
bool PwmController::enable() { return writeValue(path_+"enable","1") && verify(1); }
bool PwmController::disable() { return writeValue(path_+"enable","0") && verify(0); }
bool PwmController::setPeriod(unsigned int value) {
    if (value==0 || value>INT_MAX || duty_>static_cast<int>(value)) { set("PWM_PERIOD_RANGE",path_+"period",ERANGE); return false; }
    if (!writeValue(path_+"period",std::to_string(value))) return false;
    period_=static_cast<int>(value); return true;
}
bool PwmController::setDutyCycle(unsigned int value) {
    if (period_<=0 || value>static_cast<unsigned int>(period_)) { set("PWM_DUTY_RANGE",path_+"duty_cycle",ERANGE); return false; }
    if (!writeValue(path_+"duty_cycle",std::to_string(value))) return false;
    duty_=static_cast<int>(value); return true;
}
}
