#include "servo_output.hpp"
#include "../tools/manual_capture_options.hpp"
#include <functional>
#include <iostream>
#include <limits>
using namespace car2026;
namespace {
int checks=0;
void check(bool condition,const char* message) {
    ++checks;if(!condition) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& action,const char* message) {
    bool failed=false;try {action();}catch(const std::exception&) {failed=true;}
    check(failed,message);
}
class ServoOnlyIo final : public DeviceIo {
public:
    std::vector<uint16_t> duties;int reads=0;bool failNext=false;
    std::string servoPath=HardwareConfig{}.servo_pwm;
    PwmInfo info{300,4470,10000,1490000,3333333,100000000};
    void readBinary(const std::string&,void*,size_t) override {throw std::runtime_error("Unexpected generic read");}
    void writeBinary(const std::string&,const void*,size_t) override {throw std::runtime_error("Unexpected generic write");}
    std::string readText(const std::string&) override {throw std::runtime_error("Unexpected text read");}
    PwmInfo readPwmMetadata(const std::string& path) override {
        check(path==servoPath,"Only servo metadata is accessed");++reads;return info;
    }
    void writePwmDuty(const std::string& path,uint16_t duty) override {
        check(path==servoPath,"Only servo PWM is written; motors are forbidden");
        duties.push_back(duty);
        if(failNext) {failNext=false;throw std::runtime_error("Injected servo write failure");}
    }
};
}
int main() {
    try {
        using capture::Options;
        const auto plain=Options::parse({});
        check(!plain.steerCommand && plain.duration==180,"Existing capture remains actuator-free by default");
        const auto zero=Options::parse({"--steer-command","0","--duration","25","--allow-partial"});
        check(zero.steerCommand.has_value() && *zero.steerCommand==0,"Explicit zero is recognized as servo output mode");
        check(zero.allowPartial && zero.duration==25,"Existing capture flags remain available");
        check(*Options::parse({"--steer-command","-5","--duration","25"}).steerCommand==-5,"Signed steering command accepted");
        check(Options::parse({"--check-config","--steer-command","5","--duration","25"}).check,"Config-check mode retains explicit steering validation");
        rejects([]{Options::parse({"--steer-command","16","--duration","25"});},"Hard command cap rejects larger steering");
        rejects([]{Options::parse({"--steer-command","nan","--duration","25"});},"Nonfinite steering rejected");
        rejects([]{Options::parse({"--steer-command","5"});},"Explicit bounded duration required");
        rejects([]{Options::parse({"--steer-command","5","--duration","61"});},"Steering duration above 60 rejected");
        rejects([]{Options::parse({"--duration","0"});},"Zero duration rejected");
        rejects([]{Options::parse({"--steer-command","0","--steer-command","5","--duration","25"});},"Duplicate steering request rejected");
        rejects([]{Options::parse({"--vehicle-config","some.ini"});},"Vehicle config cannot implicitly enable steering");
        rejects([]{Options::parse({"--check-output","--steer-command","0","--duration","25"});},"Output-only check cannot silently swallow steering mode");
        rejects([]{Options::parse({"--check-output","--check-config"});},"Conflicting checks rejected");
        const auto observed=Options::parse({"--observe-steering","reverse","--duration","25","--vehicle-config","car.ini"});
        check(observed.observeSteering==ManualMotion::Reverse && !observed.steerCommand,
              "observation accepts vehicle config without enabling any servo command");
        rejects([]{Options::parse({"--observe-steering","forward","--steer-command","0","--duration","25"});},
                "even zero steering output is forbidden in observation mode");
        rejects([]{Options::parse({"--observe-steering","automatic","--duration","25"});},"observer does not guess motion direction");
        rejects([]{Options::parse({"--observe-steering","forward"});},"observer requires explicit bounded duration");
        rejects([]{Options::parse({"--observe-steering","forward","--duration","25","--check-output"});},
                "output-only check cannot silently ignore observer options");
        rejects([]{Options::parse({"--motor-pulse"});},"Motor operation is not part of capture CLI");
        rejects([]{Options::parse({"--steer-command"});},"Missing command value rejected");
        HardwareConfig config;Params params;params.max_steer_deg=15;
        {
            ServoOnlyIo io;
            {ServoOutput servo(io,config,params);check(io.duties.empty(),"Construction does not move servo");}
            check(io.duties.empty(),"No shutdown write if no command was attempted");
        }
        {
            ServoOnlyIo io;
            {
                ServoOutput servo(io,config,params);
                check(servo.set(0)==4470,"Existing center PWM mapping preserved");
                check(servo.set(5)==4220,"Existing positive PWM mapping preserved");
                check(servo.set(-5)==4629,"Existing negative PWM mapping preserved");
            }
            check(io.duties.back()==4470 && io.duties.size()==4,"Normal scope exit returns to configured zero");
        }
        {
            ServoOnlyIo io;ServoOutput servo(io,config,params);
            rejects([&]{servo.set(16);},"Configured command limit enforced before writes");
            rejects([&]{servo.set(std::numeric_limits<double>::infinity());},"Nonfinite output rejected before writes");
            check(io.duties.empty(),"Invalid commands cannot alter outputs");
            servo.set(5);servo.close();servo.close();
            check(io.duties.size()==2,"Explicit close is idempotent");
            rejects([&]{servo.set(0);},"Closed output rejects new writes");
        }
        {
            ServoOnlyIo io;
            {ServoOutput servo(io,config,params);io.failNext=true;
             rejects([&]{servo.set(5);},"Command write failure propagates");}
            check(io.duties.size()==2 && io.duties.back()==4470,"Failed command still gets a zero-return attempt");
        }
        {
            ServoOnlyIo io;
            {ServoOutput servo(io,config,params);servo.set(5);io.failNext=true;
             rejects([&]{servo.close();},"Shutdown failure is observable to recorder");}
            check(io.duties.size()==2,"Failed explicit shutdown is not silently retried in destructor");
        }
        {
            ServoOnlyIo io;auto invalid=config;invalid.servo_pwm=invalid.motor_left_pwm;
            rejects([&]{ServoOutput servo(io,invalid,params);},"Aliased servo/motor path rejected before device access");
            check(io.reads==0 && io.duties.empty(),"Invalid hardware config has no device effects");
        }
        std::cout<<"PASS "<<checks<<" manual steering checks (simulated servo; no motor access)\n";return 0;
    }catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
