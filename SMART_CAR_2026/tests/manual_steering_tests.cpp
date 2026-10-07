#include "servo_output.hpp"
#include "../tools/manual_capture_options.hpp"
#include "../tools/manual_steering_probe.hpp"
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
        const auto probing=Options::parse({"--steering-probe","reverse","--duration","45","--vehicle-config","car.ini"});
        check(probing.steeringProbe && !probing.steerCommand && !probing.observeSteering,"Probe is an explicit, exclusive mode");
        rejects([]{Options::parse({"--steering-probe","forward","--duration","45"});},"Probe does not silently select another direction");
        rejects([]{Options::parse({"--steering-probe","reverse"});},"Probe requires explicit bounded duration");
        rejects([]{Options::parse({"--steering-probe","reverse","--duration","61"});},"Probe rejects duration above 60");
        rejects([]{Options::parse({"--steering-probe","reverse","--duration","45","--steer-command","0"});},"Probe conflicts even with fixed zero");
        rejects([]{Options::parse({"--steering-probe","reverse","--duration","45","--observe-steering","reverse"});},"Probe conflicts with no-write observer");
        rejects([]{Options::parse({"--steering-probe","reverse","--duration","45","--check-output"});},"Output-only check cannot discard probe mode");
        check(Options::parse({"--steering-probe","reverse","--duration","45","--check-config"}).check,"Probe config check remains hardware-free");
        {
            using capture::EncoderRestGate;
            EncoderRestGate gate;
            check(!gate.status(0).fresh,"Missing encoders cannot pass rest gate");
            gate.sample(true,100,true,-100,0);
            check(!gate.status(0).stationary,"Untimed startup counts cannot prove rest");
            for(int i=1;i<=9;++i) gate.sample(true,0,true,0,i*.1);
            check(gate.status(.92).stationary,"Continuous fresh zero deltas permit stationary requests");
            check(!gate.status(1.2).fresh,"Stale encoder stream blocks commands");
            gate.sample(true,0,true,0,1.3);
            check(!gate.status(1.3).stationary,"Sample gap restarts full rest interval");
            gate.sample(true,1,true,-1,1.4);
            check(!gate.status(1.4).stationary,"Opposing nonzero deltas cannot cancel into rest");
            gate.sample(false,0,true,0,1.5);
            check(!gate.status(1.5).fresh,"Read failure cannot be interpreted as zero");
            gate.sample(true,0,true,0,1.6);gate.sample(true,0,true,0,1.6);
            check(!gate.status(1.6).fresh,"Repeated encoder timestamp invalidates rest");
            gate.sample(true,0,true,std::numeric_limits<double>::quiet_NaN(),1.7);
            check(!gate.status(1.7).fresh,"NaN count invalidates rest");
            gate.sample(true,0,true,0,1.8);
            check(!gate.status(1.7).fresh,"Future encoder sample rejected");
        }
        {
            capture::ProbeInput keys;keys.add("+");keys.add("q");
            check(keys.quit,"Queued quit wins before actuator writes");
            capture::ProbeInput conflict;conflict.add("R");conflict.add("+");
            check(conflict.key==capture::ProbeKey::Conflict,"Multiple control keys in one batch are rejected");
            capture::ProbeInput mark;mark.add("A");
            check(mark.key==capture::ProbeKey::None && !mark.quit,"Legacy markers cannot actuate");
        }
        {
            using capture::ProbeKey;capture::ManualSteeringProbe probe;
            const capture::EncoderRestStatus rest{true,true,0},moving{true,false,0},lost{};
            SteeringObservation image;image.state="TRACKING";image.hasReference=true;image.hasSuggestion=true;
            image.referenceId=1;image.suggestedCommand=-5; // Deliberately conflicts with keyboard +2.
            check(!probe.update(ProbeKey::Positive,rest,image,0).command,"Image reference alone cannot arm servo");
            check(!probe.update(ProbeKey::Reference,moving,image,.1).command,"Manual reference rejected while moving");
            const auto arm=probe.update(ProbeKey::Reference,rest,image,.2);
            check(arm.command==0 && arm.resetReference,"Stopped R centers before acquiring a new reference");
            SteeringObservation waiting;
            check(!probe.update(ProbeKey::None,rest,waiting,.3).command,"Reference confirmation does not repeatedly write servo");
            check(probe.update(ProbeKey::None,rest,image,.4).decision=="PROBE_REFERENCE_READY","Valid stationary reference arms keyboard commands");
            check(probe.update(ProbeKey::Positive,rest,image,.5).command==2,"Only explicit +2 is requested; image candidate -5 ignored");
            check(!probe.update(ProbeKey::Negative,moving,image,.6).command,"Cannot change direction while manually moving");
            check(!probe.update(ProbeKey::None,moving,image,.7).command,"Valid moving response recording holds fixed command");
            check(probe.update(ProbeKey::Negative,rest,image,.8).command==-2,"Explicit stopped negative request stays bounded");
            image.state="DISCONTINUOUS";image.hasSuggestion=false;
            check(probe.update(ProbeKey::None,moving,image,.9).state=="HOLD","Visual loss latches hold");
            image.state="TRACKING";image.hasSuggestion=true;
            check(!probe.update(ProbeKey::Positive,moving,image,1).command,"Visual recovery cannot auto-resume steering");
            const auto recover=probe.update(ProbeKey::Positive,rest,image,1.1);
            check(recover.command==0 && recover.state=="UNARMED","After loss, rest permits zero only, never queued +2");
            check(!probe.update(ProbeKey::Positive,rest,image,1.2).command,"Rest recovery still requires explicit R");
            probe.update(ProbeKey::Reference,rest,image,1.3);probe.update(ProbeKey::None,rest,image,1.4);
            check(!probe.update(ProbeKey::None,lost,image,1.5).command,"Encoder loss holds command without zeroing while potentially moving");
            check(probe.update(ProbeKey::None,rest,image,1.6).command==0,"Fresh rest after encoder failure allows zero");
            probe.update(ProbeKey::Reference,rest,image,1.7);probe.update(ProbeKey::None,rest,image,1.8);
            image.referenceId=2;
            check(probe.update(ProbeKey::None,moving,image,1.9).state=="HOLD","Reference identity cannot change silently");
            probe.update(ProbeKey::None,rest,image,2);
            probe.update(ProbeKey::Reference,rest,image,2.1);probe.update(ProbeKey::None,rest,image,2.2);
            image.lateralError=.081;
            check(probe.update(ProbeKey::None,moving,image,2.3).state=="HOLD","Image deviation cap latches hold before leaving view");
            probe.update(ProbeKey::None,rest,image,2.4);image.lateralError=0;
            probe.update(ProbeKey::Reference,rest,image,2.5);
            check(probe.update(ProbeKey::None,moving,image,2.6).state=="HOLD","Moving before reference is ready cancels setup");
            check(!probe.update(ProbeKey::None,rest,image,2.6).command,"Repeated processing clock cannot write servo");
        }
        {
            using capture::ProbeKey;capture::ManualSteeringProbe probe;
            const capture::EncoderRestStatus rest{true,true,0};SteeringObservation image;
            check(!probe.update(ProbeKey::Stop,rest,image,0).command,"Stop before R cannot implicitly initialize servo");
            probe.update(ProbeKey::None,rest,image,std::numeric_limits<double>::quiet_NaN());
            check(!probe.update(ProbeKey::None,rest,image,.1).command,"Startup clock fault recovery cannot initialize servo");
        }
        {
            // Independent small-angle bicycle model: metric position feedback
            // keeps its sign in reverse; yaw damping changes sign with velocity.
            auto simulate=[](double velocity,bool wrongWholeSign) {
                double lateral=.05,yaw=.08;const double dt=.01;
                for(int i=0;i<2500;++i) {
                    const double direction=velocity>0 ? 1 : -1;
                    const double delta=clamp(wrongWholeSign ? -direction*(4*lateral+2*yaw) :
                                                             -4*lateral-direction*2*yaw,-.2,.2);
                    lateral+=dt*velocity*yaw;yaw+=dt*velocity*delta/.22;
                }
                return std::hypot(lateral,yaw);
            };
            check(simulate(.1,false)<.005,"Metric forward model converges with position and yaw feedback");
            check(simulate(-.1,false)<.005,"Metric reverse model converges without reversing position feedback");
            check(simulate(-.1,true)>.09,"Whole-feedback reversal fails independent metric reverse model");
            // Pitched front-camera projection: far-minus-near is not yaw alone.
            auto projected=[](double lateral,double yaw,double distance) {
                const double pitch=.6,height=.175,cameraAhead=.23;
                return -(lateral+cameraAhead*yaw+distance*yaw)/(std::cos(pitch)*distance+std::sin(pitch)*height);
            };
            const double offsetFeature=projected(.01,0,.8)-projected(.01,0,.2);
            const double yawFeature=projected(0,.01,.8)-projected(0,.01,.2);
            check(std::abs(offsetFeature)>.001 && std::abs(yawFeature)>.001,
                  "Image heading feature mixes lateral offset and orientation in independent camera model");
        }
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
        {
            using capture::ProbeKey;
            ServoOnlyIo io;ServoOutput servo(io,config,params);capture::ManualSteeringProbe probe;
            const capture::EncoderRestStatus rest{true,true,0},moving{true,false,0};
            SteeringObservation image;image.state="TRACKING";image.hasReference=image.hasSuggestion=true;
            image.referenceId=1;image.suggestedCommand=5;
            auto apply=[&](ProbeKey key,const capture::EncoderRestStatus& gate,double time) {
                const auto action=probe.update(key,gate,image,time);
                if(action.command) servo.set(*action.command);
            };
            apply(ProbeKey::None,rest,0);
            check(io.duties.empty(),"Integrated probe cannot actuate before keyboard R");
            apply(ProbeKey::Reference,rest,.1);apply(ProbeKey::None,rest,.2);
            apply(ProbeKey::Positive,rest,.3);apply(ProbeKey::Negative,moving,.4);
            check(io.duties==std::vector<uint16_t>({4470,4370}),"Integrated probe outputs only configured zero/+2, ignores moving -2 and image +5");
            image.hasSuggestion=false;apply(ProbeKey::None,moving,.5);
            check(io.duties.size()==2,"Integrated visual loss holds last PWM while potentially moving");
            apply(ProbeKey::None,rest,.6);
            check(io.duties.back()==4470 && io.duties.size()==3,"Integrated recovery writes zero only after rest");
            image.hasSuggestion=true;apply(ProbeKey::Reference,rest,.7);apply(ProbeKey::None,rest,.8);
            apply(ProbeKey::Negative,rest,.9);
            check(io.duties.back()==4534,"Integrated -2 uses existing asymmetric factory PWM mapping");
            servo.close();check(io.duties.back()==4470,"Integrated explicit exit returns configured zero");
        }
        std::cout<<"PASS "<<checks<<" manual steering checks (simulated servo; no motor access)\n";return 0;
    }catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
