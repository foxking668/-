#include "../tools/rehearsal_servo.hpp"
#include "../tools/parking_rehearsal.hpp"
#include <iostream>
#include <atomic>
#include <mutex>
using namespace car2026;
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool ok,const char* name) {++checks;if(!ok) throw std::runtime_error(name);}
class ServoOnlyIo final:public DeviceIo {
public:
    std::mutex mutex;std::vector<uint16_t> duties;std::atomic<bool> failNext{false};
    void readBinary(const std::string&,void*,size_t) override {throw std::runtime_error("Generic input forbidden");}
    void writeBinary(const std::string&,const void*,size_t) override {throw std::runtime_error("Generic output forbidden");}
    std::string readText(const std::string&) override {throw std::runtime_error("Text input forbidden");}
    PwmInfo readPwmMetadata(const std::string& path) override {
        if(path!=HardwareConfig{}.servo_pwm) throw std::runtime_error("Only servo metadata permitted");
        return {300,4470,10000,1490000,3333333,100000000};
    }
    void writePwmDuty(const std::string& path,uint16_t duty) override {
        if(path!=HardwareConfig{}.servo_pwm) throw std::runtime_error("Motor/GPIO output forbidden");
        std::lock_guard<std::mutex> held(mutex);duties.push_back(duty);
        if(failNext.exchange(false)) throw std::runtime_error("Injected write failure");
    }
    size_t size() {std::lock_guard<std::mutex> held(mutex);return duties.size();}
};
std::deque<ServoHoldEvent> waitEvent(RehearsalServo& servo) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(std::chrono::steady_clock::now()<deadline) {
        auto events=servo.takeEvents();if(!events.empty()) return events;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("Timer did not deliver event");
}
}
int main() {
 try {
    HardwareConfig hardware;Params params;ServoOnlyIo io;
    {
        RehearsalServo servo(io,hardware,params);
        check(io.size()==0,"construct does not write");
        bool rejected=false;try {servo.beginHold(.01,1,1,1);}catch(const std::exception&) {rejected=true;}
        check(rejected,"timer cannot arm before authorized command");
        servo.set(10);std::this_thread::sleep_for(std::chrono::milliseconds(50));
        check(io.size()==1 && servo.takeEvents().empty(),"waiting for push permission does not start timer");
        servo.beginHold(.03,1,2,3);
        auto events=waitEvent(servo);
        check(events.size()==1 && events.front().error.empty(),"timer writes center");
        check(events.front().stage==1 && events.front().revision==2 && events.front().trial==3,"timer preserves stage/config/trial context");
        check(events.front().beginNs<=events.front().endNs,"actual write timestamps");
        check(servo.written()==0 && io.size()==2,"zero command tracked");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));check(io.size()==2,"expired timer does not repeat zero");
        servo.set(-5);servo.beginHold(0,1,1,1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));check(io.size()==3,"hold zero disables timer");
        servo.beginHold(.05,1,1,1);servo.set(5);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));check(io.size()==4 && servo.takeEvents().empty(),"new authorized command cancels old timer");
        servo.beginHold(.03,3,1,1);io.failNext=true;events=waitEvent(servo);
        check(events.size()==1 && !events.front().error.empty(),"automatic zero failure surfaced");
        check(servo.written()==5,"failed zero not reported as successful");
        servo.close();const auto size=io.size();servo.close();check(io.size()==size,"close idempotent");
        check(io.duties.back()==4470,"exit centers configured PWM");
    }
    {
        ServoOnlyIo feedback;RehearsalServo servo(feedback,hardware,params);bool rejected=false;
        try {servo.adjust(3);}catch(...) {rejected=true;}
        check(rejected && feedback.size()==0,"feedback cannot create unapproved servo trial");
        // Leave scheduling margin for debug/CI load; still verify feedback cannot extend expiry.
        servo.set(0);servo.beginHold(.5,0,1,1);
        for(int i=0;i<3;++i) {std::this_thread::sleep_for(std::chrono::milliseconds(15));servo.adjust(i+1);}
        const auto events=waitEvent(servo);
        check(events.size()==1 && servo.written()==0,"feedback preserves original servo deadline");
        rejected=false;try {servo.adjust(3);}catch(...) {rejected=true;}
        check(rejected,"expired servo feedback cannot reapply nonzero command");
    }
    {
        ServoOnlyIo idle;RehearsalServo servo(idle,hardware,params);servo.close();
        check(idle.size()==0,"exit before authorization never writes");
    }
    {
        ServoOnlyIo centered;RehearsalServo servo(centered,hardware,params);servo.set(0);
        check(servo.beginHold(.02,0,1,1).has_value(),"zero command also starts configured positive timer");
        const auto events=waitEvent(servo);
        check(events.size()==1 && events.front().error.empty() && servo.written()==0,"timed straight trial emits completion while remaining centered");
    }
    {
        ServoOnlyIo cancelled;RehearsalServo servo(cancelled,hardware,params);servo.set(10);servo.beginHold(.08,1,1,1);servo.close();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        check(cancelled.size()==2,"exit cancels pending timer before cleanup zero");
    }
    for(int stageIndex:{1,3}) {
        const double primary=stageIndex==1 ? -10. : 12.,correction=stageIndex==1 ? 5. : -5.;
        ParkingTuning tuning;tuning.stage=stageIndex+1;tuning.stages[size_t(stageIndex)]={primary,.5,.03};
        tuning.stages[size_t(stageIndex)].correction={correction,.02};
        ParkingRehearsal trial(tuning);ServoOnlyIo pairIo;RehearsalServo servo(pairIo,hardware,params);
        const auto primaryRequest=trial.update(0,RehearsalKey::Enter);const auto primaryDuty=servo.set(*primaryRequest);trial.acknowledge(true,0);
        trial.update(.5,RehearsalKey::None);trial.update(.51,RehearsalKey::Enter);
        servo.beginHold(trial.holdSeconds(),trial.stage(),trial.revision(),trial.trial(),trial.segment());
        auto events=waitEvent(servo);
        check(events.front().error.empty() && events.front().stage==stageIndex && events.front().segment==0 && pairIo.size()==2,"primary deadline centers and retains stage/segment tags");
        trial.holdCompleted(events.front().stage,events.front().trial,events.front().segment);
        const auto correctionRequest=trial.update(1,RehearsalKey::None);
        check(correctionRequest==correction && !trial.finished(),"successful primary deadline authorizes configured opposite correction");
        const auto correctionDuty=servo.set(*correctionRequest);trial.acknowledge(true,1);
        servo.beginHold(trial.holdSeconds(),trial.stage(),trial.revision(),trial.trial(),trial.segment());
        events=waitEvent(servo);
        check(events.front().stage==stageIndex && events.front().segment==1 && events.front().error.empty(),"correction deadline has its own stage/segment tags");
        trial.holdCompleted(events.front().stage,events.front().trial,events.front().segment);
        check(trial.finished() && servo.written()==0 && pairIo.size()==4,"real timer and model complete both bends without motor access");
        check(pairIo.duties==std::vector<uint16_t>{uint16_t(primaryDuty),4470,uint16_t(correctionDuty),4470},"actual duty writes follow primary/zero/correction/zero order");
    }
    for(int stageIndex:{1,3}) {
        const double primary=stageIndex==1 ? -10. : 12.;
        ParkingTuning tuning;tuning.stage=stageIndex+1;tuning.stages[size_t(stageIndex)]={primary,.5,.02};
        tuning.stages[size_t(stageIndex)].correction={stageIndex==1 ? 5. : -5.,.02};
        ParkingRehearsal trial(tuning);ServoOnlyIo pairIo;RehearsalServo servo(pairIo,hardware,params);
        const auto request=trial.update(0,RehearsalKey::Enter);servo.set(*request);trial.acknowledge(true,0);
        trial.update(.5,RehearsalKey::None);trial.update(.51,RehearsalKey::Enter);
        pairIo.failNext=true;servo.beginHold(trial.holdSeconds(),trial.stage(),trial.revision(),trial.trial(),trial.segment());
        const auto events=waitEvent(servo);
        check(!events.front().error.empty() && events.front().stage==stageIndex && events.front().segment==0,"each bend transition zero failure is observable");
        trial.stop("AUTO_CENTER_FAILED");
        check(!trial.update(1,RehearsalKey::None) && !trial.pendingCorrection() && servo.written()==primary,"failed transition cannot trigger opposite correction");
    }
    std::cout<<checks<<" timed servo checks passed\n";return 0;
 } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
