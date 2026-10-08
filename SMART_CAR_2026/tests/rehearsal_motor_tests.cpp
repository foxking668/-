#include "../tools/rehearsal_motor.hpp"
#include "../tools/rehearsal_timing.hpp"
#include "../tools/parking_rehearsal.hpp"
#include <iostream>
using namespace car2026;
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool ok,const char* name) {++checks;if(!ok) throw std::runtime_error(name);}
template<class F> void rejects(F action,const char* name) {bool failed=false;try {action();}catch(...) {failed=true;}check(failed,name);}
class MotorOnlyIo final:public DeviceIo {
public:
    std::mutex mutex;std::vector<std::pair<std::string,int>> writes;bool failLeft=false;int failedRightZeros=0;
    void readBinary(const std::string&,void*,size_t) override {throw std::runtime_error("No sensor reads permitted");}
    void writeBinary(const std::string&,const void*,size_t) override {throw std::runtime_error("Typed output required");}
    std::string readText(const std::string&) override {throw std::runtime_error("No text reads permitted");}
    PwmInfo readPwmMetadata(const std::string& path) override {
        if(path!=HardwareConfig{}.motor_left_pwm && path!=HardwareConfig{}.motor_right_pwm) throw std::runtime_error("No unrelated metadata");
        return {10000,0,10000,0,100000,100000000};
    }
    void writePwmDuty(const std::string& path,uint16_t duty) override {
        if(path!=HardwareConfig{}.motor_left_pwm && path!=HardwareConfig{}.motor_right_pwm) throw std::runtime_error("No unrelated PWM");
        std::lock_guard<std::mutex> held(mutex);writes.emplace_back(path,duty);
        if(path==HardwareConfig{}.motor_left_pwm && failLeft) throw std::runtime_error("Injected left failure");
        if(path==HardwareConfig{}.motor_right_pwm && duty==0 && failLeft) ++failedRightZeros;
    }
    void writeGpioLevel(const std::string& path,uint8_t value) override {
        if(path!=HardwareConfig{}.motor_left_dir && path!=HardwareConfig{}.motor_right_dir) throw std::runtime_error("No unrelated GPIO");
        std::lock_guard<std::mutex> held(mutex);writes.emplace_back(path,value);
    }
    bool has(const std::string& path,int value) {std::lock_guard<std::mutex> held(mutex);for(const auto& write:writes) if(write.first==path && write.second==value) return true;return false;}
};
std::deque<MotorEvent> waitStop(RehearsalMotor& motor) {
    auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(std::chrono::steady_clock::now()<until) {
        auto events=motor.takeEvents();for(const auto& event:events) if(event.event!="MOTOR_START") return events;
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    throw std::runtime_error("No motor stop event");
}
}
int main() {
 try {
    HardwareConfig hardware;Params params;
    {
        MotorOnlyIo io;RehearsalMotor motor(io,hardware,params);
        check(io.writes.size()==2,"initialization only zeros both motors, no direction output or encoder reads");
        motor.start(2000,2000,.03,0,1,1);auto start=motor.takeEvents();
        check(start.size()==1 && start.front().event=="MOTOR_START" && start.front().left==2000,"actual start event");
        check(io.has(hardware.motor_left_pwm,400) && io.has(hardware.motor_right_pwm,400),"2000 maps to four percent of PWM duty_max");
        const auto stop=waitStop(motor);
        check(stop.back().event=="MOTOR_STOP_DURATION" && stop.back().error.empty() && !motor.active(),"independent deadline stops both motors without main loop");
        check(stop.back().endNs>=start.front().endNs,"actual stop timestamps");
        motor.start(-2000,-2000,.03,1,2,2);motor.takeEvents();motor.stop("MOTOR_STOP_PAUSE");
        check(io.has(hardware.motor_left_dir,1-int(hardware.motor_left_forward_level)),"reverse level derived from configured polarity");
        check(motor.takeEvents().back().event=="MOTOR_STOP_PAUSE" && !motor.active(),"pause stops without restart");
        rejects([&] {motor.start(2000,2000,1,1,1,1);},"wrong stage direction rejected");
        rejects([&] {motor.start(12001,2000,1,0,1,1);},"oversized output rejected rather than clipped");
        rejects([&] {motor.start(2000,2000,0,0,1,1);},"zero duration cannot arm motor");
        rejects([&] {motor.start(-2000,-2000,1,5,1,1);},"stop stage cannot move");
        motor.close();const auto n=io.writes.size();motor.close();check(n==io.writes.size(),"close idempotent");
    }
    {
        MotorOnlyIo io;auto shortWatch=params;shortWatch.frame_timeout_s=.1;
        RehearsalMotor motor(io,hardware,shortWatch);motor.start(2000,2000,1,0,1,1);motor.takeEvents();
        const auto events=waitStop(motor);check(events.back().event=="MOTOR_STOP_WATCHDOG","stalled capture heartbeat forces zero before long duration");
    }
    {
        MotorOnlyIo io;auto shortWatch=params;shortWatch.frame_timeout_s=.1;
        RehearsalMotor motor(io,hardware,shortWatch);motor.start(2000,2000,.18,0,1,1);motor.takeEvents();
        for(int i=0;i<6;++i) {std::this_thread::sleep_for(std::chrono::milliseconds(30));motor.heartbeat();}
        auto events=motor.takeEvents();if(events.empty()) events=waitStop(motor);
        check(events.back().event=="MOTOR_STOP_DURATION","valid heartbeat cannot extend absolute motion deadline");
    }
    {
        MotorOnlyIo io;RehearsalMotor motor(io,hardware,params);io.failLeft=true;
        rejects([&] {motor.start(2000,2000,1,0,1,1);},"start failure surfaced");
        check(io.failedRightZeros>0,"left failure still attempts right zero");
        rejects([&] {motor.start(2000,2000,1,0,1,1);},"failed output latched, no silent retry");
    }
    ParkingTuning tuning;tuning.stage=1;tuning.stages[0].motorLeft=tuning.stages[0].motorRight=2000;
    tuning.validate();check(!tuning.stages[0].powered(),"2000 with time zero is inert");
    tuning.stages[0].motorSeconds=1;tuning.validate();
    auto bad=tuning;bad.single=false;rejects([&] {bad.validate();},"powered full sequence rejected during commissioning");
    rejects([&] {tuning.validate(15,1999);},"actual vehicle PWM cap enforced");
    bad=tuning;bad.stages[5].motorLeft=2000;rejects([&] {bad.validate();},"stage6 remains all zero");
    const auto delivered=ParkingTuning::load("deploy/config/parking_tuning.ini");
    check(delivered.stages[4].motorRight==2000 && delivered.stages[5].motorRight==0,"all five motion stages use 2000, final stage remains zero");
    auto partialSource=delivered.source;const auto keyPosition=partialSource.find("motor_left_command=2000");
    partialSource.erase(keyPosition,std::string("motor_left_command=2000\n").size());
    rejects([&] {ParkingTuning::parse(partialSource);},"partial motor triplet rejected");
    for(double value:{-1.,12001.,std::numeric_limits<double>::quiet_NaN()}) {
        bad=tuning;bad.stages[0].motorLeft=value;rejects([&] {bad.validate();},"invalid command rejected");
    }
    bad=tuning;bad.stages[0].motorSeconds=121;rejects([&] {bad.validate();},"oversized motor duration rejected");
    ParkingRehearsal model(tuning);model.update(0,RehearsalKey::Enter);model.acknowledge(true,0);model.update(.5,RehearsalKey::None);
    model.update(.6,RehearsalKey::Pause);check(model.motorRestartRequired() && model.state()==ParkingRehearsal::State::AwaitApply,"P requires new motion authorization");
    model.update(.7,RehearsalKey::Continue);check(model.recordingEnabled() && model.state()==ParkingRehearsal::State::AwaitApply,"C only resumes capture");
    auto changed=tuning;changed.stages[0].motorLeft=2100;model.applySaved(changed,.8);model.acknowledge(true,.8);
    model.update(2,RehearsalKey::None);check(model.state()==ParkingRehearsal::State::AwaitApply,"file save cannot bypass paused motor restart latch");
    check(model.update(2.1,RehearsalKey::Enter).has_value(),"fresh Enter authorizes restart");model.acknowledge(true,2.1);model.update(2.6,RehearsalKey::None);
    check(model.state()==ParkingRehearsal::State::Running && !model.motorRestartRequired(),"restart observes servo settle");
    changed.stages[0].motorRight=2200;check(model.applySaved(changed,3).has_value(),"motor-only file change authorizes new trial");model.acknowledge(true,3);
    check(model.state()==ParkingRehearsal::State::Settling,"powered file save always settles rather than immediate drive");
    model.update(3.5,RehearsalKey::None);
    SavedTuningWatcher watcher(tuning,15,12000);auto watched=delivered.source;
    watched.replace(watched.find("motor_run_time_s=0"),std::string("motor_run_time_s=0").size(),"motor_run_time_s=1");
    watcher.observe(watched,0);check(watcher.observe(watched,.4).has_value(),"watcher detects motor duration change");
    ParkingRehearsal pausedLegacy(ParkingTuning{});pausedLegacy.update(0,RehearsalKey::Pause);
    auto powered=ParkingTuning{};powered.stages[1].motorLeft=2000;powered.stages[1].motorSeconds=1;
    pausedLegacy.applySaved(powered,.1);pausedLegacy.acknowledge(true,.1);
    check(pausedLegacy.motorRestartRequired(),"enabling motors during legacy pause does not start motion");
    RehearsalTiming timing("delta");timing.begin(0,1,1,int64_t(1e9));timing.motorStart(int64_t(1.1e9));
    const auto sample=[&](double seconds,double left,double right,bool valid=true) {return timing.add({valid,left,int64_t(seconds*1e9)},{valid,right,int64_t(seconds*1e9)});};
    sample(1.2,999,999);sample(1.4,3,-3);sample(1.6,0,0);timing.motorStop(int64_t(1.7e9),"MOTOR_STOP_DURATION");
    sample(1.8,0,0);sample(2,0,0);sample(2.4,0,0);
    check(timing.stationaryAfter(int64_t(1.7e9)),"valid zero increments after stop form half-second tail");
    sample(2.5,0,0,false);check(!timing.stationaryAfter(int64_t(1.7e9)),"invalid sample is never proof of stationary vehicle");
    timing.end(int64_t(10e9),"EXIT");check(timing.rows().front().end==int64_t(1.7e9),"stationary tail and save waiting cannot extend stage duration");
    check(timing.rows().front().firstMotion==int64_t(1.4e9),"first encoder read discarded as unknown preceding interval");
    RehearsalTiming cumulative("cumulative32");cumulative.begin(0,1,1,0);
    cumulative.add({true,100,int64_t(1e9)},{true,100,int64_t(1e9)});
    cumulative.add({true,100,int64_t(1.3e9)},{true,100,int64_t(1.3e9)});
    cumulative.add({true,100,int64_t(1.6e9)},{true,100,int64_t(1.6e9)});
    check(cumulative.stationaryAfter(1),"unchanged cumulative counts count as stationary");
    std::ostringstream out;timing.writeSummary(out,0);check(out.str().find("stage_duration_s")!=std::string::npos,"timing CSV declares measured scope");
    std::cout<<checks<<" motor/timing checks passed\n";return 0;
 } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
