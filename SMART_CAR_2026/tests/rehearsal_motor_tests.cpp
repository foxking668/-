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
    uint32_t leftMax=10000,rightMax=10000;
    bool failRightNonzero=false;
    void readBinary(const std::string&,void*,size_t) override {throw std::runtime_error("No sensor reads permitted");}
    void writeBinary(const std::string&,const void*,size_t) override {throw std::runtime_error("Typed output required");}
    std::string readText(const std::string&) override {throw std::runtime_error("No text reads permitted");}
    PwmInfo readPwmMetadata(const std::string& path) override {
        if(path!=HardwareConfig{}.motor_left_pwm && path!=HardwareConfig{}.motor_right_pwm) throw std::runtime_error("No unrelated metadata");
        return {17000,0,path==HardwareConfig{}.motor_left_pwm ? leftMax : rightMax,0,58824,100000000};
    }
    void writePwmDuty(const std::string& path,uint16_t duty) override {
        if(path!=HardwareConfig{}.motor_left_pwm && path!=HardwareConfig{}.motor_right_pwm) throw std::runtime_error("No unrelated PWM");
        std::lock_guard<std::mutex> held(mutex);writes.emplace_back(path,duty);
        if(path==HardwareConfig{}.motor_left_pwm && failLeft) throw std::runtime_error("Injected left failure");
        if(path==HardwareConfig{}.motor_right_pwm && duty>0 && failRightNonzero) throw std::runtime_error("Injected right nonzero failure");
        if(path==HardwareConfig{}.motor_right_pwm && duty==0 && failLeft) ++failedRightZeros;
    }
    void writeGpioLevel(const std::string& path,uint8_t value) override {
        if(path!=HardwareConfig{}.motor_left_dir && path!=HardwareConfig{}.motor_right_dir) throw std::runtime_error("No unrelated GPIO");
        std::lock_guard<std::mutex> held(mutex);writes.emplace_back(path,value);
    }
    bool has(const std::string& path,int value) {std::lock_guard<std::mutex> held(mutex);for(const auto& write:writes) if(write.first==path && write.second==value) return true;return false;}
};
std::deque<MotorEvent> waitStop(RehearsalMotor& motor) {
    std::deque<MotorEvent> collected;
    auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(std::chrono::steady_clock::now()<until) {
        bool stopped=false;
        for(const auto& event:motor.takeEvents()) {collected.push_back(event);if(event.event.rfind("MOTOR_STOP_",0)==0) stopped=true;}
        if(stopped) return collected;
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    throw std::runtime_error("No motor stop event");
}
}
int main() {
 try {
    HardwareConfig hardware;Params params;
    {
        MotorOnlyIo io;RehearsalMotor motor(io,hardware,params);int updates=0;
        motor.setSpeedController([&](double left,double right) {
            ++updates;if(left!=1.25 || right!=2.5) throw std::runtime_error("Feedback mismatch");
            return std::array<int,2>{3000,4000};
        });
        rejects([&]{motor.start(0,0,.16,1,1,1,true);},"speed start requires fresh encoder pair");
        motor.submitSpeedFeedback(1.25,2.5);
        motor.start(0,0,.16,1,1,1,true);const auto start=motor.takeEvents().front();
        const auto events=waitStop(motor);int samples=0;
        for(const auto& event:events) if(event.event=="MOTOR_PID_UPDATE") {
            ++samples;check(event.beginNs>=start.endNs+40000000,"PID does not update before control interval");
            check(event.writtenRaw[0]==3000 && event.writtenRaw[1]==4000 && event.measuredRps[0]==1.25,"PID records actual per-wheel PWM and native feedback");
        }
        check(updates>=2 && samples==updates,"worker updates without camera-thread motor writes");
        check(events.back().event=="MOTOR_STOP_DURATION" && !motor.active(),"feedback cannot extend original deadline");
        check(io.writes.back()==std::make_pair(hardware.motor_right_pwm,0),"duration expiry ends with both wheels zero");
        const int endedUpdates=updates;std::this_thread::sleep_for(std::chrono::milliseconds(70));
        check(updates==endedUpdates && motor.outputs()==std::array<int,2>{0,0},"late feedback cannot re-arm stopped motors");
        motor.submitSpeedFeedback(1.25,2.5);
        motor.start(0,0,1,1,1,2,true);motor.stop("MOTOR_STOP_PAUSE");const int pausedUpdates=updates;
        motor.submitSpeedFeedback(5,5);std::this_thread::sleep_for(std::chrono::milliseconds(70));
        check(updates==pausedUpdates && !motor.active(),"paused motor stays stopped after encoder feedback");
    }
    {
        MotorOnlyIo io;RehearsalMotor motor(io,hardware,params);
        motor.setSpeedController([](double,double){return std::array<int,2>{3000,4000};});
        motor.submitSpeedFeedback(0,0);io.failRightNonzero=true;
        motor.start(0,0,1,1,1,1,true);motor.takeEvents();const auto events=waitStop(motor);
        check(events.back().event=="MOTOR_STOP_PID_FAILED" && !events.back().error.empty() && !motor.active(),"PID write failure independently stops the trial");
        check(motor.outputs()==std::array<int,2>{0,0},"both outputs cleared after partial feedback write");
        rejects([&]{motor.start(0,0,1,1,1,2,true);},"failed controller cannot start another powered trial");
    }
    {
        MotorOnlyIo io;RehearsalMotor motor(io,hardware,params);
        check(io.writes.size()==2,"initialization only zeros both motors, no direction output or encoder reads");
        motor.start(2000,2000,.03,0,1,1);auto start=motor.takeEvents();
        check(start.size()==1 && start.front().event=="MOTOR_START" && start.front().left==2000,"actual start event");
        check(io.has(hardware.motor_left_pwm,2000) && io.has(hardware.motor_right_pwm,2000) && !io.has(hardware.motor_left_pwm,400),"2000 writes raw 2000 regardless of normalized scale");
        check(start.front().writtenRaw[0]==2000 && start.front().gpio[0]==0 && start.front().dutyMax[0]==10000 && start.front().frequency[0]==17000,"start records raw write, direction, full scale and frequency");
        check(motorEventDetail(start.front()).find("units=raw_pwm")!=std::string::npos,"shared CSV and terminal diagnostics declare raw units");
        check(motorEventDetail(start.front()).find("right_gpio=0")!=std::string::npos,"diagnostic fields distinguish left and right GPIO");
        const auto stop=waitStop(motor);
        check(stop.back().event=="MOTOR_STOP_DURATION" && stop.back().error.empty() && !motor.active(),"independent deadline stops both motors without main loop");
        check(stop.back().endNs>=start.front().endNs,"actual stop timestamps");
        check(stop.back().writtenRaw[0]==0 && stop.back().writtenRaw[1]==0,"stop event records zero payloads rather than start PWM");
        check(stop.back().motorZeroNs>=start.front().endNs && stop.back().motorZeroNs<=stop.back().endNs,"motor zero timestamp separate from steering return latency");
        int adjustments=0;
        check(!motor.whileActive([&] {++adjustments;}),"expired motor cannot issue late steering");
        motor.setStopAction([&] {++adjustments;});
        motor.start(-2000,-2000,.03,1,2,2);motor.takeEvents();motor.stop("MOTOR_STOP_PAUSE");
        check(io.has(hardware.motor_left_dir,1-int(hardware.motor_left_forward_level)),"reverse level derived from configured polarity");
        check(motor.takeEvents().back().event=="MOTOR_STOP_PAUSE" && !motor.active(),"pause stops without restart");
        check(adjustments==1,"motor stop runs independent centering action");
        rejects([&] {motor.start(2000,2000,1,1,1,1);},"wrong stage direction rejected");
        rejects([&] {motor.start(10001,2000,1,0,1,1);},"device oversized output rejected rather than clipped");
        rejects([&] {motor.start(1999.5,2000,1,0,1,1);},"fractional raw PWM cannot be silently rounded");
        rejects([&] {motor.start(2000,2000,0,0,1,1);},"zero duration cannot arm motor");
        rejects([&] {motor.start(-2000,-2000,1,5,1,1);},"stop stage cannot move");
        motor.close();const auto n=io.writes.size();motor.close();check(n==io.writes.size(),"close idempotent");
    }
    for(uint32_t invalidMax:{0u,65536u}) {
        MotorOnlyIo io;io.rightMax=invalidMax;
        rejects([&] {RehearsalMotor motor(io,hardware,params);},"invalid full scale cannot initialize a motor worker");
        check(io.writes.size()==4 && !io.has(hardware.motor_left_dir,0),"metadata failure clears both outputs without writing direction");
    }
    {
        MotorOnlyIo io;RehearsalMotor motor(io,hardware,params);io.failRightNonzero=true;
        rejects([&] {motor.start(-3000,-4000,1,1,1,1);},"right start failure surfaced after left raw output");
        const auto event=motor.takeEvents().front();
        check(io.has(hardware.motor_left_pwm,3000) && event.writtenRaw[0]==0 && event.writtenRaw[1]==0,"partial start failure clears both wheel outputs");
        check(event.event=="MOTOR_START_FAILED" && event.left==-3000 && event.right==-4000 && !motor.active(),"failed start retains request and never arms timer");
        check(motorEventDetail(event).find("write_result=failed")!=std::string::npos,"failed start is not presented as successful output");
    }
    {
        MotorOnlyIo io;io.leftMax=20000;io.rightMax=16000;
        auto smallLegacy=params;smallLegacy.pwm_limit=100;
        RehearsalMotor motor(io,hardware,smallLegacy);
        motor.start(15000,13000,.03,0,1,1);const auto event=motor.takeEvents().front();
        check(io.has(hardware.motor_left_pwm,15000) && io.has(hardware.motor_right_pwm,13000),"raw tuning ignores normalized vehicle cap and artificial 12000 cap");
        check(event.dutyMax[0]==20000 && event.dutyMax[1]==16000,"per-wheel full scales preserved");
        motor.stop("MOTOR_STOP_TEST");motor.takeEvents();
        motor.start(2000,0,.03,0,1,2);const auto asymmetric=motor.takeEvents().front();
        check(asymmetric.writtenRaw[0]==2000 && asymmetric.writtenRaw[1]==0,"single wheel raw output keeps stationary wheel zero");
        motor.stop("MOTOR_STOP_TEST");motor.takeEvents();
        const auto before=io.writes.size();
        rejects([&] {motor.start(2000,16001,1,0,1,3);},"right device bound checked before left starts");
        check(io.writes.size()==before && !motor.active(),"invalid asymmetric PWM has no nonzero side effects");
    }
    {
        MotorOnlyIo io;io.leftMax=io.rightMax=65535;RehearsalMotor motor(io,hardware,params);
        motor.start(-65535,-65535,.03,1,1,1);const auto event=motor.takeEvents().front();
        check(event.writtenRaw[0]==65535 && event.gpio[0]==1,"uint16 maximum preserves raw payload and reverse polarity");
        motor.stop("MOTOR_STOP_TEST");
        rejects([&] {motor.start(-65536,-2000,1,1,1,2);},"wire format overflow rejected before casting");
    }
    {
        MotorOnlyIo io;RehearsalMotor motor(io,hardware,params);int corrections=0,centers=0;
        motor.setStopAction([&] {++centers;});motor.start(2000,2000,.12,0,1,1);motor.takeEvents();
        for(int i=0;i<8;++i) {std::this_thread::sleep_for(std::chrono::milliseconds(20));motor.whileActive([&] {++corrections;});motor.heartbeat();}
        check(!motor.active() && corrections>0 && centers==1,"feedback never extends timer and stop centers once");
        check(!motor.whileActive([&] {++corrections;}),"no feedback after asynchronous center");
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
        const auto event=motor.takeEvents().front();
        check(event.writtenRaw[0]==-1 && event.writtenRaw[1]==0 && !event.error.empty(),"failed left zero logged unknown, successful right zero logged zero");
        rejects([&] {motor.start(2000,2000,1,0,1,1);},"failed output latched, no silent retry");
    }
    ParkingTuning tuning;tuning.stage=1;tuning.stages[0].motorLeft=tuning.stages[0].motorRight=2000;
    tuning.validate();check(!tuning.stages[0].powered(),"2000 with time zero is inert");
    tuning.stages[0].motorSeconds=1;tuning.validate();
    auto bad=tuning;bad.single=false;rejects([&] {bad.validate();},"powered full sequence rejected during commissioning");
    auto freeTuning=tuning;freeTuning.stages[0].motorLeft=65535;freeTuning.validate();
    check(freeTuning.stages[0].motorLeft==65535,"parser imposes no normalized or 2000 cap before device metadata is available");
    bad=tuning;bad.stages[5].motorLeft=2000;rejects([&] {bad.validate();},"stage6 remains all zero");
    const auto delivered=ParkingTuning::load("deploy/config/parking_tuning.ini");
    check(delivered.stages[4].motorRight==2000 && delivered.stages[5].motorRight==0,"all five motion stages use 2000, final stage remains zero");
    auto partialSource=delivered.source;const auto keyPosition=partialSource.find("motor_left_command=2000");
    partialSource.erase(keyPosition,std::string("motor_left_command=2000\n").size());
    rejects([&] {ParkingTuning::parse(partialSource);},"partial motor triplet rejected");
    for(double value:{-1.,65536.,1999.5,std::numeric_limits<double>::quiet_NaN()}) {
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
    SavedTuningWatcher watcher(tuning,15);auto watched=delivered.source;
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
    {
        RehearsalTiming bends("delta");
        bends.begin(1,1,1,int64_t(1e9));bends.motorStart(int64_t(1.1e9));
        bends.segmentBegin(1,1,0,int64_t(1.1e9));bends.segmentEnd(1,1,0,int64_t(6.1e9));
        bends.segmentBegin(1,1,1,int64_t(6.2e9));bends.motorStop(int64_t(7.2e9),"MOTOR_STOP_STAGE");
        bends.segmentEnd(1,1,1,int64_t(8e9));bends.motorStop(int64_t(9e9),"EXIT_ZERO");
        const auto& row=bends.rows().front();
        check(row.segmentEnd[0]-row.segmentStart[0]==int64_t(5e9),"first bend time measured independently of right correction");
        check(row.segmentEnd[1]-row.segmentStart[1]==int64_t(1e9),"correction duration closes at output stop, excluding exit tail");
        check(row.motorStop==int64_t(7.2e9),"repeated exit zero cannot inflate motor output duration");
        check(!bends.elapsed(3,1,int64_t(10e9)),"previous timing is not displayed under a new stage");
        check(bends.elapsed(1,1,int64_t(10e9)) && std::abs(*bends.elapsed(1,1,int64_t(10e9))-6.1)<1e-8,"completed stage duration stays frozen during save waiting");
        std::ostringstream summary;bends.writeSummary(summary,0);
        check(summary.str().find("primary_duration_s,correction_duration_s")!=std::string::npos,"timing declares both turn-segment durations");
        check(summary.str().find("NOT_RUN,分支直退,,,")!=std::string::npos,"skipped branch stage is explicitly unknown rather than zero seconds");
        std::istringstream csv(summary.str());std::string line;size_t lines=0;
        while(std::getline(csv,line)) {check(std::count(line.begin(),line.end(),',')==16,"all timing summary rows match the CSV header");++lines;}
        check(lines==7,"summary represents all six stages, including unexecuted stages");
        bends.begin(5,1,1,int64_t(10e9));bends.segmentBegin(5,1,0,int64_t(10e9));bends.end(int64_t(12e9),"STAGE_END");
        check(bends.rows().back().end-bends.rows().back().ready==int64_t(2e9),"stationary confirmation records its actual manual duration");
        rejects([&] {bends.begin(6,1,1,int64_t(13e9));},"invalid stage index cannot corrupt the six-stage summary");
    }
    std::cout<<checks<<" motor/timing checks passed\n";return 0;
 } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
