#include "../tools/parking_rehearsal.hpp"
#include <iostream>
using namespace car2026;
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool ok,const char* message) {++checks;if(!ok) throw std::runtime_error(message);}
template<class F> void rejects(F f,const char* message) {bool caught=false;try{f();}catch(...){caught=true;}check(caught,message);}
std::string changed(std::string s,const std::string& a,const std::string& b) {
    const auto pos=s.find(a);if(pos==std::string::npos) throw std::runtime_error("Missing test anchor: "+a);
    s.replace(pos,a.size(),b);return s;
}
}
int main() {
 try {
    const auto source=ParkingTuning::readSource("deploy/parking_tuning.sysfs_pwm.example.ini");
    auto tuning=ParkingTuning::parse(source,false,false,true);
    check(tuning.sysfsPwmControl && tuning.ccSpeedControl && !tuning.reverseRightOnly,"new mode reuses cc PID and both wheels");
    check(tuning.source==source,"snapshot retains exact original PWM tuning");
    check(tuning.referenceSpeed.leftRps==7.8125 && tuning.referenceSpeed.rightRps==7.8125,"40 equivalent pulses/5ms converts to 7.8125rps");
    check(tuning.referenceSpeed.pwmLimit==7500,"logical1500 converts to7500ns, never writes1500ns silently");
    check(tuning.referenceSpeed.kp==64 && tuning.referenceSpeed.ki==32 && tuning.referenceSpeed.kd==48,"original7.25 PID gains unchanged");
    for(const auto& item:tuning.stages) check(!item.powered() && item.command==0,"default stages centered and motion disabled");
    check(parkingPwmServoNs(0,3333333)==1490000,"center4470 has correct300Hz pulse");
    check(parkingPwmServoNs(parkingPwmDelta(3370),3333333)==1123333,"right measured window maps to1123333ns");
    check(parkingPwmServoNs(parkingPwmDelta(5170),3333333)==1723333,"left measured window maps to1723333ns");
    check(parkingMotorLimitNs(3500)==17500 && parkingMotorLimitNs(20)==100,"logical motor bounds converted exactly");
    check(parkingCountsToRps(1)==.1953125 && parkingCountsToRps(400)==78.125,"target counts bounds not confused withrps");
    for(double pwm:{3369.,5171.,4470.5,double(NAN)}) rejects([&]{parkingPwmDelta(pwm);},"bad servo count rejected");
    for(double counts:{0.,401.,40.5,double(NAN)}) rejects([&]{parkingCountsToRps(counts);},"bad speed target rejected");
    rejects([]{parkingPwmServoNs(0,3040000);},"legacy servo period cannot silently use PWM compatibility");
    for(double limit:{0.,19.,3501.,1500.5}) rejects([&]{parkingMotorLimitNs(limit);},"invalid or sub-minimum PID limit rejected");
    auto trial=changed(source,"[stage_2]\nservo_pwm=4470","[stage_2]\nservo_pwm=5000");
    trial=changed(trial,"stage_2]\nservo_pwm=5000\nsettle_time_s=0.5\nhold_time_s=0\nmotor_run_time_s=0", "stage_2]\nservo_pwm=5000\nsettle_time_s=0.5\nhold_time_s=2.5\nmotor_run_time_s=3.5");
    trial=changed(trial,"correction_servo_pwm=4470\ncorrection_hold_time_s=0", "correction_servo_pwm=4000\ncorrection_hold_time_s=1");
    const auto bend=ParkingTuning::parse(trial,false,false,true);
    check(bend.stages[1].command==-530 && bend.stages[1].correction.command==470,"same stage model uses negativeleft positive right");
    check(bend.stages[1].motorLeft==1 && bend.stages[1].motorRight==1 && bend.stages[1].motorSeconds==3.5,"both reverse wheels and fractional duration preserved");
    ParkingRehearsal model(bend);
    check(model.command()==-530 && model.holdSeconds()==2.5,"initial bend uses original5000 request");
    SavedTuningWatcher watcher(bend,105);
    auto saved=changed(trial,"servo_pwm=5000","servo_pwm=5100");
    check(!watcher.observe(saved,1),"partial save is debounced");
    const auto updated=watcher.observe(saved,1.4);
    check(updated && updated->source==saved && updated->stages[1].command==-630,"saved PWM reloaded without switching into angleparser");
    auto bad=changed(saved,"servo_pwm=5100","servo_pwm=5300");
    watcher.observe(bad,2);rejects([&]{watcher.observe(bad,2.4);},"bad save rejected");
    saved=changed(saved,"motor_target_counts=40","motor_target_counts=50");
    watcher.observe(saved,3);check(watcher.observe(saved,3.4)->referenceSpeed.leftRps==9.765625,"watcher recovers after invalid save");
    rejects([&]{ParkingTuning::parse(changed(trial,"motor_control=speed","motor_control=raw"),false,false,true);},"factory raw mode requires explicit reimplementation");
    rejects([&]{ParkingTuning::parse(changed(trial,"motor_target_counts=40","motor_target_counts=40\nmotor_target_rps=40"),false,false,true);},"rps and counts cannot mix");
    rejects([&]{ParkingTuning::parse(changed(trial,"servo_pwm=5000","servo_pwm=5000\nservo_pwm=5100"),false,false,true);},"duplicate raw key rejected before translation");
    rejects([&]{ParkingTuning::parse(changed(trial,"servo_pwm=5000","servo_pwm=4000"),false,false,true);},"wrong left bend direction rejected");
    rejects([&]{ParkingTuning::parse(changed(trial,"[stage_6]\nservo_pwm=4470","[stage_6]\nservo_pwm=5000"),false,false,true);},"stage6 remains centered");
    rejects([&]{ParkingTuning::parse(changed(trial,"[stage_3]","[stage_9]"),false,false,true);},"unknown stage rejected");
    SysfsPwmLineController line;
    check(line.update(79,1000000000,16,4,1,120)==0,"centered7.25 target produces PWM4470");
    check(line.update(59,1020000000,16,4,1,120)==-120,"left error uses larger rawPWM with step cap");
    for(int i=2;i<15;++i) line.update(-100,1000000000+int64_t(i)*20000000,16,4,1,120);
    check(line.update(-100,1400000000,16,4,1,120)==-700,"left steering bounded at5170");
    line.reset();for(int i=0;i<20;++i) line.update(300,2000000000+int64_t(i)*20000000,16,4,1,120);
    check(line.update(300,2500000000,16,4,1,120)==1100,"right steering bounded at3370");
    rejects([&]{line.update(NAN,3000000000,16,4,1,120);},"nonfinite line feedback rejected");
    std::cout<<"sysfs_pwm_tuning_tests: "<<checks<<" checks passed\n";
 } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
