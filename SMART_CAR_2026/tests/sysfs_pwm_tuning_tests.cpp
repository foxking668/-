#include "../tools/parking_rehearsal.hpp"
#include <iostream>
using namespace car2026;
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool ok,const char* reason) {++checks;if(!ok) throw std::runtime_error(reason);}
template<class F> void rejects(F f,const char* reason) {bool caught=false;try{f();}catch(const std::exception&){caught=true;}check(caught,reason);}
std::string changed(std::string s,const std::string& a,const std::string& b) {
    const auto pos=s.find(a);if(pos==std::string::npos) throw std::runtime_error("Missing test anchor: "+a);
    s.replace(pos,a.size(),b);return s;
}
std::string replaceAll(std::string s,const std::string& a,const std::string& b) {
    size_t at=0;while((at=s.find(a,at))!=std::string::npos) {s.replace(at,a.size(),b);at+=b.size();}return s;
}
}
int main() {
 try {
    const auto source=ParkingTuning::readSource("deploy/parking_tuning.sysfs_pwm.example.ini");
    auto tuning=ParkingTuning::parse(source,false,false,true);
    check(tuning.sysfsPwmControl && tuning.ccSpeedControl && !tuning.reverseRightOnly,"original-servo mode retains cc PID and both wheels");
    check(tuning.source==source,"snapshot retains original angle tuning file");
    check(tuning.referenceSpeed.leftRps==7.8125 && tuning.referenceSpeed.rightRps==7.8125,"motor counts conversion unchanged");
    check(tuning.referenceSpeed.pwmLimit==7500,"logical1500 motor limit remains7500ns");
    check(tuning.referenceSpeed.kp==64 && tuning.referenceSpeed.ki==32 && tuning.referenceSpeed.kd==48,"original motor PID gains unchanged");
    for(const auto& item:tuning.stages) check(!item.powered() && item.command==0,"fresh template centers all stages and disables motion");
    check(parkingMotorLimitNs(3500)==17500 && parkingMotorLimitNs(20)==100,"motor PWM bounds unchanged");
    check(parkingCountsToRps(1)==.1953125 && parkingCountsToRps(400)==78.125,"speed target bounds unchanged");
    for(double counts:{0.,401.,40.5,double(NAN)}) rejects([&]{parkingCountsToRps(counts);},"invalid motor target rejected");
    for(double limit:{0.,19.,3501.,1500.5}) rejects([&]{parkingMotorLimitNs(limit);},"invalid motor ceiling rejected");
    auto trial=changed(source,"[stage_2]\nsteer_command=0","[stage_2]\nsteer_command=-10.5");
    trial=changed(trial,"stage_2]\nsteer_command=-10.5\nsettle_time_s=0.5\nhold_time_s=0\nmotor_run_time_s=0", "stage_2]\nsteer_command=-10.5\nsettle_time_s=0.5\nhold_time_s=2.5\nmotor_run_time_s=3.5");
    trial=changed(trial,"correction_steer_command=0\ncorrection_hold_time_s=0", "correction_steer_command=5.25\ncorrection_hold_time_s=1");
    auto active=ParkingTuning::parse(trial,false,false,true);
    check(active.stages[1].command==-10.5 && active.stages[1].correction.command==5.25,"fractional signed angle commands survive motor conversion");
    check(active.stages[1].motorSeconds==3.5 && active.stages[1].holdSeconds==2.5,"stage and segment times unchanged");
    ParkingRehearsal model(active);
    check(model.update(0,RehearsalKey::Enter)==-10.5,"one Enter requests original-formula command");
    model.acknowledge(true,0);model.update(.5,RehearsalKey::None);
    check(model.state()==ParkingRehearsal::State::Running,"original settle and authorization state retained");
    model.holdCompleted(1,model.trial(),0);
    check(model.update(3,RehearsalKey::None)==5.25,"first bend transitions to right correction in angle units");
    SavedTuningWatcher watcher(active,105);
    const auto saved=changed(trial,"steer_command=-10.5","steer_command=-12.5");
    check(!watcher.observe(saved,1),"partial save is debounced");
    auto updated=watcher.observe(saved,1.4);
    check(updated && updated->source==saved && updated->stages[1].command==-12.5,"saved original-angle command reloads without PWM reinterpretation");
    auto bad=changed(saved,"steer_command=-12.5","steer_command=-76");
    watcher.observe(bad,2);rejects([&]{watcher.observe(bad,2.4);},"out-of-range saved command rejected");
    const auto recovered=changed(saved,"motor_target_counts=40","motor_target_counts=50");
    watcher.observe(recovered,3);check(watcher.observe(recovered,3.4)->referenceSpeed.leftRps==9.765625,"watcher recovers without losing speed settings");
    rejects([&]{ParkingTuning::parse(changed(trial,"motor_control=speed","motor_control=raw"),false,false,true);},"raw motor mode remains rejected");
    rejects([&]{ParkingTuning::parse(changed(trial,"motor_target_counts=40","motor_target_counts=40\nmotor_target_rps=40"),false,false,true);},"motor units cannot mix");
    rejects([&]{ParkingTuning::parse(changed(trial,"steer_command=-10.5","steer_command=-10.5\nsteer_command=-12"),false,false,true);},"duplicate angle command rejected");
    rejects([&]{ParkingTuning::parse(changed(trial,"steer_command=-10.5","steer_command=10"),false,false,true);},"wrong first bend direction rejected");
    rejects([&]{ParkingTuning::parse(changed(trial,"correction_steer_command=5.25","correction_steer_command=-5"),false,false,true);},"wrong correction direction rejected");
    rejects([&]{ParkingTuning::parse(changed(trial,"[stage_6]\nsteer_command=0","[stage_6]\nsteer_command=5"),false,false,true);},"final stage remains centered");
    rejects([&]{ParkingTuning::parse(changed(trial,"[stage_3]","[stage_9]"),false,false,true);},"unknown stage rejected");
    const auto oldNeutral=replaceAll(source,"steer_command=0","servo_pwm=4470");
    auto compatible=ParkingTuning::parse(oldNeutral,false,false,true);
    for(const auto& item:compatible.stages) check(item.command==0 && item.correction.command==0,"only old neutral aliases map to zero");
    check(compatible.source==oldNeutral,"legacy neutral aliases stay visible in saved snapshot");
    rejects([&]{ParkingTuning::parse(changed(oldNeutral,"servo_pwm=4470","servo_pwm=4900"),false,false,true);},"old non-neutral PWM must never be guessed as an angle");
    rejects([&]{ParkingTuning::parse(changed(oldNeutral,"servo_pwm=4470","servo_pwm=4470\nsteer_command=0"),false,false,true);},"old and new names cannot define the same command twice");
    for(const auto* value:{"-75.1","105.1","4470","nan"})
        rejects([&]{ParkingTuning::parse(changed(trial,"steer_command=-10.5",std::string("steer_command=")+value),false,false,true);},"PWM counts or invalid angle cannot become steering commands");
    std::ostringstream summary;
    writeTuningSummary(summary,tuning,"tuning.ini","sysfs",true,true);
    check(summary.str().find("阶段1巡线=关闭（无相机模式")!=std::string::npos,"camera bypass still overrides configured image mode");
    check(summary.str().find("原版-15偏移")!=std::string::npos,"summary identifies original offset");
    check(summary.str().find("3370右")==std::string::npos,"summary no longer claims old PWM calibration");
    std::cout<<"sysfs_pwm_tuning_tests: "<<checks<<" checks passed\n";
 } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
