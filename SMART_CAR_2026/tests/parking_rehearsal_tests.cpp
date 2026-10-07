#include "../tools/parking_rehearsal.hpp"
#include <iostream>
#include <limits>
using namespace car2026::capture;
int checks=0;
void check(bool ok,const char* name) {++checks;if(!ok) throw std::runtime_error(name);}
template<class F> void rejects(F action,const char* name) {bool failed=false;try {action();}catch(const std::exception&) {failed=true;}check(failed,name);}
std::string config(bool single=true,int stage=2) {
    std::string result="[session]\nmode="+std::string(single?"single":"full")+"\nstage="+std::to_string(stage)+"\n";
    for(int i=1;i<=6;++i) result+="[stage_"+std::to_string(i)+"]\nsteer_command="+std::to_string(i==2?-10:i==4?10:0)+"\nsettle_time_s=0.5\nhold_time_s=5\n";
    return result;
}
void start(ParkingRehearsal& model,double time) {
    auto request=model.update(time,RehearsalKey::Enter);check(request.has_value(),"apply requires Enter");
    model.acknowledge(true,time);check(!model.update(time+.49,RehearsalKey::Enter),"settling discards Enter");
    model.update(time+.5,RehearsalKey::Enter);
    check(model.state()==ParkingRehearsal::State::AwaitStart,"settling boundary never starts trial");
    check(!model.update(time+.51,RehearsalKey::Enter),"start permission does not write");
    check(model.state()==ParkingRehearsal::State::Running,"second permission starts");
}
int main() {
 try {
    const auto initial=ParkingTuning::parse(config());check(initial.stage==2 && initial.single,"single stage2 parsed");
    check(ParkingTuning::parse("\xef\xbb\xbf"+config()).stage==2,"UTF8 BOM accepted");
    rejects([] {ParkingTuning::parse(config()+"steer_command=1\n");},"duplicate key rejected");
    rejects([] {ParkingTuning::parse(config()+"[stage_2]\n");},"duplicate section rejected");
    rejects([] {ParkingTuning::parse(config()+"unknown=1\n");},"unknown key rejected");
    rejects([] {ParkingTuning::parse("[wrong]\n");},"unknown section rejected");
    rejects([] {ParkingTuning::parse("[session]\nmode=single\nstage=2\n");},"missing six-stage params rejected");
    for(const auto& value:{"nan","inf","16","-16"}) {
        auto text=config();text.replace(text.find("steer_command=-10"),17,"steer_command="+std::string(value));
        rejects([&] {ParkingTuning::parse(text);},"invalid command rejected");
    }
    for(const auto& value:{"0.49","5.01","nan"}) {
        auto text=config();text.replace(text.find("settle_time_s=0.5"),17,"settle_time_s="+std::string(value));
        rejects([&] {ParkingTuning::parse(text);},"invalid settle rejected");
    }
    for(const auto& value:{"-1","120.01","nan"}) {
        auto text=config();text.replace(text.find("hold_time_s=5"),13,"hold_time_s="+std::string(value));
        rejects([&] {ParkingTuning::parse(text);},"invalid hold duration rejected");
    }
    for(int stage:{0,7}) rejects([&] {ParkingTuning::parse(config(true,stage));},"stage range rejected");
    auto text=config();text.replace(text.find("single"),6,"automatic");rejects([&] {ParkingTuning::parse(text);},"unknown mode rejected");
    rejects([&] {initial.validate(5);},"vehicle command ceiling checked");
    check(parseRehearsalInput({"",""})==RehearsalKey::Invalid,"queued Enter rejected");
    check(parseRehearsalInput({"10"})==RehearsalKey::Invalid,"numeric terminal tuning rejected");
    check(parseRehearsalInput({"","Q",""})==RehearsalKey::Quit,"quit overrides queued permissions");
    check(parseRehearsalInput({"","P"})==RehearsalKey::Pause,"pause overrides permission");
    for(const auto& pair:std::vector<std::pair<std::string,RehearsalKey>>{{"",RehearsalKey::Enter},{"P",RehearsalKey::Pause},{"C",RehearsalKey::Continue},{"R",RehearsalKey::Reload},{"Q",RehearsalKey::Quit}})
        check(parseRehearsalInput({pair.first})==pair.second,"named key");
    ParkingRehearsal single(initial);check(single.stage()==1,"direct stage2 without stage1");
    check(!single.update(10000,RehearsalKey::None) && !single.finished(),"wait has no duration finish/write");
    start(single,10001);check(single.trial()==1,"first trial number");
    single.update(20000,RehearsalKey::None);check(!single.finished(),"long running trial never times out");
    single.update(20001,RehearsalKey::Enter);check(single.stage()==1 && single.nextStage()==1,"single never advances");
    start(single,20002);check(single.trial()==2,"repeat increments stage trial");
    single.update(20003,RehearsalKey::Pause);check(single.paused(),"pause selected");
    auto proposal=ParkingTuning::parse(config(true,4));proposal.stages[3].command=12;
    single.propose(proposal);check(single.stage()==1 && single.command()==-10,"preview no apply");
    check(!single.update(20004,RehearsalKey::Enter) && single.paused(),"paused Enter no actuation");
    check(!single.update(20005,RehearsalKey::Continue) && single.stage()==1,"resume no apply");
    auto request=single.update(20006,RehearsalKey::Enter);
    check(request && *request==12 && single.stage()==3 && single.revision()==2,"apply proposed stage only on Enter");
    single.acknowledge(true,20006);single.update(20007,RehearsalKey::None);
    check(single.state()==ParkingRehearsal::State::AwaitStart,"no motion without permission");
    rejects([&] {single.propose(proposal);},"reload requires pause");
    ParkingRehearsal full(ParkingTuning::parse(config(false,1)));
    for(int i=0;i<6;++i) {
        start(full,10.*i);check(full.stage()==i && full.trial()==1,"stage independent trial count");
        full.update(10.*i+1,RehearsalKey::Enter);
        check(full.stage()==i && full.state()==ParkingRehearsal::State::AwaitApply,"stop cannot change steering/stage");
        check(full.nextStage()==std::min(i+1,5),"full next stage preview");
    }
    full.update(10000,RehearsalKey::None);check(!full.finished(),"stage6 does not auto finish recording");
    full.update(10001,RehearsalKey::Quit);check(full.outcome()=="USER_QUIT","manual finish");
    check(!full.update(10002,RehearsalKey::Enter),"quit latched");
    ParkingRehearsal failed(initial);failed.update(0,RehearsalKey::Enter);failed.acknowledge(false,0);
    check(failed.outcome()=="SERVO_WRITE_FAILED" && !failed.update(1,RehearsalKey::Enter),"failed write latched");
    ParkingRehearsal clock(initial);clock.update(2,RehearsalKey::None);clock.update(1,RehearsalKey::Enter);
    check(clock.outcome()=="INVALID_TIME","time reversal prevents write");
    ParkingRehearsal nonfinite(initial);nonfinite.update(std::numeric_limits<double>::quiet_NaN(),RehearsalKey::Enter);
    check(nonfinite.finished(),"nonfinite time prevents write");
    std::cout<<checks<<" parking rehearsal checks passed\n";return 0;
 } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
