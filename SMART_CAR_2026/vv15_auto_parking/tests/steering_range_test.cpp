#include "Parking/SteeringValidation.hpp"
#include "Parking/SteeringOutput.hpp"
#include <iostream>
#include <limits>
using namespace AutoParking;
int checks=0;
void check(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
template<class F> void rejects(F f,const std::string& key){
    try{f();}catch(const std::runtime_error& e){
        check(std::string(e.what()).find(key)!=std::string::npos,"error identifies invalid stage and field");return;
    }
    check(false,"invalid input must be rejected");
}
struct Steer {
    int writes=0;float angle=0,offset=0,duty=0;
    void setSteerOffset(float v){offset=v;}
    bool setAngle(float v){angle=v;++writes;duty=Contral::SteeringPulse::dutyForAngle(v,offset);return true;}
};
int main(){try{
    Config c;c.stages[0].pwm=9000;c.stages[0].seconds=1;
    c.stages[1]={-120.5,.5,.5,2,130.25,.5,12000};
    c.stages[3]={140.75,.5,.5,2,-150.5,.5,9000};
    c.validate();validateSteeringCommands(c,20);
    check(true,"primary and correction exceed the former +/-90 limit with decimals");
    Steer servo;
    for(double offset:{-150.,0.,20.,95.,150.}) for(double command:{-180.,-120.5,-90.,0.,90.,130.25,180.}) {
        float angle=static_cast<float>(90.0+command);
        angle+=static_cast<float>(offset);
        const float original=(((angle-90.0f)/90.0f)*5.0f)*(float)(3040000/100.0f)+1520000.0f;
        float duty;
        check(Contral::SteeringPulse::commandToDuty(command,offset,duty) && duty==original,
              "shared calculation exactly preserves original float arithmetic across offsets");
        check(writeSteeringCommand(&servo,command,offset) && servo.duty==original &&
              servo.angle==static_cast<float>(90.0+command),"output neither clamps nor reverses commands");
    }
    float low,center,high;
    Contral::SteeringPulse::commandToDuty(-120,20,low);
    Contral::SteeringPulse::commandToDuty(0,20,center);
    Contral::SteeringPulse::commandToDuty(120,20,high);
    check(low<center && center<high && std::abs(center-1553777.75f)<1,"measured offset20 and sign order preserved");
    // These are electrical limits only; no mechanical range is inferred or recommended.
    float duty;
    check(Contral::SteeringPulse::commandToDuty(-920,20,duty) && duty==0,"zero duty boundary");
    check(Contral::SteeringPulse::commandToDuty(880,20,duty) && duty==3040000,"period boundary");
    const int before=servo.writes;const float beforeOffset=servo.offset;
    for(double command:{-921.,881.,std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::max()})
        check(!writeSteeringCommand(&servo,command,20) && servo.writes==before && servo.offset==beforeOffset,
              "invalid pulse/nonfinite/float overflow rejected before driver mutation");
    check(!writeSteeringCommand(&servo,0,std::numeric_limits<double>::infinity()),"invalid offset rejected");
    check(!writeSteeringCommand<Steer>(nullptr,0,20),"null driver rejected");
    rejects([&]{auto x=c;x.stages[1].steer=-921;validateSteeringCommands(x,20);},"stage_2 steer_command");
    rejects([&]{auto x=c;x.stages[3].correction=-921;validateSteeringCommands(x,20);},"stage_4 correction_steer_command");
    rejects([&]{auto x=c;x.stages[1].correctionSeconds=0;x.stages[1].correction=881;
                validateSteeringCommands(x,20);},"stage_2 correction_steer_command");
    rejects([&]{validateSteeringCommands(c,std::numeric_limits<double>::quiet_NaN());},"stage_1 steer_command");
    rejects([&]{auto x=c;x.stages[1].steer=120;x.validate();},"Correction must oppose");
    rejects([&]{auto x=c;x.stages[3].correction=120;x.validate();},"Correction must oppose");
    rejects([&]{auto x=c;x.stages[0].steer=1;x.validate();},"Stage 1 uses live lane");
    rejects([&]{auto x=c;x.stages[5].steer=1;x.validate();},"Stage 1 uses live lane");
    rejects([&]{auto x=c;x.stages[1].steer=std::numeric_limits<double>::quiet_NaN();x.validate();},"stage_2");
    std::cout<<"PASS "<<checks<<" steering range/formula/preflight/output checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
