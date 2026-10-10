#include "../tools/reference_motor_speed.hpp"
#include <iostream>
#include <limits>
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool condition,const char* name) {++checks;if(!condition) throw std::runtime_error(name);}
template<class F> void rejects(F action,const char* name) {bool failed=false;try {action();}catch(...) {failed=true;}check(failed,name);}
}
int main() {
    try {
        // Repeated lifetime/reset exercises the original fixed pool without leaks.
        for(int lifetime=0;lifetime<30;++lifetime) {
            ReferenceMotorSpeed speed;ReferenceMotorTuning tuning;
            rejects([&]{speed.update(0,0);},"unconfigured controller cannot produce output");
            speed.reset(tuning,1,true,true);
            check(speed.targets()==std::array<double,2>{9,9},"cc target speed units preserved");
            check(speed.update(0,0)==std::array<int,2>{1296,1296},"cc exact first incremental output");
            check(speed.update(0,0)==std::array<int,2>{1152,1152},"cc exact second incremental output");
            for(int i=0;i<100;++i) speed.update(0,0);
            check(speed.update(0,0)==std::array<int,2>{12000,12000},"cc output limitation preserved");
            speed.reset(tuning,-1,true,true);
            check(speed.update(0,0)==std::array<int,2>{-1296,-1296},"reverse uses signed speed target and original algorithm");
            speed.reset(tuning,1,true,true);
            check(speed.update(9,0)==std::array<int,2>{0,1296},"independent encoder feedback on each wheel");
            speed.reset(tuning,1,false,true);
            check(speed.update(80,0)==std::array<int,2>{0,1296},"disabled wheel is never driven by overspeed PID");
            tuning.leftRps=tuning.rightRps=0;speed.reset(tuning,1,true,true);
            check(speed.update(80,80)==std::array<int,2>{0,0},"zero common target holds both wheels at zero PWM");
            tuning.leftRps=.1;tuning.rightRps=.2;speed.reset(tuning,1,true,true);
            check(speed.update(0,0)==std::array<int,2>{100,100},"reference minimum nonzero PWM is 100 ns");
            tuning.leftRps=tuning.rightRps=9;tuning.pwmLimit=50000;speed.reset(tuning,1,true,true);
            for(int i=0;i<200;++i) speed.update(0,0);
            check(speed.update(0,0)==std::array<int,2>{50000,50000},"user can tune through full physical PWM period");
            rejects([&]{speed.update(std::numeric_limits<double>::quiet_NaN(),0);},"invalid feedback rejected");
            rejects([&]{speed.reset(tuning,0,true,true);},"undefined motion direction rejected");
            tuning.pwmLimit=50001;rejects([&]{speed.reset(tuning,1,true,true);},"PWM cannot exceed physical period");
        }
        std::cout<<checks<<" reference cc motor PID checks passed (no hardware)\n";return 0;
    } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
