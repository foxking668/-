#include "../src/reference_725_servo.hpp"
#include <iostream>
#include <limits>
namespace {
unsigned checks=0;
void check(bool result,const char* reason) {++checks;if(!result) throw std::runtime_error(reason);}
template<class F> void rejects(F f,const char* reason) {
    bool rejected=false;try {f();}catch(const std::runtime_error&) {rejected=true;}
    check(rejected,reason);
}
// Independently transcribed original call chain, without the wrapper's constants.
uint32_t original(float command) {
    float angle=90.0f+command;
    angle+=-15.0f;
    float duty=(((angle-90.0f)/90.0f)*5.0f)*(float(3040000/100.0f))+1520000.0f;
    return static_cast<unsigned int>(duty);
}
}
int main() {
    try {
        using namespace car2026;
        check(reference725ServoInitPulse()==1520000,"original initialization precedes -15 offset");
        check(reference725ServoCommandPulse(0,3040000)==1494666,"runtime zero includes original -15 offset and unsigned truncation");
        check(reference725ServoCommandPulse(-75,3040000)==1368000,"adjusted zero-degree input boundary");
        check(reference725ServoCommandPulse(105,3040000)==1672000,"adjusted 180-degree input boundary");
        for(int quarter=-300;quarter<=420;++quarter) {
            const double command=quarter*.25;
            check(reference725ServoCommandPulse(command,3040000)==original(float(command)),"original float expression matches at quarter-command intervals");
        }
        check(reference725ServoCommandPulse(-10,3040000)<reference725ServoCommandPulse(0,3040000),"negative command lowers pulse exactly as original call");
        check(reference725ServoCommandPulse(10,3040000)>reference725ServoCommandPulse(0,3040000),"positive command raises pulse exactly as original call");
        for(double value:{-75.01,105.01,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
            rejects([&]{reference725ServoCommandPulse(value,3040000);},"invalid command rejected before output");
        rejects([]{reference725ServoCommandPulse(0,3333333);},"old 300Hz period cannot silently retain original formula label");
        std::cout<<checks<<" original 7.25 servo checks passed (no hardware).\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
