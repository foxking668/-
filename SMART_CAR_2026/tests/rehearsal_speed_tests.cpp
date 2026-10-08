#include "../tools/rehearsal_speed.hpp"
#include <iostream>
#include <sstream>
using namespace car2026::capture;
int checks=0;
void check(bool value,const char* message) {++checks;if(!value) throw std::runtime_error(message);}
bool near(double a,double b) {return std::abs(a-b)<1e-7;}
WheelSpeedSample sample(double count,double seconds,bool valid=true) {return {valid,count,int64_t(seconds*1e9)};}
int main() {
 try {
    RehearsalSpeedRecorder delta("delta",1,-1);
    check(!delta.add(1,0,1,1,0,0,sample(999,0),sample(-999,.02)),"first read discards pre-recording deltas");
    check(!delta.add(1,0,1,1,0,0,sample(10,.2),sample(-12,.22)),"short window buffered");
    check(!delta.add(1,0,1,1,0,0,sample(10,.4),sample(-12,.42)),"counts not converted with nominal fps");
    auto w=delta.add(1,0,1,1,0,0,sample(10,.6),sample(-12,.62));
    check(w && near(w->rate(0),50) && near(w->rate(1),60),"true time and hardware signs produce forward counts/s");
    check(!w->calibrated(),"unvalidated vehicle defaults never produce cm/s");
    w=delta.add(1,0,1,1,0,0,sample(0,1.2),sample(0,1.22));
    check(w && near(w->rate(0),0),"stopped interval recorded as zero");
    const auto& t=delta.totals().begin()->second;
    check(t.windows==2 && near(t.absoluteCounts[0]/t.seconds[0],25),"mean weights time including stopped windows");
    check(near(t.absoluteCounts[0]/t.movingSeconds[0],50),"moving-window mean excludes zero windows");
    std::ostringstream output;delta.writeSummary(output);
    check(output.str().find("center_mean_abs_cm_s")!=std::string::npos && output.str().find("50,60,50,60,0,0,,")!=std::string::npos,"uncalibrated center summary stays blank");
    delta.reset();check(!delta.latest(),"pause clears stale displayed speed");
    check(!delta.add(1,0,1,2,.1,.2,sample(800,4),sample(-800,4)),"resume baseline discards pause motion");
    w=delta.add(1,0,1,2,.1,.2,sample(-30,4.6),sample(36,4.6));
    check(w && near(w->centerCmPerSecond(),-8.5),"reverse speed stays negative with independent wheel scales");
    check(!delta.add(1,1,1,2,.1,.2,sample(-20,5),sample(20,5)),"correction segment starts new baseline");
    w=delta.add(1,1,1,2,.1,.2,sample(-12,5.6),sample(18,5.6));
    check(w && near(w->rate(0),-20) && delta.totals().size()==3,"correction window separately summarized");
    check(!delta.add(1,1,1,2,.1,.2,sample(200,5.8,false),sample(-200,5.8)),"invalid sensor resets both wheels");
    check(!delta.add(1,1,1,2,.1,.2,sample(1000,6),sample(-1000,6)),"first post-error read discarded");
    check(!delta.add(1,1,1,2,.1,.2,sample(5000,10),sample(-5000,10)),"large read gap excluded");
    w=delta.add(1,1,1,2,.1,.2,sample(-6,10.6),sample(6,10.6));
    check(w && near(w->rate(0),-10),"no speed spike after gap");
    check(!delta.add(1,1,1,3,.1,.2,sample(500,11),sample(-500,11)),"calibration/config revision starts new speed bucket");
    check(!delta.add(1,1,1,3,.1,.2,sample(std::numeric_limits<car2026::EncoderCount>::max(),11.6),sample(0,11.6)),"saturation not treated as motion");
    RehearsalSpeedRecorder cumulative("cumulative16",1,1);
    cumulative.add(4,0,1,1,0,0,sample(32760,0),sample(32760,0));
    w=cumulative.add(4,0,1,1,0,0,sample(-32766,.5),sample(-32766,.5));
    check(w && near(w->rate(0),20),"cumulative wraparound uses existing driver decoder");
    check(!cumulative.add(4,0,1,1,0,0,sample(1,.5),sample(1,.5)),"duplicate timestamp cannot divide by zero");
    check(!cumulative.add(4,0,1,1,0,0,sample(65535,1),sample(65535,1)),"invalid cumulative16 domain rejected even at baseline");
    RehearsalSpeedRecorder cumulative32("cumulative32",1,1);
    cumulative32.add(4,0,1,1,0,0,sample(2147483647.,0),sample(2147483647.,0));
    w=cumulative32.add(4,0,1,1,0,0,sample(-2147483648.,.5),sample(-2147483648.,.5));
    check(w && near(w->rate(0),2),"int32 extrema are valid cumulative wraparound values");
    RehearsalSpeedRecorder asymmetric("delta",1,1);
    asymmetric.add(0,0,1,1,0,0,sample(0,0),sample(0,.1));
    w=asymmetric.add(0,0,1,1,0,0,sample(30,.6),sample(35,.8));
    check(w && near(w->rate(0),50) && near(w->rate(1),50),"left and right use their own read timestamps");
    std::cout<<checks<<" speed checks passed\n";return 0;
 } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
