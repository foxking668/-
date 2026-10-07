#include "visual_observer.hpp"
#include <iostream>
#include <limits>
#include <functional>
using namespace car2026;
namespace {
int checks=0;
void check(bool condition,const char* message) {
    ++checks;if(!condition) throw std::runtime_error(message);
}
Observation line(double x=.5) {
    Observation observation;observation.frameValid=true;
    observation.blackPath.x.assign(240,x);observation.blackPath.confidence=1;
    measurePath(observation.blackPath);return observation;
}
void establish(VisualSteeringObserver& observer,double x=.5) {
    for(int n=0;n<3;++n) {
        const auto result=observer.observe(line(x),n*.2,n*.2);
        check(result.hasSuggestion==(n==2),"reference requires consecutive stable fresh frames");
    }
}
}
int main() {
    try {
        Params params;params.confirm_frames=3;
        VisualSteeringObserver forward(params,ManualMotion::Forward),reverse(params,ManualMotion::Reverse);
        establish(forward);establish(reverse);
        auto f=forward.observe(line(.55),.6,.6),r=reverse.observe(line(.55),.6,.6);
        check(f.hasSuggestion && r.hasSuggestion && f.suggestedCommand>0 && r.suggestedCommand<0,
              "forward and reverse suggestions have explicitly opposite image-correction signs");
        check(std::abs(f.suggestedCommand+r.suggestedCommand)<1e-12,"direction selection does not alter image errors");
        check(std::abs(f.lateralError-.05)<1e-12,"target remains initial image position rather than center override");
        const unsigned id=f.referenceId;
        auto bad=line();bad.blackPath.ambiguous=true;
        f=forward.observe(bad,.8,.8);
        check(f.state=="AMBIGUOUS" && !f.hasSuggestion && f.hasReference && f.referenceId==id,
              "ambiguity cancels suggestion while preserving reference identity");
        for(int n=0;n<3;++n) {
            f=forward.observe(line(.55),1.+n*.2,1.+n*.2);
            check(f.hasSuggestion==(n==2),"recovery needs fresh consecutive confirmation without resetting target");
        }
        check(f.referenceId==id && f.lateralError>.04,"recovery does not adopt displaced line as a new target");
        f=forward.observe(line(.55),1.4,1.5);
        check(f.state=="REPEATED_FRAME" && !f.hasSuggestion,"duplicate timestamp cannot accumulate commands");
        f=forward.observe(line(),1.3,1.5);
        check(f.state=="BACKWARD_TIME" && !f.hasSuggestion,"backward frame timestamp is rejected");
        f=forward.observe(line(),1.6,2.1);
        check(f.state=="STALE_FRAME" && !f.hasSuggestion,"age watchdog rejects delayed processing");
        f=forward.observe(line(),2.3,2.3);
        check(f.state=="FRAME_GAP" && !f.hasSuggestion,"long new-frame gap cancels prior suggestion");
        f=forward.observe(line(),std::numeric_limits<double>::quiet_NaN(),2.5);
        check(f.state=="INVALID_TIME" && !f.hasSuggestion,"nonfinite time cannot produce NaN commands");
        f=forward.observe(line(),2.5,2.4);
        check(f.state=="INVALID_TIME","processing time preceding frame is rejected");

        VisualSteeringObserver quality(params,ManualMotion::Forward);establish(quality);
        auto partial=line();for(int y=95;y<120;++y) partial.blackPath.x[y]=-1;
        f=quality.observe(partial,.6,.6);
        check(f.state=="PARTIAL_REFERENCE" && !f.hasSuggestion,"far-row loss cannot be replaced by heading fallback");
        bad=line();bad.blackPath.discontinuous=true;
        f=quality.observe(bad,.8,.8);
        check(f.state=="DISCONTINUOUS" && !f.hasSuggestion,"mixed fragments never yield steering advice");
        bad=line();bad.blackPath.confidence=.5;
        f=quality.observe(bad,1.,1.);
        check(f.state=="LOW_QUALITY","observer uses stricter confidence than basic path visibility");
        bad=line();bad.frameValid=false;
        f=quality.observe(bad,1.2,1.2);
        check(f.state=="INVALID_FRAME","invalid camera input preserves no suggestion");
        bad=line();bad.blackPath.heading=std::numeric_limits<double>::infinity();
        f=quality.observe(bad,1.4,1.4);
        check(f.state=="LOW_QUALITY" && !f.hasSuggestion,"nonfinite image heading cannot create a command");
        bad=line();bad.blackPath.lateral=10;
        f=quality.observe(bad,1.6,1.6);
        check(f.state=="LOW_QUALITY" && !f.hasSuggestion,"malformed normalized lateral feature is rejected");
        bad=line();bad.blackPath.confidence=2;
        f=quality.observe(bad,1.8,1.8);
        check(f.state=="LOW_QUALITY","confidence outside normalized range is rejected");

        VisualSteeringObserver bounded(params,ManualMotion::Forward);establish(bounded,.1);
        double previous=0;
        for(int n=1;n<=17;++n) {
            f=bounded.observe(line(.1+n*.05),.4+n*.2,.4+n*.2);
            check(f.hasSuggestion && std::abs(f.suggestedCommand)<=5 &&
                  std::abs(f.suggestedCommand-previous)<=.600001,"suggestions obey command and per-second rate limits");
            previous=f.suggestedCommand;
        }
        check(std::abs(f.suggestedCommand-5)<1e-12,"large accumulated image error saturates at the diagnostic cap");
        bounded.reset();establish(bounded,.7);
        f=bounded.observe(line(.7),.6,.6);
        check(f.referenceId==2 && std::abs(f.lateralError)<1e-12,"explicit reset creates a distinct image reference");
        check(parseManualMotion("forward")==ManualMotion::Forward && parseManualMotion("reverse")==ManualMotion::Reverse,
              "manual movement direction is explicit rather than guessed from encoders");
        bool threw=false;try {parseManualMotion("automatic");}catch(...) {threw=true;}
        check(threw,"ambiguous motion selection is rejected");
        std::cout<<"PASS "<<checks<<" visual observer checks (no hardware library or actuator interface)\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
