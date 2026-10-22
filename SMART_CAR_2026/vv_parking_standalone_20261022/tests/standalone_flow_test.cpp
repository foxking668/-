#include "Parking/StandaloneTestRuntime.hpp"
#include "NewTrack/TrackPolicy.hpp"
#include <cassert>
#include <iostream>

Parking::Config currentConfig() {
    Parking::Config c;
    c.center=102; c.servoOffset=95; c.approachMs=4000; c.settleMs=200;
    c.leftEnabled=false; c.rightEnabled=true;
    c.stages={{{true,-107,12700,2450,2000},{true,108,12700,3500,2000},{true,0,12700,1700,0}}};
    return c;
}
int main() {
    using namespace Parking;
    using namespace ParkingTest;
    assert(!runtime().enabled()); // Competition entry does not acquire test semantics.
    for(auto m : {Mode::Full,Mode::ApproachOnly,Mode::ReverseOnly,Mode::S2,Mode::S4,Mode::S5})
        assert(parseMode(modeName(m))==m);
    bool bad=false; try { parseMode("unknown"); } catch(const std::exception&) {bad=true;} assert(bad);
    const auto c=currentConfig(); assert(validate(c).empty());
    const auto full=selectStages(c,Mode::Full);
    assert(full.approachMs==4000 && full.stages[0].totalMs==2450 && full.stages[1].holdMs==2000);
    Controller controller; assert(controller.arm(full,4200));
    assert(controller.tick(1000).phase==Phase::StopHold);
    assert(controller.tick(4200,false).phase==Phase::StopHold);
    assert(controller.tick(4200,true).phase==Phase::Approach);
    assert(controller.tick(9000).phase==Phase::Approach); // No forward command: timer must not run.
    controller.forwardApplied(10000);
    assert(controller.tick(13999).phase==Phase::Approach);
    auto out=controller.tick(14000);
    assert(out.phase==Phase::S2 && out.angle==-5 && out.leftPwm==0 && out.rightPwm==0);
    assert(controller.tick(14199).rightPwm==0);
    out=controller.tick(14200); assert(out.rightPwm==-12700 && out.leftPwm==0 && out.angle==-5);
    assert(controller.tick(16199).angle==-5);
    assert(controller.tick(16200).angle==102);
    out=controller.tick(16450); assert(out.phase==Phase::S4 && out.angle==210 && out.rightPwm==0);
    assert(controller.tick(16650).rightPwm==-12700);
    assert(controller.tick(18650).angle==102);
    out=controller.tick(19950); assert(out.phase==Phase::S5 && out.rightPwm==0);
    assert(controller.tick(20150).rightPwm==-12700);
    out=controller.tick(21650); assert(out.phase==Phase::Done && out.leftPwm==0 && out.rightPwm==0);
    assert(!controller.arm(c,0)); // One run cannot silently rearm.
    Controller approach; assert(approach.arm(selectStages(c,Mode::ApproachOnly),3200));
    assert(approach.tick(3200).phase==Phase::Approach); approach.forwardApplied(3250);
    assert(approach.tick(7249).phase==Phase::Approach);
    out=approach.tick(7250); assert(out.phase==Phase::Done && out.rightPwm==0);
    for(auto m : {Mode::ReverseOnly,Mode::S2,Mode::S4,Mode::S5}) {
        const auto selected=selectStages(c,m);
        assert(selected.approachMs==0);
        Controller single; assert(single.arm(selected,0));
        out=single.tick(0);
        const auto expected=m==Mode::S4?Phase::S4:m==Mode::S5?Phase::S5:Phase::S2;
        assert(out.phase==expected && out.rightPwm==0);
        if(m!=Mode::ReverseOnly) {
            const int i=m==Mode::S2?0:m==Mode::S4?1:2;
            assert(single.tick(200).rightPwm==-12700);
            assert(single.tick(c.stages[i].totalMs).phase==Phase::Done);
        }
    }
    // The original zebra confirmation is still required; a manual start is not a detection.
    NewTrack::TrackPolicy::ZebraConfirmation zebra;
    for(int t=0;t<10;++t) assert(!zebra.update(false,false,false,t*50,true,true,3,2,3,150,6000).triggered);
    bool detected=false;
    for(int t=10;t<20;++t) detected |= zebra.update(true,true,true,t*50,true,true,3,2,3,150,6000).triggered;
    assert(detected);
    runtime().configure(Mode::Full, "");
    runtime().frame(true, 2.5f);
    runtime().armed(c);
    runtime().motor(Phase::Idle,"search",1000,1000,7,6,6);
    assert(!runtime().finished());
    runtime().motor(Phase::Approach,"line_follow",1000,1000,7,7,7);
    assert(runtime().phase()==Phase::Approach && !runtime().finished());
    runtime().motor(Phase::Done,"parking",0,0,0,0,0);
    assert(runtime().finished()); // Completion is observed only after zero output publication.
    std::cout << "PASS: original timing, actual forward-command origin, all modes, zero transitions, zebra confirmation\n";
}
