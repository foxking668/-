#include "../tools/parking_rehearsal.hpp"
#include <iostream>
#include <limits>
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool ok,const char* name) {++checks;if(!ok) throw std::runtime_error(name);}
template<class F> void rejects(F action,const char* name) {bool failed=false;try {action();}catch(...) {failed=true;}check(failed,name);}
LineGray road(int center=80,int width=8,int background=160,int dark=30,bool dashed=false) {
    LineGray image;image.fill(static_cast<unsigned char>(background));
    for(int y=0;y<82;++y) for(int x=center-width/2;x<center+(width+1)/2;++x)
        if(!dashed || y%20<12) image[size_t(y*160+x)]=static_cast<unsigned char>(dark);
    return image;
}
}
int main() {
 try {
    LineFollowTuning tuning;BlackLineTracker tracker;
    auto line=tracker.analyze(road(),tuning);
    check(line.reliable && line.rows==33 && std::abs(line.target-79.5)<.01,"centered black line uses many real rows");
    tracker.reset();line=tracker.analyze(road(65),tuning);
    check(line.reliable && line.target<70,"left line recognized");
    tracker.reset();line=tracker.analyze(road(96),tuning);
    check(line.reliable && line.target>90,"right line recognized");
    tracker.reset();check(!tracker.analyze(road(80,80),tuning).reliable,"wide black object rejected");
    tracker.reset();check(!tracker.analyze(road(80,8,100,95),tuning).reliable,"low contrast floor rejected");
    tracker.reset();check(tracker.analyze(road(80,8,70,15),tuning).reliable,"dim floor with real contrast remains usable");
    tracker.reset();line=tracker.analyze(road(80,8,160,30,true),tuning);
    check(line.reliable && line.gaps>0 && line.rows<33,"dash gaps do not inflate real row confidence");
    tracker.reset();auto few=road();for(int y=0;y<70;++y) for(int x=0;x<160;++x) few[size_t(y*160+x)]=160;
    check(!tracker.analyze(few,tuning).reliable,"near-only patch cannot start motor");
    tracker.reset();tracker.analyze(road(),tuning);
    check(!tracker.analyze(road(115),tuning).reliable,"large temporal target jump rejected");
    tracker.reset();LineGray blank;blank.fill(140);
    check(!tracker.analyze(blank,tuning).reliable,"no line rejected");
    auto sloped=road();sloped.fill(160);
    for(int y=0;y<82;++y) {int x=70+y/4;for(int k=-3;k<=3;++k) sloped[size_t(y*160+x+k)]=30;}
    tracker.reset();line=tracker.analyze(sloped,tuning);
    check(line.reliable && line.farX<line.nearX,"direction preview follows actual far-near trend");
    LineSteeringController controller;LineObservation right;right.reliable=true;right.target=100;right.nearX=right.farX=100;
    check(controller.update(right,tuning,0)==0,"first active frame starts centered");
    double previous=0;
    for(int i=1;i<=30;++i) {
        double command=controller.update(right,tuning,i*.05);
        check(command>=0 && command<=tuning.maxCommand && std::abs(command-previous)<=1.500001,"right sign, amplitude and slew limits");previous=command;
    }
    right.target=right.nearX=right.farX=40;
    for(int i=31;i<=60;++i) {
        double command=controller.update(right,tuning,i*.05);
        check(std::abs(command)<=tuning.maxCommand && std::abs(command-previous)<=1.500001,"left transition remains filtered and bounded");previous=command;
    }
    check(previous<0,"left line eventually produces left command");
    controller.reset();right.target=right.nearX=right.farX=80;
    controller.update(right,tuning,0);check(controller.update(right,tuning,.1)==0,"small center noise stays in deadband");
    rejects([&] {controller.update(right,tuning,.1);},"duplicate timestamps rejected");
    right.reliable=false;rejects([&] {controller.update(right,tuning,.2);},"unreliable line cannot steer");
    LineFollowSession session;session.wait(1);
    LineObservation good;good.reliable=true;
    session.observe(good,1);session.observe(good,1.05);check(!session.ready(),"two frames insufficient");
    session.observe(good,1.15);check(session.ready() && session.waiting(),"readiness persists beyond one loop");
    session.started();check(!session.waiting() && !session.lost(1.2,tuning),"valid line start retains last detection");
    check(!session.lost(1.4,tuning),"one missing frame at recorded 0.2s cadence may recover");
    check(session.lost(1.6,tuning),"lost-line deadline expires without valid detections");
    session.wait(2);check(!session.ready() && !session.expired(2,tuning),"restart resets confirmation and timeout");
    session.observe(good,2.1);session.observe(LineObservation{},2.2);session.observe(good,2.3);
    check(!session.ready(),"bad frame resets consecutive acquisition");
    check(session.expired(5.1,tuning),"unacquired line ends without motor start");
    session.wait(6);session.observe(good,6.01);session.observe(good,6.02);session.observe(good,6.03);
    check(!session.ready(),"confirmation span belongs to actual observations, not sensor/recording delay");
    auto delivered=ParkingTuning::load("deploy/config/parking_tuning.ini");
    check(delivered.stages[0].line.enabled && !delivered.stages[0].powered(),"new template enables observation but zero time stays inert");
    auto bad=delivered;bad.stages[0].holdSeconds=1;rejects([&] {bad.validate();},"line and independent fixed servo hold cannot conflict");
    bad=delivered;bad.stages[0].command=5;rejects([&] {bad.validate();},"line starts from zero rather than fixed turn");
    bad=delivered;bad.stages[0].line.maxCommand=16;rejects([&] {bad.validate();},"angle cap checked");
    bad=delivered;bad.stages[0].line.cropTop=.7;bad.stages[0].line.cropBottom=.8;rejects([&] {bad.validate();},"thin ROI rejected");
    bad=delivered;bad.stages[1].line.enabled=true;rejects([&] {bad.validate();},"reverse stages cannot accidentally follow forward camera line");
    bad=delivered;bad.stages[0].line.kp=std::numeric_limits<double>::quiet_NaN();rejects([&] {bad.validate();},"nonfinite gain rejected");
    auto next=delivered;next.stages[0].line.kp=.4;
    ParkingRehearsal model(delivered);check(model.applySaved(next,0).has_value(),"line gain save authorizes active stage restart");
    next.stages[0].motorSeconds=1;model.applySaved(next,.1);model.acknowledge(true,.1);
    check(model.state()==ParkingRehearsal::State::Settling,"powered line reload waits before acquisition");
    SavedTuningWatcher watcher(delivered,15);auto source=delivered.source;
    source.replace(source.find("line_kp=0.35"),std::string("line_kp=0.35").size(),"line_kp=0.4");
    watcher.observe(source,0);check(watcher.observe(source,.4).has_value(),"watcher includes line parameters");
    std::cout<<checks<<" line-follow checks passed\n";return 0;
 } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
