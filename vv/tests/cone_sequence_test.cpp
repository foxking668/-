#include "NewTrack/ConeSequence.hpp"
#include <cassert>
#include <cstdio>
#include <iostream>
#include <sstream>
using namespace NewTrack::ConeSequence;
Observation view(double x=60,double bottom=55) {
    Observation o;o.valid=true;o.roadValid=true;o.candidates.push_back({x,bottom-25,12,25,true});return o;
}
Observation road() {Observation o;o.valid=true;o.roadValid=true;return o;}
int main(int argc,char** argv) {
    assert(argc==2);Settings s;s.enabled=true;std::string error;
    assert(loadFile(argv[1],s,error));assert(s.entryTimeMs==4500 && s.entryEarlyMs==500 && s.entryLateMs==500 && s.zoneSpeed==6);
    Controller c;auto update=[&](long long t,const Observation& o,bool allowed=true,int lap=0) {c.update(t,lap,true,t,allowed,s,o);};
    update(3900,view());update(3933,view());assert(c.phase()==Phase::IDLE);
    update(4000,view());update(4033,view());assert(c.forcing() && c.entryReason()=="VISUAL_WINDOW" && c.error()==-30 && c.speed()==6);
    s.zoneSpeed=2;s.errors[0]=1;update(4133,road());assert(c.error()==-30 && c.speed()==6);
    update(4233,road());assert(c.guiding() && c.phase()==Phase::LEFT && c.error()==0 && c.pulseRemainingMs()==0);
    // A stopped visual-guidance wait cannot consume time or restart the forced pulse.
    c.reportDriveAllowed(false);const auto remaining=c.remainingMs();update(8000,road());assert(c.guiding() && c.remainingMs()==remaining);
    for(int t=8100;t<=10400;t+=100) update(t,road());assert(c.needsStop());
    c.reset();s=Settings{};s.enabled=true;
    update(4000,view());update(4033,view());
    auto pass=[&](long long t,double x,Phase expected) {
        update(t,view(x,60));update(t+33,view(x,78));update(t+66,view(x,85));
        update(t+99,road());update(t+132,road());assert(c.phase()==expected);
    };
    pass(4066,60,Phase::RIGHT);pass(4231,20,Phase::THIRD_LEFT);pass(4396,60,Phase::RETURN_RIGHT);
    auto offCenter=road();offCenter.roadCenter=90;update(4561,offCenter);update(4594,road());assert(!c.routeComplete());
    update(4627,offCenter);update(4660,road());assert(!c.routeComplete());update(4693,road());assert(c.routeComplete());
    assert(c.speed()==7);for(int t=4726;t<=10326;t+=100) update(t,road());assert(c.phase()==Phase::DONE);
    update(10400,view());assert(c.phase()==Phase::DONE);update(10433,road(),true,1);assert(c.phase()==Phase::LEFT);
    // Deadline enters without an observed cone; time alone can only end the pulse, never count a cone.
    c.reset();update(4999,road());assert(!c.used());update(5000,road());assert(c.entryReason()=="TIME_FALLBACK");
    update(5100,road());update(5200,road());assert(c.guiding() && !c.routeComplete());
    // Near acquisition is permitted only as two-frame evidence, not an instant pass.
    c.reset();update(5000,road());auto clipped=view(60,45);clipped.candidates[0].acquire=false;clipped.candidates[0].nearClipped=true;
    update(5033,clipped);assert(!c.nearSeen());update(5066,clipped);assert(c.nearSeen());
    c.reportDriveAllowed(false);update(5099,road());c.reportDriveAllowed(false);update(5132,road());assert(c.phase()==Phase::LEFT);
    // Duplicate timestamps, gaps and invalid frames cannot provide two-frame evidence.
    c.reset();update(4000,view());update(4000,view());assert(!c.used());update(4300,view());assert(!c.used());
    update(4333,view());assert(c.used());update(4366,view(60,78));assert(c.nearSeen());
    update(4399,{},false);const auto paused=c.remainingMs();update(9000,road());assert(c.remainingMs()==paused && !c.nearSeen());
    c.update(9033,0,false,9033,true,s,road());assert(!c.active());update(9066,view());assert(c.routeComplete() && !c.active());
    c.reset();s.entryEarlyMs=s.entryLateMs=0;update(4499,road());assert(!c.used());update(4500,road());assert(c.used());
    for(int t=4800;t<=9900;t+=300) update(t,road());assert(c.needsStop());
    c.reset();s.pulseMs[0]=0;update(4500,road());assert(c.guiding() && c.error()==0 && c.pulseRemainingMs()==0);
    c.reset();s=Settings{};s.enabled=true;
    // Parser rejects old formats, partial writes, duplicates, unknown/NaN, non-integer times and inverted limits.
    std::ifstream input(argv[1]);std::ostringstream all;all<<input.rdbuf();const std::string original=all.str();
    auto invalid=[&](const std::string& text) {std::ofstream f("bad_cone_tuning.tmp");f<<text;f.close();Settings next=s;assert(!loadFile("bad_cone_tuning.tmp",next,error));assert(next.zoneSpeed==s.zoneSpeed);};
    invalid("first_left_speed=1\n");invalid(original+"\ncone_zone_speed=3\n");invalid(original+"\nunknown=1\n");
    invalid(original.substr(original.find('\n')+1)); // Old 26-key format cannot silently change duration semantics.
    for(const auto& pair:std::vector<std::pair<std::string,std::string>>{{"cone_zone_speed=6","cone_zone_speed=nan"},
        {"entry_time_ms=4500","entry_time_ms=4500.5"},{"first_left_timeout_ms=2500","first_left_timeout_ms=200"},
        {"entry_early_ms=500","entry_early_ms=5000"},{"wait_duration_ms=0","wait_duration_ms=1"}}) {
        auto text=original;const auto pos=text.find(pair.first);assert(pos!=std::string::npos);text.replace(pos,pair.first.size(),pair.second);invalid(text);
    }
    assert(!loadFile("missing_cone_tuning.tmp",s,error));std::remove("bad_cone_tuning.tmp");
    std::cout<<"PASS cone route: window/two fresh frames/deadline, visual L/R/L/R, bounded pulse/visual fallback/timeout, lock, pause/gap, once per lap, strict parser\n";
}
