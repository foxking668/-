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
    assert(loadFile(argv[1],s,error));assert(s.entryTimeMs==4500 && s.entryEarlyMs==500 && s.entryLateMs==500 && s.zoneSpeed==9);
    Controller c;auto update=[&](long long t,const Observation& o,bool allowed=true,int lap=0) {c.update(t,lap,true,t,allowed,s,o);};
    update(3900,view());update(3933,view());assert(c.phase()==Phase::IDLE);
    update(4000,view());assert(c.phase()==Phase::IDLE);
    update(4033,view());assert(c.phase()==Phase::LEFT && c.entryReason()=="VISUAL_WINDOW" && c.error()==-30 && c.speed()==9);
    // Changing hot parameters cannot mutate this action.
    s.zoneSpeed=2;s.errors[0]=1;assert(c.speed()==9 && c.error()==-30);
    update(4066,view(60,75));update(4099,view(60,83));assert(c.nearSeen());
    update(4132,road());update(4165,road());assert(c.phase()==Phase::LEFT); // Minimum time not yet met.
    update(4198,road());update(4231,road());update(4264,road());assert(c.phase()==Phase::RIGHT && c.error()==58);
    auto pass=[&](long long t,double x,Phase expected) {
        update(t,view(x,55));update(t+33,view(x,70));update(t+66,view(x,82));
        for(int dt=99;dt<=231;dt+=33) update(t+dt,road());assert(c.phase()==expected);
    };
    pass(4297,20,Phase::THIRD_LEFT);pass(4561,60,Phase::RETURN_RIGHT);
    auto offCenter=road();offCenter.roadCenter=90;
    update(4825,offCenter);update(4858,road());update(4891,offCenter);assert(c.phase()==Phase::RETURN_RIGHT);
    update(4924,road());assert(c.phase()==Phase::RETURN_RIGHT);update(4957,road());
    assert(c.phase()==Phase::POST && c.routeComplete() && c.releaseAtMs()==4957);
    assert(c.speed()==7);for(int t=5089;t<11000;t+=100) update(t,road());assert(c.phase()==Phase::DONE);
    update(12000,view());assert(c.phase()==Phase::DONE);update(12033,road(),true,1);assert(c.phase()==Phase::LEFT); // Next lap deadline.
    // Deadline triggers without a cone, including a zero-width entry window.
    c.reset();s=Settings{};s.enabled=true;update(4999,road());assert(!c.used());update(5000,road());
    assert(c.phase()==Phase::LEFT && c.entryReason()=="TIME_FALLBACK");
    // Disappearance before reaching near cannot advance. Neither can time alone.
    update(5033,view());update(5066,view());for(int t=5099;t<7400;t+=33) update(t,road());
    assert(c.phase()==Phase::LEFT);for(int t=7409;t<=7541;t+=33) update(t,road());
    assert(c.needsStop() && !c.routeComplete());
    // Duplicate timestamps, gaps and invalid frames cannot provide two-frame evidence.
    c.reset();update(4000,view());update(4000,view());assert(!c.used());update(4300,view());assert(!c.used());
    update(4333,view());assert(c.used());update(4366,view(60,78));assert(c.nearSeen());
    update(4399,{},false);const auto remaining=c.remainingMs();update(9000,road());assert(c.remainingMs()==remaining && !c.nearSeen());
    update(9033,road());assert(c.phase()==Phase::LEFT);
    c.update(9066,0,false,9066,true,s,road());assert(!c.active());update(9099,view());assert(c.routeComplete() && !c.active());
    // Exact deadline entry with both floating ranges zero.
    c.reset();s.entryEarlyMs=s.entryLateMs=0;update(4499,road());assert(!c.used());update(4500,road());assert(c.used());
    c.reset();s=Settings{};s.enabled=true;
    // Parser rejects old formats, partial writes, duplicates, unknown/NaN, non-integer times and inverted limits.
    std::ifstream input(argv[1]);std::ostringstream all;all<<input.rdbuf();const std::string original=all.str();
    auto invalid=[&](const std::string& text) {std::ofstream f("bad_cone_tuning.tmp");f<<text;f.close();Settings next=s;assert(!loadFile("bad_cone_tuning.tmp",next,error));assert(next.zoneSpeed==s.zoneSpeed);};
    invalid("first_left_speed=1\n");invalid(original+"\ncone_zone_speed=3\n");invalid(original+"\nunknown=1\n");
    for(const auto& pair:std::vector<std::pair<std::string,std::string>>{{"cone_zone_speed=9","cone_zone_speed=nan"},
        {"entry_time_ms=4500","entry_time_ms=4500.5"},{"first_left_timeout_ms=2500","first_left_timeout_ms=200"},
        {"entry_early_ms=500","entry_early_ms=5000"},{"wait_duration_ms=0","wait_duration_ms=1"}}) {
        auto text=original;const auto pos=text.find(pair.first);assert(pos!=std::string::npos);text.replace(pos,pair.first.size(),pair.second);invalid(text);
    }
    assert(!loadFile("missing_cone_tuning.tmp",s,error));std::remove("bad_cone_tuning.tmp");
    std::cout<<"PASS cone route: window/two fresh frames/deadline, visual L/R/L/R, minimum/timeout, lock, pause/gap, once per lap, strict parser\n";
}
