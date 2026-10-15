#include "NewTrack/ConeSequence.hpp"
#include <cassert>
#include <cstdio>
#include <iostream>
using namespace NewTrack::ConeSequence;
int main(int argc,char** argv) {
    assert(argc==2);Settings s;s.enabled=true;std::string error;
    assert(loadFile(argv[1],s,error));assert(s.leftMs==500 && s.waitMs==200 && s.rightMs==700 && s.postMs==5500);
    Controller c;c.update(1000,0,true,true,true,s);assert(c.phase()==Phase::LEFT && c.remainingMs()==500 && c.speed()==1);
    Settings changed=s;changed.leftMs=10;changed.leftSpeed=15;
    for(int t=1100;t<1500;t+=100)c.update(t,0,true,true,true,changed);
    assert(c.phase()==Phase::LEFT && c.speed()==1 && c.settings().leftPratio==.65);
    c.update(1500,0,true,true,true,changed);assert(c.phase()==Phase::WAIT && c.speed()==7 && c.remainingMs()==200);
    c.update(1600,0,true,false,true,s);c.update(1700,0,true,false,true,s);
    assert(c.phase()==Phase::RIGHT && c.speed()==3 && c.remainingMs()==700);
    c.update(1800,0,true,false,false,s);const auto remaining=c.remainingMs();
    c.update(5000,0,true,false,false,s);c.update(5033,0,true,false,true,s);
    assert(c.remainingMs()==remaining); // No wall-clock jump or skipped turn during a stop.
    for(int t=5133;t<=5733;t+=100)c.update(t,0,true,true,true,s);
    assert(c.phase()==Phase::POST && !c.steering() && c.speed()==7);
    for(int t=5833;t<=11233;t+=100)c.update(t,0,true,true,true,s);
    assert(c.phase()==Phase::DONE && c.used());
    c.update(11266,0,true,true,true,s);assert(c.phase()==Phase::DONE);
    c.update(12000,1,true,true,true,s);assert(c.phase()==Phase::LEFT); // Only a real lap change rearms.
    c.update(12500,1,true,false,true,s);assert(c.remainingMs()==500); // Camera gap is not charged as driving.
    c.update(12533,1,false,false,true,s);assert(!c.active());
    c.update(12566,1,true,true,true,s);assert(!c.active() && c.used());
    const auto original=s;
    {std::ofstream f("bad_cone_tuning.tmp");f<<"left_speed=nan\n";}
    assert(!loadFile("bad_cone_tuning.tmp",s,error) && s.leftMs==original.leftMs && s.leftSpeed==original.leftSpeed);
    {std::ofstream f("bad_cone_tuning.tmp");f<<"left_speed=1\nleft_speed=2\n";}
    assert(!loadFile("bad_cone_tuning.tmp",s,error));
    assert(!loadFile("missing_cone_tuning.tmp",s,error));
    std::remove("bad_cone_tuning.tmp");
    std::cout<<"PASS cone sequence: exact phases/speeds, locked settings, pause, stale frame, one per lap, disable, invalid config\n";
}
