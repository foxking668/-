#include "Parking/AutoParking.hpp"
#include <iostream>
#include <functional>
#include <cstdio>

using namespace AutoParking;
int checks=0;
void check(bool v,const char* why) {++checks;if(!v) throw std::runtime_error(why);}
void rejects(const std::function<void()>& f) {bool bad=false;try{f();}catch(const std::exception&){bad=true;}check(bad,"expected rejection");}
Config config() {
    Config c;
    c.stages[0].pwm=9000;c.stages[0].seconds=2.5;
    c.stages[1]={-40,.5,1,2.5,5,.5,12000};
    c.stages[3]={12,.5,.4,1,-5,.2,9000};
    c.stages[4]={0,.5,0,.8,0,0,8000};
    c.stages[5].hold=.1;c.validate();return c;
}
int main(int argc,char** argv) {
    try {
        if(argc>1) {auto c=Config::read(argv[1]);check(c.zebraCount==2,"example trigger");}
        auto c=config();Controller m(c);
        const std::string roundtrip=std::string(argv[0])+".snapshot.ini";
        {std::ofstream saved(roundtrip);c.write(saved);}
        const auto restored=Config::read(roundtrip);std::remove(roundtrip.c_str());
        check(restored.stages[0].seconds==2.5 && restored.stages[1].correctionSeconds==.5 && restored.rightOnly,
              "actual config snapshot roundtrip");
        check(!m.onZebra(1,1),"first zebra must not trigger");
        check(m.onZebra(2,10),"second zebra must trigger");
        auto o=m.tick(11,true,{true,11,-20});check(o.left==0 && m.phase()==Phase::WaitResume,"stop barrier");
        m.tick(13,true,{true,13,-20});
        o=m.tick(14,false,{true,14,-20});check(o.left==9000 && o.right==9000,"both forward");
        check(m.elapsed(14)==0,"timer awaits output acknowledgement");m.acknowledge(o,true,14.02);
        check(!m.onZebra(2,14.3),"duplicate event must not reset run");
        o=m.tick(15,false,{true,15,12});check(o.steer==12,"live lane error");m.acknowledge(o,true,15.01);
        check(m.elapsed(16)>1.9,"repeated output must not restart timer");
        o=m.tick(16.521,false,{});check(o.left==0 && o.right==0 && m.phase()==Phase::Preparing,"deadline independent of invalid frame");
        check(m.stage()==2,"first bend after stage1");m.acknowledge(o,true,16.522);
        o=m.tick(16.55,false,{});check(o.steer==-40 && o.right==0,"prepare steering without drive");
        m.acknowledge(o,true,16.57);
        o=m.tick(17.06,false,{});check(o.right==0,"settle measured after correct steering write");
        o=m.tick(17.071,false,{});check(o.left==0 && o.right==-12000 && o.steer==-40,"right-only reverse");
        m.acknowledge(o,true,17.08);
        o=m.tick(18.09,false,{});check(o.steer==5 && o.right==-12000,"automatic opposite correction");
        o=m.tick(18.59,false,{});check(o.steer==0 && o.right==-12000,"center while remaining motor time continues");
        o=m.tick(19.581,false,{});check(o.right==0 && m.stage()==4,"skip stage3 and stop before stage4");
        m.acknowledge(o,true,19.582);o=m.tick(19.6,false,{});m.acknowledge(o,true,19.61);
        o=m.tick(20.12,false,{});m.acknowledge(o,true,20.13);check(o.steer==12 && o.right<0,"second bend");
        o=m.tick(20.54,false,{});check(o.steer==-5,"left correction");
        o=m.tick(21.131,false,{});check(o.right==0 && m.stage()==5,"second bend deadline");
        m.acknowledge(o,true,21.132);o=m.tick(21.65,false,{});m.acknowledge(o,true,21.66);
        check(o.right==-8000 && o.steer==0,"garage straight reverse");
        o=m.tick(22.461,false,{});m.acknowledge(o,true,22.462);
        check(m.stage()==6 && m.phase()==Phase::StopConfirmation && o.right==0,"stop confirmation");
        o=m.tick(22.563,false,{});check(m.phase()==Phase::Done && o.left==0 && o.right==0,"completion");
        check(!m.cameraNeeded(),"no camera in reverse or done");
        Controller failed(c);failed.onZebra(2,1);o=failed.tick(2,false,{true,2,0});failed.acknowledge(o,false,2.01);
        check(failed.phase()==Phase::Fault && failed.elapsed(3)==0,"failed write never starts timer");
        o=failed.tick(4,false,{});check(o.left==0 && o.right==0,"fault latches zero");
        Controller lost(c);lost.onZebra(2,1);o=lost.tick(2,false,{true,2,0});lost.acknowledge(o,true,2.001);
        o=lost.tick(2.36,false,{true,2,0});check(lost.phase()==Phase::Fault && o.right==0,"stale image stops advance");
        Controller waiting(c);waiting.onZebra(2,1);waiting.tick(5,true,{});waiting.tick(6,false,{});
        waiting.tick(9.01,false,{});check(waiting.phase()==Phase::Fault,"no line startup timeout");
        Controller zeroReverse(c);zeroReverse.onZebra(2,1);o=zeroReverse.tick(4,false,{true,4,0});
        check(o.left>0,"restart does not depend on a reverse-end callback");
        Controller time(c);time.onZebra(2,2);time.tick(1,false,{});check(time.phase()==Phase::Fault,"clock reversal");
        auto both=c;both.rightOnly=false;both.stages[0].seconds=.01;Controller dual(both);
        dual.onZebra(2,1);o=dual.tick(2,false,{true,2,0});dual.acknowledge(o,true,2);
        o=dual.tick(2.02,false,{});o=dual.tick(2.03,false,{});dual.acknowledge(o,true,2.03);
        o=dual.tick(2.54,false,{});check(o.left==-12000 && o.right==-12000,"optional both-wheel reverse");
        rejects([&]{auto x=c;x.stages[1].seconds=1;x.validate();});
        rejects([&]{auto x=c;x.stages[1].correction=-5;x.validate();});
        rejects([&]{auto x=c;x.stages[3].steer=-12;x.validate();});
        rejects([&]{auto x=c;x.stages[0].pwm=50001;x.validate();});
        rejects([&]{auto x=c;x.stages[5].pwm=100;x.validate();});
        rejects([&]{auto x=c;x.lineLost=.8;x.validate();});
        rejects([&]{Config::number("nan");});
        rejects([&]{Config::number("1.5seconds");});
        std::cout<<"PASS "<<checks<<" automatic parking state/config/timing checks\n";return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
