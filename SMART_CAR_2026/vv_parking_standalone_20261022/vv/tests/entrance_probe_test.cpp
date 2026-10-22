#include "NewTrack/EntranceProbe.hpp"
#include "NewTrack/TrackPolicy.hpp"
#include "NewTrack/EightNeighbor.hpp"
#include <cstdlib>
#include <iostream>
#include <random>
using namespace NewTrack;
int checks=0;
void verify(bool ok,int line) {++checks;if(!ok){std::cerr<<"Failed at "<<line<<"\n";std::abort();}}
#define check(x) verify((x),__LINE__)

int main() {
    TrackPolicy::ProbeSettings settings;TrackPolicy::EntranceProbe probe;
    settings.requireOpening=false; // Explicitly exercise the optional v3 time-only compatibility switch.
    settings.requireWidthGrowth=false; // Legacy left-only test inputs; width validation is tested below.
    std::vector<int> straight(60,20),wide(60,15),narrow(60,25),missing(60,-1);
    auto step=[&](long long now,const std::vector<int>& l,bool entry=false,bool paused=false,bool gate=true,bool just=false,bool inside=false) {
        return probe.update(gate,just,entry,inside,false,true,paused,l,80,now,settings);
    };
    check(step(1000,straight,false,false,true,true).phase==1);
    check(step(3999,straight).phase==1);
    check(!step(4000,straight).started); // The wait permits detection; it is not an immediate steering command.
    check(!step(4040,straight).started);check(step(4080,straight).started);
    check(!step(4100,wide).accepted); // Camera settling guard.
    check(!step(4200,wide).accepted);check(!step(4240,wide).accepted);
    check(!step(4280,wide).accepted);check(!step(4320,wide).accepted);
    const auto yes=step(4360,wide);check(yes.accepted&&yes.deltaX==-5&&yes.sampleRows>=6);
    probe.reset();step(5000,straight,false,false,true,true);
    check(!step(5040,straight,true).started);check(!step(5080,straight,true).started);
    check(step(5120,straight,true).started); // Confirm before starting steering.
    step(5240,narrow);step(5280,narrow);step(5320,narrow);step(5360,narrow);check(step(5400,narrow).rejected);
    check(step(5699,straight).phase==3);check(step(5700,straight).phase==4);
    check(!step(5740,wide,true,false,true,true).started); // No immediate retry during recovery cooldown.
    probe.reset();step(10000,straight,false,false,true,true);step(10040,straight,true);step(10080,straight,true);step(10120,straight,true);
    for(int i=0;i<10;++i) check(!step(10240+i*40,missing).accepted);
    check(step(11220,missing).unknown); // Lost edge is not widening evidence.
    probe.reset();step(20000,straight,false,false,true,true);
    step(21000,straight,false,true);step(22000,straight,false,true);
    check(step(23000,straight).phase==1); // Stop duration does not eat the wait interval.
    check(!step(25000,straight).started);
    check(!step(26000,straight).started);check(!step(26040,straight).started);check(step(26080,straight).started);
    probe.reset();step(30000,straight,false,false,true,true);step(30040,straight,true);step(30080,straight,true);step(30120,straight,true);
    step(30320,wide);step(30360,straight);step(30400,wide);
    check(!step(30440,wide).accepted); // A noisy frame breaks consecutive confirmation.
    check(step(30480,wide).accepted);
    check(step(30520,wide,false,false,true,false,true).phase==0);
    probe.reset();std::vector<int> clipped=straight;
    for(int y=25;y<39;++y) clipped[y]=0;
    step(31000,clipped,false,false,true,true);step(31040,clipped,true);step(31080,clipped,true);step(31120,clipped,true);
    step(31320,clipped,true);step(31360,clipped,true);
    const auto late=step(31400,clipped,true);
    check(late.accepted&&late.deltaX==0&&late.clippedRows==14);
    probe.reset();step(32000,clipped,false,false,true,true);step(32040,clipped,true);step(32080,clipped,true);step(32120,clipped,true);
    step(32320,clipped,false);step(32360,clipped,false);
    check(!step(32400,clipped,false).accepted); // Border alone is not an entrance.

    settings.requireOpening=true;probe.reset();
    step(33000,straight,false,false,true,true);
    check(!step(36000,straight).started); // Default: timeout on a straight road must NOT turn left.
    const auto guarded=probe.update(true,false,false,false,false,true,false,straight,80,36040,settings,true);
    check(!guarded.started);
    check(!probe.update(true,false,false,false,false,true,false,straight,80,36080,settings,true).started);
    check(probe.update(true,false,false,false,false,true,false,straight,80,36120,settings,true).started); // Opening must persist before steering.
    probe.reset();step(37000,straight,false,false,true,true);
    check(!step(37040,straight,true).started);check(!step(37080,straight,true).started);
    check(step(37120,straight,true).started);
    settings.requireWidthGrowth=true;probe.reset();
    std::vector<int> right(60,70),translatedRight(60,65);
    auto widthStep=[&](long long t,const std::vector<int>& l,const std::vector<int>& r,bool entry=false,bool just=false) {
        return probe.update(true,just,entry,false,false,true,false,l,80,t,settings,true,&r);
    };
    widthStep(38000,straight,right,false,true);widthStep(38040,straight,right,true);
    widthStep(38080,straight,right,true);widthStep(38120,straight,right,true);
    widthStep(38320,wide,translatedRight);widthStep(38360,wide,translatedRight);
    check(!widthStep(38400,wide,translatedRight).accepted); // Both edges move left: camera translation is NOT wider road.
    widthStep(38440,wide,right);widthStep(38480,wide,right);
    const auto wider=widthStep(38520,wide,right);
    check(wider.accepted && wider.deltaWidth==5 && wider.widthRows>=6);

    TrackPolicy::ZebraConfirmation zebra;
    auto z=[&](bool far,bool near,bool strong,long long now,bool allowed=true) {
        return zebra.update(far,near,strong,now,allowed,true,3,2,4,150,6000);
    };
    z(true,false,false,1000);z(true,false,false,1040);check(z(true,false,false,1080).justArmed);
    check(!z(false,true,false,1200).triggered);
    check(!z(false,true,false,1240).triggered);check(z(false,true,false,1280).triggered);
    for(int i=0;i<5;++i) check(!z(true,true,true,1320+i*40).triggered); // One stop per lap.
    zebra.reset(); // New lap must allow another zebra even with missed far ROI.
    z(false,true,true,20000);z(false,true,true,20040);z(false,true,true,20080);
    check(z(false,true,true,20120).triggered);
    zebra.reset();z(false,true,true,30000);z(false,true,true,30040);
    check(!z(false,true,true,30080,false).triggered); // Ring/startup block resets partial fallback.
    check(!z(false,true,true,30120).triggered);
    zebra.reset();z(true,false,false,40000);z(true,false,false,40040);z(true,false,false,40080);
    check(z(false,false,false,46100).expired);
    z(false,true,true,46200);z(false,true,true,46240);z(false,true,true,46280);
    check(z(false,true,true,46320).triggered); // Expired far latch does not disable strong near.

    TrackPolicy::ForkConfirmation fork;
    check(!fork.update(true,false,1000,true,2,8000));
    check(!fork.update(true,false,1040,true,2,8000));check(fork.armed());
    check(fork.update(true,true,1080,true,2,8000));
    check(!fork.update(true,true,1120,false,2,8000));check(!fork.armed());
    check(!fork.update(true,true,5000,true,2,8000));check(fork.update(true,true,5040,true,2,8000));
    check(!fork.update(false,true,13041,true,2,8000)); // Old far candidate expires.

    const int w=80,h=60;
    std::vector<unsigned char> binary(w*h,0);
    for(int y=0;y<h;++y) {
        int l=20,r=43+y/2;
        if(y<10 || (y>=29 && y<=43)) l=0;
        else if(y<=28) l=26-std::abs(y-18);
        for(int x=l;x<=r;++x) binary[y*w+x]=255;
    }
    const auto boundary=EightNeighbor::trace(binary.data(),w,h,w);
    auto e=Roundabout::detect(boundary.left,boundary.right,w,{},boundary.leftPath,boundary.rightPath);
    std::cout<<"MOORE_8 D/M/U="<<e.down.y<<"/"<<e.mid.y<<"/"<<e.up.y<<" right="<<e.rightContinuous<<"\n";
    check(e.down.valid()&&e.mid.valid()&&e.up.valid()&&e.rightContinuous);
    for(const auto* path:{&boundary.leftPath,&boundary.rightPath}) {
        check(path->size()>20);
        for(std::size_t i=1;i<path->size();++i) {
            const auto a=(*path)[i-1],b=(*path)[i];
            check(std::abs(a.x-b.x)<=1 && std::abs(a.y-b.y)<=1);
            check(binary[b.y*w+b.x]!=0);
        }
    }
    auto withBlack=binary;
    for(int y=0;y<h;++y) withBlack[y*w+40]=0;
    const auto split=EightNeighbor::trace(withBlack.data(),w,h,w,boundary.left,boundary.right);
    check(split.left==boundary.left && split.right==boundary.right); // Center black line is not the outer edge.
    Roundabout::Controller controller;Roundabout::Settings cfg;cfg.entryValidationRequired=true;
    auto result=controller.update(boundary.left,boundary.right,w,10000,true,true,cfg,boundary.leftPath,boundary.rightPath);
    result=controller.update(boundary.left,boundary.right,w,10040,true,true,cfg,boundary.leftPath,boundary.rightPath);
    check(result.stage==1);
    result=controller.update(boundary.left,boundary.right,w,10080,true,true,cfg,boundary.leftPath,boundary.rightPath,true);
    check(result.stage==2&&result.active);
    controller.reset();
    check(controller.update(boundary.left,boundary.right,w,20000,false,true,cfg,boundary.leftPath,boundary.rightPath,true).stage==0);
    // A single surviving opening is sufficient after measured leftward expansion: no stage-1 delay.
    std::vector<int> l(60,20),r(60,70);for(int y=18;y<32;++y) l[y]=0;
    check(controller.update(l,r,w,20040,true,true,cfg,{}, {},true).stage==2);
    std::mt19937 rng(91);
    for(int test=0;test<120;++test) {
        const int width=12+rng()%90,height=12+rng()%80,stride=width+7;
        std::vector<unsigned char> noise(stride*height);
        for(auto& px:noise) px=(rng()%3)?255:0;
        const auto b=EightNeighbor::trace(noise.data(),width,height,stride);
        check(b.left.size()==static_cast<std::size_t>(height));
        for(auto point:b.leftPath) check(point.x>=0 && point.x<width && point.y>=0 && point.y<height);
    }
    std::cout<<"PASS "<<checks<<" eight-neighbor/probe/zebra-two-lap/far-near checks.\n";
}
