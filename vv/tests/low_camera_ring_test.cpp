#include "NewTrack/Roundabout.hpp"
#include "NewTrack/EntranceProbe.hpp"
#include <cstdlib>
#include <iostream>
#include <vector>
using namespace NewTrack;
using namespace NewTrack::Roundabout;

namespace {
int checks=0;
void verify(bool ok,int line) {
    ++checks;
    if(!ok) {std::cerr<<"low_camera_ring_test failure at "<<line<<"\n";std::abort();}
}
#define CHECK(x) verify((x),__LINE__)
struct Lane {std::vector<int> left,right;};
Lane lowCamera() {
    Lane a;a.left.assign(60,0);a.right.assign(60,60);
    for(int y=10;y<=28;++y) a.left[y]=26-std::abs(y-18);
    for(int y=27;y<=34;++y) a.right[y]=79;
    return a;
}
Lane standard() {
    Lane a;a.left.assign(60,20);a.right.assign(60,60);
    for(int y=0;y<10;++y) a.left[y]=0;
    for(int y=10;y<=28;++y) a.left[y]=26-std::abs(y-18);
    for(int y=29;y<=43;++y) a.left[y]=0;
    return a;
}
}

int main() {
    Settings s;s.lowCameraEntryEnabled=true;s.entryValidationRequired=true;
    const Lane low=lowCamera();
    const auto e=detect(low.left,low.right,80,s);
    std::cout<<"low fixture: candidate="<<e.lowCameraEntry<<" D/M/U="<<e.down.y<<"/"<<e.mid.y<<"/"<<e.up.y
             <<" usable="<<e.usable<<" rightContinuous="<<e.rightContinuous<<std::endl;
    CHECK(e.lowCameraEntry && e.usable);
    CHECK(!e.down.valid()); // Never fabricate the missing real lower corner.
    CHECK(e.virtualDown.x==0 && e.virtualDown.y==53);
    CHECK(e.lowCameraMid.valid() && e.lowCameraUp.valid() && e.lowCameraMid.y>e.lowCameraUp.y);
    CHECK(e.rightOpeningRows==8 && e.rightEntry.y==35);
    CHECK(e.leftBreaks==detect(low.left,low.right,80,Settings{}).leftBreaks);
    CHECK(!detect(low.left,low.right,80,Settings{}).lowCameraEntry); // Feature can be disabled.

    // A sustained opening must be observed, with both real support spans.
    Lane bad=low;
    bad.right.assign(60,60);bad.right[30]=79;
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry);
    bad=low;bad.right[30]=-1;
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry); // Unknown is not white road.
    bad=low;bad.right[35]=-1;
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry);
    bad=low;bad.right.assign(60,79);
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry); // Whiteout has no support.
    bad=low;bad.left.assign(60,0);
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry); // Intersection without an island arc.
    bad=low;bad.left.assign(60,20);
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry); // Right opening alone is insufficient.
    bad=low;for(int y=50;y<=53;++y) bad.left[y]=-1;
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry); // Missing lower trace is not clipping.
    bad=low;for(int y=50;y<=53;++y) bad.left[y]=10;
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry);
    bad=low;bad.left[18]=38;
    CHECK(!detect(bad.left,bad.right,80,s).lowCameraEntry); // Unsupported convex spike.

    Controller controller;
    auto run=[&](const Lane& lane,long long t,bool allow=true,bool paused=false,bool enter=false) {
        return controller.update(lane.left,lane.right,80,t,allow,true,s,{}, {},enter,paused);
    };
    for(int n=0;n<5;++n) CHECK(run(low,1000+n*40,false).stage==0);
    CHECK(run(low,1200).stage==0);
    CHECK(run(low,1240).stage==0);
    auto confirmed=run(low,1280);
    CHECK(confirmed.stage==1 && confirmed.lowCameraEntryConfirmed);
    CHECK(!confirmed.evidence.down.valid() && confirmed.evidence.leftBreaks==e.leftBreaks);
    CHECK(low.left==lowCamera().left && low.right==lowCamera().right);
    CHECK(confirmed.left[e.lowCameraMid.y]==e.lowCameraMid.x);
    CHECK(confirmed.left[e.lowCameraUp.y]==e.lowCameraUp.x);
    CHECK(confirmed.left[e.virtualDown.y]==0);
    CHECK(run(low,1320).stage==1); // Validation still required; no entry from the synthetic anchor.
    auto entered=run(low,1360,true,false,true);
    CHECK(entered.stage==2 && entered.right[e.lowCameraUp.y]==e.lowCameraUp.x);
    CHECK(entered.right[e.virtualDown.y]!=0); // Stage 2 uses the actual upper arc point.
    Lane noRightOpening=low;noRightOpening.right.assign(60,60);
    CHECK(!detect(noRightOpening.left,noRightOpening.right,80,s).lowCameraEntry);
    CHECK(detect(noRightOpening.left,noRightOpening.right,80,s).lowCameraUp.valid());
    CHECK(run(noRightOpening,1400).stage==2);
    CHECK(run(noRightOpening,1440).stage==2);
    CHECK(run(noRightOpening,1480).stage==2); // Real upper arc survives even when right opening disappears.

    // A stopped/empty camera never resets state and never consumes entry timeout.
    std::vector<int> empty;
    auto stopped=controller.update(empty,empty,0,10000,true,true,s,{}, {},false,true);
    CHECK(stopped.stage==2 && !stopped.changed && stopped.left==entered.left);
    Lane lost;lost.left.assign(60,-1);lost.right=lost.left;
    CHECK(run(lost,20000).stage==2);
    CHECK(run(low,20040).stage==2);
    CHECK(run(low,20100).stage==2);

    // 【新行为】低相机确认改为"lowCameraWindowMs 时间窗口内累计候选帧"：
    //   窗口内允许夹杂非候选帧（旧实现立即清零并要求严格连续，且把下限硬编码成 3，
    //   而实车环岛入口常常只闪现 1~2 帧，导致永远无法确认）；窗口过期才清零。
    {
        Settings sw=s;sw.lowCameraConfirmFrames=2;
        Controller cc;
        auto runw=[&](const Lane& lane,long long t) {
            return cc.update(lane.left,lane.right,80,t,true,true,sw,{}, {},false,false);
        };
        CHECK(runw(low,30000).stage==0);   // 第1帧候选 -> 计数1
        CHECK(runw(bad,30040).stage==0);   // 窗口内非候选 -> 保留计数
        CHECK(runw(low,30080).stage==1);   // 第2帧候选 -> 计数2 -> 确认
        cc.reset();
        CHECK(runw(low,31000).stage==0);   // 计数1
        CHECK(runw(bad,31000+sw.lowCameraWindowMs+300).stage==0); // 超窗口 -> 清零
        CHECK(runw(low,31000+sw.lowCameraWindowMs+600).stage==0);// 重新计数1
        CHECK(runw(low,31000+sw.lowCameraWindowMs+1000).stage==1);// 计数2 -> 确认
    }
    // 阈值下限为 1：单个强候选帧即可确认（实车环岛入口只闪现 1 帧）。
    {
        Settings s1=s;s1.lowCameraConfirmFrames=1;
        Controller c1;
        CHECK(c1.update(low.left,low.right,80,40000,true,true,s1,{}, {},false,false).stage==1);
    }
    // 窗口内位置发生跳变的候选同样计数（窗口累计不再要求位置严格一致）。
    {
        Settings s2=s;s2.lowCameraConfirmFrames=2;
        Controller c2;
        Lane shifted=low;shifted.right.assign(60,69);
        for(int y=27;y<=34;++y) shifted.right[y]=79;
        CHECK(detect(shifted.left,shifted.right,80,s).lowCameraEntry);
        CHECK(c2.update(low.left,low.right,80,50000,true,true,s2,{}, {},false,false).stage==0);
        CHECK(c2.update(shifted.left,shifted.right,80,50040,true,true,s2,{}, {},false,false).stage==1);
    }

    // Standard three-corner entry remains available and passes through every anchor.
    controller.reset();const Lane old=standard();
    CHECK(run(old,32000).stage==0);const auto oldResult=run(old,32040);
    CHECK(oldResult.stage==1 && !oldResult.lowCameraEntryConfirmed);
    for(Point p:{oldResult.evidence.down,oldResult.evidence.mid,oldResult.evidence.up})
        CHECK(oldResult.left[p.y]==p.x);

    // Sparse isolated line loss is not the continuous exit opening required by the FSM.
    Lane scattered=standard();
    for(int y=22;y<=40;y+=2) scattered.left[y]=scattered.right[y]=-1;
    CHECK(detect(scattered.left,scattered.right,80,s).bothMissingRows==1);
    for(int y=30;y<38;++y) scattered.left[y]=scattered.right[y]=-1;
    CHECK(detect(scattered.left,scattered.right,80,s).bothMissingRows>=8);

    // Thresholds and the geometric crop row scale with resolution.
    Lane big;
    for(int y=0;y<120;++y) {big.left.push_back(low.left[y/2]*2);big.right.push_back(low.right[y/2]*2);}
    const auto large=detect(big.left,big.right,160,s);
    CHECK(large.lowCameraEntry && !large.down.valid());
    CHECK(large.virtualDown.y==107 && large.rightOpeningRows>=12);

    // Probe confirmation is BEFORE steering, not only after leftward motion.
    TrackPolicy::EntranceProbe probe;TrackPolicy::ProbeSettings ps;
    auto probeStep=[&](long long t,bool seen,int x,int y,bool pause=false,bool fallback=false,bool just=false) {
        return probe.update(true,just,seen,false,false,true,pause,low.left,80,t,ps,fallback,&low.right,x,y);
    };
    CHECK(probeStep(40000,false,-1,-1,false,false,true).phase==1);
    CHECK(!probeStep(40040,true,60,35).started && !probe.controlsSteering());
    CHECK(!probeStep(40080,false,-1,-1).started && !probe.controlsSteering());
    CHECK(!probeStep(40120,true,60,35).started);
    CHECK(!probeStep(40160,true,60,35).started);
    CHECK(!probeStep(40200,true,69,35).started); // Unstable location.
    CHECK(!probeStep(40240,true,69,35).started);
    CHECK(probeStep(40280,true,69,35).started && probe.controlsSteering());
    probe.reset();probeStep(41000,false,-1,-1,false,false,true);
    CHECK(!probeStep(44000,false,-1,-1).started);
    CHECK(!probeStep(44040,false,60,35,false,true).started);
    CHECK(!probeStep(44080,false,60,35,false,true).started);
    CHECK(probeStep(44120,false,60,35,false,true).started);
    probe.reset();probeStep(45000,false,-1,-1,false,false,true);
    probeStep(45040,true,60,35);probeStep(45080,true,60,35);
    CHECK(!probeStep(49000,true,60,35,true).started && !probe.controlsSteering());
    CHECK(!probeStep(49040,true,60,35).started);
    CHECK(!probeStep(49080,true,60,35).started);
    CHECK(probeStep(49120,true,60,35).started);
    std::cout<<"PASS "<<checks<<" low-camera opening, virtual geometry, pre-steer debounce, dropout checks.\n";
}
