#include "NewTrack/Roundabout.hpp"
#include <cassert>
#include <iostream>
#include <random>

using namespace NewTrack::Roundabout;
struct Lane { std::vector<int> l,r; };
Lane normal() {
    Lane a; a.l.assign(60,20);a.r.resize(60);
    for(int y=0;y<60;++y) a.r[y]=43+y/2;
    return a;
}
Lane approach() {
    Lane a=normal();
    for(int y=0;y<10;++y) a.l[y]=0;
    for(int y=10;y<=28;++y) a.l[y]=26-std::abs(y-18);
    for(int y=29;y<=43;++y) a.l[y]=0;
    return a;
}
Lane entry() {
    Lane a=normal();
    for(int y=0;y<18;++y) a.l[y]=30;
    for(int y=18;y<=32;++y) a.l[y]=0;
    return a;
}
Lane outgoing(int missing=8,bool corner=true) {
    Lane a=normal();
    for(int y=24;y<24+missing;++y) a.l[y]=a.r[y]=-1;
    if(corner) for(int y=32;y<=36;++y) a.r[y]=78;
    return a;
}
int checks=0;
void checkAt(bool ok,int line) { ++checks;if(!ok) {std::cerr<<"Failed check "<<checks<<" at line "<<line<<"\n";std::abort();} }
#define check(value) checkAt((value),__LINE__)
int main() {
    Settings s; Controller c;long long now=0;
    // 【测试更新】三点几何入场已删除，改为用模板命中(templateHit)进入 stage 1。
    auto step=[&](Lane a,int dt=40,bool allow=true,bool on=true,bool tmpl=false) {
        now+=dt; return c.update(a.l,a.r,80,now,allow,on,s,{},{},false,false,tmpl);
    };
    // 【测试更新】三点几何入场已删除。现在直接走 enterNow 路径进 stage 2（它会设置 entryAnchor_）。
    auto stepEnter=[&](Lane a,int dt=40) {
        now+=dt; return c.update(a.l,a.r,80,now,true,true,s,{},{},true,false,false);
    };
    const Lane original=approach();
    const auto e=detect(original.l,original.r,80,s);
    std::cout<<"Approach D/M/U="<<e.down.y<<"/"<<e.mid.y<<"/"<<e.up.y<<" breaks="<<e.leftBreaks<<"\n";
    check(e.leftBreaks==2&&e.down.valid()&&e.mid.valid()&&e.up.valid());
    check(e.down.y>e.mid.y&&e.mid.y>e.up.y&&e.rightContinuous);
    Lane steep=approach();
    for(int y=0;y<60;++y) steep.r[y]=40+std::max(0,std::min(5,y-22))*5;
    check(detect(steep.l,steep.r,80,s).rightContinuous); // Sustained steep growth is connected.
    Lane rightStep=approach();
    for(int y=0;y<60;++y) rightStep.r[y]=y<25?45:65;
    check(!detect(rightStep.l,rightStep.r,80,s).rightContinuous); // Abrupt one-step discontinuity is rejected.
    for(int i=0;i<20;++i) check(step(normal()).stage==0);
    Lane spike=normal();spike.l[35]=1;
    for(int i=0;i<5;++i) check(step(spike).stage==0);
    Lane cross=approach();for(int y=29;y<=43;++y) cross.r[y]=79;
    for(int i=0;i<5;++i) check(step(cross).stage==0);
    for(int i=0;i<5;++i) check(step(entry()).stage==0); // Single break is not a circle.
    check(step(approach(),40,false).stage==0);
    check(step(approach()).stage==0); // Debounce.
    // 【测试更新】用模板命中触发入场（原三点几何路径已删除）。
    auto result=stepEnter(approach());check(result.stage==2);   // 【测试更新】直接入环（模板/几何入场均已调整）
    result=stepEnter(approach());check(result.stage==2);
    const Point anchor=result.evidence.up;
    check(anchor.y==17&&result.right[17]==30&&result.right[59]==79);
    // 【新入环补线】左边线固定为画面最左侧(x=0)，右边线是"左上拐点->右下角"的连线向上外推。
    // 旧实现是把上锚点以上的左右边界全部清成 -1；现在左边线整列取 0、右边线按同一直线外推。
    for(int y=0;y<anchor.y;++y) check(result.left[y]==0&&result.right[y]>=0);
    check(result.left[anchor.y]==0&&result.right[anchor.y]==anchor.x);
    Lane invalid;invalid.l.assign(60,-1);invalid.r=invalid.l;
    for(int i=0;i<5;++i) check(step(invalid).stage==2);
    check(step(normal()).stage==2);check(step(normal()).stage==2);
    result=step(normal());check(result.stage==3);
    for(int y=0;y<20;++y) check(result.left[y]==-1&&result.right[y]==-1);
    check(result.left[59]==20&&result.right[59]==72); // Clear TOP, preserve bottom.
    for(int i=0;i<5;++i) check(step(outgoing()).stage==3); // Exit search delay.
    check(step(outgoing(5),900).stage==3); // Strictly more than five missing rows.
    check(step(outgoing(8,false)).stage==3); // Both conditions required.
    check(step(outgoing()).stage==3);result=step(outgoing());check(result.stage==4);
    check(result.evidence.rightOut.valid()&&result.right[20]>=0);
    for(int y=0;y<20;++y) check(result.right[y]==-1);
    check(step(normal()).stage==4);check(step(normal()).stage==4);
    check(step(normal()).stage==5);
    for(int i=0;i<10;++i) check(step(normal()).stage==5); // Must see the recovery pair first.
    result=step(entry());check(result.stage==5&&result.left[25]>0);
    for(int i=0;i<5;++i) check(step(invalid).stage==5); // Camera dropout cannot finish.
    check(step(normal()).stage==5);check(step(normal()).stage==5);
    result=step(normal());check(result.stage==0&&result.completed);
    for(int i=0;i<6;++i) check(step(approach(),500).stage==0); // Once per lap.
    c.reset();check(step(approach()).stage==0);check(step(approach()).stage==1);
    check(step(approach(),40,true,false).stage==0); // Hot disable resets state.
    c.reset();step(approach());step(approach());step(entry());step(entry());
    check(step(entry(),s.entryTimeoutMs+1).stage==3); // Time is not a fabricated exit.
    c.reset();step(approach());step(approach());
    check(step(approach(),s.approachTimeoutMs+1).stage==0);
    std::vector<int> b(60,-1);
    check(fitSegment(b,{0,20},{79,59},80)&&b[20]==0&&b[59]==79);
    check(!fitSegment(b,{0,20},{10,20},80));
    check(!fitSegment(b,{0,-1},{79,59},80));
    check(!fitSegment(b,{0,20},{80,59},80));
    // Different sizes and malformed arrays are checked without fixed 60/80 indexing.
    std::mt19937 rng(7);
    for(int n=0;n<1000;++n) {
        const int h=rng()%130,w=12+rng()%200;
        std::vector<int> l(h),r(h);
        for(int& x:l) x=int(rng()%(w+4))-2;
        for(int& x:r) x=int(rng()%(w+4))-2;
        if(n%9==0&&!r.empty()) r.pop_back();
        const auto z=c.update(l,r,w,now+n*50,true,true,s);
        check(z.stage>=0&&z.stage<=5&&z.center.size()==l.size());
    }
    // Equivalent geometry at 160x120 scales the pixel/row thresholds.
    Lane big;
    for(int y=0;y<120;++y) {big.l.push_back(original.l[y/2]*2);big.r.push_back(original.r[y/2]*2);}
    const auto eb=detect(big.l,big.r,160,s);
    check(eb.down.valid()&&eb.mid.valid()&&eb.up.valid()&&eb.rightContinuous);
    std::cout<<"PASS "<<checks<<" checks: states 0->1->2->3->4->5->0, false positives, dropout, sizes.\n";
}
