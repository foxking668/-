#include "../tools/parking_simulation.hpp"
#include <iostream>
#include <limits>
#include <set>

namespace {
using namespace car2026;
using namespace car2026::parking;
int checks=0;
void check(bool value,const char* text) {++checks;if(!value) throw std::runtime_error(text);}
bool close(double a,double b,double tolerance=1e-9) {return std::abs(a-b)<tolerance;}
template<class F> void rejects(F function,const char* text) {bool threw=false;try{function();}catch(const std::exception&){threw=true;}check(threw,text);}
struct Fixture {
    Geometry geometry=exampleGeometry();Plan plan=makePlan(geometry);Area area=exampleArea(geometry);
    Controller controller{plan,area,exampleLimits()};Feedback f;
    Fixture() {f.pose=plan.staging;f.poseValid=f.encoderValid=f.stationary=true;f.referenceId=1;}
    Decision tick(bool visible=false) {
        f.time+=.1;f.poseTime=f.encoderTime=f.rearTime=f.dashedTime=f.time;f.intersection=visible;
        return controller.update(f);
    }
    Decision prepareFirst() {
        for(int n=0;n<3;++n) tick(true);
        for(int n=0;n<5;++n) tick(false);
        for(int n=0;n<3;++n) tick(true);
        for(int n=0;n<3&&controller.phase()!=Phase::WaitStationary;++n) tick();
        return tick();
    }
    void startFirst() {
        const auto d=prepareFirst();check(d.phase==Phase::WaitStationary&&d.steering==geometry.firstCommand,"Stopped start prepares calibrated first command");
        check(controller.acknowledgeSteering(true,f),"Successful stationary write starts first reverse phase");
        while(controller.phase()==Phase::SteeringSettling) tick();
    }
    void finishFirst() {
        const auto& s=plan.segments[0];const int n=int(std::ceil(s.length/.004));
        for(int i=1;i<=n;++i) {
            f.pose=s.at(s.length*i/n);f.totalTravel+=s.length/n;f.speed=-.04;f.stationary=false;tick();
        }
        f.speed=0;f.stationary=true;
        for(int n=0;n<3&&controller.phase()!=Phase::WaitStationary;++n) tick();
        check(controller.phase()==Phase::WaitStationary,"First arc waits for stop before selecting branch straight");
        auto d=tick();check(d.steering==0,"Branch transition explicitly requests zero steering");
        check(controller.acknowledgeSteering(true,f),"Acknowledged zero starts branch phase");
        while(controller.phase()==Phase::SteeringSettling) tick();
    }
};
}
int main() {
    try {
        auto g=exampleGeometry();const auto p=makePlan(g);
        check(close(p.staging.forward,g.firstRadius*std::tan(g.branchAngle/2)),"Staging is circle tangent distance, not vehicle length");
        check(close(p.segments[0].end.heading,-g.branchAngle),"Reverse left first arc reaches negative body heading");
        for(const auto& segment:p.segments) {
            const auto end=segment.at(segment.length);
            check(close(end.forward,segment.end.forward)&&close(end.left,segment.end.left)&&close(end.heading,segment.end.heading),"Segment endpoint matches exact signed arc");
        }
        for(int i=0;i<2;++i) {
            const auto onBranch=i==0?p.segments[0].end:p.segments[1].end;
            check(close(std::sin(g.branchAngle)*onBranch.forward+std::cos(g.branchAngle)*onBranch.left,0),"Both tangent points lie on independently defined branch line");
        }
        check(close(p.aligned.left,g.bayOffset)&&close(p.aligned.heading,0),"Unequal-radius arcs finish on parallel bay axis");
        check(close(p.aligned.forward,-g.bayOffset/std::tan(g.branchAngle)-g.secondRadius*std::tan(g.branchAngle/2)),"Bay alignment position follows second tangent construction");
        check(close(p.terminal.forward,p.aligned.forward-g.finalReverse),"Final straight is reverse along aligned axis");
        auto mirror=makePlan(exampleGeometry(-1));
        check(close(mirror.aligned.forward,p.aligned.forward)&&close(mirror.aligned.left,-p.aligned.left),"Right bay mirrors left bay without changing staging travel");
        auto q=integrate({},-1,0);check(close(q.forward,-1)&&close(q.left,0),"Zero-curvature reverse translates backward");
        q=integrate({},-pi*.5,1);check(close(q.forward,-1)&&close(q.left,1)&&close(q.heading,-pi*.5),"Independent quarter circle verifies reverse heading and displacement");
        q=integrate({},.1,1e-12);check(close(q.forward,.1)&&std::abs(q.left)<1e-10,"Near-zero curvature is continuous without division cancellation");
        rejects([]{integrate({},1e308,1e308);},"Unbounded finite motion cannot overflow pose arithmetic");
        rejects([]{auto bad=exampleGeometry();bad.branchAngle=0;makePlan(bad);},"Zero branch angle cannot produce a tangent plan");
        rejects([]{auto bad=exampleGeometry();bad.firstRadius=bad.secondRadius=4;makePlan(bad);},"Too-large radii are infeasible, not hidden by negative straight length");
        rejects([]{auto bad=exampleGeometry();bad.firstCommand=5;makePlan(bad);},"Observed command sign must match first reverse side");
        rejects([]{auto bad=exampleGeometry();bad.secondRadius=std::numeric_limits<double>::quiet_NaN();makePlan(bad);},"NaN radius is rejected");
        rejects([]{makePlan(Geometry{});},"Known angle and dimensions alone cannot masquerade as measured turning response");
        auto noStraight=g;noStraight.bayOffset=(g.firstRadius+g.secondRadius)*(1-std::cos(g.branchAngle));
        check(makePlan(noStraight).segments[1].length<1e-9,"Exactly touching arcs can omit branch straight");
        auto area=exampleArea(g);check(fitsArea(p,area),"Full swept body fits explicitly supplied synthetic corridors");
        auto tight=area;tight.mainHalfWidth=tight.branchHalfWidth=tight.bayHalfWidth=.10;
        check(!fitsArea(p,tight),"Rear-axle path alone does not certify corner clearance");
        auto shallow=area;shallow.backForward=p.terminal.forward+.02;
        check(!fitsArea(p,shallow),"Vehicle rear and back-wall clearance can invalidate a geometrically connected path");
        check(!area.containsBody({0,0,std::numeric_limits<double>::quiet_NaN()},g),"Nonfinite vehicle pose cannot pass footprint check");
        rejects([&]{fitsArea(p,area,0);},"Sweep sampling step must be positive");
        rejects([&]{auto changed=p;changed.segments[0].length=std::numeric_limits<double>::quiet_NaN();fitsArea(changed,area);},"Public plan mutation cannot bypass validated trajectory geometry");
        rejects([&]{p.segments[0].at(std::numeric_limits<double>::quiet_NaN());},"NaN progress cannot silently clamp to a valid trajectory point");
        IntersectionCounter counter;
        for(int n=0;n<20;++n) check(!counter.update(true,true),"One persistent junction never triggers second crossing");
        check(counter.count()==1,"First junction is counted once");
        for(int n=0;n<8;++n) counter.update(false,false);
        check(!counter.update(true,true)&&counter.count()==1,"Missing observations do not rearm intersection counting");
        for(int n=0;n<5;++n) counter.update(false,true);
        check(!counter.update(true,true)&&!counter.update(true,true)&&counter.update(true,true),"Second distinct junction appearance triggers on confirmation only");
        for(int n=0;n<5;++n) counter.update(false,true);
        for(int n=0;n<3;++n) check(!counter.update(true,true),"Third appearance cannot retrigger second event");
        {
            Fixture t;t.startFirst();const auto before=t.f.totalTravel;t.f.pose=t.plan.segments[0].end;
            auto d=t.tick();check(d.phase==Phase::Fault&&d.speed==0&&!d.steering&&close(before,t.f.totalTravel),"Pose teleport without continuous feedback stops rather than switching by yaw");
            check(t.tick().phase==Phase::Fault,"Fault remains latched after recovered input");
        }
        {
            Fixture t;t.startFirst();t.finishFirst();t.f.pose.left+=.01;
            const auto d=t.tick();check(d.phase==Phase::ReverseBranch&&d.steering&&*d.steering>0,"Reverse correction for left offset turns wheels right, independent of forward PID");
            check(std::abs(*d.steering)<=5,"Feedback cannot extrapolate beyond observed command envelope");
        }
        {
            Fixture t;check(!t.tick().ownsMotion,"Parking does not claim route motion before second junction");t.f.poseTime=0;t.f.time+=.30;t.f.encoderTime=t.f.time;
            auto d=t.controller.update(t.f);check(d.phase==Phase::Fault&&d.speed==0,"Stale ground pose halts controller");
        }
        {
            Fixture t;t.tick();t.f.referenceId=2;
            check(t.tick().phase==Phase::Fault,"Changing world reference cannot silently replace the parking target");
        }
        {
            Fixture t;t.tick();t.f.totalTravel=.09;
            check(t.tick().phase==Phase::Fault,"Impossible distance step relative to elapsed time stops controller");
        }
        {
            Fixture t;t.f.pose.heading=.4;t.prepareFirst();
            check(t.controller.phase()==Phase::Fault,"Advance rejects excessive heading error instead of driving in an unintended direction");
        }
        {
            Fixture t;t.tick();t.f.time+=.1;t.f.encoderTime=t.f.time;
            check(t.controller.update(t.f).phase==Phase::Fault,"Repeated pose frame cannot count as independent confirmations");
        }
        {
            Fixture t;t.tick();t.f.rearWallValid=true;t.f.rearRange=.07;
            check(t.tick().phase==Phase::Fault,"Valid too-close wall echo stops even before final straight");
        }
        {
            Fixture t;t.tick();t.f.frontRangeValid=true;t.f.frontRange=.07;t.f.frontTime=t.f.time+.1;
            check(t.tick().phase==Phase::Fault,"Front obstacle range stops even with otherwise valid world pose");
        }
        {
            Fixture t;t.tick();t.f.rearWallValid=true;t.f.rearRange=std::numeric_limits<double>::quiet_NaN();
            check(t.tick().phase==Phase::Fault,"Valid flag cannot hide NaN sonar range");
        }
        {
            Fixture t;t.prepareFirst();t.controller.acknowledgeSteering(true,t.f);
            check(t.controller.phase()==Phase::SteeringSettling,"Write acknowledgement waits for linkage settle before authorizing travel");
            const auto wait=t.tick();check(wait.speed==0&&!wait.steering,"Settle wait emits neither motion nor repeated steering requests");
            t.f.stationary=false;check(t.tick().phase==Phase::Fault,"Moving before steering settle is complete locks stop");
        }
        {
            Fixture t;
            for(int n=0;n<3;++n)t.tick(true);for(int n=0;n<5;++n)t.tick();for(int n=0;n<3;++n)t.tick(true);for(int n=0;n<3;++n)t.tick();
            auto d=t.tick();check(d.steering.has_value(),"Steering request issued once after stop");
            check(!t.tick().steering,"Unacknowledged steering request is not resent");
            t.f.stationary=false;check(!t.controller.acknowledgeSteering(true,t.f)&&t.controller.phase()==Phase::Fault,"Successful software write does not authorize motion if stationary gate changed");
        }
        for(double side:{1.,-1.}) for(Pose error: {Pose{},Pose{0,.005,.5*pi/180},Pose{0,-.005,-.5*pi/180}}) {
            const auto geometry=exampleGeometry(side);std::set<Phase> phases;bool bounds=true;
            const auto r=simulate(geometry,exampleArea(geometry),error,[&](const Feedback&,const Decision& d){
                phases.insert(d.phase);if(d.steering) bounds=bounds&&std::abs(*d.steering)<=5;
            });
            if(!r.completed) std::cerr<<"simulation side="<<side<<" error="<<error.left<<" reason="<<r.fault<<'\n';
            check(r.completed,"Closed-loop synthetic parking completes for both sides and small initial errors");
            const auto end=makePlan(geometry).terminal;
            check(std::hypot(r.finalPose.forward-end.forward,r.finalPose.left-end.left)<=exampleLimits().positionTolerance,"Synthetic completion requires terminal position, not distance alone");
            check(bounds&&phases.count(Phase::ReverseFirst)&&phases.count(Phase::ReverseBranch)&&phases.count(Phase::ReverseSecond)&&phases.count(Phase::Stopping),"Simulation covers both arcs, branch, stopping and calibrated command bounds");
        }
        auto silent=[](const Feedback&,const Decision&){};
        bool branchUsed=false;
        const auto joined=simulate(noStraight,exampleArea(noStraight),{},[&](const Feedback&,const Decision& d){branchUsed=branchUsed||d.phase==Phase::ReverseBranch;});
        check(joined.completed,"Touching circles complete without a negative or artificial straight segment");
        check(!branchUsed,"Zero-length branch phase is skipped explicitly");
        auto missing=simulate(g,area,{},silent,false,true);
        check(!missing.completed&&missing.fault=="EXPECTED_BACK_WALL_RANGE_UNAVAILABLE","No expected wall echo cannot falsely complete parking");
        auto noDash=simulate(g,area,{},silent,true,false);
        check(!noDash.completed&&noDash.fault=="PARKING_TIMEOUT","Range arrival without final dashed confirmation holds stopped until timeout");
        std::cout<<"PASS "<<checks<<" reverse parking checks; synthetic metric feedback, no hardware adapters\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
