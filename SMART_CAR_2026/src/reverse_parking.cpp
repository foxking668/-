#include "reverse_parking.hpp"

namespace car2026 { namespace parking {
namespace {
void require(bool ok,const char* reason) { if(!ok) throw std::runtime_error(reason); }
bool finitePose(const Pose& p) {
    return std::isfinite(p.forward)&&std::isfinite(p.left)&&std::abs(p.forward)<=100&&std::abs(p.left)<=100&&
           std::isfinite(p.heading)&&std::abs(p.heading)<=pi+1e-9;
}
double angle(double value) { return wrapDegrees(value*180/pi)*pi/180; }
double distance(const Pose& a,const Pose& b) { return std::hypot(a.forward-b.forward,a.left-b.left); }
}
Pose integrate(const Pose& p,double travel,double curvature) {
    require(finitePose(p)&&std::isfinite(travel)&&std::isfinite(curvature)&&std::abs(travel)<=100&&std::abs(curvature)<=100,"Invalid rear-axle motion");
    const double half=travel*curvature*.5;
    // Midpoint/sinc form is continuous at zero curvature, unlike division by k.
    const double scale=std::abs(half)<1e-8 ? 1-half*half/6 : std::sin(half)/half;
    return {p.forward+travel*scale*std::cos(p.heading+half),
            p.left+travel*scale*std::sin(p.heading+half),angle(p.heading+2*half)};
}
void Geometry::validate() const {
    for(double v : {wheelbase,width,frontExtent,rearExtent,branchAngle,firstRadius,secondRadius,
                    bayOffset,finalReverse,firstCommand,secondCommand}) require(std::isfinite(v),"Nonfinite parking geometry");
    require(wheelbase>=.08&&wheelbase<=.6&&width>=.10&&width<=.5,"Invalid vehicle dimensions");
    require(frontExtent>=wheelbase&&frontExtent<=.8&&rearExtent>0&&rearExtent<=.4,"Invalid body overhang");
    require(branchAngle>=5*pi/180&&branchAngle<=75*pi/180,"Branch angle outside supported parallel-bay geometry");
    require(firstRadius>=.08&&firstRadius<=5&&secondRadius>=.08&&secondRadius<=5,"Missing/invalid observed turning radii");
    require(std::abs(bayOffset)>=width&&std::abs(bayOffset)<=5&&finalReverse>0&&finalReverse<=2,"Invalid bay offset or final reverse length");
    const double side=bayOffset>0?1:-1;
    require(firstCommand*side<0&&secondCommand*side>0&&std::abs(firstCommand)<=15&&std::abs(secondCommand)<=15,
            "Commands must match confirmed right-positive steering sign and 15-command bound");
}
Pose Segment::at(double progress) const {
    require(std::isfinite(progress)&&std::isfinite(length)&&length>=0,"Invalid trajectory progress");
    return integrate(start,-clamp(progress,0.,length),curvature);
}
Plan makePlan(const Geometry& g) {
    g.validate();Plan p;p.geometry=g;
    const double a=g.branchAngle,side=g.bayOffset>0?1:-1;
    const double branch=(std::abs(g.bayOffset)-(g.firstRadius+g.secondRadius)*(1-std::cos(a)))/std::sin(a);
    require(branch>=-1e-10,"Turning radii cannot fit this lateral offset at the confirmed branch angle");
    p.staging={g.firstRadius*std::tan(a*.5),0,0};
    const double lengths[]={g.firstRadius*a,std::max(0.,branch),g.secondRadius*a,g.finalReverse};
    const double curvatures[]={side/g.firstRadius,0,-side/g.secondRadius,0};
    const char* names[]={"ReverseFirst","ReverseBranch","ReverseSecond","ReverseStraight"};
    Pose pose=p.staging;
    for(int i=0;i<4;++i) {
        p.segments[i]={names[i],pose,{},lengths[i],curvatures[i]};
        pose=p.segments[i].at(lengths[i]);p.segments[i].end=pose;p.reverseLength+=lengths[i];
    }
    p.aligned=p.segments[2].end;p.terminal=pose;
    require(distance(p.aligned,{p.aligned.forward,g.bayOffset,0})<1e-8&&std::abs(p.aligned.heading)<1e-8,
            "Parallel-bay endpoint consistency failure");
    return p;
}
void Area::validate(const Geometry& g) const {
    for(double v : {mainHalfWidth,branchHalfWidth,bayHalfWidth,branchStartAlong,branchEndAlong,mouthForward,backForward,clearance})
        require(std::isfinite(v),"Nonfinite parking area");
    require(mainHalfWidth>g.width*.5&&branchHalfWidth>g.width*.5&&bayHalfWidth>g.width*.5,
            "Corridor narrower than vehicle or area not supplied");
    require(mainHalfWidth<=2&&branchHalfWidth<=2&&bayHalfWidth<=2&&clearance>=0&&clearance<=.10,
            "Invalid corridor widths or clearance");
    require(backForward<mouthForward&&mouthForward-backForward<=3,"Invalid back-wall/mouth positions");
    require(branchEndAlong>branchStartAlong&&branchEndAlong-branchStartAlong<=10,"Actual branch corridor caps must be supplied");
}
bool Area::containsBody(const Pose& p,const Geometry& g,double extra) const {
    g.validate();validate(g);
    if(!finitePose(p)||!std::isfinite(extra)||extra<0) return false;
    // Rectangular cells cover the full body. Each cell's containing disk must
    // fit wholly in at least one corridor. Conservative at corridor overlaps.
    const int nx=int(std::ceil((g.frontExtent+g.rearExtent)/.02)),ny=int(std::ceil(g.width/.02));
    const double dx=(g.frontExtent+g.rearExtent)/nx,dy=g.width/ny;
    const double margin=clearance+extra+std::hypot(dx,dy)*.5;
    const double c=std::cos(p.heading),s=std::sin(p.heading),a=g.branchAngle;
    const double side=g.bayOffset>0?1:-1;
    for(int i=0;i<nx;++i) for(int j=0;j<ny;++j) {
        const double x=-g.rearExtent+(i+.5)*dx,y=-g.width*.5+(j+.5)*dy;
        const double worldX=p.forward+c*x-s*y,worldY=p.left+s*x+c*y;
        // Extend back-wall plane conservatively across all corridors.
        if(worldX<backForward+margin) return false;
        const bool main=std::abs(worldY)+margin<=mainHalfWidth;
        const double along=-std::cos(a)*worldX+side*std::sin(a)*worldY;
        const bool branch=std::abs(std::sin(a)*worldX+side*std::cos(a)*worldY)+margin<=branchHalfWidth&&
                          along>=branchStartAlong+margin&&along<=branchEndAlong-margin;
        const bool bay=worldX<=mouthForward-margin&&std::abs(worldY-g.bayOffset)+margin<=bayHalfWidth;
        if(!main&&!branch&&!bay) return false;
    }
    return true;
}
bool fitsArea(const Plan& p,const Area& a,double step) {
    p.geometry.validate();a.validate(p.geometry);
    // Public plan values cannot bypass geometry validation after construction.
    const auto canonical=makePlan(p.geometry);
    const auto samePose=[](const Pose& x,const Pose& y) {
        return finitePose(x)&&distance(x,y)<1e-9&&std::abs(angle(x.heading-y.heading))<1e-9;
    };
    require(samePose(p.staging,canonical.staging)&&samePose(p.aligned,canonical.aligned)&&samePose(p.terminal,canonical.terminal)&&
            std::isfinite(p.reverseLength)&&std::abs(p.reverseLength-canonical.reverseLength)<1e-9,"Parking plan changed after construction");
    for(int i=0;i<4;++i) {
        const auto& s=p.segments[i];const auto& expected=canonical.segments[i];
        require(std::isfinite(s.length)&&std::isfinite(s.curvature)&&std::abs(s.length-expected.length)<1e-9&&
                std::abs(s.curvature-expected.curvature)<1e-9&&samePose(s.start,expected.start)&&samePose(s.end,expected.end),
                "Parking segment changed after construction");
    }
    require(std::isfinite(step)&&step>0&&step<=.05,"Swept check step must be within 0..5cm");
    const double radius=std::hypot(std::max(p.geometry.frontExtent,p.geometry.rearExtent),p.geometry.width*.5);
    for(const auto& segment:p.segments) {
        // This inflation bounds translation and corner rotation between samples.
        const double extra=step*(1+radius*std::abs(segment.curvature));
        const int n=std::max(1,int(std::ceil(segment.length/step)));
        for(int i=0;i<=n;++i) if(!a.containsBody(segment.at(segment.length*i/n),p.geometry,extra)) return false;
    }
    return true;
}
IntersectionCounter::IntersectionCounter(int confirmation,int clearance)
    :confirmation_(confirmation),clearance_(clearance) {
    require(confirmation>=1&&confirmation<=20&&clearance>=2&&clearance<=30,"Invalid intersection confirmation counts");
}
bool IntersectionCounter::update(bool visible,bool valid) {
    if(count_>=2) return false;
    if(!valid) {hits_=clears_=0;return false;} // absence of data is not leaving a junction
    if(armed_) {
        hits_=visible?hits_+1:0;
        if(hits_>=confirmation_) {count_=std::min(2,count_+1);armed_=false;hits_=clears_=0;return count_==2;}
    } else {
        clears_=visible?0:clears_+1;
        if(clears_>=clearance_) {armed_=true;hits_=clears_=0;}
    }
    return false;
}
const char* phaseName(Phase p) {
    switch(p) {
#define P(n) case Phase::n:return #n;
    P(AwaitIntersection) P(Advance) P(WaitStationary) P(SteeringSettling) P(ReverseFirst) P(ReverseBranch)
    P(ReverseSecond) P(ReverseStraight) P(Stopping) P(Complete) P(Fault)
#undef P
    }
    return "Invalid";
}
void Limits::validate() const {
    for(double v:{freshness,stageTimeout,totalTimeout,steeringSettle,advanceSpeed,reverseSpeed,minimumReverseSpeed,
                  positionTolerance,headingTolerance,maximumPoseJump,maximumTravelStep,lateralGain,
                  headingGain,maximumTrackingError,targetRearRange,minimumRearRange,minimumFrontRange})
        require(std::isfinite(v),"Nonfinite controller limits");
    require(freshness>=.05&&freshness<=.5&&stageTimeout>=5&&stageTimeout<=60&&totalTimeout>=stageTimeout&&totalTimeout<=300,"Invalid parking time limits");
    require(steeringSettle>=.1&&steeringSettle<=1,"Invalid steering settle wait");
    require(advanceSpeed>0&&advanceSpeed<=.10&&reverseSpeed>0&&reverseSpeed<=.08&&
            minimumReverseSpeed>0&&minimumReverseSpeed<=reverseSpeed,"Invalid parking speed limits");
    require(positionTolerance>=.005&&positionTolerance<=.05&&headingTolerance>0&&headingTolerance<=5*pi/180,
            "Invalid endpoint tolerances");
    require(maximumPoseJump>positionTolerance&&maximumPoseJump<=.3&&maximumTravelStep>0&&maximumTravelStep<=.15,
            "Invalid feedback jump limits");
    require(lateralGain>0&&lateralGain<=10&&headingGain>0&&headingGain<=5&&maximumTrackingError>positionTolerance&&maximumTrackingError<=.2,
            "Invalid tracking gains/error limits");
    require(minimumRearRange>=.02&&targetRearRange>minimumRearRange&&targetRearRange<=.4,"Back-wall stop range must be calibrated");
    require(minimumFrontRange>=.02&&minimumFrontRange<=.3,"Invalid front obstacle stop range");
    IntersectionCounter counter(confirmation,intersectionClearance);(void)counter;
}
Controller::Controller(Plan p,Area a,Limits limits)
    :plan_(std::move(p)),area_(a),limits_(limits),intersections_(limits.confirmation,limits.intersectionClearance) {
    limits_.validate();require(fitsArea(plan_,area_),"Swept vehicle body cannot fit the supplied parking area");
}
bool Controller::fresh(double sample,double now) const { return std::isfinite(sample)&&sample>=0&&sample<=now&&now-sample<=limits_.freshness; }
void Controller::enter(Phase p,const Feedback& f) { phase_=p;stageTime_=f.time;stageTravel_=f.totalTravel;confirmations_=0; }
void Controller::fail(const std::string& reason) { phase_=Phase::Fault;pending_=false;fault_=reason; }
Decision Controller::result(double speed,std::optional<double> command) const {
    return {phase_,phase_==Phase::Fault||phase_==Phase::Complete?0:speed,
            phase_==Phase::Fault?std::optional<double>{}:command,
            phase_==Phase::Fault||phase_==Phase::Complete,fault_,phase_!=Phase::AwaitIntersection};
}
bool Controller::endpoint(const Feedback& f,const Pose& goal) const {
    return distance(f.pose,goal)<=limits_.positionTolerance&&std::abs(angle(f.pose.heading-goal.heading))<=limits_.headingTolerance;
}
void Controller::prepare(Phase next,double command,const Feedback& f) {
    next_=next;pendingCommand_=command;pending_=false;enter(Phase::WaitStationary,f);
}
double Controller::commandForCurvature(double k) const {
    const auto& g=plan_.geometry;
    const double first=g.bayOffset>0?1/g.firstRadius:-1/g.firstRadius;
    const double second=-std::copysign(1/g.secondRadius,first);
    // Interpolation inside the observed response envelope only. Extrapolation
    // beyond the measured left/right command pair is forbidden.
    const bool firstSide=k*first>=0;
    const double bound=firstSide?first:second,cmd=firstSide?g.firstCommand:g.secondCommand;
    return cmd*clamp(k/bound,0.,1.);
}
Decision Controller::update(const Feedback& f) {
    if(phase_==Phase::Fault||phase_==Phase::Complete) return result();
    if(!std::isfinite(f.time)||f.time<0||!std::isfinite(f.totalTravel)||f.totalTravel<0||
       !std::isfinite(f.speed)||std::abs(f.speed)>.20||!finitePose(f.pose)||!f.poseValid||!f.encoderValid||
       !f.referenceId||!fresh(f.poseTime,f.time)||!fresh(f.encoderTime,f.time)) {fail("INVALID_OR_STALE_METRIC_FEEDBACK");return result();}
    if(initialized_&&(f.time<=lastTime_||f.time-lastTime_>limits_.freshness||f.poseTime<=lastPoseTime_||
       f.totalTravel<lastTravel_||f.totalTravel-lastTravel_>limits_.maximumTravelStep||
       f.totalTravel-lastTravel_>.20*(f.time-lastTime_)+.005||
       distance(f.pose,lastPose_)>limits_.maximumPoseJump||std::abs(angle(f.pose.heading-lastPose_.heading))>15*pi/180||
       f.referenceId!=reference_)) {fail("FEEDBACK_DISCONTINUITY_OR_REFERENCE_CHANGED");return result();}
    if(!initialized_) {initialized_=true;reference_=f.referenceId;stageTime_=taskTime_=f.time;stageTravel_=f.totalTravel;}
    lastTime_=f.time;lastPoseTime_=f.poseTime;lastTravel_=f.totalTravel;lastPose_=f.pose;
    if(!area_.containsBody(f.pose,plan_.geometry)) {fail("VEHICLE_BODY_OUTSIDE_AREA");return result();}
    if(f.frontRangeValid&&fresh(f.frontTime,f.time)&&
       (!std::isfinite(f.frontRange)||f.frontRange<=limits_.minimumFrontRange)) {
        fail("FRONT_OBSTACLE_RANGE_LIMIT");return result();
    }
    if(f.rearWallValid&&fresh(f.rearTime,f.time)) {
        if(!std::isfinite(f.rearRange)||f.rearRange<=0) {fail("INVALID_REAR_RANGE");return result();}
        if(f.rearRange<=limits_.minimumRearRange) {fail("REAR_CLEARANCE_LIMIT");return result();}
    }
    if(phase_!=Phase::AwaitIntersection&&(f.time-stageTime_>limits_.stageTimeout||f.time-taskTime_>limits_.totalTimeout)) {
        fail("PARKING_TIMEOUT");return result();
    }
    if(phase_==Phase::AwaitIntersection) {
        if(intersections_.update(f.intersection,true)) {taskTime_=f.time;enter(Phase::Advance,f);}
        return result();
    }
    if(phase_==Phase::Advance) {
        if(endpoint(f,plan_.staging)) {
            if(++confirmations_>=limits_.confirmation) prepare(Phase::ReverseFirst,plan_.geometry.firstCommand,f);
            return result();
        }
        confirmations_=0;
        if(f.pose.forward>plan_.staging.forward+limits_.positionTolerance) {fail("STAGING_OVERSHOOT");return result();}
        const double e=f.pose.left,heading=angle(f.pose.heading);
        if(std::abs(e)>limits_.maximumTrackingError||std::abs(heading)>15*pi/180) {fail("MAIN_AXIS_TRACKING_ERROR");return result();}
        return result(std::min(limits_.advanceSpeed,std::max(.01,plan_.staging.forward-f.pose.forward)),
                      commandForCurvature(-limits_.lateralGain*e-limits_.headingGain*heading));
    }
    if((phase_==Phase::WaitStationary||phase_==Phase::SteeringSettling)&&next_==Phase::Complete&&
       (!endpoint(f,plan_.terminal)||!f.rearWallValid||!fresh(f.rearTime,f.time)||
           !std::isfinite(f.rearRange)||f.rearRange>limits_.targetRearRange+.02||!f.terminalDashed||!fresh(f.dashedTime,f.time))) {
        fail("TERMINAL_CONFIRMATION_LOST_BEFORE_ZERO");return result();
    }
    if(phase_==Phase::SteeringSettling) {
        if(!f.stationary||std::abs(f.speed)>=.015||f.totalTravel-stageTravel_>1e-9) {
            fail("MOVED_BEFORE_STEERING_SETTLED");return result();
        }
        if(f.time-stageTime_>=limits_.steeringSettle) enter(next_,f);
        return result();
    }
    if(phase_==Phase::WaitStationary) {
        if(pending_) return result(); // no repeated request, even while waiting for acknowledgement
        if(f.stationary&&std::abs(f.speed)<.015) {pending_=true;return result(0,pendingCommand_);}
        return result();
    }
    const bool terminal=phase_==Phase::ReverseStraight||phase_==Phase::Stopping;
    if(terminal&&(!f.rearWallValid||!fresh(f.rearTime,f.time)||!std::isfinite(f.rearRange)||f.rearRange<=0)) {
        fail("EXPECTED_BACK_WALL_RANGE_UNAVAILABLE");return result();
    }
    const bool dashed=f.terminalDashed&&fresh(f.dashedTime,f.time);
    if(phase_==Phase::Stopping) {
        const bool ok=endpoint(f,plan_.terminal)&&dashed&&f.rearRange<=limits_.targetRearRange+.02&&f.stationary&&std::abs(f.speed)<.015;
        if(!ok) confirmations_=0;
        else if(f.dashedTime>lastConfirmedDashed_) {++confirmations_;lastConfirmedDashed_=f.dashedTime;}
        if(confirmations_>=limits_.confirmation) prepare(Phase::Complete,0,f);
        return result(); // never resumes reverse after reaching range limit
    }
    const int index=phase_==Phase::ReverseFirst?0:phase_==Phase::ReverseBranch?1:phase_==Phase::ReverseSecond?2:3;
    const auto& segment=plan_.segments[index];const double travel=f.totalTravel-stageTravel_;
    if(terminal&&f.rearRange<=limits_.targetRearRange) {enter(Phase::Stopping,f);return result();}
    if(travel>segment.length+.08) {fail("SEGMENT_OVERSHOOT");return result();}
    if(endpoint(f,segment.end)&&travel>=segment.length-limits_.positionTolerance&&
       (!terminal||f.rearRange<=limits_.targetRearRange+.02)) {
        if(++confirmations_>=limits_.confirmation) {
            if(index==0) {
                if(plan_.segments[1].length<1e-9) prepare(Phase::ReverseSecond,plan_.geometry.secondCommand,f);
                else prepare(Phase::ReverseBranch,0,f);
            } else if(index==1) prepare(Phase::ReverseSecond,plan_.geometry.secondCommand,f);
            else if(index==2) prepare(Phase::ReverseStraight,0,f);
            else enter(Phase::Stopping,f);
        }
        return result();
    }
    confirmations_=0;const auto reference=segment.at(travel);
    const double dx=f.pose.forward-reference.forward,dy=f.pose.left-reference.left;
    const double lateral=-std::sin(reference.heading)*dx+std::cos(reference.heading)*dy;
    const double heading=angle(f.pose.heading-reference.heading);
    if(distance(f.pose,reference)>limits_.maximumTrackingError||std::abs(heading)>15*pi/180) {
        fail("REVERSE_TRAJECTORY_TRACKING_ERROR");return result();
    }
    const double curvature=segment.curvature-limits_.lateralGain*lateral+limits_.headingGain*heading;
    // Reverse feedback signs follow signed-travel geometry, not forward PID.
    double speed=std::min(limits_.reverseSpeed,std::max(limits_.minimumReverseSpeed,(segment.length-travel)*.5));
    if(terminal) speed=std::min(speed,std::max(limits_.minimumReverseSpeed,(f.rearRange-limits_.targetRearRange)*.5));
    return result(-speed,commandForCurvature(curvature));
}
bool Controller::acknowledgeSteering(bool success,const Feedback& f) {
    if(phase_!=Phase::WaitStationary||!pending_) return false;
    pending_=false;
    if(!success||!std::isfinite(f.time)||f.time<lastTime_||f.time-stageTime_>limits_.stageTimeout||f.time-taskTime_>limits_.totalTimeout||
       !f.stationary||!std::isfinite(f.speed)||std::abs(f.speed)>=.015||!f.poseValid||!finitePose(f.pose)||
       !f.encoderValid||!fresh(f.poseTime,f.time)||!fresh(f.encoderTime,f.time)||f.referenceId!=reference_||
       !std::isfinite(f.totalTravel)||std::abs(f.totalTravel-lastTravel_)>1e-9||distance(f.pose,lastPose_)>limits_.positionTolerance*.2||
       std::abs(angle(f.pose.heading-lastPose_.heading))>limits_.headingTolerance*.2||!area_.containsBody(f.pose,plan_.geometry)||
       (f.rearWallValid&&fresh(f.rearTime,f.time)&&(!std::isfinite(f.rearRange)||f.rearRange<=limits_.minimumRearRange))||
       (f.frontRangeValid&&fresh(f.frontTime,f.time)&&(!std::isfinite(f.frontRange)||f.frontRange<=limits_.minimumFrontRange))||
       (next_==Phase::Complete&&(!endpoint(f,plan_.terminal)||!f.rearWallValid||!fresh(f.rearTime,f.time)||
        !std::isfinite(f.rearRange)||f.rearRange<=limits_.minimumRearRange||f.rearRange>limits_.targetRearRange+.02||
        !f.terminalDashed||!fresh(f.dashedTime,f.time)))) {
        fail("STEERING_WRITE_FAILED_OR_REST_CHANGED");return false;
    }
    enter(Phase::SteeringSettling,f);return true;
}
}} // namespace car2026::parking
