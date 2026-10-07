#pragma once
#include "reverse_parking.hpp"

namespace car2026 { namespace parking {
// SYNTHETIC example only: radii, bay offset, road widths, back wall and range
// below are NOT measurements of the user's track. Never used by live tools.
inline Geometry exampleGeometry(double side=1) {
    Geometry g;g.firstRadius=.6;g.secondRadius=.5;g.bayOffset=side*.6;
    g.finalReverse=.20;g.firstCommand=-side*5;g.secondCommand=side*5;return g;
}
inline Area exampleArea(const Geometry& g) {
    Area a;a.mainHalfWidth=.35;a.branchHalfWidth=.40;a.bayHalfWidth=.25;
    a.branchStartAlong=-.40;a.branchEndAlong=std::abs(g.bayOffset)/std::sin(g.branchAngle)+.40;
    a.mouthForward=-std::abs(g.bayOffset)/std::tan(g.branchAngle);a.backForward=a.mouthForward-.80;return a;
}
inline Limits exampleLimits() { Limits l;l.targetRearRange=.15;return l; }
struct SimulationResult { bool completed=false;int steps=0;Pose finalPose;std::string fault; };
template<class Sink>
SimulationResult simulate(const Geometry& geometry,const Area& area,Pose initialError,Sink sink,
                          bool rearEcho=true,bool dashed=true) {
    auto plan=makePlan(geometry);auto limits=exampleLimits();Controller controller(plan,area,limits);
    Feedback f;f.pose={-.15+initialError.forward,initialError.left,initialError.heading};
    f.poseValid=f.encoderValid=true;f.referenceId=1;
    double command=0,velocity=0,rest=.8;const double dt=.05;
    for(int step=0;step<3000;++step) {
        f.time=step*dt;f.poseTime=f.encoderTime=f.rearTime=f.dashedTime=f.time;
        // Two separate appearances, with confirmed disappearance in between.
        f.intersection=step<5||(step>=12&&step<17);
        f.speed=velocity;f.stationary=rest>=.8;
        f.rearWallValid=rearEcho;f.rearRange=limits.targetRearRange+std::max(0.,f.pose.forward-plan.terminal.forward);
        f.terminalDashed=dashed&&std::hypot(f.pose.forward-plan.terminal.forward,f.pose.left-plan.terminal.left)<.05;
        const auto d=controller.update(f);sink(f,d);
        if(d.finished) return {d.phase==Phase::Complete,step+1,f.pose,controller.fault()};
        if(d.steering) {
            command=*d.steering;
            if(d.phase==Phase::WaitStationary) controller.acknowledgeSteering(true,f);
        }
        velocity=d.speed;
        const double firstCurvature=std::copysign(1/geometry.firstRadius,geometry.bayOffset);
        const bool firstSide=command*geometry.firstCommand>=0;
        const double curvature=firstSide?firstCurvature*command/geometry.firstCommand:
            -std::copysign(1/geometry.secondRadius,geometry.bayOffset)*command/geometry.secondCommand;
        f.pose=integrate(f.pose,velocity*dt,curvature);f.totalTravel+=std::abs(velocity)*dt;
        rest=velocity==0?rest+dt:0;
    }
    return {false,3000,f.pose,"SIMULATION_LIMIT"};
}
}} // namespace car2026::parking
