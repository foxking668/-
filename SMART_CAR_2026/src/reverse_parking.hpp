#pragma once
#include "core.hpp"
#include <array>
#include <optional>

namespace car2026 { namespace parking {
// World origin: main/branch intersection. +forward follows initial vehicle
// heading; +left is vehicle left. Pose refers to REAR AXLE midpoint, not camera.
// All values below are metric estimates, never IMU measurements or image pixels.
struct Pose { double forward=0,left=0,heading=0; }; // metres, metres, radians
Pose integrate(const Pose& pose,double signedTravel,double leftCurvature);
struct Geometry {
    double wheelbase=.22,width=.19,frontExtent=.32,rearExtent=.035;
    double branchAngle=36.7*pi/180;
    double firstRadius=0,secondRadius=0; // observed motion radii; not PWM angles
    double bayOffset=0,finalReverse=0;
    // Explicit measured command/curvature correspondence. Positive command
    // turns front wheels right on this car; first left-back arc uses negative.
    double firstCommand=0,secondCommand=0;
    void validate() const;
};
struct Area {
    double mainHalfWidth=0,branchHalfWidth=0,bayHalfWidth=0;
    double branchStartAlong=0,branchEndAlong=0; // actual drivable corridor caps, including junction overlap
    double mouthForward=0,backForward=0,clearance=.02;
    void validate(const Geometry& geometry) const;
    bool containsBody(const Pose& pose,const Geometry& geometry,double extraMargin=0) const;
};
struct Segment {
    const char* name="";Pose start,end;double length=0,curvature=0;
    Pose at(double progress) const;
};
struct Plan {
    Geometry geometry;std::array<Segment,4> segments;
    Pose staging,aligned,terminal;
    double reverseLength=0;
};
// Main and bay axes are parallel. The two circles are tangent to the SAME
// branch line; circles with unequal radii are supported, on either bay side.
Plan makePlan(const Geometry& geometry);
// Conservative swept-body check: covers between trajectory samples as well
// as body interior. May reject tight but feasible paths; never a point-car test.
bool fitsArea(const Plan& plan,const Area& area,double step=.01);

class IntersectionCounter {
public:
    explicit IntersectionCounter(int confirmation=3,int clearance=5);
    bool update(bool visible,bool observationFresh); // true only on count #2
    int count() const { return count_; }
private:
    int confirmation_,clearance_,hits_=0,clears_=0,count_=0;bool armed_=true;
};
enum class Phase { AwaitIntersection,Advance,WaitStationary,SteeringSettling,ReverseFirst,
    ReverseBranch,ReverseSecond,ReverseStraight,Stopping,Complete,Fault };
const char* phaseName(Phase phase);
struct Feedback {
    double time=0,poseTime=0,encoderTime=0,totalTravel=0,speed=0;
    Pose pose;bool poseValid=false,encoderValid=false,stationary=false;
    uint64_t referenceId=0;
    bool intersection=false;
    bool frontRangeValid=false;double frontRange=0,frontTime=0;
    // Valid only if this echo is from the expected vertical back wall.
    bool rearWallValid=false;double rearRange=0,rearTime=0;
    bool terminalDashed=false;double dashedTime=0;
};
struct Limits {
    double freshness=.25,stageTimeout=25,totalTimeout=120;
    double steeringSettle=.30; // wait after software write; not physical angle feedback
    double advanceSpeed=.05,reverseSpeed=.04,minimumReverseSpeed=.015;
    double positionTolerance=.025,headingTolerance=3*pi/180;
    double maximumPoseJump=.15,maximumTravelStep=.10;
    double lateralGain=4,headingGain=2,maximumTrackingError=.12;
    double targetRearRange=0,minimumRearRange=.08,minimumFrontRange=.08;
    int confirmation=3,intersectionClearance=5;
    void validate() const;
};
struct Decision {
    Phase phase=Phase::AwaitIntersection;
    double speed=0;
    // Optional software request; caller must gate/write it and acknowledge.
    // No hardware writes occur in this module. speed is also only a request.
    std::optional<double> steering;
    bool finished=false;std::string reason;
    bool ownsMotion=false; // route executor retains motion until second junction
};
class Controller {
public:
    Controller(Plan plan,Area area,Limits limits);
    Decision update(const Feedback& feedback);
    // Acknowledge only an actual successful servo write while still at rest.
    bool acknowledgeSteering(bool succeeded,const Feedback& feedback);
    Phase phase() const { return phase_; }
    const std::string& fault() const { return fault_; }
private:
    Plan plan_;Area area_;Limits limits_;IntersectionCounter intersections_;
    Phase phase_=Phase::AwaitIntersection,next_=Phase::ReverseFirst;
    bool initialized_=false,pending_=false;int confirmations_=0;
    uint64_t reference_=0;
    double lastTime_=0,lastPoseTime_=-1,lastTravel_=0,stageTime_=0,taskTime_=0,stageTravel_=0;
    double lastConfirmedDashed_=-1;
    Pose lastPose_;std::string fault_;double pendingCommand_=0;
    void enter(Phase phase,const Feedback& feedback);
    void fail(const std::string& reason);
    bool fresh(double sample,double now) const;
    bool endpoint(const Feedback& feedback,const Pose& goal) const;
    double commandForCurvature(double curvature) const;
    Decision result(double speed=0,std::optional<double> steering={}) const;
    void prepare(Phase next,double command,const Feedback& feedback);
};
}} // namespace car2026::parking
