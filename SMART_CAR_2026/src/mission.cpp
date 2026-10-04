#include "core.hpp"

namespace car2026 {
const char* stageName(Stage s) {
    switch(s) {
#define S(n) case Stage::n:return #n;
    S(Depart) S(ToCones) S(Cones) S(ToRing) S(RingEntry) S(RingLap) S(RingExit)
    S(ToCross) S(CrossApproach) S(CrossWait) S(CrossPass) S(ToGarage)
    S(GarageAlign) S(GarageAdvance) S(GarageReverse) S(Complete) S(Fault)
#undef S
    }
    return "Invalid";
}
void Mission::enter(Stage next,const Telemetry& t) {
    stage_=next;stageTime_=t.time;stageDistance_=t.distance;hits_=clears_=0;
    // Paths from a different manoeuvre must never leak into the next one.
    heldPath_=Path{};lastPathTime_=-1e9;
}
bool Mission::confirmed(bool condition) {
    hits_=condition ? hits_+1 : 0;
    return hits_>=int(params_.confirm_frames);
}
bool Mission::clearConfirmed(bool condition) {
    clears_=condition ? clears_+1 : 0;
    return clears_>=int(params_.clear_frames);
}
double Mission::follow(const Path& path,double& speed,const Telemetry& t) {
    const Path* chosen=&path;
    if(path.confidence>=params_.line_min_confidence) { heldPath_=path;lastPathTime_=t.time; }
    else if(heldPath_.valid() && t.time-lastPathTime_<=params_.line_hold_s) {
        chosen=&heldPath_;speed=std::min(speed,params_.cross_speed);
    } else { fail("Path lost or blocked: "+std::string(stageName(stage_)));speed=0;return 0; }
    speed/=1+params_.curvature_slow_gain*std::abs(chosen->curvature);
    return clamp(params_.lateral_gain*chosen->lateral+params_.heading_gain*chosen->heading,
                 -params_.max_steer_deg,params_.max_steer_deg);
}
Command Mission::update(const Observation& o,const Telemetry& t,const Path& avoidance) {
    Command cmd;
    if(!initialized_) { initialized_=true;stageTime_=lastTime_=t.time;stageDistance_=t.distance; }
    const double dt=clamp(t.time-lastTime_,0.,.2);lastTime_=t.time;
    if(!std::isfinite(t.time)||!std::isfinite(t.distance)||!std::isfinite(t.speed)||!t.encoderValid)
        fail("Invalid telemetry");
    if(!o.frameValid && stage_!=Stage::Complete && stage_!=Stage::Fault) fail("Invalid camera frame");
    const double elapsed=t.time-stageTime_,travel=t.distance-stageDistance_;
    bool ring=stage_==Stage::RingEntry||stage_==Stage::RingLap||stage_==Stage::RingExit;
    bool parking=stage_==Stage::GarageAlign||stage_==Stage::GarageAdvance||stage_==Stage::GarageReverse;
    if((ring||parking) && (!t.yawValid || (params_.require_imu && !t.yawMeasured))) fail("Fresh yaw required for roundabout/parking");
    const double limit=stage_==Stage::CrossWait ? params_.cross_wait_timeout_s : ring ? params_.ring_timeout_s :
                       parking ? params_.parking_timeout_s : stage_==Stage::Cones ? params_.cone_timeout_s : params_.stage_timeout_s;
    if(elapsed>limit && stage_!=Stage::Complete && stage_!=Stage::Fault) fail("Stage timeout: "+std::string(stageName(stage_)));
    cmd.speed=params_.cruise_speed;
    auto black=[&] { cmd.steer=follow(o.blackPath,cmd.speed,t); };
    auto road=[&] { cmd.steer=follow(o.roadPath,cmd.speed,t); };
    switch(stage_) {
    case Stage::Depart:
        cmd.speed=params_.parking_speed;black();
        if(stage_!=Stage::Fault && travel>=params_.garage_depart_m) enter(Stage::ToCones,t);
        break;
    case Stage::ToCones:
        if(confirmed(!o.cones.empty())) {enter(Stage::Cones,t);cmd.speed=params_.cone_speed;cmd.steer=follow(avoidance,cmd.speed,t);}
        else black();
        break;
    case Stage::Cones:
        cmd.speed=params_.cone_speed;cmd.steer=follow(avoidance,cmd.speed,t);
        if(stage_!=Stage::Fault && clearConfirmed(o.cones.empty() && o.blackPath.confidence>=params_.line_min_confidence) && travel>=params_.cone_min_travel_m)
            enter(Stage::ToRing,t);
        break;
    case Stage::ToRing:
        cmd.speed=params_.ring_speed;
        if(confirmed(o.ringCandidate && o.roadPath.valid())) {
            if(!t.yawValid || (params_.require_imu && !t.yawMeasured)) {fail("Roundabout yaw unavailable");break;}
            ringYaw_=t.yaw;enter(Stage::RingEntry,t);road();
            cmd.steer=clamp(cmd.steer+params_.ring_side*params_.ring_entry_bias_deg,-params_.max_steer_deg,params_.max_steer_deg);
        } else if(o.blackPath.confidence>=params_.line_min_confidence) black(); else road();
        break;
    case Stage::RingEntry:
        cmd.speed=params_.ring_speed;road();
        cmd.steer=clamp(cmd.steer+params_.ring_side*params_.ring_entry_bias_deg*clamp(1-travel/params_.ring_entry_distance_m,0.,1.),-params_.max_steer_deg,params_.max_steer_deg);
        if(stage_!=Stage::Fault && travel>=params_.ring_entry_distance_m) enter(Stage::RingLap,t);
        break;
    case Stage::RingLap: {
        cmd.speed=params_.ring_speed;road();
        // Positive progress corresponds to the configured turn direction. Clockwise yaw is negative.
        const double progress=-params_.ring_side*(t.yaw-ringYaw_);
        if(progress < -35) {fail("Roundabout rotating opposite configured direction");break;}
        if(progress>params_.ring_max_yaw_deg) {fail("Roundabout exit missed");break;}
        if(stage_!=Stage::Fault && confirmed(progress>=params_.ring_min_yaw_deg && travel>=params_.ring_min_travel_m &&
                                           o.blackPath.confidence>=params_.line_min_confidence)) enter(Stage::RingExit,t);
        break;
    }
    case Stage::RingExit:
        cmd.speed=params_.ring_speed;black();
        if(stage_!=Stage::Fault && travel>=params_.ring_exit_min_travel_m && confirmed(o.blackPath.confidence>=params_.line_min_confidence)) enter(Stage::ToCross,t);
        break;
    case Stage::ToCross:
        if(confirmed(o.stripe)) {
            enter(Stage::CrossApproach,t);crossStopRequired_=params_.stop_every_cross!=0 || o.bar;
            // Hold approach heading when stripes occlude the guide line.
            if(o.blackPath.confidence>=params_.line_min_confidence) heldPath_=o.blackPath;
            else if(o.roadPath.confidence>=params_.line_min_confidence) heldPath_=o.roadPath;
            lastPathTime_=t.time;
            cmd.speed=params_.cross_speed;cmd.steer=lastSteer_;
        } else black();
        break;
    case Stage::CrossApproach:
        cmd.speed=params_.cross_speed;
        crossStopRequired_=crossStopRequired_ || o.bar;
        if(travel>params_.cross_approach_max_m) {fail("Crossing stop position not observed");break;}
        if(o.blackPath.confidence>=params_.line_min_confidence) black();
        else if(o.roadPath.confidence>=params_.line_min_confidence) road();
        else cmd.steer=lastSteer_; // bounded by crossing distance and stage timeout
        if(o.stripe && o.stripeY>=params_.cross_near_y_ratio) {
            if(crossStopRequired_) {
                // Decelerate progressively before the stop row, avoiding reverse braking.
                cmd.speed=params_.cross_speed*clamp((params_.cross_stop_y_ratio-o.stripeY)/
                    std::max(.02,params_.cross_stop_y_ratio-params_.cross_near_y_ratio),0.,1.);
                if(o.stripeY>=params_.cross_stop_y_ratio) {enter(Stage::CrossWait,t);stopStarted_=spoke_=false;cmd.speed=0;}
            } else if(o.barObservable) enter(Stage::CrossPass,t);
            else {cmd.speed=0; /* Unknown bar state must not be interpreted as clear. */}
        }
        break;
    case Stage::CrossWait:
        cmd.speed=0;cmd.steer=0;
        if(std::abs(t.speed)<.015) {
            if(!stopStarted_) {stopStarted_=true;stopTime_=t.time;}
            if(!spoke_) {cmd.speak=true;spoke_=true;}
        } else {stopStarted_=false;clears_=0;}
        if(clearConfirmed(o.barObservable && !o.bar) && stopStarted_ && t.time-stopTime_>=params_.cross_stop_s)
            enter(Stage::CrossPass,t);
        break;
    case Stage::CrossPass:
        cmd.speed=params_.cross_speed;
        if(o.bar) {enter(Stage::CrossWait,t);stopStarted_=spoke_=false;cmd.speed=0;break;}
        if(o.blackPath.confidence>=params_.line_min_confidence) black();
        else if(o.roadPath.confidence>=params_.line_min_confidence) road();
        else cmd.steer=lastSteer_;
        if(travel>=params_.cross_pass_m && clearConfirmed(!o.stripe && o.blackPath.confidence>=params_.line_min_confidence)) enter(Stage::ToGarage,t);
        if(travel>params_.cross_pass_m*2.5) fail("Cannot reacquire line after crossing");
        break;
    case Stage::ToGarage:
        black();
        if(stage_!=Stage::Fault && travel>=params_.garage_min_after_cross_m && confirmed(o.garage)) {
            ++completedLaps_;
            if(completedLaps_<2) enter(Stage::ToCones,t);
            else {
                if(!t.yawValid || (params_.require_imu && !t.yawMeasured)) {fail("Parking yaw unavailable");break;}
                // Reverse axis points OUT of the bay: calibrate this angle from the actual junction.
                parkingYaw_=t.yaw+params_.garage_outward_yaw_delta_deg;
                enter(Stage::GarageAlign,t);
            }
        }
        break;
    case Stage::GarageAlign: {
        cmd.speed=params_.parking_speed;
        double error=wrapDegrees(parkingYaw_-t.yaw);
        cmd.steer=clamp(-error*params_.parking_heading_gain,-params_.max_steer_deg,params_.max_steer_deg);
        if(confirmed(std::abs(error)<4) && travel>=params_.garage_align_distance_m) enter(Stage::GarageAdvance,t);
        break;
    }
    case Stage::GarageAdvance:
        cmd.speed=params_.parking_speed;
        cmd.steer=clamp(-wrapDegrees(parkingYaw_-t.yaw)*params_.parking_heading_gain,-params_.max_steer_deg,params_.max_steer_deg);
        if(travel>=params_.garage_advance_m) {
            cmd.speed=0;
            if(std::abs(t.speed)<.015) enter(Stage::GarageReverse,t);
        }
        break;
    case Stage::GarageReverse:
        cmd.speed=-params_.parking_speed;
        // Steering-to-yaw relation changes sign in reverse.
        cmd.steer=clamp(wrapDegrees(parkingYaw_-t.yaw)*params_.parking_heading_gain,-params_.max_steer_deg,params_.max_steer_deg);
        if(travel>=params_.garage_reverse_m) {cmd.speed=0; if(std::abs(t.speed)<.015) enter(Stage::Complete,t);}
        break;
    case Stage::Complete: cmd.speed=0;cmd.steer=0;break;
    case Stage::Fault: cmd.speed=0;cmd.steer=0;break;
    }
    if(stage_==Stage::Fault) {cmd.speed=0;cmd.steer=0;cmd.reason=fault_;}
    if(stage_==Stage::Complete) {cmd.speed=0;cmd.steer=0;cmd.reason="Parking manoeuvre finished by calibrated odometry";}
    const double step=params_.steer_rate_deg_s*dt;
    cmd.steer=clamp(cmd.steer,lastSteer_-step,lastSteer_+step);
    if(stage_==Stage::Fault||stage_==Stage::Complete||stage_==Stage::CrossWait) cmd.steer=0;
    lastSteer_=cmd.steer;cmd.stage=stage_;cmd.completedLaps=completedLaps_;
    return cmd;
}
} // namespace car2026
