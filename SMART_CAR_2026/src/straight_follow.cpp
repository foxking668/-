#include "straight_follow.hpp"

namespace car2026 {
void StraightImageTarget::validate() const {
    const double farX=nearX+headingFeature;
    // Require the complete target line to stay in the fitted interval.
    const double slope=headingFeature/(.45-.84);
    const double topX=nearX+slope*(.40-.84),bottomX=nearX+slope*(.90-.84);
    if(!std::isfinite(nearX) || !std::isfinite(headingFeature) || nearX<0 || nearX>1 ||
       !std::isfinite(farX) || farX<0 || farX>1 || !std::isfinite(topX) || !std::isfinite(bottomX) ||
       topX<0 || topX>1 || bottomX<0 || bottomX>1)
        throw std::runtime_error("Straight target must describe a visible normalized image line");
}
StraightLineFollower::StraightLineFollower(const Params& params,StraightImageTarget target)
    : params_(params),target_(target) {
    params_.validate();target_.validate();
}
void StraightLineFollower::reset() {
    lastFrameTime_=lastNow_=-1;lastSuggestion_=filteredLateral_=filteredHeading_=0;
    hasFeatures_=hasFilter_=false;pathRows_=0;confirmed_=0;lastFeatures_=StraightImageFeatures{};
}
StraightFollowObservation StraightLineFollower::reject(const char* reason,double age) {
    confirmed_=0;hasFilter_=false;lastSuggestion_=0;
    // Preserve the last accepted path shape/location. A lost frame does not
    // authorize silently accepting a distant branch as the same reference.
    StraightFollowObservation result;result.state=reason;result.frameAge=age;return result;
}
StraightFollowObservation StraightLineFollower::observe(const Observation& observation,double frameTime,double now) {
    if(!std::isfinite(frameTime) || !std::isfinite(now) || frameTime<0 || now<frameTime ||
       (lastNow_>=0 && now<lastNow_)) return reject("INVALID_TIME");
    const double age=now-frameTime;
    if(lastFrameTime_>=0 && frameTime<=lastFrameTime_)
        return reject(frameTime==lastFrameTime_ ? "REPEATED_FRAME" : "BACKWARD_TIME",age);
    const double dt=lastFrameTime_<0 ? 0 : frameTime-lastFrameTime_;
    lastFrameTime_=frameTime;lastNow_=now;
    if(age>params_.frame_timeout_s) return reject("STALE_FRAME",age);
    if(dt>params_.frame_timeout_s) return reject("FRAME_GAP",age);
    if(!observation.frameValid) return reject("INVALID_FRAME",age);
    const auto& path=observation.blackPath;
    if(path.ambiguous) return reject("AMBIGUOUS",age);
    if(path.discontinuous) return reject("DISCONTINUOUS",age);
    if(!std::isfinite(path.confidence) || path.confidence<std::max(.65,params_.line_min_confidence) ||
       path.confidence>1 || !std::isfinite(path.lateral) || std::abs(path.lateral)>.5 ||
       !std::isfinite(path.heading) || std::abs(path.heading)>1) return reject("LOW_QUALITY",age);
    if(pathRows_ && path.x.size()!=pathRows_) return reject("PATH_SIZE_CHANGED",age);
    if(path.x.size()<24 || !imageBandAvailable(path,.84) ||
       !imageBandAvailable(path,.65) || !imageBandAvailable(path,.45)) return reject("PARTIAL_PATH",age);
    StraightImageFeatures features;
    if(!fitStraightImageFeatures(path,features)) return reject("NOT_STRAIGHT",age);
    constexpr double maximumFeatureStep=.06,confirmationTolerance=.015,alignmentTolerance=.015;
    if(hasFeatures_ && (std::abs(features.lateral-lastFeatures_.lateral)>maximumFeatureStep ||
                       std::abs(features.heading-lastFeatures_.heading)>maximumFeatureStep))
        return reject("FEATURE_JUMP",age);
    if(confirmed_ && confirmed_<int(params_.confirm_frames) &&
       (std::abs(features.lateral-lastFeatures_.lateral)>confirmationTolerance ||
        std::abs(features.heading-lastFeatures_.heading)>confirmationTolerance)) confirmed_=0;
    lastFeatures_=features;hasFeatures_=true;pathRows_=path.x.size();
    confirmed_=std::min(confirmed_+1,int(params_.confirm_frames));
    StraightFollowObservation result;
    result.frameAge=age;result.isStraight=result.hasErrors=true;result.features=features;
    result.confirmedFrames=confirmed_;
    result.lateralError=features.lateral-(target_.nearX-.5);
    result.headingFeatureError=features.heading-target_.headingFeature;
    if(confirmed_<int(params_.confirm_frames)) {result.state="WAIT_STRAIGHT";return result;}
    if(!hasFilter_) {
        filteredLateral_=result.lateralError;filteredHeading_=result.headingFeatureError;hasFilter_=true;
    } else {
        const double alpha=dt/(params_.straight_filter_s+dt);
        filteredLateral_+=alpha*(result.lateralError-filteredLateral_);
        filteredHeading_+=alpha*(result.headingFeatureError-filteredHeading_);
    }
    result.filteredLateralError=filteredLateral_;result.filteredHeadingError=filteredHeading_;
    const auto deadband=[&](double error) {
        return std::abs(error)<=params_.straight_deadband_image ? 0 :
               error-std::copysign(params_.straight_deadband_image,error);
    };
    const double limit=std::min(params_.straight_max_command,params_.max_steer_deg);
    const double desired=clamp(params_.straight_lateral_gain*deadband(filteredLateral_)+
                               params_.straight_heading_gain*deadband(filteredHeading_),-limit,limit);
    const double step=params_.straight_command_rate_s*dt;
    result.suggestedCommand=clamp(desired,lastSuggestion_-step,lastSuggestion_+step);
    lastSuggestion_=result.suggestedCommand;
    result.hasSuggestion=true;result.state="TRACKING_STRAIGHT";
    // Raw errors determine alignment; smoothing cannot hide current offset.
    result.aligned=std::abs(result.lateralError)<=alignmentTolerance &&
                   std::abs(result.headingFeatureError)<=alignmentTolerance;
    return result;
}
} // namespace car2026
