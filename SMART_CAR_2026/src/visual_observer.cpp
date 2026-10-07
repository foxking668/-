#include "visual_observer.hpp"

namespace car2026 {
namespace {
constexpr double lateralGain=8,headingFeatureGain=12,commandRate=3;
constexpr double stableFeatureTolerance=.015,maximumFeatureStep=.06,imageDeadband=.005;
// A fixed image target is meaningful here only for a local straight segment.
// Limits are normalized image units, not physical curvature or vehicle yaw.
constexpr double fitTop=.40,fitBottom=.90,maximumFitResidual=.008;
struct StraightImageFeatures { double lateral=0,heading=0; };
bool fitStraightImageFeatures(const Path& path,StraightImageFeatures& features) {
    const int h=int(path.x.size());
    double sumY=0,sumX=0,sumYY=0,sumYX=0;int count=0,total=0;
    for(int y=int(h*fitTop);y<=int(h*fitBottom);++y) {
        ++total;const double x=path.x[y];
        if(!std::isfinite(x) || x>1 || x< -1) return false;
        if(x<0) continue;
        const double row=double(y)/h;
        sumY+=row;sumX+=x;sumYY+=row*row;sumYX+=row*x;++count;
    }
    if(count*5<total*4) return false;
    const double denominator=count*sumYY-sumY*sumY;
    if(denominator<=1e-12) return false;
    const double slope=(count*sumYX-sumY*sumX)/denominator;
    const double intercept=(sumX-slope*sumY)/count;
    for(int y=int(h*fitTop);y<=int(h*fitBottom);++y)
        if(path.x[y]>=0 && std::abs(path.x[y]-(intercept+slope*double(y)/h))>maximumFitResidual)
            return false;
    features.lateral=intercept+slope*.84-.5;
    features.heading=slope*(.45-.84);
    return std::isfinite(features.lateral) && std::isfinite(features.heading);
}
bool bandAvailable(const Path& path,double ratio) {
    const int h=int(path.x.size()),center=int(h*ratio),radius=std::max(2,h/30);
    int available=0,total=0;
    for(int y=std::max(0,center-radius);y<std::min(h,center+radius+1);++y) {
        const double x=path.x[y];++total;
        if(!std::isfinite(x) || x>1 || x< -1) return false;
        if(x>=0) ++available;
    }
    return total && available*2>=total;
}
}
ManualMotion parseManualMotion(const std::string& value) {
    if(value=="forward") return ManualMotion::Forward;
    if(value=="reverse") return ManualMotion::Reverse;
    throw std::runtime_error("Manual motion must be forward or reverse");
}
const char* manualMotionName(ManualMotion motion) {
    return motion==ManualMotion::Forward ? "forward" : "reverse";
}
VisualSteeringObserver::VisualSteeringObserver(const Params& params,ManualMotion motion)
    : motion_(motion),timeout_(params.frame_timeout_s),
      minimumConfidence_(std::max(.65,params.line_min_confidence)),
      maximumCommand_(std::min(5.,params.max_steer_deg)),confirmationFrames_(int(params.confirm_frames)) {
    params.validate();
}
void VisualSteeringObserver::reset() {
    lastFrameTime_=-1;lastSuggestion_=0;stableFrames_=0;
    hasReference_=false;hasLastFeature_=false;referenceId_=0;
}
SteeringObservation VisualSteeringObserver::reject(const char* reason,double age) {
    stableFrames_=0;lastSuggestion_=0;hasLastFeature_=false;
    SteeringObservation result;result.state=reason;result.frameAge=age;
    result.hasReference=hasReference_;result.referenceId=referenceId_;
    return result;
}
SteeringObservation VisualSteeringObserver::observe(const Observation& observation,double frameTime,double now) {
    if(!std::isfinite(frameTime) || !std::isfinite(now) || frameTime<0 || now<frameTime)
        return reject("INVALID_TIME");
    const double age=now-frameTime;
    if(lastFrameTime_>=0 && frameTime<=lastFrameTime_)
        return reject(frameTime==lastFrameTime_ ? "REPEATED_FRAME" : "BACKWARD_TIME",age);
    const double dt=lastFrameTime_<0 ? 0 : frameTime-lastFrameTime_;
    lastFrameTime_=frameTime;
    if(age>timeout_) return reject("STALE_FRAME",age);
    if(dt>timeout_) return reject("FRAME_GAP",age);
    if(!observation.frameValid) return reject("INVALID_FRAME",age);
    const auto& path=observation.blackPath;
    if(path.ambiguous) return reject("AMBIGUOUS",age);
    if(path.discontinuous) return reject("DISCONTINUOUS",age);
    if(!std::isfinite(path.confidence) || path.confidence<minimumConfidence_ || path.confidence>1 ||
       !std::isfinite(path.lateral) || std::abs(path.lateral)>.5 ||
       !std::isfinite(path.heading) || std::abs(path.heading)>1) return reject("LOW_QUALITY",age);
    if(path.x.size()<24 || !bandAvailable(path,.84) || !bandAvailable(path,.65) || !bandAvailable(path,.45))
        return reject("PARTIAL_REFERENCE",age);
    StraightImageFeatures features;
    if(!fitStraightImageFeatures(path,features)) return reject("UNSUITABLE_REFERENCE",age);
    if(hasLastFeature_ && (std::abs(features.lateral-lastLateral_)>maximumFeatureStep ||
                          std::abs(features.heading-lastHeading_)>maximumFeatureStep))
        return reject("FEATURE_JUMP",age);
    lastLateral_=features.lateral;lastHeading_=features.heading;hasLastFeature_=true;
    if(!hasReference_) {
        if(stableFrames_ && (std::abs(features.lateral-candidateLateral_)>stableFeatureTolerance ||
                            std::abs(features.heading-candidateHeading_)>stableFeatureTolerance)) stableFrames_=0;
        ++stableFrames_;
        candidateLateral_+=(features.lateral-candidateLateral_)/stableFrames_;
        candidateHeading_+=(features.heading-candidateHeading_)/stableFrames_;
        if(stableFrames_>=confirmationFrames_) {
            referenceLateral_=candidateLateral_;referenceHeading_=candidateHeading_;
            hasReference_=true;referenceId_=++referenceSequence_;
        }
    } else stableFrames_=std::min(stableFrames_+1,confirmationFrames_);
    SteeringObservation result;result.frameAge=age;result.hasReference=hasReference_;result.referenceId=referenceId_;
    if(stableFrames_<confirmationFrames_) {
        result.state=hasReference_ ? "RECONFIRM_REFERENCE" : "WAIT_REFERENCE";
        return result;
    }
    result.state="TRACKING";result.hasSuggestion=true;
    result.lateralError=features.lateral-referenceLateral_;
    result.headingFeatureError=features.heading-referenceHeading_;
    const double lateral=std::abs(result.lateralError)<imageDeadband ? 0 : result.lateralError;
    const double heading=std::abs(result.headingFeatureError)<imageDeadband ? 0 : result.headingFeatureError;
    const double sign=motion_==ManualMotion::Forward ? 1 : -1;
    // Retained for recorded diagnostic compatibility, NOT a validated reverse
    // feedback law. Image features are coupled; this sign is not actuation approval.
    const double target=clamp(sign*(lateralGain*lateral+headingFeatureGain*heading),-maximumCommand_,maximumCommand_);
    result.suggestedCommand=clamp(target,lastSuggestion_-commandRate*dt,lastSuggestion_+commandRate*dt);
    lastSuggestion_=result.suggestedCommand;
    return result;
}
} // namespace car2026
