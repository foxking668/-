#pragma once
#include "straight_image_features.hpp"

namespace car2026 {
// Explicit target for forward motion: normalized x at row .84 and x(.45)-x(.84).
// Supplied by the caller; never captured from an arbitrary starting pose.
// A nominal (.5,0) target is a diagnostic assumption, not vehicle calibration.
struct StraightImageTarget {
    double nearX=.5,headingFeature=0;
    void validate() const;
};
struct StraightFollowObservation {
    std::string state="WAIT_STRAIGHT";
    bool isStraight=false,hasErrors=false,hasSuggestion=false,aligned=false;
    int confirmedFrames=0;
    double frameAge=0,lateralError=0,headingFeatureError=0;
    double filteredLateralError=0,filteredHeadingError=0,suggestedCommand=0;
    StraightImageFeatures features;
};
// Forward-only diagnostic controller. Reuses Vision's black path; no hardware,
// IMU, speed commands, or mission transitions. Call reset() only for an explicit
// new tracking session, never automatically to absorb an observed displacement.
class StraightLineFollower {
public:
    StraightLineFollower(const Params& params,StraightImageTarget target);
    StraightFollowObservation observe(const Observation& observation,double frameTime,double now);
    void reset();
private:
    Params params_;StraightImageTarget target_;
    double lastFrameTime_=-1,lastNow_=-1,lastSuggestion_=0;
    double filteredLateral_=0,filteredHeading_=0;
    StraightImageFeatures lastFeatures_;
    bool hasFeatures_=false,hasFilter_=false;
    size_t pathRows_=0;
    int confirmed_=0;
    StraightFollowObservation reject(const char* reason,double age=0);
};
} // namespace car2026
