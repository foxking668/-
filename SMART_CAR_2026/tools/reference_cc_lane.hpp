#pragma once
#include "rehearsal_line.hpp"
#include "../legacy/cc_lane/steering_geometry.hpp"
#include <opencv2/core.hpp>
namespace car2026 { namespace capture {
struct CcLaneObservation {
    LineObservation line{};
    float error=0;
    bool centerBlackReliable=false,singleEdgeUsed=false;
    double rawTarget=0;
    int bilateralRows=0,leftOnlyRows=0,rightOnlyRows=0;
};
// Single image-thread adapter to cc(1)'s ordinary NewTrack lane pipeline.
class ReferenceCcLane {
public:
    void reset();
    CcLaneObservation analyze(const cv::Mat& frame);
};
inline double ccLaneServoCommand(const CcLaneObservation& observation,double limit) {
    if(!observation.line.reliable || !std::isfinite(observation.error) || !std::isfinite(limit) || limit<=0 || limit>ccParkingSteerLimit)
        throw std::runtime_error("Invalid CC lane observation or servo limit");
    // cc sends angle=90+error. Our servo API receives the relative error;
    // its existing mechanical offset and native pulse conversion remain intact.
    return std::clamp(double(observation.error),-limit,limit);
}
}}
