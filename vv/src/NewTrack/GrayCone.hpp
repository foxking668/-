#pragma once
#include "ConeSequence.hpp"
#include <opencv2/core.hpp>

namespace NewTrack { namespace GrayCone {
struct Result {
    ConeSequence::Observation observation;
    cv::Mat mask;
    int threshold=0,rejected=0,components=0;
};
// Shape/contrast candidates only. Temporal entry/pass decisions belong to ConeSequence.
Result detect(const cv::Mat& image,const ConeSequence::Settings& settings);
} }
