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
struct Guidance {bool valid=false,blocked=false;double target=0;};
Guidance guide(const ConeSequence::Observation& observation,const ConeSequence::Candidate* target,
    ConeSequence::Phase phase,const std::vector<int>& left,const std::vector<int>& right,int width,
    int marginPx80=4,int minCorridorPx80=12);
} }
