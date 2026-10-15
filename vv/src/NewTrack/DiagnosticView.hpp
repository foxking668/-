#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include "Roundabout.hpp"
namespace NewTrack { namespace DiagnosticView {
inline cv::Size canvasSize() { return {960,800}; }
struct Snapshot {
    std::vector<int> left,right,center,black;
    std::vector<Roundabout::Point> corners;
    Roundabout::Point virtualDown;
    cv::Rect coneBox;
    cv::Mat blueMask;
    double targetX=-1;
    bool valid=false,blackUsed=false,coneHeld=false;
    int forkNearPercent=70;
    std::vector<std::string> lines;
};
// These matrices and arrays are taken from ONE control calculation; drawing never changes them.
cv::Mat render(const cv::Mat& bgr,const cv::Mat& raw,const cv::Mat& closed,const Snapshot& snapshot);
} }
